"""move_gripper and grip: the gripper skills of the new skill model (skill-api-spec 1.1, 1.5, 2, 3).

``move_gripper`` moves the TCPs of one or more hands to a target with nothing held: a point on a placed object (with a
stop-short offset), a named tool pose of the robot, an offset from where the hands are now (``by``), or an absolute
pose (``at``). It is the old go_to_point and move_to_pose (tool targets) on one set of motion options
(``motion``, resolved by taskweave_core.motion: skill yaml, robot override, the point's own data, the step):

    mode free    joint space: the old go_to_point approach (planned to offset + approach_m, then straight in) when a
                 point has an approach, else the old move_to_pose path (IK, then ``_go``: straight when free, elbow
                 swing first, else the planner). ``planner`` direct / planned pick one of them.
    mode linear  the TCPs on straight lines from where they are (``_linear``).

``grip`` opens, closes or sets the opening (mm) of the fingers of some hands (the old gripper, with the opening table
and stall logic), and keeps the per-hand hold registry (hold.py): closing hands that are at an object point makes them
hold it, opening takes them out of the hold.

The skill yaml layout is spec 2.2 (``speed:``, ``accel:``, ``motion:``, ``torso_fallback:``, ``tolerances:``); the
old flat keys keep working.
"""

from __future__ import annotations

import copy
import math
from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

from taskweave_core import motion
from taskweave_core.config import Pose2D
from taskweave_core.config.schemas import HAND_RULES, VEC3, hands_schema, motion_schema
from taskweave_core.gripper import joint_for_opening, opening_for_joint, opening_table
from taskweave_core.skill import SkillResult

from .joints import _FingerMotion, _JointSkill, _PointMotion

# skills/<skill>.yaml (spec 2.2) -> the joint skill parameter names
SPEED_PARAMS = {'joint_rad_s': 'max_joint_speed', 'joint_linear_m_s': 'max_linear_speed', 'tcp_m_s': 'tcp_speed',
                'tcp_rad_s': 'tcp_turn'}
TOLERANCE_PARAMS = {'position': 'position_tolerance', 'settle_s': 'settle_s', 'stall_timeout_s': 'stall_timeout_s',
                    'state_timeout_s': 'state_timeout_s'}
DEFAULT_MODE = 'free'     # the mode of a skill whose yaml has no ``motion.mode``
VIA_POSE = 'pose:'        # motion.via entry: a named pose of this robot; else <instance>/<point>
RESOURCES = 'x-taskweave-resources'   # what a step uses (the live plan graph)


def translate_params(params: dict) -> tuple[dict, dict]:
    """Skill yaml parameters in the spec 2.2 layout -> (the flat joint skill parameters, the rest: accel and
    torso_fallback). ``motion`` is left out: motion.resolve reads it at run time."""
    flat = dict(params)
    speed, tolerances = flat.pop('speed', None) or {}, flat.pop('tolerances', None) or {}
    extra = {'accel': dict(flat.pop('accel', None) or {}), 'torso_fallback': flat.pop('torso_fallback', None) or {}}
    flat.pop('motion', None)
    flat.update({name: speed[key] for key, name in SPEED_PARAMS.items() if key in speed})
    flat.update({name: tolerances[key] for key, name in TOLERANCE_PARAMS.items() if key in tolerances})
    if extra['torso_fallback']:
        flat['lift_joint'], flat['lift_values'] = extra['torso_fallback']['joint'], extra['torso_fallback']['values']
    return flat, extra


def hands_arg(value, default='auto'):
    """A ``hands`` argument as a rule word (auto, any_one, all) or a list of hand names."""
    if value is None:
        return default
    if isinstance(value, str):
        return value if value in HAND_RULES else [value]
    return list(value)


@dataclass
class Target:
    """Where move_gripper goes: ``tools`` {hand: {tcp, approach, closing}} (base frame), plus what goes with them."""
    label: str
    hands: list
    tools: dict
    joints: dict = field(default_factory=dict)         # joints set with the tools (a named pose's, e.g. a lift)
    data: dict = field(default_factory=dict)           # the target's own motion data (motion.resolve layer 3)
    before: Callable[[float], dict] | None = None      # the tools this far further back along their approach
    reached: tuple | None = None                       # (point, instance name, object_in_base): hold.reach on arrival


class MoveGripper(_PointMotion):
    name = 'move_gripper'
    description = ('Move the gripper (TCP) of one or more hands to a target with nothing held: a point on a placed '
                   'object (to.point, offset_m stops short along its approach), a named tool pose of this robot '
                   '(to.pose), an offset from where the hands are now (to.by, with turn and tilt) or an absolute '
                   'pose (to.at, one hand). mode free: collision-free path in joint space (with a planner), the last '
                   'approach_m of a point in a straight line; mode linear: the TCPs on straight lines. motion sets '
                   'speed, elbow side, planner and the like; its defaults come from the skill, the robot and the '
                   'point. Arriving at a point with no offset makes closing the grippers a grasp of that object.')
    args_schema = {
        'type': 'object',
        'required': ['to'],
        'additionalProperties': False,
        RESOURCES: ['hands', 'torso'],
        'properties': {
            'to': {'description': 'Where the gripper goes.', 'oneOf': [
                {'title': 'object point', 'type': 'object', 'required': ['point'], 'additionalProperties': False,
                 'properties': {
                     'point': {'type': 'string', 'x-taskweave-point': 'instance_point',
                               'description': '<instance>/<point>: a point taught on a placed object (e.g. the grasp '
                                              'point of a frame in a bin).'},
                     'offset_m': {'type': 'number', 'minimum': 0, 'x-taskweave-unit': 'm',
                                  'description': "Stop this far back along the point's approach direction "
                                                 '(pre-grasp).'}}},
                {'title': 'named tool pose', 'type': 'object', 'required': ['pose'], 'additionalProperties': False,
                 'properties': {
                     'pose': {'type': 'string', 'x-taskweave-point': 'tool_pose',
                              'description': 'A named pose of this robot that places tools (robots/<robot>/poses.yaml: '
                                             'tools).'}}},
                {'title': 'relative to now', 'type': 'object', 'required': ['by'], 'additionalProperties': False,
                 'properties': {
                     'by': {**VEC3, 'description': '[x, y, z] in metres, in the chosen frame.'},
                     'frame': {'enum': ['robot', 'tool'], 'default': 'robot',
                               'description': 'robot: x forward, y left, z up. tool: along the TCP axes (z = '
                                              'approach).'},
                     'turn_deg': {'type': 'number', 'description': 'Turn about the vertical (frame robot) or the '
                                                                   'approach axis (frame tool), degrees.'},
                     'tilt_deg': {'type': 'number', 'description': 'Tilt about the y axis, degrees.'}}},
                {'title': 'absolute pose', 'type': 'object', 'required': ['at'], 'additionalProperties': False,
                 'properties': {
                     'at': {**VEC3, 'description': '[x, y, z] in metres, robot frame: where the TCP goes. Exactly '
                                                   'one hand.'},
                     'rpy_deg': {**VEC3, 'description': 'Orientation (roll, pitch, yaw) in degrees, robot frame. '
                                                        'Default: keep.'}}}]},
            'hands': hands_schema(),
            'motion': motion_schema(),
        },
    }
    needs = ["the robot stands where the target is in reach (e.g. at the object's stance)", 'no object held']
    achieves = ['gripper at the target']
    default_timeout_s = 60.0

    def setup(self, node, config, robot, params):
        params, extra = translate_params(params)
        self.accel, self.torso_fallback = extra['accel'], extra['torso_fallback']
        self.path_kind, self.plan_scale = None, 1.0
        super().setup(node, config, robot, params)
        self.hands = list(robot.data.get('grippers', {}))
        self.args_schema = copy.deepcopy(self.args_schema)
        self.args_schema['properties']['hands'] = hands_schema(self.hands)

    # ---- running -------------------------------------------------------------
    def execute(self, ctx, args):
        refusal = self._holding_refusal()
        if refusal is not None:
            return refusal
        target = self._target(args)
        if isinstance(target, SkillResult):
            return target
        m = motion.values(motion.resolve(self.config, self.robot.name, self.name, args, target.data))
        mode, planner = m.get('mode', DEFAULT_MODE), m.get('planner', 'auto')
        unfit = self._unfit(target, mode, planner)
        if unfit is not None:
            return unfit
        legs = self._via(m.get('via') or [], target)
        if isinstance(legs, SkillResult):
            return legs
        self.hold.leave(target.hands)      # these hands go elsewhere: no longer at a point
        self._refresh_ik()
        saved = self.p, self.tcp_speed, self.tcp_turn, self.plan_scale
        self._speeds(m)
        try:
            for leg in [*legs, target]:
                result, worst, label, kind = self._leg(ctx, leg, m, mode, planner, leg is target)
                if result is not None:
                    return result
        finally:
            self.p, self.tcp_speed, self.tcp_turn, self.plan_scale = saved
        miss = self._misses(target, m.get('tolerance'))
        if miss:
            return SkillResult.failed(f'{label}: {miss}')
        if target.reached:       # at the point itself: closing the grippers now grasps this object
            self.hold.reach(target.hands, *target.reached)
        return SkillResult.succeeded(f'at {label} (max error {worst:.3f})', world_state={'arm_pose': label},
                                     hands=target.hands, mode=mode, path=kind, max_error=float(worst), motion=m)

    def _leg(self, ctx, leg: Target, m: dict, mode: str, planner: str, final: bool):
        """One leg to ``leg``'s tools: (failure or None, worst joint error, label, path kind)."""
        label, elbow = leg.label, m.get('elbow')
        if mode == 'linear' and leg.tools:   # a via pose of joints only has no line to follow
            label += ' (linear)'
            result, worst = self._linear(ctx, leg.tools, label)
            return result, worst, label, 'linear'
        torso = m.get('torso', 'keep') == 'auto' and self.lift_joint
        lifts = [{**leg.joints, self.lift_joint: float(v)} for v in self.lift_values] if torso else []
        approach = float(m.get('approach_m', 0.0)) if final and leg.before else 0.0
        if approach > 0:                      # planned to offset + approach, then straight in
            result, worst, label = self._approach_in(ctx, leg.tools, leg.before(approach), lifts, label, elbow,
                                                     approach, planner)
            return result, worst, label, 'approach'
        result, worst = self._reach(ctx, leg.joints, leg.tools, label, fallback_joints=lifts, elbow=elbow,
                                    planner=planner)
        return result, worst, label, self.path_kind

    def _unfit(self, target: Target, mode: str, planner: str):
        """A REJECTED result if this robot cannot do what the options ask for."""
        if target.tools and self.ik is None:
            return SkillResult.rejected(f'{target.label} has tool targets, but the robot profile has no ik')
        if mode == 'linear' and self.ik is None:
            return SkillResult.rejected('mode linear needs the profile ik section')
        if planner == 'planned' and self.planner is None:
            return SkillResult.rejected('planner planned needs the profile planning section')
        return None

    def _speeds(self, m: dict) -> None:
        """This run's speeds: the standard (skill yaml, robot override) x motion.speed, or motion.speed_abs. The
        global override is applied by the move itself (phase clock). Planned paths are timed by the planner: they are
        stretched by the joint speed fraction (_slowed)."""
        standard = m.get('standard_speed') or {}
        fraction, absolute = m.get('speed'), m.get('speed_abs') or {}

        def effective(key: str, now: float) -> float:
            return motion.effective_speed(standard.get(key, now), None, fraction, absolute.get(key))

        joint = effective('joint_rad_s', self.p.max_joint_speed)
        accel = m.get('accel') or 1.0
        self.p = replace(self.p, max_joint_speed=joint, min_duration_s=self.p.min_duration_s / accel,
                         max_linear_speed=effective('joint_linear_m_s', self.p.max_linear_speed))
        self.plan_scale = min(1.0, joint / (standard.get('joint_rad_s') or joint))
        self.tcp_speed = effective('tcp_m_s', self.tcp_speed)
        self.tcp_turn = effective('tcp_rad_s', self.tcp_turn)

    def _slowed(self, path):
        if self.plan_scale >= 1.0:
            return path
        from .motion_plan import Trajectory
        return Trajectory(path.names, [t / self.plan_scale for t in path.times], path.points)

    def _misses(self, target: Target, tolerance) -> str | None:
        """With ``motion.tolerance``: where a hand ended further than that (m) from its target."""
        if not tolerance:
            return None
        from .cartesian import tcp_targets
        try:
            reached = self.hold.tcp_poses(target.tools)
        except LookupError:
            return None
        goals = tcp_targets(target.tools)
        off = {hand: float(np.linalg.norm(reached[hand][:3, 3] - goal[:3, 3])) for hand, goal in goals.items()}
        late = [f'{hand} is {miss:.3f} m off' for hand, miss in off.items() if miss > tolerance]
        return f'{", ".join(late)} (tolerance {tolerance:g} m)' if late else None

    # ---- targets ---------------------------------------------------------------
    def _target(self, args: dict):
        to, rule = args['to'], hands_arg(args.get('hands'))
        if rule == 'any_one':
            return SkillResult.rejected('hands any_one is not available yet: name the hand or hands')
        if 'point' in to:
            target = self._point_target(to['point'], float(to.get('offset_m', 0.0)), rule)
            if isinstance(target, Target):
                target.data = motion.target_data_for(self.config, self.robot.name, args)
            return target
        if 'pose' in to:
            return self._pose_target(to['pose'], rule)
        return self._relative_target(to, rule)

    def _select(self, rule, defined, what: str):
        """The hands to move: ``rule`` (auto: all the target defines; all; names) among those ``defined``."""
        hands = list(defined) if rule == 'auto' else list(self.hands) if rule == 'all' else list(rule)
        missing = [hand for hand in hands if hand not in defined]
        if not hands:
            return SkillResult.rejected(f'{what} has no tool poses')
        if missing:
            return SkillResult.rejected(f'{what} has no tool poses for {", ".join(missing)}')
        return hands

    def _point_target(self, reference: str, offset: float, rule):
        from .points import instance_matrix, point_poses, point_targets

        try:
            base_in_cell = self.hold.transform(self.cell_frame, self.robot.base['frame'])
        except LookupError as error:
            return SkillResult.failed(f'where is the robot? {error}')
        # a symmetric object (e.g. a frame lying the other way round) is grasped from the robot's side
        base = Pose2D(float(base_in_cell[0, 3]), float(base_in_cell[1, 3]),
                      math.atan2(base_in_cell[1, 0], base_in_cell[0, 0]))
        try:
            instance, point = self.config.instance_point(reference, self.robot.name, base)
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        names = None if rule == 'auto' else list(self.hands) if rule == 'all' else list(rule)
        hands = self._select(rule, point_poses(point, names), reference)
        if isinstance(hands, SkillResult):
            return hands
        names = None if rule == 'auto' else hands          # auto takes the pair, whatever the hands are
        label = reference + (f' (-{offset:g} m)' if offset else '')
        reached = None
        if offset == 0.0:
            reached = (reference, instance.name, np.linalg.inv(base_in_cell) @ instance_matrix(instance))
        return Target(label, hands, point_targets(point, instance, base_in_cell, offset, names),
                      before=lambda extra: point_targets(point, instance, base_in_cell, offset + extra, names),
                      reached=reached)

    def _pose_target(self, name: str, rule):
        try:
            pose = self.config.named_pose(self.robot.name, name)
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        if not pose['tools']:
            return SkillResult.rejected(f'pose {name} has no tool targets (joint values only): use move_joints')
        hands = self._select(rule, pose['tools'], f'pose {name}')
        if isinstance(hands, SkillResult):
            return hands
        return Target(name, hands, {hand: pose['tools'][hand] for hand in hands}, joints=pose['joints'],
                      data={'elbow': pose['elbow']} if pose.get('elbow') else {})

    def _relative_target(self, to: dict, rule):
        from .ik import rpy_matrix
        from .points import moved_tcp, target_of

        hands = list(self.hands) if rule in ('auto', 'all') else list(rule)
        if 'at' in to and len(hands) != 1:
            return SkillResult.rejected('at needs exactly one hand')
        try:
            now = self.hold.tcp_poses(hands)
        except KeyError as error:
            return SkillResult.rejected(f'the robot profile has no tool frame for {error.args[0]}')
        except LookupError as error:
            return SkillResult.failed(f'where is the gripper? {error}')
        if 'at' in to:
            goal = now[hands[0]].copy()
            if to.get('rpy_deg') is not None:
                goal = rpy_matrix(to['at'], np.radians(to['rpy_deg']))
            goal[:3, 3] = to['at']
            goals, label = {hands[0]: goal}, f'{hands[0]} to {[round(float(v), 3) for v in to["at"]]}'
        else:
            goals = {hand: moved_tcp(pose, to['by'], to.get('frame', 'robot'), float(to.get('turn_deg', 0.0)),
                                     float(to.get('tilt_deg', 0.0))) for hand, pose in now.items()}
            label = f'{" and ".join(hands)} by {[round(float(v), 3) for v in to["by"]]}' + \
                (f', turn {to["turn_deg"]:g} deg' if to.get('turn_deg') else '') + \
                (f', tilt {to["tilt_deg"]:g} deg' if to.get('tilt_deg') else '')
        return Target(label, hands, {hand: target_of(goal) for hand, goal in goals.items()})

    def _via(self, entries: list, target: Target):
        """The legs before the target for ``motion.via`` ('pose:<name>' or '<instance>/<point>'), or a REJECTED."""
        legs = []
        for entry in entries:
            if entry.startswith(VIA_POSE):
                try:
                    pose = self.config.named_pose(self.robot.name, entry[len(VIA_POSE):])
                except KeyError as error:
                    return SkillResult.rejected(str(error.args[0]))
                legs.append(Target(f'via {entry}', target.hands, pose['tools'], joints=pose['joints']))
            else:
                leg = self._point_target(entry, 0.0, target.hands)
                if isinstance(leg, SkillResult):
                    return leg
                legs.append(replace(leg, label=f'via {entry}', reached=None, before=None))
        return legs


class Grip(_FingerMotion):
    name = 'grip'
    description = ('Open or close the fingers of one or more hands, or set their opening in mm. Closing hands that '
                   'are at an object point (move_gripper to.point with no offset) makes them hold that object, also '
                   'one hand of a two-hand point; opening takes hands out of their hold, and a hold left with no '
                   'hands is released. opening sets the finger gap (opening_mm) and leaves the holds alone. open '
                   'with place_as records where the released object now is, as a placed object of that name '
                   '(object state). Closing on an object takes it off its stack in the object state.')
    args_schema = {
        'type': 'object',
        'required': ['action'],
        'additionalProperties': False,
        RESOURCES: ['hands'],
        'properties': {
            'action': {'enum': ['open', 'close', 'opening'], 'description': 'open / close, or opening: go to '
                                                                            'opening_mm.'},
            'hands': hands_schema((), ('auto', 'all')),
            'opening_mm': {'type': 'number', 'minimum': 0, 'x-taskweave-unit': 'mm',
                           'x-taskweave-show-if': {'action': ['opening', 'open', 'close']},
                           'description': 'opening: required, the finger gap. close: stay this wide (default: the '
                                          "point's grip, else fully closed). open: open only this wide."},
            'place_as': {'type': 'string', 'x-taskweave-show-if': {'action': ['open']},
                         'description': 'With open: the released object is a placed object of this name now '
                                        '(object state).'},
            'motion': motion_schema(['speed', 'accel', 'tolerance']),
        },
        'if': {'properties': {'action': {'const': 'opening'}}},
        'then': {'required': ['opening_mm']},
    }
    achieves = ['gripper open, or closed (holding an object if one was between the fingers)']
    default_timeout_s = 20.0

    def setup(self, node, config, robot, params):
        params, _ = translate_params(params)
        for key in ('lift_joint', 'lift_values', 'tcp_speed', 'tcp_turn'):   # arm motion settings of a shared yaml
            params.pop(key, None)
        self.cell_frame = params.pop('cell_frame', 'world')  # TF frame of the cell (place_as)
        _JointSkill.setup(self, node, config, robot, params)
        self.hold.on_reset.append(config.clear_object_state)  # a sim reset puts every object back
        self.hands = list(robot.data.get('grippers', {}))
        self.args_schema = copy.deepcopy(self.args_schema)
        self.args_schema['properties']['hands'] = hands_schema(self.hands, ('auto', 'all'))

    def execute(self, ctx, args):
        action = args['action']
        if action == 'opening' and args.get('opening_mm') is None:
            return SkillResult.rejected('opening_mm needed with action opening')
        hands = self._hands(hands_arg(args.get('hands')), action)
        if action == 'opening':
            holding = self.hold.hand_holds
            for hand in hands:
                if hand in holding:
                    return SkillResult.rejected(f'{hand} holds {holding[hand]}: open it with action open, or move the '
                                                f'object first')
        grippers = {hand: self.robot.role(f'grippers.{hand}') for hand in hands}
        opening = None if args.get('opening_mm') is None else float(args['opening_mm']) / 1000.0
        if opening is not None:
            for hand in hands:
                if opening_table(grippers[hand]) is None:
                    return SkillResult.rejected(f'{hand} has no opening table (profile grippers.{hand}.opening_m)')
        key = 'closed' if action == 'close' else 'open'
        goal = {g['joint']: float(g[key]) for g in grippers.values()}
        grip_m = {}                       # close: how far each hand stays open
        if action == 'close':
            grip_m = {hand: opening for hand in hands} if opening is not None else self._point_grip(hands)
        elif opening is not None:         # opening, or open only this wide
            goal = {g['joint']: joint_for_opening(g, opening) for g in grippers.values()}
        goal.update({grippers[hand]['joint']: joint_for_opening(grippers[hand], value)
                     for hand, value in grip_m.items()})
        saved = self.p
        self._speeds(motion.values(motion.resolve(self.config, self.robot.name, self.name, args)))
        try:
            result, final = self._move(ctx, goal, f'{" and ".join(hands)} gripper {action}', stall_ok=action != 'open')
        finally:
            self.p = saved
        if result is not None:
            return result
        return self._finish(args, hands, grippers, goal, final, grip_m)

    def _speeds(self, m: dict) -> None:
        """The finger speed of this run: standard x motion.speed (or speed_abs), accel for the shortest move."""
        standard = (m.get('standard_speed') or {}).get('joint_rad_s', self.p.max_joint_speed)
        speed = motion.effective_speed(standard, None, m.get('speed'), (m.get('speed_abs') or {}).get('joint_rad_s'))
        self.p = replace(self.p, max_joint_speed=speed, min_duration_s=self.p.min_duration_s / (m.get('accel') or 1.0),
                         position_tolerance=m.get('tolerance') or self.p.position_tolerance)

    def _hands(self, rule, action: str) -> list:
        """The hands to act on: names, all, or auto (close: the hands at an object point; open: the holding hands;
        else all)."""
        if rule == 'all' or (rule == 'auto' and action == 'opening'):
            return list(self.hands)
        if rule != 'auto':
            return list(rule)
        picked = self.hold.reached_hands(self.hands) if action == 'close' else self.hold.hand_holds
        return [hand for hand in self.hands if hand in picked] or list(self.hands)

    def _finish(self, args: dict, hands: list, grippers: dict, goal: dict, final: dict, grip_m: dict):
        action = args['action']
        state, holding, measured = {}, {}, {}
        for hand, g in grippers.items():
            stopped_short = abs(final[g['joint']] - goal[g['joint']]) > self.p.position_tolerance
            # closed to the point's grip: on the part (the sim's parts have no colliders to stop the fingers)
            holding[hand] = action == 'close' and (stopped_short or hand in grip_m)
            if opening_table(g) is not None:
                measured[hand] = round(opening_for_joint(g, final[g['joint']]) * 1000.0, 1)
            if action != 'opening':
                state[f'{hand}_gripper'] = 'holding' if holding[hand] else ('closed' if action == 'close' else 'open')
        message = ', '.join(f'{h}: {s}' for h, s in state.items()) or \
            ', '.join(f'{h}: {mm:g} mm' for h, mm in measured.items())
        if grip_m:
            message += ' (grip ' + ', '.join(f'{h} {value * 1000:.0f} mm' for h, value in grip_m.items()) + ')'
        if action == 'close':
            message += self._close(hands, state)
        elif action == 'open':
            message += self._open(args, hands, state)
        else:
            holding = {hand: hand in self.hold.hand_holds for hand in hands}
        return SkillResult.succeeded(message, holding=holding, opening_mm=measured, hands=hands, world_state=state)

    def _close(self, hands: list, state: dict) -> str:
        """The registry after a close: hands at an object point hold that object."""
        try:
            grasped = self.hold.close_hands(hands)
        except LookupError as error:
            return f'; hold not registered: where is the gripper? {error}'
        message = ''
        for name, info in grasped.items():
            message += f'; {" and ".join(info["hands"])} hold {name}'
            state['held_object'] = state.get('held_object') or name
            if info['new']:
                try:
                    message += f'; {self.config.take_object(name)}'  # one off its stack (object state)
                except (KeyError, ValueError) as error:
                    message += f'; object state not updated: {error}'
        return message

    def _open(self, args: dict, hands: list, state: dict) -> str:
        """The registry after an open: hands let go; ``place_as`` records the object when the last hands let go."""
        message = ''
        if args.get('place_as'):
            holds = [h for h in self.hold.held_all() if set(h['hands']) & set(hands)]
            whole = [h for h in holds if set(h['hands']) <= set(hands)]
            if whole:
                message += f'; {self._place(args["place_as"], whole[0])}'
            else:
                message += f'; {args["place_as"]} not placed: the object is still held' if holds else \
                    f'; nothing held, {args["place_as"]} not placed'
        let_go = self.hold.open_hands(hands)
        for name, info in let_go.items():
            message += f'; {" and ".join(info["hands"])} let go of {name}' + (' (released)' if info['released'] else '')
        if let_go:
            remaining = self.hold.held_all()
            state['held_object'] = remaining[0]['object'] if remaining else None
        return message

    def _point_grip(self, hands) -> dict[str, float]:
        """How far (m) each of ``hands`` stays open when closing at the object point it is at (the robot's grip_m for
        the point, robots/<robot>/object_points.yaml); hands without one, or whose gripper has no opening_m table,
        close fully."""
        grip = {}
        for hand, at in self.hold.reached_hands(hands).items():
            try:
                instance = self.config.object_instances()[at['instance']]
                point = self.config.object_point(instance.object, at['point'].rpartition('/')[2], self.robot.name)
            except (KeyError, ValueError):
                continue
            value = (point.get('grip_m') or {}).get(hand)
            if value is not None and opening_table(self.robot.role(f'grippers.{hand}')) is not None:
                grip[hand] = float(value)
        return grip
