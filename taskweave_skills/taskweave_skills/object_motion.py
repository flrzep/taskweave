"""move_object and set_posture: the held-object skills of the new skill model (skill-api-spec 1.2, 1.3, 2, 3).

``move_object`` moves the object the robot holds with the hands that hold it (one or two), every hand keeping its grip
on it. The target is one of: an offset from where the object is (``by``, with turn and tilt), a position (``at``), a
named held pose of the robot for this kind of object (``pose``), or a contact of a placed object (``onto`` +
``contact``). ``motion.mode``:

    linear   (default) the hands on straight lines, the object rigid between them (the old move_object).
    free     joint space: IK for every hand's goal, then the path of a free move (straight when free, elbow swing first,
             else the planner). With two or more hands the sampled path is checked by FK: the distances between the
             hands must stay within ``grip_drift_m`` (skill yaml, default 0.02 m) of what they are now, else the move
             fails before anything moves and the message says to use linear.

``set_posture`` changes the arm shape while the hands stay: the elbows of the holding hands (else of all hands) swing
toward a side along the arms' null space. A held object does not move. It fails before moving when the swing would stop
further than ``tolerance_deg`` from the side.

Both reuse the old move_object's internals (_onto, _swing_plan, the linear and free move paths of the joint skills) and
the motion options of gripper_motion (skill yaml, robot override, step: ``taskweave_core.motion``). Until the switch
of the live missions the old ``joints:MoveObject`` stays the implementation of ``move_object`` in the cell's skill yaml;
this module's classes are what ``skills/move_object.yaml`` and ``skills/set_posture.yaml`` name afterwards.
"""

from __future__ import annotations

import copy
import itertools

import numpy as np

from taskweave_core import motion
from taskweave_core.config.schemas import ELBOW_SIDES, VEC3, hands_schema, motion_schema
from taskweave_core.skill import SkillResult

from .gripper_motion import RESOURCES, MoveGripper, Target, hands_arg
from .joints import SWING_STRAIGHT_STEPS, interpolate
from .joints import _ObjectMotion

OBJECT_MODE = 'linear'                    # the mode of move_object whose yaml has no ``motion.mode``
HOLD_RULES = ('auto', 'all')              # hands words of the held-object skills: the hold's hands, all hands


class _HeldMotion(MoveGripper, _ObjectMotion):
    """Shared setup of the held-object skills: the motion settings of move_gripper (speeds, accel, torso fallback) and
    the held-object helpers of the old move_object."""

    hand_rules = HOLD_RULES
    drift_hands: tuple = ()        # hands whose distances _move watches (free move_object with several hands)

    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self.args_schema = copy.deepcopy(self.args_schema)
        self.args_schema['properties']['hands'] = hands_schema(self.hands, self.hand_rules)

    def _unfit_for(self, mode: str, planner: str):
        """A REJECTED result if this robot cannot do what the options ask for."""
        if self.ik is None:
            return SkillResult.rejected(f'mode {mode} needs the profile ik section')
        if planner == 'planned' and self.planner is None:
            return SkillResult.rejected('planner planned needs the profile planning section')
        return None

    def _watch(self, ctx, hands: list, goals: dict, label: str, m: dict):
        """After the move: a hand that ended further than ``motion.tolerance`` (m) from its goal fails the step."""
        miss = self._misses(Target(label, hands, {hand: _target_of(goal) for hand, goal in goals.items()}),
                            m.get('tolerance'))
        return SkillResult.failed(f'{label}: {miss}') if miss else None


def _target_of(goal: np.ndarray) -> dict:
    from .points import target_of
    return target_of(goal)


class MoveObject(_HeldMotion):
    name = 'move_object'
    description = ('Move the object held with the grippers (grip close at its point first) with the hands that hold '
                   'it: every hand keeps its grip, locked in orientation and distance to the object. Target (to): by '
                   'an offset in the robot frame (x forward, y left, z up) with optional turn about the vertical and '
                   'tilt (far side up); at a position (where the object origin goes, orientation kept); pose, a '
                   'named held pose of this robot for this kind of object (e.g. carry: torso first, then straight '
                   'lines); or onto a contact of a placed object (the held object contact goes there, offset_m short '
                   'of it). motion.mode linear: straight lines (default); free: joint-space path with a planner, '
                   'with two hands checked so the hands do not drift apart (otherwise use linear).')
    args_schema = {
        'type': 'object',
        'required': ['to'],
        'additionalProperties': False,
        RESOURCES: ['hands', 'torso'],
        'properties': {
            'to': {'description': 'Where the held object goes.', 'oneOf': [
                {'title': 'by an offset', 'type': 'object', 'required': ['by'], 'additionalProperties': False,
                 'properties': {
                     'by': {**VEC3, 'description': '[x, y, z] in metres, robot frame (x forward, y left, z up).'},
                     'turn_deg': {'type': 'number',
                                  'description': 'Turn about the vertical through the object, degrees.'},
                     'tilt_deg': {'type': 'number',
                                  'description': 'Tilt about the robot y axis through the object, far side up, '
                                                 'degrees.'}}},
                {'title': 'to a position', 'type': 'object', 'required': ['at'], 'additionalProperties': False,
                 'properties': {
                     'at': {**VEC3, 'description': '[x, y, z] in metres, robot frame: where the object origin goes. '
                                                   'Orientation kept.'}}},
                {'title': 'held pose', 'type': 'object', 'required': ['pose'], 'additionalProperties': False,
                 'properties': {
                     'pose': {'type': 'string', 'x-taskweave-point': 'held_pose',
                              'description': 'A held pose of this robot for the object type '
                                             '(robots/<robot>/held_poses.yaml), e.g. carry.'}}},
                {'title': 'onto a contact', 'type': 'object', 'required': ['onto', 'contact'],
                 'additionalProperties': False,
                 'properties': {
                     'onto': {'type': 'string', 'x-taskweave-point': 'instance_contact',
                              'description': '<instance>/<contact>: where on a placed object the held object goes.'},
                     'contact': {'type': 'string', 'description': "The held object's contact that goes there."},
                     'offset_m': {'type': 'number', 'minimum': 0, 'x-taskweave-unit': 'm',
                                  'description': 'Stop this far before the contact, against its engage '
                                                 'direction.'}}}]},
            'object': {'type': 'string', 'x-taskweave-point': 'held_object',
                       'description': 'Only needed when several objects are held: which one (placed-object name).'},
            'hands': hands_schema((), HOLD_RULES),
            'motion': motion_schema(['mode', 'speed', 'speed_abs', 'accel', 'planner', 'tolerance']),
        },
    }
    needs = ['an object held with the grippers']
    achieves = ['the held object moved']
    default_timeout_s = 60.0

    # ---- running -------------------------------------------------------------
    def execute(self, ctx, args):
        to = args['to']
        hold = self._hold(args)
        if isinstance(hold, SkillResult):
            return hold
        m = motion.values(motion.resolve(self.config, self.robot.name, self.name, args))
        mode, planner = m.get('mode', OBJECT_MODE), m.get('planner', 'auto')
        unfit = self._unfit_for(mode, planner)
        if unfit is not None:
            return unfit
        goal = None
        if 'pose' not in to:
            goal = self._goal(hold, to)
            if isinstance(goal, SkillResult):
                return goal
        self._refresh_ik()
        saved = self.p, self.tcp_speed, self.tcp_turn, self.plan_scale
        self._speeds(m)
        try:
            if goal is None:
                result = self._carry_pose(ctx, hold, to['pose'], m, mode, planner)
            else:
                result = self._carry(ctx, hold, goal[0], goal[1], m, mode, planner)
        finally:
            self.p, self.tcp_speed, self.tcp_turn, self.plan_scale = saved
        if isinstance(result, SkillResult):
            return result
        label, worst, kind = result
        return SkillResult.succeeded(f'moved {label} (max error {worst:.3f})',
                                     world_state={'held_object': hold['object']}, held_object=hold['object'],
                                     hands=hold['hands'], mode=mode, path=kind, max_error=float(worst), motion=m)

    def _hold(self, args: dict):
        """The hold to move: the one ``args.object`` names, or the only one (or the one the named hands are in). A
        REJECTED result if there is none, several, or ``hands`` is not exactly the hands of the hold."""
        holds = self.hold.held_all()
        if not holds:
            return SkillResult.rejected('nothing is held: move_gripper to the object, then grip close')
        rule, name = hands_arg(args.get('hands')), args.get('object')
        named = None if rule in HOLD_RULES else list(rule)
        if name:
            hold = next((h for h in holds if h['object'] == name), None)
            if hold is None:
                return SkillResult.rejected(f'{name} is not held (held: {", ".join(h["object"] for h in holds)})')
        else:
            candidates = [h for h in holds if named is None or set(h['hands']) & set(named)]
            if not candidates:
                text = '; '.join(f'{h["object"]} by {" and ".join(h["hands"])}' for h in holds)
                return SkillResult.rejected(f'{" and ".join(named)} hold nothing (held: {text})')
            if len(candidates) > 1:
                return SkillResult.rejected('several objects held: name one with object:')
            hold = candidates[0]
        if named is not None and set(named) != set(hold['hands']):
            others = [hand for hand in hold['hands'] if hand not in named]
            if others:
                return SkillResult.rejected(f'{hold["object"]} is also held by {" and ".join(others)}: include them '
                                            'or open them')
            return SkillResult.rejected(f'{" and ".join(h for h in named if h not in hold["hands"])} does not hold '
                                        f'{hold["object"]}: hands must be the hands that hold it '
                                        f'({" and ".join(hold["hands"])})')
        return hold

    def _goal(self, hold: dict, to: dict):
        """(the held object's pose in the base frame after the move, a label), or a REJECTED / FAILED result."""
        if 'onto' in to:
            moved = self._onto(hold, {'to': to['onto'], 'contact': to['contact'], 'offset_m': to.get('offset_m', 0.0)})
            if isinstance(moved, SkillResult):
                return moved
            return moved, f'{hold["object"]} {to["contact"]} to {to["onto"]}' + \
                (f' (-{to["offset_m"]:g} m)' if to.get('offset_m') else '')
        from .points import moved_object

        moved = moved_object(hold['object_in_base'], to.get('by'), to.get('at'), float(to.get('turn_deg', 0.0)),
                             float(to.get('tilt_deg', 0.0)))
        return moved, f'{hold["object"]} ' + (f'to {[round(float(v), 3) for v in to["at"]]}' if 'at' in to else
                                              f'by {[round(float(v), 3) for v in to["by"]]}') + \
            (f', {to["turn_deg"]:g} deg' if to.get('turn_deg') else '') + \
            (f', tilted {to["tilt_deg"]:g} deg' if to.get('tilt_deg') else '')

    def _carry(self, ctx, hold: dict, moved: np.ndarray, label: str, m: dict, mode: str, planner: str):
        """Move the held object to ``moved`` (4x4, base frame) with every hand of the hold: (label, worst joint error,
        path kind) or a failure result. Updates the hold afterwards."""
        goals = {hand: moved @ pose for hand, pose in hold['tcp_in_object'].items()}
        if mode == 'linear':
            result, worst = self._linear(ctx, {}, label, goals=goals)
            kind = 'linear'
        else:
            tools = {hand: _target_of(goal) for hand, goal in goals.items()}
            self.drift_hands = tuple(goals) if len(goals) > 1 else ()
            try:      # the hands' IK goals keep the grasp as it is (no flipped fingers on a held object)
                result, worst = self._reach(ctx, {}, tools, label, flip=False, planner=planner)
            finally:
                self.drift_hands = ()
            kind = self.path_kind
        if result is not None:
            return result
        failure = self._watch(ctx, list(goals), goals, label, m)
        if failure is not None:
            return failure
        self.hold.moved(moved, hold['object'])
        return label, worst, kind

    def _carry_pose(self, ctx, hold: dict, name: str, m: dict, mode: str, planner: str):
        """Held pose ``name`` for the held object's type: its joints (the lift, not the arms') first, then the
        waypoints one after the other, each like any other move of the object."""
        from .ik import rpy_matrix

        instance = self.config.object_instances().get(hold['object'])
        if instance is None:
            return SkillResult.rejected(f'the held {hold["object"]} is not a placed object: its type is unknown')
        try:
            pose = self.config.held_pose(self.robot.name, instance.object, name)
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        arm = self._arm_joints(pose['joints'])
        if arm:
            return SkillResult.rejected(f'held pose {name} sets arm joints {arm}: only joints the held object moves '
                                        'with (e.g. a lift) can be set')
        label, worst, kind = f'{hold["object"]} to {name}', 0.0, 'direct'
        hand = hold['hands'][0]
        if pose['joints']:
            result, worst = self._go(ctx, pose['joints'], f'{label} (joints)', planner=planner)
            if result is not None:
                return result
            kind = self.path_kind
            try:    # the arms kept their joints: the object moved with them
                tcp = self.hold.tcp_poses([hand])[hand]
            except LookupError as error:
                return SkillResult.failed(f'{label}: where is the held object now? {error}')
            self.hold.moved(tcp @ np.linalg.inv(hold['tcp_in_object'][hand]), hold['object'])
        for index, waypoint in enumerate(pose['path'], start=1):
            hold = self.hold.hold_of(hand)
            step = self._carry(ctx, hold, rpy_matrix(waypoint.xyz, waypoint.rpy),
                               f'{label} ({index}/{len(pose["path"])})', m, mode, planner)
            if isinstance(step, SkillResult):
                return step
            _, worst, kind = step
        return label, worst, kind

    # ---- the grip stays: free paths of several hands -----------------------------------------------------------
    def _move(self, ctx, goal: dict, label: str, stall_ok: bool = False, path=None):
        if self.drift_hands:
            failure = self._drift(goal, path, label)
            if failure is not None:
                return failure, None
        return super()._move(ctx, goal, label, stall_ok, path)

    def _drift(self, goal: dict, path, label: str):
        """A FAILED result if on the way to ``goal`` (a planned ``path``, else the straight joint move) the distances
        between the hands change by more than p.grip_drift_m: the hands would pull at, or let go of, the object."""
        start = self._start()
        if start is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)')
        hands = list(self.drift_hands)
        if path is None:
            states = [{**start, **interpolate(start, goal, k / SWING_STRAIGHT_STEPS)}
                      for k in range(SWING_STRAIGHT_STEPS + 1)]
        else:      # joints the planner did not move go straight alongside (_move)
            rest = {n: v for n, v in goal.items() if n not in path.names}
            states = [{**start, **interpolate(start, rest, t / path.duration if path.duration else 1.0),
                       **dict(zip(path.names, row, strict=True))} for t, row in zip(path.times, path.points,
                                                                                      strict=True)]
        poses = [self._tcp_fk(state, hands) for state in states]
        if poses[0] is None:
            return SkillResult.rejected(f'{label}: the hands of a free move need the robot\'s URDF chains to be '
                                        'checked for drifting apart: use mode linear')
        worst = 0.0
        for first, second in itertools.combinations(hands, 2):
            gap = [float(np.linalg.norm(pose[first][:3, 3] - pose[second][:3, 3])) for pose in poses]
            worst = max(worst, *(abs(g - gap[0]) for g in gap))
        if worst > self.p.grip_drift_m:
            return SkillResult.failed(f'{label}: the hands would drift apart on a free path ({worst:.3f} m); '
                                      'use mode linear')
        return None

    def _tcp_fk(self, positions: dict, hands: list) -> dict | None:
        """{hand: TCP pose (4x4, base frame)} for the joints ``positions`` (every joint), from the URDF chains; None
        if a hand has no chain."""
        from .ik import rpy_matrix

        chains = getattr(self.ik, 'chains', None) or {}
        if any(hand not in chains for hand in hands):
            return None
        return {hand: chains[hand][0].fk(positions) @ rpy_matrix(self.ik.tools[hand].get('xyz', [0, 0, 0]),
                                                                 self.ik.tools[hand].get('rpy', [0, 0, 0]))
                for hand in hands}


class SetPosture(_HeldMotion):
    name = 'set_posture'
    description = ('Change the arm shape while the hands stay where they are: the elbows swing toward a side '
                   '(robot frame, out = away from the body) along the arms\' null space, as far as joint limits and '
                   'collisions allow. Default hands: the hands that hold something, else all. A held object does not '
                   'move. Fails before moving if the elbows would stop more than tolerance_deg off the side.')
    args_schema = {
        'type': 'object',
        'required': ['elbow'],
        'additionalProperties': False,
        RESOURCES: ['hands'],
        'properties': {
            'elbow': {'enum': ELBOW_SIDES,
                      'description': 'The hands and any held object stay; the arms swing their elbows toward this '
                                     'side (robot frame, out = away from the body), as far as joint limits and '
                                     'collisions allow.'},
            'tolerance_deg': {'type': 'number', 'exclusiveMinimum': 0, 'x-taskweave-unit': 'deg',
                              'description': 'Fail if the elbows would stop further than this from the side '
                                             '(default: skill config elbow_tolerance_deg).'},
            'hands': hands_schema((), HOLD_RULES),
            'motion': motion_schema(['speed', 'speed_abs', 'accel', 'tolerance']),
        },
    }
    needs = ['the hands stay in reach of where they are']
    achieves = ['the elbows toward the given side, the hands unchanged']
    default_timeout_s = 60.0

    def execute(self, ctx, args):
        side, rule = args['elbow'], hands_arg(args.get('hands'))
        holding = [hand for hand in self.hands if hand in self.hold.hand_holds]
        hands = (holding or list(self.hands)) if rule == 'auto' else list(self.hands) if rule == 'all' else list(rule)
        label = f'elbows of {" and ".join(hands)} {side}'
        m = motion.values(motion.resolve(self.config, self.robot.name, self.name, args))
        self._refresh_ik()
        saved = self.p, self.tcp_speed, self.tcp_turn, self.plan_scale
        self._speeds(m)
        try:
            try:
                starts = self.hold.tcp_poses(hands)
            except LookupError as error:
                return SkillResult.failed(f'{label}: where are the hands? {error}')
            failure, path, report, swung = self._swing_plan(ctx, hands, side, label, args.get('tolerance_deg'))
            if failure is not None:
                return failure
            data = {'hands': hands, 'mode': 'null_space', 'motion': m,
                    'report': {hand: {'from_deg': a, 'to_deg': b} for hand, (a, b, _) in report.items()}}
            if path is None:
                return SkillResult.succeeded(f'{label}: already as far as they go ({swung})', path='none', **data)
            goal = dict(zip(path.names, path.points[-1], strict=True))
            result, _ = self._move(ctx, goal, label, path=path)
            if result is not None:
                return result
            self._check_hands(starts, label)
            failure = self._watch(ctx, hands, starts, label, m)
            if failure is not None:
                return failure
        finally:
            self.p, self.tcp_speed, self.tcp_turn, self.plan_scale = saved
        return SkillResult.succeeded(f'{label}: {swung}', path='swing', **data)

