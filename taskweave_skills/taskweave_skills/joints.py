"""Joint-space skills for robots that take joint position targets on a topic (sensor_msgs/JointState).

Robot roles used (robots/<robot>/profile.yaml):
    joints:   {command: /joint_command, state: /joint_states, linear: [<prismatic joint>, ...]}
    grippers: {<side>: {joint: <name>, open: <value>, closed: <value>}}
Named poses: robots/<robot>/poses.yaml, ``<pose>: {<joint>: <value>, ...}`` (rad or m).

Moves are streamed as position targets: a straight joint-space move, or, if the profile has a ``planning`` section,
a collision-free trajectory from MoveIt (motion_plan.py). A ros2_control implementation can later serve the same
skill names (see skills/<skill>.yaml implementations) without changing missions.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, fields

import numpy as np
from sensor_msgs.msg import JointState

from taskweave_core.config import Pose2D
from taskweave_core.skill import Skill, SkillResult


@dataclass
class Params:
    max_joint_speed: float = 0.6       # rad/s (revolute) used to time a move
    max_linear_speed: float = 0.1      # m/s (prismatic, e.g. a lift)
    linear_joints: tuple = ()          # prismatic joints (metres), from the profile's joints.linear
    position_tolerance: float = 0.02   # rad or m
    settle_s: float = 0.3              # within tolerance this long before success
    stall_timeout_s: float = 3.0       # after the planned move: no progress this long -> fail / holding
    min_duration_s: float = 0.5
    rate_hz: float = 50.0
    state_timeout_s: float = 1.0       # no joint states this long -> fail

    @classmethod
    def from_config(cls, params: dict) -> Params:
        known = {f.name for f in fields(cls)} - {'linear_joints'}  # robot-specific: profile joints.linear
        unknown = set(params) - known
        if unknown:
            raise ValueError(f'unknown joint skill parameters: {", ".join(sorted(unknown))}')
        return cls(**{k: float(v) for k, v in params.items()})


def smoothstep(fraction: float) -> float:
    fraction = max(0.0, min(1.0, fraction))
    return fraction * fraction * (3 - 2 * fraction)


def plan_duration(start: dict, goal: dict, p: Params) -> float:
    """Time for the slowest joint at its configured speed."""
    duration = p.min_duration_s
    for name, target in goal.items():
        speed = p.max_linear_speed if name in p.linear_joints else p.max_joint_speed
        duration = max(duration, abs(target - start[name]) / speed)
    return duration


def interpolate(start: dict, goal: dict, fraction: float) -> dict:
    s = smoothstep(fraction)
    return {name: start[name] + (target - start[name]) * s for name, target in goal.items()}


class JointInterface:
    """Latest joint state plus a command publisher, shared by the joint skills of one server."""

    _instances: dict = {}

    @classmethod
    def for_node(cls, node, profile) -> JointInterface:
        key = id(node)
        if key not in cls._instances:
            cls._instances[key] = cls(node, profile.role('joints'))
        return cls._instances[key]

    def __init__(self, node, topics: dict):
        self.lock = threading.Lock()
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.stamp = -math.inf
        self.publisher = node.create_publisher(JointState, topics['command'], 10)
        node.create_subscription(JointState, topics['state'], self._state, 10)
        self.state_topic = topics['state']

    def _state(self, msg: JointState) -> None:
        with self.lock:
            self.positions.update(zip(msg.name, msg.position, strict=False))
            if msg.velocity:
                self.velocities.update(zip(msg.name, msg.velocity, strict=False))
            self.stamp = time.monotonic()

    def snapshot(self, max_age: float) -> dict | None:
        """Every joint seen on the state topic (a planner's start state)."""
        with self.lock:
            return dict(self.positions) if time.monotonic() - self.stamp <= max_age else None

    def current(self, names, max_age: float) -> dict | None:
        with self.lock:
            if time.monotonic() - self.stamp > max_age or any(n not in self.positions for n in names):
                return None
            return {n: self.positions[n] for n in names}

    def command(self, targets: dict) -> None:
        self.publisher.publish(JointState(name=list(targets), position=[float(v) for v in targets.values()]))


class _JointSkill(Skill):
    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self.p = Params.from_config(params)
        self.p.linear_joints = tuple(robot.role('joints').get('linear', ()))
        self.joints = JointInterface.for_node(node, robot)
        self._held: dict = {}
        from .hold import HoldState
        self.hold = HoldState.for_node(node, robot)

    def _holding_refusal(self):
        """A free-arm motion would pull the holding hands apart: refuse while something is held."""
        hold = self.hold.held()
        if hold is None:
            return None
        return SkillResult.rejected(f'holding {hold["object"]} with {" and ".join(hold["tools"])}: move it with '
                                    'move_object, or open the gripper first')

    def stop(self) -> None:
        """Hold where the joints are now."""
        if self._held:
            current = self.joints.current(list(self._held), self.p.state_timeout_s)
            if current:
                self.joints.command(current)

    def _move(self, ctx, goal: dict, label: str, stall_ok: bool = False, path=None):
        """Move the joints in ``goal``; returns (result or None, final positions). ``stall_ok``: stopping early
        (e.g. fingers blocked by an object) ends the move without failing. ``path``: a planned trajectory
        (motion_plan.Trajectory) the joints follow instead of a straight joint-space move."""
        self._held = goal
        start = self.joints.current(list(goal), self.p.state_timeout_s)
        if start is None:
            missing = [n for n in goal if n not in self.joints.positions]
            reason = f'unknown joints {missing}' if missing else f'no joint states on {self.joints.state_topic}'
            return SkillResult.failed(reason), None
        if path is None:
            duration = plan_duration(start, goal, self.p)
            reference = lambda t: interpolate(start, goal, t / duration)  # noqa: E731
        else:  # goal joints the planner did not move go straight alongside
            duration = max(path.duration, self.p.min_duration_s)
            rest = {n: v for n, v in goal.items() if n not in path.names}
            reference = lambda t: {**interpolate(start, rest, t / duration), **path.at(t)}  # noqa: E731
        t0 = ctx.now()
        period = 1.0 / self.p.rate_hz
        settled_since, best, best_time = None, math.inf, None
        while True:
            if ctx.should_stop:
                self.stop()
                return ctx.interrupted(), None
            now = ctx.now()
            fraction = (now - t0) / duration
            self.joints.command(reference(now - t0))
            current = self.joints.current(list(goal), self.p.state_timeout_s)
            if current is None:
                self.stop()
                return SkillResult.failed(f'joint states lost on {self.joints.state_topic}'), None
            errors = {n: abs(goal[n] - current[n]) for n in goal}
            worst = max(errors, key=errors.get)
            ctx.feedback(min(1.0, fraction), f'{label}: worst {worst} {errors[worst]:.3f}')
            if fraction >= 1.0:
                if errors[worst] <= self.p.position_tolerance:
                    settled_since = settled_since if settled_since is not None else now
                    if now - settled_since >= self.p.settle_s:
                        return None, current
                else:
                    settled_since = None
                    total = sum(errors.values())
                    if best_time is None or total < best - self.p.position_tolerance / 4:
                        best, best_time = total, now
                    elif now - best_time > self.p.stall_timeout_s:
                        if stall_ok:
                            return None, current
                        self.stop()
                        return SkillResult.failed(
                            f'{label} not reached: {worst} is {errors[worst]:.3f} off after stalling'), None
            time.sleep(period)


class MoveToPose(_JointSkill):
    name = 'move_to_pose'
    description = ('Move to a named pose of this robot (for example home or carry) and hold it. A pose is joint '
                   'values, or tool (TCP) targets in robot space solved by inverse kinematics. With a motion planner '
                   'the path avoids collisions with the robot itself and the planning scene; without one it is a '
                   'straight joint-space move. While an object is held, only poses without arm joints or tool '
                   'targets (for example a lift height) are allowed: the arms keep their joints and the object moves '
                   'with them.')
    args_schema = {
        'type': 'object',
        'required': ['pose'],
        'properties': {'pose': {'type': 'string', 'x-taskweave-point': 'joint_pose',
                                'description': 'named pose from robots/<robot>/poses.yaml'}},
        'additionalProperties': False,
    }
    needs = ['the motion path is free of obstacles']
    achieves = ['joints at the named pose']
    default_timeout_s = 60.0

    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self.ik = self.planner = None
        if robot.data.get('ik'):
            from taskweave_core.config.limits import joint_limits

            from .ik import IkClient
            self.ik = IkClient(node, robot, limits=joint_limits(config, robot.name))
        if robot.data.get('planning'):
            from .motion_plan import MotionPlanner
            self.planner = MotionPlanner(node, robot)

    def _refresh_ik(self) -> None:
        """The profile's current ik.choose (edited in the HMI) for the next solve."""
        if self.ik is not None:
            self.ik.set_choice(self.config.robot(self.robot.name).data['ik'].get('choose'))

    def execute(self, ctx, args):
        pose = self.config.named_pose(self.robot.name, args['pose'])
        self._refresh_ik()
        held = self.hold.held()
        if held is not None and (pose['tools'] or self._arm_joints(pose['joints'])):
            return self._holding_refusal()
        self.hold.reached_point(None, None, (), None)
        result, worst = self._reach(ctx, pose['joints'], pose['tools'], args['pose'], elbow=pose.get('elbow'))
        if result is not None:
            return result
        if held is not None:  # the arms kept their joints: the object moved with them (e.g. the lift)
            tool = held['tools'][0]
            try:
                tcp = self.hold.tcp_poses([tool])[tool]
            except LookupError as error:
                return SkillResult.failed(f'{args["pose"]} reached, but where is the held object now? {error}')
            self.hold.moved(tcp @ np.linalg.inv(held['tcp_in_object'][tool]))
        return SkillResult.succeeded(f'at {args["pose"]} (max error {worst:.3f})',
                                     world_state={'arm_pose': args['pose']})

    def _arm_joints(self, joints: dict) -> list[str]:
        """The joints of ``joints`` that belong to a tool's arm (all of them if the profile has no ik section)."""
        import re

        if self.ik is None:
            return list(joints)
        patterns = [re.compile(self.ik.joint_pattern.format(tool=tool)) for tool in self.ik.groups]
        return [name for name in joints if any(p.fullmatch(name) for p in patterns)]

    def _reach(self, ctx, joints: dict, tools: dict, label: str, fallback_joints=(), linear=False, flip=True,
               elbow=None):
        """Set ``joints`` and put each tool on its target (IK), along a planned path if there is a planner. If IK
        finds no solution with ``joints``, each of ``fallback_joints`` (e.g. other lift heights) is tried instead.
        ``linear``: the TCPs move on straight lines (cartesian.linear_path) instead of a planned path; the lift
        stays. Returns (failure result or None, worst joint error at the end)."""
        from .ik import IkError

        if linear:
            return self._linear(ctx, tools, label)
        goal = dict(joints)
        if (self.ik or self.planner) and self.joints.snapshot(self.p.state_timeout_s) is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)'), 0
        if tools:
            if self.ik is None:
                return SkillResult.rejected(f'{label} has tool targets, but the robot profile has no ik'), 0
            ctx.feedback(0.0, 'solving IK')
            first_error = None
            for attempt in [joints, *fallback_joints]:
                try:
                    goal = {**attempt, **self.ik.solve(tools, attempt, lambda: ctx.should_stop,
                                                       current=self.joints.snapshot(self.p.state_timeout_s),
                                                       allow_flip=flip, elbow=elbow)}
                    break
                except IkError as error:
                    first_error = first_error or error
            else:
                tried = f' (also tried {", ".join(str(a) for a in fallback_joints)})' if fallback_joints else ''
                return SkillResult.failed(f'{first_error}{tried}'), 0
        return self._go(ctx, goal, label)

    def _go(self, ctx, goal: dict, label: str):
        """Move to the joint ``goal`` along a planned path if there is a planner. Returns (failure or None, worst
        joint error at the end)."""
        from .motion_plan import PlanError

        path = None
        if self.planner is not None:
            ctx.feedback(0.0, 'planning')
            start = self.joints.snapshot(self.p.state_timeout_s)
            if start is None:
                return SkillResult.failed(f'no joint states on {self.joints.state_topic}'), 0
            try:
                path = self.planner.plan(start, goal, lambda: ctx.should_stop)
            except PlanError as error:
                return SkillResult.failed(f'{label}: {error}'), 0
        result, final = self._move(ctx, goal, label, path=path)
        if result is not None:
            return result, 0
        return None, max(abs(goal[n] - final[n]) for n in goal)


    def _linear(self, ctx, tools: dict, label: str, goals=None):
        """Move the TCPs of ``tools`` ({tool: {tcp, approach, closing}}, or ``goals``: {tool: 4x4}) on straight
        lines from where they are. Returns (failure result or None, worst joint error)."""
        import re

        from .cartesian import PathError, linear_path, matching_flip, tcp_targets

        if self.ik is None:
            return SkillResult.rejected(f'{label}: a straight-line motion needs the profile\'s ik section'), 0
        current = self.joints.snapshot(self.p.state_timeout_s)
        if current is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)'), 0
        goals = goals if goals is not None else tcp_targets(tools)
        patterns = [re.compile(self.ik.joint_pattern.format(tool=tool)) for tool in goals]
        arm = {n: v for n, v in current.items() if any(p.fullmatch(n) for p in patterns)}
        try:
            starts = self.hold.tcp_poses(goals)
            goals = matching_flip(goals, starts, self.robot.data.get('tool_frames', {}))
            ctx.feedback(0.0, 'straight-line path')
            path = linear_path(self.ik, starts, goals, arm, self.tcp_speed, self.tcp_turn)
        except (LookupError, PathError) as error:
            return SkillResult.failed(f'{label}: {error}'), 0
        goal = dict(zip(path.names, path.points[-1], strict=True))
        result, final = self._move(ctx, goal, label, path=path)
        if result is not None:
            return result, 0
        return None, max(abs(goal[n] - final[n]) for n in goal)


class GoToPoint(MoveToPose):
    name = 'go_to_point'
    description = ('Move one or both tools to a point on a placed object, for example the grasp point of a frame in a '
                   'bin: the point is taught relative to the object, the robot solves where its tools must go from '
                   'where it stands. offset_m stops short of the point along its approach direction (a pre-grasp '
                   'pose). Collision-free path with a motion planner to approach_m before the point, then a straight '
                   'line along the approach for the rest. The torso lift stays where it is unless the point is only '
                   'in reach at another of the configured lift heights.')
    args_schema = {
        'type': 'object',
        'required': ['point'],
        'properties': {
            'point': {'type': 'string', 'x-taskweave-point': 'instance_point',
                      'description': '<instance>/<point>: a placed object (cell/objects.yaml) and its point'},
            'offset_m': {'type': 'number', 'minimum': 0, 'description': 'stop this far back along the approach'},
            'approach_m': {'type': 'number', 'minimum': 0,
                           'description': 'the last part of the way, in a straight line along the approach '
                                          '(default from the skill config; 0 = planned path all the way)'},
            'tools': {'type': 'array', 'items': {'type': 'string'}, 'minItems': 1,
                      'description': 'only these tools (default: every tool the point has)'},
            'linear': {'type': 'boolean', 'description': 'move the TCPs on straight lines from where they are '
                                                         '(final approach), instead of a planned path'},
        },
        'additionalProperties': False,
    }
    needs = ["the robot stands where the point is in reach (e.g. at the object's stance)"]
    achieves = ['tools at the object point']
    default_timeout_s = 60.0

    def setup(self, node, config, robot, params):
        params = dict(params)
        self.cell_frame = params.pop('cell_frame', 'world')  # TF frame of the cell (object and location poses)
        # if the point is out of reach at the current lift: these heights of this joint are tried, in order
        self.lift_joint, self.lift_values = params.pop('lift_joint', None), list(params.pop('lift_values', []))
        self.tcp_speed = float(params.pop('tcp_speed', 0.05))  # m/s and rad/s for straight-line motions
        self.tcp_turn = float(params.pop('tcp_turn', 0.3))
        self.approach_m = float(params.pop('approach_m', 0.0))  # straight final approach (m) unless args say other
        super().setup(node, config, robot, params)

    def execute(self, ctx, args):
        from .cartesian import PathError, linear_path, tcp_targets
        from .ik import flipped
        from .points import instance_matrix, point_targets

        refusal = self._holding_refusal()
        if refusal is not None:
            return refusal
        try:
            base_in_cell = self.hold.transform(self.cell_frame, self.robot.base['frame'])
        except LookupError as error:
            return SkillResult.failed(f'where is the robot? {error}')
        # a symmetric object (e.g. a frame lying the other way round) is grasped from the robot's side
        base = Pose2D(float(base_in_cell[0, 3]), float(base_in_cell[1, 3]),
                      math.atan2(base_in_cell[1, 0], base_in_cell[0, 0]))
        try:
            instance, point = self.config.instance_point(args['point'], self.robot.name, base)
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        offset = float(args.get('offset_m', 0.0))
        try:
            tools = point_targets(point, instance, base_in_cell, offset, args.get('tools'))
        except KeyError as error:
            return SkillResult.rejected(f'{args["point"]}: {error.args[0]}')
        if not tools:
            return SkillResult.rejected(f'{args["point"]} has no tool poses')
        label = args['point'] + (f' (-{offset:g} m)' if offset else '') + (' (linear)' if args.get('linear') else '')
        lifts = [{self.lift_joint: float(v)} for v in self.lift_values] if self.lift_joint else []
        self.hold.reached_point(None, None, (), None)
        self._refresh_ik()
        elbow = point.get('elbow')  # the point's elbow side (robot frame), else the profile's
        default = point.get('approach_m') if point.get('approach_m') is not None else self.approach_m
        approach = 0.0 if args.get('linear') else float(args.get('approach_m', default))
        if approach > 0:  # planned (free) path to the start of the approach, then straight in
            before = point_targets(point, instance, base_in_cell, offset + approach, args.get('tools'))
            end = self._end_of_approach(ctx, tools, lifts, label, elbow)  # how the point is reached decides
            if isinstance(end, SkillResult):
                return end
            lift, flips, solution = end
            tools = {t: flipped(v) if t in flips else v for t, v in tools.items()}
            before = {t: flipped(v) if t in flips else v for t, v in before.items()}
            try:  # solved backwards from the point: the approach starts in the configuration that ends there
                path = linear_path(self.ik, tcp_targets(tools), tcp_targets(before), {**lift, **solution},
                                   self.tcp_speed, self.tcp_turn).reversed()
            except PathError as error:
                return SkillResult.failed(f'{label}: no straight {approach:g} m approach to the point: {error}')
            first, last = ({**lift, **dict(zip(path.names, path.points[i], strict=True))} for i in (0, -1))
            result, _ = self._go(ctx, first, f'{label} (-{approach:g} m approach)')
            if result is not None:
                return result
            label += f' ({approach:g} m straight)'
            result, final = self._move(ctx, last, label, path=path)
            worst = 0 if result is not None else max(abs(last[n] - final[n]) for n in last)
        else:
            result, worst = self._reach(ctx, {}, tools, label, fallback_joints=lifts, linear=bool(args.get('linear')),
                                        elbow=elbow)
        if result is not None:
            return result
        if offset == 0.0:  # at the point itself: closing the grippers now grasps this object
            self.hold.reached_point(args['point'], instance.name, list(tools),
                                    np.linalg.inv(base_in_cell) @ instance_matrix(instance))
        return SkillResult.succeeded(f'at {label} (max error {worst:.3f})', world_state={'arm_pose': label})


    def _end_of_approach(self, ctx, tools: dict, lifts: list, label: str, elbow=None):
        """Solve the point itself first: the lift height (current, else the first of ``lifts``), the tools whose grasp
        is reached flipped and the arm joints there, so the approach can be solved back from it."""
        from .ik import IkError

        if self.ik is None:
            return SkillResult.rejected(f'{label}: a straight approach needs the profile ik section')
        first_error = None
        for attempt in [{}, *lifts]:
            try:
                solution = self.ik.solve(tools, attempt, lambda: ctx.should_stop,
                                         current=self.joints.snapshot(self.p.state_timeout_s), elbow=elbow)
            except IkError as error:
                first_error = first_error or error
                continue
            return attempt, {tool for tool, info in self.ik.last.items() if info.get('flipped')}, solution
        return SkillResult.failed(f'{label}: the point itself is out of reach: {first_error}')


class MoveObject(GoToPoint):
    name = 'move_object'
    description = ('Move the object held with the grippers (after go_to_point to its point and gripper close): every '
                   'holding tool moves with it, locked in orientation and distance to the object and to each other. '
                   'The object moves on a straight line: by an offset in the robot\'s frame (x forward, y left, z up), '
                   'optionally turning about the vertical through the object and tilting it (far side up); to a '
                   'position in the robot\'s frame (at: where its origin goes, orientation kept); or to a '
                   'contact on a placed object: one of the held object\'s contacts goes onto it (e.g. a frame\'s hook '
                   'plate onto a hook), offset_m short of it against the contact\'s engage direction.')
    args_schema = {
        'type': 'object',
        'oneOf': [{'required': ['by']}, {'required': ['to', 'contact']}, {'required': ['at']}],
        'properties': {
            'by': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 3, 'maxItems': 3,
                   'description': '[x, y, z] in metres, robot frame'},
            'at': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 3, 'maxItems': 3,
                   'description': '[x, y, z] in metres, robot frame: where the object\'s origin goes'},
            'turn_deg': {'type': 'number', 'description': 'with by: turn about the vertical through the object (deg)'},
            'tilt_deg': {'type': 'number', 'description': 'with by: tilt about the robot\'s y axis through the object, '
                                                          'far side up (degrees)'},
            'to': {'type': 'string', 'x-taskweave-point': 'instance_contact',
                   'description': '<instance>/<contact>: where on a placed object the held object goes'},
            'contact': {'type': 'string', 'description': 'the held object\'s contact that goes there'},
            'offset_m': {'type': 'number', 'minimum': 0,
                         'description': 'with to: stop this far before the contact, against its engage direction'},
        },
        'additionalProperties': False,
    }
    needs = ['an object held with the grippers']
    achieves = ['the held object moved']
    default_timeout_s = 60.0

    def execute(self, ctx, args):
        import math

        hold = self.hold.held()
        if hold is None:
            return SkillResult.rejected('nothing is held: go_to_point to the object, then close the grippers')
        obj = hold['object_in_base']
        if 'at' in args:
            args = {**args, 'by': [float(a) - float(o) for a, o in zip(args['at'], obj[:3, 3], strict=True)]}
        if 'to' in args:
            moved = self._onto(hold, args)
            if isinstance(moved, SkillResult):
                return moved
            label = f'{hold["object"]} {args["contact"]} to {args["to"]}' + \
                (f' (-{args["offset_m"]:g} m)' if args.get('offset_m') else '')
        else:
            turn = math.radians(float(args.get('turn_deg', 0.0)))
            tilt = -math.radians(float(args.get('tilt_deg', 0.0)))  # about +y a positive angle lowers the far side
            c, s = math.cos(turn), math.sin(turn)
            ct, st = math.cos(tilt), math.sin(tilt)
            rotate = np.eye(4)
            rotate[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.array([[ct, 0, st], [0, 1, 0],
                                                                                     [-st, 0, ct]])
            pivot, back = np.eye(4), np.eye(4)
            pivot[:3, 3], back[:3, 3] = obj[:3, 3], -obj[:3, 3]
            shift = np.eye(4)
            shift[:3, 3] = [float(v) for v in args['by']]
            moved = shift @ pivot @ rotate @ back @ obj
            label = f'{hold["object"]} ' + (f'to {args["at"]}' if 'at' in args else
                                            f'by {[round(float(v), 3) for v in args["by"]]}') + \
                (f', {args["turn_deg"]:g} deg' if args.get('turn_deg') else '') + \
                (f', tilted {args["tilt_deg"]:g} deg' if args.get('tilt_deg') else '')
        goals = {tool: moved @ pose for tool, pose in hold['tcp_in_object'].items()}
        result, worst = self._linear(ctx, {}, label, goals=goals)
        if result is not None:
            return result
        self.hold.moved(moved)
        return SkillResult.succeeded(f'moved {label} (max error {worst:.3f})',
                                     world_state={'held_object': hold['object']})

    def _onto(self, hold: dict, args: dict):
        """The held object's pose (base frame) with its contact ``args['contact']`` on the placed contact
        ``args['to']``, ``offset_m`` back against that contact's z (engage direction); a SkillResult if that cannot be
        resolved."""
        from .points import onto_pose

        instances = self.config.object_instances()
        held = instances.get(hold['object'])
        if held is None:
            return SkillResult.rejected(f'the held {hold["object"]} is not in cell/objects.yaml: its contacts are unknown')
        try:
            target, contact = self.config.instance_contact(args['to'])
            mine = self.config.object_contact(held.object, args['contact'])
            base_in_cell = self.hold.transform(self.cell_frame, self.robot.base['frame'])
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        except LookupError as error:
            return SkillResult.failed(f'where is the robot? {error}')
        return onto_pose(target, contact, mine, base_in_cell, float(args.get('offset_m', 0.0)))


class Gripper(_JointSkill):
    name = 'gripper'
    description = ('Open or close the left, right or both grippers. Reports whether an object is held after closing. '
                   'Opening with place_as records where the released object now is, as a placed object of that name '
                   '(cell/objects.yaml), so it can be picked again or used to teach points on.')
    args_schema = {
        'type': 'object',
        'required': ['side', 'action'],
        'properties': {
            'side': {'enum': ['left', 'right', 'both']},
            'action': {'enum': ['open', 'close']},
            'place_as': {'type': 'string', 'description': 'with open: save the released object\'s pose under this '
                                                          'name in cell/objects.yaml'},
        },
        'additionalProperties': False,
    }

    def setup(self, node, config, robot, params):
        params = dict(params)
        self.cell_frame = params.pop('cell_frame', 'world')  # TF frame of the cell (place_as)
        super().setup(node, config, robot, params)
    achieves = ['gripper open, or closed (holding an object if one was between the fingers)']
    default_timeout_s = 20.0

    def execute(self, ctx, args):
        sides = ['left', 'right'] if args['side'] == 'both' else [args['side']]
        grippers = {side: self.robot.role(f'grippers.{side}') for side in sides}
        key = 'open' if args['action'] == 'open' else 'closed'
        goal = {g['joint']: float(g[key]) for g in grippers.values()}
        result, final = self._move(ctx, goal, f'{args["side"]} gripper {args["action"]}',
                                   stall_ok=args['action'] == 'close')
        if result is not None:
            return result
        state, holding = {}, {}
        for side, g in grippers.items():
            stopped_short = abs(final[g['joint']] - float(g[key])) > self.p.position_tolerance
            holding[side] = args['action'] == 'close' and stopped_short
            state[f'{side}_gripper'] = 'holding' if holding[side] else key
        message = ', '.join(f'{s}: {v}' for s, v in state.items())
        if args['action'] == 'close':
            grasped = self.hold.grasp(sides)  # at an object point reached by go_to_point
            if grasped:
                message += f'; holding {grasped}'
                state['held_object'] = grasped
        else:
            placed = self._placed(args['place_as'], sides) if args.get('place_as') else None
            released = self.hold.release(sides)
            if released:
                message += f'; released {released}'
                state['held_object'] = None
            if placed:
                message += f'; {placed}'
        return SkillResult.succeeded(message, holding=holding, world_state=state)

    def _placed(self, name: str, sides) -> str:
        """Save the held object's pose (cell frame, yaw only) as the placed object ``name``; says what was done."""
        import math

        hold = self.hold.held()
        if hold is None or not set(hold['tools']) & set(sides):
            return f'nothing held, {name} not placed'
        instance = self.config.object_instances().get(hold['object'])
        if instance is None:
            return f'{hold["object"]} is not a placed object, {name} not placed'
        try:
            pose = self.hold.transform(self.cell_frame, self.robot.base['frame']) @ hold['object_in_base']
        except LookupError as error:
            return f'where is the robot? {error}; {name} not placed'
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, pose[2, 2]))))
        path = self.config.save_object_instance(name, instance.object, float(pose[0, 3]), float(pose[1, 3]),
                                                float(pose[2, 3]), yaw)
        return (f'placed {name} at ({pose[0, 3]:.3f}, {pose[1, 3]:.3f}, {pose[2, 3]:.3f}), yaw '
                f'{math.degrees(yaw):.1f} deg in {path.name}' + (f' (tilted {tilt:.0f} deg: stored level)'
                                                                  if tilt > 2 else ''))
