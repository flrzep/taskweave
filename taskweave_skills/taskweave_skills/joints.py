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

from sensor_msgs.msg import JointState

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
                   'straight joint-space move.')
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
            from .ik import IkClient
            self.ik = IkClient(node, robot)
        if robot.data.get('planning'):
            from .motion_plan import MotionPlanner
            self.planner = MotionPlanner(node, robot)

    def execute(self, ctx, args):
        pose = self.config.named_pose(self.robot.name, args['pose'])
        result, worst = self._reach(ctx, pose['joints'], pose['tools'], args['pose'])
        if result is not None:
            return result
        return SkillResult.succeeded(f'at {args["pose"]} (max error {worst:.3f})',
                                     world_state={'arm_pose': args['pose']})

    def _reach(self, ctx, joints: dict, tools: dict, label: str):
        """Set ``joints`` and put each tool on its target (IK), along a planned path if there is a planner.
        Returns (failure result or None, worst joint error at the end)."""
        from .ik import IkError
        from .motion_plan import PlanError

        goal = dict(joints)
        if (self.ik or self.planner) and self.joints.snapshot(self.p.state_timeout_s) is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)'), 0
        if tools:
            if self.ik is None:
                return SkillResult.rejected(f'{label} has tool targets, but the robot profile has no ik'), 0
            ctx.feedback(0.0, 'solving IK')
            try:
                goal.update(self.ik.solve(tools, joints, lambda: ctx.should_stop))
            except IkError as error:
                return SkillResult.failed(str(error)), 0
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


class GoToPoint(MoveToPose):
    name = 'go_to_point'
    description = ('Move one or both tools to a point on a placed object, for example the grasp point of a frame in a '
                   'bin: the point is taught relative to the object, the robot solves where its tools must go from '
                   'where it stands. offset_m stops short of the point along its approach direction (a pre-grasp '
                   'pose). Collision-free path with a motion planner; the torso lift stays where it is.')
    args_schema = {
        'type': 'object',
        'required': ['point'],
        'properties': {
            'point': {'type': 'string', 'x-taskweave-point': 'object_point',
                      'description': '<instance>/<point>: a placed object (cell/objects.yaml) and its point'},
            'offset_m': {'type': 'number', 'minimum': 0, 'description': 'stop this far back along the approach'},
            'tools': {'type': 'array', 'items': {'type': 'string'}, 'minItems': 1,
                      'description': 'only these tools (default: every tool the point has)'},
        },
        'additionalProperties': False,
    }
    needs = ["the robot stands where the point is in reach (e.g. at the object's stance)"]
    achieves = ['tools at the object point']
    default_timeout_s = 60.0

    def setup(self, node, config, robot, params):
        params = dict(params)
        self.cell_frame = params.pop('cell_frame', 'world')  # TF frame of the cell (object and location poses)
        super().setup(node, config, robot, params)
        from tf2_ros import Buffer, TransformListener
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, node)

    def execute(self, ctx, args):
        import rclpy

        from .ik import pose_matrix
        from .points import point_targets

        try:
            instance, point = self.config.instance_point(args['point'])
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        base = self.robot.base['frame']
        try:
            t = self.tf.lookup_transform(self.cell_frame, base, rclpy.time.Time()).transform
        except Exception as error:  # noqa: BLE001 - several tf2 exception types
            return SkillResult.failed(f'where is the robot? no transform {self.cell_frame} -> {base}: {error}')
        try:
            tools = point_targets(point, instance, pose_matrix(t.translation, t.rotation),
                                  float(args.get('offset_m', 0.0)), args.get('tools'))
        except KeyError as error:
            return SkillResult.rejected(f'{args["point"]}: {error.args[0]}')
        if not tools:
            return SkillResult.rejected(f'{args["point"]} has no tool poses')
        label = args['point'] + (f' (-{args["offset_m"]:g} m)' if args.get('offset_m') else '')
        result, worst = self._reach(ctx, {}, tools, label)
        if result is not None:
            return result
        return SkillResult.succeeded(f'at {label} (max error {worst:.3f})', world_state={'arm_pose': label})


class Gripper(_JointSkill):
    name = 'gripper'
    description = 'Open or close the left, right or both grippers. Reports whether an object is held after closing.'
    args_schema = {
        'type': 'object',
        'required': ['side', 'action'],
        'properties': {
            'side': {'enum': ['left', 'right', 'both']},
            'action': {'enum': ['open', 'close']},
        },
        'additionalProperties': False,
    }
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
        return SkillResult.succeeded(', '.join(f'{s}: {v}' for s, v in state.items()), holding=holding,
                                     world_state=state)
