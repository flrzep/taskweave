"""Joint-space skills for robots that take joint position targets on a topic (sensor_msgs/JointState).

Robot roles used (robots/<robot>/profile.yaml):
    joints:   {command: /joint_command, state: /joint_states}
    grippers: {<side>: {joint: <name>, open: <value>, closed: <value>}}
Named poses: robots/<robot>/poses.yaml, ``<pose>: {<joint>: <value>, ...}`` (rad or m).

These are the simple implementations. A ros2_control / MoveIt implementation can later serve the same skill names
(see skills/<skill>.yaml implementations) without changing missions.
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
    linear_joints: tuple = ()          # joint names that are prismatic (metres)
    position_tolerance: float = 0.02   # rad or m
    settle_s: float = 0.3              # within tolerance this long before success
    stall_timeout_s: float = 3.0       # after the planned move: no progress this long -> fail / holding
    min_duration_s: float = 0.5
    rate_hz: float = 50.0
    state_timeout_s: float = 1.0       # no joint states this long -> fail

    @classmethod
    def from_config(cls, params: dict) -> Params:
        known = {f.name for f in fields(cls)}
        unknown = set(params) - known
        if unknown:
            raise ValueError(f'unknown joint skill parameters: {", ".join(sorted(unknown))}')
        values = {k: (tuple(v) if k == 'linear_joints' else float(v)) for k, v in params.items()}
        return cls(**values)


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
        self.joints = JointInterface.for_node(node, robot)
        self._held: dict = {}

    def stop(self) -> None:
        """Hold where the joints are now."""
        if self._held:
            current = self.joints.current(list(self._held), self.p.state_timeout_s)
            if current:
                self.joints.command(current)

    def _move(self, ctx, goal: dict, label: str, stall_ok: bool = False):
        """Move the joints in ``goal``; returns (result or None, final positions). ``stall_ok``: stopping early
        (e.g. fingers blocked by an object) ends the move without failing."""
        self._held = goal
        start = self.joints.current(list(goal), self.p.state_timeout_s)
        if start is None:
            missing = [n for n in goal if n not in self.joints.positions]
            reason = f'unknown joints {missing}' if missing else f'no joint states on {self.joints.state_topic}'
            return SkillResult.failed(reason), None
        duration = plan_duration(start, goal, self.p)
        t0 = ctx.now()
        period = 1.0 / self.p.rate_hz
        settled_since, best, best_time = None, math.inf, None
        while True:
            if ctx.should_stop:
                self.stop()
                return ctx.interrupted(), None
            now = ctx.now()
            fraction = (now - t0) / duration
            self.joints.command(interpolate(start, goal, fraction))
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
    description = ('Move joints to a named pose of this robot (for example home or carry) and hold it. '
                   'Joint-space motion without collision checking.')
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

    def execute(self, ctx, args):
        goal = self.config.joint_pose(self.robot.name, args['pose'])
        result, final = self._move(ctx, goal, args['pose'])
        if result is not None:
            return result
        worst = max(abs(goal[n] - final[n]) for n in goal)
        return SkillResult.succeeded(f'at {args["pose"]} (max error {worst:.3f})',
                                     world_state={'arm_pose': args['pose']})


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
