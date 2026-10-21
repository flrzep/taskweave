"""navigate_to: drive the base to a named location or stance with a simple pose controller.

A direct controller without obstacle avoidance; taskweave_skills.nav2 serves the same skill through Nav2 and uses
this controller for the final approach.
Robot roles used: ``base.type``, ``base.cmd_vel``, ``base.odom`` (pose in the cell frame), ``base.limits``.
With the profile's ``torso: {joint, drive_m, tolerance_m}`` the torso goes to ``drive_m`` before the base moves (a
move_joints step: planner, speed override, joint limits), unless it is within ``tolerance_m`` of it or a hand holds
something (the carry height stays; the result says so). The result's ``data.torso_drive`` reports it; drive_straight
and the Nav2 implementation do the same. Without ``drive_m`` the torso is never touched.
In a cell in the targets layout this class is ``move.base`` (parameters ``skills/move/base.yaml``): ``Move`` gives it
the resolved pose, the torso (``torso`` of the step: ``keep``, or the joint values to go to instead of ``drive_m``)
and the step's ``motion.speed`` and ``motion.accel`` (fractions) in ``args[RESOLVED]`` (``pose``, ``torso``, ``speed``,
``accel``).
Parameters (``skills/navigate_to.yaml``; values below are defaults, each capped by the robot's base limits):
"""

from __future__ import annotations

import functools
import math
import threading
import time
from dataclasses import dataclass, fields, replace

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from taskweave_core.config import Pose2D, wrap_angle
from taskweave_core.config.limits import base_limits, capped_by
from taskweave_core.config.schemas import ROBOT_PROFILE, default
from taskweave_core.executive import Outcome
from taskweave_core.skill import Skill, SkillResult

from .parts import RESOLVED, part_key

MIN_DT_S = 1e-3           # s, floor of a control step (first tick, coarse clock): keeps the accel limits finite
TORSO_TOLERANCE_M = default(ROBOT_PROFILE, 'torso', 'tolerance_m')   # torso.tolerance_m default
REVERSE_HYSTERESIS = 0.3  # rad, differential base: once reversing, keep reversing until the target is this much
                          # further in front than straight across


@dataclass
class Params:
    max_linear: float = 0.5          # m/s
    max_angular: float = 1.0         # rad/s
    linear_accel: float = 0.5        # m/s^2
    angular_accel: float = 1.5       # rad/s^2
    brake_share: float = 0.8         # commands slow down at this share of the accelerations
    lead_s: float = 0.0              # s, steer from where the last command takes the base this much later: makes up for
                                     # a base that follows commands late and odometry that arrives late (no overshoot)
    k_linear: float = 1.2            # 1/s, proportional gain on position error
    k_angular: float = 2.0           # 1/s, proportional gain on heading error
    position_tolerance: float = 0.03  # m
    yaw_tolerance: float = 0.03      # rad
    settle_s: float = 0.3            # s inside tolerance before success
    turn_in_place: float = 0.5       # rad, differential base: rotate first when heading error is larger
    reverse_within_m: float = 0.5    # m, differential base: a point behind it and this close is reached backwards
                                     # (not turned towards and back again)
    stuck_timeout_s: float = 5.0     # s without stuck_progress_m of improvement -> fail
    stuck_progress_m: float = 0.02   # m, progress = distance + yaw_weight * heading error
    yaw_weight: float = 0.2          # m per rad, how heading error counts toward progress
    odom_timeout_s: float = 1.0      # s without odometry -> fail
    rate_hz: float = 20.0
    override_slew_per_s: float = 2.0  # the speed override changes the velocity limits by at most this (fraction/s)

    @classmethod
    def from_config(cls, params: dict, limits: dict) -> Params:
        known = {f.name for f in fields(cls)}
        unknown = set(params) - known
        if unknown:
            raise ValueError(f'unknown navigate_to parameters: {", ".join(sorted(unknown))}')
        result = cls(**{k: float(v) for k, v in params.items()})
        result.max_linear = min(result.max_linear, limits.get('linear', math.inf))
        result.max_angular = min(result.max_angular, limits.get('angular', math.inf))
        return result


__doc__ += '\n'.join(f'  {f.name}: {f.default}' for f in fields(Params))


def _limit(value: float, previous: float, maximum: float, max_step: float) -> float:
    value = max(-maximum, min(maximum, value))
    return max(previous - max_step, min(previous + max_step, value))


def _toward(error: float, gain: float, accel: float) -> float:
    """Speed toward ``error``: proportional, but never faster than the base can stop from within the error (braking
    at ``accel``). A pure P law asks for gain x speed of deceleration, more than ``accel`` allows near full speed, and
    the base runs past the target."""
    return math.copysign(min(gain * abs(error), math.sqrt(2.0 * accel * abs(error))), error)


def scaled(p: Params, override: float, accel: float = 1.0) -> Params:
    """``p`` with the velocity limits at the speed override (a fraction) and the accelerations at ``accel`` (a
    fraction); ``p`` itself at 100 %."""
    if override >= 1.0 and accel >= 1.0:
        return p
    override = min(override, 1.0)
    return replace(p, max_linear=p.max_linear * override, max_angular=p.max_angular * override,
                   linear_accel=p.linear_accel * accel, angular_accel=p.angular_accel * accel)


def command(pose: Pose2D, target: Pose2D, base_type: str, p: Params, previous: tuple, dt: float) -> tuple:
    """One control step. Returns (vx, vy, wz) in the base frame."""
    brake_linear, brake_angular = p.brake_share * p.linear_accel, p.brake_share * p.angular_accel
    if p.lead_s > 0:  # where the base will be when this command takes effect
        vx, vy, wz = previous
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        pose = Pose2D(pose.x + (c * vx - s * vy) * p.lead_s, pose.y + (s * vx + c * vy) * p.lead_s,
                      wrap_angle(pose.yaw + wz * p.lead_s))
    dx, dy = target.x - pose.x, target.y - pose.y
    distance = math.hypot(dx, dy)
    yaw_error = wrap_angle(target.yaw - pose.yaw)
    cos_yaw, sin_yaw = math.cos(pose.yaw), math.sin(pose.yaw)
    ex, ey = cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy  # error in the base frame
    if base_type == 'holonomic':
        speed = min(_toward(distance, p.k_linear, brake_linear), p.max_linear)
        vx, vy = (ex / distance * speed, ey / distance * speed) if distance > 1e-6 else (0.0, 0.0)
        wz = _toward(yaw_error, p.k_angular, brake_angular)
    else:  # differential: face the target (or, close by, turn its back to it), drive, then turn to the final heading
        heading_error = wrap_angle(math.atan2(dy, dx) - pose.yaw) if distance > 1e-6 else 0.0
        direction = 1.0
        across = math.pi / 2 - (REVERSE_HYSTERESIS if previous[0] < 0 else 0.0)
        if distance <= p.reverse_within_m and abs(heading_error) > across:
            heading_error, direction = wrap_angle(heading_error + math.pi), -1.0  # hysteresis: keeps reversing
        far = distance > p.position_tolerance
        # Dead band: between half and full tolerance only creep on if already facing the point. Turning towards a
        # point a few millimetres away would fight the final-heading turn and never settle.
        near_but_facing = distance > 0.5 * p.position_tolerance and abs(heading_error) < p.turn_in_place / 4
        if far and abs(heading_error) > p.turn_in_place:
            vx, wz = 0.0, _toward(heading_error, p.k_angular, brake_angular)
        elif far or near_but_facing:
            speed = direction * min(_toward(distance, p.k_linear, brake_linear), p.max_linear) * math.cos(heading_error)
            vx, wz = speed, _toward(heading_error if far else yaw_error, p.k_angular, brake_angular)
        else:
            vx, wz = 0.0, _toward(yaw_error, p.k_angular, brake_angular)
        vy = 0.0
    return (_limit(vx, previous[0], p.max_linear, p.linear_accel * dt),
            _limit(vy, previous[1], p.max_linear, p.linear_accel * dt),
            _limit(wz, previous[2], p.max_angular, p.angular_accel * dt))


def _with_drive_height(execute):
    """``execute`` of a skill that drives the base, preceded by the move of the torso to the profile's torso.drive_m
    and followed by its report in ``data.torso_drive``. A torso move that fails ends the step before the base moves."""
    @functools.wraps(execute)
    def wrapper(self, ctx, args):
        if self.base['type'] == 'fixed':
            return execute(self, ctx, args)
        resolved = (args.get(RESOLVED) if isinstance(args, dict) else None) or {}
        self._fraction, self._accel_fraction = resolved.get('speed') or 1.0, resolved.get('accel') or 1.0
        drive, failure = self._drive_height(ctx, resolved.get('torso'))
        if failure is not None:
            return failure
        result = execute(self, ctx, args)
        if drive and hasattr(result, 'data'):
            result.data['torso_drive'] = drive
        return result
    return wrapper


class NavigateTo(Skill):
    name = 'navigate_to'
    description = ('Drive the robot base to a named location or stance and stop there, '
                   'facing the stored heading. Direct path, no obstacle avoidance. The Nav2 implementation '
                   '(taskweave_skills.nav2:Nav2NavigateTo) plans around obstacles and drives its Nav2 phase '
                   'never slower than its param min_override (default 0.3): at a 10 % override Nav2 drives '
                   '30 %, the direct controller 10 %. The step result reports it as data.speed.nav2_override.')
    args_schema = {
        'type': 'object',
        'required': ['target'],
        'properties': {
            'target': {'type': 'string', 'x-taskweave-point': 'base_target',
                       'description': 'location name, "location:<name>" or "stance:<name>"'},
        },
        'additionalProperties': False,
    }
    needs = ['the path to the target is free']
    achieves = ['robot base at the target pose']
    default_timeout_s = 120.0

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if 'execute' in cls.__dict__:
            cls.execute = _with_drive_height(cls.__dict__['execute'])

    def configure(self, params):
        super().configure(params)
        self.base = self.robot.base
        limits = base_limits(self.config, self.robot.name)
        self.p = Params.from_config(params, limits)
        wanted = Params.from_config(params, {})
        self.capped = capped_by({'linear': wanted.max_linear, 'angular': wanted.max_angular}, limits)  # motion.capped

    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self._fraction = self._accel_fraction = 1.0   # the step's motion.speed and motion.accel (Move)
        self.pose: Pose2D | None = None
        self.pose_time = -math.inf
        self.lock = threading.Lock()
        self.cmd = node.create_publisher(Twist, self.base['cmd_vel'], 10)
        node.create_subscription(Odometry, self.base['odom'], self._odom, 10)
        self._lift = None   # the torso's move to torso.drive_m: a move_joints of its own, for robots with that key
        if robot.data.get('joints') and ((robot.data.get('torso') or {}).get('joint') or config.targets_format()):
            from .joint_motion import MoveJoints
            key = part_key(config, 'move_joints')
            implementation, _ = config.select_implementation(key, robot.name)
            self._lift = MoveJoints()
            self._lift.name = key
            self._lift.setup(node, config, robot, config.skill_params(key, implementation))

    def _drive_height(self, ctx, torso=None):
        """Move the torso before the base moves: (what to report in the result or None, a failure result or None).
        ``torso``: None the profile's torso.drive_m (nothing without it), ``keep`` nothing, or {joint: value} the
        joints to go to instead (``Move``'s torso option or the joints of a base target). Not while a hand holds
        something (the object would go with the torso)."""
        spec = self.config.robot(self.robot.name).data.get('torso') or {}
        if torso == 'keep':
            return None, None
        if isinstance(torso, dict) and torso:
            goal, what = {joint: float(value) for joint, value in torso.items()}, 'torso to ' + ', '.join(
                f'{joint} {float(value):g}' for joint, value in torso.items())
        else:
            if spec.get('drive_m') is None:
                return None, None
            goal = {spec['joint']: float(spec['drive_m'])}
            what = f'torso to the drive height {float(spec["drive_m"]):g}'
        if self._lift is None:
            return None, SkillResult.failed('the torso is to move before driving, but this skill server started '
                                             'without a joint interface: restart it')
        joint = next(iter(goal))
        here = self._lift.joints.current(list(goal), self._lift.p.state_timeout_s)
        if here is None:
            return None, SkillResult.failed(f'no joint states on {self._lift.joints.state_topic} (torso {joint})')
        drive = {'joint': joint, 'from_m': round(here[joint], 4), 'to_m': goal[joint], 'moved': False}
        if len(goal) > 1:
            drive['joints'] = dict(goal)
        holding = self._lift.hold.hand_holds
        if holding:
            drive['skipped'] = f'{", ".join(sorted(set(holding.values())))} held: the torso stays at its carry height'
            return drive, None
        tolerance = float(spec.get('tolerance_m', TORSO_TOLERANCE_M))
        if all(abs(here[name] - value) <= tolerance for name, value in goal.items()):
            return drive, None
        ctx.feedback(0.0, what)
        result = self._lift.execute(ctx, {'joints': goal})
        if result.outcome != Outcome.SUCCEEDED:
            return None, replace(result, message=f'{what}: {result.message}')
        drive['moved'] = True
        return drive, None

    def _odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        with self.lock:
            self.pose, self.pose_time = pose, time.monotonic()

    def _current(self) -> Pose2D | None:
        with self.lock:
            fresh = time.monotonic() - self.pose_time < self.p.odom_timeout_s
            return self.pose if fresh else None

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.cmd.publish(msg)

    def stop(self) -> None:
        self._publish(0.0, 0.0, 0.0)

    def _capped_data(self) -> dict:
        """Result data naming the base limits that held the speeds down (``motion.capped``), if any."""
        return {'motion': {'capped': dict(self.capped)}} if self.capped else {}

    def execute(self, ctx, args):
        if self.base['type'] == 'fixed':
            return SkillResult.rejected('robot base is fixed')
        target = self._goal_pose(args)
        if self._current() is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        return self._drive(ctx, target, args['target'])

    def _goal_pose(self, args) -> Pose2D:
        """The pose to drive to (cell frame): what ``Move`` resolved, else the location or stance ``args.target``
        names (the older cell layouts)."""
        if args.get(RESOLVED):
            return args[RESOLVED]['pose']
        return self.config.base_target(args['target'], self.robot.name, self._current())

    execute = _with_drive_height(execute)

    def _drive(self, ctx, target: Pose2D, label: str) -> SkillResult:
        """Direct controller loop to ``target`` (cell frame) until settled, stuck, lost or interrupted. The velocity
        limits follow the speed override every cycle; the stuck check runs on work time (stuck_timeout_s / o)."""
        pose = self._current()
        if pose is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        start_distance = max(math.hypot(target.x - pose.x, target.y - pose.y), 1e-6)
        clock = ctx.work_clock(self.p.override_slew_per_s)
        best, best_time, settled_since, phase_travelling = math.inf, clock(ctx.now()), None, None
        previous, last = (0.0, 0.0, 0.0), ctx.now()
        period = 1.0 / self.p.rate_hz
        while True:
            if ctx.should_stop:
                self.stop()
                return ctx.interrupted()
            pose = self._current()
            if pose is None:
                self.stop()
                return SkillResult.failed(f'odometry lost on {self.base["odom"]}')
            now = ctx.now()
            dt, last = max(now - last, MIN_DT_S), now
            distance = math.hypot(target.x - pose.x, target.y - pose.y)
            yaw_error = abs(wrap_angle(target.yaw - pose.yaw))
            if distance <= self.p.position_tolerance and yaw_error <= self.p.yaw_tolerance:
                settled_since = settled_since if settled_since is not None else now
                if now - settled_since >= self.p.settle_s:
                    self.stop()
                    return SkillResult.succeeded(
                        f'at {label} ({distance * 1000:.0f} mm, {math.degrees(yaw_error):.1f} deg)',
                        world_state={'robot_location': label}, **self._capped_data())
            else:
                settled_since = None
            # A differential base first turns towards the target: while travelling, progress counts its heading to the
            # target, not the final heading (which may get worse during that turn). A new phase restarts the check.
            travelling = self.base['type'] != 'holonomic' and distance > self.p.position_tolerance
            work = clock(now)
            if travelling != phase_travelling:
                phase_travelling, best, best_time = travelling, math.inf, work
            turn = abs(wrap_angle(math.atan2(target.y - pose.y, target.x - pose.x) - pose.yaw)) if travelling \
                else yaw_error
            if travelling and distance <= self.p.reverse_within_m:
                turn = min(turn, math.pi - turn)  # close by, a point behind is reached backwards
            remaining = distance + self.p.yaw_weight * turn
            if remaining < best - self.p.stuck_progress_m:
                best, best_time = remaining, work
            elif work - best_time > self.p.stuck_timeout_s:
                self.stop()
                return SkillResult.failed(f'no progress for {self.p.stuck_timeout_s:g} s, {distance:.2f} m from target')
            previous = command(pose, target, self.base['type'],
                               scaled(self.p, clock.o * self._fraction, self._accel_fraction), previous, dt)
            self._publish(*previous)
            ctx.feedback(1.0 - min(distance / start_distance, 1.0), f'{distance:.2f} m to go')
            time.sleep(period)
