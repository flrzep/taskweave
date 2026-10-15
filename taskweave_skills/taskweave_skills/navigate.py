"""navigate_to: drive the base to a named location or stance with a simple pose controller.

A direct controller without obstacle avoidance; taskweave_skills.nav2 serves the same skill through Nav2 and uses
this controller for the final approach.
Robot roles used: ``base.type``, ``base.cmd_vel``, ``base.odom`` (pose in the cell frame), ``base.limits``.
Parameters (``skills/navigate_to.yaml``; values below are defaults, each capped by the robot's base limits):
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, fields

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from taskweave_core.config import Pose2D, wrap_angle
from taskweave_core.skill import Skill, SkillResult


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
    stuck_timeout_s: float = 5.0     # s without stuck_progress_m of improvement -> fail
    stuck_progress_m: float = 0.02   # m, progress = distance + yaw_weight * heading error
    yaw_weight: float = 0.2          # m per rad, how heading error counts toward progress
    odom_timeout_s: float = 1.0      # s without odometry -> fail
    rate_hz: float = 20.0

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
    else:  # differential: face the target, drive, then turn to the final heading
        heading_error = wrap_angle(math.atan2(dy, dx) - pose.yaw) if distance > 1e-6 else 0.0
        far = distance > p.position_tolerance
        # Dead band: between half and full tolerance only creep on if already facing the point. Turning towards a
        # point a few millimetres away would fight the final-heading turn and never settle.
        near_but_facing = distance > 0.5 * p.position_tolerance and abs(heading_error) < p.turn_in_place / 4
        if far and abs(heading_error) > p.turn_in_place:
            vx, wz = 0.0, _toward(heading_error, p.k_angular, brake_angular)
        elif far or near_but_facing:
            speed = min(_toward(distance, p.k_linear, brake_linear), p.max_linear) * math.cos(heading_error)
            vx, wz = speed, _toward(heading_error if far else yaw_error, p.k_angular, brake_angular)
        else:
            vx, wz = 0.0, _toward(yaw_error, p.k_angular, brake_angular)
        vy = 0.0
    return (_limit(vx, previous[0], p.max_linear, p.linear_accel * dt),
            _limit(vy, previous[1], p.max_linear, p.linear_accel * dt),
            _limit(wz, previous[2], p.max_angular, p.angular_accel * dt))


class NavigateTo(Skill):
    name = 'navigate_to'
    description = ('Drive the robot base to a named location or stance and stop there, '
                   'facing the stored heading. Direct path, no obstacle avoidance.')
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

    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self.base = robot.base
        self.p = Params.from_config(params, self.base.get('limits', {}))
        self.pose: Pose2D | None = None
        self.pose_time = -math.inf
        self.lock = threading.Lock()
        self.cmd = node.create_publisher(Twist, self.base['cmd_vel'], 10)
        node.create_subscription(Odometry, self.base['odom'], self._odom, 10)

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

    def execute(self, ctx, args):
        if self.base['type'] == 'fixed':
            return SkillResult.rejected('robot base is fixed')
        target = self.config.base_target(args['target'], self.robot.name, self._current())
        if self._current() is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        return self._drive(ctx, target, args['target'])

    def _drive(self, ctx, target: Pose2D, label: str) -> SkillResult:
        """Direct controller loop to ``target`` (cell frame) until settled, stuck, lost or interrupted."""
        pose = self._current()
        if pose is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        start_distance = max(math.hypot(target.x - pose.x, target.y - pose.y), 1e-6)
        best, best_time, settled_since, phase_travelling = math.inf, ctx.now(), None, None
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
            dt, last = max(now - last, 1e-3), now
            distance = math.hypot(target.x - pose.x, target.y - pose.y)
            yaw_error = abs(wrap_angle(target.yaw - pose.yaw))
            if distance <= self.p.position_tolerance and yaw_error <= self.p.yaw_tolerance:
                settled_since = settled_since if settled_since is not None else now
                if now - settled_since >= self.p.settle_s:
                    self.stop()
                    return SkillResult.succeeded(
                        f'at {label} ({distance * 1000:.0f} mm, {math.degrees(yaw_error):.1f} deg)',
                        world_state={'robot_location': label})
            else:
                settled_since = None
            # A differential base first turns towards the target: while travelling, progress counts its heading to the
            # target, not the final heading (which may get worse during that turn). A new phase restarts the check.
            travelling = self.base['type'] != 'holonomic' and distance > self.p.position_tolerance
            if travelling != phase_travelling:
                phase_travelling, best, best_time = travelling, math.inf, now
            turn = abs(wrap_angle(math.atan2(target.y - pose.y, target.x - pose.x) - pose.yaw)) if travelling \
                else yaw_error
            remaining = distance + self.p.yaw_weight * turn
            if remaining < best - self.p.stuck_progress_m:
                best, best_time = remaining, now
            elif now - best_time > self.p.stuck_timeout_s:
                self.stop()
                return SkillResult.failed(f'no progress for {self.p.stuck_timeout_s:g} s, {distance:.2f} m from target')
            previous = command(pose, target, self.base['type'], self.p, previous, dt)
            self._publish(*previous)
            ctx.feedback(1.0 - min(distance / start_distance, 1.0), f'{distance:.2f} m to go')
            time.sleep(period)
