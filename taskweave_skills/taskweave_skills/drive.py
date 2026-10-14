"""drive_straight: a short straight move of the base, forward or backward, keeping its heading.

For the last bit into a tight spot and back out of it, where navigate_to cannot go: Nav2 sees the robot's footprint
(arms reaching out, a carried object) overlap what the arms reach over, and the direct controller of a differential
base turns to face its target first, which would swing the arms round. Here the base keeps its heading and only drives
along it (a holonomic base may also go sideways). No obstacle avoidance: the mission says where this is safe, and
``max_distance_m`` bounds it.

Parameters (``skills/drive_straight.yaml``), besides navigate_to's direct-controller ones (odometry, limits):
  speed: 0.15                 m/s
  max_distance_m: 1.0         longer moves are rejected
"""

from __future__ import annotations

import math
import time

from taskweave_core.config import wrap_angle
from taskweave_core.skill import SkillResult

from .navigate import NavigateTo


class DriveStraight(NavigateTo):
    name = 'drive_straight'
    description = ('Drive the base straight forward (positive) or backward (negative) by a short distance, keeping its '
                   'heading, without obstacle avoidance: to move up to a rack or table the arms reach over, and back '
                   'out again. A holonomic base can also move sideways.')
    args_schema = {
        'type': 'object',
        'required': ['distance_m'],
        'properties': {
            'distance_m': {'type': 'number', 'description': 'forward (+) or backward (-), metres'},
            'sideways_m': {'type': 'number', 'description': 'left (+) or right (-), metres; holonomic bases only'},
        },
        'additionalProperties': False,
    }
    needs = ['the strip the base drives over is free']
    achieves = ['robot base moved by the distance']
    default_timeout_s = 60.0

    def setup(self, node, config, robot, params):
        params = dict(params)
        self.speed = float(params.pop('speed', 0.15))
        self.max_distance = float(params.pop('max_distance_m', 1.0))
        super().setup(node, config, robot, params)

    def execute(self, ctx, args):
        forward, sideways = float(args['distance_m']), float(args.get('sideways_m', 0.0))
        if self.base['type'] == 'fixed':
            return SkillResult.rejected('robot base is fixed')
        if sideways and self.base['type'] != 'holonomic':
            return SkillResult.rejected(f'a {self.base["type"]} base cannot move sideways')
        if math.hypot(forward, sideways) > self.max_distance:
            return SkillResult.rejected(f'{math.hypot(forward, sideways):.2f} m is more than max_distance_m '
                                        f'{self.max_distance:g}: use navigate_to')
        start = self._current()
        if start is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        c, s = math.cos(start.yaw), math.sin(start.yaw)
        tx, ty = start.x + c * forward - s * sideways, start.y + s * forward + c * sideways
        speed = min(self.speed, self.p.max_linear)
        total = max(math.hypot(forward, sideways), 1e-6)
        best, best_time, settled_since = math.inf, ctx.now(), None
        previous, last = (0.0, 0.0), ctx.now()
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
            dx, dy = tx - pose.x, ty - pose.y
            ex = math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy      # remaining, in the base frame
            ey = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
            yaw_error = wrap_angle(start.yaw - pose.yaw)
            remaining = math.hypot(ex, ey) if sideways else abs(ex)    # a differential base cannot correct ey
            if remaining <= self.p.position_tolerance:
                settled_since = settled_since if settled_since is not None else now
                if now - settled_since >= self.p.settle_s:
                    self.stop()
                    return SkillResult.succeeded(f'drove {forward:+.2f} m' + (f', {sideways:+.2f} m sideways'
                                                                             if sideways else '')
                                                 + f' ({remaining * 1000:.0f} mm off)')
            else:
                settled_since = None
            if remaining < best - self.p.stuck_progress_m:
                best, best_time = remaining, now
            elif now - best_time > self.p.stuck_timeout_s:
                self.stop()
                return SkillResult.failed(f'no progress for {self.p.stuck_timeout_s:g} s, {remaining:.2f} m to go')
            vx = max(-speed, min(speed, self.p.k_linear * ex))
            vy = max(-speed, min(speed, self.p.k_linear * ey)) if sideways else 0.0
            step = self.p.linear_accel * dt
            vx = max(previous[0] - step, min(previous[0] + step, vx))
            vy = max(previous[1] - step, min(previous[1] + step, vy))
            previous = (vx, vy)
            self._publish(vx, vy, max(-self.p.max_angular, min(self.p.max_angular, self.p.k_angular * yaw_error)))
            ctx.feedback(1.0 - min(remaining / total, 1.0), f'{remaining:.2f} m to go')
            time.sleep(1.0 / self.p.rate_hz)
