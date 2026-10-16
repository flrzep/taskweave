"""drive_straight: a short straight move of the base, forward or backward, keeping its heading.

For the last bit into a tight spot and back out of it, where navigate_to cannot go: Nav2 sees the robot's footprint
(arms reaching out, a carried object) overlap what the arms reach over, and the direct controller of a differential
base turns to face its target first, which would swing the arms round. Here the base keeps its heading and only drives
along it (a holonomic base may also go sideways). No obstacle avoidance: the mission says where this is safe, and
``max_distance_m`` bounds it.

Instead of a distance it can drive ``to`` a base target (a stance or location) that lies straight ahead or behind:
the robot-specific distance then lives with the stance (robots/<robot>/stances.yaml), not in the mission.

Parameters (``skills/drive_straight.yaml``), besides navigate_to's direct-controller ones (odometry, limits):
  speed: 0.15                 m/s
  max_distance_m: 1.0         longer moves are rejected
  to_offset_m: 0.05           with to: how far the target may lie off the straight line (sideways, differential base)
  to_turn_deg: 5.0            with to: how much its heading may differ (the heading is kept, not turned)
"""

from __future__ import annotations

import math
import time

from taskweave_core.config import wrap_angle
from taskweave_core.skill import SkillResult

from .navigate import MIN_DT_S, NavigateTo


class DriveStraight(NavigateTo):
    name = 'drive_straight'
    description = ('Drive the base straight forward (positive) or backward (negative) by a short distance, keeping its '
                   'heading, without obstacle avoidance: to move up to a rack or table the arms reach over, and back '
                   'out again. A holonomic base can also move sideways.')
    args_schema = {
        'type': 'object',
        'oneOf': [{'required': ['distance_m']}, {'required': ['to']}],
        'properties': {
            'distance_m': {'type': 'number', 'description': 'forward (+) or backward (-), metres'},
            'to': {'type': 'string', 'x-taskweave-point': 'base_target',
                   'description': 'a stance or location straight ahead or behind (instead of distance_m)'},
            'sideways_m': {'type': 'number', 'description': 'left (+) or right (-), metres; holonomic bases only'},
        },
        'additionalProperties': False,
    }
    needs = ['the strip the base drives over is free']
    achieves = ['robot base moved by the distance']
    default_timeout_s = 60.0
    param_defaults = {'speed': 0.15, 'max_distance_m': 1.0, 'to_offset_m': 0.05, 'to_turn_deg': 5.0}  # module doc

    def setup(self, node, config, robot, params):
        params = dict(params)
        own = {key: float(params.pop(key, default)) for key, default in self.param_defaults.items()}
        self.speed, self.max_distance, self.to_offset = own['speed'], own['max_distance_m'], own['to_offset_m']
        self.to_turn = math.radians(own['to_turn_deg'])
        super().setup(node, config, robot, params)

    def execute(self, ctx, args):
        if self.base['type'] == 'fixed':
            return SkillResult.rejected('robot base is fixed')
        if 'to' in args:
            moves = self._towards(args['to'])
            if isinstance(moves, SkillResult):
                return moves
            forward, sideways = moves
        else:
            forward, sideways = float(args['distance_m']), float(args.get('sideways_m', 0.0))
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
        clock = ctx.work_clock(self.p.override_slew_per_s)  # speed x override, stuck check on work time
        best, best_time, settled_since = math.inf, clock(ctx.now()), None
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
            dt, last = max(now - last, MIN_DT_S), now
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
            work = clock(now)
            if remaining < best - self.p.stuck_progress_m:
                best, best_time = remaining, work
            elif work - best_time > self.p.stuck_timeout_s:
                self.stop()
                return SkillResult.failed(f'no progress for {self.p.stuck_timeout_s:g} s, {remaining:.2f} m to go')
            limit = speed * clock.o if clock.o < 1.0 else speed
            vx = max(-limit, min(limit, self.p.k_linear * ex))
            vy = max(-limit, min(limit, self.p.k_linear * ey)) if sideways else 0.0
            step = self.p.linear_accel * dt
            vx = max(previous[0] - step, min(previous[0] + step, vx))
            vy = max(previous[1] - step, min(previous[1] + step, vy))
            previous = (vx, vy)
            self._publish(vx, vy, max(-self.p.max_angular, min(self.p.max_angular, self.p.k_angular * yaw_error)))
            ctx.feedback(1.0 - min(remaining / total, 1.0), f'{remaining:.2f} m to go')
            time.sleep(1.0 / self.p.rate_hz)

    def _towards(self, reference: str):
        """(forward, sideways) to the base target ``reference`` in the robot's frame, or a SkillResult if it does not lie
        straight ahead or behind with the same heading (sideways only for a holonomic base)."""
        pose = self._current()
        if pose is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        try:
            target = self.config.base_target(reference, self.robot.name, pose)
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        dx, dy = target.x - pose.x, target.y - pose.y
        forward = math.cos(pose.yaw) * dx + math.sin(pose.yaw) * dy
        sideways = -math.sin(pose.yaw) * dx + math.cos(pose.yaw) * dy
        turn = abs(wrap_angle(target.yaw - pose.yaw))
        if turn > self.to_turn:
            return SkillResult.rejected(f'{reference} faces {math.degrees(turn):.0f} deg away from the robot\'s heading: '
                                        'drive_straight keeps the heading (navigate_to there first)')
        if self.base['type'] != 'holonomic':
            if abs(sideways) > self.to_offset:
                return SkillResult.rejected(f'{reference} is {sideways:+.2f} m to the side of the straight line: '
                                            'navigate_to a spot in line with it first')
            sideways = 0.0
        return forward, sideways
