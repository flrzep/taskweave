"""navigate_to with Nav2: a collision-free path to the target, then the direct controller for the last centimetres.

Sends nav2_msgs/NavigateToPose to ``action`` with the target pose in ``frame``. That frame must be the frame the robot's
``base.odom`` pose is reported in (the cell frame; in Nav2 terms the map frame). Nav2 is good at getting there and
slow at settling, so as soon as the robot is within ``handover_distance`` of the target the Nav2 goal is canceled and
the direct controller of ``navigate.py`` drives the rest with its tolerances. If Nav2 gives up further away, the skill
fails with Nav2's reason.

Before the Nav2 goal: if the robot's footprint overlaps an obstacle on the map where it stands (at a pick stance the
envelope reaches into the bin the arms reach over), Nav2 would see it in collision and not move it at all. The robot
then first backs straight out until the footprint is clear (depart.py), after checking the strip behind it is free.

While the robot holds an object (hold.py) and navigation.yaml has a ``carry_envelope`` (the robot plus the carried
object), the costmaps get that footprint for the drive and the departure check uses it; otherwise the envelope. With
a carry
envelope the footprint is set on every goal, so a costmap restart (clock_guard) cannot leave the wrong one.

Parameters (``skills/navigate_to.yaml``): the direct controller's (see ``navigate.py``), plus
  action: navigate_to_pose      Nav2 NavigateToPose action
  frame: map                    frame of the goal pose
  server_timeout_s: 10.0        wait for the Nav2 action server
  handover_distance: 0.3        m; direct controller takes over within this distance
  map_topic: /map               occupancy grid the departure check uses (the map Nav2 plans on)
  depart_max_m: 1.0             back out at most this far (0: never)
  depart_margin_m: 0.1          beyond the point where the footprint comes clear
  depart_speed: 0.15            m/s backwards
  depart_slack_s: 5.0           backing out fails after this plus 3x the time it takes at depart_speed
  costmaps: /global_costmap/global_costmap,/local_costmap/local_costmap   nodes whose footprint follows the hold
  speed_limit_topic: /speed_limit   Nav2 controller_server's nav2_msgs/SpeedLimit topic
  speed_limit: true             true: the global speed override goes to Nav2: velocities as a SpeedLimit percentage
                                (100 %
                                = 'no limit') and, if nominal_accel is set, MPPI's accelerations at nominal x o^2 (time
                                scaling by 1/o: a velocity cap alone does nothing to drives that stay below it). False:
                                only 'no limit' and the nominal accelerations are sent, once. Followed during a drive
                                whenever the override changes (0.1 % steps), restored when the Nav2 phase ends
  controller_node: /controller_server   node whose parameters hold the accelerations (SetParameters service)
  controller_param_prefix: FollowPath   controller plugin prefix of those parameters
  nominal_accel: {}             nominal accelerations of the controller, e.g. {ax_max: 2.0, ax_min: -2.0, az_max: 2.0}
                                (m/s^2, rad/s^2, same numbers as the Nav2 params). Not read back from the controller (it
                                may still hold scaled values after a restart). Empty: accelerations are not touched
  min_accel_scale: 0.01         floor of o^2 on the accelerations (never zero)
  min_override: 0.3             floor of the speed override for the Nav2 phase only (0 = no floor): the SpeedLimit,
                                the accelerations (nominal x o^2) and the nominal_time allowances all use
                                max(o, min_override). MPPI cannot do the fine final approach into a stance at 10 %
                                (it stalled short of the goal); test mode at 10 % = arm motion 10 %, Nav2 drives
                                30 %. Arms, torso, grippers, drive_straight and the direct controller after the
                                handover follow the real override. The effective value is reported in the step
                                result as data.speed.nav2_override (percent)
  nominal_time: {}              time allowances on the controller node (full names, no prefix), scaled by 1/o with
                                the accelerations, e.g. {progress_checker.movement_time_allowance: 20.0}: at 10 % a
                                drive takes 10x as long, so its stuck check waits 10x as long (unchanged at 100 %)
"""

from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace

from taskweave_core.config import Pose2D
from taskweave_core.skill import SkillResult

from .navigate import NavigateTo

NAV2_DEFAULTS = {'action': 'navigate_to_pose', 'frame': 'map', 'server_timeout_s': 10.0, 'handover_distance': 0.3,
                 'map_topic': '/map', 'depart_max_m': 1.0, 'depart_margin_m': 0.1, 'depart_speed': 0.15,
                 'costmaps': '/global_costmap/global_costmap,/local_costmap/local_costmap',
                 'speed_limit_topic': '/speed_limit', 'speed_limit': True,
                 'controller_node': '/controller_server', 'controller_param_prefix': 'FollowPath',
                 'nominal_accel': {}, 'min_accel_scale': 0.01, 'nominal_time': {}, 'min_override': 0.3}
POLL_S = 0.05


def nav2_override(nav: dict, override: float) -> float:
    """The override Nav2 is driven at (a fraction): ``max(override, min_override)``, and 1 when the speed limit is off
    (Nav2 is then not slowed at all)."""
    if not nav.get('speed_limit'):
        return 1.0
    return max(override, nav.get('min_override', 0.0))


def nav2_parameters(navigation: dict, base: dict, controller: str = 'FollowPath', carrying: bool = False) -> dict:
    """Nav2 parameters for robots/<robot>/navigation.yaml and the robot profile's base (no ROS needed).

    Returns {'costmap': {...}, 'controller': {...}, 'motion_model': str}: costmap values for both costmaps, MPPI values
    for the controller plugin ``controller`` (prefixed), and the MPPI motion model (set at start-up only). A base that
    is not holonomic never drives sideways, whatever the settings say. ``carrying``: the footprint of the carry
    envelope, if there is one."""
    from .depart import envelope, padding

    front, back, left, right = envelope(navigation, base, carrying)
    polygon = [[front, left], [front, -right], [-back, -right], [-back, left]]
    costmap = {'footprint': str(polygon), 'footprint_padding': padding(navigation)}
    if 'inflation_radius' in navigation:
        costmap['inflation_layer.inflation_radius'] = float(navigation['inflation_radius'])
    holonomic = base.get('type') == 'holonomic'
    driving = navigation.get('driving', {})
    values = {}
    if 'max_forward' in driving:
        values['vx_max'] = float(driving['max_forward'])
    if 'max_reverse' in driving:
        values['vx_min'] = -float(driving['max_reverse'])
    if 'max_turn' in driving:
        values['wz_max'] = float(driving['max_turn'])
    if 'sideways' in driving or not holonomic:
        sideways = holonomic and driving.get('sideways', True)
        values['vy_max'] = float(driving.get('max_sideways', 0.5)) if sideways else 0.0
    if 'prefer_forward' in driving:
        weight = float(driving['prefer_forward'])
        values['PathAngleCritic.mode'] = 0 if weight > 0 else 2           # 0: face along the path, 2: any heading
        values['PathAngleCritic.cost_weight'] = weight if weight > 0 else any_heading_weight(driving)
        values['PreferForwardCritic.cost_weight'] = weight                 # penalises reversing
    return {'costmap': costmap, 'controller': {f'{controller}.{k}': v for k, v in values.items()},
            'motion_model': 'Omni' if holonomic else 'DiffDrive'}


def any_heading_weight(driving: dict) -> float:
    """PathAngleCritic weight without a forward preference (follows the path, any heading): navigation.yaml
    driving.any_heading_weight, else its schema default."""
    from taskweave_core.config.schemas import NAVIGATION, default

    return float(driving.get('any_heading_weight', default(NAVIGATION, 'driving', 'any_heading_weight')))


class Nav2NavigateTo(NavigateTo):
    description = ('Drive the robot base to a named location or stance along a collision-free path (Nav2, avoids '
                   'mapped and seen obstacles) and stop there precisely, facing the stored heading.')
    needs = ['a free path to the target exists']
    param_defaults = {**NAV2_DEFAULTS, 'depart_slack_s': 5.0}  # see the module doc
    NAV2_SHARE = 0.9  # of the step's progress feedback reported while Nav2 drives (the direct controller the rest)

    def setup(self, node, config, robot, params):
        params = dict(params)
        self.nav = {key: type(default)(params.pop(key, default)) for key, default in self.param_defaults.items()}
        super().setup(node, config, robot, params)
        from nav2_msgs.action import NavigateToPose  # only needed by this implementation
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self.NavigateToPose = NavigateToPose
        from nav2_msgs.msg import SpeedLimit

        self.SpeedLimit, self._speed_sent, self._accel_sent, self._accel_warned = SpeedLimit, None, None, False
        self.accel_client = None
        # None: unknown (a restart does not know Nav2's limit)
        self.speed_limit = node.create_publisher(SpeedLimit, self.nav['speed_limit_topic'], 10)
        self.speed_limit.publish(SpeedLimit(percentage=True, speed_limit=0.0))  # a previous server may have died during
        # a slowed drive. Not recorded as sent: discovery may not have reached Nav2 yet, the first drive sends it again
        self.client = ActionClient(node, NavigateToPose, self.nav['action'], callback_group=ReentrantCallbackGroup())
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.remaining = math.nan
        self.map = None  # (grid, info) of the latest occupancy grid
        from rcl_interfaces.srv import SetParameters

        from .hold import HoldState
        self.hold = HoldState.for_node(node, robot)
        self.SetParameters = SetParameters
        if self.nav['nominal_accel'] or self.nav['nominal_time']:
            self.accel_client = node.create_client(SetParameters, f'{self.nav["controller_node"]}/set_parameters',
                                                   callback_group=ReentrantCallbackGroup())
        self.costmaps = {name: node.create_client(SetParameters, f'{name}/set_parameters',
                                                  callback_group=ReentrantCallbackGroup())
                         for name in filter(None, (n.strip() for n in self.nav['costmaps'].split(',')))}
        if self.nav['depart_max_m'] > 0:
            from nav_msgs.msg import OccupancyGrid
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(OccupancyGrid, self.nav['map_topic'], self._map, latched)

    def _map(self, msg) -> None:
        import numpy as np

        info = SimpleNamespace(resolution=msg.info.resolution, origin_x=msg.info.origin.position.x,
                               origin_y=msg.info.origin.position.y)
        self.map = (np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width), info)

    def _carrying(self) -> bool:
        return self.hold.held() is not None and bool(self.config.navigation(self.robot.name).get('carry_envelope'))

    def _footprint(self, ctx, carrying: bool) -> SkillResult | None:
        """Set the costmaps' footprint for this drive (carry envelope or envelope); a failure if a costmap refuses.
        Without a carry envelope the footprint never changes from the one Nav2 was started with."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

        navigation = self.config.navigation(self.robot.name)
        if not navigation.get('carry_envelope'):
            return None
        footprint = nav2_parameters(navigation, self.base, carrying=carrying)['costmap']['footprint']
        request = self.SetParameters.Request(parameters=[Parameter(name='footprint', value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=footprint))])
        for name, client in self.costmaps.items():
            if not client.wait_for_service(timeout_sec=self.nav['server_timeout_s']):
                return SkillResult.failed(f'cannot set the footprint: {name} is not running')
            future = client.call_async(request)
            if not self._wait(ctx, future):
                return ctx.interrupted()
            failed = [r.reason for r in future.result().results if not r.successful]
            if failed:
                return SkillResult.failed(f'{name} refused the footprint: {"; ".join(failed)}')
        return None

    def _depart(self, ctx, carrying: bool = False) -> SkillResult | None:
        """Back straight out if the footprint overlaps an obstacle where the robot stands (see the module doc)."""
        from .depart import departure, footprint_box

        pose, grid = self._current(), self.map
        if self.nav['depart_max_m'] <= 0 or grid is None or pose is None:
            return None
        box = footprint_box(self.config.navigation(self.robot.name), self.base, carrying)
        try:
            distance = departure(grid[0], grid[1], (pose.x, pose.y, pose.yaw), box, self.nav['depart_max_m'],
                                 self.nav['depart_margin_m'])
        except ValueError as error:
            return SkillResult.failed(f'cannot drive off: {error}')
        if distance <= 0:
            return None
        self.node.get_logger().info(f'navigate_to: footprint overlaps an obstacle, backing out {distance:.2f} m')
        clock = ctx.work_clock(self.p.override_slew_per_s)  # speed x override, deadline on work time
        slack = self.nav['depart_slack_s'] + 3.0 * distance / self.nav['depart_speed']
        start, deadline = pose, clock(ctx.now()) + slack
        while True:
            if ctx.should_stop:
                self.stop()
                return ctx.interrupted()
            pose = self._current()
            if pose is None:
                self.stop()
                return SkillResult.failed(f'odometry lost on {self.base["odom"]}')
            travelled = math.hypot(pose.x - start.x, pose.y - start.y)
            if travelled >= distance:
                self.stop()
                return None
            if clock(ctx.now()) > deadline:
                self.stop()
                return SkillResult.failed(f'backing out: only {travelled:.2f} of {distance:.2f} m')
            ctx.feedback(0.0, f'backing out {distance - travelled:.2f} m')
            self._publish(-self.nav['depart_speed'] * (clock.o if clock.o < 1.0 else 1.0), 0.0, 0.0)
            time.sleep(POLL_S)

    def stop(self) -> None:
        with self.goal_lock:
            handle, self.goal_handle = self.goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        self._limit_speed(1.0)
        super().stop()

    def _limit_speed(self, override: float) -> None:
        """Nav2's controller at the speed override: SpeedLimit as a percentage of its own limits (0 = no limit) and
        the accelerations at nominal x o^2. Sent when it changes (so a mid-drive override change is followed); the
        first call after start always sends (what Nav2 holds is unknown, e.g. after a restart during a slowed
        drive), at 100 % as 'no limit' and the nominal accelerations. With speed_limit off only those are sent. The
        override is floored at min_override (see the module doc)."""
        override = nav2_override(self.nav, override)
        percent = min(100.0, round(100.0 * override, 1))
        self._scale_accel(percent)
        if percent == self._speed_sent:
            return
        self.speed_limit.publish(self.SpeedLimit(percentage=True, speed_limit=percent if percent < 100.0 else 0.0))
        self._speed_sent = percent

    def _scale_accel(self, percent: float) -> None:
        """MPPI's accelerations at nominal x o^2 through the controller server's parameter service (not configured:
        untouched), and the time allowances of nominal_time at nominal / o. An unavailable service is a warning
        (once per outage), not a failure; the next call retries."""
        nominal = self.nav.get('nominal_accel') or {}
        times = self.nav.get('nominal_time') or {}
        if not (nominal or times) or self.accel_client is None or percent == self._accel_sent:
            return
        if not self.accel_client.service_is_ready():
            if not self._accel_warned:
                self.node.get_logger().warning(f'navigate_to: {self.nav["controller_node"]} parameter service not '
                                               'available, Nav2 accelerations not scaled')
                self._accel_warned = True
            return
        self._accel_warned = False
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

        factor = max((percent / 100.0) ** 2, self.nav['min_accel_scale'])
        prefix = self.nav['controller_param_prefix']
        request = self.SetParameters.Request()
        request.parameters = [Parameter(name=f'{prefix}.{key}', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value) * factor)) for key, value in nominal.items()]
        stretch = 1.0 / max(percent / 100.0, self.nav['min_accel_scale'])   # time scales by 1/o
        request.parameters += [Parameter(name=key, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(value) * stretch)) for key, value in times.items()]
        future = self.accel_client.call_async(request)
        self._accel_sent = percent
        future.add_done_callback(self._accel_done)

    def _accel_done(self, future) -> None:
        try:
            failed = [r.reason for r in future.result().results if not r.successful]
        except Exception as error:  # noqa: BLE001
            failed = [str(error)]
        if failed:
            self._accel_sent = None  # retry at the next change / drive
            self.node.get_logger().warning(f'navigate_to: Nav2 accelerations not set: {"; ".join(failed)}')

    def _goal(self, target: Pose2D):
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = self.nav['frame']
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target.x, target.y
        goal.pose.pose.orientation.z = math.sin(target.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(target.yaw / 2)
        return goal

    def _wait(self, ctx, future) -> bool:
        """Poll a future; False if the step was canceled or timed out first."""
        while not future.done():
            if ctx.should_stop:
                return False
            time.sleep(POLL_S)
        return True

    def execute(self, ctx, args):
        self._nav2_min = None
        result = self._navigate(ctx, args)
        if self._nav2_min is not None:   # the Nav2 phase ran: say at what override
            speed = result.data.setdefault('speed', {})
            speed['nav2_override'] = round(100 * self._nav2_min, 1)
        return result

    def _nav2_limit(self, override: float) -> None:
        """Nav2 follows the (floored) override; the lowest value of the phase goes into the step result."""
        self._limit_speed(override)
        effective = nav2_override(self.nav, override)
        self._nav2_min = effective if self._nav2_min is None else min(self._nav2_min, effective)

    def _navigate(self, ctx, args):
        if self.base['type'] == 'fixed':
            return SkillResult.rejected('robot base is fixed')
        target = self.config.base_target(args['target'], self.robot.name, self._current())
        if self._current() is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        if not self.client.wait_for_server(timeout_sec=self.nav['server_timeout_s']):
            return SkillResult.failed(f'Nav2 is not running (no {self.nav["action"]} action server)')
        carrying = self._carrying()
        refused = self._footprint(ctx, carrying)
        if refused is not None:
            return refused
        if carrying:
            self.node.get_logger().info('navigate_to: carrying an object, using the carry envelope')
        departed = self._depart(ctx, carrying)
        if departed is not None:
            return departed
        self.remaining = math.nan
        clock = ctx.work_clock(self.p.override_slew_per_s)
        clock(ctx.now())
        self._nav2_limit(clock.o)
        sent = self.client.send_goal_async(self._goal(target), feedback_callback=self._feedback)
        if not self._wait(ctx, sent):
            self.stop()
            return ctx.interrupted()
        handle = sent.result()
        if not handle.accepted:
            self._limit_speed(1.0)
            return SkillResult.failed('Nav2 rejected the goal')
        with self.goal_lock:
            self.goal_handle = handle
        result = handle.get_result_async()
        start = None
        while not result.done():
            if ctx.should_stop:
                self.stop()
                return ctx.interrupted()
            if not math.isnan(self.remaining):
                start = start or max(self.remaining, 1e-6)
                ctx.feedback(self.NAV2_SHARE * (1.0 - min(self.remaining / start, 1.0)),
                             f'Nav2: {self.remaining:.2f} m to go')
            pose = self._current()
            if pose and math.hypot(target.x - pose.x, target.y - pose.y) <= self.nav['handover_distance']:
                self.stop()  # close enough: the direct controller is faster and more precise than Nav2 here
                return self._drive(ctx, target, args['target'])
            clock(ctx.now())
            self._nav2_limit(clock.o)
            time.sleep(POLL_S)
        with self.goal_lock:
            self.goal_handle = None
        self._limit_speed(1.0)
        outcome = result.result()
        pose = self._current()
        distance = math.hypot(target.x - pose.x, target.y - pose.y) if pose else math.inf
        if distance > self.nav['handover_distance']:
            reason = getattr(outcome.result, 'error_msg', '') or f'status {outcome.status}'
            return SkillResult.failed(f'Nav2 did not reach {args["target"]}: {reason} ({distance:.2f} m away)')
        return self._drive(ctx, target, args['target'])

    def _feedback(self, message) -> None:
        self.remaining = float(message.feedback.distance_remaining)
