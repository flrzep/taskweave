"""navigate_to with Nav2: a collision-free path to the target, then the direct controller for the last centimetres.

Sends nav2_msgs/NavigateToPose to ``action`` with the target pose in ``frame``. That frame must be the frame the robot's
``base.odom`` pose is reported in (the cell frame; in Nav2 terms the map frame). Nav2 is good at getting there and
slow at settling, so as soon as the robot is within ``handover_distance`` of the target the Nav2 goal is canceled and
the direct controller of ``navigate.py`` drives the rest with its tolerances. If Nav2 gives up further away, the skill
fails with Nav2's reason.

Before the Nav2 goal: if the robot's footprint overlaps an obstacle on the map where it stands (at a pick stance the
envelope reaches into the bin the arms reach over), Nav2 would see it in collision and not move it at all. The robot
then first backs straight out until the footprint is clear (depart.py), after checking the strip behind it is free.

Parameters (``skills/navigate_to.yaml``): the direct controller's (see ``navigate.py``), plus
  action: navigate_to_pose      Nav2 NavigateToPose action
  frame: map                    frame of the goal pose
  server_timeout_s: 10.0        wait for the Nav2 action server
  handover_distance: 0.3        m; direct controller takes over within this distance
  map_topic: /map               occupancy grid the departure check uses (the map Nav2 plans on)
  depart_max_m: 1.0             back out at most this far (0: never)
  depart_margin_m: 0.1          beyond the point where the footprint comes clear
  depart_speed: 0.15            m/s backwards
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
                 'map_topic': '/map', 'depart_max_m': 1.0, 'depart_margin_m': 0.1, 'depart_speed': 0.15}
POLL_S = 0.05
NEUTRAL_PATH_ANGLE_WEIGHT = 2.0  # PathAngleCritic without a forward preference (follows the path, any heading)


def nav2_parameters(navigation: dict, base: dict, controller: str = 'FollowPath') -> dict:
    """Nav2 parameters for robots/<robot>/navigation.yaml and the robot profile's base (no ROS needed).

    Returns {'costmap': {...}, 'controller': {...}, 'motion_model': str}: costmap values for both costmaps, MPPI values
    for the controller plugin ``controller`` (prefixed), and the MPPI motion model (set at start-up only). A base that
    is not holonomic never drives sideways, whatever the settings say."""
    if navigation.get('envelope'):
        e = navigation['envelope']
        front, back, left, right = e['front'], e['back'], e['left'], e['right']
    else:  # the chassis footprint, centred on the base frame
        length, width = base['footprint']
        front, back, left, right = length / 2, length / 2, width / 2, width / 2
    polygon = [[front, left], [front, -right], [-back, -right], [-back, left]]
    costmap = {'footprint': str(polygon), 'footprint_padding': float(navigation.get('padding', 0.01))}
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
        values['PathAngleCritic.cost_weight'] = weight if weight > 0 else NEUTRAL_PATH_ANGLE_WEIGHT
        values['PreferForwardCritic.cost_weight'] = weight                 # penalises reversing
    return {'costmap': costmap, 'controller': {f'{controller}.{k}': v for k, v in values.items()},
            'motion_model': 'Omni' if holonomic else 'DiffDrive'}


class Nav2NavigateTo(NavigateTo):
    description = ('Drive the robot base to a named location or stance along a collision-free path (Nav2, avoids '
                   'mapped and seen obstacles) and stop there precisely, facing the stored heading.')
    needs = ['a free path to the target exists']

    def setup(self, node, config, robot, params):
        params = dict(params)
        self.nav = {key: type(default)(params.pop(key, default)) for key, default in NAV2_DEFAULTS.items()}
        super().setup(node, config, robot, params)
        from nav2_msgs.action import NavigateToPose  # only needed by this implementation
        from rclpy.action import ActionClient
        from rclpy.callback_groups import ReentrantCallbackGroup

        self.NavigateToPose = NavigateToPose
        self.client = ActionClient(node, NavigateToPose, self.nav['action'], callback_group=ReentrantCallbackGroup())
        self.goal_handle = None
        self.goal_lock = threading.Lock()
        self.remaining = math.nan
        self.map = None  # (grid, info) of the latest occupancy grid
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

    def _depart(self, ctx) -> SkillResult | None:
        """Back straight out if the footprint overlaps an obstacle where the robot stands (see the module doc)."""
        from .depart import departure, footprint_box

        pose, grid = self._current(), self.map
        if self.nav['depart_max_m'] <= 0 or grid is None or pose is None:
            return None
        box = footprint_box(self.config.navigation(self.robot.name), self.base)
        try:
            distance = departure(grid[0], grid[1], (pose.x, pose.y, pose.yaw), box, self.nav['depart_max_m'],
                                 self.nav['depart_margin_m'])
        except ValueError as error:
            return SkillResult.failed(f'cannot drive off: {error}')
        if distance <= 0:
            return None
        self.node.get_logger().info(f'navigate_to: footprint overlaps an obstacle, backing out {distance:.2f} m')
        start, deadline = pose, ctx.now() + 5.0 + 3.0 * distance / self.nav['depart_speed']
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
            if ctx.now() > deadline:
                self.stop()
                return SkillResult.failed(f'backing out: only {travelled:.2f} of {distance:.2f} m')
            ctx.feedback(0.0, f'backing out {distance - travelled:.2f} m')
            self._publish(-self.nav['depart_speed'], 0.0, 0.0)
            time.sleep(POLL_S)

    def stop(self) -> None:
        with self.goal_lock:
            handle, self.goal_handle = self.goal_handle, None
        if handle is not None:
            handle.cancel_goal_async()
        super().stop()

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
        if self.base['type'] == 'fixed':
            return SkillResult.rejected('robot base is fixed')
        target = self.config.base_target(args['target'], self.robot.name)
        if self._current() is None:
            return SkillResult.failed(f'no odometry on {self.base["odom"]}')
        if not self.client.wait_for_server(timeout_sec=self.nav['server_timeout_s']):
            return SkillResult.failed(f'Nav2 is not running (no {self.nav["action"]} action server)')
        departed = self._depart(ctx)
        if departed is not None:
            return departed
        self.remaining = math.nan
        sent = self.client.send_goal_async(self._goal(target), feedback_callback=self._feedback)
        if not self._wait(ctx, sent):
            self.stop()
            return ctx.interrupted()
        handle = sent.result()
        if not handle.accepted:
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
                ctx.feedback(0.9 * (1.0 - min(self.remaining / start, 1.0)), f'Nav2: {self.remaining:.2f} m to go')
            pose = self._current()
            if pose and math.hypot(target.x - pose.x, target.y - pose.y) <= self.nav['handover_distance']:
                self.stop()  # close enough: the direct controller is faster and more precise than Nav2 here
                return self._drive(ctx, target, args['target'])
            time.sleep(POLL_S)
        with self.goal_lock:
            self.goal_handle = None
        outcome = result.result()
        pose = self._current()
        distance = math.hypot(target.x - pose.x, target.y - pose.y) if pose else math.inf
        if distance > self.nav['handover_distance']:
            reason = getattr(outcome.result, 'error_msg', '') or f'status {outcome.status}'
            return SkillResult.failed(f'Nav2 did not reach {args["target"]}: {reason} ({distance:.2f} m away)')
        return self._drive(ctx, target, args['target'])

    def _feedback(self, message) -> None:
        self.remaining = float(message.feedback.distance_remaining)
