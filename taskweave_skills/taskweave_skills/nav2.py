"""navigate_to with Nav2: a collision-free path to the target, then the direct controller for the last centimetres.

Sends nav2_msgs/NavigateToPose to ``action`` with the target pose in ``frame``. That frame must be the frame the robot's
``base.odom`` pose is reported in (the cell frame; in Nav2 terms the map frame). Nav2 is good at getting there and
slow at settling, so as soon as the robot is within ``handover_distance`` of the target the Nav2 goal is canceled and
the direct controller of ``navigate.py`` drives the rest with its tolerances. If Nav2 gives up further away, the skill
fails with Nav2's reason.

Parameters (``skills/navigate_to.yaml``): the direct controller's (see ``navigate.py``), plus
  action: navigate_to_pose      Nav2 NavigateToPose action
  frame: map                    frame of the goal pose
  server_timeout_s: 10.0        wait for the Nav2 action server
  handover_distance: 0.3        m; direct controller takes over within this distance
"""

from __future__ import annotations

import math
import threading
import time

from taskweave_core.config import Pose2D
from taskweave_core.skill import SkillResult

from .navigate import NavigateTo

NAV2_DEFAULTS = {'action': 'navigate_to_pose', 'frame': 'map', 'server_timeout_s': 10.0, 'handover_distance': 0.3}
POLL_S = 0.05


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
