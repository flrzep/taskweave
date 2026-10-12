"""Tool targets -> joint values through a GetPositionIK service (MoveIt's /compute_ik).

A tool target is the canonical TCP (grasp centre, z = approach, y = closing) in the robot's base frame:
``{tcp: [x, y, z], approach: [...], closing: [...]}``. The profile's ``tool_frames`` give the TCP relative to the
flange link, so the IK target for the flange is TCP * (flange -> TCP)^-1. Profile section ``ik``:

    ik:
      service: /compute_ik            # default
      timeout_s: 1.0                  # per attempt (each seed, then the current state)
      fallback_timeout_s: 2.0         # for the fallback group's global search (default: timeout_s)
      groups: {left: left_arm, right: right_arm}    # planning group per tool
      joints: '{tool}_joint[1-7]'     # which solved joints to command (regex, {tool} = tool name)
      seeds:                          # optional known-good postures, tried in order
        - {left_joint1: -1.55, ...}
      fallback_groups: {left: left_arm_global, right: right_arm_global}   # optional, global solver
      avoid_collisions: true          # optional: a solution in collision counts as none (next start is tried)

A redundant arm (7 joints) reaches a TCP pose in a whole family of configurations, and which one a solver returns
depends on where it starts. IK starts from each seed posture in turn, then from the current state; the pose's fixed
joints (e.g. the torso lift) are applied on top, so the arms are solved for the torso height the pose sets. With a
deterministic local solver (pick_ik mode local: gradient descent from the start), a named pose then always solves
to the same configuration, whatever pose the robot comes from; only targets no seed reaches depend on the current
state. If nothing converges, the fallback group (a global solver) searches from the current state.
"""

from __future__ import annotations

import math
import re
import time

import numpy as np

SUCCESS = 1
POLL_S = 0.02


class IkError(RuntimeError):
    """No joint solution for a tool target; the message names the tool and target."""


def tcp_matrix(target: dict) -> np.ndarray:
    """4x4 pose of the canonical TCP: z = approach, y = closing (made orthogonal to z), x = y cross z."""
    z = np.asarray(target['approach'], float)
    z /= np.linalg.norm(z)
    y = np.asarray(target['closing'], float)
    y = y - (y @ z) * z
    if np.linalg.norm(y) < 1e-9:
        raise IkError(f'closing direction {target["closing"]} is parallel to approach {target["approach"]}')
    y /= np.linalg.norm(y)
    pose = np.eye(4)
    pose[:3, :3] = np.column_stack([np.cross(y, z), y, z])
    pose[:3, 3] = target['tcp']
    return pose


def rpy_matrix(xyz, rpy) -> np.ndarray:
    (cr, cp, cy), (sr, sp, sy) = np.cos(rpy), np.sin(rpy)
    pose = np.eye(4)
    pose[:3, :3] = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                             [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                             [-sp, cp * sr, cp * cr]])
    pose[:3, 3] = xyz
    return pose


def pose_matrix(position, orientation) -> np.ndarray:
    """4x4 from geometry_msgs Point and Quaternion."""
    w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
    pose = np.eye(4)
    pose[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    pose[:3, 3] = position.x, position.y, position.z
    return pose


def tool_target(flange: np.ndarray, tool_frame: dict) -> dict:
    """The tool target (TCP, approach, closing) for a flange pose: the inverse of flange_target."""
    tcp = flange @ rpy_matrix(tool_frame.get('xyz', [0, 0, 0]), tool_frame.get('rpy', [0, 0, 0]))
    return {'tcp': [round(float(v), 4) for v in tcp[:3, 3]], 'approach': [round(float(v), 4) for v in tcp[:3, 2]],
            'closing': [round(float(v), 4) for v in tcp[:3, 1]]}


def flange_target(target: dict, tool_frame: dict) -> np.ndarray:
    """Pose of the tool's flange link that puts the TCP on ``target``."""
    offset = rpy_matrix(tool_frame.get('xyz', [0, 0, 0]), tool_frame.get('rpy', [0, 0, 0]))  # flange -> TCP
    return tcp_matrix(target) @ np.linalg.inv(offset)


def quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """(w, x, y, z) of a rotation matrix (stable for every rotation)."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2 * math.sqrt(1 + trace)
        return s / 4, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2 * math.sqrt(1 + m[i, i] - m[j, j] - m[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = s / 4
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return (m[k, j] - m[j, k]) / s, *q


class IkClient:
    def __init__(self, node, robot):
        from moveit_msgs.srv import GetPositionIK  # only needed when the robot has an ik section
        from rclpy.callback_groups import ReentrantCallbackGroup

        spec = robot.data['ik']
        self.groups, self.tools = spec['groups'], robot.data.get('tool_frames', {})
        self.timeout_s = float(spec.get('timeout_s', 1.0))
        self.fallback_timeout_s = float(spec.get('fallback_timeout_s', self.timeout_s))
        self.joint_pattern = spec['joints']
        self.frame = robot.base['frame']
        self.service = spec.get('service', '/compute_ik')
        self.fallback_groups = spec.get('fallback_groups', {})
        self.avoid_collisions = bool(spec.get('avoid_collisions', False))
        self.seeds = [{name: float(value) for name, value in seed.items()} for seed in spec.get('seeds', [])]
        self.Request = GetPositionIK.Request
        self.client = node.create_client(GetPositionIK, self.service, callback_group=ReentrantCallbackGroup())

    def solve(self, tools: dict, fixed: dict, should_stop=lambda: False) -> dict[str, float]:
        """Joint values for every tool target (plus nothing else); raises IkError."""
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        if not self.client.wait_for_service(timeout_sec=2.0):
            raise IkError(f'no IK service {self.service} (is MoveIt running?)')
        result: dict[str, float] = {}
        for tool, target in tools.items():
            if tool not in self.groups or tool not in self.tools:
                raise IkError(f'tool {tool!r} has no IK group or tool frame in the robot profile')
            pose = flange_target(target, self.tools[tool])
            attempts = [(self.groups[tool], seed, self.timeout_s) for seed in [*self.seeds, {}]]  # {}: current state
            if tool in self.fallback_groups:
                attempts.append((self.fallback_groups[tool], {}, self.fallback_timeout_s))
            for group, seed, timeout_s in attempts:
                request = self._request(tool, group, pose, timeout_s)
                start = {**seed, **fixed, **result}
                request.ik_request.robot_state = RobotState(
                    joint_state=JointState(name=list(start), position=list(start.values())), is_diff=True)
                response = self._call(request, tool, timeout_s, should_stop)
                if response.error_code.val == SUCCESS:
                    break
            else:
                raise IkError(f'no IK solution for {tool}: TCP {list(target["tcp"])}, '
                              f'approach {list(target["approach"])} (code {response.error_code.val})')
            pattern = re.compile(self.joint_pattern.format(tool=tool))
            state = response.solution.joint_state
            result.update({name: float(value) for name, value in zip(state.name, state.position, strict=False)
                           if pattern.fullmatch(name)})
        return result

    def _request(self, tool: str, group: str, pose: np.ndarray, timeout_s: float):
        request = self.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name, ik.avoid_collisions = group, self.tools[tool]['link'], self.avoid_collisions
        ik.pose_stamped.header.frame_id = self.frame
        position = ik.pose_stamped.pose.position
        position.x, position.y, position.z = (float(v) for v in pose[:3, 3])
        w, x, y, z = quaternion(pose[:3, :3])
        orientation = ik.pose_stamped.pose.orientation
        orientation.w, orientation.x, orientation.y, orientation.z = w, x, y, z
        ik.timeout.sec, ik.timeout.nanosec = int(timeout_s), int((timeout_s % 1) * 1e9)
        return request

    def _call(self, request, tool: str, timeout_s: float, should_stop):
        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout_s + 5.0
        while not future.done():
            if should_stop() or time.monotonic() > deadline:
                raise IkError(f'IK for {tool} interrupted' if should_stop() else f'IK service timed out for {tool}')
            time.sleep(POLL_S)
        return future.result()
