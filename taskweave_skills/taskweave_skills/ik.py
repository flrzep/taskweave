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
FRAME_TRANSFORM_FAILURE = -21
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


def target_warnings(target: dict) -> list[str]:
    """Likely mistakes in a tool target that IK alone reports only as 'no solution'."""
    warnings = []
    tcp, approach = np.asarray(target['tcp'], float), np.asarray(target['approach'], float)
    if np.linalg.norm(tcp[:2]) > 0.2 and approach[:2] @ tcp[:2] < -0.3 * np.linalg.norm(approach[:2]) * np.linalg.norm(
            tcp[:2]):
        warnings.append('approach points back toward the robot: it is the direction the gripper moves into the object '
                        '(forward = [1, 0, 0]); the wrist sits behind the TCP along it')
    return warnings


def shifts(step: float, max_shift: float):
    """TCP offsets for the reach search, smallest first: along each base axis, both ways."""
    for k in range(1, int(round(max_shift / step)) + 1):
        for axis in range(3):
            for sign in (1, -1):
                offset = [0.0, 0.0, 0.0]
                offset[axis] = sign * k * step
                yield offset


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
                if response.error_code.val == FRAME_TRANSFORM_FAILURE:
                    raise IkError(f'IK for {tool}: MoveIt has no transform to {self.frame} (TF not published?)')
                flange = ', '.join(f'{v:.2f}' for v in pose[:3, 3])
                raise IkError(f'no IK solution for {tool}: TCP {list(target["tcp"])}, approach '
                              f'{list(target["approach"])} needs the flange at ({flange}) in {self.frame}: out of '
                              f'reach, too close to the body, or in collision (code {response.error_code.val})')
            pattern = re.compile(self.joint_pattern.format(tool=tool))
            state = response.solution.joint_state
            result.update({name: float(value) for name, value in zip(state.name, state.position, strict=False)
                           if pattern.fullmatch(name)})
        return result

    def step(self, tool: str, flange: np.ndarray, current: dict, timeout_s: float = 0.05) -> dict[str, float] | None:
        """Arm joints that put the tool's flange on ``flange`` (4x4, base frame), by one local descent from the
        ``current`` joints (a small jog step), or None. Not repeatable like solve(): it follows the arm."""
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        request = self._request(tool, self.groups[tool], flange, timeout_s)
        request.ik_request.robot_state = RobotState(
            joint_state=JointState(name=list(current), position=[float(v) for v in current.values()]), is_diff=True)
        response = self._call(request, tool, timeout_s, lambda: False)
        if response.error_code.val != SUCCESS:
            return None
        pattern = re.compile(self.joint_pattern.format(tool=tool))
        state = response.solution.joint_state
        return {name: float(value) for name, value in zip(state.name, state.position, strict=False)
                if pattern.fullmatch(name)}

    def reachable(self, tool: str, target: dict, fixed: dict, timeout_s: float = 0.15) -> bool:
        """One quick IK attempt (the fallback group's global search if there is one): a probe, not a solution."""
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        request = self._request(tool, self.fallback_groups.get(tool, self.groups[tool]),
                                flange_target(target, self.tools[tool]), timeout_s)
        request.ik_request.robot_state = RobotState(
            joint_state=JointState(name=list(fixed), position=[float(v) for v in fixed.values()]), is_diff=True)
        return self._call(request, tool, timeout_s, lambda: False).error_code.val == SUCCESS

    def nearest_reachable(self, tool: str, target: dict, fixed: dict, step: float = 0.05,
                          max_shift: float = 0.4) -> dict | None:
        """The target with its TCP moved the least along a base axis (same orientation) that IK solves, or None."""
        for offset in shifts(step, max_shift):
            candidate = {**target, 'tcp': [round(float(v) + d, 4) for v, d in zip(target['tcp'], offset, strict=True)]}
            if self.reachable(tool, candidate, fixed):
                try:
                    self.solve({tool: candidate}, fixed)  # confirm the way a move would solve it
                except IkError:
                    continue
                return candidate
        return None

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
