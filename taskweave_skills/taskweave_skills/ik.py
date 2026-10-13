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
      mirror: {right: left}           # optional: the right arm mirrors the left with the same joint values
                                      # (names with the tool prefix swapped); a mirrored target then solves the same
      choose:                         # optional: pick the best of several solutions instead of the first
        elbow: down                   # preferred elbow side: down | up | out | in | [x, y, z] (y = away from the body)
        elbow_links: ['{tool}_link2', '{tool}_link4', '{tool}_link6']   # shoulder, elbow, wrist centres
        limit_margin_deg: 25          # joints closer than this to a limit cost (more the closer)
        weights: {limits: 2.0, elbow: 1.0, motion: 0.2, symmetry: 0.5}
        starts: 0                     # extra start postures (fixed pseudo-random, so repeatable) for more candidates
        start_timeout_s: 0.05         # per extra start

A redundant arm (7 joints) reaches a TCP pose in a whole family of configurations, and which one a solver returns
depends on where it starts. IK starts from each seed posture in turn, then from the current state; the pose's fixed
joints (e.g. the torso lift) are applied on top, so the arms are solved for the torso height the pose sets. With a
deterministic local solver (pick_ik mode local: gradient descent from the start), a named pose then always solves
to the same configuration, whatever pose the robot comes from; only targets no seed reaches depend on the current
state. If nothing converges, the fallback group (a global solver) searches from the current state.

With ``choose``, every start that converges is a candidate, for the target and, if the tool frame says
``flip: true`` (a grasp that is the same turned 180 degrees about the approach), for the flipped target too. The
candidate with the lowest cost wins: joints near their limits (an arm at a limit cannot move on, e.g. lift a held
object), the elbow away from the preferred side (the swivel of the elbow about the shoulder-wrist line, measured with
/compute_fk; "out" and y are mirrored for an arm on the right, so symmetric targets give mirrored arms), and the size
of the move from the current joints. With ``mirror``, an arm is solved after the arm it mirrors, starts from the mirror
image of that arm's solution and prefers it (``symmetry``): symmetric targets give mirrored arms. Industrial
controllers store such a configuration with every taught point; here the profile gives the default.
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


ELBOW_WORDS = {'down': (0.0, 0.0, -1.0), 'up': (0.0, 0.0, 1.0), 'out': (0.0, 1.0, 0.0), 'in': (0.0, -1.0, 0.0),
               'back': (-1.0, 0.0, 0.0), 'forward': (1.0, 0.0, 0.0)}


def flipped(target: dict) -> dict:
    """The same grasp turned 180 degrees about the approach (closing reversed)."""
    return {**target, 'closing': [-float(v) for v in target['closing']]}


def preferred_elbow(spec, shoulder) -> np.ndarray:
    """Preferred elbow direction in the base frame. y means away from the body: mirrored for a shoulder at -y."""
    vector = np.asarray(ELBOW_WORDS[spec] if isinstance(spec, str) else spec, float)
    if shoulder[1] < 0:
        vector = vector * [1.0, -1.0, 1.0]
    return vector / np.linalg.norm(vector)


def elbow_deviation(shoulder, elbow, wrist, preferred) -> float:
    """Angle (rad, 0..pi) between where the elbow points, seen along the shoulder-wrist line, and ``preferred``."""
    shoulder, elbow, wrist = (np.asarray(v, float) for v in (shoulder, elbow, wrist))
    axis = wrist - shoulder
    axis /= np.linalg.norm(axis)
    side = (elbow - shoulder) - ((elbow - shoulder) @ axis) * axis
    want = preferred - (preferred @ axis) * axis
    if np.linalg.norm(side) < 1e-6 or np.linalg.norm(want) < 1e-6:
        return 0.0  # arm stretched, or the preference lies along the arm: no side to prefer
    cos = side @ want / np.linalg.norm(side) / np.linalg.norm(want)
    return float(math.acos(max(-1.0, min(1.0, cos))))


def limit_cost(values: dict, limits: dict, margin: float) -> float:
    """0 while every joint is at least ``margin`` from its limits, growing quadratically to 1 per joint at a limit."""
    cost = 0.0
    for name, value in values.items():
        if name in limits and margin > 0:
            lower, upper = limits[name]
            room = min(value - lower, upper - value)
            cost += max(0.0, (margin - room) / margin) ** 2
    return cost


class IkClient:
    def __init__(self, node, robot, limits: dict | None = None):
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
        self.choose = spec.get('choose')  # None: the first solution wins (seed order)
        self.mirror = spec.get('mirror', {})  # tool -> the tool whose joints it mirrors
        self.limits = limits or {}
        self.last: dict[str, dict] = {}   # per tool: how the last solve() chose (for messages)
        if self.choose is not None:
            from moveit_msgs.srv import GetPositionFK

            self.fk = node.create_client(GetPositionFK, self.choose.get('fk_service', '/compute_fk'),
                                         callback_group=ReentrantCallbackGroup())

    def solve(self, tools: dict, fixed: dict, should_stop=lambda: False, current: dict | None = None
              ) -> dict[str, float]:
        """Joint values for every tool target (plus nothing else); raises IkError. ``current``: the joints now, for
        the size-of-move cost when choosing."""
        if not self.client.wait_for_service(timeout_sec=2.0):
            raise IkError(f'no IK service {self.service} (is MoveIt running?)')
        result: dict[str, float] = {}
        self.last = {}
        chosen: dict[str, dict] = {}  # per tool: its arm joints, for the arm that mirrors it
        order = sorted(tools, key=lambda t: self.mirror.get(t) in tools)  # mirror sources first
        for tool in order:
            target = tools[tool]
            if tool not in self.groups or tool not in self.tools:
                raise IkError(f'tool {tool!r} has no IK group or tool frame in the robot profile')
            flip = self.choose is not None and self.tools[tool].get('flip')
            variants = [target, flipped(target)] if flip else [target]
            local = [(self.groups[tool], seed, self.timeout_s) for seed in [*self.seeds, {}]]  # {}: current state
            image = self._mirrored(tool, chosen)
            if image:
                local.insert(0, (self.groups[tool], image, self.timeout_s))
            if self.choose is not None:
                local += [(self.groups[tool], start, float(self.choose.get('start_timeout_s', 0.05)))
                          for start in self._starts(tool)]
            fallback = [(self.fallback_groups[tool], {}, self.fallback_timeout_s)] if tool in self.fallback_groups \
                else []
            candidates, response = [], None
            for attempts in (local, fallback):  # the global search only if no local descent converged
                for index, variant in enumerate(variants):
                    for group, seed, timeout_s in attempts:
                        response = self._attempt(tool, group, variant, {**seed, **fixed, **result}, timeout_s,
                                                 should_stop)
                        if response.error_code.val == SUCCESS:
                            candidates.append((index, self._arm(tool, response)))
                            if self.choose is None:
                                break
                    if candidates and self.choose is None:
                        break
                if candidates:
                    break
            if not candidates:
                if response is not None and response.error_code.val == FRAME_TRANSFORM_FAILURE:
                    raise IkError(f'IK for {tool}: MoveIt has no transform to {self.frame} (TF not published?)')
                flange = ', '.join(f'{v:.2f}' for v in flange_target(target, self.tools[tool])[:3, 3])
                code = response.error_code.val if response is not None else '?'
                raise IkError(f'no IK solution for {tool}: TCP {list(target["tcp"])}, approach '
                              f'{list(target["approach"])} needs the flange at ({flange}) in {self.frame}: out of '
                              f'reach, too close to the body, or in collision (code {code})')
            _, arm = self._best(tool, candidates, {**fixed, **result}, current or {}, image)
            chosen[tool] = arm
            result.update(arm)
        return result

    def _mirrored(self, tool: str, chosen: dict) -> dict:
        """The mirror image of the solution of the arm this tool mirrors (same values, names renamed), or {}."""
        source = self.mirror.get(tool)
        if source not in chosen:
            return {}
        return {name.replace(f'{source}_', f'{tool}_', 1): value for name, value in chosen[source].items()}

    def _starts(self, tool: str) -> list[dict]:
        """``choose.starts`` postures spread over the tool's joint ranges, the same every time (seeded)."""
        pattern = re.compile(self.joint_pattern.format(tool=tool))
        names = sorted(n for n in self.limits if pattern.fullmatch(n))
        rng = np.random.default_rng(0)
        starts = []
        for _ in range(int(self.choose.get('starts', 0)) if names else 0):
            start = {}
            for name in names:
                lower, upper = (max(v, -math.pi) if i == 0 else min(v, math.pi)
                                for i, v in enumerate(self.limits[name]))
                start[name] = float(rng.uniform(lower, upper))
            starts.append(start)
        return starts

    def _attempt(self, tool, group, target, start, timeout_s, should_stop):
        from moveit_msgs.msg import RobotState
        from sensor_msgs.msg import JointState

        request = self._request(tool, group, flange_target(target, self.tools[tool]), timeout_s)
        request.ik_request.robot_state = RobotState(
            joint_state=JointState(name=list(start), position=[float(v) for v in start.values()]), is_diff=True)
        return self._call(request, tool, timeout_s, should_stop)

    def _arm(self, tool: str, response) -> dict[str, float]:
        pattern = re.compile(self.joint_pattern.format(tool=tool))
        state = response.solution.joint_state
        return {name: float(value) for name, value in zip(state.name, state.position, strict=False)
                if pattern.fullmatch(name)}

    def _best(self, tool: str, candidates: list, fixed: dict, current: dict, image: dict | None = None):
        """The candidate (variant index, arm joints) with the lowest cost; records the choice in self.last."""
        unique = []
        for index, arm in candidates:
            if not any(i == index and all(abs(arm[n] - other[n]) < 1e-3 for n in arm) for i, other in unique):
                unique.append((index, arm))
        if self.choose is None:
            self.last[tool] = {'flipped': False, 'candidates': len(unique)}
            return unique[0]
        weights = {'limits': 2.0, 'elbow': 1.0, 'motion': 0.2, 'symmetry': 0.5, **self.choose.get('weights', {})}
        margin = math.radians(float(self.choose.get('limit_margin_deg', 25.0)))
        scored = []
        for index, arm in unique:
            limits = limit_cost(arm, self.limits, margin)
            elbow = self._elbow(tool, {**fixed, **arm}) if self.choose.get('elbow') else 0.0
            motion = sum(abs(arm[n] - current[n]) for n in arm if n in current) / len(arm) if current else 0.0
            symmetry = sum(abs(arm[n] - image[n]) for n in arm if n in image) / len(arm) if image else 0.0
            cost = (weights['limits'] * limits + weights['elbow'] * elbow / math.pi + weights['motion'] * motion
                    + weights['symmetry'] * symmetry)
            scored.append((cost, index, arm, elbow, limits))
        _, index, arm, elbow, limits = min(scored, key=lambda item: item[0])
        self.last[tool] = {'flipped': bool(index), 'candidates': len(unique), 'elbow_deg': round(math.degrees(elbow)),
                           'limit_cost': round(limits, 3)}
        return index, arm

    def _elbow(self, tool: str, joints: dict) -> float:
        """Elbow deviation (rad) from the preferred side for these joints (FK through MoveIt)."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionFK
        from sensor_msgs.msg import JointState

        links = [link.format(tool=tool) for link in self.choose.get(
            'elbow_links', ['{tool}_link2', '{tool}_link4', '{tool}_link6'])]
        request = GetPositionFK.Request()
        request.header.frame_id = self.frame
        request.fk_link_names = links
        request.robot_state = RobotState(joint_state=JointState(
            name=list(joints), position=[float(v) for v in joints.values()]), is_diff=True)
        if not self.fk.wait_for_service(timeout_sec=2.0):
            raise IkError('no FK service /compute_fk (is MoveIt running?)')
        future = self.fk.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                raise IkError(f'FK service timed out for {tool}')
            time.sleep(POLL_S)
        poses = future.result().pose_stamped
        if len(poses) != 3:
            return 0.0
        shoulder, elbow, wrist = (np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for p in poses)
        return elbow_deviation(shoulder, elbow, wrist, preferred_elbow(self.choose['elbow'], shoulder))

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
