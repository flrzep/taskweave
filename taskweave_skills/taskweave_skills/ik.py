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
        limit_margin_deg: 25          # joints closer than this to a limit cost (more the closer)
        weights: {limits: 2.0, elbow: 1.0, motion: 0.2, symmetry: 0.5}
        starts: 0                     # extra start postures (fixed pseudo-random, so repeatable) for more candidates
        start_timeout_s: 0.05         # per extra start

The elbow (its side when choosing, continuation, straight lines and swings) needs the profile's description of the arm:
``arms: {left: {elbow: {shoulder: left_link2, elbow: left_link4, wrist: left_link6}}}``, the links at the shoulder,
elbow and wrist centres. An arm without it has no elbow control (deprecated: ``choose.elbow_links`` with {tool}).
Step sizes and thresholds of the chain stepping and the continuation: profile ``motion_tuning`` (schemas.py).

A redundant arm (7 joints) reaches a TCP pose in a whole family of configurations, and which one a solver returns
depends on where it starts. IK starts from each seed posture in turn, then from the current state; the pose's fixed
joints (e.g. the torso lift) are applied on top, so the arms are solved for the torso height the pose sets. With a
deterministic local solver (pick_ik mode local: gradient descent from the start), a named pose then always solves
to the same configuration, whatever pose the robot comes from; only targets no seed reaches depend on the current
state. The current state is the joints the skill passes (a joint at the goal of its last move exactly at that goal),
not MoveIt's measured state: a descent from a start a milliradian off may converge to another configuration or not at
all, so identical runs would not choose the same. If nothing converges, the fallback group (a global solver) searches
from the current state; its answer is not repeatable (logged).

A target a short move from where the arm is (IkClient._continue: within 25 cm and 45 degrees of the flange) is first
reached by continuation: the flange walked there in a straight line on the URDF chain (cartesian.walk_to), the joints
moving continuously, no IK search. That solution wins unless it collides or puts the elbow far off its side: a seed
can converge to another branch of a 7-joint arm (the wrist turned over, joint 5 or 7 half a turn round) for a few cm
of TCP, and whether a descent from the current state converges can hang on a milliradian. A jump like that, chosen
because no continuation got there, is logged.

With ``choose``, every start that converges is a candidate, for the target and, if the tool frame says
``flip: true`` (a grasp that is the same turned 180 degrees about the approach), for the flipped target too. The
candidate with the lowest cost wins: joints near their limits (an arm at a limit cannot move on, e.g. lift a held
object), the elbow away from the preferred side (the swivel of the elbow about the shoulder-wrist line, measured with
/compute_fk; "out" and y are mirrored for an arm on the right, so symmetric targets give mirrored arms), and the size
of the move from the current joints. With ``mirror``, an arm is solved after the arm it mirrors, starts from the mirror
image of that arm's solution and prefers it (``symmetry``): symmetric targets give mirrored arms. Industrial
controllers store such a configuration with every taught point; here the profile gives the default.

With the profile's ``ik.torso`` ({joint, candidates, weights: {torso}, line_sample_m}) ``solve_with_torso`` also
chooses the torso height: see there.
"""

from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass, field

import numpy as np

from taskweave_core.config.schemas import IK_WEIGHTS, ROBOT_PROFILE, default
from taskweave_core.config.tuning import motion_tuning

SUCCESS = 1
FRAME_TRANSFORM_FAILURE = -21
POLL_S = 0.02
VALIDITY_POLL_S = 0.002  # poll period of a collision check


class IkError(RuntimeError):
    """No joint solution for a tool target; the message names the tool and target."""


class TorsoError(IkError):
    """No torso height works; ``rejected`` lists every height with its reason."""

    def __init__(self, message: str, rejected: list | None = None):
        super().__init__(message)
        self.rejected = rejected or []


@dataclass
class TorsoChoice:
    """solve_with_torso's answer: the torso ``height`` (also in ``fixed``, the fixed joints with it set), the arm
    joints of the first sample, which tools' grasps are flipped, the ``cost`` and every ``rejected`` height."""
    height: float
    fixed: dict
    solution: dict
    flips: set
    cost: float
    rejected: list = field(default_factory=list)

    def report(self) -> dict:
        return {'height': round(self.height, 4), 'cost': round(self.cost, 4), 'rejected': self.rejected}


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


def target_warnings(target: dict, near_m: float = 0.2, back_cos: float = 0.3) -> list[str]:
    """Likely mistakes in a tool target that IK alone reports only as 'no solution': a TCP more than ``near_m`` out
    whose approach points back toward the robot (cosine below -``back_cos``, seen from above)."""
    warnings = []
    tcp, approach = np.asarray(target['tcp'], float), np.asarray(target['approach'], float)
    if np.linalg.norm(tcp[:2]) > near_m and approach[:2] @ tcp[:2] < -back_cos * np.linalg.norm(
            approach[:2]) * np.linalg.norm(tcp[:2]):
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


WEIGHTS = IK_WEIGHTS                                                       # ik.choose.weights defaults
LIMIT_MARGIN_DEG = default(ROBOT_PROFILE, 'ik', 'choose', 'limit_margin_deg')   # ik.choose.limit_margin_deg default
START_TIMEOUT_S = default(ROBOT_PROFILE, 'ik', 'choose', 'start_timeout_s')     # ik.choose.start_timeout_s default
TORSO_WEIGHT = default(ROBOT_PROFILE, 'ik', 'torso', 'weights', 'torso')           # ik.torso.weights.torso default
LINE_SAMPLE_M = default(ROBOT_PROFILE, 'ik', 'torso', 'line_sample_m')             # ik.torso.line_sample_m default


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
        self.torso = spec.get('torso')     # {joint, candidates, weights}: the torso height search (solve_with_torso)
        self.seeds = [{name: float(value) for name, value in seed.items()} for seed in spec.get('seeds', [])]
        self.Request = GetPositionIK.Request
        self.client = node.create_client(GetPositionIK, self.service, callback_group=ReentrantCallbackGroup())
        self.node, self.fk = node, None
        self.mirror = spec.get('mirror', {})  # tool -> the tool whose joints it mirrors
        self.limits = limits or {}
        self.last: dict[str, dict] = {}   # per tool: how the last solve() chose (for messages)
        self.elbow_side = None             # the preferred elbow of the solve() running
        self.tuning = motion_tuning(robot)  # profile motion_tuning, defaults filled in (cartesian.py, chain.py)
        self.arms = robot.data.get('arms') or {}
        self.warned: set = set()           # tools told once that they have no elbow description
        self.set_choice(spec.get('choose'))

    def set_choice(self, choose: dict | None) -> None:
        """How to choose between solutions (profile ik.choose; None: the first solution wins, seed order). Skills
        set it from the profile before solving, so an edit (HMI) applies without a restart."""
        self.choose = choose
        if choose is not None and self.fk is None:
            from moveit_msgs.srv import GetPositionFK
            from rclpy.callback_groups import ReentrantCallbackGroup

            self.fk = self.node.create_client(GetPositionFK, choose.get('fk_service', '/compute_fk'),
                                              callback_group=ReentrantCallbackGroup())

    def elbow_links(self, tool: str) -> tuple[str, str, str] | None:
        """The links at the shoulder, elbow and wrist centres of the tool's arm (profile arms.<tool>.elbow; deprecated:
        ik.choose.elbow_links with {tool}), or None: the arm has no elbow control."""
        elbow = (self.arms.get(tool) or {}).get('elbow')
        if elbow:
            return elbow['shoulder'], elbow['elbow'], elbow['wrist']
        links = (self.choose or {}).get('elbow_links')
        return tuple(link.format(tool=tool) for link in links) if links else None

    def attach_chains(self, urdf, validity_group: str | None = None) -> list[str]:
        """Kinematic chains (chain.py) from the URDF for straight-line moves (cartesian.linear_path): base -> each
        tool's flange link, with the arm joints IK moves. ``validity_group``: the MoveIt group whose collisions a step
        is checked against (default: the tool's IK group). Returns the tools that got one."""
        from moveit_msgs.srv import GetStateValidity
        from rclpy.callback_groups import ReentrantCallbackGroup

        from .chain import Chain

        self.chains = {}
        for tool, frame in self.tools.items():
            if tool not in self.groups:
                continue
            try:
                chain = Chain.from_urdf(urdf, self.frame, frame['link'], tuning=self.tuning['chain'])
            except (ValueError, OSError):
                continue
            elbow = self.elbow_links(tool)
            if elbow and any(link not in chain.links for link in elbow):
                self.node.get_logger().warning(f'IK for {tool}: elbow links {list(elbow)} (profile arms.{tool}.elbow) '
                                               f'are not all on the chain to {frame["link"]}: no elbow control')
                elbow = None
            chain.elbow = elbow
            pattern = re.compile(self.joint_pattern.format(tool=tool))
            self.chains[tool] = (chain, [name for name in chain.movable if pattern.fullmatch(name)])
        self.validity_group = validity_group
        self.validity = self.node.create_client(GetStateValidity, '/check_state_validity',
                                                callback_group=ReentrantCallbackGroup())
        return sorted(self.chains)

    def valid(self, joints: dict, group: str) -> bool:
        """MoveIt's collision check of ``joints`` on top of its current state (True if the service is not there)."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        if not self.validity.wait_for_service(timeout_sec=1.0):
            return True
        request = GetStateValidity.Request(group_name=self.validity_group or group, robot_state=RobotState(
            joint_state=JointState(name=list(joints), position=[float(v) for v in joints.values()]), is_diff=True))
        future = self.validity.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done():
            if time.monotonic() > deadline:
                raise IkError('collision check (/check_state_validity) did not answer')
            time.sleep(VALIDITY_POLL_S)
        return future.result().valid

    def solve(self, tools: dict, fixed: dict, should_stop=lambda: False, current: dict | None = None,
              allow_flip: bool = True, elbow=None) -> dict[str, float]:
        """Joint values for every tool target (plus nothing else); raises IkError. ``current``: the joints now, for
        the size-of-move cost when choosing. ``allow_flip`` False: solve the targets as given (no flipped grasp).
        ``elbow``: preferred elbow side for this solve (e.g. an object point's), else the profile's."""
        if not self.client.wait_for_service(timeout_sec=2.0):
            raise IkError(f'no IK service {self.service} (is MoveIt running?)')
        self.elbow_side = elbow or (self.choose or {}).get('elbow')
        for tool in tools:
            if self.elbow_side and tool not in self.warned and self.elbow_links(tool) is None:
                self.warned.add(tool)
                self.node.get_logger().warning(f'IK for {tool}: elbow {self.elbow_side} asked for, but the robot '
                                               f'profile has no arms.{tool}.elbow: no elbow control, side ignored')
        result: dict[str, float] = {}
        self.last = {}
        here = self._here(current or {})  # the current state start: the joints as given ({}: MoveIt's own state)
        chosen: dict[str, dict] = {}  # per tool: its arm joints, for the arm that mirrors it
        order = sorted(tools, key=lambda t: self.mirror.get(t) in tools)  # mirror sources first
        for tool in order:
            target = tools[tool]
            if tool not in self.groups or tool not in self.tools:
                raise IkError(f'tool {tool!r} has no IK group or tool frame in the robot profile')
            flip = allow_flip and self.choose is not None and self.tools[tool].get('flip')
            variants = [target, flipped(target)] if flip else [target]
            positions = {**(current or {}), **fixed, **result}
            # a seed names the arm it solves; the other arms (the one holding an object) stay where they are now, so
            # the collision check sees the real scene (arms solved earlier in this call override: ``result``)
            others = {n: float(v) for n, v in (current or {}).items() if any(
                re.fullmatch(self.joint_pattern.format(tool=t), n) for t in self.groups if t != tool)}
            continued, why = self._continue(tool, variants, positions)
            if continued is not None:  # the arm can get there as it is: no seed or search decides
                chosen[tool] = continued
                result.update(continued)
                continue
            local = [(self.groups[tool], seed, self.timeout_s) for seed in [*self.seeds, here]]
            image = self._mirrored(tool, chosen)
            if image:
                local.insert(0, (self.groups[tool], image, self.timeout_s))
            if self.choose is not None:
                local += [(self.groups[tool], start, float(self.choose.get('start_timeout_s', START_TIMEOUT_S)))
                          for start in self._starts(tool)]
            fallback = [(self.fallback_groups[tool], {}, self.fallback_timeout_s)] if tool in self.fallback_groups \
                else []
            candidates, response, searched = [], None, False
            for attempts in (local, fallback):  # the global search only if no local descent converged
                if attempts is fallback and fallback:
                    self.node.get_logger().warning(f'IK for {tool}: no local descent converged, global search '
                                                   f'({fallback[0][0]}): its answer is not repeatable')
                for index, variant in enumerate(variants):
                    for group, seed, timeout_s in attempts:
                        response = self._attempt(tool, group, variant, {**seed, **others, **fixed, **result}, timeout_s,
                                                 should_stop)
                        if response.error_code.val == SUCCESS:
                            candidates.append((index, self._arm(tool, response)))
                            if self.choose is None:
                                break
                    if candidates and self.choose is None:
                        break
                if candidates:
                    searched = attempts is fallback
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
            if why:
                self._jump_warning(tool, arm, positions, why)
            if searched:
                self.last[tool]['global'] = True      # the global search's answer is not repeatable
            chosen[tool] = arm
            result.update(arm)
        return result

    def solve_with_torso(self, samples, fixed: dict, current: dict, should_stop=lambda: False, elbow=None,
                         allow_flip: bool = True) -> TorsoChoice:
        """The torso height (profile ik.torso) that solves ``samples`` for every hand at once, with the least cost.
        ``samples``: a list of tool target sets {tool: target} (or a function of the height returning it: a line moves
        with the torso); the first is the main one (its arm cost counts), the others (the pre-grasp, points along a
        line) must be solvable and collision free too. For each candidate height (the profile's grid and the current
        one, nearest first) every sample is solved (``solve``) and its full state checked (/check_state_validity: the
        torso, the solved arms and the idle arms as they are now). Cost: the arm cost of the main
        sample (ik.choose weights: joint limits, elbow, arm motion) + ik.torso.weights.torso * |height change|; ties go
        to the smaller change, then the lower height, so the same input gives the same height. Raises TorsoError."""
        spec = self.torso
        joint = spec['joint']
        if joint not in current:
            raise IkError(f'torso joint {joint} has no state yet')
        now, weight = float(current[joint]), float((spec.get('weights') or {}).get('torso', TORSO_WEIGHT))
        heights = sorted({round(float(v), 6) for v in spec['candidates']} | {now},
                         key=lambda h: (round(abs(h - now), 9), h))
        best, rejected = None, []
        for index, height in enumerate(heights):
            if best is not None and weight * abs(height - now) >= best.cost:   # every later one costs more
                rejected += [{'height': round(h, 4), 'reason': 'not tried: moves the torso more than the best'}
                             for h in heights[index:]]
                break
            try:
                choice = self._torso_height(samples, height, fixed, current, joint, now, weight, should_stop, elbow,
                                            allow_flip)
            except TorsoError as error:
                rejected.append({'height': round(height, 4), 'reason': str(error)})
                continue
            if best is None or (round(choice.cost, 9), round(abs(choice.height - now), 9), choice.height) < (
                    round(best.cost, 9), round(abs(best.height - now), 9), best.height):
                best = choice
        rejected.sort(key=lambda item: item['height'])
        if best is None:
            raise TorsoError('no torso height works: ' + '; '.join(f'{r["height"]:g}: {r["reason"]}' for r in rejected),
                             rejected)
        best.rejected = rejected
        return best

    def _torso_height(self, samples, height, fixed, current, joint, now, weight, should_stop, elbow,
                      allow_flip) -> TorsoChoice:
        """One candidate of solve_with_torso, or TorsoError saying why it is out."""
        with_h = {**fixed, joint: height}
        goals = samples(height) if callable(samples) else samples
        main = None
        for k, tools in enumerate(goals):
            where = f'sample {k}: ' if k else ''
            try:
                arm = self.solve(tools, with_h, should_stop, current=current, allow_flip=allow_flip, elbow=elbow)
            except TorsoError:
                raise
            except IkError as error:
                raise TorsoError(f'{where}{str(error).split(":")[0]}') from error
            if k == 0:
                main = (arm, {t for t, info in self.last.items() if info.get('flipped')}, self._arm_cost(arm, current))
            if not self._state_valid({joint: height, **self.idle_joints(tools, current), **arm}, next(iter(tools))):
                raise TorsoError(f'{where}collision (idle arm included)')
        arm, flips, cost = main
        return TorsoChoice(height, with_h, arm, flips, cost + weight * abs(height - now))

    def idle_joints(self, tools, current: dict) -> dict[str, float]:
        """The arm joints of ``current`` that no tool of ``tools`` moves: the idle arms, kept where they are, so a
        collision check sees the same state every time (not MoveIt's live one)."""
        patterns = [re.compile(self.joint_pattern.format(tool=t)) for t in self.groups if t not in tools]
        return {n: float(v) for n, v in current.items() if any(p.fullmatch(n) for p in patterns)}

    def _state_valid(self, joints: dict, tool: str) -> bool:
        """Full-state collision check of ``joints`` (on the current state); True without the check service."""
        return True if getattr(self, 'validity', None) is None else self.valid(joints, self.groups[tool])

    def _arm_cost(self, arm: dict, current: dict) -> float:
        """The cost solve() chose by for the arms it just solved (self.last): joint-limit room, elbow side and the
        size of the arm move (symmetry left out: it only breaks ties between mirrored solutions)."""
        weights = {**WEIGHTS, **(self.choose or {}).get('weights', {})}
        cost = 0.0
        for tool, info in self.last.items():
            joints = [n for n in arm if re.fullmatch(self.joint_pattern.format(tool=tool), n)]
            motion = sum(abs(arm[n] - current[n]) for n in joints if n in current) / max(len(joints), 1)
            cost += (weights['limits'] * info.get('limit_cost', 0.0) + weights['motion'] * motion
                     + weights['elbow'] * math.radians(info.get('elbow_deg', 0.0)) / math.pi)
        return cost

    def tcps_at(self, tools, state: dict) -> dict[str, np.ndarray]:
        """TCP poses (4x4, base frame) of ``tools`` for the joints in ``state`` (chains only; tools without a chain
        are left out): where the hands would be after a torso move with the arms keeping their joints."""
        poses = {}
        for tool in tools:
            if tool in getattr(self, 'chains', {}) and tool in self.tools:
                frame = self.tools[tool]
                poses[tool] = self.chains[tool][0].fk(state) @ rpy_matrix(frame.get('xyz', [0, 0, 0]),
                                                                          frame.get('rpy', [0, 0, 0]))
        return poses

    def _continue(self, tool: str, variants: list, positions: dict):
        """The continuation of the arm's configuration to a target a short move away (at most motion_tuning
        continuation max_m and max_deg from the flange now): the flange walked there in a straight line on the URDF
        chain (cartesian.walk_to), the joints moving continuously from ``positions`` (every chain joint now). It wins
        over seed and search solutions, which may be another branch of a 7-joint arm (a wrist turned over: joint 5 or 7
        half a turn round for a few cm of TCP), unless it ends in collision or with the elbow more than elbow_deg off
        the preferred side. Returns (arm joints or None, why not; None if no continuation applies: no chain, no current
        joints, too far)."""
        from .cartesian import elbow_deviation_of, rotation_angle, walk_to

        tuning = self.tuning['continuation']
        max_m, max_deg, elbow_deg = tuning['max_m'], tuning['max_deg'], tuning['elbow_deg']
        chains = getattr(self, 'chains', {})
        if tool not in chains or any(n not in positions for n in chains[tool][0].movable):
            return None, None
        chain, arm = chains[tool]
        now, whys = chain.fk(positions), []
        for index, variant in enumerate(variants):
            goal = flange_target(variant, self.tools[tool])
            distance, turn = np.linalg.norm(goal[:3, 3] - now[:3, 3]), rotation_angle(now[:3, :3].T @ goal[:3, :3])
            if distance > max_m or turn > math.radians(max_deg):
                continue
            joints, why = walk_to(self, tool, positions, goal)
            if joints is None:
                whys.append(why)
                continue
            state = {**positions, **joints}
            elbow = elbow_deviation_of(chain, state, self.elbow_side) if self.elbow_side and chain.elbow else 0.0
            if elbow > math.radians(elbow_deg):
                whys.append(f'elbow {math.degrees(elbow):.0f} deg off {self.elbow_side}')
                continue
            try:
                free = not self.avoid_collisions or self.valid({n: state[n] for n in chain.movable}, self.groups[tool])
            except IkError as error:  # no answer: the IK search decides
                whys.append(str(error))
                continue
            if not free:
                whys.append('in collision')
                continue
            margin = math.radians(float((self.choose or {}).get('limit_margin_deg', LIMIT_MARGIN_DEG)))
            self.last[tool] = {'flipped': bool(index), 'candidates': 1, 'continued': True,
                               'elbow_deg': round(math.degrees(elbow)), 'limit_cost': round(limit_cost(
                                   joints, self.limits, margin), 3)}
            return joints, ''
        return None, ('; '.join(whys) or None)

    def _jump_warning(self, tool: str, arm: dict, positions: dict, why: str) -> None:
        """Log a solution that turns a joint more than motion_tuning continuation.jump_deg for a short move (another
        branch of the arm), chosen because no continuation got there (``why``)."""
        jump_deg = self.tuning['continuation']['jump_deg']
        turns = {n: abs(v - positions[n]) for n, v in arm.items() if n in positions}
        joint = max(turns, key=turns.get, default=None)
        if joint is not None and turns[joint] > math.radians(jump_deg):
            self.node.get_logger().warning(
                f'IK for {tool}: configuration jump, {joint} turns {math.degrees(turns[joint]):.0f} deg for a short '
                f'move: no continuous way there ({why})')

    def _here(self, current: dict) -> dict:
        """The joints of ``current`` an IK start sets: every arm's and, with chains, the joints between base and arm
        (e.g. a lift). Sent explicitly, not left to MoveIt's state (the measured one, noisy while settling): a local
        descent from a start a milliradian off can converge, or not, to another configuration."""
        patterns = [re.compile(self.joint_pattern.format(tool=tool)) for tool in self.groups]
        chained = {name for chain, _ in getattr(self, 'chains', {}).values() for name in chain.movable}
        return {n: float(v) for n, v in current.items() if n in chained or any(p.fullmatch(n) for p in patterns)}

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

    def _best(self, tool: str, candidates: list, fixed: dict, current: dict, image: dict | None = None,
              same_rad: float = 1e-3):
        """The candidate (variant index, arm joints) with the lowest cost (candidates whose joints are all within
        ``same_rad`` count once); records the choice in self.last."""
        unique = []
        for index, arm in candidates:
            if not any(i == index and all(abs(arm[n] - other[n]) < same_rad for n in arm) for i, other in unique):
                unique.append((index, arm))
        if self.choose is None:
            self.last[tool] = {'flipped': False, 'candidates': len(unique)}
            return unique[0]
        weights = {**WEIGHTS, **self.choose.get('weights', {})}
        margin = math.radians(float(self.choose.get('limit_margin_deg', LIMIT_MARGIN_DEG)))
        scored = []
        for index, arm in unique:
            limits = limit_cost(arm, self.limits, margin)
            elbow = self._elbow(tool, {**fixed, **arm}) if self.elbow_side else 0.0
            motion = sum(abs(arm[n] - current[n]) for n in arm if n in current) / len(arm) if current else 0.0
            symmetry = sum(abs(arm[n] - image[n]) for n in arm if n in image) / len(arm) if image else 0.0
            cost = (weights['limits'] * limits + weights['elbow'] * elbow / math.pi + weights['motion'] * motion
                    + weights['symmetry'] * symmetry)
            scored.append((cost, index, arm, elbow, limits))
        _, index, arm, elbow, limits = min(scored, key=lambda item: item[0])
        self.last[tool] = {'flipped': bool(index), 'candidates': len(unique), 'elbow_deg': round(math.degrees(elbow)),
                           'limit_cost': round(limits, 3)}
        return index, arm

    def _elbow(self, tool: str, joints: dict, timeout_s: float = 5.0) -> float:
        """Elbow deviation (rad) from the preferred side for these joints (FK through MoveIt); 0 for an arm without an
        elbow description."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionFK
        from sensor_msgs.msg import JointState

        links = self.elbow_links(tool)
        if links is None:
            return 0.0
        request = GetPositionFK.Request()
        request.header.frame_id = self.frame
        request.fk_link_names = list(links)
        request.robot_state = RobotState(joint_state=JointState(
            name=list(joints), position=[float(v) for v in joints.values()]), is_diff=True)
        if not self.fk.wait_for_service(timeout_sec=2.0):
            raise IkError('no FK service /compute_fk (is MoveIt running?)')
        future = self.fk.call_async(request)
        deadline = time.monotonic() + timeout_s
        while not future.done():
            if time.monotonic() > deadline:
                raise IkError(f'FK service timed out for {tool}')
            time.sleep(POLL_S)
        poses = future.result().pose_stamped
        if len(poses) != 3:
            return 0.0
        shoulder, elbow, wrist = (np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z]) for p in poses)
        return elbow_deviation(shoulder, elbow, wrist, preferred_elbow(self.elbow_side, shoulder))

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

    def _call(self, request, tool: str, timeout_s: float, should_stop, margin_s: float = 5.0):
        """The service's answer to ``request``; it must come within ``timeout_s`` (the solver's) plus ``margin_s``."""
        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout_s + margin_s
        while not future.done():
            if should_stop() or time.monotonic() > deadline:
                raise IkError(f'IK for {tool} interrupted' if should_stop() else f'IK service timed out for {tool}')
            time.sleep(POLL_S)
        return future.result()
