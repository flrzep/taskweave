"""Joint-space skills for robots that take joint position targets on a topic (sensor_msgs/JointState).

Robot roles used (robots/<robot>/profile.yaml):
    joints:   {command: /joint_command, state: /joint_states, linear: [<prismatic joint>, ...]}
    grippers: {<side>: {joint: <name>, open: <value>, closed: <value>}}
Named poses: robots/<robot>/poses.yaml, ``<pose>: {<joint>: <value>, ...}`` (rad or m).

Moves are streamed as position targets: a straight joint-space move, or, if the profile has a ``planning`` section,
a collision-free trajectory from MoveIt (motion_plan.py). A ros2_control implementation can later serve the same
skill names (see skills/<skill>.yaml implementations) without changing missions.

This module holds the shared machinery, not skills: the joint interface, the parameters, the move playback
(``_JointSkill``) and the bases of move_joints, move_gripper, move_object / set_posture and grip (``_PoseMotion``,
``_PointMotion``, ``_ObjectMotion``, ``_FingerMotion``: IK, planned or straight paths, the approach, the elbow
swing plan, the placed-object record). The old skills that lived here (move_to_pose, go_to_point, gripper and the
old move_object) were removed with the migration to the new skill model (taskweave config migrate-skills).
"""

from __future__ import annotations

import functools
import math
import threading
import time
from dataclasses import dataclass, field, fields

import numpy as np
from sensor_msgs.msg import JointState

from taskweave_core.config.limits import velocity_limits
from taskweave_core.skill import Skill, SkillResult

SWING_STRAIGHT_STEPS = 20  # samples of the eased straight move that follows an elbow swing (_swing_first)


@dataclass
class Params:
    max_joint_speed: float = 0.6       # rad/s (revolute) used to time a move
    max_linear_speed: float = 0.1      # m/s (prismatic, e.g. a lift)
    linear_joints: tuple = ()          # prismatic joints (metres), from the profile's joints.linear
    velocity_limits: dict = field(default_factory=dict)   # joint -> max rad/s or m/s (limits.velocity_limits): no
                                       # commanded joint goes faster; a move that would is stretched (T2.6)
    position_tolerance: float = 0.02   # rad or m
    settle_s: float = 0.3              # within tolerance this long before success
    stall_timeout_s: float = 3.0       # after the planned move: no progress this long -> fail / holding
    stall_speed: float = 0.05          # stall_ok moves (closing fingers): creeping slower than this (rad/s, m/s) is
                                       # no progress either (a sim finger grinding through a collider)
    min_duration_s: float = 0.5
    rate_hz: float = 50.0
    state_timeout_s: float = 1.0       # no joint states this long -> fail
    elbow_tolerance_deg: float = 20.0  # an elbow swing ending further than this from its side fails (move_object
                                       # elbow) or is not used (move_to_pose: planned instead)
    override_slew_per_s: float = 2.0   # the speed override changes playback speed by at most this (fraction/s)
    grip_drift_m: float = 0.02         # free move_object, several hands: the hands may drift this far apart (m)

    @classmethod
    def from_config(cls, params: dict) -> Params:
        # robot-specific: profile joints.linear
        known = {f.name for f in fields(cls)} - {'linear_joints', 'velocity_limits'}
        unknown = set(params) - known
        if unknown:
            raise ValueError(f'unknown joint skill parameters: {", ".join(sorted(unknown))}')
        return cls(**{k: float(v) for k, v in params.items()})


def smoothstep(fraction: float) -> float:
    fraction = max(0.0, min(1.0, fraction))
    return fraction * fraction * (3 - 2 * fraction)


def joint_speed(name: str, p: Params) -> float:
    """The speed a joint moves at: the configured one (rad/s, m/s for a linear joint), capped by its velocity limit."""
    speed = p.max_linear_speed if name in p.linear_joints else p.max_joint_speed
    return min(speed, p.velocity_limits.get(name, math.inf))


def plan_duration(start: dict, goal: dict, p: Params) -> float:
    """Time for the slowest joint at its configured speed, none faster than its velocity limit."""
    duration = p.min_duration_s
    for name, target in goal.items():
        duration = max(duration, abs(target - start[name]) / joint_speed(name, p))
    return duration


def capped_joints(start: dict, goal: dict, p: Params) -> dict[str, float]:
    """The joints of a straight move whose configured speed is above their velocity limit: {joint: limit}. Only joints
    that move (a joint at its goal is not capped by anything)."""
    capped = {}
    for name, target in goal.items():
        limit = p.velocity_limits.get(name)
        speed = p.max_linear_speed if name in p.linear_joints else p.max_joint_speed
        if limit is not None and speed > limit and abs(target - start[name]) > 1e-9:
            capped[name] = limit
    return capped


def limit_path(path, p: Params):
    """``path`` (motion_plan.Trajectory) stretched in time so no joint exceeds its velocity limit on any segment;
    returns (path, {joint: limit} of the joints that needed it). A path within the limits is returned as it is."""
    factor, capped = 1.0, {}
    limits = p.velocity_limits
    for index, name in enumerate(path.names):
        limit = limits.get(name)
        if limit is None:
            continue
        for k in range(1, len(path.times)):
            dt = path.times[k] - path.times[k - 1]
            move = abs(path.points[k][index] - path.points[k - 1][index])
            if move > limit * dt * (1 + 1e-9) and move > 1e-9:
                capped[name] = limit
                factor = max(factor, move / (limit * max(dt, 1e-9)))
    if factor <= 1.0:
        return path, {}
    return type(path)(path.names, [t * factor for t in path.times], path.points), capped


def interpolate(start: dict, goal: dict, fraction: float) -> dict:
    s = smoothstep(fraction)
    return {name: start[name] + (target - start[name]) * s for name, target in goal.items()}


class JointInterface:
    """Latest joint state plus a command publisher, shared by the joint skills of one server."""

    _instances: dict = {}

    @classmethod
    def for_node(cls, node, profile) -> JointInterface:
        key = id(node)
        if key not in cls._instances:
            cls._instances[key] = cls(node, profile.role('joints'))
        return cls._instances[key]

    def __init__(self, node, topics: dict):
        self.lock = threading.Lock()
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.stamp = -math.inf
        self.reached: dict[str, float] = {}   # the goal values of the last move each joint reached (start())
        self.publisher = node.create_publisher(JointState, topics['command'], 10)
        node.create_subscription(JointState, topics['state'], self._state, 10)
        self.state_topic = topics['state']

    def _state(self, msg: JointState) -> None:
        with self.lock:
            self.positions.update(zip(msg.name, msg.position, strict=False))
            if msg.velocity:
                self.velocities.update(zip(msg.name, msg.velocity, strict=False))
            self.stamp = time.monotonic()

    def snapshot(self, max_age: float) -> dict | None:
        """Every joint seen on the state topic (a planner's start state)."""
        with self.lock:
            return dict(self.positions) if time.monotonic() - self.stamp <= max_age else None

    def start(self, max_age: float, tolerance: float) -> dict | None:
        """snapshot(), with each joint within ``tolerance`` of the goal its last move reached set to that goal: the
        start of an IK solve, plan or path is then the same in identical runs, not the settling noise of the joint
        states (an IK descent or a choice between solutions can turn a milliradian into another arm configuration)."""
        state = self.snapshot(max_age)
        if state is None:
            return None
        with self.lock:
            return {**state, **{n: v for n, v in self.reached.items() if n in state and abs(state[n] - v) <= tolerance}}

    def current(self, names, max_age: float) -> dict | None:
        with self.lock:
            if time.monotonic() - self.stamp > max_age or any(n not in self.positions for n in names):
                return None
            return {n: self.positions[n] for n in names}

    def command(self, targets: dict) -> None:
        self.publisher.publish(JointState(name=list(targets), position=[float(v) for v in targets.values()]))


def _report_caps(execute):
    """``execute`` that resets the skill's velocity caps first and reports them in a finished result as
    ``data.motion.capped`` ({joint: limit}: joints held to their limit)."""
    @functools.wraps(execute)
    def wrapper(self, ctx, args):
        self._capped = {}
        result = execute(self, ctx, args)
        if self._capped and result is not None and hasattr(result, 'data'):
            motion = result.data.setdefault('motion', {})
            if isinstance(motion, dict):
                motion['capped'] = dict(self._capped)
        return result
    return wrapper


class _JointSkill(Skill):
    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if 'execute' in cls.__dict__:
            cls.execute = _report_caps(cls.__dict__['execute'])

    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self.p = Params.from_config(params)
        self.p.linear_joints = tuple(robot.role('joints').get('linear', ()))
        self.p.velocity_limits = velocity_limits(config, robot.name)
        self._capped: dict[str, float] = {}
        self.joints = JointInterface.for_node(node, robot)
        self._held: dict = {}
        from .hold import HoldState
        self.hold = HoldState.for_node(node, robot)

    def _note_caps(self, capped: dict) -> None:
        """Remember joints held to their velocity limit by this move (reported by the execute wrapper)."""
        if capped:
            self.__dict__.setdefault('_capped', {}).update(capped)

    def _hands_moved(self, joints) -> set[str]:
        """The hands whose arm has a joint in ``joints``: members of the ``<tool>_arm`` groups, else the ik joints
        regex per tool. A robot with neither: every hand if ``joints`` is not empty."""
        import re

        from taskweave_core.config.limits import arm_group_names, joint_groups
        touched = set(joints)
        if not touched:
            return set()
        groups = joint_groups(self.config, self.robot.name)
        names = arm_group_names(self.config, self.robot.name)
        hands = {g.removesuffix('_arm') for g in names if touched & set(groups.get(g, ()))}
        for hand, gripper in (self.robot.data.get('grippers') or {}).items():   # a gripper joint is its hand's
            if gripper.get('joint') in touched:
                hands.add(hand)
        ik = getattr(self, 'ik', None)
        if ik is not None:
            hands |= {tool for tool in ik.groups
                      if any(re.fullmatch(ik.joint_pattern.format(tool=tool), n) for n in touched)}
        elif not groups:
            hands |= {g.removesuffix('_arm') for g in names}
        return hands

    def _holding_refusal(self, hands=(), joints=()):
        """REJECTED if the motion moves a hand that holds something: ``hands`` (hand names) or the arm joints in
        ``joints``. The free arm may move while the other hand holds; None then."""
        holding = self.hold.hand_holds
        if not holding:
            return None
        blocked = sorted((set(hands) | self._hands_moved(joints)) & set(holding))
        if not blocked:
            return None
        text = '; '.join(f'{hand} holds {holding[hand]}' for hand in blocked)
        free = sorted(set(self.hold.tool_frames) - set(holding))
        return SkillResult.rejected(f'{text}: the hand and its arm stay with the object, move it with move_object, or '
                                    f'open the gripper first' + (f' (free: {", ".join(free)})' if free else ''))

    def _follow_holds(self, holds: list, label: str):
        """After a motion that may have moved the base frame of the held objects (torso): each object moved with its
        hands. A failure result, or None."""
        for hold in holds:
            tool = hold['hands'][0]
            try:
                tcp = self.hold.tcp_poses([tool])[tool]
            except LookupError as error:
                return SkillResult.failed(f'{label} reached, but where is the held object now? {error}')
            self.hold.moved(tcp @ np.linalg.inv(hold['tcp_in_object'][tool]), hold['object'])
        return None

    def _start(self) -> dict | None:
        """Every joint now, a joint at the goal its last move reached at exactly that goal (JointInterface.start)."""
        return self.joints.start(self.p.state_timeout_s, self.p.position_tolerance)

    def stop(self) -> None:
        """Hold where the joints are now."""
        if self._held:
            current = self.joints.current(list(self._held), self.p.state_timeout_s)
            if current:
                self.joints.command(current)

    def _move(self, ctx, goal: dict, label: str, stall_ok: bool = False, path=None):
        """Move the joints in ``goal``; returns (result or None, final positions). ``stall_ok``: stopping early
        (e.g. fingers blocked by an object) ends the move without failing. ``path``: a planned trajectory
        (motion_plan.Trajectory) the joints follow instead of a straight joint-space move.
        Playback runs on a phase clock (work time): the trajectory is timed at 100 % and ``tau`` advances at the
        global speed override, so are the settle and stall checks after it (time / o, stall_speed x o)."""
        self._held = goal
        for name in goal:  # moving: no longer at its last goal (start())
            self.joints.reached.pop(name, None)
        start = self.joints.current(list(goal), self.p.state_timeout_s)
        if start is None:
            missing = [n for n in goal if n not in self.joints.positions]
            reason = f'unknown joints {missing}' if missing else f'no joint states on {self.joints.state_topic}'
            return SkillResult.failed(reason), None
        if path is None:
            duration = plan_duration(start, goal, self.p)
            self._note_caps(capped_joints(start, goal, self.p))
            reference = lambda t: interpolate(start, goal, t / duration)  # noqa: E731
        else:  # goal joints the planner did not move go straight alongside
            path, capped = limit_path(path, self.p)
            self._note_caps(capped)
            rest_goal = {n: v for n, v in goal.items() if n not in path.names}
            self._note_caps(capped_joints(start, rest_goal, self.p))
            duration = max(path.duration, self.p.min_duration_s)
            rest = {n: v for n, v in goal.items() if n not in path.names}
            reference = lambda t: {**interpolate(start, rest, t / duration), **path.at(t)}  # noqa: E731
        clock = ctx.work_clock(self.p.override_slew_per_s)  # phase clock: tau = integral of the override
        period = 1.0 / self.p.rate_hz
        settled_since, best, best_time = None, math.inf, None
        while True:
            if ctx.should_stop:
                self.stop()
                return ctx.interrupted(), None
            now = clock(ctx.now())  # tau: work seconds since the start of the move
            fraction = now / duration
            self.joints.command(reference(now))
            current = self.joints.current(list(goal), self.p.state_timeout_s)
            if current is None:
                self.stop()
                return SkillResult.failed(f'joint states lost on {self.joints.state_topic}'), None
            errors = {n: abs(goal[n] - current[n]) for n in goal}
            worst = max(errors, key=errors.get)
            ctx.feedback(min(1.0, fraction), f'{label}: worst {worst} {errors[worst]:.3f}')
            if fraction >= 1.0:
                if errors[worst] <= self.p.position_tolerance:
                    settled_since = settled_since if settled_since is not None else now
                    if now - settled_since >= self.p.settle_s:
                        self.joints.reached.update(goal)
                        return None, current
                else:
                    settled_since = None
                    total = sum(errors.values())
                    need = self.p.position_tolerance / 4
                    if stall_ok and best_time is not None:
                        need = max(need, self.p.stall_speed * (now - best_time))
                    if best_time is None or total < best - need:
                        best, best_time = total, now
                    elif now - best_time > self.p.stall_timeout_s:
                        if stall_ok:
                            return None, current
                        self.stop()
                        return SkillResult.failed(
                            f'{label} not reached: {worst} is {errors[worst]:.3f} off after stalling'), None
            time.sleep(period)


class _PoseMotion(_JointSkill):
    """Joint targets, IK for tool targets and the planned or straight path to them: what move_joints and move_gripper
    build on. Not a skill itself (the old move_to_pose was replaced by move_joints / move_gripper)."""

    default_timeout_s = 60.0

    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self.ik = self.planner = None
        if robot.data.get('ik'):
            from taskweave_core.config.limits import joint_limits

            from .ik import IkClient
            self.ik = IkClient(node, robot, limits=joint_limits(config, robot.name))
            from taskweave_core.config.limits import urdf_path
            urdf = urdf_path(config, robot.name)  # cell pack override, else the robot pack
            if urdf.is_file():  # straight lines on the URDF chains, away from joint limits (cartesian.linear_path)
                group = (robot.data.get('planning') or {}).get('group')
                tools = self.ik.attach_chains(urdf, group)
                tool_names = ', '.join(tools) or 'no tool'
                node.get_logger().info(f'{self.name}: straight lines on the URDF chains of {tool_names}')
        if robot.data.get('planning'):
            from .motion_plan import MotionPlanner
            self.planner = MotionPlanner(node, robot)

    def _refresh_ik(self) -> None:
        """The profile's current ik.choose (edited in the HMI) for the next solve."""
        if self.ik is not None:
            self.ik.set_choice(self.config.robot(self.robot.name).data['ik'].get('choose'))

    def _arm_joints(self, joints: dict) -> list[str]:
        """The joints of ``joints`` that belong to a tool's arm (all of them if the profile has no ik section)."""
        import re

        if self.ik is None:
            return list(joints)
        patterns = [re.compile(self.ik.joint_pattern.format(tool=tool)) for tool in self.ik.groups]
        return [name for name in joints if any(p.fullmatch(name) for p in patterns)]

    def _reach(self, ctx, joints: dict, tools: dict, label: str, fallback_joints=(), linear=False, flip=True,
               elbow=None, planner='auto'):
        """Set ``joints`` and put each tool on its target (IK), along a planned path if there is a planner. If IK
        finds no solution with ``joints``, each of ``fallback_joints`` (e.g. other lift heights) is tried instead.
        ``linear``: the TCPs move on straight lines (cartesian.linear_path) instead of a planned path; the lift
        stays. ``planner``: auto, direct or planned (see _go). Returns (failure result or None, worst joint error at
        the end)."""
        from .ik import IkError

        if linear:
            return self._linear(ctx, tools, label)
        goal = dict(joints)
        if (self.ik or self.planner) and self._start() is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)'), 0
        if tools:
            if self.ik is None:
                return SkillResult.rejected(f'{label} has tool targets, but the robot profile has no ik'), 0
            ctx.feedback(0.0, 'solving IK')
            first_error = None
            for attempt in [joints, *fallback_joints]:
                try:
                    goal = {**attempt, **self.ik.solve(tools, attempt, lambda: ctx.should_stop,
                                                       current=self._start(),
                                                       allow_flip=flip, elbow=elbow)}
                    break
                except IkError as error:
                    first_error = first_error or error
            else:
                tried = f' (also tried {", ".join(str(a) for a in fallback_joints)})' if fallback_joints else ''
                return SkillResult.failed(f'{first_error}{tried}'), 0
        return self._go(ctx, goal, label, elbow=self.ik.elbow_side if tools else None, planner=planner)

    def _go(self, ctx, goal: dict, label: str, elbow=None, planner='auto'):
        """Move to the joint ``goal`` along a planned path if there is a planner. ``elbow``: the side IK solved the
        arms for; if the straight move is blocked, the elbows swing there first (_swing_first). ``planner``: auto: as
        said; direct: the straight joint move only, failing if it is blocked; planned: always the planner. Sets
        ``path_kind`` (direct, swing_first, planned). Returns (failure or None, worst joint error at the end)."""
        from .motion_plan import PlanError

        path = None
        self.path_kind = 'direct'
        if self.planner is not None:
            ctx.feedback(0.0, 'planning')
            start = self._start()
            if start is None:
                return SkillResult.failed(f'no joint states on {self.joints.state_topic}'), 0
            stop = lambda: ctx.should_stop  # noqa: E731
            try:
                if planner == 'direct':
                    if not self.planner.free(start, goal, stop):
                        return SkillResult.failed(f'{label}: straight move blocked (planner direct)'), 0
                elif planner == 'planned' or elbow is None or not self.planner.direct:
                    path, self.path_kind = self.planner.plan(start, goal, stop), 'planned'
                elif not self.planner.free(start, goal, stop):
                    path = self._swing_first(ctx, start, goal, elbow)
                    self.path_kind = 'swing_first'
                    if not path:
                        path, self.path_kind = self.planner.plan(start, goal, stop), 'planned'
            except PlanError as error:
                return SkillResult.failed(f'{label}: {error}'), 0
            if path is not None and self.path_kind == 'planned':
                path = self._slowed(path)
        result, final = self._move(ctx, goal, label, path=path)
        if result is not None:
            return result, 0
        return None, max(abs(goal[n] - final[n]) for n in goal)

    def _slowed(self, path):
        """A planned path as it is played (move_gripper stretches it to its speed fraction)."""
        return path

    def _swing_first(self, ctx, start: dict, goal: dict, side):
        """A goal IK solved with the elbows toward ``side`` whose straight move is blocked, typically from elbows on
        the other side (halfway the hands swing through each other, and a planner's path wanders, hands and elbows
        flailing): the elbows swing toward ``side`` first, as far as the goal has them, the hands staying, then the
        arms move straight to ``goal``. That Trajectory, or None if the swing stops more than p.elbow_tolerance_deg
        short or the straight move from there is blocked too."""
        import re

        from .cartesian import PathError, elbow_deviation_of, elbow_swing_path, swing_short
        from .motion_plan import Trajectory

        chains = getattr(self.ik, 'chains', None) or {}
        patterns = {tool: re.compile(self.ik.joint_pattern.format(tool=tool)) for tool in chains}
        tools = [tool for tool, p in patterns.items() if any(p.fullmatch(n) for n in goal)]
        if not tools or any(n not in start for n in goal) or any(chains[tool][0].elbow is None for tool in tools):
            return None  # nothing to swing, or an arm without elbow control (profile arms.<tool>.elbow)
        arm = {n: v for n, v in start.items() if any(patterns[tool].fullmatch(n) for tool in tools)}
        settled = {tool: elbow_deviation_of(chains[tool][0], {**start, **goal}, side) for tool in tools}
        try:
            swing, report = elbow_swing_path(self.ik, tools, arm, start, side, self.p.max_joint_speed, settled)
        except PathError:
            return None
        short = swing_short(report, self.p.elbow_tolerance_deg, {t: math.degrees(v) for t, v in settled.items()})
        if short:
            self.node.get_logger().info(f'{self.name}: straight move blocked, elbow swing toward {side} stops short: '
                                        f'{short}; planning instead')
            return None
        if swing is None:
            return None
        swung = {**start, **dict(zip(swing.names, swing.points[-1], strict=True))}
        if not self.planner.free(swung, goal, lambda: ctx.should_stop):
            return None
        duration, steps = plan_duration(swung, goal, self.p), SWING_STRAIGHT_STEPS  # eased like a straight move
        straight = Trajectory(list(goal), [duration * k / steps for k in range(steps + 1)],
                              [list(interpolate(swung, goal, k / steps).values()) for k in range(steps + 1)])
        self.node.get_logger().info(f'{self.name}: straight move blocked: elbows toward {side} first, then straight')
        return swing.then(straight)


    def _linear(self, ctx, tools: dict, label: str, goals=None):
        """Move the TCPs of ``tools`` ({tool: {tcp, approach, closing}}, or ``goals``: {tool: 4x4}) on straight
        lines from where they are. Returns (failure result or None, worst joint error)."""
        import re

        from .cartesian import PathError, linear_path, matching_flip, tcp_targets

        if self.ik is None:
            return SkillResult.rejected(f'{label}: a straight-line motion needs the profile\'s ik section'), 0
        current = self._start()
        if current is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)'), 0
        goals = goals if goals is not None else tcp_targets(tools)
        patterns = [re.compile(self.ik.joint_pattern.format(tool=tool)) for tool in goals]
        arm = {n: v for n, v in current.items() if any(p.fullmatch(n) for p in patterns)}
        try:
            starts = self.hold.tcp_poses(goals)
            goals = matching_flip(goals, starts, self.robot.data.get('tool_frames', {}))
            ctx.feedback(0.0, 'straight-line path')
            path = linear_path(self.ik, starts, goals, arm, self.tcp_speed, self.tcp_turn, state=current)
        except (LookupError, PathError) as error:
            return SkillResult.failed(f'{label}: {error}'), 0
        goal = dict(zip(path.names, path.points[-1], strict=True))
        result, final = self._move(ctx, goal, label, path=path)
        if result is not None:
            return result, 0
        self._check_hands(goals, label)
        return None, max(abs(goal[n] - final[n]) for n in goal)

    def _check_hands(self, goals: dict, label: str, tolerance_m: float = 0.02) -> None:
        """Log where a hand ended more than ``tolerance_m`` from its goal (TF): joints that reached their targets but a
        hand that did not, e.g. pushed by a contact in the simulator."""
        try:
            reached = self.hold.tcp_poses(goals)
        except LookupError:
            return
        for tool, goal in goals.items():
            miss = float(np.linalg.norm(reached[tool][:3, 3] - goal[:3, 3]))
            if miss > tolerance_m:
                self.node.get_logger().warning(
                    f'{label}: {tool} TCP ended {miss * 100:.1f} cm from its goal: at '
                    f'{np.round(reached[tool][:3, 3], 3).tolist()}, goal {np.round(goal[:3, 3], 3).tolist()}')


class _PointMotion(_PoseMotion):
    """Reaching a point of a placed object: the approach (planned path to ``approach_m`` before the point, then a
    straight line) and the lift fallback; move_gripper builds on it (the old go_to_point was replaced by it)."""

    # skills/<skill>.yaml defaults: tcp_speed (m/s) and tcp_turn (rad/s) of straight-line TCP motions
    param_defaults = {'tcp_speed': 0.05, 'tcp_turn': 0.3}

    def setup(self, node, config, robot, params):
        params = dict(params)
        self.cell_frame = params.pop('cell_frame', 'world')  # TF frame of the cell (object and location poses)
        # if the point is out of reach at the current lift: these heights of this joint are tried, in order
        self.lift_joint, self.lift_values = params.pop('lift_joint', None), list(params.pop('lift_values', []))
        self.tcp_speed = float(params.pop('tcp_speed', self.param_defaults['tcp_speed']))
        self.tcp_turn = float(params.pop('tcp_turn', self.param_defaults['tcp_turn']))
        self.approach_m = float(params.pop('approach_m', 0.0))  # straight final approach (m) unless args say other
        super().setup(node, config, robot, params)

    def _approach_in(self, ctx, tools: dict, before: dict, lifts: list, label: str, elbow=None, approach=0.0,
                     planner='auto'):
        """Planned (free) path to ``before`` (the tools ``approach`` m back along their approach), then straight in to
        ``tools``. Returns (failure or None, worst joint error at the end, label)."""
        from .cartesian import PathError, linear_path, tcp_targets
        from .ik import flipped

        end = self._end_of_approach(ctx, tools, lifts, label, elbow)  # how the point is reached decides
        if isinstance(end, SkillResult):
            return end, 0, label
        lift, flips, solution = end
        tools = {t: flipped(v) if t in flips else v for t, v in tools.items()}
        before = {t: flipped(v) if t in flips else v for t, v in before.items()}
        try:  # solved backwards from the point: the approach starts in the configuration that ends there
            path = linear_path(self.ik, tcp_targets(tools), tcp_targets(before), {**lift, **solution},
                               self.tcp_speed, self.tcp_turn,
                               state={**(self._start() or {}), **lift, **solution}).reversed()
        except PathError as error:
            return SkillResult.failed(f'{label}: no straight {approach:g} m approach to the point: {error}'), 0, label
        first, last = ({**lift, **dict(zip(path.names, path.points[i], strict=True))} for i in (0, -1))
        result, _ = self._go(ctx, first, f'{label} (-{approach:g} m approach)', elbow=self.ik.elbow_side,
                             planner=planner)
        if result is not None:
            return result, 0, label
        label += f' ({approach:g} m straight)'
        result, final = self._move(ctx, last, label, path=path)
        return result, 0 if result is not None else max(abs(last[n] - final[n]) for n in last), label

    def _end_of_approach(self, ctx, tools: dict, lifts: list, label: str, elbow=None):
        """Solve the point itself first: the lift height (current, else the first of ``lifts``), the tools whose grasp
        is reached flipped and the arm joints there, so the approach can be solved back from it."""
        from .ik import IkError

        if self.ik is None:
            return SkillResult.rejected(f'{label}: a straight approach needs the profile ik section')
        first_error = None
        for attempt in [{}, *lifts]:
            try:
                solution = self.ik.solve(tools, attempt, lambda: ctx.should_stop,
                                         current=self._start(), elbow=elbow)
            except IkError as error:
                first_error = first_error or error
                continue
            return attempt, {tool for tool, info in self.ik.last.items() if info.get('flipped')}, solution
        return SkillResult.failed(f'{label}: the point itself is out of reach: {first_error}')


class _ObjectMotion(_PointMotion):
    """Helpers for moving a held object: the elbow swing plan and the contact pose; move_object and set_posture build
    on it (the old move_object was replaced by the two)."""

    def _swing_plan(self, ctx, tools: list, side: str, label: str, tolerance_deg: float | None = None):
        """The elbow swing of ``tools`` toward ``side``, planned but not moved: (failure or None, Trajectory or None
        if already there, cartesian.elbow_swing_path report, text of what it does). A swing that would stop more than
        ``tolerance_deg`` (default p.elbow_tolerance_deg) off ``side`` fails here, before anything moves."""
        import re

        from .cartesian import PathError, elbow_swing_path, swing_short

        tolerance = self.p.elbow_tolerance_deg if tolerance_deg is None else tolerance_deg
        if self.ik is None:
            return SkillResult.rejected(f"{label}: needs the profile's ik section"), None, {}, ''
        current = self._start()
        if current is None:
            return (SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)'),
                    None, {}, '')
        patterns = [re.compile(self.ik.joint_pattern.format(tool=tool)) for tool in tools]
        arm = {n: v for n, v in current.items() if any(p.fullmatch(n) for p in patterns)}
        try:
            ctx.feedback(0.0, 'elbow swing')
            path, report = elbow_swing_path(self.ik, list(tools), arm, current, side, self.p.max_joint_speed)
        except PathError as error:
            return SkillResult.failed(f'{label}: {error}'), None, {}, ''
        swung = ', '.join(f'{tool} {a} -> {b} deg off {side}' for tool, (a, b, _) in report.items())
        short = swing_short(report, tolerance)
        if short:  # a partial swing is not what the mission asked for: fail before moving
            return SkillResult.failed(f'{label}: the elbows would stop more than {tolerance:g} deg '
                                      f'off {side}: {short}; not moved'), None, report, swung
        return None, path, report, swung

    def _onto(self, hold: dict, args: dict):
        """The held object's pose (base frame) with its contact ``args['contact']`` on the placed contact
        ``args['to']``, ``offset_m`` back against that contact's z (engage direction); a SkillResult if that cannot be
        resolved."""
        from .points import onto_pose

        instances = self.config.object_instances()
        held = instances.get(hold['object'])
        if held is None:
            return SkillResult.rejected(f'the held {hold["object"]} is not in cell/objects.yaml: '
                                        'its contacts are unknown')
        try:
            target, contact = self.config.instance_contact(args['to'])
            mine = self.config.object_contact(held.object, args['contact'])
            base_in_cell = self.hold.transform(self.cell_frame, self.robot.base['frame'])
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        except LookupError as error:
            return SkillResult.failed(f'where is the robot? {error}')
        return onto_pose(target, contact, mine, base_in_cell, float(args.get('offset_m', 0.0)))


class _FingerMotion(_JointSkill):
    """Finger moves and the placed-object record of a release; grip builds on it (the old gripper was replaced by
    grip)."""

    def _place(self, name: str, hold: dict) -> str:
        """Save ``hold``'s object pose as the placed object ``name`` (see _placed)."""
        import math

        instance = self.config.object_instances().get(hold['object'])
        if instance is None:
            return f'{hold["object"]} is not a placed object, {name} not placed'
        try:
            pose = self.hold.transform(self.cell_frame, self.robot.base['frame']) @ hold['object_in_base']
        except LookupError as error:
            return f'where is the robot? {error}; {name} not placed'
        yaw = math.atan2(pose[1, 0], pose[0, 0])
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, pose[2, 2]))))
        self.config.put_object(name, instance.object, float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3]), yaw)
        return (f'placed {name} at ({pose[0, 3]:.3f}, {pose[1, 3]:.3f}, {pose[2, 3]:.3f}), yaw '
                f'{math.degrees(yaw):.1f} deg' + (f' (tilted {tilt:.0f} deg: stored level)' if tilt > 2 else ''))
