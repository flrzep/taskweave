"""Collision-free joint trajectories from a GetMotionPlan service (MoveIt's /plan_kinematic_path).

Profile section ``planning`` (without it, joint skills move straight in joint space):

    planning:
      service: /plan_kinematic_path   # default
      group: whole_body               # planning group containing every joint a pose moves
      pipeline: ompl                  # default
      planner: RRTConnectkConfigDefault   # optional planner id
      timeout_s: 5.0                  # planning time
      attempts: 3
      velocity_scale: 1.0             # of the planner's joint limits (MoveIt joint_limits.yaml)
      acceleration_scale: 1.0
      direct: true                    # default: a straight joint-space move when it is collision-free (checked with
      direct_step_rad: 0.03           # /check_state_validity every this many rad or m), the planner only otherwise
      escape_max_rad: 0.4             # start in collision: how far (largest joint change) the robot may move straight
      escape_speed: 0.2               # toward the goal while still touching (rad/s or m/s)
      goal_tolerance_rad: 0.001       # planned goal: each joint within this (rad or m)
      check_timeout_s: 10.0           # the straight-move collision checks give up after this
      answer_margin_s: 10.0           # the planning service must answer within timeout_s plus this

A sampling planner's path wanders (the arms swing out and back) even where the straight move is free, so the straight
move is tried first. The planner checks the robot against itself and the planning scene; the trajectory comes back timed, and the joint
skills stream it as position targets. Goal joints outside the group are moved straight alongside.

A start that already touches something (e.g. an elbow at a rack upright after driving) cannot be planned from. Then the
robot moves straight toward the goal while it touches only what it touched at the start, no deeper, until it is clear
(within ``escape_max_rad``), and the rest is planned from there. If that direction makes it worse, the error names the
contact.
"""

from __future__ import annotations

import bisect
import math
import re
import time

SUCCESS = 1
POLL_S = 0.02
PLAN_TRIES = 3
CHECK_BATCH = 8  # state validity requests in flight at once
DEEPER_M = 0.005  # an escape may not push a contact deeper than at the start by more than this
UNKNOWN_DEPTH_M = 0.3  # FCL's depth between meshes is sometimes meters off: deeper than this says nothing
RETRY = {-1, -2, -6, 99999}  # planning failed, invalid plan (final validation), timed out, pipeline failure


class PlanError(RuntimeError):
    """No trajectory; the message says why (MoveIt error code)."""


class Trajectory:
    """Timed joint positions; ``at(t)`` interpolates linearly between the planner's points."""

    def __init__(self, names: list[str], times: list[float], points: list[list[float]]):
        if not times:
            raise PlanError('empty trajectory')
        self.names, self.times, self.points = names, times, points

    @property
    def duration(self) -> float:
        return self.times[-1]

    def at(self, t: float) -> dict[str, float]:
        if t <= self.times[0]:
            return dict(zip(self.names, self.points[0], strict=True))
        if t >= self.times[-1]:
            return dict(zip(self.names, self.points[-1], strict=True))
        i = bisect.bisect_right(self.times, t)
        t0, t1 = self.times[i - 1], self.times[i]
        s = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        return {n: a + (b - a) * s for n, a, b in zip(self.names, self.points[i - 1], self.points[i], strict=True)}

    def then(self, other: Trajectory) -> Trajectory:
        """This trajectory, then ``other`` (which starts where this one ends); joints only one of them names hold
        their value from the other's nearest point."""
        names = list(dict.fromkeys([*self.names, *other.names]))

        def row(traj, point, fill):
            return [point[traj.names.index(n)] if n in traj.names else fill[n] for n in names]

        mine_end = dict(zip(self.names, self.points[-1], strict=True))
        theirs_start = dict(zip(other.names, other.points[0], strict=True))
        return Trajectory(names, [*self.times[:-1], *(self.duration + t for t in other.times)],
                          [*(row(self, p, theirs_start) for p in self.points[:-1]),
                           *(row(other, p, mine_end) for p in other.points)])

    def reversed(self) -> Trajectory:
        """The same path backwards."""
        return Trajectory(self.names, [self.duration - t for t in reversed(self.times)], self.points[::-1])


ERRORS = {-1: 'planning failed', -2: 'invalid motion plan', -4: 'control failed', -6: 'timed out',
          -10: 'start state in collision', -11: 'start state violates path constraints',
          -12: 'goal in collision', -13: 'goal violates path constraints', -14: 'goal constraints violated',
          -15: 'invalid group name', -16: 'invalid goal constraints', -17: 'invalid robot state',
          -31: 'no IK solution', 99999: 'planner failed (see the move_group log, e.g. start state outside joint limits)'}
CHECK_POLL_S = 0.002  # poll period of the state validity requests in flight
MIN_SPAN = 1e-3  # rad or m, the shortest span a straight move is timed for (no zero-length trajectory)


class MotionPlanner:
    # profile ``planning`` defaults (see the module doc)
    DEFAULTS = {'timeout_s': 5.0, 'attempts': 3, 'velocity_scale': 1.0, 'acceleration_scale': 1.0,
                'direct_step_rad': 0.03, 'escape_max_rad': 0.4, 'escape_speed': 0.2, 'goal_tolerance_rad': 1e-3,
                'check_timeout_s': 10.0, 'answer_margin_s': 10.0}

    def __init__(self, node, robot):
        from moveit_msgs.srv import GetMotionPlan, GetStateValidity  # only needed with a planning section
        from rclpy.callback_groups import ReentrantCallbackGroup

        spec = {**self.DEFAULTS, **robot.data['planning']}
        self.group = spec['group']
        self.service = spec.get('service', '/plan_kinematic_path')
        self.pipeline = spec.get('pipeline', 'ompl')
        self.planner = spec.get('planner', '')
        self.timeout_s = float(spec['timeout_s'])
        self.attempts = int(spec['attempts'])
        self.velocity_scale = float(spec['velocity_scale'])
        self.acceleration_scale = float(spec['acceleration_scale'])
        self.goal_tolerance = float(spec['goal_tolerance_rad'])
        self.check_timeout = float(spec['check_timeout_s'])
        self.answer_margin = float(spec['answer_margin_s'])
        # the planning group's joints (regex): those not in a goal stay where they are, else the planner may move
        # them anywhere on the way (e.g. a lift or the other arm while one arm moves)
        self.joints = re.compile(spec['joints']) if spec.get('joints') else None
        self.Request = GetMotionPlan.Request
        self.client = node.create_client(GetMotionPlan, self.service, callback_group=ReentrantCallbackGroup())
        self.direct = bool(spec.get('direct', True))
        self.direct_step = float(spec['direct_step_rad'])
        self.validity = node.create_client(GetStateValidity, spec.get('validity_service', '/check_state_validity'),
                                           callback_group=ReentrantCallbackGroup())
        self.escape_max = float(spec['escape_max_rad'])
        self.escape_speed = float(spec['escape_speed'])

    def plan(self, start: dict[str, float], goal: dict[str, float], should_stop=lambda: False) -> Trajectory | None:
        """Trajectory from ``start`` to ``goal``, or None if the straight joint-space move is free; raises PlanError. Only the goal joints' start values are sent, on top
        of the planner's own current state: the state topic can carry joints the planner's model lacks (e.g. a
        simulator's base joints), and MoveIt rejects unknown names."""
        from moveit_msgs.msg import Constraints, JointConstraint, RobotState
        from sensor_msgs.msg import JointState

        if not self.client.wait_for_service(timeout_sec=2.0):
            raise PlanError(f'no planning service {self.service} (is MoveIt running?)')
        request = self.Request()
        r = request.motion_plan_request
        r.group_name, r.pipeline_id, r.planner_id = self.group, self.pipeline, self.planner
        r.num_planning_attempts, r.allowed_planning_time = self.attempts, self.timeout_s
        r.max_velocity_scaling_factor, r.max_acceleration_scaling_factor = self.velocity_scale, self.acceleration_scale
        if self.joints is not None:
            goal = {**{n: v for n, v in start.items() if self.joints.fullmatch(n) and n not in goal}, **goal}
        escape = self._escape(start, goal, should_stop)
        if escape is not None:  # the start touches something: straight out toward the goal first
            clear, prefix = escape
            if self.direct and self._free(clear, goal, should_stop):
                return None  # the whole straight move: out of the contact, then free
            rest = self.plan({**start, **clear}, goal, should_stop) or self._straight(clear, goal)
            return prefix.then(rest)
        if self.direct and self._free(start, goal, should_stop):
            return None
        start = {name: start[name] for name in goal if name in start}
        r.start_state = RobotState(joint_state=JointState(name=list(start), position=[float(v) for v in start.values()]),
                                   is_diff=True)  # keeps what MoveIt knows besides joints, e.g. the base pose from TF
        r.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(value), tolerance_above=self.goal_tolerance,
                            tolerance_below=self.goal_tolerance, weight=1.0) for name, value in goal.items()])]
        for _ in range(PLAN_TRIES):  # a sampled path can fail MoveIt's final check at finer spacing: plan again
            response = self._call(request, should_stop)
            code = response.error_code.val
            if code not in RETRY:
                break
        if code != SUCCESS:
            raise PlanError(f'no collision-free path: {ERRORS.get(code, "MoveIt error")} (code {code})')
        trajectory = response.trajectory.joint_trajectory
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in trajectory.points]
        return Trajectory(list(trajectory.joint_names), times, [list(p.positions) for p in trajectory.points])

    def free(self, start: dict, goal: dict, should_stop=lambda: False) -> bool:
        """True if the straight joint-space move from ``start`` to ``goal`` is collision-free (as plan() checks it)."""
        return self._free(start, goal, should_stop)

    def _straight(self, start: dict, goal: dict) -> Trajectory:
        """The straight joint-space move from ``start`` to ``goal`` at the escape speed."""
        names = [n for n in goal if n in start]
        duration = max([abs(goal[n] - start[n]) for n in names] + [MIN_SPAN]) / self.escape_speed
        return Trajectory(names, [0.0, duration], [[start[n] for n in names], [goal[n] for n in names]])

    def _check(self, states: list[dict], should_stop):
        """/check_state_validity for each state (joint values on top of MoveIt's current state): a list of
        responses, or None if interrupted or the service is too slow."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        deadline = time.monotonic() + self.check_timeout
        responses = []
        for first in range(0, len(states), CHECK_BATCH):  # batches: the service queue drops requests beyond its depth
            futures = []
            for state in states[first:first + CHECK_BATCH]:
                request = GetStateValidity.Request()
                request.group_name = self.group
                request.robot_state = RobotState(joint_state=JointState(
                    name=list(state), position=[float(v) for v in state.values()]), is_diff=True)
                futures.append(self.validity.call_async(request))
            while not all(f.done() for f in futures):
                if should_stop() or time.monotonic() > deadline:
                    return None
                time.sleep(CHECK_POLL_S)
            responses += [f.result() for f in futures]
        return responses

    @staticmethod
    def _contacts(response) -> dict[tuple, float]:
        """{(body, body): deepest penetration} of a state validity response."""
        contacts: dict[tuple, float] = {}
        for c in response.contacts:
            pair = tuple(sorted((c.contact_body_1, c.contact_body_2)))
            contacts[pair] = max(contacts.get(pair, 0.0), abs(c.depth))
        return contacts

    def _escape(self, start: dict, goal: dict, should_stop):
        """None if ``start`` is free (or cannot be checked). If it touches something: (the first free state on the
        straight line toward ``goal``, the timed move there), when every state on the way touches only what the start
        touched and no deeper; raises PlanError otherwise."""
        names = [n for n in goal if n in start]
        if not names or not self.validity.wait_for_service(timeout_sec=2.0):
            return None
        first = self._check([{n: start[n] for n in names}], should_stop)
        if not first or first[0].valid or not first[0].contacts:
            return None  # free, or invalid for another reason (e.g. a constraint): the planner says what
        touching = self._contacts(first[0])
        what = ', '.join(f'{a} - {b}' for a, b in touching)
        span = max(abs(goal[n] - start[n]) for n in names)
        count = max(1, math.ceil(span / self.direct_step))
        last = min(count, math.ceil(self.escape_max / self.direct_step))
        states = [{n: start[n] + (goal[n] - start[n]) * i / count for n in names} for i in range(1, last + 1)]
        responses = self._check(states, should_stop)
        if responses is None:
            raise PlanError(f'start state in collision ({what}): interrupted while looking for a way out')
        for state, response in zip(states, responses, strict=True):
            if response.valid:
                return state, self._straight({n: start[n] for n in names}, state)
            contacts = self._contacts(response)
            worse = [pair for pair, depth in contacts.items() if pair not in touching
                     or touching[pair] + DEEPER_M < depth < UNKNOWN_DEPTH_M]
            if worse:
                raise PlanError(f'start state in collision ({what}); moving toward the goal touches '
                                f'{", ".join(f"{a} - {b}" for a, b in worse)}: jog it free first')
        raise PlanError(f'start state in collision ({what}); still touching after {self.escape_max:.2f} rad toward '
                        'the goal: jog it free first')

    def _free(self, start: dict, goal: dict, should_stop) -> bool:
        """True if every state on the straight joint-space line from ``start`` to ``goal`` is collision-free."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity
        from sensor_msgs.msg import JointState

        names = list(goal)
        if any(n not in start for n in names) or not self.validity.wait_for_service(timeout_sec=2.0):
            return False
        count = max(1, math.ceil(max(abs(goal[n] - start[n]) for n in names) / self.direct_step))
        deadline = time.monotonic() + self.check_timeout
        for first in range(1, count + 1, CHECK_BATCH):  # batches: the service queue drops requests beyond its depth
            futures = []
            for i in range(first, min(first + CHECK_BATCH, count + 1)):
                request = GetStateValidity.Request()
                request.group_name = self.group
                request.robot_state = RobotState(joint_state=JointState(
                    name=names, position=[float(start[n] + (goal[n] - start[n]) * i / count) for n in names]),
                    is_diff=True)
                futures.append(self.validity.call_async(request))
            while not all(f.done() for f in futures):
                if should_stop() or time.monotonic() > deadline:
                    return False
                time.sleep(CHECK_POLL_S)
            if not all(f.result().valid for f in futures):
                return False
        return True

    def _call(self, request, should_stop):
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.timeout_s + self.answer_margin
        while not future.done():
            if should_stop():
                raise PlanError('planning interrupted')
            if time.monotonic() > deadline:
                raise PlanError(f'planning service {self.service} did not answer')
            time.sleep(POLL_S)
        return future.result().motion_plan_response
