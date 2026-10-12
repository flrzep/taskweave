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

The planner checks the robot against itself and the planning scene; the trajectory comes back timed, and the joint
skills stream it as position targets. Goal joints outside the group are moved straight alongside.
"""

from __future__ import annotations

import bisect
import time

SUCCESS = 1
POLL_S = 0.02


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


ERRORS = {-1: 'planning failed', -2: 'invalid motion plan', -4: 'control failed', -6: 'timed out',
          -10: 'start state in collision', -11: 'start state violates path constraints',
          -12: 'goal in collision', -13: 'goal violates path constraints', -14: 'goal constraints violated',
          -15: 'invalid group name', -16: 'invalid goal constraints', -17: 'invalid robot state',
          -31: 'no IK solution', 99999: 'planner failed (see the move_group log, e.g. start state outside joint limits)'}


class MotionPlanner:
    def __init__(self, node, robot):
        from moveit_msgs.srv import GetMotionPlan  # only needed when the robot has a planning section
        from rclpy.callback_groups import ReentrantCallbackGroup

        spec = robot.data['planning']
        self.group = spec['group']
        self.service = spec.get('service', '/plan_kinematic_path')
        self.pipeline = spec.get('pipeline', 'ompl')
        self.planner = spec.get('planner', '')
        self.timeout_s = float(spec.get('timeout_s', 5.0))
        self.attempts = int(spec.get('attempts', 3))
        self.velocity_scale = float(spec.get('velocity_scale', 1.0))
        self.acceleration_scale = float(spec.get('acceleration_scale', 1.0))
        self.Request = GetMotionPlan.Request
        self.client = node.create_client(GetMotionPlan, self.service, callback_group=ReentrantCallbackGroup())

    def plan(self, start: dict[str, float], goal: dict[str, float], should_stop=lambda: False) -> Trajectory:
        """Trajectory from ``start`` (every known joint, applied to the planner's current state) to ``goal``; raises
        PlanError."""
        from moveit_msgs.msg import Constraints, JointConstraint, RobotState
        from sensor_msgs.msg import JointState

        if not self.client.wait_for_service(timeout_sec=2.0):
            raise PlanError(f'no planning service {self.service} (is MoveIt running?)')
        request = self.Request()
        r = request.motion_plan_request
        r.group_name, r.pipeline_id, r.planner_id = self.group, self.pipeline, self.planner
        r.num_planning_attempts, r.allowed_planning_time = self.attempts, self.timeout_s
        r.max_velocity_scaling_factor, r.max_acceleration_scaling_factor = self.velocity_scale, self.acceleration_scale
        r.start_state = RobotState(joint_state=JointState(name=list(start), position=[float(v) for v in start.values()]),
                                   is_diff=True)  # keeps what MoveIt knows besides joints, e.g. the base pose from TF
        r.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(value), tolerance_above=1e-3, tolerance_below=1e-3,
                            weight=1.0) for name, value in goal.items()])]
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.timeout_s + 10.0
        while not future.done():
            if should_stop():
                raise PlanError('planning interrupted')
            if time.monotonic() > deadline:
                raise PlanError(f'planning service {self.service} did not answer')
            time.sleep(POLL_S)
        response = future.result().motion_plan_response
        code = response.error_code.val
        if code != SUCCESS:
            raise PlanError(f'no collision-free path: {ERRORS.get(code, "MoveIt error")} (code {code})')
        trajectory = response.trajectory.joint_trajectory
        times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in trajectory.points]
        return Trajectory(list(trajectory.joint_names), times, [list(p.positions) for p in trajectory.points])
