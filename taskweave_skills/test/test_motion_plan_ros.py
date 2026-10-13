"""move_to_pose with a motion planner: follows the planned trajectory, fails when there is no path."""

import shutil
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from builtin_interfaces.msg import Duration  # noqa: E402
from moveit_msgs.srv import GetMotionPlan  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from test_joints_ros import FIXTURE, JointSim, run  # noqa: E402
from trajectory_msgs.msg import JointTrajectoryPoint  # noqa: E402

from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_skills.motion_plan import MotionPlanner, Trajectory  # noqa: E402

PLANNING = 'planning: {group: arm, timeout_s: 1.0}\n'


class FakePlanner(Node):
    """Plans start -> (j1 = -0.5 detour) -> goal, or answers with ``error``."""

    def __init__(self):
        super().__init__('fake_planner')
        self.error = 0
        self.fail_first = 0  # answer this many requests with a pipeline failure first
        self.requests = []
        self.create_service(GetMotionPlan, '/plan_kinematic_path', self.plan)

    def plan(self, request, response):
        r = request.motion_plan_request
        self.requests.append(r)
        if self.error or self.fail_first:
            response.motion_plan_response.error_code.val = self.error or 99999
            self.fail_first = max(0, self.fail_first - 1)
            return response
        start = dict(zip(r.start_state.joint_state.name, r.start_state.joint_state.position, strict=True))
        goal = {c.joint_name: c.position for c in r.goal_constraints[0].joint_constraints}
        names = list(goal)
        detour = [-0.5 if n == 'j1' else start[n] for n in names]
        points = [([start[n] for n in names], 0.0), (detour, 0.6), ([goal[n] for n in names], 1.5)]
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        trajectory.joint_names = names
        trajectory.points = [JointTrajectoryPoint(positions=p, time_from_start=Duration(sec=int(t), nanosec=int(
            (t % 1) * 1e9))) for p, t in points]
        response.motion_plan_response.error_code.val = 1
        return response


@pytest.fixture
def system(tmp_path):
    config = tmp_path / 'config'
    shutil.copytree(FIXTURE, config)
    profile = config / 'robots' / 'arm_bot' / 'profile.yaml'
    profile.write_text(profile.read_text() + PLANNING)
    rclpy.init()
    sim, planner = JointSim(), FakePlanner()
    server = SkillServer(parameter_overrides=[
        Parameter('config_dir', value=str(config)), Parameter('robot', value='arm_bot'),
        Parameter('skills', value=['move_to_pose'])])
    client = rclpy.create_node('test_client')
    seen = {'j1_min': 0.0}
    client.create_subscription(JointState, '/arm/joint_states', lambda m: seen.update(
        j1_min=min(seen['j1_min'], dict(zip(m.name, m.position, strict=False)).get('j1', 0.0))), 10)
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (sim, planner, server, client):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.3)
    yield sim, planner, client, seen
    executor.shutdown()
    for node in (sim, planner, server, client):
        node.destroy_node()
    rclpy.shutdown()


def test_trajectory_interpolates_between_points():
    path = Trajectory(['a', 'b'], [0.0, 1.0, 3.0], [[0.0, 0.0], [1.0, 2.0], [1.0, 0.0]])
    assert path.at(-1.0) == {'a': 0.0, 'b': 0.0}
    assert path.at(0.5) == {'a': 0.5, 'b': 1.0}
    assert path.at(2.0) == {'a': 1.0, 'b': 1.0}
    assert path.at(9.0) == {'a': 1.0, 'b': 0.0} and path.duration == 3.0


def test_move_follows_the_planned_path(system):
    sim, planner, client, seen = system
    result = run(client, 'move_to_pose', {'pose': 'ready'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert sim.position['j1'] == pytest.approx(1.0, abs=0.02)
    assert seen['j1_min'] < -0.3  # went through the planner's detour, not straight
    request = planner.requests[-1]
    assert request.group_name == 'arm'  # start: the goal joints only (the state topic may carry unknown joints)
    assert set(request.start_state.joint_state.name) == {'j1', 'j2', 'lift'}
    assert request.start_state.is_diff  # on top of MoveIt's state (base pose from TF)
    assert {c.joint_name for c in request.goal_constraints[0].joint_constraints} == {'j1', 'j2', 'lift'}


def test_no_path_fails_without_moving(system):
    sim, planner, client, _ = system
    planner.error = -12
    result = run(client, 'move_to_pose', {'pose': 'ready'})
    assert result.outcome == RunSkill.Result.FAILED and 'goal in collision' in result.message
    assert sim.position['j1'] == 0.0


def test_a_failed_plan_is_planned_again(system):
    sim, planner, client, _ = system
    planner.fail_first = 1  # e.g. the path failed MoveIt's final validation
    result = run(client, 'move_to_pose', {'pose': 'ready'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert len(planner.requests) == 2 and sim.position['j1'] == pytest.approx(1.0, abs=0.02)


def test_joints_outside_the_goal_are_held_where_they_are(system):
    _, planner, client, _ = system
    robot = SimpleNamespace(data={'planning': {'group': 'arm', 'timeout_s': 1.0, 'joints': 'j[12]|lift'}})
    start = {'j1': 0.0, 'j2': 0.3, 'lift': 0.1, 'base_x_joint': 4.0}  # base_x_joint: not a planning joint
    MotionPlanner(client, robot).plan(start, {'j1': 0.5})
    goal = {c.joint_name: c.position for c in planner.requests[-1].goal_constraints[0].joint_constraints}
    assert goal == pytest.approx({'j1': 0.5, 'j2': 0.3, 'lift': 0.1})  # without joints: j2 and lift would be free
