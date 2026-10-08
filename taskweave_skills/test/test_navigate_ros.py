"""navigate_to against a small kinematic base simulator (cmd_vel in, odom out), plus teaching."""

import json
import math
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.teach import Teach  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_interfaces.srv import TeachPoint  # noqa: E402
from taskweave_skills.basic import Wait  # noqa: E402
from taskweave_skills.navigate import NavigateTo  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
WALL_X = 2.0  # the simulator blocks motion beyond this x


class BaseSim(Node):
    """Integrates base-frame velocity commands and publishes odometry in the cell frame at 50 Hz."""

    def __init__(self, prefix: str, holonomic: bool):
        super().__init__(f'{prefix}_sim')
        self.x = self.y = self.yaw = 0.0
        self.cmd = Twist()
        self.holonomic = holonomic
        self.pub = self.create_publisher(Odometry, f'/{prefix}/odom', 10)
        self.create_subscription(Twist, f'/{prefix}/cmd_vel', lambda m: setattr(self, 'cmd', m), 10)
        self.create_timer(0.02, self.step)

    def step(self):
        dt, c, s = 0.02, math.cos(self.yaw), math.sin(self.yaw)
        vx, vy = self.cmd.linear.x, self.cmd.linear.y if self.holonomic else 0.0
        self.x = min(self.x + (c * vx - s * vy) * dt, WALL_X)
        self.y += (s * vx + c * vy) * dt
        yaw = self.yaw + self.cmd.angular.z * dt
        self.yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        msg = Odometry()
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.x, self.y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        self.pub.publish(msg)


@pytest.fixture(params=[('omni', True, 'omni_bot'), ('diff', False, 'diff_bot')], ids=['holonomic', 'differential'])
def system(request, tmp_path):
    import shutil

    prefix, holonomic, robot = request.param
    config = tmp_path / 'config'
    shutil.copytree(FIXTURE, config)
    rclpy.init()
    params = [Parameter('config_dir', value=str(config)), Parameter('robot', value=robot)]
    sim = BaseSim(prefix, holonomic)
    server = SkillServer(skills=[NavigateTo(), Wait()], parameter_overrides=params)
    teach = Teach(parameter_overrides=params)
    client = rclpy.create_node('test_client')
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (sim, server, teach, client):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.3)
    yield sim, client, config
    executor.shutdown()
    for node in (sim, server, teach, client):
        node.destroy_node()
    rclpy.shutdown()


def run_skill(client, skill, args, timeout_s=20.0):
    action = ActionClient(client, RunSkill, names.skill_action(skill))
    assert action.wait_for_server(timeout_sec=5)
    goal = RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args), timeout_s=timeout_s)
    handle_future = action.send_goal_async(goal)
    while not handle_future.done():
        time.sleep(0.02)
    result_future = handle_future.result().get_result_async()
    while not result_future.done():
        time.sleep(0.05)
    return result_future.result().result


def test_drives_to_location(system):
    sim, client, _ = system
    result = run_skill(client, 'navigate_to', {'target': 'dock'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert math.hypot(sim.x - 1.0, sim.y - 0.5) < 0.04 and abs(sim.yaw - 1.5708) < 0.04
    assert json.loads(result.data_json)['world_state'] == {'robot_location': 'dock'}
    assert sim.cmd.linear.x == 0.0 and sim.cmd.angular.z == 0.0


def test_unknown_target_is_rejected_without_moving(system):
    sim, client, _ = system
    result = run_skill(client, 'navigate_to', {'target': 'mars'})
    assert result.outcome == RunSkill.Result.REJECTED and "unknown location 'mars'" in result.message
    assert sim.x == 0.0 and sim.y == 0.0


def test_blocked_path_fails_as_stuck_and_stops(system):
    sim, client, _ = system
    result = run_skill(client, 'navigate_to', {'target': 'behind_wall'})
    assert result.outcome == RunSkill.Result.FAILED and 'no progress' in result.message
    time.sleep(0.2)
    assert sim.cmd.linear.x == 0.0


def test_timeout(system):
    _, client, _ = system
    result = run_skill(client, 'wait', {'seconds': 5.0}, timeout_s=0.5)
    assert result.outcome == RunSkill.Result.TIMEOUT


def test_teach_then_drive_back(system):
    sim, client, config = system
    assert run_skill(client, 'navigate_to', {'target': 'dock'}).outcome == RunSkill.Result.SUCCEEDED
    teach = client.create_client(TeachPoint, names.TEACH_POINT)
    assert teach.wait_for_service(timeout_sec=5)
    future = teach.call_async(TeachPoint.Request(name='taught_dock', kind='location'))
    while not future.done():
        time.sleep(0.02)
    assert future.result().success, future.result().message
    assert 'taught_dock' in (config / 'cell' / 'locations.yaml').read_text()
    assert run_skill(client, 'navigate_to', {'target': 'home'}).outcome == RunSkill.Result.SUCCEEDED
    result = run_skill(client, 'navigate_to', {'target': 'taught_dock'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert math.hypot(sim.x - 1.0, sim.y - 0.5) < 0.05
