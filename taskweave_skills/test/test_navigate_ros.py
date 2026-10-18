"""navigate_to against a small kinematic base simulator (cmd_vel in, odom out), plus teaching."""

import json
import math
import threading
import time
from pathlib import Path

import pytest
from conftest import connect, wait_until

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.teach import Teach  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_interfaces.srv import TeachPoint  # noqa: E402

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
        self.cmd_topic = f'/{prefix}/cmd_vel'
        self.create_subscription(Twist, self.cmd_topic, lambda m: setattr(self, 'cmd', m), 10)
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
    # production path: skill names resolved through config/skills/<name>.yaml implementations
    server = SkillServer(parameter_overrides=params + [Parameter('skills', value=['navigate_to', 'wait', 'drive_straight'])])
    teach = Teach(parameter_overrides=params)
    client = rclpy.create_node('test_client')
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (sim, server, teach, client):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    wait_until(lambda: sim.pub.get_subscription_count() > 0, what='the skill server to subscribe to the odometry')
    wait_until(lambda: sim.count_publishers(sim.cmd_topic) > 0, what='the skill server to advertise cmd_vel')
    yield sim, client, config
    executor.shutdown()
    for node in (sim, server, teach, client):
        node.destroy_node()
    rclpy.shutdown()


def run_skill(client, skill, args, timeout_s=20.0):
    action = connect(client, RunSkill, names.skill_action(skill))
    goal = RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args), timeout_s=timeout_s)
    handle_future = action.send_goal_async(goal)
    wait_until(handle_future.done, what='the goal to be accepted')
    result_future = handle_future.result().get_result_async()
    wait_until(result_future.done, timeout=timeout_s + 30, what='the skill result')
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
    wait_until(lambda: sim.cmd.linear.x == 0.0, what='the base to be told to stop')


def test_timeout(system):
    _, client, _ = system
    result = run_skill(client, 'wait', {'seconds': 5.0}, timeout_s=0.5)
    assert result.outcome == RunSkill.Result.TIMEOUT


def test_teach_then_drive_back(system):
    sim, client, config = system
    assert run_skill(client, 'navigate_to', {'target': 'dock'}).outcome == RunSkill.Result.SUCCEEDED
    teach = client.create_client(TeachPoint, names.TEACH_POINT)
    assert teach.wait_for_service(timeout_sec=30)
    future = teach.call_async(TeachPoint.Request(name='taught_dock', kind='location'))
    wait_until(future.done, what='the teach answer')
    assert future.result().success, future.result().message
    assert 'taught_dock' in (config / 'cell' / 'locations.yaml').read_text()
    assert run_skill(client, 'navigate_to', {'target': 'home'}).outcome == RunSkill.Result.SUCCEEDED
    result = run_skill(client, 'navigate_to', {'target': 'taught_dock'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert math.hypot(sim.x - 1.0, sim.y - 0.5) < 0.05


def test_drive_straight_forward_and_back_keeps_the_heading(system):
    sim, client, _ = system
    sim.yaw = 0.5
    seen = []  # the odometry with the new heading has been published (and so reaches the skill server too)
    client.create_subscription(Odometry, sim.pub.topic_name, lambda m: seen.append(m.pose.pose.orientation.z), 10)
    wait_until(lambda: seen and seen[-1] == pytest.approx(math.sin(0.25), abs=1e-3), what='odometry at the new heading')
    time.sleep(0.2)  # a few more messages for the server (a delivery delay, not a decision)
    result = run_skill(client, 'drive_straight', {'distance_m': 0.4})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert sim.x == pytest.approx(0.4 * math.cos(0.5), abs=0.04) and sim.y == pytest.approx(0.4 * math.sin(0.5), abs=0.04)
    assert sim.yaw == pytest.approx(0.5, abs=0.03)
    result = run_skill(client, 'drive_straight', {'distance_m': -0.4})   # backs out without turning round
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert math.hypot(sim.x, sim.y) < 0.04 and sim.yaw == pytest.approx(0.5, abs=0.03)


def test_drive_straight_limits(system):
    sim, client, _ = system
    result = run_skill(client, 'drive_straight', {'distance_m': 2.0})
    assert result.outcome == RunSkill.Result.REJECTED and 'max_distance_m' in result.message
    result = run_skill(client, 'drive_straight', {'distance_m': 0.0, 'sideways_m': 0.3})
    if sim.holonomic:
        assert result.outcome == RunSkill.Result.SUCCEEDED and sim.y == pytest.approx(0.3, abs=0.04)
    else:
        assert result.outcome == RunSkill.Result.REJECTED and 'sideways' in result.message


def test_drive_straight_to_a_target_in_line(system):
    sim, client, config = system
    with open(config / 'cell' / 'locations.yaml', 'a') as f:
        f.write('ahead: {x: 0.5, y: 0.01, yaw: 0.02}\n')
    result = run_skill(client, 'drive_straight', {'to': 'ahead'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert sim.x == pytest.approx(0.5, abs=0.04) and abs(sim.yaw) < 0.03   # the heading is kept, not turned
    result = run_skill(client, 'drive_straight', {'to': 'dock'})          # faces 90 deg away, off to the side
    assert result.outcome == RunSkill.Result.REJECTED and 'heading' in result.message
