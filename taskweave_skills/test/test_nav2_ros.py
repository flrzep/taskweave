"""navigate_to through Nav2 (implementation navigate_nav2) against a fake NavigateToPose server and a base simulator."""

import json
import math
import time
from pathlib import Path

import pytest
from conftest import RosRig, connect, wait_ready, wait_until

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')

from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from test_navigate_ros import BaseSim  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
NAV2_CONFIG = """implementations:
  - {name: navigate_nav2, type: script, class: "taskweave_skills.nav2:Nav2NavigateTo",
     params: {server_timeout_s: 20.0, handover_distance: 0.3}}
max_linear: 0.6
stuck_timeout_s: 2.0
"""


class FakeNav2(Node):
    """NavigateToPose server. mode 'near': puts the base 8 cm short of the goal and succeeds; 'abort': gives up."""

    def __init__(self, sim: BaseSim):
        super().__init__('fake_nav2')
        self.sim, self.mode, self.goals = sim, 'near', []
        ActionServer(self, NavigateToPose, 'navigate_to_pose', self.execute)

    def execute(self, handle):
        pose = handle.request.pose
        self.goals.append(pose)
        feedback = NavigateToPose.Feedback()
        feedback.distance_remaining = 1.0
        handle.publish_feedback(feedback)
        result = NavigateToPose.Result()
        if self.mode == 'abort':
            if hasattr(result, 'error_msg'):
                result.error_msg = 'no valid path'
            handle.abort()
            return result
        self.sim.x, self.sim.y = pose.pose.position.x - 0.08, pose.pose.position.y
        self.sim.yaw = 2 * math.atan2(pose.pose.orientation.z, pose.pose.orientation.w)
        time.sleep(0.2)
        handle.succeed()
        return result


class FakeCostmap(Node):
    """A costmap node: only its footprint parameter."""

    def __init__(self, name: str):
        super().__init__(name, namespace=name)
        self.declare_parameter('footprint', '[]')


@pytest.fixture
def system(tmp_path):
    with RosRig() as ros:
        import shutil

        config = tmp_path / 'config'
        shutil.copytree(FIXTURE, config)
        (config / 'skills' / 'navigate_to.yaml').write_text(NAV2_CONFIG)
        (config / 'robots' / 'omni_bot' / 'navigation.yaml').write_text(
            'envelope: {front: 0.4, back: 0.3, left: 0.3, right: 0.3}\n'
            'carry_envelope: {front: 1.2, back: 0.3, left: 0.4, right: 0.4}\n')
        sim = ros.add(BaseSim('omni', True))
        nav2 = ros.add(FakeNav2(sim))
        costmaps = [ros.add(FakeCostmap('global_costmap')), ros.add(FakeCostmap('local_costmap'))]
        server = SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value='omni_bot'),
            Parameter('skills', value=['navigate_to'])])
        ros.add(server)
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        nav2.costmaps = costmaps
        skill = server.skills['navigate_to']
        wait_ready(server, *skill.costmaps.values())
        wait_until(lambda: sim.pub.get_subscription_count() > 0, what='the skill server to subscribe to the odometry')
        wait_until(lambda: skill.client.server_is_ready(), what='the Nav2 action server')
        nav2.skill = skill
        client.taskweave_server = server
        yield sim, nav2, client


def run(client, target, timeout_s=20.0):
    action = connect(client, RunSkill, names.skill_action('navigate_to'))
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill='navigate_to',
                                                  args_json=json.dumps({'target': target}), timeout_s=timeout_s))
    wait_until(future.done, what='the goal to be accepted')
    result = future.result().get_result_async()
    wait_until(result.done, timeout=timeout_s + 30, what='the skill result')
    return result.result().result


def test_nav2_brings_it_close_and_the_direct_controller_finishes(system):
    sim, nav2, client = system
    result = run(client, 'dock')
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    goal = nav2.goals[-1]
    assert goal.header.frame_id == 'map' and goal.pose.position.x == pytest.approx(1.0)
    assert math.hypot(sim.x - 1.0, sim.y - 0.5) < 0.03                       # within the direct tolerance


def test_nav2_failure_far_from_the_target_fails_with_the_reason(system):
    _, nav2, client = system
    nav2.mode = 'abort'
    result = run(client, 'dock')
    assert result.outcome == RunSkill.Result.FAILED and 'Nav2 did not reach dock' in result.message


def test_without_a_hold_the_costmaps_get_the_envelope(system):
    _, nav2, client = system
    result = run(client, 'dock')
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    for costmap in nav2.costmaps:  # the carry envelope only while holding (hold.py)
        assert costmap.get_parameter('footprint').value == '[[0.4, 0.3], [0.4, -0.3], [-0.3, -0.3], [-0.3, 0.3]]'


def test_the_first_drive_after_a_restart_clears_a_limit_nav2_may_still_hold(system):
    """A skill server restarted during a slowed drive does not know what limit Nav2 holds: at 100 % the first drive
    still publishes 'no limit' (SpeedLimit 0), not nothing."""
    from nav2_msgs.msg import SpeedLimit

    _, nav2, client = system
    received = []
    client.create_subscription(SpeedLimit, '/speed_limit', received.append, 10)
    wait_until(lambda: nav2.skill.speed_limit.get_subscription_count() > 0, what='the speed limit subscription match')
    result = run(client, 'dock')
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    wait_until(lambda: received, what='the speed limit message')
    assert received and received[0].percentage and received[0].speed_limit == 0.0


def test_at_10_percent_nav2_drives_30_and_the_direct_controller_stays_at_10(system):
    """The Nav2 override floor: Nav2 gets 30 %, the handover to the direct controller follows the real 10 %."""
    from nav2_msgs.msg import SpeedLimit
    from rclpy.qos import DurabilityPolicy, QoSProfile

    from taskweave_interfaces.msg import SpeedOverride

    sim, nav2, client = system
    received, top = [], []
    client.create_subscription(SpeedLimit, '/speed_limit', received.append, 10)
    wait_until(lambda: nav2.skill.speed_limit.get_subscription_count() > 0, what='the speed limit subscription match')
    publisher = client.create_publisher(SpeedOverride, names.SPEED_OVERRIDE,
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(SpeedOverride(percent=10.0, cap_percent=50.0, test_percent=10.0))
    wait_until(lambda: client.taskweave_server.speed.fraction() == pytest.approx(0.1), what='the 10 % override')
    sim.create_timer(0.02, lambda: top.append(abs(sim.cmd.linear.x)))   # the direct controller's commands
    result = run(client, 'dock', timeout_s=60.0)
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert [m.speed_limit for m in received if m.speed_limit][:1] == [30.0]
    assert max(top) <= 0.6 * 0.1 + 1e-6                                   # max_linear 0.6 x the real 10 %
    speed = json.loads(result.data_json)['speed']
    assert speed['nav2_override'] == 30.0 and 'override_min' in speed
