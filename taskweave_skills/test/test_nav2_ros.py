"""navigate_to through Nav2 (implementation navigate_nav2) against a fake NavigateToPose server and a base simulator."""

import json
import math
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')

from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from test_navigate_ros import BaseSim  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
NAV2_CONFIG = """implementations:
  - {name: navigate_nav2, type: script, class: "taskweave_skills.nav2:Nav2NavigateTo",
     params: {server_timeout_s: 1.0, handover_distance: 0.3}}
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
    import shutil

    config = tmp_path / 'config'
    shutil.copytree(FIXTURE, config)
    (config / 'skills' / 'navigate_to.yaml').write_text(NAV2_CONFIG)
    (config / 'robots' / 'omni_bot' / 'navigation.yaml').write_text(
        'envelope: {front: 0.4, back: 0.3, left: 0.3, right: 0.3}\n'
        'carry_envelope: {front: 1.2, back: 0.3, left: 0.4, right: 0.4}\n')
    rclpy.init()
    sim = BaseSim('omni', True)
    nav2 = FakeNav2(sim)
    costmaps = [FakeCostmap('global_costmap'), FakeCostmap('local_costmap')]
    server = SkillServer(parameter_overrides=[
        Parameter('config_dir', value=str(config)), Parameter('robot', value='omni_bot'),
        Parameter('skills', value=['navigate_to'])])
    client = rclpy.create_node('test_client')
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (sim, nav2, server, client, *costmaps):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.3)
    nav2.costmaps = costmaps
    yield sim, nav2, client
    executor.shutdown()
    for node in (sim, nav2, server, client, *costmaps):
        node.destroy_node()
    rclpy.shutdown()


def run(client, target, timeout_s=20.0):
    action = ActionClient(client, RunSkill, names.skill_action('navigate_to'))
    assert action.wait_for_server(timeout_sec=5)
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill='navigate_to',
                                                  args_json=json.dumps({'target': target}), timeout_s=timeout_s))
    while not future.done():
        time.sleep(0.02)
    result = future.result().get_result_async()
    while not result.done():
        time.sleep(0.05)
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
