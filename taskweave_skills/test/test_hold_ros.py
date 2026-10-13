"""Hold state: grasp at a reached point, published for the sim and the jog node, released on open."""

import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from taskweave_skills.hold import TOPIC, HoldState  # noqa: E402

ROBOT = SimpleNamespace(base={'frame': 'base_link'}, data={'tool_frames': {
    'left': {'link': 'left_link7', 'xyz': [0.1, 0, 0], 'rpy': [0, 0, 0]},
    'right': {'link': 'right_link7', 'xyz': [0.1, 0, 0], 'rpy': [0, 0, 0]}}})


@pytest.fixture
def hold():
    rclpy.init()
    node = rclpy.create_node('hold_test')
    seen = []
    node.create_subscription(String, TOPIC, lambda m: seen.append(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    tf = StaticTransformBroadcaster(node)
    transforms = []
    for link, y in (('left_link7', 0.2), ('right_link7', -0.2)):
        t = TransformStamped(child_frame_id=link)
        t.header.frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y, t.transform.rotation.w = 0.5, y, 1.0
        transforms.append(t)
    tf.sendTransform(transforms)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    state = HoldState(node, ROBOT)
    time.sleep(0.5)
    yield state, seen
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


def test_grasp_only_at_a_reached_point_then_release(hold):
    state, seen = hold
    assert state.grasp(['left', 'right']) is None and seen[-1] == {}
    obj = np.eye(4)
    obj[:3, 3] = [0.6, 0.0, 0.0]
    state.reached_point('crate_1/rails', 'crate_1', ['left', 'right'], obj)
    assert state.grasp(['left']) is None  # one hand of a two-hand point is no grasp
    assert state.grasp(['left', 'right']) == 'crate_1'
    held = state.held()
    assert held['tcp_in_object']['left'][:3, 3] == pytest.approx([0.0, 0.2, 0.0])  # TCP 0.1 ahead of the flange
    time.sleep(0.2)
    assert seen[-1] == {'object': 'crate_1', 'point': 'crate_1/rails', 'tools': ['left', 'right'],
                        'links': ['left_link7', 'right_link7']}
    assert state.release(['right']) == 'crate_1' and state.held() is None
    time.sleep(0.2)
    assert seen[-1] == {}


def test_a_simulator_reset_ends_the_hold(hold):
    state, seen = hold
    obj = np.eye(4)
    state.reached_point('crate_1/rails', 'crate_1', ['left', 'right'], obj)
    assert state.grasp(['left', 'right']) == 'crate_1'
    state._on_reset(None)  # what the clock's jump callback calls when sim time goes back
    assert state.held() is None
    time.sleep(0.2)
    assert seen[-1] == {}
    assert state.grasp(['left', 'right']) is None  # the reached point is gone too: no grasp without a new approach
