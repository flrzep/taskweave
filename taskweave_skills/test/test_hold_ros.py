"""Hold state: grasp at a reached point, published for the sim and the jog node, released on open."""

import json
from types import SimpleNamespace

import numpy as np
import pytest
from conftest import RosRig, wait_until

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from taskweave_skills.hold import TOPIC, HoldState  # noqa: E402

ROBOT = SimpleNamespace(base={'frame': 'base_link'}, data={'tool_frames': {
    'left': {'link': 'left_link7', 'xyz': [0.1, 0, 0], 'rpy': [0, 0, 0]},
    'right': {'link': 'right_link7', 'xyz': [0.1, 0, 0], 'rpy': [0, 0, 0]}}})


def tf_ready(state):
    try:
        state.tcp_poses(['left', 'right'])
    except LookupError:
        return False
    return True


@pytest.fixture
def hold():
    with RosRig() as ros:
        node = ros.add(rclpy.create_node('hold_test'))
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
        ros.spin(2)
        state = HoldState(node, ROBOT)
        wait_until(lambda: seen, what='the first (latched) holding message')
        wait_until(lambda: tf_ready(state), what='the flange transforms in the buffer')
        yield state, seen


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
    wait_until(lambda: seen[-1].get('tools') == ['left', 'right'], what='the hold to be published')
    assert seen[-1] == {'object': 'crate_1', 'point': 'crate_1/rails', 'tools': ['left', 'right'],
                        'links': ['left_link7', 'right_link7'],     # the legacy keys (the sim's magic grasp reads them)
                        'holds': [{'object': 'crate_1', 'point': 'crate_1/rails', 'hands': ['left', 'right'],
                                   'links': ['left_link7', 'right_link7']}]}
    assert state.release(['right']) == 'crate_1' and state.held() is None
    wait_until(lambda: seen[-1] == {}, what='the release to be published')
    assert seen[-1] == {}


def test_a_simulator_reset_ends_the_hold(hold):
    state, seen = hold
    obj = np.eye(4)
    state.reached_point('crate_1/rails', 'crate_1', ['left', 'right'], obj)
    assert state.grasp(['left', 'right']) == 'crate_1'
    state._on_reset(None)  # what the clock's jump callback calls when sim time goes back
    assert state.held() is None
    wait_until(lambda: seen[-1] == {}, what='the reset to be published')
    assert seen[-1] == {}
    assert state.grasp(['left', 'right']) is None  # the reached point is gone too: no grasp without a new approach


def at_points(state, *points):
    """The hands at object points: points = (hand, point, instance)."""
    obj = np.eye(4)
    obj[:3, 3] = [0.6, 0.0, 0.0]
    for hand, point, instance in points:
        state.reach([hand], point, instance, obj)


def test_a_partial_close_is_a_hold_of_the_hands_that_closed(hold):
    state, seen = hold
    at_points(state, ('left', 'crate_1/rails', 'crate_1'), ('right', 'crate_1/rails', 'crate_1'))
    assert state.close_hands(['left']) == {'crate_1': {'hands': ['left'], 'new': True}}
    assert state.hand_holds == {'left': 'crate_1'} and state.held()['hands'] == ['left']
    assert state.held()['tools'] == ['left']                                        # the legacy view
    wait_until(lambda: seen[-1].get('tools') == ['left'], what='the partial hold to be published')
    assert seen[-1]['tools'] == ['left'] and seen[-1]['links'] == ['left_link7']
    assert seen[-1]['holds'] == [{'object': 'crate_1', 'point': 'crate_1/rails', 'hands': ['left'],
                                  'links': ['left_link7']}]
    # left holds already
    assert state.close_hands(['left', 'right']) == {'crate_1': {'hands': ['right'], 'new': False}}
    assert state.held()['hands'] == ['left', 'right'] and set(state.held()['tcp_in_object']) == {'left', 'right'}
    assert state.held()['tcp_in_object']['right'][:3, 3] == pytest.approx([0.0, -0.2, 0.0])


def test_hands_hold_different_objects_and_open_one_by_one(hold):
    state, seen = hold
    at_points(state, ('left', 'crate_1/rails', 'crate_1'), ('right', 'crate_2/rails', 'crate_2'))
    assert state.reached_hands() == {'left': {'point': 'crate_1/rails', 'instance': 'crate_1'},
                                     'right': {'point': 'crate_2/rails', 'instance': 'crate_2'}}
    assert state.close_hands(['left', 'right']).keys() == {'crate_1', 'crate_2'}
    assert state.hand_holds == {'left': 'crate_1', 'right': 'crate_2'} and len(state.held_all()) == 2
    assert state.hold_of('right')['object'] == 'crate_2' and state.hold_of('nobody') is None
    wait_until(lambda: len(seen[-1].get('holds', ())) == 2, what='both holds to be published')
    assert seen[-1]['object'] == 'crate_1' and [h['object'] for h in seen[-1]['holds']] == ['crate_1', 'crate_2']
    assert state.open_hands(['left']) == {'crate_1': {'hands': ['left'], 'released': True}}
    wait_until(lambda: seen[-1].get('object') == 'crate_2', what='the first hold to be released')
    assert seen[-1]['object'] == 'crate_2' and seen[-1]['tools'] == ['right']     # the first hold is the other one now
    assert state.open_hands(['left']) == {}                                       # nothing to let go of


def test_opening_one_of_two_hands_keeps_the_hold_the_old_release_ends_it(hold):
    state, seen = hold
    at_points(state, ('left', 'crate_1/rails', 'crate_1'), ('right', 'crate_1/rails', 'crate_1'))
    state.close_hands(['left', 'right'])
    assert state.open_hands(['right']) == {'crate_1': {'hands': ['right'], 'released': False}}
    assert state.hand_holds == {'left': 'crate_1'} and set(state.held()['tcp_in_object']) == {'left'}
    assert state.reached_hands() == {'left': {'point': 'crate_1/rails', 'instance': 'crate_1'}}
    assert state.open_hands(['left']) == {'crate_1': {'hands': ['left'], 'released': True}}
    assert state.held() is None and state.held_all() == []
    at_points(state, ('left', 'crate_1/rails', 'crate_1'), ('right', 'crate_1/rails', 'crate_1'))
    state.close_hands(['left', 'right'])
    assert state.release(['right']) == 'crate_1' and state.held() is None        # the old call: the hold ends whole


def test_leaving_a_point_and_the_old_reached_calls(hold):
    state, _ = hold
    at_points(state, ('left', 'crate_1/rails', 'crate_1'), ('right', 'crate_1/rails', 'crate_1'))
    state.leave(['left'])
    assert set(state.reached_hands()) == {'right'} and state.reached_at()['tools'] == ['right']
    state.reached_point('crate_2/rails', 'crate_2', ['left', 'right'], np.eye(4))   # the old call replaces all
    assert state.reached_at() == {'point': 'crate_2/rails', 'instance': 'crate_2', 'tools': ['left', 'right']}
    state.reached_point(None, None, (), None)
    assert state.reached_at() is None and state.reached_hands() == {}
    state.hold = {'object': 'box', 'point': 'box/top', 'tools': ['left'], 'object_in_base': None, 'tcp_in_object': {}}
    assert state.held()['hands'] == ['left'] and state.hand_holds == {'left': 'box'}      # the old attribute
    state.hold = None
    assert state.held() is None


def published(state):
    """Spy on what the hold state publishes: a list of payload dicts, in order."""
    payloads = []
    real = state.publisher.publish
    state.publisher.publish = lambda msg: (payloads.append(json.loads(msg.data)), real(msg))[1]
    return payloads


def test_opening_the_first_hand_of_a_hold_republishes_via_released(hold):
    """The sim's magic grasp stays on links[0] while attached: a changed first link needs {} first (review I1)."""
    state, _seen = hold
    at_points(state, ('left', 'crate_1/rails', 'crate_1'), ('right', 'crate_1/rails', 'crate_1'))
    state.close_hands(['left', 'right'])
    payloads = published(state)
    state.open_hands(['left'])
    assert payloads[0] == {}
    assert len(payloads) == 2 and payloads[1]['tools'] == ['right'] and payloads[1]['links'] == ['right_link7']


def test_an_unchanged_first_link_publishes_no_released_in_between(hold):
    state, _seen = hold
    at_points(state, ('left', 'crate_1/rails', 'crate_1'), ('right', 'crate_1/rails', 'crate_1'))
    state.close_hands(['left', 'right'])
    payloads = published(state)
    state.open_hands(['right'])
    assert len(payloads) == 1 and payloads[0]['tools'] == ['left']
