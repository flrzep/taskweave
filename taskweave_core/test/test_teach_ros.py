"""Teaching object points: the pad's teach button -> teach node -> tool TCP from TF -> objects/<object>.yaml."""

import time

import pytest
from conftest import RosRig, wait_until

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from taskweave_core.config import ConfigStore  # noqa: E402

OPERATOR = """joy:
  topic: /joy
  drive_button: 8
  arms:
    left: {button: 13, twist: /taskweave/jog/left}
  teach: {button: 10, point: pile/top_grasp}
"""
# TCP 0.1 m along the flange's x axis; the flange points straight down (rotated pi about y)
TOOL_FRAMES = "tool_frames:\n  left: {link: left_link7, xyz: [0.1, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}\n"


@pytest.fixture
def system(config_dir):
    with RosRig() as ros:
        from taskweave_core.operator_joy import OperatorJoy
        from taskweave_core.teach import Teach

        (config_dir / 'operator.yaml').write_text(OPERATOR)
        (config_dir / 'cell' / 'objects.yaml').write_text(
            'pile: {object: box, frame: cell, x: 1.0, y: 2.0, z: 0.1, yaw: 0.0, stack: {count: 3, pitch: 0.05}}\n')
        profile = config_dir / 'robots' / 'demo_bot' / 'profile.yaml'
        profile.write_text(profile.read_text() + TOOL_FRAMES)
        parameters = [Parameter('config_dir', value=str(config_dir)), Parameter('robot', value='demo_bot')]
        nodes = [ros.add(OperatorJoy(parameter_overrides=parameters)), ros.add(Teach(parameter_overrides=parameters)),
                 ros.add(rclpy.create_node('teach_test'))]
        flange = TransformStamped(child_frame_id='left_link7')
        flange.header.frame_id = 'world'
        flange.transform.translation.x, flange.transform.translation.y, flange.transform.translation.z = 1.2, 2.0, 0.6
        flange.transform.rotation.y, flange.transform.rotation.w = 1.0, 0.0  # pi about y: flange x -> cell -x
        StaticTransformBroadcaster(nodes[2]).sendTransform(flange)
        ros.spin(4)
        joy = nodes[2].create_publisher(Joy, '/joy', 10)
        wait_until(lambda: joy.get_subscription_count() > 0, what='the operator node to subscribe to /joy')
        wait_until(lambda: nodes[0].teach_client.service_is_ready(), what='the teach service')

        def send(buttons=()):
            joy.publish(Joy(axes=[0.0] * 6, buttons=[1 if i in buttons else 0 for i in range(15)]))
            time.sleep(0.15)

        yield nodes[0], send, ConfigStore(config_dir)


def test_the_teach_button_stores_the_jogged_tool_relative_to_the_object(system):
    operator, send, store = system

    def left_mode():  # a press can be lost while the nodes are still finding each other: press until it counts
        send(buttons=(13,))  # left arm mode: teach only the left tool
        send()
        return operator.mode == 'left'

    wait_until(left_mode, what="the pad's left arm mode", interval=0.0)
    send(buttons=(10,))  # without L1 held: ignored, so a stray press stores nothing
    time.sleep(0.5)
    store.reload()
    assert store.object_point('box', 'top_grasp')['left_tool'].xyz == (0.0, 0.1, 0.2)  # unchanged

    def taught():  # the TF buffers may still be filling: press again until the point is stored
        send(buttons=(4,))
        send(buttons=(4, 10))
        send()
        store.reload()
        return store.object_point('box', 'top_grasp')['left_tool'].xyz != (0.0, 0.1, 0.2)  # the hand-written value

    wait_until(taught, what='the taught point in objects/box.yaml', interval=0.0)
    point = store.object_point('box', 'top_grasp')
    # TCP in the cell: flange (1.2, 2.0, 0.6) + 0.1 along cell -x = (1.1, 2.0, 0.6); top item at z 0.2
    assert point['left_tool'].xyz == pytest.approx((0.1, 0.0, 0.4), abs=1e-4)
    assert 'taught' in (store.root / 'objects' / 'box.yaml').read_text()
