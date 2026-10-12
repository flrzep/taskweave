"""Pad modes of operator_joy: drive passthrough, tool modes that jog a TCP, missions force drive."""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from geometry_msgs.msg import TransformStamped, TwistStamped  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_interfaces.msg import TaskStatus  # noqa: E402

OPERATOR = """joy:
  topic: /joy
  drive_topic: /joy_drive
  drive_button: 8
  arms:
    left: {button: 13, twist: /taskweave/jog/left}
  jog: {enable_button: 4, speed: 0.1}
  actions:
    - {button: 0, skill: wait, args: {seconds: 0.1}, arm_args: {note: '{arm}'}}
"""
TOOL_FRAMES = "tool_frames:\n  left: {link: left_link7, xyz: [-0.28, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}\n"


@pytest.fixture
def pad(config_dir):
    from taskweave_core.operator_joy import OperatorJoy

    (config_dir / 'operator.yaml').write_text(OPERATOR)
    profile = config_dir / 'robots' / 'demo_bot' / 'profile.yaml'
    profile.write_text(profile.read_text() + TOOL_FRAMES)
    rclpy.init()
    operator = OperatorJoy(parameter_overrides=[Parameter('config_dir', value=str(config_dir)),
                                                Parameter('robot', value='demo_bot')])
    client = rclpy.create_node('pad_test')
    seen = {'drive': [], 'twist': [], 'mode': []}
    client.create_subscription(Joy, '/joy_drive', seen['drive'].append, 10)
    client.create_subscription(TwistStamped, '/taskweave/jog/left', seen['twist'].append, 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    client.create_subscription(String, names.OPERATOR_MODE, lambda m: seen['mode'].append(m.data), latched)
    status = client.create_publisher(TaskStatus, names.STATUS, latched)
    tf = StaticTransformBroadcaster(client)
    transform = TransformStamped(child_frame_id='left_link7')
    transform.header.frame_id = operator.base_frame
    transform.transform.rotation.w = 1.0
    tf.sendTransform(transform)
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (operator, client):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    joy = client.create_publisher(Joy, '/joy', 10)
    time.sleep(0.5)

    def send(buttons=(), axes=(0.0,) * 6):
        joy.publish(Joy(axes=list(axes), buttons=[1 if i in buttons else 0 for i in range(15)]))
        time.sleep(0.15)

    yield operator, send, seen, status
    executor.shutdown()
    for node in (operator, client):
        node.destroy_node()
    rclpy.shutdown()


def test_drive_mode_passes_the_pad_on(pad):
    operator, send, seen, _ = pad
    send(buttons=(4,), axes=(0.0, 1.0, 0, 0, 0, 0))
    assert operator.mode == 'drive' and seen['drive'] and not seen['twist']
    assert seen['mode'][-1] == 'drive'


def test_tool_mode_jogs_only_with_the_enable_button_and_blocks_the_base(pad):
    operator, send, seen, _ = pad
    send(buttons=(13,))
    send()
    assert operator.mode == 'left' and seen['mode'][-1] == 'left'
    drive_before = len(seen['drive'])
    send(axes=(0.0, 1.0, 0, 0, 0, 0))  # stick without enable: nothing moves
    assert not seen['twist']
    send(buttons=(4,), axes=(0.0, 1.0, 0, 0, 0, 0))  # enable + left stick up: forward
    twist = seen['twist'][-1]
    assert twist.header.frame_id == operator.base_frame and twist.twist.linear.x == pytest.approx(0.1)
    send()  # enable released: one stop command
    assert seen['twist'][-1].twist.linear.x == 0.0
    assert len(seen['drive']) == drive_before  # the base got nothing in the tool mode
    assert operator._for_mode(operator.actions[0])['args'] == {'seconds': 0.1, 'note': 'left'}
    send(buttons=(8,))
    assert operator.mode == 'drive'


def test_a_running_mission_puts_the_pad_back_to_drive_and_locks_modes(pad):
    operator, send, seen, status = pad
    send(buttons=(13,))
    send()
    assert operator.mode == 'left'
    status.publish(TaskStatus(state=TaskStatus.RUNNING))
    time.sleep(0.3)
    assert operator.mode == 'drive'
    send(buttons=(13,))
    assert operator.mode == 'drive'
