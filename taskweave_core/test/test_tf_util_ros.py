"""tf_util.buffer: a sim-time jump back clears the TF buffer, so new transforms are not dropped as old data."""

import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from builtin_interfaces.msg import Time  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402

from taskweave_core import tf_util  # noqa: E402


def transform(sec: int, x: float) -> TransformStamped:
    t = TransformStamped(child_frame_id='base_link')
    t.header.frame_id, t.header.stamp = 'world', Time(sec=sec)
    t.transform.translation.x, t.transform.rotation.w = x, 1.0
    return t


def test_a_clock_jump_back_clears_the_buffer():
    rclpy.init()
    try:
        node = rclpy.create_node('tf_util_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        clock = node.create_publisher(Clock, '/clock', 10)
        tf, _listener = tf_util.buffer(node)

        def tick(sec: int) -> None:
            clock.publish(Clock(clock=Time(sec=sec)))
            deadline = time.time() + 2.0
            while time.time() < deadline and node.get_clock().now().nanoseconds // 10**9 != sec:
                rclpy.spin_once(node, timeout_sec=0.05)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.02)

        tick(100)
        tf.set_transform(transform(100, 5.0), 'test')
        assert tf.lookup_transform('world', 'base_link', rclpy.time.Time()).transform.translation.x == 5.0
        tick(3)  # simulator restarted
        tf.set_transform(transform(3, 1.0), 'test')  # dropped as old data without the clear
        assert tf.lookup_transform('world', 'base_link', rclpy.time.Time()).transform.translation.x == 1.0
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_late_transform_of_the_old_run_is_cleared_too():
    rclpy.init()
    try:
        node = rclpy.create_node('tf_util_late', parameter_overrides=[Parameter('use_sim_time', value=True)])
        clock = node.create_publisher(Clock, '/clock', 10)
        tf, _listener = tf_util.buffer(node, settle_s=1.0)

        def spin(seconds: float) -> None:
            deadline = time.time() + seconds
            while time.time() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)

        clock.publish(Clock(clock=Time(sec=100)))
        spin(0.5)
        clock.publish(Clock(clock=Time(sec=3)))  # simulator restarted
        spin(0.3)
        tf.set_transform(transform(99, 5.0), 'test')  # the old run's, still in flight
        spin(0.4)  # the re-clear drops it
        tf.set_transform(transform(3, 1.0), 'test')
        assert tf.lookup_transform('world', 'base_link', rclpy.time.Time()).transform.translation.x == 1.0
        spin(1.0)  # settled: transforms stay again
        tf.set_transform(transform(4, 2.0), 'test')
        spin(0.4)
        assert tf.lookup_transform('world', 'base_link', rclpy.time.Time()).transform.translation.x == 2.0
        node.destroy_node()
    finally:
        rclpy.shutdown()
