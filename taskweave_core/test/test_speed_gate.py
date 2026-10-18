"""The cmd_vel speed gate: scaling by the override, exact at 100 %, stops and decreases at once, increases slewed."""

import time

import pytest
from conftest import RosRig, wait_until

rclpy = pytest.importorskip('rclpy')

from geometry_msgs.msg import Twist, TwistStamped  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.speed_gate import SpeedGate, is_zero, make_node  # noqa: E402
from taskweave_interfaces.msg import SpeedOverride  # noqa: E402


def twist(x=0.8, y=-0.2, wz=0.6):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = float(x), float(y), float(wz)
    return t


class Target:
    def __init__(self, value=1.0):
        self.value = value

    def __call__(self):
        return self.value


@pytest.mark.parametrize('override', [1.0, 0.5, 0.1])
def test_scales_linear_and_angular_together(override):
    gate = SpeedGate(Target(override))
    out = gate.process(twist(), 1.0)
    assert out.linear.x == pytest.approx(0.8 * override)
    assert out.linear.y == pytest.approx(-0.2 * override)
    assert out.angular.z == pytest.approx(0.6 * override)
    assert out.angular.z / out.linear.x == pytest.approx(0.6 / 0.8)       # curvature kept


def test_100_percent_passes_the_message_unchanged():
    gate = SpeedGate(Target(1.0))
    msg = twist(0.123456789, 0.0, -1e-9)
    assert gate.process(msg, 5.0) is msg
    assert (msg.linear.x, msg.angular.z) == (0.123456789, -1e-9)


def test_zero_passes_at_once_and_the_override_keeps_following():
    target = Target(0.1)
    gate = SpeedGate(target, slew=1.0)
    stop = twist(0, 0, 0)
    assert gate.process(stop, 1.0) is stop
    target.value = 1.0
    gate.process(stop, 1.5)                       # zero commands still advance the slew
    assert gate.factor == pytest.approx(0.6)


def test_decrease_applies_at_once_increase_slews():
    target = Target(1.0)
    gate = SpeedGate(target, slew=2.0)
    gate.process(twist(), 0.0)
    target.value = 0.1
    assert gate.process(twist(1.0, 0, 0), 0.02).linear.x == pytest.approx(0.1)
    target.value = 1.0
    out = gate.process(twist(1.0, 0, 0), 0.07)    # +2/s * 0.05 s = +0.1
    assert out.linear.x == pytest.approx(0.2)
    assert gate.process(twist(1.0, 0, 0), 0.07 + 0.2).linear.x == pytest.approx(0.6)
    assert gate.process(twist(1.0, 0, 0), 10.0).linear.x == 1.0


def test_stamped_keeps_the_header():
    gate = SpeedGate(Target(0.5))
    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    msg.twist = twist()
    out = gate.process(msg, 1.0, stamped=True)
    assert out.header.frame_id == 'base_link' and out.twist.linear.x == pytest.approx(0.4)
    assert msg.twist.linear.x == 0.8              # the input is not modified
    assert is_zero(TwistStamped().twist)


@pytest.fixture
def rig():
    with RosRig() as ros:
        ros.client = ros.add(rclpy.create_node('gate_test_client'))
        yield ros


def run_gate(rig, kind=Twist, **params):
    node = rig.add(make_node(**params))
    client = rig.client
    got = []
    pub_override = client.create_publisher(SpeedOverride, names.SPEED_OVERRIDE,
                                           QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client.create_subscription(kind, params['out_topic'], got.append, 10)
    pub = client.create_publisher(kind, params['in_topic'], 10)
    rig.spin(4)
    wait_until(lambda: pub.get_subscription_count() and node.count_publishers(params['out_topic']) is not None,
               what='the gate to be matched')

    def set_override(percent):
        pub_override.publish(SpeedOverride(percent=float(percent)))
        wait_until(lambda: abs(node.gate.clock.target() - percent / 100) < 1e-9, what='the override at the gate')

    def send(msg):
        n = len(got)
        deadline = time.monotonic() + 10
        while len(got) == n and time.monotonic() < deadline:
            pub.publish(msg)
            time.sleep(0.05)
        wait_until(lambda: len(got) > n, what='the gate output')
        return got[-1]
    return node, set_override, send


def test_node_without_an_override_passes_at_100(rig):
    _, _, send = run_gate(rig, in_topic='/gate_a/in', out_topic='/gate_a/out')
    out = send(twist())
    assert (out.linear.x, out.linear.y, out.angular.z) == (0.8, -0.2, 0.6)


def test_node_scales_at_50_and_10_percent_and_stops_at_once(rig):
    node, set_override, send = run_gate(rig, in_topic='/gate_b/in', out_topic='/gate_b/out')
    set_override(50)
    assert send(twist()).linear.x == pytest.approx(0.4)
    set_override(10)
    out = send(twist())
    assert out.linear.x == pytest.approx(0.08) and out.angular.z == pytest.approx(0.06)
    stop = send(twist(0, 0, 0))
    assert (stop.linear.x, stop.angular.z) == (0.0, 0.0)


def test_node_increase_is_slewed(rig):
    node, set_override, send = run_gate(rig, in_topic='/gate_c/in', out_topic='/gate_c/out', slew=0.05)
    set_override(10)
    send(twist())
    set_override(100)
    out = send(twist(1.0, 0, 0))
    assert 0.1 <= out.linear.x < 0.5                # slew 5 %/s: far from 100 % after a fraction of a second


def test_node_stamped(rig):
    _, set_override, send = run_gate(rig, kind=TwistStamped, in_topic='/gate_d/in', out_topic='/gate_d/out',
                                     stamped=True)
    set_override(50)
    msg = TwistStamped()
    msg.header.frame_id = 'base_link'
    msg.twist = twist()
    out = send(msg)
    assert out.header.frame_id == 'base_link' and out.twist.linear.x == pytest.approx(0.4)
