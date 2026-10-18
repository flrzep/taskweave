"""cmd_vel speed gate: scales a velocity command stream by the global speed override (skill API spec 4.3).

A planner that drives the base by itself (Nav2's MPPI) has its own speed caps that rarely bind (a cap at 50 % of
``vx_max`` 2 m/s does nothing to a drive averaging 0.8 m/s). The gate sits between it and the base and multiplies
``linear.x/y/z`` and ``angular.x/y/z`` by the same factor, so the path curvature is kept and the whole motion takes
1/override as long.

- The factor is the *effective* override of ``speed.WorkClock``: a decrease applies at once, an increase slews
  (``slew``, fraction/s).
- At an effective 100 % the incoming message is forwarded unchanged (the same object, no arithmetic).
- A zero command is forwarded at once and unscaled (a stop is never delayed).
- No override received yet: 100 %.

Parameters: ``in_topic`` (default ``cmd_vel_nav``), ``out_topic`` (default ``cmd_vel``), ``stamped`` (false:
geometry_msgs/Twist, true: TwistStamped, both topics), ``slew`` (fraction/s, default ``speed.DEFAULT_SLEW_PER_S``).
Run: ``ros2 run taskweave_core speed_gate --ros-args -p in_topic:=/cmd_vel_nav -p out_topic:=/base/cmd_vel``.

Nav2's progress checker (SimpleProgressChecker: ``required_movement_radius`` in ``movement_time_allowance``) counts
base *position* only, so a slowed drive can trip it, a turn in place most easily; see the note in the nav2 launch.
"""

from __future__ import annotations

from .speed import DEFAULT_SLEW_PER_S, SpeedOverride, WorkClock

_FIELDS = ('x', 'y', 'z')


def is_zero(twist) -> bool:
    return not any(getattr(twist.linear, f) or getattr(twist.angular, f) for f in _FIELDS)


def scale_twist(twist, factor: float):
    """A copy of ``twist`` (Twist) with every component multiplied by ``factor``."""
    out = type(twist)()
    for f in _FIELDS:
        setattr(out.linear, f, getattr(twist.linear, f) * factor)
        setattr(out.angular, f, getattr(twist.angular, f) * factor)
    return out


class SpeedGate:
    """The gate's rule, without ROS: ``process(twist, now)`` returns what to publish."""

    def __init__(self, target, start: float = 0.0, slew: float = DEFAULT_SLEW_PER_S):
        self.clock = WorkClock(start, target, slew)

    @property
    def factor(self) -> float:
        return self.clock.o

    def process(self, twist, now: float, stamped: bool = False):
        self.clock(now)                 # the effective override follows the target also while a zero passes
        inner = twist.twist if stamped else twist
        if self.clock.o >= 1.0 or is_zero(inner):
            return twist
        if not stamped:
            return scale_twist(twist, self.clock.o)
        out = type(twist)()
        out.header = twist.header
        out.twist = scale_twist(twist.twist, self.clock.o)
        return out


def make_node(**overrides):
    """The gate as a node (``node_name`` / parameter overrides for tests)."""
    from geometry_msgs.msg import Twist, TwistStamped
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    node = Node('speed_gate', parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
    node.declare_parameter('in_topic', 'cmd_vel_nav')
    node.declare_parameter('out_topic', 'cmd_vel')
    node.declare_parameter('stamped', False)
    node.declare_parameter('slew', DEFAULT_SLEW_PER_S)
    stamped = bool(node.get_parameter('stamped').value)
    seconds = lambda: node.get_clock().now().nanoseconds * 1e-9  # noqa: E731
    node.gate = SpeedGate(SpeedOverride.for_node(node).fraction, seconds(), float(node.get_parameter('slew').value))
    kind = TwistStamped if stamped else Twist
    out = node.create_publisher(kind, node.get_parameter('out_topic').value, 10)
    node.create_subscription(kind, node.get_parameter('in_topic').value,
                             lambda msg: out.publish(node.gate.process(msg, seconds(), stamped)), 10)
    return node


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = make_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
