"""TF buffer that survives a simulation restart.

When sim time jumps back (Stop/Play, simulator restart), a tf2 buffer still holds the old run's transforms with later
stamps: it then drops every new transform as TF_OLD_DATA and keeps answering with the stale pose (rclpy, unlike
rclcpp, does not clear on a jump). ``buffer(node)`` returns a Buffer and TransformListener whose buffer is cleared
whenever the node's clock goes back by more than ``jump_s``, and again every ``RECLEAR_S`` for ``settle_s`` after
it: transforms of the old run still in flight arrive after the jump with the old run's later stamps and would
otherwise be the latest transform until the new run's time passes them (seen 14 Oct: go_to_point solved from the
robot's pose before the reset, 7 m off). The simulator republishes the robot's transforms many times a second.
"""

from __future__ import annotations

RECLEAR_S = 0.2


def buffer(node, jump_s: float = 0.5, settle_s: float = 2.0):
    """(Buffer, TransformListener) for ``node``; keep both referenced as long as they are used."""
    import time

    from rclpy.clock import Clock, ClockType, JumpThreshold
    from rclpy.duration import Duration
    from tf2_ros import Buffer, TransformListener

    tf = Buffer()
    listener = TransformListener(tf, node)
    settle = {'until': 0.0}

    def on_jump(jump) -> None:
        tf.clear()
        settle['until'] = time.monotonic() + settle_s
        node.get_logger().warning(f'clock jumped by {jump.delta.nanoseconds * 1e-9:.1f} s: TF buffer cleared')

    def reclear() -> None:  # on the wall clock: the sim clock may stand still right after a reset
        if time.monotonic() < settle['until']:
            tf.clear()

    threshold = JumpThreshold(min_forward=None, min_backward=Duration(seconds=-jump_s), on_clock_change=True)
    tf.jump_handle = node.get_clock().create_jump_callback(threshold, post_callback=on_jump)  # keep the handle alive
    tf.reclear_timer = node.create_timer(RECLEAR_S, reclear, clock=Clock(clock_type=ClockType.STEADY_TIME))
    return tf, listener
