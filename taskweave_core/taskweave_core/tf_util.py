"""TF buffer that survives a simulation restart.

When sim time jumps back (Stop/Play, simulator restart), a tf2 buffer still holds the old run's transforms with later
stamps: it then drops every new transform as TF_OLD_DATA and keeps answering with the stale pose (rclpy, unlike
rclcpp, does not clear on a jump). ``buffer(node)`` returns a Buffer and TransformListener whose buffer is cleared
whenever the node's clock goes back by more than ``jump_s``.
"""

from __future__ import annotations


def buffer(node, jump_s: float = 0.5):
    """(Buffer, TransformListener) for ``node``; keep both referenced as long as they are used."""
    from rclpy.clock import JumpThreshold
    from rclpy.duration import Duration
    from tf2_ros import Buffer, TransformListener

    tf = Buffer()
    listener = TransformListener(tf, node)

    def on_jump(jump) -> None:
        tf.clear()
        node.get_logger().warning(f'clock jumped by {jump.delta.nanoseconds * 1e-9:.1f} s: TF buffer cleared')

    threshold = JumpThreshold(min_forward=None, min_backward=Duration(seconds=-jump_s), on_clock_change=True)
    tf.jump_handle = node.get_clock().create_jump_callback(threshold, post_callback=on_jump)  # keep the handle alive
    return tf, listener
