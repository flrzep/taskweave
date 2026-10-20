"""One reaction to a simulation clock jump (Stop/Play, simulator restart) per node.

When ROS time goes back, a tf2 buffer still holds the old run's transforms with later stamps: it drops every new
transform as TF_OLD_DATA and answers with the stale pose (rclpy, unlike rclcpp, does not clear on a jump). A node
calls ``watch(node)`` once (repeat calls return the same watcher), registers its TF buffers with ``add_buffer`` and
anything else that must forget the old run in ``callbacks``. The buffers are cleared at the jump and again every
``RECLEAR_S`` for ``settle_s``: transforms of the old run still in flight arrive after the jump and would otherwise
be the latest ones (seen 14 Oct: a solve from the robot's pose before the reset, 7 m off).
"""

from __future__ import annotations

import time

RECLEAR_S = 0.2


class ClockJump:
    def __init__(self, node, jump_s: float = 0.5, settle_s: float = 2.0):
        from rclpy.clock import Clock, ClockType, JumpThreshold
        from rclpy.duration import Duration

        self.node, self.settle_s = node, settle_s
        self.buffers: list = []
        self.callbacks: list = []              # called with no arguments after the buffers are cleared
        self.last_jump: float | None = None    # monotonic time of the last jump
        self.settle_until = 0.0
        threshold = JumpThreshold(min_forward=None, min_backward=Duration(seconds=-jump_s), on_clock_change=True)
        self.jump_handle = node.get_clock().create_jump_callback(threshold, post_callback=self._on_jump)  # keep alive
        # on the wall clock: the sim clock may stand still right after a reset
        self.timer = node.create_timer(RECLEAR_S, self._reclear, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def add_buffer(self, buffer) -> None:
        self.buffers.append(buffer)

    def recent(self, within_s: float) -> bool:
        """True when a jump happened less than ``within_s`` wall seconds ago."""
        return self.last_jump is not None and time.monotonic() - self.last_jump < within_s

    def _on_jump(self, jump) -> None:
        self.handle(jump.delta.nanoseconds * 1e-9)

    def handle(self, delta_s: float) -> None:
        self.last_jump = time.monotonic()
        self.settle_until = self.last_jump + self.settle_s
        self._clear()
        self.node.get_logger().warning(f'clock jumped by {delta_s:.1f} s: TF buffers cleared')
        for callback in list(self.callbacks):
            try:
                callback()
            except Exception as error:  # noqa: BLE001 - one hook must not stop the others
                self.node.get_logger().warning(f'clock jump hook failed: {error}')

    def _clear(self) -> None:
        for buffer in self.buffers:
            buffer.clear()

    def _reclear(self) -> None:
        if time.monotonic() < self.settle_until:
            self._clear()


def watch(node, jump_s: float = 0.5, settle_s: float = 2.0) -> ClockJump:
    """The node's ClockJump, made on the first call (later arguments are ignored)."""
    watcher = getattr(node, 'clock_jump', None)
    if not isinstance(watcher, ClockJump):
        watcher = node.clock_jump = ClockJump(node, jump_s, settle_s)
    return watcher
