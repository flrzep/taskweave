"""clock_jump.watch: a backward jump of ROS time clears the node's TF buffers and calls the registered callbacks."""

import time
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from taskweave_core import clock_jump  # noqa: E402


class FakeBuffer:
    def __init__(self):
        self.cleared = 0

    def clear(self):
        self.cleared += 1


class FakeNode:
    """Just what the watcher needs from a node; ``jump`` plays what rclpy does when the clock goes back."""

    def __init__(self):
        self.handler = None
        self.timers = []
        self.logs = []
        self.logger = SimpleNamespace(warning=self.logs.append)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return SimpleNamespace(create_jump_callback=self._register)

    def _register(self, threshold, post_callback=None, **_):
        self.handler = post_callback
        return object()

    def create_timer(self, period, callback, clock=None):
        self.timers.append(callback)
        return object()

    def jump(self, seconds: float):
        self.handler(SimpleNamespace(delta=SimpleNamespace(nanoseconds=int(seconds * 1e9))))


def test_a_jump_back_clears_every_buffer_and_calls_the_callbacks():
    node = FakeNode()
    watcher = clock_jump.watch(node)
    first, second, calls = FakeBuffer(), FakeBuffer(), []
    watcher.add_buffer(first)
    watcher.add_buffer(second)
    watcher.callbacks.append(lambda: calls.append('a'))
    watcher.callbacks.append(lambda: calls.append('b'))
    node.jump(-40.0)
    assert (first.cleared, second.cleared, calls) == (1, 1, ['a', 'b'])
    assert node.logs and 'jumped' in node.logs[0]


def test_watch_is_one_per_node_and_a_failing_callback_does_not_stop_the_others():
    node = FakeNode()
    watcher = clock_jump.watch(node)
    assert clock_jump.watch(node) is watcher
    calls = []
    watcher.callbacks.append(lambda: 1 / 0)
    watcher.callbacks.append(lambda: calls.append('ok'))
    node.jump(-5.0)
    assert calls == ['ok']


def test_recent_tells_whether_a_jump_just_happened():
    node = FakeNode()
    watcher = clock_jump.watch(node)
    assert not watcher.recent(60.0)
    node.jump(-5.0)
    assert watcher.recent(60.0)
    assert not watcher.recent(0.0)


def test_the_buffers_are_cleared_again_while_the_clock_settles():
    node = FakeNode()
    watcher = clock_jump.watch(node, settle_s=60.0)
    buf = FakeBuffer()
    watcher.add_buffer(buf)
    node.timers[0]()  # no jump yet: nothing to clear
    assert buf.cleared == 0
    node.jump(-5.0)
    node.timers[0]()
    assert buf.cleared == 2
    watcher.settle_until = time.monotonic() - 1.0
    node.timers[0]()
    assert buf.cleared == 2
