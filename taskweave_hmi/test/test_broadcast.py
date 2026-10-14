"""Live events: a client that stops reading must not block the others (no ROS graph needed)."""

import queue
import threading

import pytest

pytest.importorskip('rclpy')

from taskweave_hmi.server import Bridge  # noqa: E402


def bridge_with(*listeners):
    bridge = Bridge.__new__(Bridge)  # only the event fan-out: no node
    bridge.lock, bridge.listeners = threading.Lock(), list(listeners)
    return bridge


def test_a_full_listener_is_dropped_and_told_to_end_its_stream():
    stalled, reading = queue.Queue(maxsize=2), queue.Queue(maxsize=10)
    bridge = bridge_with(stalled, reading)
    for n in range(3):  # the third message does not fit into the stalled client's queue
        bridge._broadcast('status', n)
    assert bridge.listeners == [reading] and reading.qsize() == 3
    assert stalled.get_nowait() is None and stalled.empty()  # the stream ends; EventSource reconnects
    assert bridge.lock.acquire(blocking=False)  # and nothing kept the lock
