"""Live events: a client that stops reading must not block the others; numbered step events (no ROS graph needed)."""

import json
import queue
import threading
from collections import deque

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


def test_step_events_are_numbered_and_read_since_a_number():
    bridge = bridge_with()
    bridge.events, bridge.event_seq, bridge.outcomes = deque(maxlen=3), 0, {}
    bridge.robot, bridge.status, bridge.plan, bridge.pad_mode, bridge.recorder = 'r', {}, None, '', None
    for n in range(5):
        bridge._episode(type('Msg', (), {'data': json.dumps({'event': 'step_started', 'step_id': f's{n}'})}))
    state = bridge.snapshot()
    assert [e['seq'] for e in state['events']] == [3, 4, 5] and state['event_seq'] == 5 and state['events_from'] == 3
    assert [e['step_id'] for e in bridge.snapshot(since=4)['events']] == ['s4']
    assert bridge.snapshot(since=5)['events'] == []
