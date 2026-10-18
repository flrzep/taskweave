"""Bridge pieces without a ROS graph: live events (a client that stops reading must not block the others, numbered
step events), manual skill waits, which skill a pose test runs, the argument picker lists."""

import json
import queue
import threading
from collections import deque

import pytest

pytest.importorskip('rclpy')

from taskweave_hmi.server import Bridge, await_skill  # noqa: E402


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


def test_a_manual_skill_waits_on_work_time_and_is_given_up_when_its_server_is_gone():
    t = {'now': 0.0}

    def sleep(dt):
        t['now'] += dt

    # at 10 % a 30 s skill takes 300 s: its work time stays below the limit, so the wait goes on
    finished = lambda: t['now'] >= 300.0  # noqa: E731
    assert await_skill(finished, lambda: t['now'] * 0.1, 35.0, lambda: True, sleep) == ''
    t['now'] = 0.0
    assert await_skill(lambda: False, lambda: t['now'] * 0.1, 35.0, lambda: True, sleep) == 'did not finish in 35 s'
    assert 350.0 < t['now'] < 351.0
    assert await_skill(lambda: False, lambda: 0.0, 35.0, lambda: False, sleep) == 'skill server gone'


class FakeStore:
    def named_pose(self, _robot, name):
        poses = {'home': {'joints': {'j1': 0.0}, 'tools': {}}, 'reach': {'joints': {}, 'tools': {'left': {}}}}
        if name not in poses:
            raise KeyError(f'unknown pose {name!r}')
        return poses[name]

    def point_names(self, _robot):
        return {'joint poses': ['home', 'reach'], 'placed object points': ['bin_a/grasp']}

    def held_poses(self, _robot):
        return {'frame': {'up': {}, 'carry': {}}, 'box': {'up': {}}}


def test_a_pose_test_runs_move_joints_or_move_gripper_and_pickers_list_held_poses_and_via_targets():
    bridge = bridge_with()
    bridge.config, bridge.robot, runs = FakeStore(), 'r', []
    bridge.run_skill = lambda skill, args: runs.append((skill, args)) or {'outcome': 'succeeded'}
    bridge.go_to_pose('home')
    bridge.go_to_pose('reach')
    bridge.test_position('frame', 'grasp', {'at': 'bin_a'})
    assert runs == [('move_joints', {'pose': 'home'}), ('move_gripper', {'to': {'pose': 'reach'}}),
                    ('move_gripper', {'to': {'point': 'bin_a/grasp'}})]
    with pytest.raises(Exception, match='unknown pose'):
        bridge.go_to_pose('nowhere')
    names = bridge.point_names()
    assert names['held poses'] == ['carry', 'up']
    assert names['via targets'] == ['pose:home', 'pose:reach', 'bin_a/grasp']
