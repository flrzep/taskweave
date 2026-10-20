"""Abort for manual skills (started from the HMI outside a mission) without a ROS graph: a fake skill action server
whose goals finish when told to or when canceled, run_skill in a thread as an HTTP request runs it, and
POST /api/skill/abort over HTTP."""

import threading
import time
from collections import deque
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')

from test_speed import FakeLogger, serve  # noqa: E402, F401 - serve: the fixture

from taskweave_hmi.server import OUTCOMES, Bridge  # noqa: E402


class Future:
    """An rclpy future, done when ``ready`` is set."""

    def __init__(self, value=None):
        self.value, self.ready = value, threading.Event()

    def done(self):
        return self.ready.is_set()

    def result(self):
        return self.value

    def finish(self, value=None):
        if value is not None:
            self.value = value
        self.ready.set()


class FakeHandle:
    """A goal handle: its result comes when the skill finishes or, on cancel, as CANCELED (as the skill server does)."""

    accepted = True

    def __init__(self):
        self.result_future, self.cancels = Future(), 0

    def get_result_async(self):
        return self.result_future

    def end(self, outcome, message):
        self.result_future.finish(SimpleNamespace(result=SimpleNamespace(outcome=OUTCOMES.index(outcome),
                                                                        message=message)))

    def cancel_goal_async(self):
        self.cancels += 1
        self.end('canceled', 'canceled')
        return Future()


class FakeAction:
    """A RunSkill action client. ``found`` gates wait_for_server, ``accept`` the goal's acceptance."""

    def __init__(self):
        self.found, self.accept, self.goals = threading.Event(), threading.Event(), []
        self.found.set()
        self.accept.set()
        self.handle = FakeHandle()

    def wait_for_server(self, timeout_sec):
        return self.found.wait(timeout_sec)

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        sent = Future(self.handle)
        threading.Thread(target=lambda: self.accept.wait(10) and sent.finish(), daemon=True).start()
        return sent


def manual_bridge(action):
    """A Bridge with only what run_skill, abort_skill and the snapshot use; events are recorded in ``sent``."""
    bridge = Bridge.__new__(Bridge)  # no node
    bridge.lock, bridge.listeners, bridge.sent = threading.Lock(), [], []
    bridge.skill_lock = threading.Lock()
    bridge.manual, bridge.manual_handle, bridge.manual_abort = None, None, False
    bridge.skill_clients = {'move_joints': action, 'navigate_to': action}
    bridge.check_idle = lambda: None
    bridge.speed = SimpleNamespace(clock=lambda _start: lambda _now: 0.0)  # work time stands still: no timeout
    bridge.now_s = lambda: 0.0
    logger = FakeLogger()
    bridge.get_logger = lambda: logger
    bridge._broadcast = lambda kind, data: bridge.sent.append((kind, data))
    bridge.events, bridge.event_seq, bridge.outcomes = deque(maxlen=10), 0, {}
    bridge.robot, bridge.status, bridge.plan, bridge.pad_mode, bridge.speed_state = 'r', {}, None, 'drive', {}
    return bridge


def run_in_thread(bridge, *args, **kwargs):
    """run_skill as a request thread runs it; the returned dict gets 'result' (or 'error') when it ends."""
    out = {}

    def run():
        try:
            out['result'] = bridge.run_skill(*args, **kwargs)
        except Exception as error:  # reported to the test
            out['error'] = error
    out['thread'] = threading.Thread(target=run, daemon=True)
    out['thread'].start()
    return out


def wait_for(condition, what):
    deadline = time.monotonic() + 5.0
    while not condition():
        assert time.monotonic() < deadline, f'timed out waiting for {what}'
        time.sleep(0.01)


def finished(out):
    out['thread'].join(5.0)
    assert not out['thread'].is_alive(), 'run_skill did not return'
    assert 'error' not in out, out.get('error')
    return out['result']


def test_abort_cancels_the_running_goal_and_its_request_answers_canceled_aborted():
    action = FakeAction()
    bridge = manual_bridge(action)
    out = run_in_thread(bridge, 'move_joints', {'pose': 'home'}, label='home')
    wait_for(lambda: bridge.manual_handle is not None, 'the goal handle')
    shown = bridge.snapshot()['manual']
    assert shown['skill'] == 'move_joints' and shown['label'] == 'home' and shown['elapsed_s'] >= 0.0
    assert bridge.abort_skill() == {'ok': True, 'skill': 'move_joints'}
    assert finished(out) == {'outcome': 'canceled', 'message': 'canceled', 'aborted': True}
    assert action.handle.cancels == 1
    assert bridge.get_logger().lines == ['HMI aborted manual skill move_joints home']
    # the browser saw it start and end; afterwards nothing runs and the next one starts clean
    assert bridge.sent[0][0] == 'manual' and bridge.sent[0][1]['label'] == 'home'
    assert bridge.sent[-1] == ('manual', None)
    assert bridge.manual is None and bridge.snapshot()['manual'] is None and not bridge.manual_abort
    assert bridge.abort_skill() is None


def test_a_skill_that_ends_by_itself_is_not_aborted_and_clears_the_bar():
    action = FakeAction()
    bridge = manual_bridge(action)
    out = run_in_thread(bridge, 'move_joints', {'pose': 'home'})
    wait_for(lambda: bridge.manual_handle is not None, 'the goal handle')
    assert bridge.manual['label'] == 'pose=home'  # no label given: the args
    action.handle.end('succeeded', 'at home')
    assert finished(out) == {'outcome': 'succeeded', 'message': 'at home', 'aborted': False}
    assert action.handle.cancels == 0 and [kind for kind, _ in bridge.sent] == ['manual', 'manual']
    assert bridge.sent[-1] == ('manual', None) and bridge.snapshot()['manual'] is None


def test_abort_before_the_goal_is_accepted_cancels_it_as_soon_as_it_is():
    action = FakeAction()
    action.accept.clear()  # the skill server has not answered the goal yet
    bridge = manual_bridge(action)
    out = run_in_thread(bridge, 'move_joints', {'pose': 'home'}, label='home')
    wait_for(lambda: action.goals, 'the goal to be sent')
    assert bridge.manual_handle is None
    assert bridge.abort_skill() == {'ok': True, 'skill': 'move_joints'}
    assert action.handle.cancels == 0  # nothing to cancel yet
    action.accept.set()
    assert finished(out) == {'outcome': 'canceled', 'message': 'canceled', 'aborted': True}
    assert action.handle.cancels == 1
    assert bridge.get_logger().lines == ['HMI aborted manual skill move_joints home (before its goal was accepted)']


def test_abort_while_the_skill_server_is_looked_for_sends_nothing():
    action = FakeAction()
    action.found.clear()
    bridge = manual_bridge(action)
    out = run_in_thread(bridge, 'move_joints', {'pose': 'home'})
    wait_for(lambda: bridge.manual is not None, 'the manual skill')
    bridge.abort_skill()
    action.found.set()
    assert finished(out) == {'outcome': 'canceled', 'message': 'aborted before it started', 'aborted': True}
    assert action.goals == [] and bridge.sent[-1] == ('manual', None)


def test_post_skill_abort_over_http_ends_the_drive_request_with_aborted(serve):  # noqa: F811
    action = FakeAction()
    bridge = manual_bridge(action)
    request = serve(bridge)
    status, data = request('POST', '/api/skill/abort', {})
    assert status == 409 and data == {'ok': False, 'message': 'no manual skill is running'}
    drive = {}
    thread = threading.Thread(target=lambda: drive.update(answer=request('POST', '/api/points/drive',
                                                                          {'target': 'dock'})), daemon=True)
    thread.start()
    wait_for(lambda: bridge.manual_handle is not None, 'the drive goal')
    status, state = request('GET', '/api/state')
    assert state['manual']['skill'] == 'navigate_to' and state['manual']['label'] == 'dock'
    assert request('POST', '/api/skill/abort', {}) == (200, {'ok': True, 'skill': 'navigate_to'})
    thread.join(5.0)
    assert drive['answer'] == (200, {'outcome': 'canceled', 'message': 'canceled', 'aborted': True})
    assert request('GET', '/api/state')[1]['manual'] is None
    assert request('POST', '/api/skill/abort', {})[0] == 409
