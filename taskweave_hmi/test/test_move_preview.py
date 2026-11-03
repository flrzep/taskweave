"""A move step's preview (POST /api/skills/move/dry_run) without a ROS graph: the args it sends (dry_run on, never a
placeholder), the joint goal per group in deg / m, and the route against a fake move action: the skill's answer comes
back, it is not a manual skill (no strip, nothing to abort), and it is refused while a mission or a manual skill
runs."""

import json
import shutil
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import targets

FIXTURES = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures'
ROBOT = 'bot_lift'   # torso [lift_joint (linear), pitch_joint], left_arm, right_arm


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURES / 'targets_new', tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


# ---- the args and the answer (no ROS) ------------------------------------------------------------------------------
def test_the_preview_sends_the_steps_args_with_dry_run():
    args = {'to': 'tray/place', 'motion': {'speed': 0.5}}
    assert targets.dry_run_args(args) == {'to': 'tray/place', 'motion': {'speed': 0.5}, 'dry_run': True}
    assert 'dry_run' not in args                                      # the step's own args are left as they are
    assert targets.dry_run_args({'to': 'x', 'dry_run': False})['dry_run'] is True


@pytest.mark.parametrize('args', [{'to': '{next_stack}/grasp'}, {'hand': 'left', 'at': {'onto': '{spot}'}},
                                  {'joints': {'left_arm': ['{a}', 0]}}])
def test_a_placeholder_cannot_be_previewed(args):
    with pytest.raises(ValueError, match='filled at run time'):
        targets.dry_run_args(args)


@pytest.mark.parametrize('args', [None, [], 'to home'])
def test_args_must_be_an_object(args):
    with pytest.raises(ValueError, match='arguments as an object'):
        targets.dry_run_args(args)


def test_the_joint_goal_is_shown_per_group_in_deg_and_m(store):
    goal = {'left_arm_joint2': -0.5, 'lift_joint': 0.31234, 'pitch_joint': 0.1, 'left_arm_joint1': 1.5707963,
            'head_joint': 0.2}
    view = targets.joint_goal_view(store, ROBOT, goal)
    assert view == [
        {'group': 'torso', 'joints': [{'name': 'lift_joint', 'value': 0.312, 'unit': 'm'},
                                      {'name': 'pitch_joint', 'value': 5.7, 'unit': 'deg'}]},
        {'group': 'left_arm', 'joints': [{'name': 'left_arm_joint1', 'value': 90.0, 'unit': 'deg'},
                                         {'name': 'left_arm_joint2', 'value': -28.6, 'unit': 'deg'}]},
        {'group': 'head_joint', 'joints': [{'name': 'head_joint', 'value': 11.5, 'unit': 'deg'}]}]   # in no group


def test_the_answer_keeps_the_skills_data_and_adds_the_goal(store):
    data = {'dry_run': True, 'kind': 'joints', 'joint_goal': {'lift_joint': 0.2}, 'legs': []}
    answer = targets.dry_run_answer(store, ROBOT, 'succeeded', 'move to joint positions · home: resolved', data)
    assert answer['outcome'] == 'succeeded' and answer['data'] == data
    assert answer['goal'] == [{'group': 'torso', 'joints': [{'name': 'lift_joint', 'value': 0.2, 'unit': 'm'}]}]
    assert answer['torso'] is None
    base = targets.dry_run_answer(store, ROBOT, 'succeeded', 'navigate to · dock: resolved (nothing moved)',
                                  {'kind': 'navigate', 'pose': {'x': 1, 'y': 0, 'yaw': 0}, 'joint_goal': {},
                                   'torso': {'lift_joint': 0.4}})
    assert base['goal'] == [] and base['data']['pose'] == {'x': 1, 'y': 0, 'yaw': 0}
    assert base['torso'] == [{'group': 'torso', 'joints': [{'name': 'lift_joint', 'value': 0.4, 'unit': 'm'}]}]
    keep = targets.dry_run_answer(store, ROBOT, 'succeeded', 'move straight · by +0.3 m: resolved (nothing moved)',
                                  {'kind': 'straight', 'by': [0.3, 0.0], 'torso': 'keep', 'joint_goal': {}})
    assert keep['torso'] is None and keep['data']['torso'] == 'keep'
    refused = targets.dry_run_answer(store, ROBOT, 'rejected', "move to target: 'nowhere' is not a target", {})
    assert refused == {'outcome': 'rejected', 'message': "move to target: 'nowhere' is not a target", 'data': {},
                       'goal': [], 'torso': None}


# ---- the route, against a fake move action --------------------------------------------------------------------------
pytest.importorskip('rclpy')

from test_manual_abort import FakeAction, manual_bridge, run_in_thread, wait_for  # noqa: E402
from test_speed import serve  # noqa: E402, F401 - the fixture

from taskweave_hmi.server import OUTCOMES, Busy  # noqa: E402


class AnsweringAction(FakeAction):
    """The move action: every goal is answered at once with ``answer`` (outcome, message, data)."""

    def __init__(self, outcome='succeeded', message='', data=None):
        super().__init__()
        self.answer = (outcome, message, data or {})

    def send_goal_async(self, goal):
        sent = super().send_goal_async(goal)
        outcome, message, data = self.answer
        self.handle.result_future.finish(SimpleNamespace(result=SimpleNamespace(
            outcome=OUTCOMES.index(outcome), message=message, data_json=json.dumps(data))))
        return sent


def preview_bridge(store, action, state='idle'):
    bridge = manual_bridge(action)
    bridge.skill_clients = {'move': action}
    bridge.config, bridge.robot = store, ROBOT
    bridge.status = {'state': state}
    del bridge.check_idle   # the real one: refused while a mission runs or is paused
    return bridge


def test_a_preview_answers_the_skills_result_and_is_not_a_manual_skill(store, serve):  # noqa: F811
    data = {'dry_run': True, 'kind': 'hand', 'module': 'move.hand', 'target': 'tray/place', 'hands': ['left'],
            'joint_goal': {'left_arm_joint1': 0.0, 'lift_joint': 0.25},
            'legs': [{'label': 'tray/place (approach)', 'joint_goal': {'left_arm_joint1': 0.1}},
                     {'label': 'tray/place', 'joint_goal': {'left_arm_joint1': 0.0, 'lift_joint': 0.25}}]}
    action = AnsweringAction('succeeded', 'move to target · tray/place (left): tray/place: IK solved (nothing moved)',
                             data)
    bridge = preview_bridge(store, action)
    request = serve(bridge)
    status, answer = request('POST', '/api/skills/move/dry_run', {'args': {'to': 'tray/place', 'hands': ['left']}})
    assert status == 200 and answer['outcome'] == 'succeeded' and answer['data'] == data
    assert answer['goal'] == [{'group': 'torso', 'joints': [{'name': 'lift_joint', 'value': 0.25, 'unit': 'm'}]},
                              {'group': 'left_arm', 'joints': [{'name': 'left_arm_joint1', 'value': 0.0,
                                                                'unit': 'deg'}]}]
    goal = action.goals[0]
    assert json.loads(goal.args_json) == {'to': 'tray/place', 'hands': ['left'], 'dry_run': True}
    assert goal.skill == 'move' and goal.step_id == 'hmi-preview'
    assert bridge.sent == [] and bridge.manual is None    # no manual strip: nothing to abort
    assert not bridge.skill_lock.locked()


def test_a_refused_or_failed_preview_answers_200_with_the_skills_message(store, serve):  # noqa: F811
    action = AnsweringAction('failed', 'move to target · tray/place (left): tray/place: no IK solution',
                             {'kind': 'hand'})
    request = serve(preview_bridge(store, action))
    status, answer = request('POST', '/api/skills/move/dry_run', {'args': {'to': 'tray/place'}})
    assert status == 200 and answer['outcome'] == 'failed' and 'no IK solution' in answer['message']
    assert answer['goal'] == []


@pytest.mark.parametrize('state', ['running', 'paused'])
def test_no_preview_while_a_mission_runs(store, serve, state):  # noqa: F811
    action = AnsweringAction()
    request = serve(preview_bridge(store, action, state))
    status, answer = request('POST', '/api/skills/move/dry_run', {'args': {'to': 'home'}})
    assert status == 409 and answer['problems'] == [f'a mission is {state}; preview a move when it has finished or '
                                                    'was aborted']
    assert action.goals == []


def test_no_preview_while_a_manual_skill_runs_and_no_manual_skill_during_one(store, serve):  # noqa: F811
    action = FakeAction()   # the manual skill runs until it is told to end
    bridge = preview_bridge(store, action)
    out = run_in_thread(bridge, 'move', {'to': 'home'}, label='home')
    wait_for(lambda: bridge.manual_handle is not None, 'the manual skill')
    request = serve(bridge)
    status, answer = request('POST', '/api/skills/move/dry_run', {'args': {'to': 'home'}})
    assert status == 409 and 'manual skill is running' in answer['problems'][0]
    assert len(action.goals) == 1
    action.handle.end('succeeded', 'at home')
    out['thread'].join(5.0)
    # and the other way round: a preview holds the lock, so a manual skill waits its turn
    held = threading.Event()
    gate = threading.Event()
    slow = AnsweringAction('succeeded', 'resolved', {})
    original = slow.send_goal_async
    slow.send_goal_async = lambda goal: (held.set(), gate.wait(5), original(goal))[2]
    bridge.skill_clients = {'move': slow}
    preview = threading.Thread(target=lambda: request('POST', '/api/skills/move/dry_run', {'args': {'to': 'home'}}),
                               daemon=True)
    preview.start()
    assert held.wait(5)
    with pytest.raises(Busy, match='another manual skill is running'):
        bridge.run_skill('move', {'to': 'home'})
    gate.set()
    preview.join(5)
    assert not bridge.skill_lock.locked()


def test_a_placeholder_is_refused_before_anything_is_sent(store, serve):  # noqa: F811
    action = AnsweringAction()
    request = serve(preview_bridge(store, action))
    status, answer = request('POST', '/api/skills/move/dry_run', {'args': {'to': '{next_stack}/grasp'}})
    assert status == 422 and '{next_stack}/grasp is filled at run time' in answer['problems'][0]
    status, answer = request('POST', '/api/skills/move/dry_run', {})
    assert status == 422 and action.goals == []
    for body in ([{'to': 'home'}], 'home'):   # a body that is JSON but not an object
        status, answer = request('POST', '/api/skills/move/dry_run', body)
        assert status == 422 and 'arguments as an object' in answer['problems'][0]
    assert action.goals == []


def test_no_mission_starts_while_a_preview_or_a_manual_skill_holds_the_lock(store, serve):  # noqa: F811
    bridge = preview_bridge(store, AnsweringAction())
    bridge.start_client, started = object(), []
    bridge.call = lambda _client, request, _timeout=None: (started.append(request.mission),
                                                           SimpleNamespace(accepted=True, message='ok',
                                                                           plan_id='p1'))[1]
    request = serve(bridge)
    bridge.skill_lock.acquire()     # a preview solving, or a manual skill running
    status, answer = request('POST', '/api/start', {'mission': 'cycle'})
    assert status == 409 and 'move preview is running' in answer['problems'][0] and started == []
    bridge.skill_lock.release()
    assert request('POST', '/api/start', {'mission': 'cycle'}) == (200, {'accepted': True, 'message': 'ok',
                                                                       'plan_id': 'p1'})
    assert started == ['cycle']
