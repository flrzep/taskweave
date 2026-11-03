"""Task handler + skill server talking over real ROS 2 interfaces (in one process)."""

import time

import pytest
from conftest import RosRig, wait_until, warm_up

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import Joy  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill import Skill, SkillResult  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.task_handler import TaskHandler  # noqa: E402
from taskweave_interfaces.msg import TaskStatus  # noqa: E402
from taskweave_interfaces.srv import DescribeRobot, ListSkills, StartMission  # noqa: E402

SEEN = []   # (target, ctx.context) of every Move goal


class Move(Skill):
    name = 'move'
    description = 'Test double: arrives immediately and records where it went.'
    args_schema = {'type': 'object', 'required': ['to'],
                   'properties': {'to': {'type': 'string', 'x-taskweave-point': 'target',
                                         'x-taskweave-target-kinds': ['base']}}}

    def execute(self, ctx, args):
        SEEN.append((args['to'], ctx.context))
        ctx.feedback(1.0, 'arrived')
        return SkillResult.succeeded(world_state={'robot_at': args['to']})


class Wait(Skill):
    name = 'wait'
    description = 'Test double: waits the given seconds; 99 or more means fail.'
    args_schema = {'type': 'object', 'required': ['seconds'], 'properties': {'seconds': {'type': 'number'}}}

    def execute(self, ctx, args):
        if args['seconds'] >= 99:
            return SkillResult.failed('told to fail')
        if not ctx.sleep(args['seconds']):
            return ctx.interrupted()
        return SkillResult.succeeded()


@pytest.fixture
def system(config_dir):
    with RosRig() as ros:
        params = [Parameter('config_dir', value=str(config_dir)), Parameter('robot', value='demo_bot')]
        handler = ros.add(TaskHandler(parameter_overrides=params))
        ros.add(SkillServer(skills=[Move(), Wait()], node_name='test_skills', parameter_overrides=params))
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        statuses = []
        client.create_subscription(TaskStatus, names.STATUS, statuses.append, 10)
        warm_up(handler)
        yield handler, client, statuses, config_dir


def call(node, srv_type, name, request):
    proxy = node.create_client(srv_type, name)
    assert proxy.wait_for_service(timeout_sec=30)
    future = proxy.call_async(request)
    wait_until(future.done, what=f'the answer of {name}')
    return future.result()


def wait_for(handler, states, timeout=30):
    def reached():
        return handler.executive and handler.executive.status.state.name in states

    try:
        wait_until(reached, timeout)
    except AssertionError:
        raise AssertionError(
            f'state {handler.executive.status.state.name if handler.executive else None} not in {states}'
            + (f': {handler.executive.status.last_error}' if handler.executive else '')) from None
    return handler.executive.status


def press(joy, button):  # a press and release
    for pressed in (1, 0):
        joy.publish(Joy(buttons=[pressed if i == button else 0 for i in range(15)]))
        time.sleep(0.2)


def press_until(joy, button, condition, what, patience_s=4.0):
    """Press until ``condition()``: a press can be lost while nodes are still finding each other (or sit unprocessed
    on a loaded machine), so look for its effect for ``patience_s`` before pressing again."""
    def pressed():
        press(joy, button)
        deadline = time.monotonic() + patience_s
        while time.monotonic() < deadline:
            if condition():
                return True
            time.sleep(0.05)
        return False

    wait_until(pressed, what=what, interval=0.0)


def test_catalog_lists_skills_from_servers(system):
    _, client, _, _ = system
    skills = call(client, ListSkills, names.LIST_SKILLS, ListSkills.Request()).skills
    assert sorted(s.name for s in skills) == ['move', 'wait']


def test_mission_runs_to_success_and_updates_world(system):
    handler, client, statuses, _ = system
    response = call(client, StartMission, names.START_MISSION, StartMission.Request(mission='patrol'))
    assert response.accepted, response.message
    status = wait_for(handler, {'SUCCEEDED', 'FAILED'})
    assert status.state.name == 'SUCCEEDED', status.last_error
    assert handler.world['robot_at'] == 'home'
    wait_until(lambda: any(s.state == TaskStatus.RUNNING for s in statuses), what='a RUNNING status message')


def test_the_status_says_planning_while_the_plan_is_validated(system):
    """A slow validation (the first start after a restart took minutes in a cell with many steps) shows as running with
    the plan id, the step count and 'planning', not as a mission without a plan; a second start meanwhile is refused."""
    import threading

    handler, client, statuses, _ = system
    inside, release = threading.Event(), threading.Event()
    real = handler._validate

    def slow(plan):
        inside.set()
        release.wait(20)
        return real(plan)

    handler._validate = slow
    future = client.create_client(StartMission, names.START_MISSION)
    assert future.wait_for_service(timeout_sec=30)
    answer = future.call_async(StartMission.Request(mission='patrol'))
    try:
        wait_until(inside.is_set, what='the validation to begin')
        wait_until(lambda: any(s.step_status == 'planning' and s.plan_id == 'patrol-0001' and s.step_count > 0
                               and s.state == TaskStatus.RUNNING and s.goal for s in statuses),
                   what='the planning status with the goal')
        assert not answer.done()
        refused = call(client, StartMission, names.START_MISSION, StartMission.Request(mission='patrol'))
        assert not refused.accepted and 'already active' in refused.message
    finally:
        release.set()
    wait_until(answer.done, what='the answer of start_mission')
    assert answer.result().accepted
    wait_for(handler, {'SUCCEEDED', 'FAILED'})


def test_a_start_while_another_is_being_prepared_is_refused_and_an_abort_in_that_phase_is_kept(system):
    """Before the executive exists (the catalog is being read) the status already says planning, a second start is
    refused, and an abort is not dropped: the mission ends ABORTED before a step runs."""
    import threading

    handler, client, statuses, _ = system
    inside, release = threading.Event(), threading.Event()
    real = handler.refresh_catalog

    def slow(*args, **kwargs):
        inside.set()
        release.wait(20)
        return real(*args, **kwargs)

    handler.refresh_catalog = slow
    proxy = client.create_client(StartMission, names.START_MISSION)
    assert proxy.wait_for_service(timeout_sec=30)
    answer = proxy.call_async(StartMission.Request(mission='patrol'))
    try:
        wait_until(inside.is_set, what='the catalog read to begin')
        wait_until(lambda: any(s.step_status == 'planning' and s.goal and s.state == TaskStatus.RUNNING
                               for s in statuses), what='the planning status')
        refused = call(client, StartMission, names.START_MISSION, StartMission.Request(mission='patrol'))
        assert not refused.accepted and 'being started' in refused.message
        stopped = call(client, Trigger, names.ABORT, Trigger.Request())
        assert stopped.success, stopped.message
    finally:
        release.set()
    wait_until(answer.done, what='the answer of start_mission')
    assert not answer.result().accepted
    status = wait_for(handler, {'ABORTED'})
    assert status.last_error == 'aborted by operator' and status.step is None
    wait_until(lambda: statuses[-1].state == TaskStatus.ABORTED, what='the ABORTED status')


def test_invalid_mission_is_rejected_before_moving(system, config_dir):
    handler, client, _, _ = system
    (config_dir / 'missions' / 'lost.yaml').write_text(
        'goal: g\nrobot: demo_bot\nsteps:\n  - {instruction: go, skill: move, args: {to: nowhere}}\n')
    response = call(client, StartMission, names.START_MISSION, StartMission.Request(mission='lost'))
    assert not response.accepted
    assert "unknown target 'nowhere'" in response.message


def test_failure_retries_then_aborts_mission(system, config_dir):
    handler, client, _, _ = system
    (config_dir / 'missions' / 'fail.yaml').write_text(
        'goal: g\nrobot: demo_bot\ndefaults: {retries: 1}\n'
        'steps:\n  - {instruction: fail, skill: wait, args: {seconds: 99}}\n')
    assert call(client, StartMission, names.START_MISSION, StartMission.Request(mission='fail')).accepted
    status = wait_for(handler, {'FAILED'})
    assert 'told to fail' in status.last_error


def test_pause_resume_from_operator_services(system, config_dir):
    handler, client, _, _ = system
    (config_dir / 'missions' / 'long.yaml').write_text(
        'goal: g\nrobot: demo_bot\nsteps:\n  - {instruction: wait, skill: wait, args: {seconds: 1.5}}\n')
    assert call(client, StartMission, names.START_MISSION, StartMission.Request(mission='long')).accepted
    time.sleep(0.3)
    assert call(client, Trigger, names.PAUSE, Trigger.Request()).success
    wait_for(handler, {'PAUSED'})
    assert call(client, Trigger, names.RESUME, Trigger.Request()).success
    assert wait_for(handler, {'SUCCEEDED', 'FAILED'}).state.name == 'SUCCEEDED'


def test_describe_robot_returns_card_and_served_skills(system):
    _, client, _, _ = system
    response = call(client, DescribeRobot, names.DESCRIBE_ROBOT, DescribeRobot.Request())
    assert response.card.startswith('Robot: demo_bot')
    assert '- move(to' in response.card and 'shelf_a/front' in response.card
    assert sorted(s.name for s in response.skills) == ['move', 'wait']


@pytest.fixture
def operator(system, config_dir):
    """An operator node of its own beside the system, spun and torn down by a rig (also when the test fails)."""
    from taskweave_core.operator_joy import OperatorJoy

    with RosRig() as ros:
        node = ros.add(OperatorJoy(parameter_overrides=[Parameter('config_dir', value=str(config_dir))]))
        ros.spin(2)
        yield node


def test_operator_buttons_pause_resume_and_abort(system, operator, config_dir):
    handler, client, _, _ = system
    joy = client.create_publisher(Joy, '/joy', 10)
    wait_until(lambda: joy.get_subscription_count() > 0, what='the operator node to subscribe to /joy')
    PAUSE_RESUME, ABORT = 7, 6  # operator.yaml

    (config_dir / 'missions' / 'long.yaml').write_text(
        'goal: g\nrobot: demo_bot\nsteps:\n  - {instruction: wait, skill: wait, args: {seconds: 30}}\n')
    assert call(client, StartMission, names.START_MISSION, StartMission.Request(mission='long')).accepted
    wait_until(lambda: handler.dispatcher.goals, what='the step to be running')
    state = lambda: handler.executive.status.state.name  # noqa: E731
    press_until(joy, PAUSE_RESUME, lambda: state() == 'PAUSED', 'the pause button to pause')
    press_until(joy, PAUSE_RESUME, lambda: state() == 'RUNNING', 'the pause button to resume')
    press_until(joy, ABORT, lambda: state() == 'ABORTED', 'the abort button to abort')
    assert wait_for(handler, {'ABORTED'}).state.name == 'ABORTED'


def test_operator_button_runs_a_skill_only_when_no_mission_is_active(system, operator, config_dir):
    handler, client, _, _ = system
    joy = client.create_publisher(Joy, '/joy', 10)
    wait_until(lambda: joy.get_subscription_count() > 0, what='the operator node to subscribe to /joy')
    # the operator node also needs the skill server (action goal) and the status topic before a press can count
    wait_until(lambda: all(c.server_is_ready() for c in operator.skill_clients.values()),
               what='the operator node to see the skill server')

    # operator.yaml: button 0 -> wait 1.5 s
    press_until(joy, 0, lambda: operator.running, 'the manual action to start', patience_s=3.0)
    wait_until(lambda: not operator.running, what='the manual action to finish')

    (config_dir / 'missions' / 'long.yaml').write_text(
        'goal: g\nrobot: demo_bot\nsteps:\n  - {instruction: wait, skill: wait, args: {seconds: 30}}\n')
    assert call(client, StartMission, names.START_MISSION, StartMission.Request(mission='long')).accepted
    wait_until(lambda: operator.state == TaskStatus.RUNNING, what='the operator node to see the mission running')
    press(joy, 0)
    time.sleep(0.5)  # a negative check: give the press time to be (wrongly) acted on
    assert not operator.running  # ignored while the mission runs
    call(client, Trigger, names.ABORT, Trigger.Request())


def test_a_mission_step_gets_what_follows_it_as_context(system):
    handler, client, _, _ = system
    SEEN.clear()
    assert call(client, StartMission, names.START_MISSION, StartMission.Request(mission='patrol')).accepted
    assert wait_for(handler, {'SUCCEEDED', 'FAILED'}).state.name == 'SUCCEEDED'
    first, last = SEEN
    assert first[1]['step_index'] == 0 and first[1]['plan_id'] == handler.executive.status.plan_id
    assert [(a['skill'], a['resolved']) for a in first[1]['ahead']] == [('wait', True), ('move', True)]
    assert first[1]['ahead'][1]['args'] == {'to': 'home'}
    assert last[1]['step_index'] == 2 and last[1]['ahead'] == []


def send_move(client, **goal):
    import json

    from rclpy.action import ActionClient

    from taskweave_interfaces.action import RunSkill

    action = ActionClient(client, RunSkill, names.skill_action('move'))
    assert action.wait_for_server(timeout_sec=30)
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill='move', args_json=json.dumps({'to': 'home'}),
                                                  **goal))
    wait_until(future.done, what='the goal answer')
    assert future.result().accepted
    result = future.result().get_result_async()
    wait_until(result.done, what='the result')
    return result.result().result


def test_goals_without_or_with_unusable_context_are_still_accepted(system):
    from taskweave_interfaces.action import RunSkill

    _, client, _, _ = system
    SEEN.clear()
    for goal in ({}, {'context_json': 'not json'}, {'context_json': '[1]'}):
        assert send_move(client, **goal).outcome == RunSkill.Result.SUCCEEDED
    assert [context for _, context in SEEN] == [{}, {}, {}]
    assert send_move(client, context_json='{"ahead": []}').outcome == RunSkill.Result.SUCCEEDED
    assert SEEN[-1][1] == {'ahead': []}


def test_the_context_goes_out_even_with_a_world_value_json_cannot_write(system):
    import json
    from pathlib import Path
    from types import SimpleNamespace

    handler, _, _, _ = system
    sent = []

    class Client:
        def send_goal_async(self, goal, feedback_callback=None):
            sent.append(goal)
            return SimpleNamespace(add_done_callback=lambda f: None)

    context = {'ahead': [{'id': 'b', 'skill': 'wait', 'kind': None, 'args': {'p': Path('x')}, 'resolved': True}]}
    step = SimpleNamespace(id='a', skill='wait', instruction='', args={}, timeout_s=5)
    handler.dispatcher._send(1, step, Client(), context)
    assert json.loads(sent[0].context_json)['ahead'][0]['args'] == {'p': str(Path('x'))}
