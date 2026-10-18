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


class NavigateTo(Skill):
    name = 'navigate_to'
    description = 'Test double: arrives immediately and records where it went.'
    args_schema = {'type': 'object', 'required': ['target'],
                   'properties': {'target': {'type': 'string', 'x-taskweave-point': 'base_target'}}}

    def execute(self, ctx, args):
        ctx.feedback(1.0, 'arrived')
        return SkillResult.succeeded(world_state={'robot_at': args['target']})


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
        ros.add(SkillServer(skills=[NavigateTo(), Wait()], node_name='test_skills', parameter_overrides=params))
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
    assert sorted(s.name for s in skills) == ['navigate_to', 'wait']


def test_mission_runs_to_success_and_updates_world(system):
    handler, client, statuses, _ = system
    response = call(client, StartMission, names.START_MISSION, StartMission.Request(mission='patrol'))
    assert response.accepted, response.message
    status = wait_for(handler, {'SUCCEEDED', 'FAILED'})
    assert status.state.name == 'SUCCEEDED', status.last_error
    assert handler.world['robot_at'] == 'home'
    wait_until(lambda: any(s.state == TaskStatus.RUNNING for s in statuses), what='a RUNNING status message')


def test_invalid_mission_is_rejected_before_moving(system, config_dir):
    handler, client, _, _ = system
    (config_dir / 'missions' / 'lost.yaml').write_text(
        'goal: g\nrobot: demo_bot\nsteps:\n  - {instruction: go, skill: navigate_to, args: {target: nowhere}}\n')
    response = call(client, StartMission, names.START_MISSION, StartMission.Request(mission='lost'))
    assert not response.accepted
    assert "unknown location 'nowhere'" in response.message


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
    assert '- navigate_to(target' in response.card and 'stance:shelf_a' in response.card
    assert sorted(s.name for s in response.skills) == ['navigate_to', 'wait']


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
