"""Task handler + skill server talking over real ROS 2 interfaces (in one process)."""

import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill import Skill, SkillResult  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.task_handler import TaskHandler  # noqa: E402
from taskweave_interfaces.msg import TaskStatus  # noqa: E402
from taskweave_interfaces.srv import ListSkills, StartMission  # noqa: E402


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
    rclpy.init()
    params = [Parameter('config_dir', value=str(config_dir)), Parameter('robot', value='demo_bot')]
    handler = TaskHandler(parameter_overrides=params)
    server = SkillServer(skills=[NavigateTo(), Wait()], node_name='test_skills', parameter_overrides=params)
    client = rclpy.create_node('test_client')
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (handler, server, client):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    statuses = []
    client.create_subscription(TaskStatus, names.STATUS, statuses.append, 10)
    yield handler, client, statuses, config_dir
    executor.shutdown()
    for node in (handler, server, client):
        node.destroy_node()
    rclpy.shutdown()


def call(node, srv_type, name, request):
    proxy = node.create_client(srv_type, name)
    assert proxy.wait_for_service(timeout_sec=5)
    future = proxy.call_async(request)
    deadline = time.time() + 10
    while not future.done() and time.time() < deadline:
        time.sleep(0.02)
    return future.result()


def wait_for(handler, states, timeout=10):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if handler.executive and handler.executive.status.state.name in states:
            return handler.executive.status
        time.sleep(0.02)
    raise AssertionError(f'state {handler.executive.status.state.name if handler.executive else None} not in {states}')


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
    time.sleep(0.3)
    assert any(s.state == TaskStatus.RUNNING for s in statuses)


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
