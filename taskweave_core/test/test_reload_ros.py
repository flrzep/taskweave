"""reload_config of the task handler and the taskweave CLI that calls every process."""

import pytest
from conftest import RosRig, wait_until, warm_up

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from taskweave_core import cli, names  # noqa: E402
from taskweave_core.skill import Skill, SkillResult  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.task_handler import TaskHandler  # noqa: E402
from taskweave_interfaces.srv import ReloadConfig, StartMission  # noqa: E402


class Wait(Skill):
    name = 'wait'
    args_schema = {'type': 'object', 'required': ['seconds'], 'properties': {'seconds': {'type': 'number'}}}

    def execute(self, ctx, args):
        return SkillResult.succeeded() if ctx.sleep(args['seconds']) else ctx.interrupted()


@pytest.fixture
def system(config_dir):
    with RosRig() as ros:
        params = [Parameter('config_dir', value=str(config_dir)), Parameter('robot', value='demo_bot')]
        handler = ros.add(TaskHandler(parameter_overrides=params))
        server = ros.add(SkillServer(skills=[Wait()], node_name='test_skills', parameter_overrides=params))
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        warm_up(handler)
        yield handler, server, client, config_dir


def call(client, srv_type, service, request):
    proxy = client.create_client(srv_type, service)
    assert proxy.wait_for_service(timeout_sec=30)
    future = proxy.call_async(request)
    wait_until(future.done, what=f'the answer of {service}')
    return future.result()


def reload_call(client, paths=()):
    return call(client, ReloadConfig, names.RELOAD_CONFIG, ReloadConfig.Request(paths=list(paths)))


def start(client, mission):
    return call(client, StartMission, names.START_MISSION, StartMission.Request(mission=mission))


def mission(config, name, seconds):
    (config / 'missions' / f'{name}.yaml').write_text(
        f'goal: g\nrobot: demo_bot\nsteps:\n  - {{instruction: wait, skill: wait, args: {{seconds: {seconds}}}}}\n')


def test_the_handler_reloads_missions_and_reports_launch_files(system):
    handler, _, client, config = system
    mission(config, 'fresh', 0.1)
    (config / 'operator.yaml').write_text('joy:\n  topic: /other_joy\n')
    answer = reload_call(client)
    assert answer.status == 'reloaded' and answer.node == 'task_handler', answer.message
    assert list(answer.reloaded) == ['missions/fresh.yaml']
    assert [i.path for i in answer.needs_restart] == ['operator.yaml'] and 'operator' in answer.needs_restart[0].what
    assert [i.path for i in reload_call(client).needs_restart] == ['operator.yaml']   # reported until a restart
    assert start(client, 'fresh').accepted


def test_the_handler_is_busy_during_a_mission(system):
    handler, _, client, config = system
    mission(config, 'long', 30)
    assert start(client, 'long').accepted
    answer = reload_call(client)
    assert answer.status == 'busy' and 'mission' in answer.message and not answer.reloaded
    assert call(client, Trigger, names.ABORT, Trigger.Request()).success
    wait_until(lambda: handler.executive.status.state.name == 'ABORTED', what='the mission to abort')
    assert reload_call(client).status == 'reloaded'


def test_the_cli_calls_every_process(system, capsys):
    handler, server, client, config = system
    mission(config, 'fresh', 0.1)
    results = cli.reload_all(client)
    assert sorted(r['node'] for r in results) == ['task_handler', 'test_skills']
    assert all(r['status'] == 'reloaded' for r in results)
    assert cli.print_reload(results) == 0
    out = capsys.readouterr().out
    assert 'task_handler: reloaded' in out and 'test_skills: reloaded' in out and 'missions/fresh.yaml' in out
