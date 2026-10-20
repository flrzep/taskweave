"""reload_config of the skill server: a skill yaml edit reaches the next goal without a restart; what is bound at start
is reported as needs_restart; a call during a goal is refused as busy."""

import json
import math
from pathlib import Path

import pytest
import yaml
from conftest import TIME_SCALE, RosRig, connect, copy_config, wait_until
from test_navigate_ros import BaseSim

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_interfaces.srv import ReloadConfig  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'


@pytest.fixture
def system(tmp_path):
    with RosRig() as ros:
        config = tmp_path / 'config'
        copy_config(FIXTURE, config)
        params = [Parameter('config_dir', value=str(config)), Parameter('robot', value='omni_bot')]
        sim = ros.add(BaseSim('omni', True))
        server = ros.add(SkillServer(parameter_overrides=params
                                     + [Parameter('skills', value=['navigate_to', 'wait', 'drive_straight'])]))
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        wait_until(lambda: server.skills['drive_straight']._current() is not None, what='the first odometry')
        yield sim, server, client, config


def edit(config, path, change):
    target = config / path
    data = yaml.safe_load(target.read_text())
    change(data)
    target.write_text(yaml.safe_dump(data, sort_keys=False))


def reload_call(client, server, paths=()):
    proxy = client.create_client(ReloadConfig, f'{server.get_name()}{names.RELOAD_CONFIG_SUFFIX}')
    assert proxy.wait_for_service(timeout_sec=30)
    future = proxy.call_async(ReloadConfig.Request(paths=list(paths)))
    wait_until(future.done, what='the answer of reload_config')
    return future.result()


def send(client, skill, args, timeout_s=None):
    timeout_s = 20.0 * TIME_SCALE if timeout_s is None else timeout_s
    action = connect(client, RunSkill, names.skill_action(skill))
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args),
                                                  timeout_s=timeout_s))
    wait_until(future.done, what='the goal to be accepted')
    handle = future.result()
    result = handle.get_result_async()
    return handle, result


def run(client, skill, args):
    _, result = send(client, skill, args)
    wait_until(result.done, timeout=60, what='the skill result')
    return result.result().result


def test_a_skill_yaml_default_applies_to_the_next_goal_without_a_restart(system):
    sim, server, client, config = system
    assert run(client, 'drive_straight', {'distance_m': 0.4}).outcome == RunSkill.Result.SUCCEEDED
    edit(config, 'skills/drive_straight.yaml', lambda d: d.update(max_distance_m=0.2))
    answer = reload_call(client, server)
    assert answer.status == 'reloaded' and answer.node == server.get_name(), answer.message
    assert list(answer.reloaded) == ['skills/drive_straight.yaml'] and not answer.needs_restart
    result = run(client, 'drive_straight', {'distance_m': 0.4})
    assert result.outcome == RunSkill.Result.REJECTED and 'max_distance_m 0.2' in result.message
    again = reload_call(client, server)     # nothing changed since: nothing to do
    assert again.status == 'reloaded' and not again.reloaded and not again.needs_restart


def test_base_limits_of_the_profile_apply_live_the_rest_of_it_needs_a_restart(system):
    sim, server, client, config = system
    assert server.skills['navigate_to'].p.max_linear == 0.6
    edit(config, 'robots/omni_bot/profile.yaml', lambda d: d['base']['limits'].update(linear=0.3))
    answer = reload_call(client, server)
    assert list(answer.reloaded) == ['robots/omni_bot/profile.yaml'] and not answer.needs_restart
    assert math.isclose(server.skills['navigate_to'].p.max_linear, 0.3)
    edit(config, 'robots/omni_bot/profile.yaml', lambda d: d['base'].update(cmd_vel='/elsewhere/cmd_vel'))
    answer = reload_call(client, server)
    assert [(i.path, 'base' in i.what) for i in answer.needs_restart] == [('robots/omni_bot/profile.yaml', True)]
    assert not answer.reloaded
    assert server.skills['navigate_to'].base['cmd_vel'] == '/omni/cmd_vel'    # still what the publisher is made for
    assert [i.path for i in reload_call(client, server).needs_restart] == ['robots/omni_bot/profile.yaml']  # sticks


def test_another_implementation_needs_a_restart(system):
    sim, server, client, config = system
    edit(config, 'skills/drive_straight.yaml', lambda d: d['implementations'][0].update(name='other'))
    answer = reload_call(client, server)
    assert [i.path for i in answer.needs_restart] == ['skills/drive_straight.yaml'] and not answer.reloaded
    assert 'implementation' in answer.needs_restart[0].what


def test_a_file_that_does_not_load_changes_nothing(system):
    sim, server, client, config = system
    edit(config, 'skills/drive_straight.yaml', lambda d: d.update(max_distance_m=0.2, bogus_key=1))
    edit(config, 'skills/navigate_to.yaml', lambda d: d.update(max_linear=0.1))
    answer = reload_call(client, server)
    assert answer.status == 'failed' and 'bogus_key' in answer.message
    assert server.skills['drive_straight'].max_distance == 1.0 and server.skills['navigate_to'].p.max_linear == 0.6


def test_busy_while_a_goal_runs(system):
    sim, server, client, config = system
    handle, result = send(client, 'wait', {'seconds': 30})
    wait_until(lambda: server._active == 1, what='the goal to run')
    edit(config, 'skills/drive_straight.yaml', lambda d: d.update(max_distance_m=0.2))
    answer = reload_call(client, server)
    assert answer.status == 'busy' and not answer.reloaded and server.skills['drive_straight'].max_distance == 1.0
    handle.cancel_goal_async()
    wait_until(result.done, what='the goal to end')
    wait_until(lambda: server._active == 0, what='the server to be idle')
    answer = reload_call(client, server)
    assert answer.status == 'reloaded' and list(answer.reloaded) == ['skills/drive_straight.yaml']
    assert server.skills['drive_straight'].max_distance == 0.2
