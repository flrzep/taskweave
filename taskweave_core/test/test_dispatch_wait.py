"""The dispatcher's first dispatch to a skill can happen before DDS discovery has matched the skill server."""

import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')

from conftest import RosRig  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.executive import Outcome, Status  # noqa: E402
from taskweave_core.skill import Skill, SkillResult  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.task_handler import TaskHandler  # noqa: E402


class LateSkill(Skill):
    name = 'late_skill'
    description = 'Test double: succeeds at once.'
    args_schema = {'type': 'object', 'properties': {}}

    def execute(self, ctx, args):
        return SkillResult.succeeded()


@pytest.fixture
def rig(config_dir):
    with RosRig() as ros:
        params = [Parameter('config_dir', value=str(config_dir)), Parameter('robot', value='demo_bot'),
                  Parameter('server_wait_s', value=8.0)]
        handler = ros.add(TaskHandler(parameter_overrides=params))
        results = []
        handler.executive = SimpleNamespace(on_result=lambda token, outcome, message='', data=None:
                                            results.append((token, outcome, message)),
                                            on_feedback=lambda *a: None, tick=lambda: None, status=Status())
        handler.catalog = {n: SimpleNamespace(name=n, action_name=names.skill_action(n))
                           for n in ('late_skill', 'ghost_skill')}
        executor = ros.spin(8)
        yield handler, executor, results, params, ros.nodes


def step(skill):
    return SimpleNamespace(id='s1', skill=skill, instruction='', args={}, timeout_s=5)


def wait_result(results, timeout):
    deadline = time.monotonic() + timeout
    while not results and time.monotonic() < deadline:
        time.sleep(0.02)
    return results[0] if results else None


def test_first_dispatch_before_the_server_exists_waits_for_it(rig):
    handler, executor, results, params, nodes = rig
    started = time.monotonic()
    handler.dispatcher.dispatch(1, step('late_skill'), 1)  # no server yet, and no client yet
    assert time.monotonic() - started < 1.0, 'dispatch must not block its caller'
    time.sleep(1.0)
    server = SkillServer(skills=[LateSkill()], node_name='late_server', parameter_overrides=params)
    nodes.append(server)
    executor.add_node(server)
    result = wait_result(results, 15)
    assert result is not None, 'no result'
    assert result[1] == Outcome.SUCCEEDED, result


def test_dispatch_is_rejected_clearly_when_the_server_never_appears(rig):
    handler, _, results, _, _ = rig
    handler.set_parameters([Parameter('server_wait_s', value=0.5)])
    # a skill nobody serves (a name of its own: no stale discovery)
    handler.dispatcher.dispatch(1, step('ghost_skill'), 1)
    result = wait_result(results, 10)
    assert result is not None, 'no result'
    assert result[1] == Outcome.REJECTED
    assert "skill server for 'ghost_skill' is not available (waited 0.5 s)" in result[2]


class CountingSkill(LateSkill):
    name = 'canceled_skill'     # a name of its own: no stale discovery of another test's server
    ran = 0

    def execute(self, ctx, args):
        type(self).ran += 1
        return SkillResult.succeeded()


def test_a_cancel_during_the_server_wait_sends_no_goal_and_reports_canceled(rig):
    handler, executor, results, params, nodes = rig
    CountingSkill.ran = 0
    handler.catalog['canceled_skill'] = SimpleNamespace(name='canceled_skill',
                                                        action_name=names.skill_action('canceled_skill'))
    handler.dispatcher.dispatch(1, step('canceled_skill'), 1)   # no server yet: the wait thread starts
    time.sleep(0.5)
    handler.dispatcher.cancel(1)                              # abort/pause/timeout while it waits
    server = SkillServer(skills=[CountingSkill()], node_name='canceled_server', parameter_overrides=params)
    nodes.append(server)
    executor.add_node(server)
    result = wait_result(results, 15)
    assert result is not None, 'no result'
    assert result[1] == Outcome.CANCELED, result
    time.sleep(1.0)
    assert CountingSkill.ran == 0, 'the goal was sent after the cancel'
    assert len(results) == 1 and not handler.dispatcher.pending_cancel


def test_a_rejected_step_does_not_leak_its_cancel_token(rig):
    handler, _, results, _, _ = rig
    handler.set_parameters([Parameter('server_wait_s', value=0.3)])
    handler.dispatcher.dispatch(7, step('ghost_skill'), 1)
    handler.dispatcher.cancel(7)
    result = wait_result(results, 10)
    assert result is not None and result[1] in (Outcome.REJECTED, Outcome.CANCELED)
    assert handler.dispatcher.pending_cancel == set()
