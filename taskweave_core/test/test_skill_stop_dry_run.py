"""skill.stop() after a failed goal: never for a dry run (it moves nothing, stop() acts on the whole skill)."""

import json
import threading

import pytest

rclpy = pytest.importorskip('rclpy')

from conftest import RosRig, wait_until  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.executive import Outcome  # noqa: E402
from taskweave_core.skill import Skill, SkillResult  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402


class StopSpy(Skill):
    name = 'stop_spy'
    description = 'Test double: fails on request, or runs until released; records stop() calls.'
    args_schema = {'type': 'object', 'properties': {'dry_run': {'type': 'boolean'}, 'fail': {'type': 'boolean'},
                                                    'hold': {'type': 'boolean'}}}

    def __init__(self):
        self.stops = 0
        self.release = threading.Event()
        self.running = threading.Event()

    def execute(self, ctx, args):
        if args.get('hold'):
            self.running.set()
            if not self.release.wait(20):
                return SkillResult.failed('never released')
            return SkillResult.succeeded()
        if args.get('fail'):
            return SkillResult.failed('planned failure')
        return SkillResult.succeeded()

    def stop(self):
        self.stops += 1


@pytest.fixture
def system(config_dir):
    with RosRig() as ros:
        params = [Parameter('config_dir', value=str(config_dir)), Parameter('robot', value='demo_bot')]
        skill = StopSpy()
        ros.add(SkillServer(skills=[skill], node_name='stop_spy_server', parameter_overrides=params))
        node = ros.add(rclpy.create_node('stop_spy_client'))
        ros.spin(8)
        client = ActionClient(node, RunSkill, names.skill_action('stop_spy'))
        assert client.wait_for_server(timeout_sec=30)
        yield skill, client


def send(client, **args):
    """Send a goal; return a list that holds the result once it is there."""
    out = []
    goal = RunSkill.Goal(step_id='s', args_json=json.dumps(args), timeout_s=20.0)

    def accepted(future):
        future.result().get_result_async().add_done_callback(lambda f: out.append(f.result().result))
    client.send_goal_async(goal).add_done_callback(accepted)
    return out


def test_a_failing_dry_run_does_not_stop_the_skill(system):
    skill, client = system
    out = send(client, dry_run=True, fail=True)
    wait_until(lambda: out, what='the dry run result')
    assert out[0].outcome != Outcome.SUCCEEDED.value
    assert skill.stops == 0


def test_a_rejected_dry_run_does_not_stop_the_skill(system):
    skill, client = system
    out = send(client, dry_run=True, fail='not a boolean')   # the schema rejects it at plan time
    wait_until(lambda: out, what='the dry run result')
    assert out[0].outcome != Outcome.SUCCEEDED.value
    assert skill.stops == 0


def test_a_failing_normal_goal_still_stops_the_skill(system):
    skill, client = system
    out = send(client, fail=True)
    wait_until(lambda: out, what='the result')
    assert out[0].outcome != Outcome.SUCCEEDED.value
    assert skill.stops == 1
    out = send(client, dry_run=False, fail=True)
    wait_until(lambda: out, what='the result')
    assert skill.stops == 2


def test_a_failing_dry_run_beside_a_running_goal_does_not_stop_it(system):
    skill, client = system
    running = send(client, hold=True)
    wait_until(skill.running.is_set, what='the first goal to run')
    dry = send(client, dry_run=True, fail=True)
    wait_until(lambda: dry, what='the dry run result')
    assert skill.stops == 0 and not running
    skill.release.set()
    wait_until(lambda: running, what='the first goal to finish')
    assert running[0].outcome == Outcome.SUCCEEDED.value
    assert skill.stops == 0
