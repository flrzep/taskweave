"""The task handler's speed override over ROS: service, latched topic, parameter, test mode at run start and end."""

import time

import pytest
from conftest import RosRig, wait_until, warm_up

rclpy = pytest.importorskip('rclpy')

from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill import Skill, SkillResult  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.task_handler import TaskHandler  # noqa: E402
from taskweave_interfaces.msg import SpeedOverride  # noqa: E402
from taskweave_interfaces.srv import SetSpeedOverride, StartMission  # noqa: E402


class Wait(Skill):
    name = 'wait'
    args_schema = {'type': 'object', 'required': ['seconds'], 'properties': {'seconds': {'type': 'number'}}}

    def execute(self, ctx, args):
        if not ctx.sleep(args['seconds']):
            return ctx.interrupted()
        return SkillResult.succeeded(seen=ctx.speed.fraction())


class NavigateTo(Wait):
    name = 'navigate_to'
    args_schema = {'type': 'object'}

    def execute(self, ctx, args):
        return SkillResult.succeeded()


@pytest.fixture
def system(config_dir):
    with RosRig() as ros:
        params = [Parameter('config_dir', value=str(config_dir)), Parameter('robot', value='demo_bot')]
        handler = ros.add(TaskHandler(parameter_overrides=params))
        server = ros.add(SkillServer(skills=[NavigateTo(), Wait()], node_name='test_skills', parameter_overrides=params))
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        seen = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        client.create_subscription(SpeedOverride, names.SPEED_OVERRIDE, seen.append, latched)
        warm_up(handler)
        yield handler, server, client, seen, config_dir


def call(node, srv_type, name, request):
    proxy = node.create_client(srv_type, name)
    assert proxy.wait_for_service(timeout_sec=30)
    future = proxy.call_async(request)
    wait_until(future.done, what=f'the answer of {name}')
    return future.result()


def until(condition, timeout=30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(0.02)
    return False


def test_service_sets_the_override_topic_parameter_and_skill_servers(system):
    handler, server, client, seen, _ = system
    assert until(lambda: seen and seen[-1].percent == 100.0)
    assert seen[-1].test_percent == 10.0 and seen[-1].cap_percent == 50.0 and not seen[-1].test_mode
    response = call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE, SetSpeedOverride.Request(percent=50.0))
    assert response.ok and response.percent == 50.0
    assert until(lambda: seen[-1].percent == 50.0)
    assert handler.get_parameter('speed_override_percent').value == 50.0
    assert until(lambda: server.speed.fraction() == 0.5)
    bad = call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE, SetSpeedOverride.Request(percent=150.0))
    assert not bad.ok and bad.percent == 50.0


def test_parameter_goes_through_the_same_rules(system):
    handler, _, client, seen, _ = system
    assert handler.set_parameters([Parameter('speed_override_percent', value=30.0)])[0].successful
    assert until(lambda: seen and seen[-1].percent == 30.0)
    assert not handler.set_parameters([Parameter('speed_override_percent', value=0.5)])[0].successful
    call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE,
         SetSpeedOverride.Request(percent=0.0, test_mode=True, set_test_mode=True))
    result = handler.set_parameters([Parameter('speed_override_percent', value=90.0)])[0]
    assert handler.speed.percent <= 50.0 and (not result.successful or
                                              handler.get_parameter('speed_override_percent').value == 50.0)


def test_test_mode_lowers_for_a_run_and_restores_after(system, config_dir):
    import json

    from std_msgs.msg import String

    handler, _, client, seen, _ = system
    episodes = []
    client.create_subscription(String, names.EPISODES, lambda m: episodes.append(json.loads(m.data)), 50)
    (config_dir / 'missions' / 'short.yaml').write_text(
        'goal: g\nrobot: demo_bot\nsteps:\n  - {instruction: wait, skill: wait, args: {seconds: 0.1}}\n')
    response = call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE,
                    SetSpeedOverride.Request(percent=80.0, test_mode=True, set_test_mode=True))
    assert response.ok and response.percent == 50.0 and 'cap' in response.message
    response = call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE,
                    SetSpeedOverride.Request(percent=0.0, test_mode=True, set_test_mode=True, test_percent=20.0))
    assert response.ok and response.percent == 50.0
    assert call(client, StartMission, names.START_MISSION, StartMission.Request(mission='short')).accepted
    assert until(lambda: any(m.percent == 20.0 and m.test_mode for m in seen))
    assert until(lambda: handler.executive.status.state.name == 'SUCCEEDED')
    assert until(lambda: seen[-1].percent == 50.0)
    assert handler.get_parameter('speed_override_percent').value == 50.0
    finished = [e for e in episodes if e['event'] == 'step_finished']
    speed = finished[0]['speed']
    assert speed['override_min'] == 20.0 and 20.0 <= speed['override_avg'] <= 50.0


def test_a_percent_only_call_leaves_test_mode_alone(system):
    """P1 review fix 6: test_mode is applied only when set_test_mode is true (a percent-only call used to switch the
    latch off)."""
    handler, _, client, seen, _ = system
    request = SetSpeedOverride.Request
    call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE, request(percent=0.0, test_mode=True, set_test_mode=True))
    assert handler.speed.test_mode
    assert call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE, request(percent=20.0)).ok
    assert handler.speed.test_mode and handler.speed.percent == 20.0
    assert until(lambda: seen[-1].test_mode and seen[-1].percent == 20.0)
    call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE,
         request(percent=0.0, test_mode=False, set_test_mode=True))
    assert not handler.speed.test_mode                                  # set_test_mode true: false is applied
    call(client, SetSpeedOverride, names.SET_SPEED_OVERRIDE, request(percent=0.0, test_mode=True, set_test_mode=False))
    assert not handler.speed.test_mode                                  # set_test_mode false: test_mode is ignored
