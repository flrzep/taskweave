"""torso.drive_m: navigate_to and drive_straight move the torso to the drive height before the base moves (a robot with
a lift, a kinematic base and a joint simulator), unless something is held; without the key nothing changes."""

import json
import math
from pathlib import Path

import jsonschema
import pytest
import yaml
from conftest import TIME_SCALE, RosRig, connect, copy_config, wait_until

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs')

from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from test_navigate_ros import BaseSim  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.config import schemas  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_skills.hold import HoldState  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
OK, FAILED = RunSkill.Result.SUCCEEDED, RunSkill.Result.FAILED
PROFILE = """name: lift_bot
base: {type: differential, cmd_vel: /lb/cmd_vel, odom: /lb/odom, frame: base_link, limits: {linear: 0.8, angular: 1.5}}
joints: {command: /lb/joint_command, state: /lb/joint_states, linear: [lift]}
"""
TORSO = 'torso: {joint: lift, drive_m: 0.3}\n'


class LiftSim(Node):
    """One lift joint moving toward its command at 0.5 m/s; remembers its height when the base first gets a command."""

    def __init__(self, base: BaseSim, height: float):
        super().__init__('lift_sim')
        self.lift, self.target, self.commands, self.at_first_drive = height, height, 0, None
        self.frozen = False
        self.pub = self.create_publisher(JointState, '/lb/joint_states', 10)
        self.create_subscription(JointState, '/lb/joint_command', self.command, 10)
        self.create_subscription(Twist, base.cmd_topic, self.drive, 10)
        self.create_timer(0.02, self.step)

    def command(self, msg):
        self.commands += 1
        self.target = dict(zip(msg.name, msg.position, strict=False)).get('lift', self.target)

    def drive(self, msg):
        if self.at_first_drive is None and (msg.linear.x or msg.angular.z):
            self.at_first_drive = self.lift

    def step(self):
        if not self.frozen:
            self.lift += max(-0.01, min(0.01, self.target - self.lift))
        self.pub.publish(JointState(name=['lift'], position=[self.lift]))


def build(tmp_path, ros, torso: str, height: float):
    config = tmp_path / 'config'
    copy_config(FIXTURE, config)
    robot = config / 'robots' / 'lift_bot'
    robot.mkdir()
    (robot / 'profile.yaml').write_text(PROFILE + torso)
    (robot / 'embodiment.yaml').write_text(
        'name: lift_bot\nsummary: Differential test base with a lift.\ncapabilities: [base.differential]\n'
        'base: {footprint: [0.5, 0.4], max_speed: {linear: 0.8, angular: 1.5}}\n')
    params = [Parameter('config_dir', value=str(config)), Parameter('robot', value='lift_bot')]
    base = ros.add(BaseSim('lb', False))
    lift = ros.add(LiftSim(base, height))
    server = ros.add(SkillServer(parameter_overrides=params + [Parameter('skills', value=['navigate_to',
                                                                                         'drive_straight'])]))
    client = ros.add(rclpy.create_node('test_client'))
    ros.spin(8)
    wait_until(lambda: lift.count_subscribers('/lb/joint_command') > 0, what='the skill server to command joints')
    wait_until(lambda: all(server.skills[n]._current() is not None for n in server.skills), what='the odometry')
    return base, lift, server, client


@pytest.fixture
def system(tmp_path, request):
    torso, height = getattr(request, 'param', (TORSO, 0.0))
    with RosRig() as ros:
        yield build(tmp_path, ros, torso, height)


def run(client, skill, args, timeout_s=None):
    timeout_s = 30.0 * TIME_SCALE if timeout_s is None else timeout_s
    action = connect(client, RunSkill, names.skill_action(skill))
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args),
                                                  timeout_s=timeout_s))
    wait_until(future.done, what='the goal to be accepted')
    result = future.result().get_result_async()
    wait_until(result.done, timeout=timeout_s + 30, what='the skill result')
    return result.result().result


def hold_box(server):
    HoldState._instances[id(server)].holds['box_1'] = {
        'object': 'box_1', 'point': 'box_1/rim', 'hands': ['left', 'right'], 'tcp_in_object': {}, 'type': None,
        'object_in_base': None}


@pytest.mark.parametrize('skill, args', [('navigate_to', {'target': 'dock'}), ('drive_straight', {'distance_m': 0.3})])
def test_the_torso_goes_to_the_drive_height_before_the_base_moves(system, skill, args):
    base, lift, _, client = system
    result = run(client, skill, args)
    assert result.outcome == OK, result.message
    drive = json.loads(result.data_json)['torso_drive']
    assert drive['moved'] and drive['from_m'] == pytest.approx(0.0) and drive['to_m'] == pytest.approx(0.3)
    assert lift.lift == pytest.approx(0.3, abs=0.02)
    assert lift.at_first_drive == pytest.approx(0.3, abs=0.02)        # the move was finished before the base moved
    assert math.hypot(base.x, base.y) > 0.2


@pytest.mark.parametrize('system', [(TORSO, 0.31)], indirect=True)
def test_a_torso_within_the_tolerance_is_left_alone(system):
    _, lift, _, client = system
    result = run(client, 'navigate_to', {'target': 'dock'})
    assert result.outcome == OK, result.message
    drive = json.loads(result.data_json)['torso_drive']
    assert drive['moved'] is False and drive['from_m'] == pytest.approx(0.31)
    assert lift.commands == 0


@pytest.mark.parametrize('system', [('', 0.0)], indirect=True)
def test_without_the_key_the_torso_is_never_touched(system):
    _, lift, _, client = system
    for skill, args in (('navigate_to', {'target': 'dock'}), ('drive_straight', {'distance_m': -0.2})):
        result = run(client, skill, args)
        assert result.outcome == OK, result.message
        assert 'torso_drive' not in json.loads(result.data_json)
    assert lift.commands == 0 and lift.lift == 0.0


def test_a_held_object_keeps_the_torso_where_it_is_and_the_result_says_so(system):
    base, lift, server, client = system
    hold_box(server)
    result = run(client, 'navigate_to', {'target': 'dock'})
    assert result.outcome == OK, result.message              # driving goes on at the carry height
    drive = json.loads(result.data_json)['torso_drive']
    assert drive['moved'] is False and 'box_1' in drive['skipped']
    assert lift.commands == 0 and lift.lift == 0.0 and math.hypot(base.x, base.y) > 0.5


def test_a_torso_that_cannot_get_there_fails_before_the_base_moves(system):
    base, lift, _, client = system
    lift.frozen = True
    result = run(client, 'navigate_to', {'target': 'dock'}, timeout_s=15.0)
    assert result.outcome == FAILED and 'torso' in result.message
    assert base.x == 0.0 and base.y == 0.0


def test_the_profile_schema_takes_torso_drive_m():
    base = {'name': 'r', 'base': {'type': 'differential', 'frame': 'b'}}
    validator = jsonschema.Draft202012Validator(schemas.ROBOT_PROFILE)
    errors = lambda doc: [e.message for e in validator.iter_errors(doc)]   # noqa: E731
    without = errors(base)
    assert errors({**base, 'torso': {'joint': 'lift', 'drive_m': 0.4, 'tolerance_m': 0.01}}) == without
    assert errors({**base, 'torso': {'drive_m': 0.4}})                      # the joint is needed
    assert errors({**base, 'torso': {'joint': 'lift', 'drive': 0.4}})       # no unknown keys
    assert errors({**base, 'torso': {'joint': 'lift', 'tolerance_m': 0}})
    assert schemas.default(schemas.ROBOT_PROFILE, 'torso', 'tolerance_m') == 0.02
    assert yaml.safe_load(TORSO)['torso']['drive_m'] == 0.3
