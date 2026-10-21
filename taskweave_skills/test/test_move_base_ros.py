"""move for the base against a small kinematic base simulator (cmd_vel in, odom out), on the schema-4 test cell: the
rows of the mapping table for navigate_to and drive_straight (a base target, motion.mode straight, base: {by}), the
drive height of the torso, a joints part and a path on a base target, motion.speed, dry runs and the messages."""

import json
import math
from pathlib import Path

import pytest
from conftest import TIME_SCALE, RosRig, connect, copy_config, wait_until

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('sensor_msgs')

from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from test_navigate_ros import BaseSim  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_skills.hold import HoldState  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config_new'
OK, FAILED, REJECTED = RunSkill.Result.SUCCEEDED, RunSkill.Result.FAILED, RunSkill.Result.REJECTED


def run(client, skill, args, timeout_s=None):
    timeout_s = 30.0 * TIME_SCALE if timeout_s is None else timeout_s
    action = connect(client, RunSkill, names.skill_action(skill))
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args),
                                                  timeout_s=timeout_s))
    wait_until(future.done, what='the goal to be accepted')
    result = future.result().get_result_async()
    wait_until(result.done, timeout=timeout_s + 30, what='the skill result')
    return result.result().result


def go(client, args, outcome=OK, timeout_s=None):
    result = run(client, 'move', args, timeout_s)
    assert result.outcome == outcome, result.message
    return result, json.loads(result.data_json)


@pytest.fixture(params=[('omni', True, 'omni_bot'), ('diff', False, 'diff_bot')], ids=['holonomic', 'differential'])
def system(request, tmp_path):
    with RosRig() as ros:
        prefix, holonomic, robot = request.param
        config = tmp_path / 'config'
        copy_config(FIXTURE, config)
        params = [Parameter('config_dir', value=str(config)), Parameter('robot', value=robot)]
        sim = ros.add(BaseSim(prefix, holonomic))
        server = ros.add(SkillServer(parameter_overrides=params + [Parameter('skills', value=['move', 'wait'])]))
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        wait_until(lambda: sim.pub.get_subscription_count() > 0, what='the skill server to subscribe to the odometry')
        wait_until(lambda: sim.count_publishers(sim.cmd_topic) > 0, what='the skill server to advertise cmd_vel')
        move = server.skills['move']
        wait_until(lambda: all(move.modules[p]._current() is not None for p in ('base', 'straight')),
                   what='the first odometry at the parts')
        client.server, client.config = server, config
        yield sim, client


def test_move_serves_only_the_parts_this_robot_can_do(system):
    _, client = system
    move = client.server.skills['move']
    assert sorted(move.modules) == ['base', 'straight'] and sorted(move.unavailable) == [
        'hand', 'joints', 'object', 'posture']
    assert 'no role' in move.unavailable['joints'][0]
    assert 'bimanual' in move.unavailable['hand'][0] or 'arms.bimanual' in move.unavailable['hand'][0]
    assert move.default_timeout({'to': 'dock'}) == 120.0


def test_drives_to_a_cell_target(system):
    sim, client = system
    result, data = go(client, {'to': 'dock'})
    assert math.hypot(sim.x - 1.0, sim.y - 0.5) < 0.04 and abs(sim.yaw - 1.5708) < 0.04
    assert data['world_state'] == {'robot_location': 'dock'}
    assert data['kind'] == 'base' and data['module'] == 'move.base' and data['target'] == 'dock'
    assert result.message.startswith('move base to dock (direct): at dock (')
    assert sim.cmd.linear.x == 0.0 and sim.cmd.angular.z == 0.0


def test_an_unknown_target_is_rejected_without_moving(system):
    sim, client = system
    result = run(client, 'move', {'to': 'mars'})
    assert result.outcome == REJECTED and "unknown target 'mars'" in result.message
    assert sim.x == 0.0 and sim.y == 0.0
    result, _ = go(client, {'to': 'dock_tool'}, REJECTED)
    # the argument check says the robot lacks the tool; a robot with tools but no hand module says it cannot move hands
    assert ('does not fit robot' in result.message or result.message.startswith('move hand: ')
            or 'this robot cannot do hand moves' in result.message)


def test_a_blocked_path_fails_as_stuck_and_says_which_motion(system):
    sim, client = system
    result, data = go(client, {'to': 'behind_wall'}, FAILED)
    assert result.message.startswith('move base to behind_wall (direct): no progress') and data['kind'] == 'base'
    wait_until(lambda: sim.cmd.linear.x == 0.0, what='the base to be told to stop')


def test_a_path_on_a_base_target_makes_successive_goals(system):
    sim, client = system
    seen = []
    client.create_subscription(Twist, sim.cmd_topic, lambda m: seen.append((sim.x, sim.y)), 10)
    _, data = go(client, {'to': 'dock_path'})
    assert math.hypot(sim.x - 1.0, sim.y - 0.5) < 0.04
    assert any(abs(x - 0.5) < 0.1 and abs(y) < 0.1 for x, y in seen)      # it went by the first waypoint
    assert data['target'] == 'dock_path'


def test_drive_straight_forward_and_back_keeps_the_heading(system):
    sim, client = system
    sim.yaw = 0.5
    move = client.server.skills['move']
    wait_until(lambda: (pose := move.modules['straight']._current()) is not None and abs(pose.yaw - 0.5) < 1e-3,
               what='the new heading at the part')
    result, data = go(client, {'base': {'by': [0.4, 0]}})
    assert sim.x == pytest.approx(0.4 * math.cos(0.5), abs=0.04) and sim.yaw == pytest.approx(0.5, abs=0.03)
    assert data['kind'] == 'straight' and result.message.startswith('move straight by +0.4 m: drove +0.40 m')
    go(client, {'base': {'by': [-0.4, 0]}})
    assert math.hypot(sim.x, sim.y) < 0.04 and sim.yaw == pytest.approx(0.5, abs=0.03)


def test_drive_straight_limits_say_to_use_the_nav_mode(system):
    sim, client = system
    result, _ = go(client, {'base': {'by': [2.0, 0]}}, REJECTED)
    assert result.message.startswith('move straight: 2.00 m is more than max_distance_m 1') and (
        result.message.endswith('use motion.mode nav'))
    result = run(client, 'move', {'base': {'by': [0.0, 0.3]}})
    if sim.holonomic:
        assert result.outcome == OK and sim.y == pytest.approx(0.3, abs=0.04)
    else:
        assert result.outcome == REJECTED and 'sideways' in result.message


def test_drive_straight_to_a_target_in_line(system):
    sim, client = system
    with open(client.config / 'cell' / 'targets.yaml', 'a') as f:
        f.write('ahead: {moves: base, x: 0.5, y: 0.01, yaw: 0.02}\n')
    result, data = go(client, {'to': 'ahead', 'motion': {'mode': 'straight'}})
    assert sim.x == pytest.approx(0.5, abs=0.04) and abs(sim.yaw) < 0.03 and data['kind'] == 'straight'
    result, _ = go(client, {'to': 'dock', 'motion': {'mode': 'straight'}}, REJECTED)
    assert 'heading' in result.message and 'motion.mode nav' in result.message


def test_a_speed_fraction_slows_the_base(system):
    sim, client = system
    peak = []
    sim.create_subscription(Twist, sim.cmd_topic, lambda m: peak.append(abs(m.linear.x)), 10)
    go(client, {'base': {'by': [0.3, 0]}, 'motion': {'speed': 0.5}})
    straight_peak = max(peak)
    assert straight_peak <= 0.15 * 0.5 + 1e-6                        # the part's speed 0.15 m/s x 0.5
    peak.clear()
    go(client, {'to': 'dock', 'motion': {'speed': 0.5}})
    assert max(peak) <= 0.6 * 0.5 + 1e-6                             # max_linear 0.6 of skills/move/base.yaml x 0.5


def test_a_dry_run_of_a_base_move_resolves_and_does_not_drive(system):
    sim, client = system
    result, data = go(client, {'to': 'dock', 'dry_run': True})
    assert data['dry_run'] is True and data['pose'] == {'x': 1.0, 'y': 0.5, 'yaw': 1.5708} and data['joint_goal'] == {}
    assert sim.x == 0.0 and sim.y == 0.0 and 'nothing moved' in result.message
    _, data = go(client, {'base': {'by': [0.3, 0.0]}, 'dry_run': True})
    assert data['by'] == [0.3, 0.0] and sim.x == 0.0


def test_the_old_names_in_the_launch_serve_move(tmp_path):
    with RosRig() as ros:
        config = tmp_path / 'config'
        copy_config(FIXTURE, config)
        ros.add(BaseSim('diff', False))
        server = ros.add(SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value='diff_bot'),
            Parameter('skills', value=['navigate_to', 'wait', 'drive_straight'])]))
        assert list(server.skills) == ['move', 'wait']


def test_an_older_cell_serves_its_old_skills_and_not_move(tmp_path):
    with RosRig() as ros:
        config = tmp_path / 'config'
        copy_config(Path(__file__).parent / 'fixtures' / 'config', config)
        ros.add(BaseSim('diff', False))
        server = ros.add(SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value='diff_bot'),
            Parameter('skills', value=['move', 'navigate_to', 'wait'])]))
        assert list(server.skills) == ['navigate_to', 'wait']


# ---- the torso and the joints part ------------------------------------------------------------------------------

class JointSim(Node):
    """A lift and two head joints moving toward their commands; remembers the lift height when the base first moves."""

    def __init__(self, base: BaseSim):
        super().__init__('lift_sim')
        self.pos = {'lift': 0.0, 'head1': 0.0, 'head2': 0.0}
        self.target, self.commands, self.at_first_drive, self.frozen = dict(self.pos), 0, None, False
        self.pub = self.create_publisher(JointState, '/lb/joint_states', 10)
        self.create_subscription(JointState, '/lb/joint_command', self.command, 10)
        self.create_subscription(Twist, base.cmd_topic, self.drive, 10)
        self.create_timer(0.02, self.step)

    def command(self, msg):
        self.commands += 1
        self.target.update(dict(zip(msg.name, msg.position, strict=False)))

    def drive(self, msg):
        if self.at_first_drive is None and (msg.linear.x or msg.angular.z):
            self.at_first_drive = dict(self.pos)

    def step(self):
        if not self.frozen:
            for name, target in self.target.items():
                self.pos[name] += max(-0.01, min(0.01, target - self.pos[name]))
        self.pub.publish(JointState(name=list(self.pos), position=list(self.pos.values())))


@pytest.fixture
def lift(tmp_path):
    with RosRig() as ros:
        config = tmp_path / 'config'
        copy_config(FIXTURE, config)
        base = ros.add(BaseSim('lb', False))
        joints = ros.add(JointSim(base))
        server = ros.add(SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value='lift_bot'),
            Parameter('skills', value=['move'])]))
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        wait_until(lambda: joints.count_subscribers('/lb/joint_command') > 0, what='the skill server to command joints')
        move = server.skills['move']
        wait_until(lambda: all(move.modules[p]._current() is not None for p in ('base', 'straight')), what='odometry')
        yield base, joints, server, client


@pytest.mark.parametrize('args', [{'to': 'dock'}, {'base': {'by': [0.3, 0]}}])
def test_the_torso_goes_to_the_drive_height_before_the_base_moves(lift, args):
    base, joints, _, client = lift
    _, data = go(client, args)
    drive = data['torso_drive']
    assert drive['moved'] and drive['to_m'] == pytest.approx(0.3)
    assert joints.at_first_drive['lift'] == pytest.approx(0.3, abs=0.02)     # finished before the base moved
    assert math.hypot(base.x, base.y) > 0.2


def test_torso_keep_and_a_torso_value_of_the_step(lift):
    base, joints, _, client = lift
    _, data = go(client, {'to': 'dock', 'torso': 'keep'})
    assert 'torso_drive' not in data and joints.commands == 0
    _, data = go(client, {'to': 'start', 'torso': 0.45})
    assert data['torso_drive']['to_m'] == pytest.approx(0.45) and joints.pos['lift'] == pytest.approx(0.45, abs=0.02)
    result, _ = go(client, {'to': 'dock', 'torso': 'auto'}, REJECTED)
    assert 'torso auto needs a hand or object target' in result.message


def test_the_joints_part_of_a_base_target_moves_first_and_its_torso_replaces_the_drive_height(lift):
    base, joints, _, client = lift
    _, data = go(client, {'to': 'dock_lifted'})
    assert joints.at_first_drive['lift'] == pytest.approx(0.4, abs=0.02)         # not drive_m 0.3
    assert joints.at_first_drive['head1'] == pytest.approx(0.2, abs=0.02)
    joints.at_first_drive = None
    joints.pos['lift'] = joints.target['lift'] = 0.0
    _, data = go(client, {'to': 'start', 'torso': 'keep'})
    joints.at_first_drive = None
    _, data = go(client, {'to': 'dock_head'})
    assert joints.at_first_drive['head1'] == pytest.approx(0.2, abs=0.02)
    assert joints.at_first_drive['lift'] == pytest.approx(0.3, abs=0.02)         # the head, then drive_m for the lift


def test_a_torso_that_cannot_get_there_fails_before_the_base_moves(lift):
    base, joints, _, client = lift
    joints.frozen = True
    result, _ = go(client, {'to': 'dock'}, FAILED, timeout_s=15.0)
    assert 'torso' in result.message and result.message.startswith('move base to dock')
    assert base.x == 0.0 and base.y == 0.0


def test_a_held_object_keeps_the_torso_where_it_is(lift):
    base, joints, server, client = lift
    HoldState._instances[id(server)].holds['box_1'] = {
        'object': 'box_1', 'point': 'box_1/rim', 'hands': ['left', 'right'], 'tcp_in_object': {}, 'type': None,
        'object_in_base': None}
    _, data = go(client, {'to': 'dock'})
    assert data['torso_drive']['moved'] is False and 'box_1' in data['torso_drive']['skipped']
    assert joints.commands == 0
