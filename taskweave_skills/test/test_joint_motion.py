"""move_joints: targets (pose, joints), only, via, limits, the hold rule, speed scaling, the schema.

Most tests run the skill's execute against simulated time and joints that follow their commands exactly (as
test_speed_playback); the last one goes through a skill server and a joint simulator."""

import json
import shutil
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('sensor_msgs')

from test_speed_playback import Clock, Joints, Speed  # noqa: E402

from taskweave_core.config import ConfigStore  # noqa: E402
from taskweave_core.skill import Outcome, SkillContext  # noqa: E402
from taskweave_core.validation import oneof_problems, validate_args  # noqa: E402
from taskweave_skills import joints as joints_module  # noqa: E402
from taskweave_skills.joint_motion import MoveJoints  # noqa: E402
from taskweave_skills.joints import Params  # noqa: E402
from taskweave_skills.motion_plan import Trajectory  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
ROBOT = 'arm_bot'
START = {'j1': 0.0, 'j2': 0.0, 'lift': 0.0, 'grip': 1.0}


class FakeJoints(Joints):
    def snapshot(self, max_age):
        return dict(self.positions)

    def start(self, max_age, tolerance):
        return dict(self.positions)


class FakeHold:
    """The registry calls move_joints uses (hold.HoldState): held_all, reached_hands, leave, tcp_poses, moved."""

    def __init__(self, hold=None):
        self.hold, self.cleared, self.moved_to = hold, 0, None

    def held_all(self):
        return [{**self.hold, 'hands': list(self.hold['tools'])}] if self.hold else []

    def reached_hands(self, hands=None):
        return {'left': {}}

    def leave(self, hands):
        self.cleared += 1

    def tcp_poses(self, tools):
        return {tool: np.eye(4) for tool in tools}

    def moved(self, pose, name=None):
        self.moved_to = pose


class Rig:
    def __init__(self, monkeypatch, tmp_path, percent=100.0, hold=None):
        config = tmp_path / 'config'
        shutil.copytree(FIXTURE, config)
        store = ConfigStore(config)
        self.clock, self.speed = Clock(), Speed(percent)
        monkeypatch.setattr(joints_module.time, 'sleep', self.clock.sleep)
        skill = MoveJoints.__new__(MoveJoints)
        skill.config, skill.robot, skill.name = store, store.robot(ROBOT), 'move_joints'
        skill.p = Params.from_config(skill._joint_params(store.skill_params('move_joints')))
        skill.p.linear_joints = ('lift',)
        skill.base_p, skill._held, skill.ik, skill.planner = skill.p, {}, None, None
        skill.joints = FakeJoints(self.clock, START)
        skill.hold = FakeHold(hold)
        skill.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda *a: None))
        self.skill, self.store = skill, store

    def run(self, args):
        ctx = SkillContext(self.clock.now, 0.0, lambda: False, lambda p, s: None, speed=self.speed)
        return self.skill.execute(ctx, args)

    @property
    def commands(self):
        return self.skill.joints.commands

    def reached_at(self, joint, value):
        """Time (from the start) of the first command that has ``joint`` at ``value``."""
        return next(t for t, c in self.commands if abs(c.get(joint, 1e9) - value) < 1e-9) - 100.0


@pytest.fixture
def rig(monkeypatch, tmp_path):
    return Rig(monkeypatch, tmp_path)


def test_named_pose(rig):
    result = rig.run({'pose': 'ready'})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert rig.skill.joints.positions == {**START, 'j1': 1.0, 'j2': -0.5, 'lift': 0.2}
    assert result.data['world_state'] == {'arm_pose': 'ready'}
    assert result.data['hands'] == ['left'] and result.data['mode'] == 'free' and result.data['path'] == 'direct'
    assert result.data['speed']['scale'] == pytest.approx(1.0)
    assert rig.skill.hold.cleared == 1


def test_joint_values(rig):
    result = rig.run({'joints': {'j2': 0.7, 'lift': 0.25}})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert rig.skill.joints.positions == {**START, 'j2': 0.7, 'lift': 0.25}
    assert 'world_state' not in result.data
    assert set(rig.commands[0][1]) == {'j2', 'lift'}   # only the given joints are commanded


def test_only_restricts_the_pose_to_groups(rig):
    result = rig.run({'pose': 'ready', 'only': ['torso']})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert rig.skill.joints.positions['lift'] == 0.2 and rig.skill.joints.positions['j1'] == 0.0
    assert all(set(c) == {'lift'} for _, c in rig.commands) and result.data['hands'] == []
    result = rig.run({'pose': 'ready', 'only': ['left_arm']})
    assert result.outcome == Outcome.SUCCEEDED and result.data['hands'] == ['left']
    assert rig.skill.joints.positions['j1'] == 1.0


def test_only_rejects_an_unknown_group_and_an_empty_selection(rig):
    result = rig.run({'pose': 'ready', 'only': ['tail']})
    assert result.outcome == Outcome.REJECTED and 'robot has no group tail' in result.message
    result = rig.run({'pose': 'arm_only', 'only': ['torso']})
    assert result.outcome == Outcome.REJECTED and 'no joints of the target are in torso' in result.message
    assert not rig.commands


def test_via_passes_the_waypoints_in_order(rig):
    result = rig.run({'pose': 'ready', 'motion': {'via': ['mid', {'j1': 0.9, 'j2': 0.9}, 'pose:arm_only']}})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    at = [rig.reached_at('j2', v) for v in (0.5, 0.9, -0.4, -0.5)]
    assert at == sorted(at)
    assert rig.skill.joints.positions['j1'] == 1.0
    assert result.message.startswith('at ready')


def test_via_only_applies_to_the_selected_groups(rig):
    result = rig.run({'pose': 'ready', 'only': ['torso'], 'motion': {'via': ['lift_only']}})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert all(set(c) == {'lift'} for _, c in rig.commands)
    assert rig.reached_at('lift', 0.3) < rig.reached_at('lift', 0.2)


@pytest.mark.parametrize('args, message', [
    ({'joints': {'j1': 3.0}}, 'outside the joint limits: j1'),
    ({'joints': {'lift': -0.1}}, 'outside the joint limits: lift: -0.100 m'),
    ({'joints': {'zz': 0.1}}, 'unknown joint zz'),
    ({'pose': 'ready', 'motion': {'via': [{'j2': 9.0}]}}, 'outside the joint limits: j2'),
    ({'pose': 'reach'}, 'pose reach places tools: use move_gripper to.pose'),
    ({'pose': 'dance'}, "unknown pose 'dance'"),
    ({'pose': 'ready', 'joints': {'j1': 0.1}}, 'exactly one of'),
])
def test_rejected_before_anything_moves(rig, args, message):
    result = rig.run(args)
    assert result.outcome == Outcome.REJECTED and message in result.message, result.message
    assert not rig.commands and rig.skill.hold.cleared == 0


HOLD = {'object': 'box', 'point': 'box/top', 'tools': ['left'], 'object_in_base': None,
        'tcp_in_object': {'left': np.eye(4)}}


def test_while_holding_arm_joints_are_refused_but_torso_is_allowed(monkeypatch, tmp_path):
    rig = Rig(monkeypatch, tmp_path, hold=dict(HOLD))
    result = rig.run({'pose': 'ready'})
    assert result.outcome == Outcome.REJECTED and 'holding box' in result.message
    result = rig.run({'joints': {'j1': 0.2, 'lift': 0.1}, 'only': ['torso', 'left_arm']})
    assert result.outcome == Outcome.REJECTED and 'holding box' in result.message
    result = rig.run({'pose': 'ready', 'motion': {'via': [{'j1': 0.3}]}, 'only': ['torso']})   # via: no torso, skipped
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert not any(c.get('j1') for _, c in rig.commands)
    rig.commands.clear()
    result = rig.run({'pose': 'lift_only'})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert rig.skill.joints.positions['lift'] == 0.3
    assert rig.skill.hold.moved_to is not None   # the held object follows the hand (hold.moved from the TCP)


def test_arm_joint_of_a_via_waypoint_is_refused_while_holding(monkeypatch, tmp_path):
    rig = Rig(monkeypatch, tmp_path, hold=dict(HOLD))
    result = rig.run({'pose': 'lift_only', 'motion': {'via': [{'j2': 0.3}]}})
    assert result.outcome == Outcome.REJECTED and 'holding box' in result.message and not rig.commands


JOINT_5 = {'j1': 2.0}


@pytest.mark.parametrize('motion, percent, factor', [
    ({}, 100.0, 1.0), ({}, 50.0, 2.0), ({'speed': 0.5}, 100.0, 2.0), ({'speed': 0.5}, 50.0, 4.0),
    ({'speed_abs': {'joint_rad_s': 1.0}}, 100.0, 2.0), ({'speed': 1.0}, 25.0, 4.0)])
def test_speed_scales_the_duration(monkeypatch, tmp_path, motion, percent, factor):
    """2 rad at 2 rad/s: 1 s at 100 %. The step's speed and the global override (phase clock) multiply."""
    rig = Rig(monkeypatch, tmp_path, percent)
    result = rig.run({'joints': JOINT_5, 'motion': motion})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert rig.reached_at('j1', 2.0) == pytest.approx(factor * 1.0, rel=0.03)
    assert result.data['speed']['scale'] == pytest.approx(1 / factor, rel=0.05)


def test_half_speed_takes_twice_as_long_as_full_speed(monkeypatch, tmp_path):
    full = Rig(monkeypatch, tmp_path / 'a')   # a Rig patches time.sleep with its clock: build and run one at a time
    full.run({'joints': JOINT_5})
    half = Rig(monkeypatch, tmp_path / 'b', 50.0)
    half.run({'joints': JOINT_5})
    assert half.reached_at('j1', 2.0) / full.reached_at('j1', 2.0) == pytest.approx(2.0, rel=0.03)


def test_tolerance_and_accel_options_reach_the_move(rig):
    seen = []
    original = rig.skill._move
    rig.skill._move = lambda ctx, goal, label, stall_ok=False, path=None: (
        seen.append(rig.skill.p) or original(ctx, goal, label, stall_ok, path))
    assert rig.run({'joints': {'j2': 0.1}, 'motion': {'tolerance': 0.05, 'accel': 0.5}}).outcome == Outcome.SUCCEEDED
    assert seen[0].position_tolerance == 0.05 and seen[0].min_duration_s == pytest.approx(1.0)
    assert rig.skill.p is rig.skill.base_p and rig.skill.p.position_tolerance == 0.02   # restored after the step


def test_a_planned_path_is_timed_at_the_step_speed(monkeypatch, tmp_path):
    rig = Rig(monkeypatch, tmp_path)
    path = Trajectory(['j1'], [0.0, 1.0], [[0.0], [1.0]])
    rig.skill.planner = SimpleNamespace(plan=lambda start, goal, stop: path, free=lambda *a: True)
    result = rig.run({'joints': {'j1': 1.0}, 'motion': {'speed': 0.5}})
    assert result.outcome == Outcome.SUCCEEDED and result.data['path'] == 'planned'
    assert rig.reached_at('j1', 1.0) == pytest.approx(2.0, rel=0.03)


def test_planner_direct_is_the_straight_move_and_fails_when_blocked(monkeypatch, tmp_path):
    rig = Rig(monkeypatch, tmp_path)
    planned = []
    rig.skill.planner = SimpleNamespace(plan=lambda *a: planned.append(1), free=lambda *a: False)
    result = rig.run({'joints': {'j1': 1.0}, 'motion': {'planner': 'direct'}})
    assert result.outcome == Outcome.FAILED and 'blocked' in result.message and not planned and not rig.commands
    rig.skill.planner.free = lambda *a: True
    result = rig.run({'joints': {'j1': 1.0}, 'motion': {'planner': 'direct'}})
    assert result.outcome == Outcome.SUCCEEDED and not planned and result.data['path'] == 'direct'


def test_schema_has_distinguishable_variants_and_checks_targets(rig):
    schema = MoveJoints.args_schema
    assert oneof_problems(schema, allow_several=True) == []
    assert oneof_problems(schema) == []   # exactly one unique key per variant
    store = rig.store

    def problems(args):
        return validate_args(args, schema, store, ROBOT)

    assert problems({'pose': 'home'}) == [] and problems({'joints': {'j1': 0.1}}) == []
    assert problems({'pose': '{start_pose}', 'only': ['{group}']}) == []   # placeholders
    assert problems({'joints': {'j1': '{angle}'}, 'motion': {'via': ['{p}', 'home', {'j2': 1}]}}) == []
    assert problems({'pose': 'home', 'only': ['torso', 'left_arm'], 'motion': {'speed': 0.5, 'via': ['mid']}}) == []
    assert any('args.pose' in p and 'places tools' in p for p in problems({'pose': 'reach'}))
    assert any(p.startswith('args.pose') for p in problems({'pose': 'dance'}))
    assert any(p.startswith('args.only.0') and 'no joint group' in p for p in problems({'pose': 'home', 'only': ['x']}))
    assert any(p.startswith('args.motion.via.0') for p in problems({'pose': 'home', 'motion': {'via': ['nope']}}))
    assert any(p.startswith('args.motion.via.0') and 'places tools' in p
               for p in problems({'pose': 'home', 'motion': {'via': ['pose:reach']}}))
    assert problems({'pose': 'home', 'joints': {'j1': 0}}) and problems({}) and problems({'pose': 'home', 'x': 1})
    assert problems({'pose': 'home', 'motion': {'mode': 'linear'}})   # not a move_joints option
    assert problems({'pose': 'home', 'motion': {'speed': 2}})


def test_yaml_layout_maps_onto_the_params():
    params = MoveJoints._joint_params({
        'speed': {'joint_rad_s': 1.2, 'joint_linear_m_s': 0.25, 'tcp_m_s': 0.05}, 'accel': {'joint_rad_s2': 3.0},
        'motion': {'planner': 'auto'}, 'tolerances': {'position': 0.03, 'settle_s': 0.3}, 'stall_speed': 0.05})
    assert params == {'stall_speed': 0.05, 'max_joint_speed': 1.2, 'max_linear_speed': 0.25, 'position_tolerance': 0.03,
                      'settle_s': 0.3}
    assert MoveJoints._joint_params({'max_joint_speed': 2.0}) == {'max_joint_speed': 2.0}


def limited(rig, **limits):
    """Give the rig's skill velocity limits from a MoveIt joint_limits.yaml in the robot's files."""
    from taskweave_core.config.limits import velocity_limits
    folder = rig.store.root / 'robots' / ROBOT / 'moveit'
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'joint_limits.yaml').write_text('joint_limits:\n' + ''.join(
        f'  {joint}: {{has_velocity_limits: true, max_velocity: {value}}}\n' for joint, value in limits.items()))
    rig.skill.p.velocity_limits = velocity_limits(ConfigStore(rig.store.root), ROBOT)
    rig.skill.base_p = rig.skill.p


def test_speed_abs_above_a_joint_limit_is_capped_and_reported(rig):
    limited(rig, j1=0.5)
    result = rig.run({'joints': {'j1': 1.0, 'j2': 0.3}, 'motion': {'speed_abs': {'joint_rad_s': 4.0}}})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert rig.reached_at('j1', 1.0) == pytest.approx(2.0, rel=0.05)   # 1.0 rad at the 0.5 rad/s limit, not 0.25 s
    assert result.data['motion']['capped'] == {'j1': 0.5}              # j2 has no limit: not mentioned


def test_a_speed_within_the_limits_reports_no_cap(rig):
    limited(rig, j1=5.0)
    result = rig.run({'joints': {'j1': 1.0}, 'motion': {'speed_abs': {'joint_rad_s': 1.0}}})
    assert result.outcome == Outcome.SUCCEEDED, result.message
    assert 'capped' not in result.data.get('motion', {})


# ---- through a skill server -------------------------------------------------------------------------------------

rclpy = pytest.importorskip('rclpy')

from conftest import RosRig  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from test_joints_ros import JointSim, run  # noqa: E402

from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402


@pytest.fixture
def system(tmp_path):
    with RosRig() as ros:
        config = tmp_path / 'config'
        shutil.copytree(FIXTURE, config)
        sim = ros.add(JointSim())
        server = SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value=ROBOT),
            Parameter('skills', value=['move_joints'])])
        ros.add(server)
        client = ros.add(rclpy.create_node('test_client'))
        ros.spin(8)
        time.sleep(0.3)
        yield sim, client


def test_skill_server_runs_move_joints(system):
    sim, client = system
    result = run(client, 'move_joints', {'pose': 'ready', 'motion': {'via': ['mid']}})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert sim.position['j1'] == pytest.approx(1.0, abs=0.02) and sim.position['lift'] == pytest.approx(0.2, abs=0.02)
    data = json.loads(result.data_json)
    assert data['world_state'] == {'arm_pose': 'ready'} and data['hands'] == ['left'] and data['mode'] == 'free'
    result = run(client, 'move_joints', {'pose': 'home', 'only': ['torso']})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert sim.position['lift'] == pytest.approx(0.0, abs=0.02) and sim.position['j1'] == pytest.approx(1.0, abs=0.02)
    result = run(client, 'move_joints', {'joints': {'j1': 3.0}})
    assert result.outcome == RunSkill.Result.REJECTED and 'joint limits' in result.message
