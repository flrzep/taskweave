"""move_object and set_posture against the two-arm test simulator of test_gripper_motion (joint targets in, joint states
out, a fake MoveIt IK: joint1 = the goal's x, joint2 = its z). Targets x modes, one and two hands, the hands rules, the
grip-drift check of free moves and the elbow swing (the URDF chains are faked where a test needs them)."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
pytest.importorskip('moveit_msgs')

from test_gripper_motion import FLANGES, hold_by_registry, object_x, run, shifted, system  # noqa: E402, F401

from taskweave_core.validation import oneof_problems, validate_args  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402

OK, FAILED, REJECTED = RunSkill.Result.SUCCEEDED, RunSkill.Result.FAILED, RunSkill.Result.REJECTED
HANDS = ('left', 'right')


def fake_fk(scale_right_x: float = 1.0):
    """Forward kinematics of the test arms: x = joint1, z = joint2, y = the flange's place. ``scale_right_x`` != 1 makes
    the right hand run away from the left one when both joints move equally."""
    def fk(positions, hands):
        out = {}
        for hand in hands:
            pose = np.eye(4)
            scale = scale_right_x if hand == 'right' else 1.0
            pose[:3, 3] = [scale * positions[f'{hand}_joint1'], FLANGES[hand][1], positions[f'{hand}_joint2']]
            out[hand] = pose
        return out
    return fk


def move(system, to, outcome=OK, **rest):
    result = run(system, 'move_object', {'to': to, **rest})
    assert result.outcome == outcome, result.message
    return result, json.loads(result.data_json)


def hold_box(system, hands=None):
    """Both hands (or ``hands``) on the rim of crate_1 (a stack: taking one leaves it placed), grippers closed. The
    free move checks the hands with a rigid fake FK (the test robot has no URDF chains)."""
    system.server.skills['move_object']._tcp_fk = fake_fk()
    args = {'to': {'point': 'crate_1/rim'}, **({'hands': hands} if hands else {})}
    assert run(system, 'move_gripper', args).outcome == OK
    assert run(system, 'grip', {'action': 'close'}).outcome == OK
    return system.hold.held()


def at(system, hand, joint1, joint2, tolerance=0.03):
    position = system.sim.position
    assert position[f'{hand}_joint1'] == pytest.approx(joint1, abs=tolerance)
    assert position[f'{hand}_joint2'] == pytest.approx(joint2, abs=tolerance)


# ---- schemas ----------------------------------------------------------------------------------------------------

def test_the_schemas_pass_the_oneof_rule_and_carry_the_robots_hands(system):
    from taskweave_skills.object_motion import MoveObject, SetPosture

    for schema in (MoveObject.args_schema, SetPosture.args_schema,
                   *(system.server.skills[name].args_schema for name in ('move_object', 'set_posture'))):
        assert oneof_problems(schema, allow_several=True) == []
    skill, posture = system.server.skills['move_object'], system.server.skills['set_posture']
    props = skill.args_schema['properties']
    assert [v['required'] for v in props['to']['oneOf']] == [['by'], ['at'], ['pose'], ['onto', 'contact']]
    assert props['hands']['anyOf'][0]['enum'] == ['auto', 'all', 'left', 'right']        # no any_one here
    assert list(props['motion']['properties']) == ['mode', 'speed', 'speed_abs', 'accel', 'planner', 'tolerance']
    assert list(posture.args_schema['properties']['motion']['properties']) == ['speed', 'speed_abs', 'accel',
                                                                                'tolerance']
    assert skill.implementation == 'script:ik_held' and posture.implementation == 'script:null_space'
    assert skill.args_schema['x-taskweave-resources'] == ['hands', 'torso']
    store, robot = system.server.config, 'duo_bot'
    ok = {'to': {'pose': 'carry'}, 'object': 'crate_1'}
    assert validate_args(ok, skill.args_schema, store, robot) == []
    problems = validate_args({'to': {'pose': 'nope'}, 'object': 'nobody'}, skill.args_schema, store, robot)
    assert any(p.startswith('args.to.pose') and 'no held pose' in p for p in problems)
    assert any(p.startswith('args.object') for p in problems)
    assert validate_args({'to': {'by': [0, 0, 1], 'at': [0, 0, 1]}}, skill.args_schema, store, robot)   # ambiguous


def test_move_object_args_are_the_new_ones_only():
    from taskweave_skills import object_motion

    assert object_motion.MoveObject.name == 'move_object' and object_motion.MoveObject.args_schema['required'] == ['to']
    assert 'by' not in object_motion.MoveObject.args_schema['properties']


# ---- targets x modes --------------------------------------------------------------------------------------------
# (a ROS fixture per test costs file descriptors: one test goes through many moves)

def test_every_target_in_linear_and_free(system):
    hold_box(system)
    pile_top = system.server.config.object_instances()['pile_1'].top_z + 0.1       # z of the top contact of pile_1

    def origin():
        return system.hold.held()['object_in_base'][:3, 3]

    _, data = move(system, {'by': [0.0, 0.0, 0.05]})                       # the default mode is linear
    assert data['mode'] == 'linear' and data['motion']['mode'] == 'linear' and data['path'] == 'linear'
    assert data['hands'] == ['left', 'right'] and data['held_object'] == 'crate_1'
    assert data['world_state'] == {'held_object': 'crate_1'}
    assert data['speed']['override_avg'] == pytest.approx(100.0, abs=1.0)
    move(system, {'by': [0.0, 0.0, 0.0], 'turn_deg': 90.0}, motion={'mode': 'linear'})
    # the hands are 0.1 behind and 0.2 beside the object origin (0.6, 0): turned 90 deg left about it
    at(system, 'left', 0.4, 0.35)
    at(system, 'right', 0.8, 0.35)
    assert np.allclose(system.hold.held()['object_in_base'][:2, :2], [[0, -1], [1, 0]], atol=1e-6)
    move(system, {'by': [0.0, 0.0, 0.0], 'turn_deg': -90.0}, motion={'mode': 'linear'})
    for mode in ('linear', 'free'):
        x, z = origin()[0], origin()[2]
        _, data = move(system, {'by': [0.0, 0.0, 0.1]}, motion={'mode': mode})
        assert data['mode'] == mode and data['path'] == ('linear' if mode == 'linear' else 'direct')
        assert origin()[2] == pytest.approx(z + 0.1)
        at(system, 'left', x - 0.1, 0.4 + z)
        at(system, 'right', x - 0.1, 0.4 + z)
        move(system, {'at': [0.7, 0.0, 0.2]}, motion={'mode': mode})
        assert origin() == pytest.approx([0.7, 0.0, 0.2])
        at(system, 'left', 0.6, 0.5)
        at(system, 'right', 0.6, 0.5)
        move(system, {'onto': 'pile_1/top', 'contact': 'base', 'offset_m': 0.05}, motion={'mode': mode})
        assert origin() == pytest.approx([0.6, 0.0, pile_top - 0.05])
        at(system, 'left', 0.5, 0.3 + pile_top - 0.05)
        move(system, {'onto': 'pile_1/top', 'contact': 'base'}, motion={'mode': mode})
        assert origin() == pytest.approx([0.6, 0.0, pile_top])
        _, data = move(system, {'pose': 'carry'}, motion={'mode': mode})
        assert system.sim.position['lift'] == pytest.approx(0.1, abs=0.03)
        at(system, 'left', 0.6, 0.4)
        assert origin() == pytest.approx([0.7, 0.0, 0.1])
        assert data['world_state'] == {'held_object': 'crate_1'} and data['mode'] == mode
    for to, text in (({'onto': 'pile_1/nope', 'contact': 'base'}, 'nope'),
                     ({'onto': 'pile_1/top', 'contact': 'nope'}, 'nope'),
                     ({'pose': 'arm_pose'}, 'sets arm joints'), ({'pose': 'nope'}, 'no held pose')):
        result, _ = move(system, to, REJECTED)
        assert text in result.message


# ---- hands ------------------------------------------------------------------------------------------------------

def test_one_hand_holds_and_only_that_hand_moves(system):
    hold_box(system, ['left'])
    assert system.hold.hand_holds == {'left': 'crate_1'}
    right = {k: v for k, v in system.sim.position.items() if k.startswith('right')}
    system.server.skills['move_object']._tcp_fk = fake_fk(scale_right_x=2.0)    # would fail with two hands
    for mode in ('linear', 'free'):
        _, data = move(system, {'by': [0.0, 0.0, 0.05]}, motion={'mode': mode})
        assert data['hands'] == ['left']
    at(system, 'left', 0.5, 0.4)
    assert {k: v for k, v in system.sim.position.items() if k.startswith('right')} == right
    move(system, {'by': [0.2, 0.0, 0.0]}, hands=['left'], motion={'mode': 'free'})      # no drift check for one hand
    at(system, 'left', 0.7, 0.4)
    move(system, {'by': [0.0, 0.0, 0.0]}, hands='all')       # all: the hands of the hold, not every hand


def test_a_hands_value_must_be_exactly_the_hands_of_the_hold(system):
    result, _ = move(system, {'by': [0, 0, 0.1]}, REJECTED)
    assert result.message == 'nothing is held: move_gripper to the object, then grip close'
    hold_box(system)
    result, _ = move(system, {'by': [0, 0, 0.1]}, REJECTED, hands=['left'])
    assert result.message == 'crate_1 is also held by right: include them or open them'
    move(system, {'by': [0, 0, 0.0]}, hands=['right', 'left'])
    assert system.sim.position['left_joint2'] == pytest.approx(0.3, abs=0.03)       # the rejected one did not move
    assert run(system, 'grip', {'action': 'open', 'hands': ['right']}).outcome == OK
    result, _ = move(system, {'by': [0, 0, 0.1]}, REJECTED, hands=['right'])
    assert result.message == 'right hold nothing (held: crate_1 by left)'
    result, _ = move(system, {'by': [0, 0, 0.1]}, REJECTED, hands=['left', 'right'])
    assert result.message == 'right does not hold crate_1: hands must be the hands that hold it (left)'
    result, _ = move(system, {'by': [0, 0, 0.1]}, REJECTED, object='pile_1')
    assert result.message.startswith('pile_1 is not held')
    assert run(system, 'move_object', {'to': {'by': [0, 0, 0]}, 'hands': 'any_one'}).outcome == REJECTED
    # a second object held by the right hand (put in the registry directly): the free right hand could reach it,
    # the left hand holds crate_1 and stays with it
    assert run(system, 'move_gripper', {'to': {'point': 'pile_1/rim'}, 'hands': ['left']}).outcome == REJECTED
    system.hold.holds['pile_1'] = {'object': 'pile_1', 'point': 'pile_1/rim', 'hands': ['right'],
                                   'tcp_in_object': {'right': np.eye(4)}, 'object_in_base': np.eye(4)}
    result, _ = move(system, {'by': [0, 0, 0.1]}, REJECTED)
    assert result.message == 'several objects held: name one with object:'
    _, data = move(system, {'by': [0, 0, 0.05]}, hands=['left'])        # the hands name the hold
    assert data['held_object'] == 'crate_1' and data['hands'] == ['left']


# ---- free mode keeps the grip ------------------------------------------------------------------------------------

def test_free_with_two_hands_checks_the_grip_and_the_other_options(system):
    from dataclasses import replace

    from taskweave_skills.joints import Params

    assert Params().grip_drift_m == 0.02
    hold_box(system)
    skill = system.server.skills['move_object']
    skill._tcp_fk = fake_fk(scale_right_x=2.0)                 # the right hand runs off when both joints go the same
    before = dict(system.sim.position)
    result, _ = move(system, {'by': [0.2, 0.0, 0.0]}, FAILED, motion={'mode': 'free'})
    assert 'the hands would drift apart on a free path' in result.message and 'use mode linear' in result.message
    assert system.sim.position == before                        # nothing moved
    assert system.hold.held()['object_in_base'][0, 3] == pytest.approx(0.6)
    move(system, {'by': [0.0, 0.0, 0.0]}, motion={'mode': 'linear'})    # the straight lines do not check (rigid)
    limit = skill.p                                             # the limit is the skill's parameter
    skill.p = replace(skill.p, grip_drift_m=0.2)               # 0.086 m of drift here: fine within 0.2
    move(system, {'by': [0.2, 0.0, 0.0]}, motion={'mode': 'free'})
    skill.p = limit
    del skill._tcp_fk                                          # the real forward kinematics: this robot has no chains
    result, _ = move(system, {'by': [0.0, 0.0, 0.1]}, REJECTED, motion={'mode': 'free'})
    assert 'URDF chains' in result.message and 'use mode linear' in result.message
    assert system.hold.held()['object_in_base'][2, 3] == pytest.approx(0.0)
    # tolerance: the test's TF does not follow the joints, so the hands are always 0.1 m from the goal
    result, _ = move(system, {'by': [0.0, 0.0, 0.1]}, FAILED, motion={'tolerance': 0.01})
    assert 'm off' in result.message and 'tolerance 0.01 m' in result.message
    _, data = move(system, {'by': [0.0, 0.0, 0.0]}, motion={'speed': 0.5, 'accel': 0.5})
    assert data['motion']['speed'] == 0.5


# ---- set_posture ------------------------------------------------------------------------------------------------

class FakeSwing:
    """Stands in for cartesian.elbow_swing_path: every tool's joint2 +0.1, reporting where the elbows end."""

    def __init__(self, monkeypatch, after_deg=5, why='', path=True):
        from taskweave_skills import cartesian
        self.calls, self.after, self.why, self.path = [], after_deg, why, path
        monkeypatch.setattr(cartesian, 'elbow_swing_path', self)

    def __call__(self, ik, tools, current, state, side, joint_speed, settled=None):
        from taskweave_skills.motion_plan import Trajectory
        self.calls.append({'tools': list(tools), 'side': side})
        names = sorted(f'{tool}_joint2' for tool in tools)
        report = {tool: (90, self.after, self.why) for tool in tools}
        if not self.path:
            return None, {tool: (3, 3, '') for tool in tools}
        return Trajectory(names, [0.0, 0.5], [[current[n] for n in names], [current[n] + 0.1 for n in names]]), report


def posture(system, args, outcome=OK):
    result = run(system, 'set_posture', args)
    assert result.outcome == outcome, result.message
    return result, json.loads(result.data_json)


def test_set_posture_without_a_hold_swings_every_hand(system, monkeypatch):
    result, _ = posture(system, {'elbow': 'out'}, FAILED)             # the test robot has no URDF chains
    assert 'URDF chains' in result.message
    assert run(system, 'set_posture', {'elbow': 'sideways'}).outcome == REJECTED
    assert run(system, 'set_posture', {}).outcome == REJECTED
    assert run(system, 'set_posture', {'elbow': 'out', 'motion': {'mode': 'free'}}).outcome == REJECTED   # no mode
    swing = FakeSwing(monkeypatch)
    _, data = posture(system, {'elbow': 'out', 'motion': {'speed': 0.5, 'tolerance': 0.01}})
    assert swing.calls == [{'tools': ['left', 'right'], 'side': 'out'}]
    assert data['hands'] == ['left', 'right'] and data['path'] == 'swing' and data['motion']['speed'] == 0.5
    assert data['report'] == {'left': {'from_deg': 90, 'to_deg': 5}, 'right': {'from_deg': 90, 'to_deg': 5}}
    assert system.sim.position['left_joint2'] == pytest.approx(0.1, abs=0.03)
    swing.calls.clear()
    _, data = posture(system, {'elbow': 'down', 'hands': ['right']})
    assert swing.calls == [{'tools': ['right'], 'side': 'down'}] and data['hands'] == ['right']
    FakeSwing(monkeypatch, after_deg=60, why='left_joint2 at its limit')           # the swing would stop short
    before = dict(system.sim.position)
    result, _ = posture(system, {'elbow': 'out'}, FAILED)
    assert result.message == ('elbows of left and right out: the elbows would stop more than 20 deg off out: '
                              'left 90 -> 60 deg off (left_joint2 at its limit), '
                              'right 90 -> 60 deg off (left_joint2 at its limit); not moved')
    assert system.sim.position == before
    posture(system, {'elbow': 'out', 'tolerance_deg': 70.0})                  # the step may allow more
    FakeSwing(monkeypatch, path=False)
    result, data = posture(system, {'elbow': 'out'})
    assert 'already as far as they go' in result.message and data['path'] == 'none'


def test_set_posture_with_a_hold_swings_the_holding_hands_and_the_object_stays(system, monkeypatch):
    hold_box(system, ['left'])
    object_before = system.hold.held()['object_in_base'].copy()
    swing = FakeSwing(monkeypatch)
    _, data = posture(system, {'elbow': 'up'})
    assert swing.calls == [{'tools': ['left'], 'side': 'up'}] and data['hands'] == ['left']
    assert np.array_equal(system.hold.held()['object_in_base'], object_before)
    assert system.hold.hand_holds == {'left': 'crate_1'}
    swing.calls.clear()
    posture(system, {'elbow': 'up', 'hands': 'all'})
    assert swing.calls[0]['tools'] == ['left', 'right']


def test_the_short_stop_message_of_the_real_swing_on_a_chain(tmp_path):
    """_swing_plan on a real URDF chain (as the old move_object elbow test): the wrist reaches its limit halfway."""
    from types import SimpleNamespace

    from test_cartesian import ChainIk
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.chain import Chain
    from taskweave_skills.joints import Params
    from taskweave_skills.object_motion import SetPosture

    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    ik = ChainIk(Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW), NAMES)
    ik.joint_pattern = 'j[1-7]'
    skill = SetPosture.__new__(SetPosture)
    skill.ik, skill.p = ik, Params(max_joint_speed=1.0, elbow_tolerance_deg=20.0)
    skill.joints = SimpleNamespace(start=lambda max_age, tolerance: dict(START))
    ctx = SimpleNamespace(should_stop=False, feedback=lambda fraction, text: None)
    failure, trajectory, report, _ = skill._swing_plan(ctx, ['arm'], 'down', 'elbows of arm down')
    assert failure.outcome.name == 'FAILED' and trajectory is None
    assert failure.message == ('elbows of arm down: the elbows would stop more than 20 deg off down: arm 173 -> 53 deg '
                               'off (j5 at its limit); not moved')
    failure, trajectory, report, swung = skill._swing_plan(ctx, ['arm'], 'down', 'elbows of arm down', 180.0)
    assert failure is None and trajectory is not None and report['arm'][1] == 53


def test_the_docs_examples_load():
    import yaml
    from jsonschema import Draft202012Validator

    from taskweave_core.config import schemas
    from taskweave_skills.gripper_motion import translate_params
    from taskweave_skills.joints import Params

    docs = Path(__file__).resolve().parents[2] / 'docs' / 'examples' / 'skills'
    if not docs.is_dir():
        pytest.skip('docs/ not next to the packages (the test runner ships the packages only)')
    for name, key in (('move_object', 'grip_drift_m'), ('set_posture', 'elbow_tolerance_deg')):
        data = yaml.safe_load((docs / f'{name}.yaml').read_text(encoding='utf-8'))
        Draft202012Validator(schemas.SKILL_PARAMS).validate(data)
        assert {i['class'] for i in data['implementations']} == {
            f'taskweave_skills.object_motion:{"MoveObject" if name == "move_object" else "SetPosture"}'}
        flat, extra = translate_params({k: v for k, v in data.items() if k != 'implementations'})
        for unused in ('cell_frame', 'lift_joint', 'lift_values', 'tcp_speed', 'tcp_turn'):
            flat.pop(unused, None)
        assert key in flat and extra['accel']
        assert Params.from_config(flat).position_tolerance == 0.03
    assert math.isclose(Params.from_config({'grip_drift_m': 0.05}).grip_drift_m, 0.05)


def test_a_lift_segment_of_a_held_pose_updates_the_other_hold_too(system):       # I2
    hold_by_registry(system, 'left', 'box_1')
    hold_by_registry(system, 'right', 'pile_1')
    shifted(system, 'pile_1')                                  # the registry's pile_1 is off: only a follow fixes it
    move(system, {'pose': 'carry'}, object='box_1', motion={'mode': 'linear'})
    assert system.sim.position['lift'] == pytest.approx(0.1, abs=0.03)
    assert object_x(system, 'pile_1') == pytest.approx(0.0, abs=1e-6), 'the other hold was not followed'
