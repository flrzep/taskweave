"""move for held objects, the elbow and joints, against the two-arm simulator of test_move_hand_ros (joint targets in,
joint states out, a fake MoveIt IK: joint1 = the goal's x, joint2 = its z), on the schema-4 test cell: the rows of the
mapping table for move_object, set_posture and move_joints, the hands rules, dry runs and the messages."""

import json

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
pytest.importorskip('moveit_msgs')

from test_move_hand_ros import FAILED, OK, REJECTED, at, go, run, system  # noqa: E402, F401
from test_object_motion import FakeSwing, fake_fk  # noqa: E402


def hold_box(system, hands=None):
    """Both hands (or ``hands``) on the rim of crate_1 (a stack: taking one leaves it placed), grippers closed; the
    free move checks the hands with a rigid fake FK (the test robot has no URDF chains)."""
    system.move.modules['object']._tcp_fk = fake_fk()
    system.reanchors = []      # the test's TF is static (the hands never move there): re-anchoring would undo moves
    system.hold.reanchor = lambda name=None: system.reanchors.append(name) or True
    args = {'to': 'crate_1/rim', **({'hands': hands} if hands else {})}
    assert run(system, 'move', args).outcome == OK
    assert run(system, 'grip', {'action': 'close'}).outcome == OK
    return system.hold.held()


# ---- object ---------------------------------------------------------------------------------------------------

def test_every_target_in_linear_and_free(system):
    hold_box(system)
    pile_top = system.server.config.object_instances()['pile_1'].top_z + 0.1

    def origin():
        return system.hold.held()['object_in_base'][:3, 3]

    _, data = go(system, {'object': 'held', 'by': [0.0, 0.0, 0.05]})        # the default mode is linear
    assert data['mode'] == 'linear' and data['path'] == 'linear' and data['kind'] == 'object'
    assert data['module'] == 'move.object' and data['hands'] == ['left', 'right'] and data['held_object'] == 'crate_1'
    assert data['world_state'] == {'held_object': 'crate_1'}
    go(system, {'object': 'held', 'by': [0.0, 0.0, 0.0], 'turn_deg': 90.0, 'motion': {'mode': 'linear'}})
    at(system, 'left', 0.4, 0.35)
    at(system, 'right', 0.8, 0.35)
    assert np.allclose(system.hold.held()['object_in_base'][:2, :2], [[0, -1], [1, 0]], atol=1e-6)
    go(system, {'object': 'held', 'by': [0.0, 0.0, 0.0], 'turn_deg': -90.0, 'motion': {'mode': 'linear'}})
    for mode in ('linear', 'free'):
        x, z = origin()[0], origin()[2]
        _, data = go(system, {'object': 'held', 'by': [0.0, 0.0, 0.1], 'motion': {'mode': mode}})
        assert data['mode'] == mode and data['path'] == ('linear' if mode == 'linear' else 'direct')
        assert origin()[2] == pytest.approx(z + 0.1)
        at(system, 'left', x - 0.1, 0.4 + z)
        go(system, {'object': 'held', 'at': [0.7, 0.0, 0.2], 'motion': {'mode': mode}})
        assert origin() == pytest.approx([0.7, 0.0, 0.2])
        at(system, 'left', 0.6, 0.5)
        go(system, {'to': 'pile_1/top', 'reference': 'base', 'offset_m': 0.05, 'motion': {'mode': mode}})
        assert origin() == pytest.approx([0.6, 0.0, pile_top - 0.05])
        at(system, 'left', 0.5, 0.3 + pile_top - 0.05)
        result, data = go(system, {'to': 'pile_1/top', 'reference': 'base', 'motion': {'mode': mode}})
        assert origin() == pytest.approx([0.6, 0.0, pile_top])
        assert result.message.startswith('move object to pile_1/top (left, right): moved crate_1 base to pile_1/top')
        _, data = go(system, {'to': 'carry', 'motion': {'mode': mode}})
        assert system.sim.position['lift'] == pytest.approx(0.1, abs=0.03)
        at(system, 'left', 0.6, 0.4)
        assert origin() == pytest.approx([0.7, 0.0, 0.1])
        assert data['world_state'] == {'held_object': 'crate_1'} and data['mode'] == mode
        assert data['kind'] == 'object' and data['target'] == 'carry'
    for args, text in (({'to': 'carry', 'reference': 'x'}, 'option reference does not apply'),
                       ({'to': 'pile_1/top', 'reference': 'nope'}, "reference 'nope'"),
                       ({'to': 'arm_pose'}, 'sets arm joints'), ({'to': 'pile_1/rim'}, 'moves a hand')):
        result = run(system, 'move', args)
        assert result.outcome == REJECTED and text in result.message, result.message
    assert run(system, 'move', {'to': 'pile_1/top', 'reference': 'base', 'offset_m': 0.05}).outcome == OK


def test_a_carry_pose_with_two_waypoints_visits_both(system):
    hold_box(system)
    _, data = go(system, {'to': 'carry_steps'})
    assert json.loads(run(system, 'move', {'to': 'carry_steps'}).data_json)['kind'] == 'object'
    assert system.hold.held()['object_in_base'][:3, 3] == pytest.approx([0.7, 0.0, 0.2])
    assert data['hands'] == ['left', 'right']


def test_one_hand_holds_and_only_that_hand_moves(system):
    hold_box(system, ['left'])
    right = {k: v for k, v in system.sim.position.items() if k.startswith('right')}
    system.move.modules['object']._tcp_fk = fake_fk(scale_right_x=2.0)
    for mode in ('linear', 'free'):
        _, data = go(system, {'object': 'held', 'by': [0.0, 0.0, 0.05], 'motion': {'mode': mode}})
        assert data['hands'] == ['left']
    at(system, 'left', 0.5, 0.4)
    assert {k: v for k, v in system.sim.position.items() if k.startswith('right')} == right
    go(system, {'object': 'held', 'by': [0.2, 0.0, 0.0], 'hands': ['left'], 'motion': {'mode': 'free'}})
    go(system, {'object': 'held', 'by': [0.0, 0.0, 0.0], 'hands': 'all'})


def test_the_hands_of_an_object_move_must_be_the_hands_of_the_hold(system):
    result, _ = go(system, {'object': 'held', 'by': [0, 0, 0.1]}, REJECTED)
    assert result.message == 'move object: nothing is held: move to a hand target, then grip close'
    result, _ = go(system, {'to': 'carry'}, REJECTED)
    assert result.message.startswith('move object: nothing is held')
    hold_box(system)
    result, _ = go(system, {'object': 'held', 'by': [0, 0, 0.1], 'hands': ['left']}, REJECTED)
    assert result.message == 'move object: crate_1 is also held by right: include them or open them'
    assert run(system, 'grip', {'action': 'open', 'hands': ['right']}).outcome == OK
    result, _ = go(system, {'object': 'held', 'by': [0, 0, 0.1], 'hands': ['right']}, REJECTED)
    assert result.message == 'move object: right hold nothing (held: crate_1 by left)'
    result, _ = go(system, {'object': 'pile_1', 'by': [0, 0, 0.1]}, REJECTED)
    assert result.message.startswith('move object: pile_1 is not held')
    assert run(system, 'move', {'object': 'held', 'by': [0, 0, 0], 'hands': 'any_one'}).outcome == REJECTED
    system.hold.holds['pile_1'] = {'object': 'pile_1', 'point': 'pile_1/rim', 'hands': ['right'],
                                   'tcp_in_object': {'right': np.eye(4)}, 'object_in_base': np.eye(4)}
    result, _ = go(system, {'object': 'held', 'by': [0, 0, 0.1]}, REJECTED)
    assert result.message == 'move object: several objects held: name one with object:'


def test_a_target_for_another_object_type_says_what_is_needed(system, tmp_path):
    hold_box(system)
    (system.config / 'robots' / 'duo_bot' / 'targets.yaml').write_text(
        (system.config / 'robots' / 'duo_bot' / 'targets.yaml').read_text()
        + 'place_crate: {moves: object, type: crate, xyz: [0.7, 0.0, 0.1], rpy: [0.0, 0.0, 0.0]}\n')
    result, _ = go(system, {'to': 'place_crate'}, REJECTED)
    assert result.message == ("move object: 'place_crate' is a base-frame object target for crate: a held crate is "
                              'needed, a box is held')


def test_a_dry_run_of_an_object_move_solves_the_hands_and_moves_nothing(system):
    hold_box(system)
    before, origin = dict(system.sim.position), system.hold.held()['object_in_base'].copy()
    _, data = go(system, {'object': 'held', 'by': [0.0, 0.0, 0.1], 'dry_run': True})
    assert data['dry_run'] is True and data['kind'] == 'object'
    goal = data['joint_goal']['left_joint2']
    _, data = go(system, {'to': 'carry', 'dry_run': True})
    assert data['joint_goal']['lift'] == 0.1 and data['joint_goal']['left_joint1'] == pytest.approx(0.6, abs=0.01)
    assert system.sim.position == before and np.array_equal(system.hold.held()['object_in_base'], origin)
    go(system, {'object': 'held', 'by': [0.0, 0.0, 0.1]})                  # the real move ends where the dry run said
    assert system.sim.position['left_joint2'] == pytest.approx(goal, abs=0.03)
    system.ik.blocked = {'left'}
    result, _ = go(system, {'object': 'held', 'by': [0.0, 0.0, 0.1], 'dry_run': True}, FAILED)
    assert result.message.startswith('move object')


# ---- posture ----------------------------------------------------------------------------------------------------

def test_the_elbow_form_swings_the_named_hands(system, monkeypatch):
    result = run(system, 'move', {'elbow': 'out'})
    assert result.outcome == FAILED and 'URDF chains' in result.message      # the test robot has no URDF chains
    swing = FakeSwing(monkeypatch)
    result, data = go(system, {'elbow': 'out', 'motion': {'speed': 0.5, 'tolerance': 0.01}})
    assert swing.calls == [{'tools': ['left', 'right'], 'side': 'out'}]
    assert data['hands'] == ['left', 'right'] and data['path'] == 'swing' and data['kind'] == 'posture'
    assert result.message.startswith('move elbow out (left, right): ')
    swing.calls.clear()
    go(system, {'elbow': 'down', 'hands': ['right']})
    assert swing.calls == [{'tools': ['right'], 'side': 'down'}]
    swing.calls.clear()
    go(system, {'elbow': {'left': 'out'}})                                  # {elbow: {left: out}}
    assert swing.calls == [{'tools': ['left'], 'side': 'out'}]
    swing.calls.clear()
    go(system, {'elbow': {'left': 'out', 'right': 'in'}})
    assert swing.calls == [{'tools': ['left'], 'side': 'out'}, {'tools': ['right'], 'side': 'in'}]
    FakeSwing(monkeypatch, after_deg=60, why='left_joint2 at its limit')
    result, _ = go(system, {'elbow': 'out'}, FAILED)
    assert result.message.startswith('move elbow out (') or result.message.startswith('move elbow out:')
    go(system, {'elbow': 'out', 'tolerance_deg': 70.0})
    assert run(system, 'move', {'elbow': 'out', 'motion': {'mode': 'free'}}).outcome == REJECTED


def test_the_elbow_form_of_a_hold_swings_the_holding_hands_only(system, monkeypatch):
    hold_box(system, ['left'])
    swing = FakeSwing(monkeypatch)
    _, data = go(system, {'elbow': 'up'})
    assert swing.calls == [{'tools': ['left'], 'side': 'up'}] and data['hands'] == ['left']


# ---- joints -----------------------------------------------------------------------------------------------------

def test_a_joint_target_goes_to_its_group_values_and_only_limits_it(system):
    result, data = go(system, {'to': 'lift_up'})
    assert system.sim.position['lift'] == pytest.approx(0.1, abs=0.02)
    assert data['kind'] == 'joints' and data['module'] == 'move.joints' and data['target'] == 'lift_up'
    assert data['world_state'] == {'arm_pose': 'lift_up'} and result.message.startswith('move joints lift_up: ')
    go(system, {'to': 'head_up'})
    at(system, 'left', 0.3, 0.0)
    go(system, {'to': 'home', 'only': ['torso']})
    assert system.sim.position['lift'] == pytest.approx(0.0, abs=0.02)
    at(system, 'left', 0.3, 0.0)                                              # only the torso went
    result, _ = go(system, {'to': 'lift_up', 'only': ['left_arm']}, REJECTED)
    assert 'no joints of the target are in left_arm' in result.message


def test_the_joints_form_takes_group_values(system):
    go(system, {'joints': {'torso': 0.15, 'left_arm': [0.2, None], 'right_arm': [0.1, 0.1]}})
    assert system.sim.position['lift'] == pytest.approx(0.15, abs=0.02)
    at(system, 'left', 0.2, 0.0)
    at(system, 'right', 0.1, 0.1)
    result, _ = go(system, {'joints': {'wings': 1}}, REJECTED)
    assert "joint group 'wings'" in result.message and result.message.startswith('move joints: ')
    result, _ = go(system, {'joints': {'left_arm': 0.2}}, REJECTED)
    assert "joint group 'left_arm' has 2 joints" in result.message


def test_a_joint_move_goes_through_its_via_targets_first(system):
    go(system, {'to': 'lift_up'})
    system.sim.history.clear()
    go(system, {'to': 'head_up', 'motion': {'via': ['home']}})
    assert min(h['lift'] for h in system.sim.history if 'lift' in h) < 0.05


def test_a_dry_run_of_a_joint_move_returns_the_goal(system):
    before = dict(system.sim.position)
    _, data = go(system, {'to': 'home', 'only': ['left_arm'], 'dry_run': True})
    assert data['joint_goal'] == {'left_joint1': 0.0, 'left_joint2': 0.0} and data['dry_run'] is True
    _, data = go(system, {'joints': {'torso': 0.2}, 'dry_run': True})
    assert data['joint_goal'] == {'lift': 0.2}
    assert system.sim.position == before
    result, _ = go(system, {'elbow': 'out', 'dry_run': True}, REJECTED)
    assert 'dry_run is not available' in result.message
