"""move without a robot: what a step resolves to (the part, the kind, the calls it makes) with stub parts, over the
schema-5 test cell (fixtures/config). Every row of the mapping table of the design (old args -> move), every
REJECTED line, the torso order, the schema and the default timeouts. No ROS: the parts are stubs that remember what they
were called with, so these tests show which targets the real parts are handed (test_move_*_ros run the motions)."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore, Pose2D
from taskweave_core.config.poses import Pose3D
from taskweave_core.config.schemas import ELBOW_SIDES
from taskweave_core.config.targets import Target
from taskweave_core.executive import Outcome
from taskweave_core.skill import SkillResult
from taskweave_core.validation import oneof_problems, validate_args
from taskweave_skills.move import Move
from taskweave_skills.parts import PARTS, RESOLVED

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'


class StubHold:
    def __init__(self, holds=None):
        self.holds = holds or []
        self.hand_holds = {hand: h['object'] for h in self.holds for hand in h['hands']}

    def held_all(self):
        return self.holds

    def type_of(self, hold, instances):
        return hold.get('type')

    def transform(self, parent, child):
        raise LookupError('no transform in this test')


class StubPart:
    """What Move needs of a part: ``_hold``, ``hold``, ``_current``, a timeout; records its calls."""

    def __init__(self, hold=None, pose=None, timeout=60.0):
        self.hold, self.pose, self.default_timeout_s, self.calls = hold or StubHold(), pose, timeout, []

    def _current(self):
        return self.pose

    def _hold(self, args):
        if not self.hold.holds:
            return SkillResult.rejected('nothing is held: move to a hand target, then grip close')
        return self.hold.holds[0]

    def execute(self, ctx, args):
        self.calls.append(args)
        return SkillResult.succeeded('done', hands=['left'])


@pytest.fixture
def make(tmp_path):
    root = tmp_path / 'config'
    shutil.copytree(FIXTURE, root)
    store = ConfigStore(root)

    def build(robot='duo_bot', parts=tuple(PARTS), held=None, pose=None):
        move = Move.__new__(Move)
        move.config, move.robot, move._trial, move._used = store, store.robot(robot), False, []
        move.hands = list(move.robot.data.get('grippers', {}))
        hold = StubHold(held)
        timeouts = {'navigate': 120.0}
        move.modules = {part: StubPart(hold, pose, timeouts.get(part, 60.0)) for part in parts}
        move.unavailable = {part: ['needs arms.bimanual'] for part in PARTS if part not in parts}
        move.name = 'move'
        move._scene_problem = lambda hold: None      # the held shape is attached in the planning scene
        return move
    return build


def plan(move, **args):
    result = move._plan(args)
    return result


def rejected(result, *fragments):
    assert isinstance(result, SkillResult), f'not rejected: {result}'
    assert result.outcome == Outcome.REJECTED
    for fragment in fragments:
        assert fragment in result.message, result.message
    return result.message


# ---- the schema ---------------------------------------------------------------------------------------------

def test_the_schema_passes_the_oneof_rule_and_every_form_of_the_design_validates(make):
    assert oneof_problems(Move.args_schema, allow_several=True) == []
    move = make()
    schema = Move.schema_for(move.hands)
    steps = [{'to': 'dock'}, {'to': 'box_1/rim', 'offset_m': 0.15}, {'to': 'carry'}, {'to': 'home'},
             {'hand': 'left', 'by': [0, 0, 0.1]}, {'object': 'held', 'by': [0, 0, 0.3]},
             {'hand': 'holding', 'by': [0, 0, 0.3]}, {'hand': 'holding', 'at': [0.5, 0, 1], 'object': 'box_1'},
             {'hand': 'holding', 'by': [0, 0, 0.3], 'hands': 'all'},
             {'base': {'by': [-0.5, 0]}}, {'elbow': {'left': 'out'}}, {'joints': {'torso': 0.3}},
             {'to': 'box_1/top', 'object': 'box_1', 'reference': 'base'}, {'elbow': 'down', 'hands': 'left'}]
    for step in steps:
        assert validate_args(step, schema, move.config, 'duo_bot') == [], step


def test_a_step_with_no_form_or_two_forms_or_an_unknown_target_does_not_validate(make):
    move = make()
    check = lambda step: validate_args(step, Move.schema_for(move.hands), move.config, 'duo_bot')   # noqa: E731
    assert check({'motion': {'speed': 0.5}})
    assert check({'to': 'dock', 'hand': 'left', 'by': [0, 0, 0]})
    assert check({'hand': 'left'})                                    # by or at
    assert any('unknown target' in problem for problem in check({'to': 'mars'}))
    assert any('args.to' in problem for problem in check({'to': 'mars'}))
    assert check({'to': 'dock', 'bogus': 1})
    assert check({'base': {'by': [1]}})


def test_the_hands_of_the_robot_fill_the_schema_enums(make):
    move = make()
    schema = Move.schema_for(move.hands)
    assert schema['properties']['hand']['anyOf'][0]['enum'] == ['all', 'holding', 'left', 'right']
    assert schema['properties']['hands']['anyOf'][0]['enum'] == ['auto', 'any_one', 'all', 'left', 'right']
    props = Move.args_schema['properties']
    assert set(props) >= {'to', 'hand', 'object', 'base', 'elbow', 'joints', 'hands', 'offset_m', 'reference', 'by',
                          'at', 'rpy_deg', 'frame', 'turn_deg', 'tilt_deg', 'only', 'torso', 'tolerance_deg',
                          'motion', 'look_at', 'dry_run'}
    assert props['motion']['properties']['mode']['enum'] == ['nav', 'straight', 'free', 'linear']
    assert props['motion']['properties']['via']['items']['x-taskweave-point'] == 'target'
    assert props['elbow']['anyOf'][0]['enum'] == ELBOW_SIDES


# ---- the mapping table: every old step as a move step ---------------------------------------------------------

def test_navigate_to_a_location_or_stance_is_move_to_a_base_target(make):
    move = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    result = plan(move, to='dock')
    assert result.part == 'navigate' and result.kind == 'navigate' and result.target == 'dock'
    assert result.desc == 'dock'
    [call] = result.calls
    assert call['target'] == 'dock' and call[RESOLVED]['pose'] == Pose2D(1.0, 0.5, 1.5708, 'cell')
    assert call[RESOLVED]['torso'] is None            # the profile's drive_m applies


def test_a_stance_on_a_placed_object_is_composed_with_its_pose(make, tmp_path):
    root = tmp_path / 'config'
    (root / 'robots' / 'diff_bot' / 'objects').mkdir()
    (root / 'robots' / 'diff_bot' / 'objects' / 'box.yaml').write_text(
        'targets:\n  pick: {moves: base, x: -0.8, y: 0.0, yaw: 0.0}\n')
    move = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    result = plan(move, to='box_1/pick')
    [call] = result.calls
    assert call[RESOLVED]['pose'].x == pytest.approx(0.6 - 0.8) and call[RESOLVED]['pose'].y == pytest.approx(0.0)
    assert call['target'] == 'box_1/pick'


def test_drive_straight_rows(make):
    move = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    to = plan(move, to='dock', motion={'mode': 'straight'})            # drive_straight {to: T}
    assert to.part == 'straight' and to.calls[0]['to'] == 'dock'
    assert to.calls[0][RESOLVED]['pose'] == Pose2D(1.0, 0.5, 1.5708, 'cell')
    by = plan(move, base={'by': [0.4, 0]})                              # drive_straight {distance_m: d}
    assert by.part == 'straight' and by.calls == [{'distance_m': 0.4, RESOLVED: {'torso': None}}]
    side = plan(move, base={'by': [0.4, 0.2]})                          # {distance_m, sideways_m}
    assert side.calls[0]['distance_m'] == 0.4 and side.calls[0]['sideways_m'] == 0.2
    assert plan(move, base={'by': [-0.5, 0]}).calls[0]['distance_m'] == -0.5


def test_move_gripper_rows_to_a_point_a_pose_and_relative(make):
    move = make()
    point = plan(move, to='box_1/rim', offset_m=0.15, hands='left')
    assert point.part == 'hand' and point.target == 'box_1/rim'
    [call] = point.calls
    resolved = call[RESOLVED]
    assert isinstance(resolved['target'], Target) and resolved['target'].frame == 'object'
    assert resolved['offset_m'] == 0.15 and call['hands'] == 'left'
    assert resolved['data'] == {'approach_m': 0.1}
    pose = plan(move, to='present_left')                                # move_gripper {to: {pose: N}}
    assert pose.calls[0][RESOLVED]['target'].frame == 'base' and pose.calls[0][RESOLVED]['data'] == {'elbow': 'out'}
    assert 'hands' not in pose.calls[0]
    by = plan(move, hand='left', by=[0, 0, 0.1], frame='tool', turn_deg=5, tilt_deg=2)   # to: {by, ...}, hands
    assert by.part == 'hand' and by.calls[0]['to'] == {'by': [0, 0, 0.1], 'frame': 'tool', 'turn_deg': 5,
                                                       'tilt_deg': 2}
    assert by.calls[0]['hands'] == 'left'
    at = plan(move, hand='right', at=[0.5, 0, 0.3], rpy_deg=[0, 0, 90])
    assert at.calls[0]['to'] == {'at': [0.5, 0, 0.3], 'rpy_deg': [0, 0, 90]} and at.calls[0]['hands'] == 'right'
    assert plan(move, hand='all', by=[0, 0, 0.1]).calls[0]['hands'] == 'all'
    assert plan(move, hand=['left', 'right'], by=[0, 0, 0.1]).calls[0]['hands'] == ['left', 'right']


def test_move_gripper_motion_torso_becomes_the_torso_option(make):
    move = make()
    call = plan(move, to='box_1/rim', torso='keep', motion={'mode': 'linear', 'speed': 0.5}).calls[0]
    assert call['motion'] == {'mode': 'linear', 'speed': 0.5, 'torso': 'keep'}     # move_gripper {motion: {torso}}
    call = plan(move, to='box_1/rim').calls[0]
    assert 'torso' not in call['motion']      # nothing said: skills/move/hand.yaml's motion.torso (auto) holds


def test_move_object_rows(make):
    held = [{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}]
    move = make(held=held)
    by = plan(move, object='held', by=[0, 0, 0.3], turn_deg=10, tilt_deg=3)        # move_object {to: {by, ...}}
    assert by.part == 'carry' and by.kind == 'hand' and by.carrying == 'box_1'
    assert by.calls[0]['to'] == {'by': [0, 0, 0.3], 'turn_deg': 10, 'tilt_deg': 3}
    assert 'object' not in by.calls[0]                                           # held: the only hold
    holding = plan(move, hand='holding', by=[0, 0, 0.3], turn_deg=10, tilt_deg=3)    # schema 5: the same move
    assert holding.part == 'carry' and holding.calls == by.calls and holding.desc == by.desc
    assert plan(move, object='box_1', at=[0.5, 0, 0.3]).calls[0]['object'] == 'box_1'    # {object: name}
    assert plan(move, object='held', at=[0.5, 0, 0.3]).calls[0]['to'] == {'at': [0.5, 0, 0.3]}
    assert plan(move, hand='holding', at=[0.5, 0, 0.3], object='box_1').calls[0]['object'] == 'box_1'
    pose = plan(move, to='carry')                                              # move_object {to: {pose: N}}
    assert pose.part == 'carry' and pose.kind == 'hand' and pose.carrying == 'box_1'
    [call] = pose.calls
    held_pose = call[RESOLVED]['held']
    assert call['to'] == {'pose': 'carry'} and held_pose['joints'] == {'lift': 0.1}
    assert held_pose['path'] == [Pose3D((0.7, 0.0, 0.1), (0.0, 0.0, 0.0), 'base')]
    assert [p.xyz for p in plan(move, to='carry_steps').calls[0][RESOLVED]['held']['path']] == [(0.7, 0.0, 0.1),
                                                                                                (0.7, 0.0, 0.2)]
    onto = plan(move, to='pile_1/top', reference='base', offset_m=0.05)       # {onto: X/C, contact: K, offset_m}
    [call] = onto.calls
    assert call['to'] == {'onto': 'pile_1/top', 'contact': 'base', 'offset_m': 0.05}
    resolved = call[RESOLVED]['onto']
    assert resolved['instance'].name == 'pile_1' and resolved['pose'] == Pose3D((0, 0, 0.1), (0, 0, 0), 'box')
    assert resolved['mine'].xyz == (0.0, 0.0, 0.0)


def test_move_object_hands_and_motion_pass_through(make):
    held = [{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}]
    move = make(held=held)
    call = plan(move, object='held', by=[0, 0, 0.1], hands='all', torso='keep', motion={'mode': 'free'}).calls[0]
    assert call['hands'] == 'all' and call['motion'] == {'mode': 'free', 'torso': 'keep'}
    assert plan(move, hand='holding', by=[0, 0, 0.1], hands='all', torso='keep',
                motion={'mode': 'free'}).calls[0] == call


def test_set_posture_rows(make):
    move = make()
    one = plan(move, elbow='out')                                              # set_posture {elbow: side}
    assert one.part == 'posture' and one.calls == [{'motion': {}, 'elbow': 'out'}]
    named = plan(move, elbow='down', hands=['left'], tolerance_deg=10, motion={'speed': 0.5})
    assert named.calls == [{'motion': {'speed': 0.5}, 'tolerance_deg': 10, 'elbow': 'down', 'hands': ['left']}]
    by_hand = plan(move, elbow={'left': 'out'})                                # {elbow: {left: out}}
    assert by_hand.calls == [{'motion': {}, 'elbow': 'out', 'hands': ['left']}]
    both = plan(move, elbow={'left': 'out', 'right': 'in'})
    assert [(c['elbow'], c['hands']) for c in both.calls] == [('out', ['left']), ('in', ['right'])]


def test_move_joints_rows(make):
    move = make()
    pose = plan(move, to='lift_up')                                             # move_joints {pose: N}
    [call] = pose.calls
    assert pose.part == 'joints' and call['pose'] == 'lift_up' and call[RESOLVED]['goal'] == {'lift': 0.1}
    only = plan(move, to='home', only=['left_arm'])                             # {pose: N, only: [...]}
    assert only.calls[0]['only'] == ['left_arm'] and only.calls[0][RESOLVED]['goal'] == {
        'left_joint1': 0.0, 'left_joint2': 0.0, 'right_joint1': 0.0, 'right_joint2': 0.0, 'lift': 0.0}
    values = plan(move, joints={'torso': 0.3, 'left_arm': [0.1, None]})         # {joints: {j: v}} -> groups
    assert values.calls[0]['joints'] == {'lift': 0.3, 'left_joint1': 0.1}
    via = plan(move, to='home', motion={'via': ['lift_up', 'head_up']})         # {motion: {via: [...]}}
    assert via.calls[0][RESOLVED]['via'] == [{'lift': 0.1}, {'left_joint1': 0.3}]


def test_motion_via_of_a_hand_move_takes_hand_and_joint_targets(make):
    move = make()
    call = plan(move, to='box_1/rim', motion={'via': ['present_left', 'lift_up', 'box_1/knob']}).calls[0]
    via = call[RESOLVED]['via']
    assert [item['reference'] for item in via] == ['present_left', 'lift_up', 'box_1/knob']
    assert [item['target'].kind for item in via] == ['hand', 'joints', 'hand'] and via[1]['joints'] == {'lift': 0.1}
    assert call['motion'] == {}                     # the entries are resolved: the part reads them from RESOLVED


# ---- targets that carry a joints part, a path, a reference -----------------------------------------------------

def test_a_hand_target_with_joints_passes_them_as_joint_values(make):
    move = make()
    both = plan(move, to='present_both').calls[0]
    assert both[RESOLVED]['joints'] == {'lift': 0.1}          # the target's torso: its joints part, the part's default
    assert 'torso' not in both['motion']
    step = plan(move, to='present_both', torso=0.2).calls[0]       # the step's value wins, and keeps it
    assert step[RESOLVED]['joints'] == {'lift': 0.2} and step['motion']['torso'] == 'keep'
    auto = plan(move, to='present_both', torso='auto').calls[0]
    assert auto[RESOLVED]['joints'] == {} and auto['motion']['torso'] == 'auto'
    keep = plan(move, to='present_both', torso='keep').calls[0]
    assert keep[RESOLVED]['joints'] == {} and keep['motion']['torso'] == 'keep'


def test_a_hand_target_with_a_path_keeps_its_waypoints(make):
    target = plan(make(), to='box_1/rim_path').calls[0][RESOLVED]['target']
    assert target.is_path and len(target.poses) == 2


def test_a_base_target_with_a_path_makes_successive_goals(make):
    move = make('lift_bot', ('navigate', 'straight', 'joints'), pose=Pose2D(0.0, 0.0, 0.0))
    result = plan(move, to='dock_path')
    assert [call['target'] for call in result.calls] == ['dock_path (1/2)', 'dock_path']
    assert [call[RESOLVED]['pose'].x for call in result.calls] == [0.5, 1.0]


def test_the_joints_part_of_a_base_target_moves_first_and_its_torso_replaces_the_drive_height(make):
    move = make('lift_bot', ('navigate', 'straight', 'joints'), pose=Pose2D(0.0, 0.0, 0.0))
    result = plan(move, to='dock_lifted')
    assert result.before['joints'] == {'head1': 0.2, 'lift': 0.4}
    assert result.calls[0][RESOLVED]['torso'] == 'keep'       # the joints leg set the height: drive_m does not apply
    head = plan(move, to='dock_head')
    assert head.before['joints'] == {'head1': 0.2} and head.calls[0][RESOLVED]['torso'] is None
    keep = plan(move, to='dock_keep')
    assert keep.before is None and keep.calls[0][RESOLVED]['torso'] == 'keep'
    step = plan(move, to='dock_lifted', torso=0.35)           # the step's value: the drive module goes there
    assert step.before['joints'] == {'head1': 0.2} and step.calls[0][RESOLVED]['torso'] == {'lift': 0.35}
    assert plan(move, to='dock', torso='keep').calls[0][RESOLVED]['torso'] == 'keep'
    assert plan(move, base={'by': [0.1, 0]}, torso=0.2).calls[0][RESOLVED]['torso'] == {'lift': 0.2}


def test_motion_speed_and_accel_reach_the_base_parts(make):
    move = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    call = plan(move, to='dock', motion={'speed': 0.5, 'accel': 0.25}).calls[0]
    assert call[RESOLVED]['speed'] == 0.5 and call[RESOLVED]['accel'] == 0.25
    assert plan(move, base={'by': [0.1, 0]}, motion={'speed': 0.5}).calls[0][RESOLVED]['speed'] == 0.5
    rejected(plan(move, to='dock', motion={'speed_abs': {'tcp_m_s': 0.1}}), 'speed_abs does not apply')


# ---- every REJECTED line --------------------------------------------------------------------------------------

def test_an_option_that_does_not_apply_to_the_kind_says_which_kind_and_which_option(make):
    move = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    assert rejected(plan(move, to='dock', hands='left')) == "move: 'dock' moves the base; option hands does not apply"
    rejected(plan(move, to='dock', offset_m=0.1), "'dock' moves the base", 'offset_m does not apply')
    rejected(plan(move, to='dock', only=['torso']), 'option only does not apply')
    rejected(plan(move, to='dock', tolerance_deg=3), 'option tolerance_deg does not apply')
    rejected(plan(move, to='dock', motion={'mode': 'linear'}), 'motion.mode', 'does not apply')
    rejected(plan(move, to='dock', motion={'planner': 'direct'}), 'motion.planner does not apply')
    rejected(plan(move, to='dock', by=[0, 0, 1]), "'dock' moves the base", 'option by does not apply')
    rejected(plan(move, to='dock', torso='auto'), 'torso auto needs a hand or object target')
    rejected(plan(move, to='dock', motion={'mode': 'straight'}, reference='x'), 'option reference does not apply')


def test_the_options_of_the_other_kinds_are_checked_too(make):
    move = make(held=[{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}])
    rejected(plan(move, to='home', hands='left'), "'home' moves joints", 'option hands does not apply')
    rejected(plan(move, to='home', torso='keep'), 'torso')
    rejected(plan(move, to='home', offset_m=0.1), 'offset_m')
    rejected(plan(move, to='box_1/rim', only=['torso']), 'option only does not apply')
    rejected(plan(move, to='box_1/rim', reference='x'), 'option reference does not apply')
    rejected(plan(move, to='box_1/rim', motion={'mode': 'nav'}), 'motion.mode', 'does not apply')
    rejected(plan(move, to='carry', motion={'via': ['home']}), 'motion.via does not apply')
    rejected(plan(move, to='carry', hands='any_one'), 'hands any_one does not apply')
    rejected(plan(move, elbow='out', offset_m=0.1), 'change posture: option offset_m does not apply')
    rejected(plan(move, elbow='out', torso='auto'), 'change posture: torso')
    rejected(plan(move, elbow={'left': 'out'}, hands='left'), 'elbow names its hands')
    rejected(plan(move, joints={'torso': 0.1}, only=['torso']), 'option only applies to a joint target')
    rejected(plan(move, joints={'torso': 0.1}, hands='left'), 'move to joint positions: option hands does not apply')


def test_cell_frame_hand_and_object_targets_are_rejected(make):
    move = make()
    assert rejected(plan(move, to='dock_tool'), 'cell-frame hand targets are not supported yet',
                    "target 'dock_tool'").startswith('move to target: ')
    assert rejected(plan(move, to='dock_object')).startswith('move to target: cell-frame object targets')


def test_a_base_frame_base_target_is_rejected_until_it_is_built(make, tmp_path):
    (tmp_path / 'config' / 'robots' / 'diff_bot' / 'targets.yaml').write_text(
        'back_off: {moves: base, x: -0.3, y: 0.0, yaw: 0.0}\n')
    move = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    rejected(plan(move, to='back_off'), 'base-frame base targets are not supported yet', 'base: {by: [...]}')


def test_a_held_hand_or_nothing_held_is_a_rejection_naming_the_kind(make):
    nothing = make()
    assert rejected(plan(nothing, object='held', by=[0, 0, 0.1])) == (
        'move to target: nothing is held: move to a hand target, then grip close')
    rejected(plan(nothing, hand='holding', by=[0, 0, 0.1]), 'move to target: nothing is held')
    rejected(plan(nothing, to='carry'), 'move to target: nothing is held')
    # a base-frame target of another type than the held one
    other = make(held=[{'object': 'crate_1', 'hands': ['left', 'right'], 'type': 'crate'}])
    rejected(plan(other, to='carry'), "'carry' is a base-frame object target for box", 'a crate is held')
    nothing_typed = make(held=[{'object': 'x', 'hands': ['left'], 'type': None}])
    rejected(plan(nothing_typed, to='carry'), 'nothing is held')


def test_a_hand_move_of_a_holding_and_a_free_hand_is_refused(make):
    move = make(held=[{'object': 'box_1', 'hands': ['left'], 'type': 'box'}])
    rejected(plan(move, to='box_1/rim'), 'move to target: left holds box_1, right holds nothing', 'two steps')
    rejected(plan(move, hand='all', by=[0, 0, 0.1]), 'left holds box_1, right holds nothing')
    assert plan(move, to='box_1/rim', hands='right').part == 'hand'      # the free hand moves alone, as before
    assert plan(move, to='box_1/knob').part == 'hand'                    # a target of the free hand only
    assert plan(move, hand='right', by=[0, 0, 0.1]).part == 'hand'


def test_unknown_targets_and_wrong_kinds_say_so(make):
    move = make()
    rejected(plan(move, to='mars'), 'move: unknown target', "'mars'")
    rejected(plan(move, to='box_1/rim', motion={'via': ['dock']}), "'dock' moves the base",
             'via takes hand or joint targets')
    rejected(plan(move, to='home', motion={'via': ['present_left']}), "'present_left' moves a hand",
             'via of a joint move takes joint targets')
    rejected(plan(move, to='box_1/rim', motion={'via': ['dock_tool']}), 'cell-frame hand targets')
    rejected(plan(move, to='stance:box_1/pick'), 'stance: prefix was removed')


def test_look_at_and_the_two_hand_spellings_are_rejected(make):
    move = make()
    assert rejected(plan(move, to='dock', look_at='box_1')) == (
        'move: look_at is not implemented yet: set head through joints')
    rejected(plan(move, hand='left', hands='left', by=[0, 0, 0]), 'hand and hands are two spellings')


def test_relative_forms_check_their_keys(make):
    move = make(held=[{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}])
    rejected(plan(move, hand='left', by=[0, 0, 1], at=[0, 0, 1]), 'by or at, not both')
    rejected(plan(move, hand='left', at=[0, 0, 1], turn_deg=3), 'option turn_deg does not apply to at')
    rejected(plan(move, hand='left', by=[0, 0, 1], rpy_deg=[0, 0, 0]), 'option rpy_deg does not apply to by')
    rejected(plan(move, object='held', by=[0, 0, 1], frame='tool'), 'option frame does not apply to by')
    rejected(plan(move, hand='all', at=[0, 0, 1]), 'at needs exactly one hand')
    rejected(plan(move, hand='auto', by=[0, 0, 1]), 'name the hands')
    rejected(plan(move, hand='left', by=[0, 0, 1], offset_m=0.1), 'offset_m does not apply to a relative move')
    rejected(plan(move, hand='left', by=[0, 0, 1], torso=0.3), 'a relative move takes torso auto or keep')
    rejected(plan(move, base={'by': [1, 0]}, rpy_deg=[0, 0, 0]), 'move straight: option rpy_deg does not apply')


def test_a_robot_without_the_part_says_it_cannot_do_that_kind(make):
    move = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    rejected(plan(move, to='box_1/rim'), 'this robot cannot do hand moves', 'needs arms.bimanual')
    rejected(plan(move, elbow='out'), 'this robot cannot do posture moves')


def test_a_group_value_that_does_not_fit_the_robot_is_rejected_with_the_group(make):
    move = make()
    rejected(plan(move, joints={'wings': 0.1}), 'move to joint positions:', "joint group 'wings'")
    rejected(plan(move, joints={'left_arm': 0.1}), "joint group 'left_arm' has 2 joints")
    rejected(plan(move, joints={'torso': 'auto'}), 'torso auto needs a hand or object target')
    rejected(plan(move, joints={'torso': 'keep'}), 'names no joint to move')


def test_the_reference_of_an_object_target_comes_from_the_target_or_the_step(make, tmp_path):
    root = tmp_path / 'config'
    (root / 'objects' / 'crate.yaml').write_text(
        'name: crate\ntargets:\n  hook:\n    moves: object\n    xyz: [0.0, 0.0, 0.1]\n    rpy: [0.0, 0.0, 0.0]\n'
        '    reference: base\n  slot: {moves: object, xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}\n')
    (root / 'cell' / 'objects.yaml').write_text(
        (FIXTURE / 'cell' / 'objects.yaml').read_text()
        + 'crate_1: {object: crate, x: 2.0, y: 0.0, z: 0.0, yaw: 0.0}\n')
    held = [{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}]
    move = make(held=held)
    own = plan(move, to='crate_1/hook').calls[0]
    assert own['to']['contact'] == 'base'                       # the target's own reference
    step = plan(move, to='crate_1/hook', reference='top').calls[0]
    assert step['to']['contact'] == 'top' and step[RESOLVED]['onto']['mine'].xyz == (0.0, 0.0, 0.1)
    origin = plan(move, to='crate_1/slot').calls[0]            # no reference at all: the object's origin
    assert origin['to']['contact'] == 'origin' and origin[RESOLVED]['onto']['mine'].xyz == (0.0, 0.0, 0.0)
    rejected(plan(move, to='crate_1/hook', reference='nope'), "reference 'nope'", 'no target')
    rejected(plan(move, to='crate_1/hook', reference='rim'), "reference 'rim'", 'must be an object target')


def test_offset_m_does_not_apply_to_a_carry_pose_and_a_joints_part_is_refused_on_a_contact(make):
    move = make(held=[{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}])
    rejected(plan(move, to='carry', offset_m=0.1), 'offset_m does not apply', "'carry'")


# ---- timeouts --------------------------------------------------------------------------------------------------

def test_the_default_timeout_comes_from_the_kind_of_motion(make):
    move = make('lift_bot', ('navigate', 'straight', 'joints'), pose=Pose2D(0.0, 0.0, 0.0))
    assert move.default_timeout({'to': 'dock'}) == 120.0                    # nav
    assert move.default_timeout({'to': 'dock', 'motion': {'mode': 'straight'}}) == 60.0
    assert move.default_timeout({'base': {'by': [0.1, 0]}}) == 60.0
    assert move.default_timeout({'to': 'look_up'}) == 60.0
    assert move.default_timeout({'joints': {'torso': 0.1}}) == 60.0
    assert move.default_timeout({'to': 'mars'}) == Move.default_timeout_s   # unresolvable: the skill's own
    assert move.default_timeout({'hand': 'left', 'by': [0, 0, 1]}) == Move.default_timeout_s   # no such part here
    assert Move.default_timeout_s == 120.0


# ---- the result -------------------------------------------------------------------------------------------------

def test_the_result_names_the_kind_the_module_and_the_target(make):
    move = make()
    found = plan(move, to='lift_up')
    result = move._finish(SkillResult.failed('no progress', max_error=1), found, move.modules['joints'])
    assert result.message == 'move to joint positions · lift_up: no progress'
    assert result.data == {'max_error': 1, 'kind': 'joints', 'module': 'move.joints', 'target': 'lift_up'}
    hand = plan(move, to='box_1/rim')
    done = move._finish(SkillResult.succeeded('at box_1/rim (max error 0.004)', hands=['left', 'right']), hand,
                        move.modules['hand'])
    assert done.message == 'move to target · box_1/rim (left, right): at box_1/rim (max error 0.004)'
    assert done.data['kind'] == 'hand' and done.data['target'] == 'box_1/rim' and 'carrying' not in done.data
    refused = move._finish(SkillResult.rejected('no way'), hand, move.modules['hand'])
    assert refused.message == 'move to target: no way'
    held = make(held=[{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}])
    carry = plan(held, to='carry')
    moved = held._finish(SkillResult.succeeded('moved box_1 to carry (max error 0.003)', hands=['left', 'right']),
                         carry, held.modules['carry'])
    assert moved.message == ('move to target · carrying box_1 · carry (left, right): moved box_1 to carry '
                             '(max error 0.003)')
    assert moved.data == {'hands': ['left', 'right'], 'kind': 'hand', 'module': 'move.carry', 'target': 'carry',
                          'carrying': 'box_1'}
    drive = make('diff_bot', ('navigate', 'straight'), pose=Pose2D(0.0, 0.0, 0.0))
    nav = drive._finish(SkillResult.succeeded('at dock'), plan(drive, to='dock'), drive.modules['navigate'])
    assert nav.message == 'navigate to · dock (direct): at dock' and nav.data['module'] == 'move.navigate'
    straight = plan(drive, base={'by': [0.4, 0]})
    assert drive._finish(SkillResult.succeeded('done'), straight, drive.modules['straight']).message == (
        'move straight · by +0.4 m: done')
    posture = plan(make(), elbow='out')
    assert move._finish(SkillResult.succeeded('swung'), posture, move.modules['posture']).message == (
        'change posture · elbow out: swung')


def test_the_editor_shapes_of_hand_torso_and_the_by_at_choice(make):
    props = Move.args_schema['properties']
    assert props['hand']['x-taskweave-widget'] == 'hands'
    assert [list(option) for option in props['hand']['anyOf']] == [['type'], ['type', 'items', 'minItems',
                                                                            'uniqueItems']]
    assert props['torso']['anyOf'][0] == {'enum': ['auto', 'keep']} and props['torso']['anyOf'][1] == {'type': 'number'}
    move = make()
    narrowed = Move.schema_for(move.hands)['properties']['hand']
    assert narrowed['anyOf'][0]['enum'] == ['all', 'holding', 'left', 'right']
    assert narrowed['x-taskweave-widget'] == 'hands'
    forms = {variant['required'][0]: variant for variant in Move.args_schema['oneOf']}
    assert forms['hand']['x-taskweave-exclusive'] == forms['object']['x-taskweave-exclusive'] == ['by', 'at']
    assert Move.args_schema['not']['required'] == ['by', 'at']
    check = lambda step: validate_args(step, Move.schema_for(move.hands), move.config, 'duo_bot')   # noqa: E731
    for torso in ('auto', 'keep', 0.2, [0.1, None]):
        assert check({'hand': 'left', 'by': [0, 0, 0.1], 'torso': torso}) == [], torso
    assert check({'hand': 'left', 'by': [0, 0, 0.1], 'torso': 'up'})
    assert check({'hand': 'left', 'at': [0.3, 0.1, 1.0], 'rpy_deg': [0, 0, 0]}) == []
    both = check({'hand': 'left', 'by': [0, 0, 0.1], 'at': [0.3, 0.1, 1.0]})
    assert len(both) == 1 and 'give by or at, not both' in both[0], both
    assert check({'object': 'held', 'by': [0, 0, 0.1], 'at': [0, 0, 0]})


# ---- carrying: a hand move of a holding hand ---------------------------------------------------------------------

def with_shape(tmp_path):
    """The box type of the test cell with a collision shape (objects/box.yaml ``collision:``)."""
    path = tmp_path / 'config' / 'objects' / 'box.yaml'
    path.write_text(path.read_text() + 'collision: {box: [0.4, 0.3, 0.2]}\n')


TWO = [{'object': 'box_1', 'hands': ['left', 'right'], 'type': 'box'}]
ONE = [{'object': 'box_1', 'hands': ['left'], 'type': 'box'}]


def test_a_relative_move_of_one_hand_of_a_two_hand_hold_carries_the_whole_hold(make, tmp_path):
    with_shape(tmp_path)
    move = make(held=TWO)
    result = plan(move, hand='left', by=[0, 0, 0.1], frame='tool', turn_deg=5)
    assert result.part == 'carry' and result.kind == 'hand' and result.carrying == 'box_1'
    [call] = result.calls
    assert call['object'] == 'box_1' and call[RESOLVED] == {'hand': 'left'} and 'hands' not in call
    assert call['to'] == {'by': [0, 0, 0.1], 'frame': 'tool', 'turn_deg': 5}
    assert result.desc == 'carrying box_1 · hand left by'
    both = plan(move, hand='all', by=[0, 0, 0.1])                       # both hands of the hold: the same carry
    assert both.part == 'carry' and both.calls[0][RESOLVED] == {'hand': 'left'}
    at = plan(move, hand='right', at=[0.5, 0, 1.0], rpy_deg=[0, 90, 0])
    assert at.calls[0]['to'] == {'at': [0.5, 0, 1.0], 'rpy_deg': [0, 90, 0]} and at.calls[0][RESOLVED] == {
        'hand': 'right'}


def test_a_one_hand_hold_is_carried_by_its_hand(make, tmp_path):
    with_shape(tmp_path)
    move = make(held=ONE)
    result = plan(move, hand='left', by=[0, 0, 0.1])
    assert result.part == 'carry' and result.calls[0][RESOLVED] == {'hand': 'left'}
    target = plan(move, to='pile_1/rim', hands='left')
    assert target.part == 'carry' and target.carrying == 'box_1' and target.kind == 'hand'
    [call] = target.calls
    assert call['to'] == {'hand_target': 'pile_1/rim'} and call['object'] == 'box_1'
    assert call[RESOLVED]['hands'] == ['left'] and call[RESOLVED]['hand_target']['target'].reference == 'pile_1/rim'
    assert plan(move, hand='right', by=[0, 0, 0.1]).part == 'hand'      # the free hand: a hand move


def test_a_hand_target_of_a_two_hand_hold_carries_the_object_where_the_hands_go(make, tmp_path):
    with_shape(tmp_path)
    move = make(held=TWO)
    result = plan(move, to='pile_1/rim', offset_m=0.05, torso='keep')
    assert result.part == 'carry' and result.desc == 'carrying box_1 · pile_1/rim'
    [call] = result.calls
    assert call[RESOLVED]['hands'] == ['left', 'right'] and call[RESOLVED]['hand_target']['offset_m'] == 0.05
    assert call['motion'] == {'torso': 'keep'} and call[RESOLVED]['hand_target']['data'] == {'approach_m': 0.1}
    rejected(plan(move, to='pile_1/rim', motion={'via': ['present_left']}), 'motion.via does not apply while')
    rejected(plan(move, to='present_both'), 'joint values', 'do not apply while the hands hold something')
    rejected(plan(move, hand='left', by=[0, 0, 0.1], motion={'approach_m': 0.1}), 'motion.approach_m does not apply')


def test_the_new_carry_path_needs_the_collision_shape_of_the_held_type(make):
    move = make(held=TWO)                          # objects/box.yaml has no collision shape
    text = rejected(plan(move, hand='left', by=[0, 0, 0.1]), 'move to target: left holds box_1',
                    'objects/box.yaml has no collision:')
    assert 'hand: holding' in text
    rejected(plan(move, to='pile_1/rim'), 'objects/box.yaml has no collision:')
    untyped = make(held=[{'object': 'thing', 'hands': ['left'], 'type': None}])
    rejected(plan(untyped, hand='left', by=[0, 0, 0.1]), 'its type is unknown')
    # the moves of a held object as before: no shape needed
    assert plan(move, hand='holding', by=[0, 0, 0.1]).part == 'carry'
    assert plan(move, object='held', by=[0, 0, 0.1]).part == 'carry'
    assert plan(move, to='carry').part == 'carry'


def test_two_holds_or_a_free_hand_with_a_holding_one_are_refused(make, tmp_path):
    with_shape(tmp_path)
    two = make(held=[{'object': 'box_1', 'hands': ['left'], 'type': 'box'},
                     {'object': 'box_2', 'hands': ['right'], 'type': 'box'}])
    rejected(plan(two, hand='all', by=[0, 0, 0.1]), 'left holds box_1, right holds box_2',
             'name the hands of one held object')
    assert plan(two, hand='right', by=[0, 0, 0.1]).calls[0]['object'] == 'box_2'
    rejected(plan(make(), hand='left', hands='left', by=[0, 0, 0.1]), 'two spellings')


def test_a_hand_target_on_the_held_object_itself_is_refused(make, tmp_path):
    with_shape(tmp_path)
    for held, hands in ((TWO, None), (ONE, 'left')):
        move = make(held=held)
        args = {'to': 'box_1/rim', 'offset_m': 0.1, **({'hands': hands} if hands else {})}
        text = rejected(plan(move, **args), 'the hands hold box_1: a hand target on it cannot be reached while '
                                            'holding it; open the gripper first')
        assert text.startswith('move to target: ')
        rejected(plan(move, to='box_1/knob', hands=hands or 'all'), 'the hands hold box_1')
    assert plan(make(held=TWO), to='pile_1/rim').part == 'carry'          # another object: carried there


def test_a_shape_that_is_not_in_the_planning_scene_refuses_the_new_carry_path(make, tmp_path):
    with_shape(tmp_path)
    move = make(held=TWO)
    move._scene_problem = lambda hold: 'the planning scene has lost it (move_group restarted?)'
    text = rejected(plan(move, hand='left', by=[0, 0, 0.1]), "the held object's shape is not in the planning scene",
                    'move_group restarted')
    assert 'hand: holding' in text
    rejected(plan(move, to='pile_1/rim'), 'not in the planning scene')
    # the moves of a held object as before: no scene check
    assert plan(move, hand='holding', by=[0, 0, 0.1]).part == 'carry'
    assert plan(move, object='held', by=[0, 0, 0.1]).part == 'carry'
    assert plan(move, to='carry').part == 'carry'


def test_a_robot_without_the_carry_part_says_it_cannot_carry(make, tmp_path):
    with_shape(tmp_path)
    move = make(parts=('hand', 'joints', 'posture'), held=TWO)
    rejected(plan(move, hand='holding', by=[0, 0, 0.1]), 'this robot cannot carry held objects')
