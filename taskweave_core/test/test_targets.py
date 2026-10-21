"""Targets (cell schema 4): the examples of the design load and resolve to the numbers the old store gives for the same
entries (the old twin in fixtures/targets_old), the merge of an object type's layers, stacks, symmetric objects, every
error of the references, joint groups, writing targets and keyed overrides of a robot pack's targets.yaml."""

import math
import shutil
from pathlib import Path

import pytest
import yaml

from taskweave_core.config import ConfigError, ConfigStore, Pose2D
from taskweave_core.config import targets as model
from taskweave_core.config.limits import group_values, target_joint_problems
from taskweave_core.config.targets import GroupError, TargetError, TargetWriteError

FIXTURES = Path(__file__).parent / 'fixtures'
ROBOT = 'bot_two_arm'
NEAR = [None, Pose2D(7.0, 1.0, 0.0), Pose2D(9.0, 1.0, 3.1416), Pose2D(5.0, 3.0, 0.0)]
RENAMED = {('karakuri_hanger', 'knob'): 'knob_stance', ('frame_type2', 'set_down'): 'set_down_held'}


@pytest.fixture(autouse=True)
def state_dir(tmp_path, monkeypatch):
    monkeypatch.setenv('TASKWEAVE_STATE_DIR', str(tmp_path / 'state'))


def copy_cell(tmp_path, name='targets_new') -> Path:
    target = tmp_path / name
    shutil.copytree(FIXTURES / name, target)
    return target


@pytest.fixture
def new():
    return ConfigStore(FIXTURES / 'targets_new')


@pytest.fixture
def old():
    return ConfigStore(FIXTURES / 'targets_old')


@pytest.fixture
def cell(tmp_path) -> Path:
    return copy_cell(tmp_path)


def edit(path: Path, text: str) -> None:
    path.write_text(text, encoding='utf-8')


def append(path: Path, text: str) -> None:
    edit(path, path.read_text(encoding='utf-8') + text)


# ---- every example of the design loads ------------------------------------------------------------
def test_the_new_cell_is_in_the_targets_layout_and_the_old_twin_is_not(new, old):
    assert new.targets_format() and not old.targets_format()


def test_cell_targets_are_base_targets_in_the_cell_frame(new):
    waiting = new.target('waiting', ROBOT)
    assert (waiting.kind, waiting.frame, waiting.name, waiting.reference) == ('base', 'cell', 'waiting', 'waiting')
    assert waiting.pose == Pose2D(2.0, 0.0, 0.0, 'cell') and waiting.files == ('cell/targets.yaml',)
    assert new.target('dropoff', ROBOT).pose.yaw == 3.1416
    assert set(new.target_names(ROBOT)['base targets']) >= {
        'waiting',
        'dropoff',
        'point_a',
        'set_down',
        'spawn',
        'taught_dock',
    }


def test_a_stance_is_composed_with_the_object_pose(new):
    pick = new.target('bay_1_stack_1_front/pick', ROBOT)
    assert (pick.kind, pick.frame, pick.type) == ('base', 'object', 'frame_type2')
    assert pick.pose == Pose2D(5.0, 1.0, 0.0, 'cell')  # object at (6, 1), offset x -1
    assert pick.files == ('robots/bot_two_arm/objects/frame_type2.yaml',)
    assert new.target('karakuri_hanger/staging', ROBOT).pose.yaw == pytest.approx(-math.pi / 2, abs=1e-3)


def test_back_row_instances_override_the_whole_pose(new):
    assert new.target('bay_1_stack_1_back/pick', ROBOT).pose == Pose2D(6.0 - 1.528, 2.5 + 1.0, 0.0, 'cell')
    assert new.target('bay_2_stack_2_back/pick', ROBOT).pose == Pose2D(8.0 - 1.528, 2.5, 0.0, 'cell')
    assert new.target('bay_1_stack_1_back/pick', ROBOT).files == ('robots/bot_two_arm/objects/frame_type2.yaml',)


def test_a_hand_target_keeps_its_poses_and_settings(new):
    rails = new.target('bay_1_stack_1_front/rails_top', ROBOT)
    assert (rails.kind, rails.frame, rails.type) == ('hand', 'object', 'frame_type2')
    assert rails.tools['left_tool'].xyz == (-0.0563, 0.2513, 0.0714)
    assert rails.tools['right_tool'].frame == 'frame_type2' and rails.tool_poses.keys() == {'left', 'right'}
    assert rails.get('approach_m') == 0.1 and rails.get('elbow') == 'up'  # the robot's setting, merged in
    assert rails.files == ('objects/frame_type2.yaml', 'robots/bot_two_arm/objects/frame_type2.yaml')
    other = new.target('bay_1_stack_1_front/rails_top', 'bot_plain')  # another robot: no elbow of ours
    assert other.get('elbow') is None


def test_the_robots_setting_on_a_hand_target_and_one_hand_poses(new):
    top = new.target('part_box_1/top_grasp', ROBOT)
    assert top.get('grip_m') == {'left': 0.06, 'right': 0.06} and top.get('tool_type') == 'parallel_gripper'
    assert top.alone['left_tool'].xyz == (0.0, 0.0, 0.07) and top.tools == {}
    assert new.target('karakuri_hanger/knob', ROBOT).get('elbow') == 'out'


def test_a_base_frame_hand_target_has_a_joints_part(new):
    grasp = new.target('frame_grasp_approach', ROBOT)
    assert (grasp.kind, grasp.frame) == ('hand', 'base') and grasp.tools['right_tool'].frame == 'base'
    assert grasp.joints == {'torso': 0.5, 'head': [-0.3491, 0.0]} and grasp.get('elbow') == 'up'
    assert group_values(new, ROBOT, grasp.joints) == {'body_joint': 0.5, 'head_joint1': -0.3491, 'head_joint2': 0.0}


def test_object_targets_in_the_base_frame_are_held_paths(new):
    carry = new.target('carry', ROBOT)
    assert (carry.kind, carry.frame, carry.held_type) == ('object', 'base', 'frame_type2') and carry.is_path
    assert carry.pose.xyz == (1.0, 0.0, 0.4) and carry.pose.frame == 'base'
    stand = new.target('stand_up', ROBOT)
    assert [p.xyz for p in stand.poses] == [(1.3, 0.0, 0.5), (1.1, 0.0, 0.7)] and stand.pose == stand.poses[-1]
    assert stand.joints == {'torso': 0.55}


def test_a_contact_is_two_object_targets_joined_by_a_reference(new):
    hook = new.target('karakuri_hanger/hook', ROBOT)
    plate = new.target('bay_2_stack_2_front/hook_plate', ROBOT)
    assert hook.kind == plate.kind == 'object' and hook.frame == 'object' and hook.reference_target == 'hook_plate'
    assert plate.pose.xyz == (0.20, 0.0, 0.04) and plate.pose.frame == 'frame_type2'
    assert plate.reference_target is None


def test_joint_targets_have_no_frame(new):
    home = new.target('home', ROBOT)
    assert (home.kind, home.frame, home.pose, home.poses) == ('joints', 'none', None, ())
    assert sorted(group_values(new, ROBOT, home.joints)) == sorted(
        ['head_joint1', 'head_joint2'] + [f'{s}_arm_joint{i}' for s in ('left', 'right') for i in range(1, 8)]
    )
    assert group_values(new, ROBOT, new.target('lift_high', ROBOT).joints) == {'body_joint': 0.6}


def test_the_names_a_plan_can_use_are_listed_by_kind(new):
    names = new.target_names(ROBOT)
    assert set(names) == {'base targets', 'hand targets', 'object targets', 'joint targets', 'placed objects'}
    assert 'bay_2_stack_2_front/pick' in names['base targets'] and 'carry' in names['object targets']
    assert 'karakuri_hanger/knob' in names['hand targets'] and 'karakuri_hanger/knob_stance' in names['base targets']
    assert names['joint targets'] == ['home', 'lift_carry', 'lift_high', 'lift_low']
    assert 'part_box_1' in names['placed objects']
    assert new.object_target_names(ROBOT)['frame_type2'] == ['hook_plate', 'pick', 'rails_top', 'rails_top_steep']
    assert new.object_target_names()['frame_type2'] == ['hook_plate', 'rails_top', 'rails_top_steep']


def test_every_target_resolves(new):
    found = new.targets(ROBOT)
    assert {'waiting', 'home', 'carry', 'bay_1_stack_1_front/rails_top'} <= set(found)
    assert all(t.reference == ref for ref, t in found.items())
    assert len(found) == sum(len(v) for k, v in new.target_names(ROBOT).items() if k != 'placed objects')


# ---- the numbers of the old store -----------------------------------------------------------------
@pytest.mark.parametrize('near', NEAR)
def test_stances_resolve_to_the_numbers_of_the_old_store(old, new, near):
    count = 0
    for instance in old.object_instances().values():
        for variant in old.stance_variants(ROBOT, instance):
            name = RENAMED.get((instance.object, variant), variant)
            assert new.target(f'{instance.name}/{name}', ROBOT, near).pose == old.stance(
                f'{instance.name}/{variant}', ROBOT, near
            ), (instance.name, variant)
            count += 1
    assert count == 8  # 4 frames x pick, hanger x3, part box


@pytest.mark.parametrize('near', NEAR)
def test_points_resolve_to_the_numbers_of_the_old_store(old, new, near):
    count = 0
    for instance in old.object_instances().values():
        for point in old.object(instance.object).get('points', {}):
            ref = f'{instance.name}/{point}'
            expected_instance, expected = old.instance_point(ref, ROBOT, near)
            target = new.target(ref, ROBOT, near)
            assert target.instance == expected_instance and target.to_point() == expected, ref
            count += 1
    assert count == 10  # 4 frames x 2 points, knob, top_grasp


def test_stack_points_refer_to_the_top_item(old, new):
    target = new.target('bay_1_stack_1_front/rails_top', ROBOT)
    assert target.instance.stack_count == 3 and target.instance.top_z == pytest.approx(0.4)
    assert target.instance == old.instance_point('bay_1_stack_1_front/rails_top', ROBOT)[0]


def test_contacts_resolve_to_the_numbers_of_the_old_store(old, new):
    instance, pose = old.instance_contact('karakuri_hanger/hook')
    target = new.target('karakuri_hanger/hook', ROBOT)
    assert target.instance == instance and target.pose == pose
    assert new.target('bay_1_stack_1_front/hook_plate', ROBOT).pose == old.object_contact('frame_type2', 'hook_plate')


def test_held_paths_resolve_to_the_numbers_of_the_old_store(old, new):
    for old_name, name in (
        ('carry', 'carry'),
        ('stand_up', 'stand_up'),
        ('set_down', 'set_down_held'),
        ('hang-approach', 'hang-approach'),
    ):
        expected = old.held_pose(ROBOT, 'frame_type2', old_name)
        target = new.target(name, ROBOT)
        assert list(target.poses) == expected['path'], name
        assert group_values(new, ROBOT, target.joints) == expected['joints'], name


def test_locations_resolve_to_the_numbers_of_the_old_store(old, new):
    assert {n: new.target(n, ROBOT).pose for n in old.locations()} == old.locations()


def test_poses_resolve_to_the_numbers_of_the_old_store(old, new):
    for name in ('home', 'lift_high', 'lift_low', 'lift_carry'):
        assert group_values(new, ROBOT, new.target(name, ROBOT).joints) == old.named_pose(ROBOT, name)['joints'], name
    expected = old.named_pose(ROBOT, 'frame_grasp_approach')
    grasp = new.target('frame_grasp_approach', ROBOT)
    assert group_values(new, ROBOT, grasp.joints) == expected['joints'] and grasp.get('elbow') == expected['elbow']
    for tool, axes in expected['tools'].items():  # tcp, approach and closing, from xyz and rpy
        pose = grasp.tool_poses[tool]
        approach, closing = model.axes_from_rpy(pose.rpy)
        assert list(pose.xyz) == axes['tcp']
        assert approach == pytest.approx(axes['approach'], abs=1e-4)
        assert closing == pytest.approx(axes['closing'], abs=1e-4)


def test_partial_group_values_leave_joints_alone(old):
    new = ConfigStore(FIXTURES / 'targets_new')
    assert group_values(new, 'bot_lift', new.target('lift_high', 'bot_lift').joints) == {'lift_joint': 0.4}
    assert (
        group_values(new, 'bot_lift', new.target('lift_high', 'bot_lift').joints)
        == old.named_pose('bot_lift', 'lift_high')['joints']
    )
    assert (
        group_values(new, 'bot_lift', new.target('ready', 'bot_lift').joints)
        == old.named_pose('bot_lift', 'ready')['joints']
    )


# ---- the deprecated readers over the targets ------------------------------------------------------
def test_the_old_readers_give_the_same_numbers_on_the_new_layout(old, new):
    assert new.locations() == old.locations()
    for instance in old.object_instances().values():
        variants = {RENAMED.get((instance.object, v), v): p for v, p in old.stance_variants(ROBOT, instance).items()}
        assert new.stance_variants(ROBOT, instance) == variants
    for near in NEAR:
        assert new.stance('bay_1_stack_1_back', ROBOT, near) == old.stance('bay_1_stack_1_back', ROBOT, near)
        assert new.base_target('stance:bay_2_stack_2_front/pick', ROBOT, near) == old.base_target(
            'stance:bay_2_stack_2_front/pick', ROBOT, near
        )
        assert new.instance_point('bay_2_stack_2_front/rails_top', ROBOT, near) == old.instance_point(
            'bay_2_stack_2_front/rails_top', ROBOT, near
        )
    assert new.base_target('waiting', ROBOT) == old.base_target('waiting', ROBOT)
    assert new.robot_object_points(ROBOT) == old.robot_object_points(ROBOT)
    assert new.object_point('part_box', 'top_grasp', ROBOT) == old.object_point('part_box', 'top_grasp', ROBOT)
    assert new.object_point('frame_type2', 'rails_top') == old.object_point('frame_type2', 'rails_top')
    assert new.object_contact('karakuri_hanger', 'hook') == old.object_contact('karakuri_hanger', 'hook')
    assert new.instance_contact('karakuri_hanger/hook') == old.instance_contact('karakuri_hanger/hook')
    assert (
        new.held_pose(ROBOT, 'frame_type2', 'stand_up')['path']
        == old.held_pose(ROBOT, 'frame_type2', 'stand_up')['path']
    )
    assert new.named_pose(ROBOT, 'lift_high') == old.named_pose(ROBOT, 'lift_high')
    assert new.named_pose(ROBOT, 'home')['joints'] == old.named_pose(ROBOT, 'home')['joints']
    assert new.tool_pose_names(ROBOT) == old.tool_pose_names(ROBOT) == ['frame_grasp_approach']
    assert new.joint_pose(ROBOT, 'lift_low') == old.joint_pose(ROBOT, 'lift_low')
    with pytest.raises(KeyError, match='has no point'):
        new.object_point('frame_type2', 'hook_plate')  # an object target is not a point
    with pytest.raises(KeyError, match='no contact'):
        new.object_contact('frame_type2', 'rails_top')


def test_the_old_readers_list_the_same_names(old, new):
    names, expected = new.point_names(ROBOT), old.point_names(ROBOT)
    for key in (
        'locations',
        'tool poses',
        'joint-only poses',
        'object points',
        'placed object points',
        'placed objects',
    ):
        assert names[key] == expected[key], key
    assert names['joint poses'] == sorted(expected['joint poses'])
    assert sorted(names['held poses']) == ['carry', 'hang-approach', 'set_down_held', 'stand_up']
    assert 'karakuri_hanger/hook' in names['placed object contacts']
    assert set(names['stances']) == {n.replace('/knob', '/knob_stance') for n in expected['stances']}


def test_the_old_writers_refuse_on_the_new_layout(new):
    with pytest.raises(ValueError, match='save_target'):
        new.save_base_pose('somewhere', Pose2D(1.0, 2.0, 0.0))
    with pytest.raises(ValueError, match='save_target'):
        new.save_held_pose(ROBOT, 'frame_type2', 'x', {}, [])
    with pytest.raises(ValueError, match='save_target'):
        new.set_named_pose(ROBOT, 'home', {'body_joint': 0.1})
    with pytest.raises(ValueError, match='save_target'):
        new.save_object_point_settings('part_box', 'top_grasp', {'elbow': 'up'}, robot=ROBOT)


def test_the_old_layout_is_untouched(old):
    assert old.stance('bay_1_stack_1_back/pick', ROBOT) == Pose2D(6.0 - 1.528, 3.5, 0.0, 'cell')
    assert old.locations()['waiting'] == Pose2D(2.0, 0.0, 0.0, 'cell')
    assert not (FIXTURES / 'targets_old' / 'cell' / 'targets.yaml').exists()
    assert old.check_pack() == []
    with pytest.raises(TargetError, match='unknown target'):
        old.target('waiting', ROBOT)


# ---- symmetric objects and the handled_from key ---------------------------------------------------
def test_a_turned_object_is_handled_from_the_side_nearest_the_robot(old, new):
    placed = new.object_instances()['bay_2_stack_2_front']
    assert placed.pose.yaw == 3.1416
    near_turn = new.target('bay_2_stack_2_front/pick', ROBOT, Pose2D(7.0, 1.0, 0.0))
    assert near_turn.instance.pose.yaw == pytest.approx(0.0, abs=1e-3) and near_turn.pose.x == pytest.approx(
        7.0, abs=1e-3
    )
    as_placed = new.target('bay_2_stack_2_front/pick', ROBOT, Pose2D(9.0, 1.0, 0.0))
    assert abs(as_placed.instance.pose.yaw) == pytest.approx(math.pi, abs=1e-3)
    assert as_placed.pose.x == pytest.approx(9.0, abs=1e-3)
    assert new.target('bay_2_stack_2_front/pick', ROBOT).instance == placed  # no robot pose: as placed
    assert old.stance('bay_2_stack_2_front/pick', ROBOT, Pose2D(7.0, 1.0, 0.0)) == near_turn.pose


def test_without_handled_from_a_turned_object_is_handled_as_placed(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(path, path.read_text().replace('handled_from: pick', '# no handled_from'))
    target = ConfigStore(cell).target('bay_2_stack_2_front/pick', ROBOT, Pose2D(7.0, 1.0, 0.0))
    assert target.instance.pose.yaw == 3.1416 and target.pose.x == pytest.approx(9.0, abs=1e-3)


def test_handled_from_names_a_base_target_of_the_type(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(path, path.read_text().replace('handled_from: pick', 'handled_from: rails_top'))
    with pytest.raises(TargetError, match=r"handled_from 'rails_top' moves hand, it must be a base target"):
        ConfigStore(cell).target('bay_2_stack_2_front/pick', ROBOT, Pose2D(7.0, 1.0, 0.0))
    edit(path, path.read_text().replace('handled_from: rails_top', 'handled_from: gone'))
    with pytest.raises(TargetError, match=r"handled_from 'gone' is not a target of frame_type2 objects"):
        ConfigStore(cell).target('bay_2_stack_2_front/pick', ROBOT, Pose2D(7.0, 1.0, 0.0))


# ---- stacks ---------------------------------------------------------------------------------------
def test_a_stack_target_follows_the_count_in_the_object_state(new):
    for expected in (0.4, 0.3, 0.2):
        assert new.target('bay_1_stack_1_front/rails_top', ROBOT).instance.top_z == pytest.approx(expected)
        new.take_object('bay_1_stack_1_front')
    with pytest.raises(TargetError, match='was taken in an earlier step or run'):
        new.target('bay_1_stack_1_front/rails_top', ROBOT)  # count 0
    with pytest.raises(TargetError, match='was taken in an earlier step or run'):
        new.target('bay_1_stack_1_front/pick', ROBOT)


# ---- the errors of the references (section 3) -----------------------------------------------------
def test_error_ambiguous_plain_name(cell):
    append(cell / 'robots' / ROBOT / 'targets.yaml', 'set_down: {moves: base, x: 0.0, y: 0.0, yaw: 0.0}\n')
    with pytest.raises(TargetError) as error:
        ConfigStore(cell).target('set_down', ROBOT)
    assert str(error.value) == (
        "target 'set_down' is in cell/targets.yaml and robots/bot_two_arm/targets.yaml: rename one"
    )


def test_error_missing_plain_name(new):
    cell, robot = len(new.target_names(ROBOT)['base targets']), 9
    with pytest.raises(TargetError) as error:
        new.target('wating', ROBOT)
    text = str(error.value)
    assert text.startswith("unknown target 'wating'; did you mean waiting? (cell: 6, robot: 9; ")
    assert 'object targets need <instance>/<name>' in text and cell and robot


def test_error_plain_name_of_an_object_target_says_it_needs_an_instance(new):
    with pytest.raises(TargetError) as error:
        new.target('rails_top', ROBOT)
    assert 'needs an instance: <instance>/rails_top, e.g. bay_1_stack_1_back/rails_top' in str(error.value)


def test_error_missing_instance(new):
    with pytest.raises(TargetError) as error:
        new.target('bay_9/pick', ROBOT)
    text = str(error.value)
    assert text.startswith("unknown target 'bay_9/pick': no object 'bay_9' in cell/objects.yaml, instances: ")
    assert 'bay_1_stack_1_back' in text


def test_error_taken_instance(new):
    new.take_object('bay_2_stack_2_back')  # a single object: gone
    with pytest.raises(
        TargetError,
        match=r"'bay_2_stack_2_back' was taken in an earlier step or run \(object state: "
        r'state/objects.yaml\); reset the sim',
    ):
        new.target('bay_2_stack_2_back/pick', ROBOT)
    with pytest.raises(TargetError, match='was taken'):
        new.target('bay_2_stack_2_back/rails_top', ROBOT)


def test_error_missing_name_on_a_type(new):
    with pytest.raises(TargetError) as error:
        new.target('bay_1_stack_1_front/rails_toop', ROBOT)
    assert str(error.value) == (
        "object 'frame_type2' has no target 'rails_toop' (it has: hook_plate, pick, rails_top, rails_top_steep)"
    )


def test_error_the_robot_has_none(new):
    with pytest.raises(TargetError) as error:
        new.target('karakuri_hanger/staging', 'bot_plain')
    assert str(error.value) == (
        "robot 'bot_plain' has no 'staging' target for karakuri_hanger objects "
        '(robots/bot_plain/objects/karakuri_hanger.yaml)'
    )
    assert new.target('bay_1_stack_1_front/pick', 'bot_plain').pose == Pose2D(5.15, 1.0, 0.0, 'cell')


def test_error_kind_mismatch(new):
    with pytest.raises(TargetError) as error:
        new.target('waiting', ROBOT, kinds=('hand', 'joints'), context='via takes hand or joint targets')
    assert str(error.value) == "'waiting' moves the base: via takes hand or joint targets"
    with pytest.raises(TargetError, match="'home' moves joints: expected a target that moves a hand"):
        new.target('home', ROBOT, kinds='hand')
    assert new.target('home', ROBOT, kinds=('hand', 'joints')).kind == 'joints'
    assert new.target('bay_1_stack_1_front/rails_top', ROBOT, kinds='hand').kind == 'hand'


def test_error_frame_mismatch_for_the_held_object(new):
    carry = new.target('carry', ROBOT)
    assert carry.needs_held(None) == (
        "'carry' is a base-frame object target for frame_type2: a held frame_type2 is needed, nothing is held"
    )
    assert carry.needs_held('part_box') == (
        "'carry' is a base-frame object target for frame_type2: a held frame_type2 is needed, a part_box is held"
    )
    assert carry.needs_held('frame_type2') is None
    assert new.target('karakuri_hanger/hook', ROBOT).needs_held(None) is None  # names no type


@pytest.mark.parametrize(
    'reference,word',
    [('stance:bay_2_stack_2_front/pick', 'stance'), ('location:waiting', 'location'), ('pose:home', 'pose')],
)
def test_error_old_syntax(new, reference, word):
    with pytest.raises(TargetError) as error:
        new.target(reference, ROBOT)
    assert str(error.value) == (
        f"'{reference}': the {word}: prefix was removed in schema 4: run taskweave config migrate"
    )


def test_error_old_dotted_form(new):
    with pytest.raises(
        TargetError, match=r"'frame_type2.rails_top': the <object>.<point> form was removed in schema 4"
    ):
        new.target('frame_type2.rails_top', ROBOT)


def test_a_reference_with_two_separators_is_refused(new):
    with pytest.raises(TargetError, match='expected <name> or <instance>/<name>'):
        new.target('a/b/c', ROBOT)


def test_target_errors_are_key_errors_with_plain_messages(new):
    with pytest.raises(KeyError):
        new.target('nothing', ROBOT)
    assert not str(TargetError("it's: a message")).startswith('"')


# ---- the entries themselves -----------------------------------------------------------------------
def test_settings_for_an_unknown_target_are_refused(cell):
    append(cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml', '')
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(path, path.read_text().replace('  rails_top: {elbow: up}', '  rails_top: {elbow: up}\n  ghost: {elbow: up}'))
    with pytest.raises(TargetError, match=r"settings for an unknown target 'ghost' of frame_type2 objects"):
        ConfigStore(cell).target('bay_1_stack_1_front/ghost', ROBOT)


def test_a_stance_only_the_robot_has_needs_moves(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'part_box.yaml'
    append(path, '  side: {x: 0.0, y: 0.0, yaw: 0.0}\n')
    with pytest.raises(TargetError, match='settings for an unknown target'):
        ConfigStore(cell).target('part_box_1/side', ROBOT)


def test_moves_must_agree_between_layers(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(
        path,
        path.read_text().replace('  rails_top: {elbow: up}', '  rails_top: {moves: base, x: 0.0, y: 0.0, yaw: 0.0}'),
    )
    with pytest.raises(
        TargetError,
        match=r"target 'rails_top' moves hand in objects/frame_type2.yaml but base in "
        r'robots/bot_two_arm/objects/frame_type2.yaml: they must agree',
    ):
        ConfigStore(cell).target('bay_1_stack_1_front/rails_top', ROBOT)


def test_path_and_a_single_pose_together_are_an_error(cell):
    append(
        cell / 'cell' / 'targets.yaml', 'both: {moves: base, x: 1.0, y: 0.0, yaw: 0.0, path: [{x: 1, y: 1, yaw: 0}]}\n'
    )
    with pytest.raises(TargetError, match=r"target 'both' \(cell/targets.yaml\): path and a single pose \(x, y, yaw\)"):
        ConfigStore(cell).target('both', ROBOT)


def test_a_path_target_visits_waypoints_in_order(cell):
    append(
        cell / 'cell' / 'targets.yaml', 'tour: {moves: base, path: [{x: 1, y: 0, yaw: 0}, {x: 2, y: 1, yaw: 1.0}]}\n'
    )
    tour = ConfigStore(cell).target('tour', ROBOT)
    assert tour.is_path and [(p.x, p.y) for p in tour.poses] == [(1.0, 0.0), (2.0, 1.0)] and tour.pose.yaw == 1.0


def test_a_kind_has_its_own_keys(cell):
    append(cell / 'cell' / 'targets.yaml', 'bad: {moves: base, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n')
    with pytest.raises(TargetError, match='does not apply to a target that moves the base'):
        ConfigStore(cell).target('bad', ROBOT)


def test_an_explicit_frame_must_say_what_the_file_says(cell):
    append(
        cell / 'cell' / 'targets.yaml',
        'same: {moves: base, frame: cell, x: 0.0, y: 0.0, yaw: 0.0}\n'
        'other: {moves: base, frame: base, x: 0.0, y: 0.0, yaw: 0.0}\n',
    )
    store = ConfigStore(cell)
    assert store.target('same', ROBOT).frame == 'cell'
    with pytest.raises(TargetError, match="frame 'base' does not fit this file"):
        store.target('other', ROBOT)


# ---- the merge of an object type's layers (2.7) ---------------------------------------------------
def test_the_merge_replaces_whole_keys_and_merges_grip_alone_and_joints_per_part():
    base = {
        'moves': 'hand',
        'left_tool': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]},
        'grip_m': {'left': 0.1, 'right': 0.1},
        'alone': {'left_tool': {'xyz': [1, 1, 1], 'rpy': [0, 0, 0]}},
        'joints': {'torso': 0.1, 'head': [0, 0]},
    }
    over = {
        'left_tool': {'xyz': [9, 9, 9], 'rpy': [1, 1, 1]},
        'grip_m': {'left': 0.2},
        'alone': {'right_tool': {'xyz': [2, 2, 2], 'rpy': [0, 0, 0]}},
        'joints': {'head': [1, 1]},
        'elbow': 'up',
    }
    merged = model.merge_layers('x', [('a', base), ('b', over)])
    assert merged['left_tool']['xyz'] == [9, 9, 9] and merged['grip_m'] == {'left': 0.2, 'right': 0.1}
    assert set(merged['alone']) == {'left_tool', 'right_tool'}
    assert merged['joints'] == {'torso': 0.1, 'head': [1, 1]} and merged['elbow'] == 'up'
    assert merged['moves'] == 'hand'


def test_the_merge_is_later_wins_in_the_order_of_the_four_layers(cell):
    obj = cell / 'objects' / 'frame_type2.yaml'
    robot = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    append(
        obj,
        """instances:
  bay_1_stack_1_front:
    rails_top: {elbow: in, approach_m: 0.3}
""",
    )
    append(
        robot,
        """  bay_1_stack_1_front: {rails_top: {elbow: down}}
""",
    )
    store = ConfigStore(cell)
    assert store.target('bay_1_stack_1_front/rails_top', ROBOT).data['elbow'] == 'down'  # layer 4 wins
    assert store.target('bay_1_stack_1_front/rails_top', ROBOT).data['approach_m'] == 0.3  # layer 3 over 1
    assert store.target('bay_1_stack_1_front/rails_top', ROBOT).files == (
        'objects/frame_type2.yaml',
        'robots/bot_two_arm/objects/frame_type2.yaml',
    )
    assert store.target('bay_2_stack_2_front/rails_top', ROBOT).data['elbow'] == 'up'  # layers 1 and 2
    assert store.target('bay_2_stack_2_front/rails_top', ROBOT).data['approach_m'] == 0.1


def test_a_layer_with_a_path_replaces_a_single_pose_and_back():
    base = {'moves': 'object', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    merged = model.merge_layers('x', [('a', base), ('b', {'path': [{'xyz': [1, 1, 1], 'rpy': [0, 0, 0]}]})])
    assert 'xyz' not in merged and merged['path'][0]['xyz'] == [1, 1, 1]
    merged = model.merge_layers('x', [('a', merged), ('b', {'xyz': [2, 2, 2], 'rpy': [0, 0, 0]})])
    assert 'path' not in merged and merged['xyz'] == [2, 2, 2]
    assert model.target_problems(merged, 'object') == []


def test_an_instance_override_changes_only_the_keys_it_gives(cell):
    robot = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    append(robot, '  bay_1_stack_1_front: {pick: {x: -2.0}}\n')
    store = ConfigStore(cell)
    assert store.target('bay_1_stack_1_front/pick', ROBOT).pose == Pose2D(4.0, 1.0, 0.0, 'cell')


# ---- joint groups ---------------------------------------------------------------------------------
GROUPS = {'torso': ['lift', 'pitch'], 'head': ['pan', 'tilt'], 'left_arm': ['a1', 'a2', 'a3'], 'single': ['only']}


def test_group_values_numbers_lists_and_null():
    assert model.group_joint_values({'head': [0.1, 0.2]}, GROUPS) == {'pan': 0.1, 'tilt': 0.2}
    assert model.group_joint_values({'head': [None, 0.2], 'single': 1}, GROUPS) == {'tilt': 0.2, 'only': 1.0}
    assert model.group_joint_values({'torso': 0.4}, GROUPS, 'lift') == {'lift': 0.4}  # the lift joint
    assert model.group_joint_values({'torso': [0.4, None]}, GROUPS, 'lift') == {'lift': 0.4}
    assert model.group_joint_values({'torso': 'auto', 'left_arm': [1, 2, 3]}, GROUPS, 'lift') == {
        'a1': 1.0,
        'a2': 2.0,
        'a3': 3.0,
    }
    assert model.group_joint_values({'head': [1, 2], 'torso': 0.1}, GROUPS, 'lift', only=['head']) == {
        'pan': 1.0,
        'tilt': 2.0,
    }
    assert model.torso_setting({'torso': 'keep'}) == 'keep' and model.torso_setting({'head': [0, 0]}) is None


@pytest.mark.parametrize(
    'joints,torso,message',
    [
        ({'neck': [0, 0]}, 'lift', "joint group 'neck': this robot has no such group (it has: head, left_arm"),
        ({'head': [0.1]}, None, "joint group 'head' has 2 joints (pan, tilt): give a list of 2 values"),
        ({'left_arm': 0.1}, None, "joint group 'left_arm' has 3 joints (a1, a2, a3): give a list of 3 values"),
        ({'torso': 0.1}, None, "joint group 'torso' has 2 joints (lift, pitch): give a list of 2 values"),
        ({'head': [0, 'x']}, None, "'x' is not a number or null"),
        ({'head': 'auto'}, None, "'auto' is not a number or a list"),
        ({'head': True}, None, 'is not a number or a list'),
    ],
)
def test_group_values_errors_name_the_group(joints, torso, message):
    with pytest.raises(GroupError, match=message.replace('(', r'\(').replace(')', r'\)')):
        model.group_joint_values(joints, GROUPS, torso)


def test_a_robot_without_a_head_group_rejects_head_values(new):
    assert target_joint_problems(new, 'bot_plain', {'head': [0.0, 0.0]})[0].startswith(
        "joint group 'head': this robot has no such group"
    )
    assert target_joint_problems(new, ROBOT, {'head': [0.0, 0.0], 'torso': 0.5}) == []
    assert 'outside' in target_joint_problems(new, ROBOT, {'torso': 0.9})[0]
    assert 'has 2 joints' in target_joint_problems(new, ROBOT, {'head': [0.0]})[0]


def test_joint_part_problems_without_a_robot():
    assert model.joints_problems({'torso': 'auto'}, spatial=True) == []
    assert 'needs a hand or object pose' in model.joints_problems({'torso': 'auto'}, spatial=False)[0]
    assert 'for the torso only' in model.joints_problems({'head': 'keep'}, spatial=True)[0]
    assert 'not a number, a list' in model.joints_problems({'torso': 'high'}, spatial=True)[0]
    assert model.joints_problems({'torso': 'keep'}, spatial=False) == []


# ---- tool axes and roll-pitch-yaw -----------------------------------------------------------------
@pytest.mark.parametrize(
    'approach,closing,rpy',
    [
        ((1, 0, 0), (0, -1, 0), (0.0, -math.pi / 2, -math.pi)),  # the grasp pose of the design: pitch -90, roll 0
        ((1, 0, 0), (0, 1, 0), (0.0, math.pi / 2, 0.0)),
        ((0, 0, 1), (0, 1, 0), (0.0, 0.0, 0.0)),
        ((0, 1, 0), (0, 0, 1), None),
        ((0.3, -0.5, 0.8), (0.9, 0.3, -0.1), None),
    ],
)
def test_rpy_from_axes_and_back(approach, closing, rpy):
    found = model.rpy_from_axes(approach, closing)
    if rpy is not None:
        assert found == pytest.approx(rpy, abs=1e-9)
    z, y = model.axes_from_rpy(found)
    unit = math.sqrt(sum(c * c for c in approach))
    assert z == pytest.approx([c / unit for c in approach], abs=1e-9)
    dot = sum(a * b for a, b in zip(closing, approach, strict=True)) / unit**2
    square = [c - dot * a for c, a in zip(closing, approach, strict=True)]
    norm = math.sqrt(sum(c * c for c in square))
    assert y == pytest.approx([c / norm for c in square], abs=1e-9)


def test_the_design_numbers_for_the_grasp_pose():
    assert model.rpy_from_axes((1, 0, 0), (0, -1, 0)) == pytest.approx((0.0, -1.5708, -3.1416), abs=1e-4)
    assert model.rpy_from_axes((1, 0, 0), (0, 1, 0)) == pytest.approx((0.0, 1.5708, 0.0), abs=1e-4)
    with pytest.raises(ValueError, match='no direction'):
        model.rpy_from_axes((0, 0, 0), (0, 1, 0))


# ---- the move skill's options per kind ------------------------------------------------------------
def test_move_options_per_kind():
    assert model.move_option_problems('base', {'torso': 'keep', 'motion': {'mode': 'nav', 'speed': 0.5}}) == []
    assert model.move_option_problems('base', {'hands': 'all'}) == ['move base: option hands does not apply']
    assert model.move_option_problems('base', {'torso': 'auto'})[0].startswith('move base: torso')
    assert (
        model.move_option_problems(
            'hand',
            {
                'hands': 'any_one',
                'offset_m': 0.1,
                'torso': 'auto',
                'motion': {'mode': 'linear', 'via': ['a'], 'planner': 'direct'},
            },
        )
        == []
    )
    assert model.move_option_problems('object', {'motion': {'via': ['a']}}) == [
        'move object: motion.via does not apply'
    ]
    assert model.move_option_problems('joints', {'only': ['head'], 'motion': {'planner': 'planned'}}) == []
    assert model.move_option_problems('joints', {'offset_m': 0.1}) == ['move joints: option offset_m does not apply']
    assert 'motion.mode' in model.move_option_problems('hand', {'motion': {'mode': 'nav'}})[0]
    assert model.move_option_problems('elbow', {'tolerance_deg': 5, 'motion': {'tolerance': 0.1}}) == []
    assert model.move_option_problems('straight', {'motion': {'mode': 'nav'}})[0].startswith(
        'move straight: motion.mode'
    )


# ---- writing targets ------------------------------------------------------------------------------
def reread(cell) -> ConfigStore:
    return ConfigStore(cell)


def test_save_target_updates_a_cell_target_and_keeps_comments_and_style(cell):
    store = ConfigStore(cell)
    written = store.save_target('waiting', {'moves': 'base', 'x': 2.5, 'y': 0.0, 'yaw': 0.0}, ROBOT)
    assert written == [cell / 'cell' / 'targets.yaml']
    text = (cell / 'cell' / 'targets.yaml').read_text()
    assert 'waiting: {moves: base, x: 2.5, y: 0.0, yaw: 0.0}       # centre of the waiting zone' in text
    assert text.startswith('# targets in the cell frame')
    assert reread(cell).target('waiting', ROBOT).pose.x == 2.5


def test_save_target_creates_targets_where_they_belong(cell):
    store = ConfigStore(cell)
    store.save_target('bench', {'moves': 'base', 'x': 1.0, 'y': 2.0, 'yaw': 0.5}, ROBOT)
    store.save_target('lift_mid', {'moves': 'joints', 'joints': {'torso': 0.3}}, ROBOT)
    store.save_target(
        'pose_b',
        {
            'moves': 'hand',
            'right_tool': {'xyz': [0.4, 0.2, 0.5], 'rpy': [0, 0, 0]},
            'elbow': 'out',
            'joints': {'torso': 0.4},
        },
        ROBOT,
    )
    store.save_target(
        'dock_tool', {'moves': 'hand', 'frame': 'cell', 'left_tool': {'xyz': [1, 1, 1], 'rpy': [0, 0, 0]}}, ROBOT
    )
    cell_text = (cell / 'cell' / 'targets.yaml').read_text()
    robot_text = (cell / 'robots' / ROBOT / 'targets.yaml').read_text()
    assert 'bench: {moves: base, x: 1.0, y: 2.0, yaw: 0.5}' in cell_text and 'dock_tool:' in cell_text
    assert 'frame:' not in cell_text
    assert 'lift_mid: {moves: joints, joints: {torso: 0.3}}' in robot_text and 'pose_b:' in robot_text
    fresh = reread(cell)
    assert fresh.target('dock_tool', ROBOT).frame == 'cell'
    assert fresh.target('pose_b', ROBOT).tools['right_tool'].xyz == (0.4, 0.2, 0.5)
    assert fresh.check_pack() == []


def test_save_target_refuses_an_invalid_target_and_a_name_in_the_other_namespace(cell):
    store = ConfigStore(cell)
    with pytest.raises(ValueError, match='does not apply to a target that moves the base'):
        store.save_target('x', {'moves': 'base', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, ROBOT)
    with pytest.raises(ValueError, match='a joints target needs joints'):
        store.save_target('x', {'moves': 'joints'}, ROBOT)
    with pytest.raises(ValueError, match='no frame'):
        store.save_target('x', {'moves': 'joints', 'frame': 'cell', 'joints': {'torso': 1}}, ROBOT)
    with pytest.raises(ValueError, match='invalid point name'):
        store.save_target('1x', {'moves': 'base', 'x': 0, 'y': 0, 'yaw': 0}, ROBOT)
    with pytest.raises(ValueError, match='nothing written'):  # the schema of the file
        store.save_target('x', {'moves': 'base', 'x': 'far', 'y': 0, 'yaw': 0}, ROBOT)
    assert 'far' not in (cell / 'cell' / 'targets.yaml').read_text()
    with pytest.raises(TargetWriteError, match='rename one'):  # waiting is a cell target; the robot's file would clash
        append(cell / 'robots' / ROBOT / 'targets.yaml', 'waiting: {moves: base, x: 0, y: 0, yaw: 0}\n')
        ConfigStore(cell).save_target('waiting', {'moves': 'base', 'x': 0, 'y': 0, 'yaw': 0}, ROBOT)


def test_save_target_on_an_object_type_splits_robot_and_object_parts(cell):
    store = ConfigStore(cell)
    written = store.save_target(
        'bay_1_stack_1_front/rails_wide',
        {
            'moves': 'hand',
            'left_tool': {'xyz': [0, 0.3, 0.07], 'rpy': [-1.9, 0, -1.5]},
            'approach_m': 0.12,
            'elbow': 'up',
            'grip_m': {'left': 0.02},
        },
        ROBOT,
    )
    assert written == [cell / 'objects' / 'frame_type2.yaml', cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml']
    robot_file = yaml.safe_load((cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml').read_text())
    assert robot_file['targets']['rails_wide'] == {'elbow': 'up', 'grip_m': {'left': 0.02}}
    object_file = yaml.safe_load((cell / 'objects' / 'frame_type2.yaml').read_text())
    assert 'elbow' not in object_file['targets']['rails_wide']
    found = reread(cell).target('bay_2_stack_2_front/rails_wide', ROBOT)
    assert found.get('elbow') == 'up' and found.get('approach_m') == 0.12 and found.tools['left_tool'].xyz[1] == 0.3
    assert reread(cell).target('bay_2_stack_2_front/rails_wide', 'bot_plain').get('elbow') is None
    assert reread(cell).check_pack() == []
    assert '# first guess' not in (cell / 'objects' / 'frame_type2.yaml').read_text()
    assert '# rails_top tilted 40 deg steeper' in (cell / 'objects' / 'frame_type2.yaml').read_text()


def test_save_target_saves_a_stance_and_an_instance_override(cell):
    store = ConfigStore(cell)
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    store.save_target('bay_1_stack_1_front/pick', {'moves': 'base', 'x': -1.1, 'y': 0.0, 'yaw': 0.0}, ROBOT)
    assert yaml.safe_load(path.read_text())['targets']['pick']['x'] == -1.1
    assert '# chassis front 0.44 m ahead of base_link' in path.read_text()
    store.save_target(
        'bay_1_stack_1_front/pick', {'moves': 'base', 'x': -1.3, 'y': 0.2, 'yaw': 0.0}, ROBOT, instance_override=True
    )
    assert yaml.safe_load(path.read_text())['instances']['bay_1_stack_1_front'] == {
        'pick': {'x': -1.3, 'y': 0.2, 'yaw': 0.0}
    }
    fresh = reread(cell)
    assert fresh.target('bay_1_stack_1_front/pick', ROBOT).pose == Pose2D(4.7, 1.2, 0.0, 'cell')
    assert fresh.target('bay_2_stack_2_front/pick', ROBOT, Pose2D(9, 1, 0)).pose.x == pytest.approx(9.1, abs=0.01)
    store.save_target('part_box_1/side', {'moves': 'base', 'x': 0.0, 'y': 0.5, 'yaw': 1.5}, ROBOT)
    assert reread(cell).target('part_box_1/side', ROBOT).kind == 'base'
    with pytest.raises(TargetWriteError, match='cannot be saved as moving hand'):
        store.save_target(
            'bay_1_stack_1_front/pick', {'moves': 'hand', 'left_tool': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}}, ROBOT
        )
    with pytest.raises(KeyError, match='no object'):
        store.save_target('nothing/pick', {'moves': 'base', 'x': 0, 'y': 0, 'yaw': 0}, ROBOT)
    with pytest.raises(ValueError, match='instance_override is for'):
        store.save_target('pick', {'moves': 'base', 'x': 0, 'y': 0, 'yaw': 0}, ROBOT, instance_override=True)


def test_save_target_sets_only_the_robots_setting(cell):
    store = ConfigStore(cell)
    store.save_target('part_box_1/top_grasp', {'elbow': 'in'}, ROBOT)
    assert reread(cell).target('part_box_1/top_grasp', ROBOT).get('elbow') == 'in'
    assert 'elbow' not in (cell / 'objects' / 'part_box.yaml').read_text()
    with pytest.raises(TargetWriteError, match='settings for an unknown target'):
        store.save_target('part_box_1/nothing', {'elbow': 'in'}, ROBOT)
    store.save_target('part_box_1/top_grasp', {}, ROBOT)  # clearing the settings
    assert 'top_grasp' not in (cell / 'robots' / ROBOT / 'objects' / 'part_box.yaml').read_text()


def test_delete_target(cell):
    store = ConfigStore(cell)
    assert store.delete_target('taught_dock', ROBOT) == [cell / 'cell' / 'targets.yaml']
    assert 'taught_dock' not in (cell / 'cell' / 'targets.yaml').read_text()
    assert '# centre of the waiting zone' in (cell / 'cell' / 'targets.yaml').read_text()
    store.delete_target('lift_carry', ROBOT)
    store.delete_target('bay_1_stack_1_front/rails_top_steep', ROBOT)
    store.delete_target('bay_1_stack_1_back/pick', ROBOT, instance_override=True)
    fresh = reread(cell)
    assert 'lift_carry' not in fresh.target_names(ROBOT)['joint targets']
    assert 'rails_top_steep' not in fresh.object_target_names()['frame_type2']
    assert 'bay_1_stack_1_back' not in (cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml').read_text()
    assert fresh.target('bay_1_stack_1_back/pick', ROBOT).pose.y == 2.5  # the type's pick again
    store.delete_target('bay_1_stack_1_front/rails_top', ROBOT)  # the settings go with it
    assert 'rails_top:' not in (cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml').read_text()
    with pytest.raises(TargetError, match='unknown target'):
        store.delete_target('nowhere', ROBOT)
    with pytest.raises(TargetError, match='has no target'):
        store.delete_target('bay_1_stack_1_front/rails_toop', ROBOT)
    assert reread(cell).check_pack() == []


def test_rename_target_keeps_position_and_comment(cell):
    store = ConfigStore(cell)
    store.rename_target('point_a', 'point_b', ROBOT)
    text = (cell / 'cell' / 'targets.yaml').read_text()
    assert 'point_b: {moves: base, x: 4.0, y: 3.5, yaw: 0.0}       # free floor between waiting zone and bins' in text
    assert text.index('point_b') < text.index('set_down')
    with pytest.raises(TargetWriteError, match='exists already'):
        store.rename_target('point_b', 'waiting', ROBOT)
    with pytest.raises(ValueError, match='invalid point name'):
        store.rename_target('point_b', 'a b', ROBOT)


def test_rename_a_stance_follows_handled_from_and_a_shared_target_follows_the_robots_settings(cell):
    store = ConfigStore(cell)
    store.rename_target('bay_1_stack_1_front/pick', 'aisle', ROBOT)
    robot_text = (cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml').read_text()
    assert 'handled_from: aisle' in robot_text and 'aisle:' in robot_text and '  pick:' not in robot_text
    assert 'bay_1_stack_1_back: {aisle:' in robot_text  # the instance overrides follow
    assert 'pick' in (cell / 'robots' / 'bot_plain' / 'objects' / 'frame_type2.yaml').read_text()
    fresh = reread(cell)
    assert fresh.target('bay_1_stack_1_front/aisle', ROBOT).pose == Pose2D(5.0, 1.0, 0.0, 'cell')
    assert fresh.target('bay_2_stack_2_front/aisle', ROBOT, Pose2D(7.0, 1.0, 0.0)).instance.pose.yaw == pytest.approx(
        0.0, abs=1e-3
    )
    fresh.rename_target('bay_1_stack_1_front/rails_top', 'rails_a', ROBOT)
    again = reread(cell)
    assert 'rails_top:' not in (cell / 'objects' / 'frame_type2.yaml').read_text()
    assert again.target('bay_1_stack_1_front/rails_a', ROBOT).get('elbow') == 'up'
    with pytest.raises(TargetWriteError, match='has a target'):
        again.rename_target('bay_1_stack_1_front/rails_a', 'rails_top_steep', ROBOT)
    assert again.check_pack() == []


def test_save_target_can_rename_in_the_same_call(cell):
    ConfigStore(cell).save_target('point_a', {'moves': 'base', 'x': 4.5, 'y': 3.5, 'yaw': 0.0}, ROBOT, rename='point_z')
    assert reread(cell).target('point_z', ROBOT).pose.x == 4.5


# ---- a robot pack's targets.yaml with a keyed cell override ---------------------------------------
def pack_cell(tmp_path) -> Path:
    """The new cell with the robot's profile, embodiment and targets in a robot pack beside it and an override."""
    cell = copy_cell(tmp_path)
    pack = tmp_path / 'packs' / ROBOT
    pack.mkdir(parents=True)
    for name in ('profile.yaml', 'embodiment.yaml', 'targets.yaml'):
        shutil.move(str(cell / 'robots' / ROBOT / name), str(pack / name))
    edit(pack / 'pack.yaml', f'name: {ROBOT}\nversion: 0.3.0\nschema_version: 2\n')
    edit(cell / 'system.yaml', 'robot_packs: [../packs]\n')
    return cell


def test_the_cell_overrides_a_robot_pack_target_whole_and_keeps_the_rest(tmp_path):
    cell = pack_cell(tmp_path)
    edit(
        cell / 'robots' / ROBOT / 'targets.yaml',
        """~override: keys
frame_grasp_approach:
  moves: hand
  left_tool: {xyz: [0.5, -0.3, 0.6], rpy: [0.0, -1.5708, -3.1416]}
  elbow: out
lift_high: ~delete
""",
    )
    store = ConfigStore(cell)
    grasp = store.target('frame_grasp_approach', ROBOT)
    assert grasp.tools == {'left_tool': grasp.tools['left_tool']} and grasp.get('elbow') == 'out'
    assert grasp.joints == {}  # replaced whole: depth 1, the pack's joints part is gone
    assert store.target('home', ROBOT).kind == 'joints'  # the rest is the pack's
    with pytest.raises(TargetError, match='unknown target'):
        store.target('lift_high', ROBOT)
    assert store.check_pack() == []


def test_saving_a_pack_target_writes_a_keyed_override_and_leaves_the_pack(tmp_path):
    cell = pack_cell(tmp_path)
    pack_file = tmp_path / 'packs' / ROBOT / 'targets.yaml'
    before = pack_file.read_text()
    edit(
        cell / 'robots' / ROBOT / 'targets.yaml',
        '~override: keys\nlift_carry: {moves: joints, joints: {torso: 0.45}}\n',
    )
    store = ConfigStore(cell)
    written = store.save_target('lift_high', {'moves': 'joints', 'joints': {'torso': 0.65}}, ROBOT)
    assert written == [cell / 'robots' / ROBOT / 'targets.yaml'] and pack_file.read_text() == before
    override = yaml.safe_load((cell / 'robots' / ROBOT / 'targets.yaml').read_text())
    assert override == {
        '~override': 'keys',
        'lift_carry': {'moves': 'joints', 'joints': {'torso': 0.45}},
        'lift_high': {'moves': 'joints', 'joints': {'torso': 0.65}},
    }
    assert ConfigStore(cell).target('lift_high', ROBOT).joints == {'torso': 0.65}
    assert ConfigStore(cell).check_pack() == []


def test_a_cell_pack_that_is_not_keyed_must_be_a_whole_targets_file(tmp_path):
    cell = pack_cell(tmp_path)
    edit(cell / 'robots' / ROBOT / 'targets.yaml', 'only_one: {moves: joints, joints: {torso: 0.2}}\n')
    store = ConfigStore(cell)
    assert store.target_names(ROBOT)['joint targets'] == ['only_one']


def test_a_keyed_override_with_no_pack_file_is_refused(tmp_path):
    cell = copy_cell(tmp_path)
    edit(cell / 'robots' / ROBOT / 'targets.yaml', '~override: keys\nhome: ~delete\n')
    with pytest.raises(ConfigError, match='keyed override'):
        ConfigStore(cell).target('home', ROBOT)


# ---- the formats that are format only, and the type's own targets -----------------------------------------------
def test_cell_frame_hand_and_object_targets_parse_and_report_their_frame(cell):
    append(
        cell / 'cell' / 'targets.yaml',
        'dock_tool: {moves: hand, left_tool: {xyz: [1, 1, 1], rpy: [0, 0, 0]}}\n'
        'dock_part: {moves: object, xyz: [2, 2, 0], rpy: [0, 0, 1.0]}\n',
    )
    store = ConfigStore(cell)
    tool, part = store.target('dock_tool', ROBOT), store.target('dock_part', ROBOT)
    assert (tool.kind, tool.frame, part.kind, part.frame) == ('hand', 'cell', 'object', 'cell')
    assert tool.tools['left_tool'].frame == 'cell' and part.pose.frame == 'cell' and part.pose.xyz == (2.0, 2.0, 0.0)
    assert store.check_pack() == []


def test_a_base_frame_base_target_is_a_relative_drive_with_a_joints_part(cell):
    append(
        cell / 'robots' / ROBOT / 'targets.yaml',
        'back_out: {moves: base, x: -0.5, y: 0.0, yaw: 0.0, joints: {torso: 0.5}}\n',
    )
    store = ConfigStore(cell)
    back = store.target('back_out', ROBOT)
    assert (back.kind, back.frame) == ('base', 'base') and back.pose == Pose2D(-0.5, 0.0, 0.0, 'base')
    assert back.joints == {'torso': 0.5} and back.torso() == 0.5
    assert store.check_pack() == []


def test_a_hand_target_can_be_a_path_of_waypoints(cell):
    append(
        cell / 'robots' / ROBOT / 'targets.yaml',
        'sweep:\n  moves: hand\n  path:\n'
        '  - {left_tool: {xyz: [0.4, 0.2, 0.5], rpy: [0, 0, 0]}}\n'
        '  - {left_tool: {xyz: [0.5, 0.2, 0.6], rpy: [0, 0, 0]},\n'
        '     right_tool: {xyz: [0.5, -0.2, 0.6], rpy: [0, 0, 0]}}\n',
    )
    store = ConfigStore(cell)
    sweep = store.target('sweep', ROBOT)
    assert sweep.is_path and len(sweep.poses) == 2 and set(sweep.tools) == {'left_tool', 'right_tool'}
    assert set(sweep.poses[0]) == {'left_tool'}
    assert store.check_pack() == []


def test_a_type_target_without_a_placed_object_is_in_the_objects_own_frame(new):
    from taskweave_core.config import target_store

    rails = target_store.resolve_type(new, ROBOT, 'frame_type2', 'rails_top')
    assert rails.instance is None and rails.get('elbow') == 'up' and rails.tools['left_tool'].frame == 'frame_type2'
    assert target_store.resolve_type(new, None, 'frame_type2', 'rails_top').get('elbow') is None
    with pytest.raises(TargetError, match="object 'frame_type2' has no target 'x'"):
        target_store.resolve_type(new, ROBOT, 'frame_type2', 'x')


# ---- P6-1b: gaps found by the HMI -------------------------------------------------------------------
HAND = {'moves': 'hand', 'left_tool': {'xyz': [0.1, 0.2, 0.3], 'rpy': [0, 0, 0]}, 'approach_m': 0.12345678}


def crate_cell(cell) -> ConfigStore:
    """A type nothing is placed for."""
    edit(cell / 'objects' / 'crate.yaml', 'name: crate\ntargets: {}\n')
    return ConfigStore(cell)


def test_a_type_with_nothing_placed_has_targets_by_type_address(cell):
    store = crate_cell(cell)
    written = store.save_target('lid', {**HAND, 'elbow': 'up'}, ROBOT, type='crate')
    assert written == [cell / 'objects' / 'crate.yaml', cell / 'robots' / ROBOT / 'objects' / 'crate.yaml']
    fresh = reread(cell)
    found = fresh.type_target('crate', 'lid', ROBOT)
    assert found.frame == 'object' and found.instance is None and found.get('elbow') == 'up'
    assert fresh.type_target('crate', 'lid').get('elbow') is None  # no robot: the type's layer only
    assert fresh.object_target_names(ROBOT)['crate'] == ['lid']
    fresh.rename_target('lid', 'cover', ROBOT, type='crate')
    assert reread(cell).type_target('crate', 'cover', ROBOT).get('elbow') == 'up'
    reread(cell).delete_target('cover', ROBOT, type='crate')
    assert reread(cell).object_target_names(ROBOT)['crate'] == []
    with pytest.raises(TargetError, match='no object type'):
        store.save_target('lid', HAND, ROBOT, type='nothing')
    with pytest.raises(TargetWriteError, match='bare target name'):
        store.save_target('bay_1_stack_1_front/x', HAND, ROBOT, type='crate')
    with pytest.raises(TargetWriteError, match='instance_override'):
        store.save_target('lid', HAND, ROBOT, type='crate', instance_override=True)


def test_saving_a_hand_target_keeps_each_key_in_its_layer(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(path, path.read_text().replace('rails_top: {elbow: up}', 'rails_top: {elbow: up, approach_m: 0.2}'))
    store = ConfigStore(cell)
    assert store.target('bay_1_stack_1_front/rails_top', ROBOT).get('approach_m') == 0.2
    data = dict(store.target('bay_1_stack_1_front/rails_top', ROBOT).data)
    data['approach_m'] = 0.15
    data['grip_m'] = {'left': 0.02}
    store.save_target('bay_1_stack_1_front/rails_top', data, ROBOT)
    robot_file = yaml.safe_load(path.read_text())['targets']['rails_top']
    own = yaml.safe_load((cell / 'objects' / 'frame_type2.yaml').read_text())['targets']['rails_top']
    assert robot_file == {'elbow': 'up', 'approach_m': 0.15, 'grip_m': {'left': 0.02}}
    assert own['approach_m'] == 0.1 and 'elbow' not in own and 'grip_m' not in own
    assert reread(cell).target('bay_1_stack_1_front/rails_top', ROBOT).get('approach_m') == 0.15


def test_split_layers_by_part_and_new_keys():
    from taskweave_core.config.target_edit import split_layers

    own, mine = split_layers(
        {'moves': 'hand', 'a': 1, 'elbow': 'up', 'grip_m': {'left': 1, 'right': 2}},
        {'moves': 'hand', 'a': 0, 'grip_m': {'left': 0}},
        {'grip_m': {'right': 0}},
    )
    assert own == {'moves': 'hand', 'a': 1, 'grip_m': {'left': 1}}
    assert mine == {'elbow': 'up', 'grip_m': {'right': 2}}


def test_a_partial_instance_override_merges_with_the_type_for_the_old_stance_offsets(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(path, path.read_text().replace('instances:', 'instances:\n  bay_1_stack_1_front: {pick: {x: -1.6}}'))
    offsets = ConfigStore(cell).stance_offsets(ROBOT)
    assert offsets['instances']['bay_1_stack_1_front']['pick'] == {'x': -1.6, 'y': 0.0, 'yaw': 0.0}


def test_one_bad_target_does_not_hide_the_other_poses(cell):
    append(
        cell / 'robots' / ROBOT / 'targets.yaml',
        'bad_pose: {moves: joints, joints: {tail: 1.0}}\n'
        'bad_held: {moves: object, joints: {tail: 1.0}, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n',
    )
    store = ConfigStore(cell)
    assert 'home' in store.joint_poses(ROBOT) and 'bad_pose' not in store.joint_poses(ROBOT)
    assert 'carry' in store.held_poses(ROBOT).get('frame_type2', {})
    assert store.named_pose(ROBOT, 'lift_low')['joints']
    with pytest.raises(GroupError, match='tail'):
        store.named_pose(ROBOT, 'bad_pose')
    from taskweave_core.config import target_store as ts
    from taskweave_core.config.limits import pose_problems

    assert set(ts.pose_target_problems(store, ROBOT)) == {'bad_pose', 'bad_held'}
    assert 'tail' in pose_problems(store, ROBOT, 'bad_pose')[0]


def test_the_generic_point_editors_refuse_old_files_on_the_new_layout(new):
    with pytest.raises(ValueError, match='save_target'):
        new.update_point('cell/locations.yaml', ['x'], Pose2D(0, 0, 0))
    with pytest.raises(ValueError, match='save_target'):
        new.delete_point(f'robots/{ROBOT}/poses.yaml', ['home'])
    with pytest.raises(ValueError, match='save_target'):
        new.delete_point('objects/frame_type2.yaml', ['points', 'x'])


def test_write_refusals_are_target_write_errors_and_lookups_stay_key_errors(cell):
    from taskweave_core.config import TargetWriteError as exported

    assert exported is TargetWriteError and issubclass(TargetWriteError, ValueError)
    assert not issubclass(TargetWriteError, KeyError)
    store = ConfigStore(cell)
    for call in (
        lambda: store.save_target('x', {'moves': 'joints'}, ROBOT),
        lambda: store.save_target('1x', {'moves': 'base', 'x': 0, 'y': 0, 'yaw': 0}, ROBOT),
        lambda: store.save_target('part_box_1/top_grasp', {'moves': 'base', 'x': 0, 'y': 0, 'yaw': 0}, ROBOT),
        lambda: store.rename_target('point_a', 'waiting', ROBOT),
        lambda: store.save_target('pick', {'moves': 'base', 'x': 0, 'y': 0, 'yaw': 0}, ROBOT, instance_override=True),
    ):
        with pytest.raises(TargetWriteError):
            call()
    with pytest.raises(TargetError):
        store.delete_target('nowhere', ROBOT)


def test_renaming_an_instance_only_target_changes_only_that_instance(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(
        path,
        path.read_text().replace(
            'instances:',
            'instances:\n  bay_1_stack_1_front: {side: {moves: base, x: 0.0, y: 1.0, yaw: 0.0}}\n'
            '  bay_2_stack_2_front: {side: {moves: base, x: 0.0, y: -1.0, yaw: 0.0}}',
        ),
    )
    store = ConfigStore(cell)
    store.rename_target('bay_1_stack_1_front/side', 'edge', ROBOT)
    tables = yaml.safe_load(path.read_text())['instances']
    assert 'edge' in tables['bay_1_stack_1_front'] and 'side' not in tables['bay_1_stack_1_front']
    assert 'side' in tables['bay_2_stack_2_front'] and 'edge' not in tables['bay_2_stack_2_front']
    assert reread(cell).check_pack() == []


def test_deleting_a_type_target_with_overrides_is_refused_unless_cascaded(cell):
    store = ConfigStore(cell)  # pick has overrides in bay_1_stack_1_back and bay_2_stack_2_back, no moves in them
    with pytest.raises(TargetWriteError, match=r'bay_1_stack_1_back.*delete them together'):
        store.delete_target('bay_1_stack_1_front/pick', ROBOT)
    assert 'pick:' in (cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml').read_text()
    store.delete_target('bay_1_stack_1_front/pick', ROBOT, cascade=True)
    text = (cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml').read_text()
    assert 'pick' not in yaml.safe_load(text).get('targets', {})
    assert 'bay_1_stack_1_back' not in text and 'bay_2_stack_2_back' not in text


def test_joint_groups_of_is_the_inverse_of_group_values(new):
    from taskweave_core.config.limits import joint_groups_of

    joints = {'left_arm': [0.1, None, 0.3, None, None, None, None], 'torso': 0.4, 'head': [None, 0.2]}
    values = group_values(new, ROBOT, joints)
    assert joint_groups_of(new, ROBOT, values) == joints
    assert joint_groups_of(new, ROBOT, values, keep={'torso': 'auto', 'right_arm': 'keep'})['right_arm'] == 'keep'
    with pytest.raises(GroupError, match='not in a joint group'):
        joint_groups_of(new, ROBOT, {'tail_joint': 1.0})


def test_save_target_notes_the_entry_and_rounds_to_four_decimals(cell):
    import datetime

    store = ConfigStore(cell)
    store.save_target('bench', {'moves': 'base', 'x': 1.234567, 'y': 2.0, 'yaw': 0.5}, ROBOT)
    text = (cell / 'cell' / 'targets.yaml').read_text()
    assert 'bench: {moves: base, x: 1.2346, y: 2.0, yaw: 0.5}' in text
    assert f'# saved {datetime.date.today().isoformat()}' in text
    store.save_target('waiting', {'moves': 'base', 'x': 2.5, 'y': 0.0, 'yaw': 0.0}, ROBOT)  # keeps its comment
    assert '# centre of the waiting zone' in (cell / 'cell' / 'targets.yaml').read_text()
    store.save_target('waiting', {'moves': 'base', 'x': 2.6, 'y': 0.0, 'yaw': 0.0}, ROBOT, note='edited in the HMI')
    text = (cell / 'cell' / 'targets.yaml').read_text()
    assert '# edited in the HMI' in text and 'centre of the waiting zone' not in text
    assert reread(cell).check_pack() == []


def test_the_old_name_lists_include_a_hand_target_only_the_robot_has(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(
        path,
        path.read_text().replace(
            '  rails_top: {elbow: up}',
            '  rails_top: {elbow: up}\n  mine: {moves: hand, left_tool: {xyz: [0, 0, 0], rpy: [0, 0, 0]}}',
        ),
    )
    names = ConfigStore(cell).point_names(ROBOT)
    assert 'frame_type2.mine' in names['object points'] and 'bay_1_stack_1_front/mine' in names['placed object points']
    assert 'frame_type2.mine' not in ConfigStore(cell).point_names('bot_plain')['object points']


# ---- second round of fixes (P6-1c) -------------------------------------------------------------------------------
def test_a_schema_failure_after_an_edit_is_a_write_error_and_still_a_value_error(cell, monkeypatch):
    from taskweave_core.config import store as store_module

    def refuse(schema, data, label):
        raise ValueError(f'{label}: nope; nothing written')

    monkeypatch.setattr(store_module, '_check', refuse)
    with pytest.raises(TargetWriteError, match='nothing written') as refused:
        ConfigStore(cell).save_target('waiting', {'moves': 'base', 'x': 2.5, 'y': 0.0, 'yaw': 0.0}, ROBOT)
    assert isinstance(refused.value, ValueError)


def test_a_new_key_of_a_target_only_the_robot_file_defines_stays_in_that_file():
    from taskweave_core.config.target_edit import split_layers

    entry = {'moves': 'hand', 'left_tool': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}, 'approach_m': 0.04}
    robot_only = {'moves': 'hand', 'left_tool': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}}
    assert split_layers(entry, {}, robot_only) == ({}, entry)
    common, mine = split_layers(entry, {'moves': 'hand'}, {'elbow': 'up'})  # the type defines it: as before
    assert common['approach_m'] == 0.04 and 'approach_m' not in mine


def test_an_unknown_or_taken_instance_is_a_lookup_error(cell):
    assert isinstance(ConfigStore(cell).missing_instance('nothing', 'unknown nothing'), TargetError)
    with pytest.raises(TargetError):
        ConfigStore(cell).save_target('nothing/pick', {'elbow': 'in'}, ROBOT)


def test_the_old_stance_offsets_use_the_type_and_robot_layers_of_every_base_target(cell):
    edit(cell / 'objects' / 'part_box.yaml', (cell / 'objects' / 'part_box.yaml').read_text()
         + '  side: {moves: base, x: 0.5, y: 0.25, yaw: 1.5}\n')
    robot_file = cell / 'robots' / ROBOT / 'objects' / 'part_box.yaml'
    edit(robot_file, robot_file.read_text() + '  side: {y: 0.75}\n')   # a partial entry over the type's
    objects = ConfigStore(cell).stance_offsets(ROBOT)['objects']
    assert objects['part_box']['side'] == {'x': 0.5, 'y': 0.75, 'yaw': 1.5}
    assert objects['part_box']['pick'] == {'x': -0.8, 'y': 0.0, 'yaw': 0.0}   # defined in objects/part_box.yaml only
    assert objects['frame_type2']['pick']['x'] == -1.0   # a base target of the robot's file alone


def test_the_delete_refusal_for_operators_does_not_name_the_code_argument(cell):
    path = cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'
    edit(path, path.read_text().replace('{pick: {x: -1.528, y: 0.0, yaw: 0.0}}',
                                        '{pick: {x: -1.528, y: 0.0, yaw: 0.0}, rails_top_steep: {elbow: out}}', 1))
    with pytest.raises(TargetWriteError, match='still used by settings') as refused:
        ConfigStore(cell).delete_target('rails_top_steep', ROBOT, type='frame_type2')
    assert 'cascade' not in str(refused.value)


def test_the_option_table_holds_the_relative_and_hands_rules():
    assert model.RELATIVE_KEYS['hand']['at'] == ('at', 'rpy_deg') and model.RELATIVE_KEYS['object']['by'][0] == 'by'
    assert model.move_option_problems('hand', {'offset_m': 0.1}, relative=True) == [
        'move hand: option offset_m does not apply to a relative move'
    ]
    assert model.move_option_problems('object', {'reference': 'x', 'hands': 'all'}, relative=True) == [
        'move object: option reference does not apply to a relative move'
    ]
    assert model.move_option_problems('hand', {'offset_m': 0.1}) == []
    assert model.move_option_problems('object', {'hands': 'any_one'}, target=True) == [
        'move object: option hands any_one does not apply'
    ]
    assert model.move_option_problems('hand', {'hands': 'any_one'}, target=True) == []
    assert model.move_option_problems('object', {'hands': 'any_one'}) == []
