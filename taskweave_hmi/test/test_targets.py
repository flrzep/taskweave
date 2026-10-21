"""The HMI on a cell pack in the targets layout (cell schema 4, fixture targets_new): every Teach save, rename and
delete of each kind goes through the store's target API and reads back the same, the old point writers are never
called, missions name targets as ``move {to}`` (and the old skills' forms), and the pickers get the target groups."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import arm, held, points, robot, scene, targets

FIXTURES = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures'
ROBOT = 'bot_two_arm'
OLD_WRITERS = ('save_base_pose', 'save_stance', 'save_object_point', 'save_object_point_settings',
               'save_object_point_poses', 'rename_object_point', 'update_point', 'delete_point', 'set_named_pose',
               'set_joint_pose', 'save_held_pose')
MISSION = """goal: Use targets
steps:
- instruction: Wait
  skill: move
  args: {to: waiting}
- instruction: To the frame
  skill: move
  args: {to: bay_1_stack_1_front/pick}
- instruction: Grasp
  skill: move
  args: {to: bay_1_stack_1_front/rails_top, offset_m: 0.1, motion: {via: [frame_grasp_approach]}}
- instruction: Carry
  skill: move
  args: {to: carry}
- instruction: Lift
  skill: move
  args: {to: lift_high}
- instruction: A placeholder is not counted
  skill: move
  args: {to: '{frame}/pick'}
"""
OLD_STEPS = """goal: Old skills on the new cell, until the switch
steps:
- instruction: To the back row
  skill: navigate_to
  args: {target: 'stance:bay_1_stack_1_back'}
- instruction: To the knob
  skill: move_gripper
  args: {to: {point: karakuri_hanger/knob}}
- instruction: Home
  skill: move_joints
  args: {pose: home}
- instruction: Set it down
  skill: move_object
  args: {to: {pose: set_down_held}}
"""


@pytest.fixture
def store(tmp_path, monkeypatch):
    shutil.copytree(FIXTURES / 'targets_new', tmp_path / 'config')
    profile = tmp_path / 'config' / 'robots' / ROBOT / 'profile.yaml'
    text = profile.read_text().replace('closed: 1.0}', 'closed: 1.0, opening_m: [[0.0, 0.08], [1.0, 0.0]]}')
    profile.write_text(text + "ik: {groups: {left: left_arm, right: right_arm}, joints: '{tool}_arm_joint[1-7]'}\n")
    for name in OLD_WRITERS:  # a writer on a targets cell never uses the old point files
        monkeypatch.setattr(ConfigStore, name, lambda *_, _name=name, **__: pytest.fail(f'old writer {_name} called'))
    config = ConfigStore(tmp_path / 'config')
    assert config.targets_format()
    return config


def missions(store, *texts):
    for index, text in enumerate(texts):
        (store.root / 'missions' / f'm{index}.yaml').write_text(text)
    store.reload()


def fresh(store):
    return ConfigStore(store.root)


# ---- locations: cell base targets ---------------------------------------------------------------------------------
def test_a_location_is_created_moved_renamed_and_deleted_as_a_cell_base_target(store):
    points.save(store, ROBOT, 'location', ['dock'], {'x': 1.0, 'y': -2.0, 'yaw': 0.5, 'create': True})
    assert fresh(store).target('dock', ROBOT).data == {'moves': 'base', 'x': 1.0, 'y': -2.0, 'yaw': 0.5}
    with pytest.raises(points.PointError, match='already exists'):
        points.save(store, ROBOT, 'location', ['dock'], {'x': 0, 'y': 0, 'yaw': 0, 'create': True})
    file = points.save(store, ROBOT, 'location', ['waiting'], {'x': 2.5, 'y': 0.0, 'yaw': 0.0})
    text = (store.root / 'cell' / 'targets.yaml').read_text()
    assert file == 'cell/targets.yaml' and text.startswith('# targets in the cell frame')
    assert '# centre of the drop-off zone' in text
    assert fresh(store).target('waiting', ROBOT).pose.x == pytest.approx(2.5)
    points.save(store, ROBOT, 'location', ['dock'], {'x': 1.0, 'y': -2.0, 'yaw': 0.5, 'rename': 'charger'})
    assert 'charger' in fresh(store).locations() and 'dock' not in fresh(store).locations()
    points.delete(store, ROBOT, 'location', ['charger'])
    assert 'charger' not in fresh(store).locations()
    data = points.overview(fresh(store), ROBOT)
    assert data['files']['locations'] == 'cell/targets.yaml' and {loc['name'] for loc in data['locations']} == {
        'waiting', 'dropoff', 'point_a', 'set_down', 'spawn', 'taught_dock'}


def test_a_location_cannot_take_the_name_of_another_kind_of_target(store):
    with pytest.raises(points.PointError, match='moves joints'):
        points.save(store, ROBOT, 'location', ['home'], {'x': 0, 'y': 0, 'yaw': 0})


# ---- stances: base targets of a type for the robot ------------------------------------------------------------------
def test_a_stance_of_the_type_is_saved_created_renamed_and_deleted(store):
    file = points.save(store, ROBOT, 'stance', ['objects', 'frame_type2', 'pick'], {'x': -0.9, 'y': 0.0, 'yaw': 0.0})
    assert file == f'robots/{ROBOT}/objects/frame_type2.yaml'
    text = (store.root / file).read_text()
    assert text.startswith('handled_from: pick') and '# the robot\'s setting on the type\'s target' in text
    again = fresh(store)
    assert again.target('bay_1_stack_1_front/pick', ROBOT).pose.x == pytest.approx(6.0 - 0.9)
    assert again.target('bay_1_stack_1_back/pick', ROBOT).pose.x == pytest.approx(6.0 - 1.528)  # its override stays
    points.save(store, ROBOT, 'stance', ['objects', 'part_box', 'side'], {'x': 0.0, 'y': -0.7, 'yaw': 1.5708,
                                                                         'create': True})
    assert fresh(store).target('part_box_1/side', ROBOT).kind == 'base'
    with pytest.raises(points.PointError, match='already exists'):
        points.save(store, ROBOT, 'stance', ['objects', 'part_box', 'side'], {'x': 0, 'y': 0, 'yaw': 0, 'create': True})
    points.save(store, ROBOT, 'stance', ['objects', 'part_box', 'side'], {'x': 0.0, 'y': -0.7, 'yaw': 1.5708,
                                                                         'rename': 'left_side'})
    assert 'left_side' in fresh(store).object_target_names(ROBOT)['part_box']
    points.delete(store, ROBOT, 'stance', ['objects', 'part_box', 'left_side'])
    assert 'left_side' not in fresh(store).object_target_names(ROBOT)['part_box']


def test_an_instance_override_is_saved_and_deleted_in_the_instances_entry(store):
    points.save(store, ROBOT, 'stance', ['instances', 'bay_1_stack_1_back', 'pick'], {'x': -1.6, 'y': 1.0, 'yaw': 0.0})
    entry = targets.robot_layer(fresh(store), ROBOT, 'frame_type2', ['instances', 'bay_1_stack_1_back', 'pick'])
    assert entry == {'x': -1.6, 'y': 1.0, 'yaw': 0.0}   # settings on the type's pick: no moves of its own
    points.save(store, ROBOT, 'stance', ['instances', 'bay_1_stack_1_front', 'pick'], {'x': -1.1, 'y': 0, 'yaw': 0,
                                                                                      'create': True})
    data = points.overview(fresh(store), ROBOT)
    front = next(i for o in data['objects'] for i in o['instances'] if i['name'] == 'bay_1_stack_1_front')
    assert front['overrides'][0]['variant'] == 'pick' and front['overrides'][0]['fallback']
    assert front['overrides'][0]['offset']['x'] == pytest.approx(-1.1)
    with pytest.raises(points.PointError, match='overridden for bay_1_stack_1_back, bay_1_stack_1_front'):
        points.delete(store, ROBOT, 'stance', ['objects', 'frame_type2', 'pick'])
    with pytest.raises(points.PointError, match='has its name'):
        points.save(store, ROBOT, 'stance', ['instances', 'bay_1_stack_1_front', 'pick'],
                    {'x': -1.1, 'y': 0, 'yaw': 0, 'rename': 'pick_front'})
    points.delete(store, ROBOT, 'stance', ['instances', 'bay_1_stack_1_front', 'pick'])
    assert fresh(store).target('bay_1_stack_1_front/pick', ROBOT).pose.x == pytest.approx(5.0)


def test_a_stance_of_a_type_needs_a_placed_object_of_it(store):
    (store.root / 'objects' / 'pallet.yaml').write_text('name: pallet\n')
    with pytest.raises(points.PointError, match='no placed pallet'):
        points.save(store, ROBOT, 'stance', ['objects', 'pallet', 'front'], {'x': 0, 'y': 0, 'yaw': 0})


def test_a_stance_cannot_take_the_name_of_a_hand_target_of_the_type(store):
    with pytest.raises(points.PointError, match='moves hand'):
        points.save(store, ROBOT, 'stance', ['objects', 'karakuri_hanger', 'knob'], {'x': 0, 'y': 0, 'yaw': 0})


# ---- object points: hand targets of a type, the robot's settings in its objects file -----------------------------
def test_an_object_point_is_saved_with_the_robot_settings_in_the_robot_file(store):
    hand = {'xyz': [-0.05, 0.25, 0.08], 'rpy': [-1.9, 0.0, -1.5]}
    file = arm.save(store, ROBOT, 'frame_type2', 'rails_top',
                    {'hands': {'left': hand}, 'approach_m': 0.12, 'elbow': 'out', 'grip_m': {'left': 0.05}}, set())
    assert file == 'objects/frame_type2.yaml'
    again = fresh(store)
    own = targets.type_layer(again, 'frame_type2', 'rails_top')
    assert own['left_tool'] == hand and own['approach_m'] == 0.12 and 'elbow' not in own
    assert own['right_tool']['xyz'] == [-0.0563, -0.2513, 0.0714]          # the other hand is kept
    mine = targets.robot_layer(again, ROBOT, 'frame_type2', ['targets', 'rails_top'])
    assert mine == {'elbow': 'out', 'grip_m': {'left': 0.05}}
    other = targets.robot_layer(again, 'bot_plain', 'frame_type2', ['targets', 'rails_top'])
    assert other == {}                                                     # another robot's settings untouched
    point = next(p for p in points.object_points(again, 'frame_type2', {}, ROBOT) if p['name'] == 'rails_top')
    assert point['elbow'] == 'out' and point['grip_m'] == {'left': 0.05} and point['tools']['left'] == hand
    points.save_object_point(again, 'frame_type2', 'rails_top', {'elbow': None, 'grip_m': {'left': None}}, ROBOT)
    assert targets.robot_layer(fresh(store), ROBOT, 'frame_type2', ['targets', 'rails_top']) == {}
    assert 'rails_top:' not in (store.root / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml').read_text()
    text = (store.root / 'objects' / 'frame_type2.yaml').read_text()
    assert '# both hands from above on the long side rails' in text


def test_an_object_point_is_renamed_and_deleted_through_the_store(store):
    arm.save(store, ROBOT, 'frame_type2', 'rails_top_steep', {'approach_m': 0.1, 'rename': 'steep'}, set())
    names = fresh(store).object_target_names(ROBOT)['frame_type2']
    assert 'steep' in names and 'rails_top_steep' not in names
    arm.save(store, ROBOT, 'karakuri_hanger', 'knob', {'elbow': 'up', 'rename': 'latch'}, set())
    assert targets.robot_layer(fresh(store), ROBOT, 'karakuri_hanger', ['targets', 'latch']) == {'elbow': 'up'}
    arm.delete(store, ROBOT, 'frame_type2', 'steep', set())
    assert 'steep' not in fresh(store).object_target_names(ROBOT)['frame_type2']
    with pytest.raises(arm.ArmError, match='has no point'):
        arm.delete(store, ROBOT, 'frame_type2', 'hook_plate', set())   # an object target, not a point


# ---- robot poses: joint and hand targets in the robot's targets.yaml -------------------------------------------------
def test_a_joint_pose_is_written_as_joint_groups(store):
    file = robot.save_pose(store, ROBOT, 'lift_high', {'joints': {'body_joint': 0.65}}, set())
    assert Path(file) == Path(f'robots/{ROBOT}/targets.yaml')
    assert fresh(store).target('lift_high', ROBOT).data == {'moves': 'joints', 'joints': {'torso': 0.65}}
    robot.save_pose(store, ROBOT, 'look', {'joints': {'head_joint2': -0.3}, 'create': True}, set())
    assert fresh(store).target('look', ROBOT).joints == {'head': [None, -0.3]}   # the other head joint left alone
    assert fresh(store).named_pose(ROBOT, 'look')['joints'] == {'head_joint2': -0.3}
    with pytest.raises(robot.RobotSettingsError, match='already exists'):
        robot.save_pose(store, ROBOT, 'look', {'joints': {'head_joint2': 0.0}, 'create': True}, set())
    robot.save_pose(store, ROBOT, 'look', {'joints': {'head_joint2': -0.3}, 'rename': 'look_down'}, set())
    robot.delete_pose(store, ROBOT, 'look_down', set())
    assert 'look_down' not in fresh(store).target_names(ROBOT)['joint targets']
    with pytest.raises(robot.RobotSettingsError, match='no such joint'):
        robot.save_pose(store, ROBOT, 'odd', {'joints': {'wheel_joint': 1.0}}, set())


def test_a_tool_pose_is_a_hand_target_in_the_base_frame(store):
    hand = {'xyz': [0.5, 0.3, 0.6], 'rpy': [0.0, 1.5708, 0.0]}
    arm.save(store, ROBOT, 'robot', 'frame_grasp_approach',
             {'hands': {'right': hand}, 'joints': {'body_joint': 0.45, 'head_joint1': -0.3491, 'head_joint2': 0.0},
              'elbow': 'up'}, set())
    target = fresh(store).target('frame_grasp_approach', ROBOT)
    assert target.kind == 'hand' and target.data['right_tool'] == hand and 'left_tool' not in target.data
    assert target.joints == {'torso': 0.45, 'head': [-0.3491, 0.0]} and target.data['elbow'] == 'up'
    text = (store.root / 'robots' / ROBOT / 'targets.yaml').read_text()
    assert text.startswith('# base-frame and joint targets') and '# joint angles, not tool targets' in text
    by_name = {p['name']: p for p in robot.overview(fresh(store), ROBOT, set())['poses']}
    assert by_name['frame_grasp_approach']['tools']['right']['tcp'] == [0.5, 0.3, 0.6]
    assert by_name['frame_grasp_approach']['tools']['right']['approach'] == pytest.approx([1.0, 0.0, 0.0], abs=1e-4)
    positions = {p['name']: p for p in arm.overview(fresh(store), ROBOT, set())['positions'] if p['scope'] == 'robot'}
    assert positions['frame_grasp_approach']['hands']['right'] == hand


def test_a_capture_writes_the_current_joints_and_hands(store):
    joints = {'body_joint': 0.3, 'head_joint1': 0.1, 'head_joint2': 0.2, 'wheel': 5.0}
    arm.capture_pose(store, ROBOT, 'lift_low', {}, joints, None, set())
    assert fresh(store).target('lift_low', ROBOT).joints == {'torso': 0.3}
    tools = {'left': {'tcp': [0.4, 0.2, 0.5], 'approach': [1, 0, 0], 'closing': [0, 1, 0]},
             'right': {'tcp': [0.4, -0.2, 0.5], 'approach': [1, 0, 0], 'closing': [0, -1, 0]}}
    arm.capture_pose(store, ROBOT, 'frame_grasp_approach', {'tools': ['left']}, joints, tools, set())
    target = fresh(store).target('frame_grasp_approach', ROBOT)
    assert target.data['left_tool']['xyz'] == [0.4, 0.2, 0.5]
    assert target.data['right_tool']['xyz'] == [0.48, 0.295, 0.6]          # the other hand kept


def test_a_pose_cannot_take_the_name_of_another_kind_of_target(store):
    with pytest.raises(robot.RobotSettingsError, match='moves object'):
        robot.save_pose(store, ROBOT, 'carry', {'joints': {'body_joint': 0.3}}, set())
    with pytest.raises(robot.RobotSettingsError, match='moves base'):
        robot.save_pose(store, ROBOT, 'waiting', {'joints': {'body_joint': 0.3}}, set())


# ---- held poses: object targets in the base frame -------------------------------------------------------------------
def test_a_held_pose_is_an_object_target_saved_created_renamed_and_deleted(store):
    file = held.save(store, ROBOT, 'frame_type2', 'carry', {'joints': {'body_joint': 0.45}, 'path': [
        {'xyz': [1.0, 0, 0.45], 'rpy': [0, -0.8727, 0]}, {'xyz': [1.0, 0, 0.5], 'rpy': [0, -0.8727, 0]}]})
    assert file == f'robots/{ROBOT}/targets.yaml'
    target = fresh(store).target('carry', ROBOT)
    assert target.kind == 'object' and target.held_type == 'frame_type2' and len(target.poses) == 2
    assert target.joints == {'torso': 0.45}
    assert '# for driving: lift 0.5' in (store.root / file).read_text()
    held.save(store, ROBOT, 'part_box', 'box_up', {'path': [{'xyz': [0.8, 0, 0.3], 'rpy': [0, 0, 0]}]})
    assert fresh(store).target('box_up', ROBOT).held_type == 'part_box'
    held.save(store, ROBOT, 'part_box', 'box_up', {'path': [{'xyz': [0.8, 0, 0.3], 'rpy': [0, 0, 0]}],
                                                   'rename': 'box_high'})
    poses = {p['name']: p for p in held.overview(fresh(store), ROBOT)}
    assert 'box_high' in poses and 'box_up' not in poses and poses['box_high']['object_type'] == 'part_box'
    held.delete(store, ROBOT, 'part_box', 'box_high')
    assert 'box_high' not in fresh(store).target_names(ROBOT)['object targets']
    held.save(store, ROBOT, held.ANY_TYPE, 'lift_any', {'path': [{'xyz': [0.7, 0, 0.3], 'rpy': [0, 0, 0]}]})
    assert fresh(store).target('lift_any', ROBOT).held_type is None


def test_a_held_pose_keeps_its_name_unique_per_robot(store):
    with pytest.raises(held.HeldError, match='for frame_type2 objects'):
        held.save(store, ROBOT, 'part_box', 'carry', {'path': [{'xyz': [1, 0, 0], 'rpy': [0, 0, 0]}]})
    with pytest.raises(held.HeldError, match='moves joints'):
        held.save(store, ROBOT, 'frame_type2', 'home', {'path': [{'xyz': [1, 0, 0], 'rpy': [0, 0, 0]}]})
    with pytest.raises(held.HeldError, match='no held pose'):
        held.delete(store, ROBOT, 'frame_type2', 'nothing')


# ---- missions: move {to} and the old skills' forms ------------------------------------------------------------------
def test_missions_using_a_target_are_found_in_move_steps_and_old_steps(store):
    missions(store, MISSION, OLD_STEPS)
    used = points.usage(store, ROBOT)
    assert used[('location', 'waiting')] == ['m0']
    assert used[('objects', 'frame_type2', 'pick')] == ['m0']
    assert used[('instances', 'bay_1_stack_1_back', 'pick')] == ['m1']       # stance:<instance>: its first variant
    assert used[('object_point', 'frame_type2', 'rails_top')] == ['m0']
    assert used[('object_point', 'karakuri_hanger', 'knob')] == ['m1']
    poses = robot.pose_usage(store, set())
    assert poses['lift_high'] == ['m0'] and poses['frame_grasp_approach'] == ['m0'] and poses['home'] == ['m1']
    assert held.used_names(store)['carry'] == ['m0'] and held.used_names(store)['set_down_held'] == ['m1']
    assert not any('{' in key for key in targets.references(store))
    with pytest.raises(points.PointError, match='used by m0'):
        points.delete(store, ROBOT, 'location', ['waiting'])
    with pytest.raises(points.PointError, match='used by m0'):
        points.save(store, ROBOT, 'stance', ['objects', 'frame_type2', 'pick'], {'x': 0, 'y': 0, 'yaw': 0,
                                                                                'rename': 'grab'})
    with pytest.raises(arm.ArmError, match='used by m0'):
        arm.delete(store, ROBOT, 'frame_type2', 'rails_top', set())
    with pytest.raises(robot.RobotSettingsError, match='used by m1'):
        robot.delete_pose(store, ROBOT, 'home', set())
    with pytest.raises(held.HeldError, match='used by m0'):
        held.delete(store, ROBOT, 'frame_type2', 'carry')
    by_name = {p['name']: p for p in robot.overview(store, ROBOT, set())['poses']}
    assert by_name['lift_high']['used_by'] == ['m0']
    frame = next(o for o in points.overview(store, ROBOT)['objects'] if o['object'] == 'frame_type2')
    assert frame['variants'][0]['used_by'] == ['m0']
    assert next(p for p in frame['points'] if p['name'] == 'rails_top')['used_by'] == ['m0']


# ---- pickers and manual skills --------------------------------------------------------------------------------------
def test_the_pickers_have_the_target_groups_and_the_old_lists(store):
    names = points.picker_names(store, ROBOT)
    assert 'waiting' in names['base targets'] and 'karakuri_hanger/staging' in names['base targets']
    assert 'bay_2_stack_2_front/rails_top' in names['hand targets'] and 'frame_grasp_approach' in names['hand targets']
    assert {'carry', 'karakuri_hanger/hook'} <= set(names['object targets'])
    assert 'lift_high' in names['joint targets'] and 'part_box_1' in names['placed objects']
    assert 'waiting' in names['locations'] and 'carry' in names['held poses'] and names['via targets']


def test_the_old_layout_pickers_have_no_target_groups(tmp_path):
    shutil.copytree(FIXTURES / 'config', tmp_path / 'config')
    old = ConfigStore(tmp_path / 'config')
    names = points.picker_names(old, 'demo_bot')
    assert 'base targets' not in names and 'held poses' in names and 'via targets' in names
    assert names['locations'] == sorted(old.locations())


def test_manual_skills_run_move_with_the_reference_move_takes(store):
    assert targets.target_reference(store, ROBOT, 'stance:karakuri_hanger') == 'karakuri_hanger/staging'
    assert targets.target_reference(store, ROBOT, 'stance:karakuri_hanger/hang') == 'karakuri_hanger/hang'
    assert targets.target_reference(store, ROBOT, 'location:waiting') == 'waiting'
    assert targets.target_reference(store, ROBOT, 'waiting') == 'waiting'
    assert targets.move_args('bay_1_stack_1_front/rails_top', hands='left', offset_m=None) == {
        'to': 'bay_1_stack_1_front/rails_top', 'hands': 'left'}
    assert held.test_request('carry', True) == ('move', {'to': 'carry'})
    assert held.test_request('carry', False) == ('move_object', {'to': {'pose': 'carry'}})
    assert targets.serves_move(['wait', 'move', 'grip'], store) and not targets.serves_move(['navigate_to'], store)
    assert not targets.serves_move(None, store)                             # no catalog answer, no skills/move.yaml
    (store.root / 'skills' / 'move.yaml').write_text('implementations: []\n')
    assert targets.serves_move(None, store)


def test_the_hand_motion_settings_come_from_the_move_layout_when_the_cell_has_it(store):
    assert targets.approach_default(store) == 0.0
    (store.root / 'skills' / 'move').mkdir()
    (store.root / 'skills' / 'move' / 'hand.yaml').write_text('motion: {approach_m: 0.07}\ncell_frame: map\n')
    store.reload()
    assert targets.approach_default(store) == 0.07
    assert targets.module_config(store, 'move_gripper', 'hand')['cell_frame'] == 'map'


def test_the_3d_view_shows_the_cell_base_targets_and_the_hand_targets(store):
    data = scene.config_scene(store)
    assert {loc['name'] for loc in data['locations']} >= {'waiting', 'taught_dock'}
    shown = {(p['object_type'], p['name']) for p in data['points']}
    assert ('frame_type2', 'rails_top') in shown and ('karakuri_hanger', 'knob') in shown
    assert ('frame_type2', 'hook_plate') not in shown                       # an object target is not a hand point


def test_the_lists_read_merged_targets_and_name_a_bad_one_instead_of_failing(store):
    stances = store.root / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml'   # an override may give only some keys
    stances.write_text(stances.read_text().replace('bay_2_stack_2_back: {pick: {x: -1.528, y: 0.0, yaw: 0.0}}',
                                                   'bay_2_stack_2_back: {pick: {x: -1.6}}'))
    poses = store.root / 'robots' / ROBOT / 'targets.yaml'
    poses.write_text(poses.read_text() + 'bad: {moves: joints, joints: {head: [0.1]}}\n')
    again = fresh(store)
    frame = next(o for o in points.overview(again, ROBOT)['objects'] if o['object'] == 'frame_type2')
    back = next(i for i in frame['instances'] if i['name'] == 'bay_2_stack_2_back')
    assert back['overrides'][0]['offset'] == {'x': -1.6, 'y': 0.0, 'yaw': 0.0}
    by_name = {p['name']: p for p in robot.overview(again, ROBOT, set())['poses']}
    assert 'has 2 joints' in by_name['bad']['problems'][0] and by_name['home']['problems'] == []
