import pytest

from taskweave_core.config import ConfigError, ConfigStore, Pose2D, wrap_angle


def test_mission_loads_and_validates(config_dir):
    mission = ConfigStore(config_dir).mission('patrol')
    assert mission['robot'] == 'demo_bot'
    assert [s['skill'] for s in mission['steps']] == ['navigate_to', 'wait', 'navigate_to']


def test_invalid_mission_names_file_and_key(config_dir):
    with pytest.raises(ConfigError) as error:
        ConfigStore(config_dir).mission('broken')
    assert 'broken.yaml' in str(error.value)
    assert "'explode' is not one of" in str(error.value) and 'steps.0.on_failure' in str(error.value)


def test_missing_file_is_reported(config_dir):
    with pytest.raises(ConfigError, match='file not found'):
        ConfigStore(config_dir).mission('does_not_exist')


def test_robot_roles(config_dir):
    robot = ConfigStore(config_dir).robot('demo_bot')
    assert robot.base['type'] == 'holonomic'
    assert robot.role('grippers.left')['action'] == '/left_gripper/command'
    with pytest.raises(KeyError, match='arms.left'):
        robot.role('arms.left')


def test_base_targets_resolve_by_prefix(config_dir):
    store = ConfigStore(config_dir)
    assert store.base_target('home', 'demo_bot') == Pose2D(0.0, 0.0, 0.0)
    assert store.base_target('location:dock', 'demo_bot').x == 1.5
    assert store.base_target('stance:shelf_a', 'demo_bot').yaw == pytest.approx(1.5708)
    with pytest.raises(KeyError, match='unknown location'):
        store.base_target('nowhere', 'demo_bot')
    with pytest.raises(KeyError, match='unknown stance'):
        store.base_target('stance:nowhere', 'demo_bot')


def test_object_points_are_object_relative(config_dir):
    point = ConfigStore(config_dir).object_point('box', 'top_grasp')
    assert point['left_tool'].frame == 'box'
    assert point['approach'] == {'axis': 'z', 'distance': 0.1}


def test_optional_files_default_empty(config_dir):
    store = ConfigStore(config_dir)
    assert store.skill_params('wait') == {}
    assert store.skill_params('navigate_to')['max_linear'] == 0.5


def test_teach_location_keeps_comments(config_dir):
    store = ConfigStore(config_dir)
    path = store.save_base_pose('charger', Pose2D(2.0, 1.0, 0.5))
    text = path.read_text()
    assert '# start position' in text
    assert 'charger: {frame: cell, x: 2.0, y: 1.0, yaw: 0.5}' in text
    assert store.base_target('charger', 'demo_bot') == Pose2D(2.0, 1.0, 0.5)


def test_teach_stance_needs_robot_and_valid_name(config_dir):
    store = ConfigStore(config_dir)
    with pytest.raises(ValueError):
        store.save_base_pose('shelf_a', Pose2D(0, 0, 0), kind='stance')
    with pytest.raises(ValueError, match='invalid point name'):
        store.save_base_pose('bad name!', Pose2D(0, 0, 0))
    with pytest.raises(ValueError, match="no object 'ghost'"):
        store.save_base_pose('ghost', Pose2D(0, 0, 0), kind='stance', robot='demo_bot')


def test_stance_is_object_pose_times_robot_offset(config_dir):
    store = ConfigStore(config_dir)
    front = store.base_target('stance:shelf_a', 'demo_bot')                                # default = first variant
    assert (front.x, front.y, front.yaw, front.frame) == pytest.approx((3.0, 0.25, 1.5708, 'cell'))
    side = store.base_target('stance:shelf_a/side', 'demo_bot')
    assert (side.x, side.y, side.yaw) == pytest.approx((2.2, 1.0, 0.0))
    assert store.base_target('stance:shelf_b', 'demo_bot').y == pytest.approx(0.1)        # instance override
    assert store.base_target('stance:shelf_b/side', 'demo_bot').x == pytest.approx(4.2)   # type variant still there
    with pytest.raises(KeyError, match="no 'top' stance for 'shelf'"):
        store.base_target('stance:shelf_a/top', 'demo_bot')
    with pytest.raises(KeyError, match="no stance for 'crate'"):
        store.base_target('stance:crate_1', 'demo_bot')
    assert store.stance_names('demo_bot') == ['stance:shelf_a', 'stance:shelf_a/side', 'stance:shelf_b',
                                              'stance:shelf_b/side']


def test_rotated_object_turns_the_stance_with_it(config_dir):
    store = ConfigStore(config_dir)
    crate = store.object_instances()['crate_1']
    assert crate.object == 'crate' and crate.z == pytest.approx(0.4)
    offset = Pose2D(-0.5, 0.0, 0.0, 'crate')                       # 0.5 m behind the crate along its x axis
    pose = crate.pose.compose(offset)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, -2.5, 1.5708), abs=1e-4)
    back = pose.relative_to(crate.pose)
    assert (back.x, back.y, back.yaw) == pytest.approx((-0.5, 0.0, 0.0), abs=1e-9)


def test_teach_stance_stores_the_offset_for_the_type_or_the_instance(config_dir):
    store = ConfigStore(config_dir)
    path, offset = store.save_stance('shelf_a/side', Pose2D(2.0, 1.1, 0.1), 'demo_bot')
    assert (offset.x, offset.y, offset.yaw, offset.frame) == pytest.approx((-1.0, 0.1, 0.1, 'shelf'))
    assert '# 0.75 m in front, facing the shelf' in path.read_text()            # comments kept
    assert store.base_target('stance:shelf_b/side', 'demo_bot').x == pytest.approx(4.0)   # all shelves follow
    store.save_base_pose('shelf_a', Pose2D(3.1, 0.3, 1.6), kind='stance_instance', robot='demo_bot')
    assert store.base_target('stance:shelf_a', 'demo_bot').x == pytest.approx(3.1)     # only shelf_a changed
    assert store.base_target('stance:shelf_b', 'demo_bot').y == pytest.approx(0.1)
    store.save_stance('crate_1', Pose2D(1.0, -2.5, 1.5708), 'demo_bot')               # first stance of a type
    assert store.stance_names('demo_bot')[-1] == 'stance:crate_1'


def test_update_rename_and_delete_points_keep_comments_and_order(config_dir):
    store = ConfigStore(config_dir)
    path = store.update_point('cell/locations.yaml', ['home'], Pose2D(0.5, 0.0, 0.0), rename='start')
    text = path.read_text()
    start = next(line for line in text.splitlines() if line.startswith('start:'))
    assert start.startswith('start: {frame: cell, x: 0.5, y: 0.0, yaw: 0.0}') and '# start position' in start
    assert text.index('start:') < text.index('dock:')                                  # place kept
    store.delete_point('cell/locations.yaml', ['dock'])
    assert list(store.locations()) == ['start']
    with pytest.raises(ValueError, match="no point 'dock'"):
        store.update_point('cell/locations.yaml', ['dock'], rename='docking')
    store.update_point('cell/locations.yaml', ['dock'], Pose2D(1.0, 1.0, 0.0))
    with pytest.raises(ValueError, match="'start' already exists"):
        store.update_point('cell/locations.yaml', ['dock'], rename='start')
    with pytest.raises(ValueError, match='invalid point name'):
        store.update_point('cell/locations.yaml', ['dock'], rename='2nd dock')


def test_deleting_the_last_override_removes_the_instance_entry(config_dir):
    store = ConfigStore(config_dir)
    path = store.delete_point('robots/demo_bot/stances.yaml', ['instances', 'shelf_b', 'front'])
    assert store.stance_offsets('demo_bot')['instances'] == {}
    assert 'instances: {}' in path.read_text() and '# 0.75 m in front' in path.read_text()
    assert store.base_target('stance:shelf_b', 'demo_bot').y == pytest.approx(0.25)    # falls back to the type
    store.update_point('robots/demo_bot/stances.yaml', ['instances', 'shelf_b', 'front'],
                       Pose2D(0, -1, 1.5708, 'shelf'))
    assert store.base_target('stance:shelf_b', 'demo_bot').y == pytest.approx(0.0)
    with pytest.raises(ValueError, match='not a point file'):
        store.update_point('missions/patrol.yaml', ['steps'], Pose2D(0, 0, 0))


def test_wrap_angle():
    assert wrap_angle(3.5) == pytest.approx(3.5 - 2 * 3.141592653589793)
    assert wrap_angle(-0.1) == pytest.approx(-0.1)


def test_edits_by_other_processes_are_picked_up(config_dir):
    reader, writer = ConfigStore(config_dir), ConfigStore(config_dir)
    assert 'charger' not in reader.locations()
    writer.save_base_pose('charger', Pose2D(2.0, 1.0, 0.5))
    assert reader.base_target('charger', 'demo_bot') == Pose2D(2.0, 1.0, 0.5)


def test_instance_point_resolves_a_placed_objects_point(config_dir):
    from taskweave_core.validation import check_point
    (config_dir / 'cell' / 'objects.yaml').write_text('box_1: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0}\n')
    store = ConfigStore(config_dir)
    instance, point = store.instance_point('box_1/top_grasp')
    assert instance.object == 'box' and point['left_tool'].xyz == (0.0, 0.1, 0.2)
    with pytest.raises(KeyError, match='box_9/top_grasp'):
        store.instance_point('box_9/top_grasp')
    assert check_point('object_point', 'box_1/nope', store, 'demo_bot') is not None


def test_object_instances_can_be_stacks(config_dir):
    (config_dir / 'cell' / 'objects.yaml').write_text(
        'pile: {object: box, frame: cell, x: 1.0, y: 2.0, z: 0.1, yaw: 0.0, stack: {count: 3, pitch: 0.05}}\n')
    pile = ConfigStore(config_dir).object_instances()['pile']
    assert pile.stack_count == 3 and pile.top_z == pytest.approx(0.2)


def test_teach_object_point_stores_tcps_relative_to_the_top_item(config_dir):
    import math

    from taskweave_core.config.poses import quaternion_from_rpy
    (config_dir / 'cell' / 'objects.yaml').write_text(
        'pile: {object: box, frame: cell, x: 1.0, y: 2.0, z: 0.1, yaw: 1.5708, stack: {count: 3, pitch: 0.05}}\n')
    store = ConfigStore(config_dir)
    down = quaternion_from_rpy((0.0, math.pi, 0.0))
    path, stored = store.save_object_point('pile/top_grasp', {'right': ((1.0, 2.5, 0.4), down)}, 'taught today')
    assert path.name == 'box.yaml' and stored['right'].xyz == pytest.approx((0.5, 0.0, 0.2), abs=1e-4)
    point = store.object_point('box', 'top_grasp')
    assert point['right_tool'].xyz == pytest.approx((0.5, 0.0, 0.2), abs=1e-4)
    assert point['left_tool'].xyz == (0.0, 0.1, 0.2) and point['grip_width_m'] == 0.05  # rest kept
    assert 'right_tool: {xyz: [0.5, 0.0, 0.2]' in path.read_text() and '# taught today' in path.read_text()
    store.save_object_point('pile/new_point', {'left': ((1.0, 2.0, 0.3), down)})
    assert store.object_point('box', 'new_point')['left_tool'].xyz == pytest.approx((0.0, 0.0, 0.1), abs=1e-4)
    with pytest.raises(ValueError, match='<instance>/<point>'):
        store.save_object_point('nowhere/top_grasp', {'left': ((0, 0, 0), down)})
