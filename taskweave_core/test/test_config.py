import pytest

from taskweave_core.config import ConfigError, ConfigStore, Pose2D, wrap_angle


def test_mission_loads_and_validates(config_dir):
    mission = ConfigStore(config_dir).mission('patrol')
    assert mission['robot'] == 'demo_bot'
    assert [s['skill'] for s in mission['steps']] == ['move', 'wait', 'move']


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
    assert point['approach_m'] == 0.1


def test_optional_files_default_empty(config_dir):
    store = ConfigStore(config_dir)
    assert store.skill_params('wait') == {}
    assert store.skill_params('move.base')['max_linear'] == 0.5


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


def test_rotated_object_turns_the_stance_with_it(config_dir):
    store = ConfigStore(config_dir)
    crate = store.object_instances()['crate_1']
    assert crate.object == 'crate' and crate.z == pytest.approx(0.4)
    offset = Pose2D(-0.5, 0.0, 0.0, 'crate')                       # 0.5 m behind the crate along its x axis
    pose = crate.pose.compose(offset)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, -2.5, 1.5708), abs=1e-4)
    back = pose.relative_to(crate.pose)
    assert (back.x, back.y, back.yaw) == pytest.approx((-0.5, 0.0, 0.0), abs=1e-9)


def test_wrap_angle():
    assert wrap_angle(3.5) == pytest.approx(3.5 - 2 * 3.141592653589793)
    assert wrap_angle(-0.1) == pytest.approx(-0.1)


def test_edits_by_other_processes_are_picked_up(config_dir):
    reader, writer = ConfigStore(config_dir), ConfigStore(config_dir)
    assert 'charger' not in reader.locations()
    writer.save_target('charger', {'moves': 'base', 'x': 2.0, 'y': 1.0, 'yaw': 0.5}, 'demo_bot')
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


def test_object_type_may_carry_simulator_keys(config_dir):
    path = config_dir / 'objects' / 'crate.yaml'
    path.write_text('name: crate\nsim: {box: [0.06, 0.06, 0.1], on_hold: glue}\n')
    assert ConfigStore(config_dir).object('crate')['sim']['box'] == [0.06, 0.06, 0.1]
    path.write_text('name: crate\nsim: 3\n')
    with pytest.raises(ConfigError, match='sim'):
        ConfigStore(config_dir).object('crate')


def test_object_type_collision_shapes(config_dir):
    path = config_dir / 'objects' / 'crate.yaml'
    for text in ('{box: [0.06, 0.06, 0.1], xyz: [0, 0, 0.05], rpy: [0, 0, 1.57]}',
                 '{cylinder: {radius: 0.03, height: 0.2}}',
                 '[{box: [0.1, 0.1, 0.1]}, {cylinder: {radius: 0.03, height: 0.2}, xyz: [0, 0, 0.1]}]'):
        path.write_text('name: crate\ncollision: ' + text + '\n')
        assert ConfigStore(config_dir).object('crate')['collision']
    for bad in ('{}', '{box: [1, 2]}', '{box: [0, 1, 1]}', '{box: [1, 1, 1], cylinder: {radius: 1, height: 1}}',
                '{cylinder: {radius: 1}}', '{sphere: 1}', '{box: [1, 1, 1], xyz: [0, 0]}', '[]', '3'):
        path.write_text('name: crate\ncollision: ' + bad + '\n')
        with pytest.raises(ConfigError, match='collision'):
            ConfigStore(config_dir).object('crate')


def turned_shelf(config_dir):
    """shelf_c lies turned 180 deg; shelves are declared 180 deg symmetric (e.g. a frame in a front row)."""
    (config_dir / 'objects' / 'shelf.yaml').write_text(
        'name: shelf\nyaw_symmetry_deg: 180\n'
        'targets:\n  grip:\n    moves: hand\n    left_tool: {xyz: [0.0, 0.2, 0.5], rpy: [0.0, 0.0, 0.0]}\n')
    with open(config_dir / 'cell' / 'objects.yaml', 'a') as f:
        f.write('shelf_c: {object: shelf, frame: cell, x: 7.0, y: 1.0, yaw: 3.14159}\n')
    return ConfigStore(config_dir)


def test_symmetric_object_is_approached_from_the_robots_side(config_dir):
    store = turned_shelf(config_dir)
    as_placed = store.base_target('stance:shelf_c', 'demo_bot')        # no robot pose: the offset as placed
    assert as_placed.y == pytest.approx(1.75)                          # behind the turned shelf
    robot = Pose2D(7.0, -1.0, 0.0)                                     # in the aisle at y < 1
    stance = store.base_target('stance:shelf_c', 'demo_bot', near=robot)
    assert (stance.x, stance.y) == pytest.approx((7.0, 0.25))          # in front, like shelf_a
    assert stance.yaw == pytest.approx(1.5708, abs=1e-4)
    # an object type without symmetry (here: without an object file) stays as placed
    crate = store.object_instances()['crate_1']
    assert store.handling_instance(crate, 'demo_bot', robot) == crate


def test_symmetric_object_point_uses_the_robots_side(config_dir):
    store = turned_shelf(config_dir)
    robot = Pose2D(7.0, 0.25, 1.5708)
    instance, _ = store.instance_point('shelf_c/grip', 'demo_bot', robot)
    assert instance.pose.yaw == pytest.approx(0.0, abs=1e-4)           # handled like an unturned shelf
    assert store.instance_point('shelf_c/grip')[0].pose.yaw == pytest.approx(3.14159)
