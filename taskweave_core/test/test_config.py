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
    assert "'robot' is a required property" in str(error.value)


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
        store.save_base_pose('x', Pose2D(0, 0, 0), kind='stance')
    with pytest.raises(ValueError, match='invalid point name'):
        store.save_base_pose('bad name!', Pose2D(0, 0, 0))
    store.save_base_pose('shelf_b', Pose2D(4.0, 0.25, 1.57), kind='stance', robot='demo_bot')
    assert store.base_target('stance:shelf_b', 'demo_bot').x == 4.0


def test_wrap_angle():
    assert wrap_angle(3.5) == pytest.approx(3.5 - 2 * 3.141592653589793)
    assert wrap_angle(-0.1) == pytest.approx(-0.1)


def test_edits_by_other_processes_are_picked_up(config_dir):
    reader, writer = ConfigStore(config_dir), ConfigStore(config_dir)
    assert 'charger' not in reader.locations()
    writer.save_base_pose('charger', Pose2D(2.0, 1.0, 0.5))
    assert reader.base_target('charger', 'demo_bot') == Pose2D(2.0, 1.0, 0.5)
