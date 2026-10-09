import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import robot

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'
CATALOG = [{'name': 'move_to_pose', 'args_schema': {'properties': {'pose': {'x-taskweave-point': 'joint_pose'}}}},
           {'name': 'wait', 'args_schema': {'properties': {'seconds': {'type': 'number'}}}}]
POSES = """# Named joint poses of the demo bot
home: {lift: 0.0, j1: 0.0, j2: 0.0}   # IK error 0 mm
carry: {lift: 0.2, j1: 1.0, j2: -0.5}
"""
MISSION = """goal: Carry something
steps:
  - instruction: Take the carry pose
    skill: move_to_pose
    args: {pose: carry}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    (tmp_path / 'config' / 'robots' / ROBOT / 'poses.yaml').write_text(POSES)
    (tmp_path / 'config' / 'missions' / 'carry.yaml').write_text(MISSION)
    return ConfigStore(tmp_path / 'config')


def test_overview_lists_poses_with_their_missions(store):
    data = robot.overview(store, ROBOT, robot.pose_args(CATALOG))
    poses = {p['name']: p for p in data['poses']}
    assert poses['carry']['used_by'] == ['carry'] and poses['home']['used_by'] == []
    assert data['navigation'] == {} and data['files']['poses'] == f'robots/{ROBOT}/poses.yaml'


def test_saving_a_pose_marks_it_edited_and_keeps_the_file_readable(store):
    args = robot.pose_args(CATALOG)
    robot.save_pose(store, ROBOT, 'home', {'joints': {'lift': 0.1, 'j1': 0.5, 'j2': 0.0}}, args)
    text = (store.root / 'robots' / ROBOT / 'poses.yaml').read_text()
    assert text.startswith('# Named joint poses of the demo bot')
    home = next(line for line in text.splitlines() if line.startswith('home:'))
    assert 'j1: 0.5' in home and '# edited in the HMI' in home and 'IK error' not in home
    robot.save_pose(store, ROBOT, 'high', {'joints': {'lift': 0.6}, 'create': True}, args)      # partial pose
    assert store.joint_pose(ROBOT, 'high') == {'lift': 0.6}
    with pytest.raises(robot.RobotSettingsError, match='already exists'):
        robot.save_pose(store, ROBOT, 'high', {'joints': {'lift': 0.5}, 'create': True}, args)
    with pytest.raises(robot.RobotSettingsError, match='numbers'):
        robot.save_pose(store, ROBOT, 'high', {'joints': {'lift': 'up'}}, args)


def test_used_poses_cannot_be_renamed_or_deleted(store):
    args = robot.pose_args(CATALOG)
    with pytest.raises(robot.RobotSettingsError, match='used by carry'):
        robot.delete_pose(store, ROBOT, 'carry', args)
    with pytest.raises(robot.RobotSettingsError, match='used by carry'):
        robot.save_pose(store, ROBOT, 'carry', {'joints': {'lift': 0.2}, 'rename': 'hold'}, args)
    robot.save_pose(store, ROBOT, 'home', {'joints': {'lift': 0.0}, 'rename': 'rest'}, args)
    robot.delete_pose(store, ROBOT, 'rest', args)
    assert list(store.joint_poses(ROBOT)) == ['carry']


def test_navigation_is_checked_saved_and_turned_into_a_footprint(store):
    settings = {'envelope': {'front': 0.95, 'back': 0.3, 'left': 0.45, 'right': 0.4}, 'padding': 0.05,
                'inflation_radius': 0.65}
    robot.save_navigation(store, ROBOT, settings)
    assert store.navigation(ROBOT) == settings
    assert robot.footprint(settings) == [[0.95, 0.45], [0.95, -0.4], [-0.3, -0.4], [-0.3, 0.45]]
    with pytest.raises(robot.RobotSettingsError, match='envelope.front'):
        robot.save_navigation(store, ROBOT, {'envelope': {**settings['envelope'], 'front': 0}})
    assert store.navigation(ROBOT) == settings                                       # nothing written


def test_pose_args_fall_back_when_the_catalog_names_none():
    assert robot.pose_args([]) == robot.DEFAULT_POSE_ARGS
