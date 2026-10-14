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
reach:
  joints: {lift: 0.3}
  tools:
    left: {tcp: [0.6, 0.2, 0.4], approach: [1, 0, 0], closing: [0, 1, 0]}
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
    assert list(store.joint_poses(ROBOT)) == ['carry', 'reach']


def test_navigation_is_checked_and_saved(store):
    settings = {'envelope': {'front': 0.95, 'back': 0.3, 'left': 0.45, 'right': 0.4}, 'padding': 0.05,
                'inflation_radius': 0.65,
                'driving': {'max_forward': 0.5, 'max_reverse': 0.0, 'max_turn': 1.0, 'sideways': True,
                            'max_sideways': 0.2, 'prefer_forward': 5.0}}
    robot.save_navigation(store, ROBOT, settings)                                     # demo_bot is holonomic
    assert store.navigation(ROBOT) == settings
    assert 'sideways: true' in (store.root / 'robots' / ROBOT / 'navigation.yaml').read_text()   # stays a boolean
    with pytest.raises(robot.RobotSettingsError, match='envelope.front'):
        robot.save_navigation(store, ROBOT, {'envelope': {**settings['envelope'], 'front': 0}})
    with pytest.raises(robot.RobotSettingsError, match='above the base limit'):
        robot.save_navigation(store, ROBOT, {**settings, 'driving': {'max_forward': 9.0}})
    assert store.navigation(ROBOT) == settings                                       # nothing written


def test_a_base_that_is_not_holonomic_cannot_drive_sideways():
    settings = {'envelope': {'front': 0.5, 'back': 0.5, 'left': 0.3, 'right': 0.3}, 'driving': {'sideways': True}}
    with pytest.raises(robot.RobotSettingsError, match="'differential' and cannot drive sideways"):
        robot.check_navigation(settings, {'type': 'differential'})
    assert robot.check_navigation({**settings, 'driving': {'sideways': False}}, {'type': 'differential'})


def test_tool_poses_are_listed_and_saved(store):
    args = robot.pose_args(CATALOG)
    poses = {p['name']: p for p in robot.overview(store, ROBOT, args)['poses']}
    assert poses['reach']['joints'] == {'lift': 0.3} and poses['reach']['tools']['left']['tcp'] == [0.6, 0.2, 0.4]
    assert poses['home']['tools'] == {}
    target = {'tcp': [0.7, 0.1, 0.5], 'approach': [0, 0, -1], 'closing': [1, 0, 0]}
    with pytest.raises(robot.RobotSettingsError, match='has no IK'):                 # demo_bot has no ik section
        robot.save_pose(store, ROBOT, 'reach', {'joints': {'lift': 0.3}, 'tools': {'left': target}}, args)
    profile = store.root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text() + "ik: {groups: {left: left_arm}, joints: '{tool}_joint[1-7]'}\n")
    robot.save_pose(store, ROBOT, 'reach', {'joints': {'lift': 0.3}, 'tools': {'left': target}}, args)
    assert store.named_pose(ROBOT, 'reach')['tools']['left'] == target
    with pytest.raises(robot.RobotSettingsError, match='parallel'):
        robot.save_pose(store, ROBOT, 'reach', {'tools': {'left': {**target, 'closing': [0, 0, 2]}}}, args)


def test_poses_outside_the_joint_limits_are_refused_and_flagged(store):
    args = robot.pose_args(CATALOG)
    profile = store.root / 'robots' / ROBOT / 'profile.yaml'
    limits = '{lift: [0.0, 0.5], j1: [-1, 1], j2: [-1, 1]}'
    profile.write_text(profile.read_text() + f'joints: {{command: /c, state: /s, linear: [lift], limits: {limits}}}\n')
    with pytest.raises(robot.RobotSettingsError, match='lift: 0.800 m is outside 0.000..0.500 m'):
        robot.save_pose(store, ROBOT, 'home', {'joints': {'lift': 0.8, 'j1': 0.0, 'j2': 0.0}}, args)
    with pytest.raises(robot.RobotSettingsError, match='no such joint'):
        robot.save_pose(store, ROBOT, 'new', {'joints': {'leg_joint1': 0.1}, 'create': True}, args)
    data = robot.overview(store, ROBOT, args)
    assert data['joint_limits']['lift'] == [0.0, 0.5]
    poses = {p['name']: p for p in data['poses']}
    assert poses['home']['problems'] == [] and poses['carry']['problems'] == []


def test_pose_args_fall_back_when_the_catalog_names_none():
    assert robot.pose_args([]) == robot.DEFAULT_POSE_ARGS


def test_the_ik_choice_is_shown_with_defaults_and_saved(store):
    assert robot.overview(store, ROBOT, robot.DEFAULT_POSE_ARGS)['ik_choice'] is None   # demo_bot has no ik
    profile = store.root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text() + "ik:\n  groups: {left: left_arm}\n  joints: 'left_j[1-7]'\n")
    choice = robot.overview(store, ROBOT, robot.DEFAULT_POSE_ARGS)['ik_choice']
    assert not choice['enabled'] and choice['elbow'] is None and choice['weights']['limits'] == 2.0
    robot.save_ik_choice(store, ROBOT, {'elbow': 'down', 'limit_margin_deg': 30, 'weights': {'motion': 0.5}})
    choice = robot.overview(store, ROBOT, robot.DEFAULT_POSE_ARGS)['ik_choice']
    assert choice['enabled'] and choice['elbow'] == 'down' and choice['limit_margin_deg'] == 30
    assert choice['weights'] == {'limits': 2.0, 'elbow': 1.0, 'motion': 0.5, 'symmetry': 0.5}
    with pytest.raises(robot.RobotSettingsError, match='elbow'):
        robot.save_ik_choice(store, ROBOT, {'elbow': 'sideways'})
    with pytest.raises(robot.RobotSettingsError, match='outside'):
        robot.save_ik_choice(store, ROBOT, {'limit_margin_deg': 120})
