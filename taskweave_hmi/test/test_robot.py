import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import robot

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'
CATALOG = [{'name': 'move_joints', 'args_schema': {'properties': {'pose': {'x-taskweave-point': 'joint_pose_only'}}}},
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
    skill: move_joints
    args: {pose: carry}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    (tmp_path / 'config' / 'robots' / ROBOT / 'poses.yaml').write_text(POSES)
    (tmp_path / 'config' / 'missions' / 'carry.yaml').write_text(MISSION)
    return ConfigStore(tmp_path / 'config')


def test_grippers_lists_the_joint_grippers_with_their_opening_table():
    spec = {'left': {'joint': 'left_gripper_joint', 'open': 0.0, 'closed': 1.7, 'opening_m': [[0.0, 0.12], [1.7, 0.0]]},
            'right': {'joint': 'right_gripper_joint', 'closed': 1.7},
            'hand': {'action': '/hand/command', 'open': 1.0, 'closed': 0.0}}   # action grippers: no joint to show
    assert robot.grippers(spec) == {
        'left': {'joint': 'left_gripper_joint', 'open': 0.0, 'closed': 1.7, 'opening_m': [[0.0, 0.12], [1.7, 0.0]]},
        'right': {'joint': 'right_gripper_joint', 'open': 0.0, 'closed': 1.7, 'opening_m': None}}


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


def test_pose_args_fall_back_when_the_catalog_names_none():
    assert robot.pose_args([]) == robot.DEFAULT_POSE_ARGS == {('move_joints', 'pose'), ('move_gripper', 'to.pose')}


def test_pose_args_find_nested_pose_arguments_in_every_variant():
    catalog = [{'name': 'move_gripper', 'args_schema': {'properties': {'to': {'oneOf': [
        {'properties': {'point': {'x-taskweave-point': 'instance_point'}}},
        {'properties': {'pose': {'x-taskweave-point': 'tool_pose'}}}]}, 'hands': {'type': 'string'}}}},
        {'name': 'move_joints', 'args_schema': {'properties': {'pose': {'x-taskweave-point': 'joint_pose_only'}}}}]
    assert robot.pose_args(catalog) == {('move_gripper', 'to.pose'), ('move_joints', 'pose')}


def test_a_pose_a_nested_argument_names_counts_as_used(store):
    (store.root / 'missions' / 'reach.yaml').write_text(
        'goal: Reach\nsteps:\n- instruction: Reach out\n  skill: move_gripper\n  args: {to: {pose: reach}}\n')
    used = robot.pose_usage(store, robot.DEFAULT_POSE_ARGS)
    assert used['reach'] == ['reach'] and used['carry'] == ['carry']


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

