import math
import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import arm, robot

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'
CATALOG = [{'name': 'move_to_pose', 'args_schema': {'properties': {'pose': {'x-taskweave-point': 'joint_pose'}}}}]
ARGS = robot.pose_args(CATALOG)
POSES = """home: {lift: 0.0, j1: 0.0, j2: 0.0}
reach:
  joints: {lift: 0.3}
  tools:
    left: {tcp: [0.6, 0.2, 0.4], approach: [1, 0, 0], closing: [0, 1, 0]}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    (tmp_path / 'config' / 'robots' / ROBOT / 'poses.yaml').write_text(POSES)
    profile = tmp_path / 'config' / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text() + "ik: {groups: {left: left_arm}, joints: '{tool}_joint[1-7]'}\n")
    return ConfigStore(tmp_path / 'config')


def test_hand_numbers_convert_both_ways():
    target = {'tcp': [0.6, 0.2, 0.4], 'approach': [1, 0, 0], 'closing': [0, 1, 0]}
    hand = arm.hand_from_target(target)
    assert hand['xyz'] == [0.6, 0.2, 0.4] and math.isclose(hand['rpy'][1], math.pi / 2, abs_tol=1e-3)
    back = arm.target_from_hand(hand)
    for key in ('tcp', 'approach', 'closing'):
        assert all(math.isclose(a, b, abs_tol=1e-3) for a, b in zip(back[key], target[key]))
    tilted = {'tcp': [0.1, 0.0, 0.2], 'approach': [0, 0.7071, -0.7071], 'closing': [1, 0, 0]}
    again = arm.target_from_hand(arm.hand_from_target(tilted))
    assert all(math.isclose(a, b, abs_tol=1e-3) for a, b in zip(again['approach'], tilted['approach']))


def test_overview_lists_robot_poses_and_object_points_alike(store):
    data = arm.overview(store, ROBOT, ARGS)
    by_key = {(p['scope'], p['name']): p for p in data['positions']}
    assert by_key[('robot', 'home')]['hands'] == {} and by_key[('robot', 'home')]['joints']['lift'] == 0.0
    assert by_key[('robot', 'reach')]['hands']['left']['xyz'] == [0.6, 0.2, 0.4]
    assert by_key[('box', 'top_grasp')]['hands']['left']['xyz'] == [0.0, 0.1, 0.2]
    assert 'box' in data['object_types']


def test_saving_numbers_writes_each_kind_to_its_file(store):
    hand = {'xyz': [0.5, 0.2, 0.4], 'rpy': [0.0, math.pi / 2, 0.0]}
    arm.save(store, ROBOT, 'robot', 'reach', {'hands': {'left': hand}, 'joints': {'lift': 0.25}, 'elbow': 'down'}, ARGS)
    pose = store.named_pose(ROBOT, 'reach')
    assert pose['tools']['left']['tcp'] == [0.5, 0.2, 0.4] and pose['joints'] == {'lift': 0.25} and pose['elbow'] == 'down'
    arm.save(store, ROBOT, 'box', 'top_grasp', {'hands': {'left': hand}, 'approach_m': 0.12, 'elbow': None}, ARGS)
    point = store.object_point('box', 'top_grasp')
    assert list(point['left_tool'].xyz) == [0.5, 0.2, 0.4] and point['approach_m'] == 0.12
    with pytest.raises(arm.ArmError, match='three numbers'):
        arm.save(store, ROBOT, 'box', 'top_grasp', {'hands': {'left': {'xyz': [0, 0], 'rpy': [0, 0, 0]}}}, ARGS)


def test_object_points_rename_and_delete(store):
    hand = {'xyz': [0.0, 0.1, 0.2], 'rpy': [0.0, 1.5708, 0.0]}
    arm.save(store, ROBOT, 'box', 'wide_grasp', {'hands': {'left': hand}, 'rename': 'wide'}, ARGS)
    assert 'wide' in store.object('box')['points'] and 'wide_grasp' not in store.object('box')['points']
    arm.delete(store, ROBOT, 'box', 'wide', ARGS)
    assert 'wide' not in store.object('box')['points']
    with pytest.raises(arm.ArmError, match='no object type'):
        arm.delete(store, ROBOT, 'crate', 'x', ARGS)


def test_capture_keeps_the_form_and_joint_set(store):
    joints = {'lift': 0.4, 'j1': 0.3, 'j2': -0.2, 'wheel': 5.0}
    arm.capture_pose(store, ROBOT, 'home', {}, joints, None, ARGS)
    assert store.joint_pose(ROBOT, 'home') == {'lift': 0.4, 'j1': 0.3, 'j2': -0.2}
    arm.capture_pose(store, ROBOT, 'park', {'form': 'joints'}, joints, None, ARGS)    # new: the joint poses' joints
    assert store.joint_pose(ROBOT, 'park') == {'lift': 0.4, 'j1': 0.3, 'j2': -0.2}
    with pytest.raises(arm.ArmError, match='forward kinematics'):
        arm.capture_pose(store, ROBOT, 'reach', {}, joints, None, ARGS)
    targets = {'left': {'tcp': [0.55, 0.2, 0.4], 'approach': [1, 0, 0], 'closing': [0, 1, 0]}}
    arm.capture_pose(store, ROBOT, 'reach', {}, joints, targets, ARGS)
    assert store.named_pose(ROBOT, 'reach')['tools']['left']['tcp'] == [0.55, 0.2, 0.4]


def test_a_capture_takes_the_closed_grippers_as_the_points_grip(store):
    profile = store.root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text().replace('left: {action: /left_gripper/command,',
                                                   'left: {joint: left_finger, action: /left_gripper/command,'))
    store.reload()
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 0.62}) == {'left': 0.62}
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 0.05}) == {}          # open: keep the old grip
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': -0.2}) == {}          # past open is open too
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 1.3}) == {'left': 1.0}   # within open..closed
    assert arm.closed_grips(store, ROBOT, ['right'], {'left_finger': 0.62}) == {}   # another hand was taught
