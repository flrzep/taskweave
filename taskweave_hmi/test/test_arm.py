import math
import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import arm, robot

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'
CATALOG = [{'name': 'move_joints', 'args_schema': {'properties': {'pose': {'x-taskweave-point': 'joint_pose_only'}}}}]
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
        assert all(math.isclose(a, b, abs_tol=1e-3) for a, b in zip(back[key], target[key], strict=True))
    tilted = {'tcp': [0.1, 0.0, 0.2], 'approach': [0, 0.7071, -0.7071], 'closing': [1, 0, 0]}
    again = arm.target_from_hand(arm.hand_from_target(tilted))
    assert all(math.isclose(a, b, abs_tol=1e-3) for a, b in zip(again['approach'], tilted['approach'], strict=True))


def test_a_capture_takes_the_closed_grippers_as_the_points_grip(store):
    profile = store.root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text().replace(
        'left: {action: /left_gripper/command,',
        'left: {joint: left_finger, opening_m: [[0.0, 0.08], [1.0, 0.0]], action: /left_gripper/command,'))
    store.reload()
    assert arm.grip_ranges(store, ROBOT) == {'left': [0.0, 0.08]}
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 0.62}) == {'left': 0.0304}   # opening, m
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 0.05}) == {}          # open: keep the old grip
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': -0.2}) == {}          # past open is open too
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 1.3}) == {'left': 0.0}   # within the table
    assert arm.closed_grips(store, ROBOT, ['right'], {'left_finger': 0.62}) == {}   # another hand was taught
    profile.write_text(profile.read_text().replace('left: {joint: left_finger,', 'left: {joint: left_finger, '
                                                   'closed_from: 0.7,'))
    store.reload()
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 0.62}) == {}          # the profile's closed_from: open
    assert arm.closed_grips(store, ROBOT, [], {'left_finger': 0.75}) != {}
