import math
import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import arm, targets

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'
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
    hand = targets.hand_from_axes(target)
    assert hand['xyz'] == [0.6, 0.2, 0.4] and math.isclose(hand['rpy'][1], math.pi / 2, abs_tol=1e-3)
    back = arm.target_from_hand(hand)
    for key in ('tcp', 'approach', 'closing'):
        assert all(math.isclose(a, b, abs_tol=1e-3) for a, b in zip(back[key], target[key], strict=True))
    tilted = {'tcp': [0.1, 0.0, 0.2], 'approach': [0, 0.7071, -0.7071], 'closing': [1, 0, 0]}
    again = arm.target_from_hand(targets.hand_from_axes(tilted))
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


def test_point_targets_at_an_object_in_the_base_frame():
    # the robot at (1, 0) turned 90° (facing +y), the object at (1, 2), top 0.8 m up, turned 90° too: the object
    # sits 2 m ahead of the robot, its frame lined up with the base's
    base = {'xyz': [1.0, 0.0, 0.0], 'quat': [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]}
    instance = {'x': 1.0, 'y': 2.0, 'top_z': 0.8, 'yaw': math.pi / 2}
    # a point 0.1 m in front of the object's origin (its -x), approach along the object's +x
    found = arm.point_targets(base, instance, {'left': {'xyz': [-0.1, 0.0, 0.05], 'rpy': [0.0, math.pi / 2, 0.0]}})
    assert found['left']['tcp'] == pytest.approx([1.9, 0.0, 0.85], abs=1e-4)
    assert found['left']['approach'] == pytest.approx([1.0, 0.0, 0.0], abs=1e-4)
    assert found['left']['closing'] == pytest.approx([0.0, 1.0, 0.0], abs=1e-4)


def test_hand_distances_from_the_object():
    base = {'xyz': [0.0, 0.0, 0.0], 'quat': [1.0, 0.0, 0.0, 0.0]}
    instance = {'x': 1.0, 'y': 0.0, 'top_z': 0.5, 'yaw': 0.0}
    current = {'left': {'tcp': [1.0, 0.0, 0.6]}, 'right': {'tcp': [0.0, 0.0, 0.5]}}
    assert arm.hand_distances(base, instance, current) == {'left': 0.1, 'right': 1.0}


FIXTURES = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures'


@pytest.fixture
def cell(tmp_path):
    shutil.copytree(FIXTURES / 'targets_new', tmp_path / 'cell_config')
    return ConfigStore(tmp_path / 'cell_config')


def test_reach_targets_turn_a_symmetric_object_to_the_robots_side(cell):
    from taskweave_core.config import Pose2D

    # bay_2_stack_2_front: a 180 deg symmetric frame stack placed turned (yaw pi) at (8, 1); bot_two_arm's pick
    # stance is 1 m behind the object's -x: as placed at (9, 1), turned at (7, 1)
    base = {'xyz': [7.0, 1.0, 0.0], 'quat': [1.0, 0.0, 0.0, 0.0]}
    def reach(near):
        return arm.reach_targets(cell, 'bot_two_arm', 'bay_2_stack_2_front', near, base)

    near_side, near_where = reach(Pose2D(7.0, 1.0, 0.0))
    far_side, far_where = reach(Pose2D(9.0, 1.0, math.pi))
    assert set(near_side) == {'rails_top', 'rails_top_steep'}        # the hand points (hook_plate is an object one)
    assert math.isclose(math.cos(near_where['yaw']), 1.0, abs_tol=1e-3)   # handled from x = 7: turned to yaw 0
    assert math.isclose(math.cos(far_where['yaw']), -1.0, abs_tol=1e-3)   # from x = 9: as placed
    instance = cell.object_instances()['bay_2_stack_2_front']
    assert near_where['top_z'] == pytest.approx(instance.top_z)             # the stack's top item
    # the same base, the object turned: the left hand's point lands on the other rail
    assert near_side['rails_top']['left']['tcp'][1] == pytest.approx(0.2513, abs=1e-3)
    assert far_side['rails_top']['left']['tcp'][1] == pytest.approx(-0.2513, abs=1e-3)
    only, _ = arm.reach_targets(cell, 'bot_two_arm', 'bay_2_stack_2_front', None, base, ['rails_top'])
    assert set(only) == {'rails_top'}
    with pytest.raises(arm.ArmError, match='no placed object'):
        arm.reach_targets(cell, 'bot_two_arm', 'nowhere', None, base)


def test_reach_targets_report_a_broken_point_and_check_the_rest(cell, monkeypatch):
    from taskweave_core.config import TargetError

    base = {'xyz': [7.0, 1.0, 0.0], 'quat': [1.0, 0.0, 0.0, 0.0]}
    real = cell.target

    def target(reference, robot, near=None, **kw):
        if reference.endswith('/rails_top_steep'):
            raise TargetError("'bay_2_stack_2_front/rails_top_steep': left_tool needs xyz")
        return real(reference, robot, near, **kw)

    monkeypatch.setattr(cell, 'target', target)
    found, _ = arm.reach_targets(cell, 'bot_two_arm', 'bay_2_stack_2_front', None, base)
    assert found['rails_top_steep'] == {'error': "'bay_2_stack_2_front/rails_top_steep': left_tool needs xyz"}
    assert set(found['rails_top']) == {'left', 'right'}


def test_near_hands_take_both_hands_of_a_frame_grasp(cell):
    # both hands on bay_2_stack_2_front's rails (rails_top, 0.25 m out to each side of the origin, from x = 7)
    from taskweave_core.config import Pose2D

    base = {'xyz': [7.0, 1.0, 0.0], 'quat': [1.0, 0.0, 0.0, 0.0]}
    found, where = arm.reach_targets(cell, 'bot_two_arm', 'bay_2_stack_2_front', Pose2D(7.0, 1.0, 0.0), base)
    distances = arm.hand_distances(base, where, found['rails_top'])
    assert all(0.25 < m < 0.3 for m in distances.values())
    assert sorted(arm.near_hands(distances)) == ['left', 'right']
    # one hand there, the other far: that one; none near: the nearest; no hands: none
    assert arm.near_hands({'left': distances['left'], 'right': 0.9}) == ['left']
    assert arm.near_hands({'left': 1.2, 'right': 0.9}) == ['right']
    assert arm.near_hands({}) == []
