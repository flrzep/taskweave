import math

import pytest

from taskweave_core.config import ConfigStore
from taskweave_core.config.limits import joint_limits, joint_problems, pose_problems
from taskweave_core.validation import check_point

URDF = """<?xml version="1.0"?>
<robot name="demo_bot">
  <link name="base_link"/><link name="torso"/><link name="arm"/><link name="wheel"/>
  <joint name="lift" type="prismatic"><parent link="base_link"/><child link="torso"/>
    <limit lower="0" upper="0.5" effort="1" velocity="1"/></joint>
  <joint name="j1" type="revolute"><parent link="torso"/><child link="arm"/>
    <limit lower="-1.5" upper="1.5" effort="1" velocity="1"/></joint>
  <joint name="wheel" type="continuous"><parent link="base_link"/><child link="wheel"/></joint>
</robot>
"""
POSES = """home: {lift: 0.1, j1: 0.0}
too_high: {lift: 0.7, j1: 0.0}
borrowed: {leg_joint1: 0.3}
reach:
  joints: {lift: 0.6}
  tools:
    left: {tcp: [0.6, 0.2, 0.4], approach: [1, 0, 0], closing: [0, 1, 0]}
"""


@pytest.fixture
def store(config_dir):
    folder = config_dir / 'robots' / 'demo_bot'
    (folder / 'moveit').mkdir()
    (folder / 'moveit' / 'robot.urdf').write_text(URDF)
    (folder / 'poses.yaml').write_text(POSES)
    profile = folder / 'profile.yaml'
    profile.write_text(profile.read_text() + 'joints: {command: /cmd, state: /js, linear: [lift]}\n')
    return ConfigStore(config_dir)


def test_limits_come_from_the_urdf_and_the_profile_overrides(store):
    limits = joint_limits(store, 'demo_bot')
    assert limits['lift'] == (0.0, 0.5) and limits['j1'] == (-1.5, 1.5)
    assert limits['wheel'] == (-math.inf, math.inf)
    profile = store.root / 'robots' / 'demo_bot' / 'profile.yaml'
    profile.write_text(profile.read_text().replace('linear: [lift]}', 'linear: [lift], limits: {lift: [0.0, 0.4]}}'))
    assert joint_limits(ConfigStore(store.root), 'demo_bot')['lift'] == (0.0, 0.4)


def test_poses_outside_the_limits_or_with_foreign_joints_are_problems(store):
    assert pose_problems(store, 'demo_bot', 'home') == []
    assert pose_problems(store, 'demo_bot', 'too_high') == ['lift: 0.700 m is outside 0.000..0.500 m']
    assert pose_problems(store, 'demo_bot', 'borrowed') == ['leg_joint1: this robot has no such joint']
    assert pose_problems(store, 'demo_bot', 'reach') == ['lift: 0.600 m is outside 0.000..0.500 m']  # tool pose joints
    assert 'deg is outside' in joint_problems({'j1': 2.0}, joint_limits(store, 'demo_bot'))[0]


def test_missions_cannot_use_a_pose_outside_the_limits(store):
    assert check_point('joint_pose', 'home', store, 'demo_bot') is None
    assert 'does not fit robot' in check_point('joint_pose', 'too_high', store, 'demo_bot')


def test_a_robot_without_known_limits_is_not_checked(config_dir):
    store = ConfigStore(config_dir)
    (config_dir / 'robots' / 'demo_bot' / 'poses.yaml').write_text('anything: {x: 99.0}\n')
    assert joint_limits(store, 'demo_bot') == {} and pose_problems(store, 'demo_bot', 'anything') == []
