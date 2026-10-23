import math

import pytest

from taskweave_core.config import ConfigStore
from taskweave_core.config.limits import joint_limits, joint_problems, mimic_joints

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


@pytest.fixture
def store(config_dir):
    folder = config_dir / 'robots' / 'demo_bot'
    (folder / 'moveit').mkdir()
    (folder / 'moveit' / 'robot.urdf').write_text(URDF)
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


def test_mimic_joints_name_the_joint_they_follow(store, config_dir):
    urdf = config_dir / 'robots' / 'demo_bot' / 'moveit' / 'robot.urdf'
    assert mimic_joints(store, 'demo_bot') == {}
    urdf.write_text(URDF.replace('</robot>', '''  <link name="finger"/>
  <joint name="finger_joint" type="revolute"><parent link="arm"/><child link="finger"/>
    <limit lower="0" upper="1" effort="1" velocity="1"/><mimic joint="j1" multiplier="-1"/></joint>
</robot>'''))
    assert mimic_joints(store, 'demo_bot') == {'finger_joint': 'j1'}
    urdf.unlink()
    assert mimic_joints(ConfigStore(config_dir), 'demo_bot') == {}


def test_joints_outside_the_limits_or_foreign_are_problems(store):
    limits, linear = joint_limits(store, 'demo_bot'), {'lift'}
    assert joint_problems({'lift': 0.1, 'j1': 0.0}, limits, linear) == []
    assert joint_problems({'lift': 0.7}, limits, linear) == ['lift: 0.700 m is outside 0.000..0.500 m']
    assert joint_problems({'leg_joint1': 0.3}, limits, linear) == ['leg_joint1: this robot has no such joint']
    assert 'deg is outside' in joint_problems({'j1': 2.0}, limits)[0]       # (targets: test_targets.py pose_problems)


def test_a_robot_without_known_limits_is_not_checked(config_dir):
    store = ConfigStore(config_dir)
    assert joint_limits(store, 'demo_bot') == {} and joint_problems({'x': 99.0}, {}) == []


# ---- velocity limits (T2.6) --------------------------------------------------------------------------------------

def test_velocity_limits_come_from_joint_limits_yaml_then_the_urdf(store):
    from taskweave_core.config.limits import velocity_limits
    assert velocity_limits(store, 'demo_bot') == {'lift': 1.0, 'j1': 1.0}   # the URDF (wheel has no velocity)
    folder = store.root / 'robots' / 'demo_bot' / 'moveit'
    (folder / 'joint_limits.yaml').write_text(
        'joint_limits:\n'
        '  j1: {has_velocity_limits: true, max_velocity: 0.4}\n'
        '  lift: {has_velocity_limits: false, max_velocity: 9.0}\n'      # flag off: the URDF says
        '  wheel: {has_velocity_limits: true, max_velocity: 2.5}\n')
    assert velocity_limits(ConfigStore(store.root), 'demo_bot') == {'lift': 1.0, 'j1': 0.4, 'wheel': 2.5}


def test_no_velocity_limits_known_is_empty(config_dir):
    from taskweave_core.config.limits import velocity_limits
    assert velocity_limits(ConfigStore(config_dir), 'no_such_bot') == {}


def test_base_limits_and_capped_by(store):
    from taskweave_core.config.limits import base_limits, capped_by
    profile = store.root / 'robots' / 'demo_bot' / 'profile.yaml'
    assert base_limits(store, 'demo_bot') == {'linear': 1.0, 'angular': 1.5}
    profile.write_text(profile.read_text().replace('linear: 1.0,', 'linear: 0.8,'))
    assert base_limits(ConfigStore(store.root), 'demo_bot') == {'linear': 0.8, 'angular': 1.5}
    assert capped_by({'linear': 1.0, 'angular': 1.5, 'x': 3}, {'linear': 0.8, 'angular': 1.5}) == {'linear': 0.8}
