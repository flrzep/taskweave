"""A small cell with a neutral robot pack (``test_bot``) for the launch-description tests: profile with a base,
navigation.yaml, and a moveit/ folder, kept in a robot pack folder next to the cell as ``system.yaml`` lists it."""

import importlib.util
import shutil
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'taskweave_bringup' / 'launch'
CORE_LAUNCH = ROOT / 'taskweave_core' / 'launch' / 'taskweave.launch.py'
SHIPPED_PACKS = ROOT / 'taskweave_robots'

PROFILE = """\
name: test_bot
base: {type: differential, cmd_vel: /test/cmd_vel, odom: /test/odom, frame: test_base}
"""
EMBODIMENT = """\
name: test_bot
summary: A neutral test robot
capabilities: [base.differential]
"""
NAVIGATION = """\
envelope: {front: 0.5, back: 0.2, left: 0.3, right: 0.3}
padding: 0.05
inflation_radius: 0.5
driving: {max_forward: 1.0, max_reverse: 0.4, max_turn: 1.0, prefer_forward: 2.0}
"""
URDF = '<robot name="test_bot"><link name="test_base"><collision><geometry><mesh filename="meshes/hull.stl"/>' \
       '</geometry></collision></link></robot>\n'
SRDF = '<robot name="test_bot"/>\n'
KINEMATICS = 'arm: {kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin}\n'
JOINT_LIMITS = 'joint_limits:\n  test_joint: {has_velocity_limits: true, max_velocity: 1.5}\n'


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')
    return path


@pytest.fixture
def cell(tmp_path) -> Path:
    """A cell pack whose system.yaml names the pack folder ../packs; returns the cell (config) directory."""
    config = tmp_path / 'config'
    write(config / 'pack.yaml', 'name: test_cell\nversion: 1\nschema_version: 3\n')
    write(config / 'system.yaml', 'robot: test_bot\nrobot_packs: [../packs]\n')
    pack = tmp_path / 'packs' / 'test_bot'
    write(pack / 'pack.yaml', 'name: test_bot\nversion: 0.1.0\nschema_version: 1\ndescription: test\n')
    write(pack / 'profile.yaml', PROFILE)
    write(pack / 'embodiment.yaml', EMBODIMENT)
    write(pack / 'navigation.yaml', NAVIGATION)
    moveit = pack / 'moveit'
    write(moveit / 'robot.urdf', URDF)
    write(moveit / 'robot.srdf', SRDF)
    write(moveit / 'kinematics.yaml', KINEMATICS)
    write(moveit / 'joint_limits.yaml', JOINT_LIMITS)
    return config


@pytest.fixture
def no_moveit_cell(cell) -> Path:
    shutil.rmtree(cell.parent / 'packs' / 'test_bot' / 'moveit')
    return cell


def load_launch(name: str):
    """A launch file of the package as a module (they are not importable packages)."""
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
