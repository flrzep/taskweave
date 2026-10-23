import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import held

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'
HELD = """# where demo_bot holds things
box:
  carry:                # driving
    joints: {lift: 0.5}
    path:
    - {xyz: [0.8, 0.0, 0.1], rpy: [0.0, 0.0, 0.0]}
"""
MISSION = """goal: Carry a box
loop: once
steps:
- instruction: Carry it
  skill: move_object
  args: {to: {pose: carry}}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    (tmp_path / 'config' / 'robots' / ROBOT / 'held_poses.yaml').write_text(HELD)
    (tmp_path / 'config' / 'missions' / 'carry_box.yaml').write_text(MISSION)
    return ConfigStore(tmp_path / 'config')


def test_bad_held_poses_are_refused(store):
    with pytest.raises(held.HeldError, match='at least one waypoint'):
        held.save(store, ROBOT, 'box', 'carry', {'path': []})
    with pytest.raises(held.HeldError, match='waypoint 1 x y z'):
        held.save(store, ROBOT, 'box', 'carry', {'path': [{'xyz': [1, 0], 'rpy': [0, 0, 0]}]})
    with pytest.raises(held.HeldError, match='no object type'):
        held.save(store, ROBOT, 'crate', 'carry', {'path': [{'xyz': [1, 0, 0], 'rpy': [0, 0, 0]}]})
    with pytest.raises(held.HeldError, match='used by carry_box'):
        held.save(store, ROBOT, 'box', 'carry', {'path': [{'xyz': [1, 0, 0], 'rpy': [0, 0, 0]}], 'rename': 'drive'})
    with pytest.raises(held.HeldError, match='used by carry_box'):
        held.delete(store, ROBOT, 'box', 'carry')


def test_the_held_pose_a_step_names_is_read_from_to_pose_and_the_run_args_nest_it():
    assert held.args('carry') == {'to': {'pose': 'carry'}}
    assert held.pose_of({'to': {'pose': 'carry'}}) == 'carry'
    assert held.pose_of({'to': {'by': [0, 0, 1]}}) is None and held.pose_of({'to': {'pose': '{p}'}}) is None
    assert held.pose_of({}) is None and held.pose_of(None) is None
