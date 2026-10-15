import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import arm, held

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
  args: {pose: carry}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    (tmp_path / 'config' / 'robots' / ROBOT / 'held_poses.yaml').write_text(HELD)
    (tmp_path / 'config' / 'missions' / 'carry_box.yaml').write_text(MISSION)
    return ConfigStore(tmp_path / 'config')


def test_held_poses_are_listed_with_their_missions(store):
    poses = held.overview(store, ROBOT)
    assert poses == [{'object_type': 'box', 'name': 'carry', 'joints': {'lift': 0.5},
                      'path': [{'xyz': [0.8, 0.0, 0.1], 'rpy': [0.0, 0.0, 0.0]}], 'used_by': ['carry_box']}]
    assert arm.overview(store, ROBOT, set())['held'] == poses


def test_a_held_pose_is_saved_created_and_renamed(store):
    held.save(store, ROBOT, 'box', 'carry', {'joints': {'lift': 0.4}, 'path': [
        {'xyz': [0.8, 0, 0.2], 'rpy': [0, 0, 0]}, {'xyz': ['0.8', 0, 0.3], 'rpy': [0, -1.5708, 0]}]})
    held.save(store, ROBOT, 'box', 'stand_up', {'path': [{'xyz': [1, 0, 0.3], 'rpy': [0, -1.5708, 0]}]})
    text = (store.root / 'robots' / ROBOT / 'held_poses.yaml').read_text()
    assert text.startswith('# where demo_bot holds things') and 'carry:                # driving' in text
    assert '- {xyz: [0.8, 0.0, 0.3], rpy: [0.0, -1.5708, 0.0]}' in text
    pose = ConfigStore(store.root).held_pose(ROBOT, 'box', 'carry')
    assert pose['joints'] == {'lift': 0.4} and [p.xyz for p in pose['path']] == [(0.8, 0.0, 0.2), (0.8, 0.0, 0.3)]
    assert ConfigStore(store.root).held_pose(ROBOT, 'box', 'stand_up')['joints'] == {}
    held.save(store, ROBOT, 'box', 'stand_up', {'path': [{'xyz': [1, 0, 0.3], 'rpy': [0, 0, 0]}], 'rename': 'upright'})
    assert sorted(p['name'] for p in held.overview(ConfigStore(store.root), ROBOT)) == ['carry', 'upright']


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


def test_an_unused_held_pose_can_be_deleted(store):
    held.save(store, ROBOT, 'box', 'spare', {'path': [{'xyz': [1, 0, 0], 'rpy': [0, 0, 0]}]})
    held.delete(store, ROBOT, 'box', 'spare')
    assert [p['name'] for p in held.overview(ConfigStore(store.root), ROBOT)] == ['carry']
