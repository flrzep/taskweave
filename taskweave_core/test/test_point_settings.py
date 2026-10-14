"""Editing object point settings, tool poses and the robot's IK choice (HMI): comments and other keys are kept."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    profile = tmp_path / 'config' / 'robots' / 'demo_bot' / 'profile.yaml'
    profile.write_text(profile.read_text() + "ik:\n  groups: {left: left_arm}\n  joints: 'left_j[1-7]'\n"
                       "  choose: {elbow: out, limit_margin_deg: 25}   # kept comment\n")
    return ConfigStore(tmp_path / 'config')


def test_point_settings_are_set_and_cleared(store):
    store.save_object_point_settings('box', 'top_grasp', {'approach_m': 0.12, 'elbow': 'down'})
    point = store.object_point('box', 'top_grasp')
    assert point['approach_m'] == 0.12 and point['elbow'] == 'down'
    assert point['grip_width_m'] == 0.05 and point['left_tool'].xyz == (0.0, 0.1, 0.2)  # the rest is kept
    store.save_object_point_settings('box', 'top_grasp', {'approach_m': None, 'elbow': None})
    point = store.object_point('box', 'top_grasp')
    assert point['approach_m'] is None and point['elbow'] is None


def test_an_unknown_elbow_side_is_refused_and_nothing_written(store):
    before = (store.root / 'objects' / 'box.yaml').read_text()
    with pytest.raises(ValueError, match='elbow'):
        store.save_object_point_settings('box', 'top_grasp', {'elbow': 'sideways'})
    assert (store.root / 'objects' / 'box.yaml').read_text() == before


def test_tool_poses_are_replaced_in_the_object_frame(store):
    store.save_object_point_poses('box', 'wide_grasp', {'left': {'xyz': [0.01, 0.02, 0.25], 'rpy': [0, 3.1416, 0]}})
    pose = store.object_point('box', 'wide_grasp')['left_tool']
    assert pose.xyz == (0.01, 0.02, 0.25) and pose.rpy[1] == pytest.approx(3.1416)
    with pytest.raises(ValueError, match='no point'):
        store.save_object_point_poses('box', 'nowhere', {'left': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}})


def test_the_ik_choice_is_updated_and_the_rest_of_the_profile_kept(store):
    store.save_ik_choice('demo_bot', {'elbow': 'down', 'weights': {'motion': 0.5}})
    choose = store.robot('demo_bot').data['ik']['choose']
    assert choose == {'elbow': 'down', 'limit_margin_deg': 25, 'weights': {'motion': 0.5}}
    assert '# kept comment' in (store.root / 'robots' / 'demo_bot' / 'profile.yaml').read_text()
    store.save_ik_choice('demo_bot', {'elbow': None})
    assert 'elbow' not in store.robot('demo_bot').data['ik']['choose']
