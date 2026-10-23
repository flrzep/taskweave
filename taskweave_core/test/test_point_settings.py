"""Editing the robot's IK choice (HMI): comments and other keys are kept."""

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


def test_the_ik_choice_is_updated_and_the_rest_of_the_profile_kept(store):
    store.save_ik_choice('demo_bot', {'elbow': 'down', 'weights': {'motion': 0.5}})
    choose = store.robot('demo_bot').data['ik']['choose']
    assert choose == {'elbow': 'down', 'limit_margin_deg': 25, 'weights': {'motion': 0.5}}
    assert '# kept comment' in (store.root / 'robots' / 'demo_bot' / 'profile.yaml').read_text()
    store.save_ik_choice('demo_bot', {'elbow': None})
    assert 'elbow' not in store.robot('demo_bot').data['ik']['choose']
