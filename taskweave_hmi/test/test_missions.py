import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi.missions import MissionError, list_missions, save, template, validate

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
CATALOG = {
    'navigate_to': {'type': 'object', 'required': ['target'],
                    'properties': {'target': {'type': 'string', 'x-taskweave-point': 'base_target'}}},
    'wait': {'type': 'object', 'required': ['seconds'], 'properties': {'seconds': {'type': 'number', 'minimum': 0}}},
}


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


def test_list_marks_invalid_files(store):
    missions = {m['name']: m for m in list_missions(store)}
    assert missions['patrol']['valid'] and missions['patrol']['steps'] == 3
    assert not missions['broken']['valid'] and 'robot' in missions['broken']['error']


def test_save_validates_before_writing(store):
    mission = template('demo_bot')
    mission['steps'][0] = {'instruction': 'go', 'skill': 'navigate_to', 'args': {'target': 'nowhere'}}
    with pytest.raises(MissionError) as error:
        save('new_one', mission, store, CATALOG, 'demo_bot')
    assert any("unknown location 'nowhere'" in p for p in error.value.problems)
    assert not (store.root / 'missions' / 'new_one.yaml').exists()


def test_save_keeps_top_level_comments_and_round_trips(store):
    path = store.root / 'missions' / 'patrol.yaml'
    path.write_text('# my patrol, edited by hand\n' + path.read_text())
    mission = store.mission('patrol')
    mission = {**mission, 'steps': mission['steps'][:2], 'defaults': {**mission['defaults'], 'retries': 3}}
    save('patrol', mission, store, CATALOG, 'demo_bot')
    text = path.read_text()
    assert text.startswith('# my patrol, edited by hand')
    reloaded = ConfigStore(store.root).mission('patrol')
    assert len(reloaded['steps']) == 2 and reloaded['defaults']['retries'] == 3


def test_schema_and_robot_problems_are_reported(store):
    mission = template('other_bot')
    mission['steps'][0]['on_failure'] = 'explode'
    problems = validate(mission, store, CATALOG, 'demo_bot')
    assert any('on_failure' in p for p in problems)
    mission['steps'][0]['on_failure'] = 'skip'
    problems = validate(mission, store, CATALOG, 'demo_bot')
    assert any("this system drives 'demo_bot'" in p for p in problems)


def test_invalid_name_rejected(store):
    with pytest.raises(MissionError, match='invalid mission name'):
        save('../escape', template('demo_bot'), store, CATALOG, 'demo_bot')
