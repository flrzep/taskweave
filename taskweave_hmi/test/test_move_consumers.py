"""The HMI as a consumer of the move skill and the targets (P6-4), without ROS: the plan preview with each step's kind
and resources, the places of a check's problems (step and field), the teach request per cell layout, the move skill's
options in the catalog, and the held poses the store cannot read."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import arm, held, missions, targets

FIXTURES = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures'
ROBOT = 'bot_two_arm'
NEW_FIELDS = ('name', 'kind', 'instance_override', 'tools')
OLD_FIELDS = ('name', 'kind', 'tools')
MISSION = """goal: Every form of move
steps:
- {id: drive, instruction: Drive, skill: move, args: {to: waiting}}
- {id: straight, instruction: Drive straight, skill: move, args: {to: waiting, motion: {mode: straight}}}
- {id: tuck, instruction: Arms home, skill: move, args: {to: home}}
- {id: lift, instruction: Lift, skill: move, args: {to: lift_high}}
- {id: hands, instruction: Hands, skill: move, args: {to: frame_grasp_approach}}
- {id: back, instruction: Back out, skill: move, args: {base: {by: [-0.3, 0]}}}
- {id: up, instruction: Up, skill: move, args: {hand: left, by: [0, 0, 0.06], torso: keep}}
- {id: carry, instruction: Carry, skill: move, args: {to: carry}}
- {id: later, instruction: Later, skill: move, args: {to: '{spot}'}}
- {id: rest, instruction: Rest, skill: wait, args: {seconds: 1}}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURES / 'targets_new', tmp_path / 'config')
    (tmp_path / 'config' / 'missions' / 'forms.yaml').write_text(MISSION)
    config = ConfigStore(tmp_path / 'config')
    assert config.targets_format()
    return config


@pytest.fixture
def old_store(tmp_path):
    shutil.copytree(FIXTURES / 'config', tmp_path / 'old')
    config = ConfigStore(tmp_path / 'old')
    assert not config.targets_format()
    return config


def move_catalog() -> dict:
    from taskweave_skills.move import Move

    return {'move': Move.schema_for(['left', 'right']),
            'wait': {'type': 'object', 'properties': {'seconds': {'type': 'number'}},
                     'x-taskweave-resources': []}}


# ---- plan preview: kind and resources per step --------------------------------------------------------------------
def test_the_plan_preview_has_each_steps_kind_and_resources(store):
    plan = missions.preview_plan(store, 'forms', ROBOT, move_catalog())
    steps = {step['id']: step for step in plan['steps']}
    assert {key: step['kind'] for key, step in steps.items()} == {
        'drive': 'base', 'straight': 'straight', 'tuck': 'joints', 'lift': 'joints', 'hands': 'hand',
        'back': 'straight', 'up': 'hand', 'carry': 'object', 'later': '', 'rest': ''}
    assert steps['drive']['resources'][0] == 'base' and steps['lift']['resources'] == ['torso']
    assert steps['up']['resources'] == ['hands'] and 'hands' in steps['carry']['resources']
    assert steps['later']['resources'] == ['base', 'hands', 'torso', 'joints']   # unknown yet: what the schema says
    assert steps['rest']['resources'] == []
    assert plan['steps'][0].keys() >= {'id', 'instruction', 'skill', 'args', 'timeout_s', 'retries', 'on_failure'}


def test_the_plan_preview_without_a_catalog_still_knows_the_move_kinds(store):
    steps = {step['id']: step for step in missions.preview_plan(store, 'forms', ROBOT)['steps']}
    assert steps['hands']['kind'] == 'hand' and steps['rest'] == {**steps['rest'], 'kind': '', 'resources': []}


def test_an_old_cell_plan_preview_has_no_kinds(old_store):
    plan = missions.preview_plan(old_store, 'patrol', 'demo_bot', {})
    assert plan['steps'] and all(step['kind'] == '' for step in plan['steps'])


# ---- the places of a check's problems ------------------------------------------------------------------------------
def test_a_check_names_the_step_and_the_field_of_each_problem(store):
    mission = {'goal': 'g', 'steps': [
        {'instruction': 'ok', 'skill': 'wait', 'args': {'seconds': 1}},
        {'instruction': 'base with hands', 'skill': 'move', 'args': {'to': 'waiting', 'hands': 'left'}},
        {'instruction': 'mode', 'skill': 'move', 'args': {'to': 'home', 'motion': {'mode': 'linear'}}},
        {'instruction': 'old name', 'skill': 'navigate_to', 'args': {'target': 'waiting'}},
        {'instruction': 'unknown', 'skill': 'move', 'args': {'to': 'wating'}},
    ]}
    problems, places = missions.check(mission, store, move_catalog(), ROBOT)
    at = {(place['step'], tuple(place['path'])): problems[place['problem']] for place in places}
    assert "'waiting' moves the base" in at[(1, ('args', 'hands'))]
    assert 'motion.mode' in at[(2, ('args', 'motion', 'mode'))]
    assert 'replaced by move' in at[(3, ('skill',))]
    assert "unknown target 'wating'" in at[(4, ('args', 'to'))]
    assert all(place['include'] is None for place in places) and len(places) == len(problems)


def test_a_refused_save_carries_the_places(store):
    mission = {'goal': 'g', 'steps': [{'instruction': 'x', 'skill': 'move', 'args': {'to': 'waiting', 'offset_m': 1}}]}
    with pytest.raises(missions.MissionError) as error:
        missions.save('bad', mission, store, move_catalog(), ROBOT)
    assert [{k: v for k, v in place.items() if k != 'message'} for place in error.value.places] == [
        {'problem': 0, 'step': 0, 'path': ['args', 'offset_m'], 'include': None}]
    assert "'waiting' moves the base" in error.value.places[0]['message']
    assert error.value.problems[0].startswith("step 's1' (move): ")      # the place's message has no step prefix
    assert not error.value.places[0]['message'].startswith('step')


def test_a_problem_in_an_included_mission_marks_the_include_step(store):
    (store.root / 'missions' / 'block.yaml').write_text(
        'goal: b\nsteps:\n- {instruction: b, skill: move, args: {to: home, hands: left}}\n')
    store.reload()
    mission = {'goal': 'g', 'steps': [{'instruction': 'w', 'skill': 'wait', 'args': {'seconds': 1}},
                                      {'id': 'inc', 'instruction': 'block', 'include': 'block'}]}
    problems, places = missions.check(mission, store, move_catalog(), ROBOT)
    assert problems and [{k: v for k, v in p.items() if k != 'message'} for p in places] == [
        {'problem': 0, 'step': 1, 'path': [], 'include': 'block'}]


def test_schema_errors_of_a_step_have_places_too(store):
    problems, places = missions.check({'goal': 'g', 'steps': [{'instruction': 'x', 'skill': 'wait', 'args': {},
                                                               'on_failure': 'later'}]}, store, {}, ROBOT)
    assert problems and places[0]['step'] == 0 and places[0]['path'] == ['on_failure']


# ---- teach requests --------------------------------------------------------------------------------------------------
def test_a_teach_on_a_targets_cell_sends_the_kind_and_the_target(store):
    request = targets.teach_request(store, NEW_FIELDS, 'hand', 'part_box_1/top_grasp', ['left'])
    assert request == {'name': 'part_box_1/top_grasp', 'kind': 'hand', 'tools': ['left'], 'instance_override': False}
    assert targets.teach_request(store, NEW_FIELDS, 'base', 'bay_2_stack_2_back/pick',
                                 instance_override=True)['instance_override'] is True
    assert targets.teach_request(store, NEW_FIELDS, 'joints', 'home')['kind'] == 'joints'
    assert targets.teach_request(store, NEW_FIELDS, 'object', 'carry')['kind'] == 'object'
    with pytest.raises(ValueError, match='older than this cell pack'):
        targets.teach_request(store, OLD_FIELDS, 'hand', 'part_box_1/top_grasp')
    with pytest.raises(ValueError, match='unknown teach kind'):
        targets.teach_request(store, NEW_FIELDS, 'object_point', 'part_box_1/top_grasp')


def test_a_teach_on_an_old_cell_sends_the_old_kinds(old_store):
    for fields in (OLD_FIELDS, NEW_FIELDS):   # whatever the service has: the old cell's request
        request = targets.teach_request(old_store, fields, 'hand', 'bin_1/grasp', ['left'])
        assert request == {'name': 'bin_1/grasp', 'kind': 'object_point', 'tools': ['left']}
    assert targets.teach_request(old_store, OLD_FIELDS, 'base', 'dock')['kind'] == 'location'
    assert targets.teach_request(old_store, OLD_FIELDS, 'base', 'bin_1/pick')['kind'] == 'stance'
    assert targets.teach_request(old_store, OLD_FIELDS, 'base', 'bin_1/pick', instance_override=True)['kind'] == \
        'stance_instance'
    with pytest.raises(ValueError, match='before schema 4'):
        targets.teach_request(old_store, OLD_FIELDS, 'joints', 'home')


# ---- the move skill in the catalog ---------------------------------------------------------------------------------
def test_the_catalog_tells_the_editor_which_options_each_kind_of_move_takes():
    catalog = [{'name': 'move', 'args_schema': move_catalog()['move']}, {'name': 'wait', 'args_schema': {}}]
    annotated = targets.annotate_catalog(catalog)
    info = annotated[0]['args_schema']['x-taskweave-move']
    assert info['options']['offset_m'] == ['hand', 'object'] and info['options']['tolerance_deg'] == ['elbow']
    assert info['motion']['via'] == ['hand', 'joints'] and info['modes']['base'] == ['nav']
    assert info['forms']['to'] is None and info['forms']['base'] == ['straight']
    assert info['not_relative'] == ['offset_m', 'reference']
    assert set(info['hidden']) == {'dry_run', 'look_at'} and info['relative']['object'] == ['by', 'at', 'turn_deg',
                                                                                             'tilt_deg']
    assert annotated[1] == catalog[1] and 'x-taskweave-move' not in catalog[0]['args_schema']   # not changed in place
    assert targets.annotate_catalog([{'name': 'move', 'args_schema': {'properties': {}}}])[0]['args_schema'] == \
        {'properties': {}}


# ---- held poses the store cannot read ------------------------------------------------------------------------------
def test_an_unreadable_held_pose_is_listed_with_its_problem(store):
    path = store.root / 'robots' / ROBOT / 'targets.yaml'
    path.write_text(path.read_text() + 'tilted:\n  moves: object\n  type: frame_type2\n  joints: {nope: 1.0}\n'
                    '  path:\n  - {xyz: [1.0, 0.0, 0.4], rpy: [0.0, 0.0, 0.0]}\n')
    listed = {item['name']: item for item in held.overview(ConfigStore(store.root), ROBOT)}
    assert listed['carry']['problems'] == []
    bad = listed['tilted']
    assert bad['object_type'] == 'frame_type2' and bad['problems'] and 'nope' in bad['problems'][0]
    assert bad['path'] == [{'xyz': [1.0, 0.0, 0.4], 'rpy': [0.0, 0.0, 0.0]}] and bad['joints'] == {}
    teach = arm.overview(ConfigStore(store.root), ROBOT, set())    # the Teach tab: listed, and Capture offered
    assert teach['targets_layout'] is True and 'tilted' in {item['name'] for item in teach['held']}
