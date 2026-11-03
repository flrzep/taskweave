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
- {id: lift_old, instruction: Lift it (schema 4), skill: move, args: {object: held, by: [0, 0, 0.05]}}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURES / 'targets_new', tmp_path / 'config')
    (tmp_path / 'config' / 'missions' / 'forms.yaml').write_text(MISSION)
    config = ConfigStore(tmp_path / 'config')
    assert config.targets_format()
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
        'drive': 'navigate', 'straight': 'straight', 'tuck': 'joints', 'lift': 'joints', 'hands': 'hand',
        'back': 'straight', 'up': 'hand', 'carry': 'hand', 'later': '', 'rest': '', 'lift_old': 'hand'}
    assert steps['drive']['resources'][0] == 'base' and steps['lift']['resources'] == ['torso']
    assert steps['up']['resources'] == ['hands'] and 'hands' in steps['carry']['resources']
    assert steps['later']['resources'] == ['base', 'hands', 'torso', 'joints']   # unknown yet: what the schema says
    assert steps['rest']['resources'] == []
    assert plan['steps'][0].keys() >= {'id', 'instruction', 'skill', 'args', 'timeout_s', 'retries', 'on_failure'}


def test_the_plan_preview_has_the_notes_of_each_step(store):
    steps = {step['id']: step for step in missions.preview_plan(store, 'forms', ROBOT, move_catalog())['steps']}
    assert [note.split(': ', 1)[0] for note in steps['lift_old']['notes']] == ['args.object']
    assert 'hand: holding' in steps['lift_old']['notes'][0]       # the schema 4 form runs, noted
    assert all(step['notes'] == [] for key, step in steps.items() if key != 'lift_old')


def test_the_plan_preview_without_a_catalog_still_knows_the_move_kinds(store):
    steps = {step['id']: step for step in missions.preview_plan(store, 'forms', ROBOT)['steps']}
    assert steps['hands']['kind'] == 'hand' and steps['rest'] == {**steps['rest'], 'kind': '', 'resources': []}


# ---- the places of a check's problems, the notes ------------------------------------------------------------------
def test_a_check_places_the_notes_of_the_steps(store):
    (store.root / 'missions' / 'block.yaml').write_text(
        'goal: b\nsteps:\n- {instruction: b, skill: move, args: {object: held, at: [0.5, 0, 0.8]}}\n')
    store.reload()
    up = {'object': 'held', 'by': [0, 0, 1]}
    mission = {'goal': 'g', 'steps': [{'instruction': 'w', 'skill': 'wait', 'args': {'seconds': 1}},
                                      {'instruction': 'up', 'skill': 'move', 'args': up},
                                      {'id': 'inc', 'instruction': 'block', 'include': 'block'}]}
    problems, _ = missions.check(mission, store, move_catalog(), ROBOT)
    assert problems == []                                          # schema 4: it runs
    notes = missions.notes(mission, store, ROBOT)
    assert [{k: v for k, v in note.items() if k != 'message'} for note in notes] == [
        {'step': 1, 'path': ['args', 'object'], 'include': None}, {'step': 2, 'path': [], 'include': 'block'}]
    assert 'hand: holding' in notes[0]['message'] and notes[1]['message'].startswith('args.object: ')
    assert missions.notes({'goal': 'g', 'steps': 'none'}, store, ROBOT) == []          # a schema error: check says it
    assert missions.step_notes('wait', {'object': 'held', 'by': [0, 0, 1]}, store) == []


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
def test_a_teach_on_a_targets_cell_sends_the_kind_and_the_target():
    request = targets.teach_request(NEW_FIELDS, 'hand', 'part_box_1/top_grasp', ['left'])
    assert request == {'name': 'part_box_1/top_grasp', 'kind': 'hand', 'tools': ['left'], 'instance_override': False}
    assert targets.teach_request(NEW_FIELDS, 'base', 'bay_2_stack_2_back/pick',
                                 instance_override=True)['instance_override'] is True
    assert targets.teach_request(NEW_FIELDS, 'joints', 'home')['kind'] == 'joints'
    assert targets.teach_request(NEW_FIELDS, 'object', 'carry')['kind'] == 'object'
    with pytest.raises(ValueError, match='older than this cell pack'):
        targets.teach_request(OLD_FIELDS, 'hand', 'part_box_1/top_grasp')
    with pytest.raises(ValueError, match='unknown teach kind'):
        targets.teach_request(NEW_FIELDS, 'object_point', 'part_box_1/top_grasp')


# ---- the move skill in the catalog ---------------------------------------------------------------------------------
def test_the_catalog_tells_the_editor_which_options_each_form_of_move_takes():
    catalog = [{'name': 'move', 'args_schema': move_catalog()['move']}, {'name': 'wait', 'args_schema': {}}]
    annotated = targets.annotate_catalog(catalog)
    info = annotated[0]['args_schema']['x-taskweave-move']
    assert info['options']['offset_m'] == ['hand', 'carry'] and info['options']['tolerance_deg'] == ['posture']
    assert info['motion']['via'] == ['hand', 'joints'] and info['modes']['navigate'] == ['nav']
    assert info['forms'] == {'to': None, 'hand': ['hand'], 'object': ['carry'], 'base': ['straight'],
                             'elbow': ['posture'], 'joints': ['joints']}
    assert info['holding'] == {'value': 'holding', 'forms': ['carry']}          # hand: holding is a carry
    assert info['target_forms'] == {'base': 'navigate', 'hand': 'hand', 'object': 'carry', 'joints': 'joints'}
    assert info['labels']['navigate'] == 'navigate to' and info['labels']['hand'] == 'move to target'
    assert info['not_relative'] == ['offset_m', 'reference']
    assert info['relative'] == {'hand': ['by', 'frame', 'turn_deg', 'tilt_deg', 'at', 'rpy_deg'],
                                'carry': ['by', 'turn_deg', 'tilt_deg', 'at']}   # core's RELATIVE_KEYS, by and at
    assert set(info['hidden']) == {'dry_run', 'look_at'}
    assert annotated[1] == catalog[1] and 'x-taskweave-move' not in catalog[0]['args_schema']   # not changed in place
    assert targets.annotate_catalog([{'name': 'move', 'args_schema': {'properties': {}}}])[0]['args_schema'] == \
        {'properties': {}}


def test_the_catalog_has_no_old_ids_of_the_move_forms():
    from taskweave_core.config.targets import MOVE_FORMS

    info = targets.move_meta()
    keyed = [info['modes'], info['relative']]                  # tables keyed by form
    listed = [*info['options'].values(), *info['motion'].values(), *(v for v in info['forms'].values() if v),
              info['holding']['forms'], list(info['target_forms'].values())]
    assert all(set(table) <= set(MOVE_FORMS) for table in keyed)
    assert all(set(forms) <= set(MOVE_FORMS) for forms in listed)
    assert set(info['labels']) <= set(MOVE_FORMS)


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
    teach = arm.overview(ConfigStore(store.root), ROBOT)    # the Teach tab: listed, and Capture offered
    assert 'tilted' in {item['name'] for item in teach['held']}
