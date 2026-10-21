"""The point kind ``target`` of the plan validation, the per-kind option check, and kind / resources of a step, on the
hand-made cell in the targets layout (no skill package: the schemas are small stand-ins; the real ``move`` schema is
tested with the skills)."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_core.plan import Plan, Step
from taskweave_core.step_info import move_problems, resolve_form, step_info
from taskweave_core.validation import check_point, plan_step_info, validate_args, validate_plan

FIXTURES = Path(__file__).parent / 'fixtures'
ROBOT = 'bot_two_arm'
TARGET = {'type': 'string', 'x-taskweave-point': 'target'}
SCHEMA = {'type': 'object', 'properties': {
    'to': {**TARGET, 'x-taskweave-target-kinds': ['hand', 'joints']},
    'any': TARGET,
    'hands': {'type': 'string'}, 'torso': {}, 'motion': {'type': 'object'}, 'only': {}, 'offset_m': {}, 'by': {},
    'hand': {}, 'joints': {}, 'base': {}, 'elbow': {}, 'object': {}, 'reference': {}},
    'x-taskweave-resources': ['base', 'hands', 'torso', 'joints']}


@pytest.fixture
def store(tmp_path) -> ConfigStore:
    shutil.copytree(FIXTURES / 'targets_new', tmp_path / 'cell')
    return ConfigStore(tmp_path / 'cell')


def test_a_target_is_any_name_the_store_resolves(store):
    for name in ('waiting', 'home', 'frame_grasp_approach', 'carry', 'bay_2_stack_2_front/rails_top',
                 'bay_2_stack_2_front/pick', 'karakuri_hanger/hook'):
        assert check_point('target', name, store, ROBOT) is None, name
    for name in ('nowhere', 'bay_2_stack_2_front/nowhere', 'stance:bay_2_stack_2_front', 'location:waiting'):
        assert check_point('target', name, store, ROBOT), name


def test_the_kinds_a_field_takes_are_enforced(store):
    assert check_point('target', 'home', store, ROBOT, ('hand', 'joints')) is None
    assert check_point('target', 'frame_grasp_approach', store, ROBOT, ('hand', 'joints')) is None
    assert "'waiting' moves the base" in check_point('target', 'waiting', store, ROBOT, ('hand', 'joints'))
    assert check_point('target', 'waiting', store, ROBOT, ('base',)) is None
    assert check_point('target', 'waiting', store, ROBOT, ()) is None


def test_the_field_in_a_schema_carries_its_kinds(store):
    assert validate_args({'to': 'home', 'any': 'waiting'}, SCHEMA, store, ROBOT) == []
    [text] = validate_args({'to': 'waiting'}, SCHEMA, store, ROBOT)
    assert text.startswith('args.to: ') and 'moves the base' in text


def test_the_old_point_kinds_are_refused_on_a_targets_cell(store):
    for kind in ('base_target', 'joint_pose', 'tool_pose', 'object_point', 'instance_point', 'instance_contact',
                 'via_target', 'joint_pose_only', 'held_pose'):
        assert 'before schema 4' in check_point(kind, 'waiting', store, ROBOT), kind
    assert check_point('held_object', 'karakuri_hanger', store, ROBOT) is None
    assert check_point('joint_group', 'left_arm', store, ROBOT) is None
    assert 'unknown point kind' in check_point('nope', 'x', store, ROBOT)


def test_the_form_of_a_step(store):
    def form(**args):
        return resolve_form(args, store, ROBOT)[0]

    assert form(to='waiting') == 'base' and form(to='waiting', motion={'mode': 'straight'}) == 'straight'
    assert form(to='frame_grasp_approach') == 'hand' and form(to='carry') == 'object' and form(to='home') == 'joints'
    assert form(hand='left', by=[0, 0, 1]) == 'hand' and form(object='held', by=[0, 0, 1]) == 'object'
    assert form(base={'by': [1, 0]}) == 'straight' and form(elbow='out') == 'elbow' and form(joints={}) == 'joints'
    assert form(to='{x}') is None and form() is None
    assert 'nowhere' in resolve_form({'to': 'nowhere'}, store, ROBOT)[2]


def test_the_option_problems_name_the_target_and_the_option(store):
    [(path, text)] = move_problems({'to': 'waiting', 'hands': 'all'}, store, ROBOT)
    assert path == ['args', 'hands'] and text == "'waiting' moves the base; option hands does not apply"
    [(path, text)] = move_problems({'to': 'home', 'motion': {'mode': 'free'}}, store, ROBOT)
    assert path == ['args', 'motion', 'mode'] and 'motion.mode' in text
    assert move_problems({'to': 'frame_grasp_approach', 'hands': 'all', 'offset_m': 0.1}, store, ROBOT) == []


def test_kind_and_resources(store):
    def info(args, skill='move'):
        return step_info(skill, args, SCHEMA, store, ROBOT)

    assert info({'to': 'waiting'}) == ('base', ['base'])
    assert info({'to': 'frame_grasp_approach'}) == ('hand', ['hands', 'torso', 'joints'])    # its head joints too
    assert info({'to': 'frame_grasp_approach', 'torso': 'keep'}) == ('hand', ['hands', 'joints'])   # the step wins
    assert info({'to': 'bay_2_stack_2_front/rails_top', 'torso': 'keep'}) == ('hand', ['hands'])
    assert info({'to': 'home'}) == ('joints', ['hands', 'joints'])          # arms and head, no torso in home
    assert info({'to': 'lift_high'}) == ('joints', ['torso'])
    assert info({'to': 'home', 'only': ['head']}) == ('joints', ['joints'])
    assert info({'to': '{x}'}) == (None, ['base', 'hands', 'torso', 'joints'])
    assert info({'x': 1}, skill='other') == (None, ['base', 'hands', 'torso', 'joints'])
    assert step_info('other', {}, None, store, ROBOT) == (None, [])


def test_a_plan_of_move_steps(store):
    steps = [Step('s1', 'drive', 'move', {'to': 'waiting'}), Step('s2', 'reach', 'move', {'to': 'carry'}),
             Step('s3', 'off', 'navigate_to', {'target': 'waiting'})]
    plan = Plan('p', 'g', ROBOT, steps)
    everything = {**SCHEMA, 'properties': {**SCHEMA['properties'], 'to': {**TARGET}}}
    catalog = {'move': everything}
    problems = validate_plan(plan, catalog, store)
    assert len(problems) == 1 and 'navigate_to was replaced by move' in problems[0]
    assert [(s.kind, s.resources) for s in plan.steps] == [('base', ['base']), ('object', ['hands', 'torso']),
                                                           (None, [])]
    assert plan_step_info(plan, catalog, store)[0] == {'id': 's1', 'kind': 'base', 'resources': ['base']}
