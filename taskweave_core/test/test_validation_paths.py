"""Path-based argument validation: placeholders and point checks at any depth, structured problems, oneOf variants."""

import pytest

from taskweave_core.config import ConfigStore
from taskweave_core.config.schemas import MOTION
from taskweave_core.plan import Plan
from taskweave_core.validation import (
    oneof_problems,
    pick_variant,
    placeholder_paths,
    validate_args,
    validate_args_structured,
    validate_plan,
    validate_plan_structured,
)

VEC3 = {'type': 'array', 'items': {'type': 'number'}, 'minItems': 3, 'maxItems': 3}
HANDS = {'anyOf': [{'enum': ['auto', 'all', 'left']}, {'type': 'array', 'items': {'enum': ['left']}, 'minItems': 1}]}
SCHEMA = {
    'type': 'object', 'required': ['to'], 'additionalProperties': False, 'properties': {
        'to': {'oneOf': [
            {'title': 'object point', 'type': 'object', 'required': ['point'], 'additionalProperties': False,
             'properties': {'point': {'type': 'string', 'x-taskweave-point': 'instance_point'},
                            'offset_m': {'type': 'number', 'minimum': 0}}},
            {'title': 'named pose', 'type': 'object', 'required': ['pose'], 'additionalProperties': False,
             'properties': {'pose': {'type': 'string', 'x-taskweave-point': 'joint_pose'}}},
            {'title': 'relative', 'type': 'object', 'required': ['by'], 'additionalProperties': False,
             'properties': {'by': VEC3, 'turn_deg': {'type': 'number'}}},
        ]},
        'hands': HANDS,
        'motion': MOTION,
        'steps': {'type': 'array', 'items': {'type': 'string', 'x-taskweave-point': 'instance_point'}},
    },
}


@pytest.fixture
def store(config_dir):
    (config_dir / 'cell' / 'objects.yaml').write_text('box_1: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0}\n')
    return ConfigStore(config_dir)


def check(args, store, later=frozenset()):
    return validate_args_structured(args, SCHEMA, store, 'demo_bot', later)


def test_valid_nested_args_pass(store):
    assert check({'to': {'point': 'box_1/top_grasp', 'offset_m': 0.1}, 'motion': {'mode': 'linear'}}, store) == []
    assert check({'to': {'by': [0, 0, 0.1]}, 'hands': ['left']}, store) == []


def test_nested_point_check_reports_the_nested_path(store):
    assert check({'to': {'point': 'box_1/nope'}}, store)[0][0] == ['args', 'to', 'point']
    [(path, message)] = check({'to': {'point': 'box_1/nope'}}, store)
    assert 'no point' in message
    strings = validate_args({'to': {'point': 'box_1/nope'}}, SCHEMA, store, 'demo_bot')
    assert strings == [f'args.to.point: {message}']


def test_point_check_in_the_other_variant_and_in_list_items(store):
    [(path, message)] = check({'to': {'pose': 'nowhere'}}, store)
    assert path == ['args', 'to', 'pose'] and 'nowhere' in message
    problems = check({'to': {'by': [0, 0, 0]}, 'steps': ['box_1/top_grasp', 'box_1/nope']}, store)
    assert [p for p, _ in problems] == [['args', 'steps', 1]]


def test_point_of_an_object_put_down_earlier_is_not_checked(store):
    assert check({'to': {'point': 'floor/top_grasp'}}, store, later={'floor'}) == []
    assert check({'to': {'point': 'floor/top_grasp'}}, store) != []


def test_placeholder_for_a_nested_scalar(store):
    assert check({'to': {'point': '{frame}/rails_top'}}, store) == []
    assert check({'to': {'point': 'stance:{stack}', 'offset_m': '{d}'}}, store) == []
    # the other fields of the object are still checked
    assert check({'to': {'point': '{p}', 'offset_m': -1}}, store)[0][0] == ['args', 'to', 'offset_m']


def test_placeholder_in_one_vec3_component(store):
    assert check({'to': {'by': ['{dx}', 0, 0.1]}}, store) == []
    assert check({'to': {'by': [0, '{dy}', '{dz}']}}, store) == []
    [(path, _)] = check({'to': {'by': ['{dx}', 'text', 0.1]}}, store)   # the concrete components still count
    assert path[:3] == ['args', 'to', 'by']
    assert check({'to': {'by': ['{dx}', 0]}}, store) != []              # length is still checked


def test_placeholder_for_a_whole_field(store):
    assert check({'to': {'by': '{offset}'}}, store) == []
    assert check({'to': '{target}'}, store) == []
    assert check({'to': {'by': [0, 0, 0]}, 'hands': '{arm}'}, store) == []
    assert check({'to': {'by': [0, 0, 0]}, 'hands': ['{arm}']}, store) == []
    assert check({'to': {'by': [0, 0, 0]}, 'motion': {'mode': '{m}', 'speed': '{s}'}}, store) == []


def test_missing_required_and_unknown_keys_still_fail(store):
    assert check({}, store) != []
    assert check({'to': {'by': [0, 0, 0]}, 'extra': 1}, store) != []
    assert check({'to': {'by': '{o}', 'bogus': 1}}, store) != []     # a placeholder is no excuse for other keys
    assert check({'to': {'by': [0, 0, 0]}, 'motion': {'mode': 'sideways'}}, store)[0][0] == ['args', 'motion', 'mode']


def test_oneof_failure_is_reported_inside_the_variant(store):
    problems = check({'to': {'by': [0, 0, 0], 'offset_m': 1}}, store)
    assert problems and all(path[:2] == ['args', 'to'] for path, _ in problems)
    assert any('offset_m' in message for _, message in problems)


def test_oneof_without_any_variant_key_lists_them(store):
    [(path, message)] = check({'to': {'turn_deg': 3}}, store)
    assert path == ['args', 'to'] and 'point' in message and 'pose' in message and 'by' in message


def test_placeholder_paths():
    assert placeholder_paths({'a': ['{x}', 1, {'b': 'y:{z}'}], 'c': '{w}', 'd': 3}) == [
        ('a', 0), ('a', 2, 'b'), ('c',)]
    assert placeholder_paths('{x}') == [()] and placeholder_paths(5) == []


def test_pick_variant_by_the_unique_required_key():
    to = SCHEMA['properties']['to']
    assert pick_variant(to, {'point': '{p}', 'offset_m': 0.1}) == 0
    assert pick_variant(to, {'pose': 'home'}) == 1
    assert pick_variant(to, {'by': '{offset}'}) == 2          # a placeholder does not matter
    assert pick_variant(to, {}) == 0 and pick_variant(to, None) == 0   # a new empty step: the first variant
    assert pick_variant(to, {'turn_deg': 1}) is None and pick_variant(to, 'x') is None
    assert pick_variant({'type': 'object'}, {'a': 1}) is None
    joints = {'oneOf': [{'required': ['pose']}, {'required': ['joints']}]}
    assert pick_variant(joints, {'joints': {}}) == 1


def test_oneof_uniqueness_check():
    assert oneof_problems(SCHEMA) == []
    assert oneof_problems({'oneOf': [{'required': ['pose']}, {'required': ['joints']}]}) == []
    shared = {'properties': {'to': {'oneOf': [{'required': ['a', 'b']}, {'required': ['a', 'b']}]}}}
    problems = oneof_problems(shared)
    assert len(problems) == 2 and 'schema.properties.to.oneOf[0]' in problems[0]    # found at depth
    assert oneof_problems({'oneOf': [{'required': ['a']}, {'properties': {}}]})      # a variant without a key
    two = {'oneOf': [{'required': ['onto', 'contact']}, {'required': ['by']}]}
    assert oneof_problems(two) and oneof_problems(two, allow_several=True) == []
    assert oneof_problems({'oneOf': [{'required': ['a']}, {'required': ['a']}]})      # the same key twice


CATALOG = {'move': SCHEMA}


def plan_of(store, *steps):
    mission = {'goal': 'g', 'steps': [{'skill': 'move', 'instruction': 'go', 'args': args} for args in steps]}
    return Plan.from_mission(mission, 'p', robot='demo_bot')


def test_plan_problems_are_also_structured(store):
    plan = plan_of(store, {'to': {'by': [0, 0, 0]}}, {'to': {'by': [0, 0, 0]}}, {'to': {'point': 'box_1/nope'}})
    strings = validate_plan(plan, CATALOG, store)
    structured = validate_plan_structured(plan, CATALOG, store)
    assert len(strings) == len(structured) == 1
    assert structured[0]['path'] == ['steps', 2, 'args', 'to', 'point']
    assert strings[0] == f"step 's3' (move): args.to.point: {structured[0]['message']}"


def test_plan_structured_covers_step_level_problems(store):
    plan = plan_of(store, {'to': {'by': [0, 0, 0]}})
    plan.steps[0].skill = 'fly_me'
    [problem] = validate_plan_structured(plan, CATALOG, store)
    assert problem['path'] == ['steps', 0, 'skill'] and 'no skill server' in problem['message']
    assert validate_plan(plan, CATALOG, store)[0].startswith("step 's1' (fly_me): no skill server")


def test_flat_schemas_behave_as_before(store):
    flat = {'type': 'object', 'required': ['target', 'n'], 'properties': {
        'target': {'type': 'string', 'x-taskweave-point': 'base_target'}, 'n': {'type': 'integer'}}}
    assert validate_args({'target': '{where}', 'n': '{n}'}, flat, store, 'demo_bot') == []
    assert validate_args({'n': '{n}'}, flat, store, 'demo_bot') == ["args: 'target' is a required property"]
    problems = validate_args({'target': 'nowhere', 'n': 'x'}, flat, store, 'demo_bot')
    assert problems[0].startswith('args.n: ') and problems[1].startswith('args.target: ')


def _step(args):
    return {'skill': 'move', 'instruction': 'go', 'args': args}


def _problems(store, mission, blocks):
    from taskweave_core.plan import expand_includes
    plan = Plan.from_mission(expand_includes(mission, blocks.__getitem__), 'p', robot='demo_bot')
    return validate_plan_structured(plan, CATALOG, store)


BAD = {'to': {'point': 'box_1/nope'}}
FINE = {'to': {'by': [0, 0, 0]}}


def test_origin_of_a_plain_mission(store):
    [problem] = _problems(store, {'goal': 'g', 'steps': [_step(FINE), _step(BAD)]}, {})
    assert problem['path'] == ['steps', 1, 'args', 'to', 'point']
    assert problem['origin'] == {'step': 1, 'include': None, 'inner_path': ['steps', 1, 'args', 'to', 'point']}


def test_origin_of_a_step_from_an_include(store):
    blocks = {'inner': {'goal': 'i', 'steps': [_step(FINE), _step(FINE), _step(BAD)]}}
    mission = {'goal': 'g', 'steps': [_step(FINE), {'instruction': 'x', 'include': 'inner'}, _step(FINE)]}
    [problem] = _problems(store, mission, blocks)
    assert problem['path'] == ['steps', 3, 'args', 'to', 'point']  # the expanded plan's index
    assert problem['origin'] == {'step': 1, 'include': 'inner', 'inner_path': ['steps', 2, 'args', 'to', 'point']}


def test_origin_of_nested_includes_names_the_chain_and_the_outermost_step(store):
    blocks = {'inner': {'goal': 'i', 'steps': [_step(FINE), _step(BAD)]},
              'middle': {'goal': 'm', 'steps': [_step(FINE), {'instruction': 'x', 'include': 'inner'}]}}
    mission = {'goal': 'g', 'steps': [_step(FINE), _step(FINE), {'instruction': 'x', 'include': 'middle'}]}
    [problem] = _problems(store, mission, blocks)
    assert problem['origin'] == {'step': 2, 'include': ['middle', 'inner'],
                                 'inner_path': ['steps', 1, 'args', 'to', 'point']}


def test_origin_of_a_problem_from_a_param_placeholder(store):
    blocks = {'inner': {'goal': 'i', 'params': {'where': {}},
                        'steps': [_step(FINE), _step({'to': {'point': '{where}'}})]}}
    mission = {'goal': 'g', 'steps': [{'instruction': 'x', 'include': 'inner', 'with': {'where': 'box_1/nope'}}]}
    [problem] = _problems(store, mission, blocks)
    assert problem['origin'] == {'step': 0, 'include': 'inner', 'inner_path': ['steps', 1, 'args', 'to', 'point']}


def test_a_robot_level_problem_has_no_origin(store):
    plan = Plan.from_mission({'goal': 'g', 'steps': [_step(FINE)]}, 'p', robot='no_such_robot')
    problems = validate_plan_structured(plan, CATALOG, store)
    assert problems and problems[0]['path'] == [] and problems[0]['origin'] is None


def test_origin_does_not_show_in_steps_or_equality():
    from taskweave_core.plan import expand_includes
    expanded = expand_includes({'goal': 'g', 'steps': [{'instruction': 'x', 'skill': 'wait'}]}, lambda n: {})
    assert expanded['steps'][0] == {'instruction': 'x', 'skill': 'wait', 'id': 's1'}
    assert 'origin' not in repr(Plan.from_mission(expanded, 'p').steps[0])
