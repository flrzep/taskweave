import pytest

from taskweave_core import placeholders
from taskweave_core.config import ConfigStore
from taskweave_core.plan import Plan
from taskweave_core.planner import MissionFilePlanner
from taskweave_core.validation import validate_plan

CATALOG = {
    'navigate_to': {
        'type': 'object',
        'required': ['target'],
        'properties': {'target': {'type': 'string', 'x-taskweave-point': 'base_target'}},
        'additionalProperties': False,
    },
    'wait': {
        'type': 'object',
        'required': ['seconds'],
        'properties': {'seconds': {'type': 'number', 'minimum': 0}},
        'additionalProperties': False,
    },
    'pick': {
        'type': 'object',
        'required': ['grasp', 'count'],
        'properties': {'grasp': {'type': 'string', 'x-taskweave-point': 'object_point'}, 'count': {'type': 'integer'}},
    },
}


def plan_from(store, mission='patrol'):
    return Plan.from_mission(store.mission(mission), 'test-0001')


def test_defaults_are_applied_to_every_step(config_dir):
    plan = plan_from(ConfigStore(config_dir))
    assert [s.id for s in plan.steps] == ['to_shelf', 's2', 's3']
    assert all(s.timeout_s == 60 for s in plan.steps)
    assert plan.steps[1].on_failure == 'skip'
    assert plan.steps[0].on_failure == 'abort'
    assert plan.steps[0].retries == 1


def test_placeholders_keep_types_and_format_strings():
    world = {'bay': 2, 'stack': 'bay_2_stack_1'}
    assert placeholders.resolve({'n': '{bay}', 't': 'stance:{stack}', 'l': ['{bay}']}, world) == \
        {'n': 2, 't': 'stance:bay_2_stack_1', 'l': [2]}
    assert placeholders.names({'a': ['{x}', 'y:{z}'], 'b': 3}) == {'x', 'z'}
    with pytest.raises(placeholders.PlaceholderError, match='missing'):
        placeholders.resolve('{missing}', world)


def test_valid_plan_has_no_problems(config_dir):
    store = ConfigStore(config_dir)
    assert validate_plan(plan_from(store), CATALOG, store) == []


def test_validation_reports_every_problem_before_start(config_dir):
    store = ConfigStore(config_dir)
    plan = plan_from(store)
    plan.steps[0].args = {'target': 'stance:nowhere'}
    plan.steps[1].args = {'seconds': -1}
    plan.steps[2].skill = 'teleport'
    plan.steps[2].replan_from = 'ghost'
    problems = validate_plan(plan, CATALOG, store)
    assert any("unknown stance 'nowhere'" in p for p in problems)
    assert any('args.seconds' in p and 'minimum' in p for p in problems)
    assert any('unknown skill' in p for p in problems)
    assert any("replan_from 'ghost'" in p for p in problems)


def test_placeholder_args_are_checked_later(config_dir):
    store = ConfigStore(config_dir)
    plan = plan_from(store)
    plan.steps[0].args = {'target': 'stance:{next_stance}'}
    assert validate_plan(plan, CATALOG, store) == []


def test_object_point_references(config_dir):
    store = ConfigStore(config_dir)
    plan = plan_from(store)
    plan.steps[1].skill, plan.steps[1].args = 'pick', {'grasp': 'box.top_grasp', 'count': '{n}'}
    assert validate_plan(plan, CATALOG, store) == []
    plan.steps[1].args = {'grasp': 'box.side_grasp', 'count': 1}
    assert any("no point 'side_grasp'" in p for p in validate_plan(plan, CATALOG, store))


def test_unknown_robot_is_reported(config_dir):
    store = ConfigStore(config_dir)
    plan = plan_from(store)
    plan.robot = 'other_bot'
    assert any("robot 'other_bot'" in p for p in validate_plan(plan, CATALOG, store))


def test_mission_planner_loop_rules(config_dir):
    store = ConfigStore(config_dir)
    once = MissionFilePlanner(store, 'patrol')
    assert once.make_plan({}, 1).plan_id == 'patrol-0001'
    assert once.make_plan({}, 2) is None

    once.mission = {**once.mission, 'loop': 'forever', 'max_cycles': 2, 'stop_when': 'done'}
    assert once.make_plan({}, 2) is not None
    assert once.make_plan({}, 3) is None
    assert once.make_plan({'done': True}, 2) is None


def test_mission_planner_replans_from_named_step(config_dir):
    store = ConfigStore(config_dir)
    planner = MissionFilePlanner(store, 'patrol', goal='custom goal')
    plan = planner.make_plan({}, 1)
    assert plan.goal == 'custom goal'
    assert planner.replan(plan, 2, 'stuck', {}) == (plan, 2)
    plan.steps[2].replan_from = 'to_shelf'
    assert planner.replan(plan, 2, 'stuck', {}) == (plan, 0)
