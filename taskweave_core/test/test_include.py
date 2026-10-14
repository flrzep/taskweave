"""Mission includes: {include: <mission>, with: {...}} becomes that mission's steps with its params filled in."""

import pytest

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_core.plan import IncludeError, Plan, expand_includes

PICK = {'goal': 'pick', 'params': {'frame': {'point': 'instance'}}, 'defaults': {'timeout_s': 30},
        'steps': [{'instruction': 'to {frame}', 'skill': 'navigate_to', 'args': {'target': 'stance:{frame}'}},
                  {'id': 'grasp', 'instruction': 'grasp', 'skill': 'go_to_point',
                   'args': {'point': '{frame}/top', 'offset_m': '{offset}'}, 'replan_from': 'grasp'}]}


def load(name):
    return {'pick': PICK, 'loop_me': LOOP}[name]


LOOP = {'goal': 'loop', 'steps': [{'instruction': 'again', 'include': 'loop_me'}]}


def test_an_include_becomes_the_missions_steps():
    mission = {'goal': 'outer', 'defaults': {'timeout_s': 99}, 'steps': [
        {'instruction': 'wait', 'skill': 'wait', 'args': {'seconds': 1}},
        {'instruction': 'pick it', 'include': 'pick', 'with': {'frame': 'box_1'}, 'retries': 2}]}
    expanded = expand_includes(mission, load)
    assert [s['id'] for s in expanded['steps']] == ['s1', 's2_s1', 's2_grasp']
    second, third = expanded['steps'][1:]
    assert second['instruction'] == 'to box_1' and second['args'] == {'target': 'stance:box_1'}
    assert third['args'] == {'point': 'box_1/top', 'offset_m': '{offset}'}  # not a param: a world-state placeholder
    assert third['replan_from'] == 's2_grasp' and third['retries'] == 2 and third['timeout_s'] == 30
    plan = Plan.from_mission(expanded, 'p')
    assert [s.timeout_s for s in plan.steps] == [99, 30, 30]


def test_bad_includes_are_reported():
    with pytest.raises(IncludeError, match='needs frame'):
        expand_includes({'goal': 'g', 'steps': [{'instruction': 'x', 'include': 'pick'}]}, load)
    with pytest.raises(IncludeError, match='no parameter colour'):
        expand_includes({'goal': 'g', 'steps': [{'instruction': 'x', 'include': 'pick',
                                                 'with': {'frame': 'a', 'colour': 'red'}}]}, load)
    with pytest.raises(IncludeError, match='again'):
        expand_includes(LOOP, load)
    with pytest.raises(IncludeError, match='expand includes first'):
        Plan.from_mission({'goal': 'g', 'steps': [{'instruction': 'x', 'include': 'pick'}]}, 'p')


def test_the_store_expands_mission_files(config_dir):
    missions = config_dir / 'missions'
    missions.mkdir(exist_ok=True)
    (missions / 'part.yaml').write_text('goal: part\nparams: {seconds: {}}\n'
                                        'steps:\n- {instruction: wait, skill: wait, args: {seconds: "{seconds}"}}\n')
    (missions / 'whole.yaml').write_text('goal: whole\nsteps:\n- {instruction: part, include: part, with: {seconds: 2}}\n')
    (missions / 'broken.yaml').write_text('goal: broken\nsteps:\n- {instruction: part, include: nothing}\n')
    store = ConfigStore(config_dir)
    assert store.expanded_mission('whole')['steps'][0]['args'] == {'seconds': 2}  # a whole placeholder keeps the type
    assert 'include' in store.mission('whole')['steps'][0]  # the file itself is unchanged
    with pytest.raises(ConfigError, match="cannot include mission 'nothing'"):
        store.expanded_mission('broken')


def test_a_block_with_params_is_not_run_on_its_own(config_dir):
    from taskweave_core.planner import MissionFilePlanner
    missions = config_dir / 'missions'
    missions.mkdir(exist_ok=True)
    (missions / 'part.yaml').write_text('goal: part\nparams: {seconds: {}}\n'
                                        'steps:\n- {instruction: wait, skill: wait, args: {seconds: "{seconds}"}}\n')
    with pytest.raises(ConfigError, match='a block for other missions to include'):
        MissionFilePlanner(ConfigStore(config_dir), 'part')



def test_points_on_an_object_a_step_puts_down_are_checked_later(config_dir):
    from taskweave_core.validation import validate_plan
    store = ConfigStore(config_dir)
    catalog = {'gripper': {'type': 'object', 'properties': {'place_as': {'type': 'string'}}},
               'go_to_point': {'type': 'object', 'properties': {
                   'point': {'type': 'string', 'x-taskweave-point': 'instance_point'}}}}
    steps = [{'instruction': 'put it down', 'skill': 'gripper', 'args': {'place_as': 'on_floor'}},
             {'instruction': 'to it', 'skill': 'go_to_point', 'args': {'point': 'on_floor/top_grasp'}}]
    assert validate_plan(Plan.from_mission({'goal': 'g', 'steps': steps}, 'p', robot='demo_bot'), catalog, store) == []
    before = Plan.from_mission({'goal': 'g', 'steps': steps[::-1]}, 'p', robot='demo_bot')
    assert any('on_floor' in p for p in validate_plan(before, catalog, store))  # used before it is put down
