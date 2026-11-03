"""Mission includes: {include: <mission>, with: {...}} becomes that mission's steps with its params filled in."""

import pytest

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_core.plan import IncludeError, Plan, expand_includes

PICK = {'goal': 'pick', 'params': {'frame': {'point': 'instance'}}, 'defaults': {'timeout_s': 30},
        'steps': [{'instruction': 'to {frame}', 'skill': 'navigate_to', 'args': {'target': 'stance:{frame}'}},
                  {'id': 'grasp', 'instruction': 'grasp', 'skill': 'move_gripper',
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
    (missions / 'whole.yaml').write_text(
        'goal: whole\nsteps:\n- {instruction: part, include: part, with: {seconds: 2}}\n')
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
    catalog = {'grip': {'type': 'object', 'properties': {'place_as': {'type': 'string'}}},
               'move': {'type': 'object', 'properties': {
                   'to': {'type': 'string', 'x-taskweave-point': 'target'}}}}
    steps = [{'instruction': 'put it down', 'skill': 'grip', 'args': {'place_as': 'on_floor'}},
             {'instruction': 'to it', 'skill': 'move', 'args': {'to': 'on_floor/top_grasp'}}]
    assert validate_plan(Plan.from_mission({'goal': 'g', 'steps': steps}, 'p', robot='demo_bot'), catalog, store) == []
    before = Plan.from_mission({'goal': 'g', 'steps': steps[::-1]}, 'p', robot='demo_bot')
    assert any('on_floor' in p for p in validate_plan(before, catalog, store))  # used before it is put down



def test_facts_pick_the_fullest_stack_each_cycle_and_end_the_mission(config_dir):
    from taskweave_core.planner import MissionFilePlanner
    (config_dir / 'cell' / 'objects.yaml').write_text("""\
a_front: {object: box, frame: cell, x: 1.0, y: 0.0, yaw: 0.0, stack: {count: 1, pitch: 0.1}}
b_front: {object: box, frame: cell, x: 2.0, y: 0.0, yaw: 0.0, stack: {count: 2, pitch: 0.1}}
c_back: {object: box, frame: cell, x: 3.0, y: 0.0, yaw: 0.0, stack: {count: 9, pitch: 0.1}}
""")
    missions = config_dir / 'missions'
    missions.mkdir(exist_ok=True)
    (missions / 'cycle.yaml').write_text("""\
goal: empty the front stacks
loop: forever
facts: {next: {fullest: box, among: "*_front"}}
steps:
- {instruction: "pick from {next}", skill: wait, args: {target: "stance:{next}"}}
""")
    store = ConfigStore(config_dir)
    planner = MissionFilePlanner(store, 'cycle')
    picked = []
    for cycle in range(1, 6):
        plan = planner.make_plan({}, cycle)
        if plan is None:
            break
        picked.append(plan.steps[0].args['target'])
        store.take_object(plan.steps[0].args['target'].removeprefix('stance:'))
    assert picked == ['stance:b_front', 'stance:a_front', 'stance:b_front']   # fullest first, ties by name
    assert plan is None and cycle == 4                                           # all front stacks empty: done


def test_the_planner_keeps_each_steps_origin(config_dir):
    """The planner fills a mission's facts into its steps: the steps keep where they came from (the plan graph's
    phases)."""
    from taskweave_core.planner import MissionFilePlanner
    missions = config_dir / 'missions'
    missions.mkdir(exist_ok=True)
    (missions / 'rest.yaml').write_text('goal: r\nsteps:\n- {instruction: a, skill: wait, args: {seconds: 1}}\n')
    (missions / 'twice.yaml').write_text('goal: t\nsteps:\n- {instruction: x, skill: wait, args: {seconds: 1}}\n'
                                         '- {instruction: one, include: rest}\n- {instruction: two, include: rest}\n')
    plan = MissionFilePlanner(ConfigStore(config_dir), 'twice').make_plan({}, 1)
    assert [s.origin for s in plan.steps] == [{'step': 0, 'include': [], 'inner_step': 0},
                                              {'step': 1, 'include': ['rest'], 'inner_step': 0},
                                              {'step': 2, 'include': ['rest'], 'inner_step': 0}]


def test_the_plan_message_carries_origin_replan_from_and_max_replans():
    pytest.importorskip('taskweave_interfaces.msg')
    import json

    from taskweave_core.conversions import plan_to_msg
    msg = plan_to_msg(Plan.from_mission(expand_includes(NESTED, load_nested), 'p'))
    if not hasattr(msg.steps[0], 'origin_json'):
        pytest.skip('taskweave_interfaces without origin_json')
    assert [s.step_id for s in msg.steps] == ['s1', 's2_s1', 's2_s2_s1', 's2_s2_g', 's3']
    assert [json.loads(s.origin_json) for s in msg.steps] == [
        {'include': [], 'step': 0, 'top': 0}, {'include': ['mid'], 'step': 0, 'top': 1},
        {'include': ['mid', 'pick'], 'step': 0, 'top': 1}, {'include': ['mid', 'pick'], 'step': 1, 'top': 1},
        {'include': [], 'step': 2, 'top': 2}]
    twice = {'goal': 'twice', 'steps': [{'instruction': 'a', 'include': 'pick', 'with': {'frame': 'f'}},
                                        {'instruction': 'b', 'include': 'pick', 'with': {'frame': 'g'}}]}
    steps = plan_to_msg(Plan.from_mission(expand_includes(twice, load_nested), 'p')).steps
    assert [json.loads(s.origin_json) for s in steps] == [
        {'include': ['pick'], 'step': 0, 'top': 0}, {'include': ['pick'], 'step': 1, 'top': 0},
        {'include': ['pick'], 'step': 0, 'top': 1}, {'include': ['pick'], 'step': 1, 'top': 1}]
    assert [s.replan_from for s in msg.steps] == ['', '', '', 's2_s2_g', '']
    assert [s.max_replans for s in msg.steps] == [1, 1, 3, 3, 1]


NESTED = {'goal': 'outer', 'steps': [
    {'instruction': 'wait', 'skill': 'wait'},
    {'instruction': 'mid', 'include': 'mid', 'with': {'frame': 'f'}},
    {'instruction': 'wait', 'skill': 'wait'}]}
MID = {'goal': 'mid', 'params': {'frame': {}}, 'steps': [
    {'instruction': 'wait', 'skill': 'wait'}, {'instruction': 'pick', 'include': 'pick', 'with': {'frame': '{frame}'}}]}
PICK3 = {'goal': 'pick', 'params': {'frame': {}}, 'defaults': {'max_replans': 3}, 'steps': [
    {'instruction': 'to', 'skill': 'wait'}, {'id': 'g', 'instruction': 'grasp', 'skill': 'wait', 'replan_from': 'g'}]}


def load_nested(name):
    return {'mid': MID, 'pick': PICK3}[name]


def test_the_step_message_carries_the_validation_notes():
    pytest.importorskip('taskweave_interfaces.msg')
    from taskweave_core.conversions import step_to_msg
    from taskweave_core.plan import Step
    msg = step_to_msg(Step(id='a', instruction='i', skill='wait', notes=['note one', 'note two']))
    if not hasattr(msg, 'notes'):
        pytest.skip('taskweave_interfaces without notes')
    assert list(msg.notes) == ['note one', 'note two']
    assert list(step_to_msg(Step(id='b', instruction='i', skill='wait')).notes) == []
