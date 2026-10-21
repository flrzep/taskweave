"""Plans with the move skill on a cell of schema 4: the plan validation (one point kind ``target`` with the kinds a
field takes, the options of each kind of motion, the old skill names), the kind and resources every step gets, and the
offline check of the migrated docs example cell (``taskweave check --missions``, ``taskweave describe``)."""

import copy
import shutil
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from taskweave_core import cli  # noqa: E402
from taskweave_core.config import ConfigStore, packs  # noqa: E402
from taskweave_core.plan import Plan, Step  # noqa: E402
from taskweave_core.validation import plan_step_info, validate_plan, validate_plan_structured  # noqa: E402

GOLDEN = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'migrate_targets' / 'docs_cell'
ROBOT = 'duo_bot'


@pytest.fixture
def cell(tmp_path, monkeypatch):
    monkeypatch.setattr(packs, 'installed_location', lambda: None)
    root = tmp_path / 'cell'
    shutil.copytree(GOLDEN, root)
    return root


@pytest.fixture
def store(cell):
    return ConfigStore(cell)


@pytest.fixture
def catalog(store):
    return cli.offline_catalog(store, ROBOT)


def plan_of(*steps) -> Plan:
    """A plan of (skill, args) steps."""
    return Plan('p', 'goal', ROBOT, [Step(f's{i}', f'step {i}', skill, copy.deepcopy(args))
                                      for i, (skill, args) in enumerate(steps, 1)])


def move(**args):
    return ('move', args)


def problems(store, catalog, *steps) -> list[str]:
    return validate_plan(plan_of(*steps), catalog, store)


# ---- the docs example cell, offline ------------------------------------------------------------------------------
def test_the_catalog_is_the_three_skills(catalog):
    assert sorted(catalog) == ['grip', 'move', 'wait']


def test_check_and_describe_pass_on_the_migrated_docs_cell(cell, capsys):
    assert cli.check(str(cell), missions=True) == 0
    out = capsys.readouterr().out
    assert 'skills/move/hand.yaml' in out and 'FAIL' not in out and 'every mission plan validates' in out
    assert cli.describe_offline(str(cell), ROBOT) == 0
    card = capsys.readouterr().out
    import re

    assert re.findall(r'(?m)^- (\w+)\(', card) == ['grip', 'move', 'wait']
    assert 'base targets: ' in card and 'bin_1/pick' in card and 'joint targets: home_pose' in card


def test_every_mission_of_the_docs_cell_validates_with_kinds_and_resources(store, catalog):
    for name in store.mission_names():
        plan = Plan.from_mission(store.expanded_mission(name), name, robot=ROBOT)
        assert validate_plan(plan, catalog, store) == [], name
        for step in plan.steps:
            if step.skill == 'move':
                assert step.kind in ('base', 'straight', 'hand', 'object', 'posture', 'joints', None), (name, step.id)
                assert step.resources, (name, step.id)


# ---- one point kind, with the kinds a field takes ----------------------------------------------------------------
def test_to_takes_any_target_and_an_unknown_one_is_named(store, catalog):
    assert problems(store, catalog, move(to='home'), move(to='bin_1/pick'), move(to='bin_1/grasp'),
                    move(to='home_pose'), move(to='carry')) == []
    [text] = problems(store, catalog, move(to='nowhere'))
    assert text.startswith("step 's1' (move): args.to: ") and 'nowhere' in text


def test_via_takes_hand_and_joint_targets_only(store, catalog):
    ok = move(to='bin_1/grasp', motion={'via': ['present_left', 'home_pose']})
    assert problems(store, catalog, ok) == []
    [text] = problems(store, catalog, move(to='bin_1/grasp', motion={'via': ['home']}))
    assert 'motion.via.0' in text and "'home' moves the base" in text


def test_a_hand_target_is_checked_against_the_robots_tools(store, catalog, cell):
    path = cell / 'robots' / ROBOT / 'targets.yaml'
    path.write_text(path.read_text() + 'odd:\n  moves: hand\n  tool_type: suction\n  left_tool: {xyz: [0.5, 0, 0.5], '
                    'rpy: [0, 0, 0]}\n')
    store.reload()
    [text] = problems(store, catalog, move(to='odd'))
    assert 'does not fit robot' in text and 'suction' in text


def test_a_target_of_a_step_that_puts_the_object_down_is_not_checked_yet(store, catalog):
    steps = [('grip', {'action': 'open', 'place_as': 'dropped'}), move(to='dropped/grasp')]
    assert problems(store, catalog, *steps) == []
    assert problems(store, catalog, move(to='dropped/grasp'))     # not placed before: the name is unknown


# ---- the options of each kind of motion --------------------------------------------------------------------------
@pytest.mark.parametrize('args, fragment', [
    ({'to': 'home', 'hands': 'all'}, "'home' moves the base; option hands does not apply"),
    ({'to': 'home', 'offset_m': 0.1}, "'home' moves the base; option offset_m does not apply"),
    ({'to': 'home', 'torso': 'auto'}, 'torso auto needs a hand or object target'),
    ({'to': 'home', 'motion': {'mode': 'free'}}, "motion.mode 'free' does not apply"),
    ({'to': 'home', 'motion': {'planner': 'planned'}}, 'motion.planner does not apply'),
    ({'to': 'home', 'motion': {'speed_abs': 0.2}}, 'motion.speed_abs does not apply (the base takes motion.speed)'),
    ({'to': 'bin_1/grasp', 'reference': 'seat'}, "moves a hand; option reference does not apply"),
    ({'to': 'bin_1/grasp', 'motion': {'mode': 'nav'}}, "motion.mode 'nav' does not apply"),
    ({'to': 'bin_1/grasp', 'only': ['left_arm']}, 'option only does not apply'),
    ({'to': 'bin_1/seat', 'motion': {'via': ['present_left']}}, 'motion.via does not apply'),
    ({'to': 'bin_1/seat', 'hands': 'any_one'}, 'hands any_one does not apply'),
    ({'to': 'home_pose', 'offset_m': 0.1}, "'home_pose' moves joints; option offset_m does not apply"),
    ({'to': 'home_pose', 'torso': 'keep'}, "'home_pose' moves joints; option torso does not apply"),
    ({'to': 'home_pose', 'motion': {'mode': 'free'}}, 'motion.mode'),
    ({'to': 'home', 'by': [0, 0, 0.1]}, 'option by does not apply'),
    ({'hand': 'left', 'by': [0, 0, 0.1], 'offset_m': 0.1}, 'option offset_m does not apply to a relative move'),
    ({'hand': 'left', 'by': [0, 0, 0.1], 'torso': 0.3}, 'a relative move takes torso auto or keep'),
    ({'base': {'by': [0.5, 0]}, 'torso': 'auto'}, 'torso auto needs a hand or object target'),
    ({'base': {'by': [0.5, 0]}, 'hands': 'all'}, 'move straight: option hands does not apply'),
    ({'elbow': 'out', 'offset_m': 0.1}, 'move elbow: option offset_m does not apply'),
    ({'joints': {'torso': 0.3}, 'only': ['torso']}, 'option only applies to a joint target'),
])
def test_an_option_a_kind_does_not_take_is_a_problem(store, catalog, args, fragment):
    found = problems(store, catalog, move(**args))
    assert found and any(fragment in text for text in found), found


@pytest.mark.parametrize('args', [
    {'to': 'home'}, {'to': 'home', 'torso': 'keep'}, {'to': 'home', 'torso': 0.3},
    {'to': 'home', 'motion': {'mode': 'straight', 'speed': 0.5}},
    {'to': 'bin_1/grasp', 'hands': 'any_one', 'offset_m': 0.1, 'torso': 'auto', 'motion': {'mode': 'linear'}},
    {'to': 'bin_1/seat', 'reference': 'base', 'offset_m': 0.05, 'torso': 'keep'},
    {'to': 'home_pose', 'only': ['left_arm'], 'motion': {'planner': 'planned', 'via': ['home_pose']}},
    {'hand': 'left', 'by': [0, 0, 0.1], 'torso': 'keep'}, {'object': 'held', 'by': [0, 0, 0.1]},
    {'base': {'by': [0.5, 0]}}, {'elbow': 'out', 'tolerance_deg': 5}, {'joints': {'torso': 0.3}},
])
def test_the_options_a_kind_takes_are_fine(store, catalog, args):
    assert problems(store, catalog, move(**args)) == []


def test_a_cell_frame_hand_target_is_refused_before_the_run(store, catalog, cell):
    path = cell / 'cell' / 'targets.yaml'
    path.write_text(path.read_text() + 'dock_tool: {moves: hand, left_tool: {xyz: [1, 0, 1], rpy: [0, 0, 0]}}\n')
    store.reload()
    [text] = problems(store, catalog, move(to='dock_tool'))
    assert 'cell-frame hand targets are not supported yet' in text


def test_a_placeholder_leaves_the_kind_open(store, catalog):
    assert problems(store, catalog, move(to='{where}', hands='all'), move(to='{where}', torso='auto')) == []
    plan = plan_of(move(to='{where}'))
    validate_plan(plan, catalog, store)
    assert plan.steps[0].kind is None and plan.steps[0].resources == ['base', 'hands', 'torso', 'joints']


def test_the_structured_problem_has_the_path_of_the_option(store, catalog):
    [problem] = validate_plan_structured(plan_of(move(to='home', hands='all')), catalog, store)
    assert problem['path'] == ['steps', 0, 'args', 'hands'] and 'option hands does not apply' in problem['message']


# ---- the old skill names -----------------------------------------------------------------------------------------
@pytest.mark.parametrize('skill', ['navigate_to', 'drive_straight', 'move_gripper', 'move_object', 'set_posture',
                                   'move_joints', 'go_to_point', 'move_to_pose'])
def test_an_old_skill_name_on_a_schema_4_cell_says_move(store, catalog, skill):
    [text] = problems(store, catalog, (skill, {'target': 'x'}))
    assert f'the skill {skill} was replaced by move' in text and 'schema 4' in text and 'move {' in text
    assert 'taskweave config migrate' in text


def test_gripper_keeps_its_message_and_grip_and_wait_are_fine(store, catalog):
    [text] = problems(store, catalog, ('gripper', {}))
    assert 'was removed, use grip' in text
    assert problems(store, catalog, ('wait', {'seconds': 1}), ('grip', {'action': 'open'})) == []


def test_a_cell_before_schema_4_keeps_the_old_messages(tmp_path, catalog):
    old = tmp_path / 'old'
    shutil.copytree(Path(__file__).parent / 'fixtures' / 'config', old)
    store = ConfigStore(old, check_schema=False)
    assert not store.targets_format()
    [text] = validate_plan(plan_of(('go_to_point', {'point': 'crate_1/rim'})), {}, store)
    assert 'was removed, use move_gripper' in text


# ---- kind and resources of the steps -----------------------------------------------------------------------------
@pytest.mark.parametrize('args, kind, resources', [
    ({'to': 'home'}, 'base', ['base']),
    ({'to': 'home', 'torso': 0.3}, 'base', ['base', 'torso']),
    ({'to': 'home', 'motion': {'mode': 'straight'}}, 'straight', ['base']),
    ({'base': {'by': [-0.5, 0]}}, 'straight', ['base']),
    ({'base': {'by': [-0.5, 0]}, 'torso': 0.2}, 'straight', ['base', 'torso']),
    ({'to': 'bin_1/pick'}, 'base', ['base']),
    ({'to': 'bin_1/grasp'}, 'hand', ['hands', 'torso']),                    # the torso search is the default
    ({'to': 'bin_1/grasp', 'torso': 'keep'}, 'hand', ['hands']),
    ({'to': 'present_left'}, 'hand', ['hands', 'torso']),
    ({'to': 'present_both'}, 'hand', ['hands', 'torso']),                   # the target sets a height
    ({'to': 'present_both', 'torso': 'keep'}, 'hand', ['hands']),           # the step wins over the target
    ({'hand': 'left', 'by': [0, 0, 0.1]}, 'hand', ['hands', 'torso']),
    ({'hand': 'left', 'by': [0, 0, 0.1], 'torso': 'keep'}, 'hand', ['hands']),
    ({'to': 'carry'}, 'object', ['hands', 'torso']),
    ({'to': 'bin_1/seat', 'torso': 'keep'}, 'object', ['hands']),
    ({'object': 'held', 'by': [0, 0, 0.1], 'torso': 'keep'}, 'object', ['hands']),
    ({'elbow': 'out'}, 'posture', ['hands']),
    ({'to': 'home_pose'}, 'joints', ['hands', 'torso']),                    # the arms are hands
    ({'to': 'home_pose', 'only': ['torso']}, 'joints', ['torso']),
    ({'to': 'home_pose', 'only': ['left_arm']}, 'joints', ['hands']),
    ({'joints': {'torso': 0.3}}, 'joints', ['torso']),
    ({'joints': {'torso': 'keep', 'left_arm': [0.1, None]}}, 'joints', ['hands']),
])
def test_kind_and_resources_of_a_move_step(store, catalog, args, kind, resources):
    plan = plan_of(move(**args))
    assert validate_plan(plan, catalog, store) == []
    assert (plan.steps[0].kind, plan.steps[0].resources) == (kind, resources)
    assert plan_step_info(plan, catalog, store) == [{'id': 's1', 'kind': kind, 'resources': resources}]


def test_other_skills_keep_their_declared_resources_and_no_kind(store, catalog):
    plan = plan_of(('wait', {'seconds': 1}), ('grip', {'action': 'open'}))
    validate_plan(plan, catalog, store)
    declared = [list(catalog[s.skill].get('x-taskweave-resources', [])) for s in plan.steps]
    assert [(s.kind, s.resources) for s in plan.steps] == [(None, resources) for resources in declared]


def test_the_steps_of_an_invalid_plan_still_get_their_resources(store, catalog):
    plan = plan_of(move(to='home', hands='all'), move(to='nowhere'))
    assert validate_plan(plan, catalog, store)
    assert plan.steps[0].resources == ['base']
    assert plan.steps[1].kind is None      # does not resolve: the schema's
