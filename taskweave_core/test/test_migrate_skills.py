"""Cell pack migration 2 -> 3 (config/migrate_skills.py): the old skills become the new ones, rule by rule."""

import os
import shutil
from pathlib import Path

import pytest
import yaml

from taskweave_core.config import ConfigError, ConfigStore, packs
from taskweave_core.config import migrate_skills as ms
from taskweave_core.config.migrate import migrate, migrate_skills_only
from taskweave_core.config.skill_templates import TEMPLATES
from taskweave_core.plan import Plan
from taskweave_core.validation import validate_plan

pytest.importorskip('ruamel.yaml')

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
ROBOT = 'demo_bot'
POSES = """\
# poses of the demo bot
home: {lift: 0.0, j1: 0.0, j2: 0.0}            # joints only
reach:                                          # tools only
  tools:
    left: {tcp: [0.6, 0.2, 0.4], approach: [1, 0, 0], closing: [0, 1, 0]}
  elbow: up
grasp_ready:                                    # joints and tools
  joints: {lift: 0.3, j1: 0.5}
  tools:
    left: {tcp: [0.5, 0.1, 0.3], approach: [1, 0, 0], closing: [0, 1, 0]}
"""
PROFILE_JOINTS = """\
joints:
  command: /joint_command
  state: /joint_states
  groups: {torso: [lift], left_arm: [j1, j2]}
"""
MISSION = """\
# A block with every old skill.
goal: Old skills
params:
  frame: {point: instance, description: the placed frame}
  where: {point: tool_pose}
  rest: {point: joint_pose}
loop: once
defaults: {timeout_s: 300, retries: 0, on_failure: abort}   # sim time
steps:
- instruction: Fold the arms     # a comment on the step
  skill: move_to_pose
  args: {pose: home}   # joints only
- id: reach
  instruction: Reach out
  skill: move_to_pose
  args: {pose: reach}
- instruction: Ready the torso and reach
  skill: move_to_pose
  args: {pose: grasp_ready}
  timeout_s: 90
- instruction: After the split, no id
  skill: wait
  args: {seconds: 1}
- instruction: Reach to a parameter
  skill: move_to_pose
  args: {pose: "{where}"}
- instruction: Rest at a parameter
  skill: move_to_pose
  args: {pose: "{rest}"}
- instruction: Open
  skill: gripper
  args: {side: both, action: open}
- instruction: Pre-grasp
  skill: go_to_point
  args: {point: "{frame}/top", offset_m: 0.15}
- instruction: One hand, straight
  skill: go_to_point
  args: {point: "{frame}/top", tools: [left], linear: true, approach_m: 0.05}
- instruction: Close
  skill: gripper
  args: {side: left, action: close}
- id: lift
  instruction: Lift
  skill: move_object
  args: {by: [0, 0, 0.3]}
  replan_from: reach
- instruction: Elbows
  skill: move_object
  args: {elbow: down}
- instruction: Onto the hook
  skill: move_object
  args: {to: hanger/hook, contact: plate, offset_m: 0.1}
- instruction: Carry
  skill: move_object
  args: {pose: carry}
- instruction: Already new
  skill: move_object
  args: {to: {by: [0, 0, 0.1]}}
- instruction: Drive
  skill: navigate_to
  args: {target: "stance:{frame}"}
"""
OUTER = """\
goal: Outer
steps:
- instruction: pick
  include: block
  with: {frame: box_1, where: reach, rest: home}
  retries: 2
- id: after
  instruction: again
  skill: move_to_pose
  args: {pose: grasp_ready}
  replan_from: s1_reach
- instruction: replan into the split step
  skill: wait
  args: {seconds: 1}
  replan_from: after
"""
OPERATOR = """\
joy:
  topic: /joy
  actions:
    - {button: 0,  skill: gripper,      args: {side: both, action: close}, arm_args: {side: '{arm}'}}   # cross
    - {button: 11, skill: move_to_pose, args: {pose: home}}                                             # d-pad up
"""
OLD_SKILLS = {
    'go_to_point': """\
implementations:
  - name: ik_planned
    type: script
    class: taskweave_skills.joints:GoToPoint
    requires: {capabilities: [arms.bimanual]}
cell_frame: world
max_joint_speed: 0.9            # rad/s, slower than the default
max_linear_speed: 0.2
position_tolerance: 0.04
stall_timeout_s: 6.0
lift_joint: lift
lift_values: [0.0, 0.1]
approach_m: 0.12
tcp_speed: 0.03
tcp_turn: 0.2
state_timeout_s: 7.0
""",
    'gripper': """\
implementations:
  - {name: joint_targets, type: script, class: "taskweave_skills.joints:Gripper"}
max_joint_speed: 1.1
position_tolerance: 0.02
stall_timeout_s: 2.0
state_timeout_s: 4.0
""",
    'move_object': """\
implementations:
  - {name: ik_linear, type: script, class: "taskweave_skills.joints:MoveObject"}
cell_frame: world
max_joint_speed: 1.3
max_linear_speed: 0.3
position_tolerance: 0.05
stall_timeout_s: 5.5
tcp_speed: 0.04
tcp_turn: 0.25
state_timeout_s: 6.0
elbow_tolerance_deg: 15.0
""",
    'move_to_pose': """\
implementations:
  - {name: joint_targets, type: script, class: "taskweave_skills.joints:MoveToPose"}
max_joint_speed: 0.8
max_linear_speed: 0.15
position_tolerance: 0.01
stall_timeout_s: 4.0
state_timeout_s: 3.0
""",
}


@pytest.fixture(autouse=True)
def no_outside_packs(monkeypatch):
    monkeypatch.delenv(packs.ROBOT_PACKS_ENV, raising=False)
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


@pytest.fixture
def cell(tmp_path):
    """A schema 2 cell pack with old skills in missions, operator.yaml and skills/."""
    root = tmp_path / 'cell'
    shutil.copytree(FIXTURE, root)
    for mission in (root / 'missions').glob('*.yaml'):
        mission.unlink()
    for skill in ('fly', 'pick'):
        (root / 'skills' / f'{skill}.yaml').unlink()
    (root / 'pack.yaml').write_text('name: cell\nversion: 1\nschema_version: 2\n')
    (root / 'system.yaml').write_text('robot: demo_bot\n')
    robot = root / 'robots' / ROBOT
    (robot / 'poses.yaml').write_text(POSES)
    (robot / 'profile.yaml').write_text((robot / 'profile.yaml').read_text() + PROFILE_JOINTS)
    (root / 'missions' / 'block.yaml').write_text(MISSION)
    (root / 'missions' / 'outer.yaml').write_text(OUTER)
    (root / 'operator.yaml').write_text(OPERATOR)
    for name, text in OLD_SKILLS.items():
        (root / 'skills' / f'{name}.yaml').write_text(text)
    return root


def run(cell, dry_run=False):
    lines = []
    assert migrate_skills_only(cell, dry_run, lines.append) == 0
    return lines


def steps(cell, name='block'):
    return yaml.safe_load((cell / 'missions' / f'{name}.yaml').read_text())['steps']


def converter(cell):
    return ms.Converter(ms.PoseBook(cell, lambda line: None))


def converted(cell, skill, args, kinds=None):
    result = converter(cell).convert(skill, args, ROBOT, kinds)
    return None if result is None else ([(s, ms._plain(a)) for s, a in result.steps], result.flags)


# ---- rule by rule -----------------------------------------------------------------------------------------------
@pytest.mark.parametrize('skill, args, expected', [
    # move_to_pose
    ('move_to_pose', {'pose': 'home'}, [('move_joints', {'pose': 'home'})]),
    ('move_to_pose', {'pose': 'reach'}, [('move_gripper', {'to': {'pose': 'reach'}, 'hands': ['left'],
                                                             'motion': {'elbow': 'up'}})]),
    ('move_to_pose', {'pose': 'grasp_ready'}, [('move_joints', {'pose': 'grasp_ready_joints'}),
                                               ('move_gripper', {'to': {'pose': 'grasp_ready'}, 'hands': ['left']})]),
    # go_to_point
    ('go_to_point', {'point': 'a/top'}, [('move_gripper', {'to': {'point': 'a/top'}})]),
    ('go_to_point', {'point': 'a/top', 'offset_m': 0.15}, [('move_gripper', {'to': {'point': 'a/top',
                                                                                    'offset_m': 0.15}})]),
    ('go_to_point', {'point': 'a/top', 'tools': ['right'], 'approach_m': 0.0},
     [('move_gripper', {'to': {'point': 'a/top'}, 'hands': ['right'], 'motion': {'approach_m': 0.0}})]),
    ('go_to_point', {'point': 'a/top', 'offset_m': 0.15, 'linear': True},
     [('move_gripper', {'to': {'point': 'a/top', 'offset_m': 0.15}, 'motion': {'mode': 'linear'}})]),
    ('go_to_point', {'point': 'a/top', 'linear': False}, [('move_gripper', {'to': {'point': 'a/top'}})]),
    # move_object
    ('move_object', {'by': [0, 0, 0.1], 'turn_deg': 5, 'tilt_deg': 3},
     [('move_object', {'to': {'by': [0, 0, 0.1], 'turn_deg': 5, 'tilt_deg': 3}})]),
    ('move_object', {'at': [1, 0, 0.5]}, [('move_object', {'to': {'at': [1, 0, 0.5]}})]),
    ('move_object', {'pose': 'carry'}, [('move_object', {'to': {'pose': 'carry'}})]),
    ('move_object', {'to': 'h/hook', 'contact': 'plate', 'offset_m': 0.1},
     [('move_object', {'to': {'onto': 'h/hook', 'contact': 'plate', 'offset_m': 0.1}})]),
    ('move_object', {'elbow': 'down'}, [('set_posture', {'elbow': 'down'})]),
    # gripper
    ('gripper', {'side': 'both', 'action': 'close'}, [('grip', {'hands': 'all', 'action': 'close'})]),
    ('gripper', {'side': 'right', 'action': 'open', 'place_as': 'x'},
     [('grip', {'hands': ['right'], 'action': 'open', 'place_as': 'x'})]),
    ('gripper', {'side': '{arm}', 'action': 'open'}, [('grip', {'hands': '{arm}', 'action': 'open'})]),
])
def test_every_old_step_form_becomes_its_new_form(cell, skill, args, expected):
    steps_, _ = converted(cell, skill, args)
    assert steps_ == expected


@pytest.mark.parametrize('skill, args', [
    ('move_object', {'to': {'by': [0, 0, 0.1]}}), ('move_object', {'to': {'pose': 'carry'}, 'hands': 'all'}),
    ('move_object', {'hands': 'all'}), ('navigate_to', {'target': 'a'}), ('wait', {'seconds': 1}),
    ('grip', {'hands': 'all', 'action': 'open'}), ('move_gripper', {'to': {'point': 'a/top'}}),
    ('move_joints', {'pose': 'home'}), ('set_posture', {'elbow': 'up'})])
def test_new_and_unchanged_steps_are_left_alone(cell, skill, args):
    assert converted(cell, skill, args) is None


def test_placeholders_survive_and_a_pose_parameter_follows_its_point_kind(cell):
    assert converted(cell, 'go_to_point', {'point': '{frame}/rails', 'offset_m': '{d}'})[0] == [
        ('move_gripper', {'to': {'point': '{frame}/rails', 'offset_m': '{d}'}})]
    assert converted(cell, 'move_object', {'by': '{offset}'})[0] == [('move_object', {'to': {'by': '{offset}'}})]
    (joint, flags), (tool, _) = (converted(cell, 'move_to_pose', {'pose': '{p}'}, {'p': 'joint_pose'}),
                                 converted(cell, 'move_to_pose', {'pose': '{p}'}, {'p': 'tool_pose'}))
    assert joint == [('move_joints', {'pose': '{p}'})] and tool == [('move_gripper', {'to': {'pose': '{p}'}})]
    assert any('placeholder' in flag for flag in flags)


def test_flags_for_what_does_not_map_cleanly(cell):
    _, flags = converted(cell, 'go_to_point', {'point': 'a/top', 'linear': True, 'approach_m': 0.05})
    assert converted(cell, 'go_to_point', {'point': 'a/top', 'linear': True, 'approach_m': 0.05})[0] == [
        ('move_gripper', {'to': {'point': 'a/top'}, 'motion': {'mode': 'linear'}})]
    assert any('approach_m 0.05 dropped' in flag for flag in flags)
    steps_, flags = converted(cell, 'move_to_pose', {'pose': 'nowhere'})
    assert steps_ == [('move_joints', {'pose': 'nowhere'})] and any('not in the poses' in f for f in flags)
    steps_, flags = converted(cell, 'gripper', {'side': 'both', 'action': 'open', 'speed': 1})
    assert steps_[0][1]['speed'] == 1 and any('unknown old args' in f for f in flags)


# ---- the files --------------------------------------------------------------------------------------------------
def test_missions_are_rewritten_with_comments_order_ids_and_references_kept(cell):
    lines = run(cell)
    text = (cell / 'missions' / 'block.yaml').read_text()
    assert text.startswith('# A block with every old skill.\ngoal: Old skills\nparams:')
    assert 'defaults: {timeout_s: 300, retries: 0, on_failure: abort}   # sim time' in text
    assert '- instruction: Fold the arms     # a comment on the step\n  skill: move_joints\n' \
           '  args: {pose: home}   # joints only' in text
    assert 'args: {pose: "{where}"}' not in text and 'to: {pose: "{where}"}' in text
    assert 'args: {to: {point: "{frame}/top", offset_m: 0.15}}' in text and 'stance:{frame}' in text
    mission = steps(cell)
    assert [s.get('id') for s in mission[:6]] == [None, 'reach', 's3_joints', 's3_tools', 's4', 's5']
    joints, tools = mission[2], mission[3]
    assert joints['skill'] == 'move_joints' and joints['args'] == {'pose': 'grasp_ready_joints'}
    assert tools['skill'] == 'move_gripper' and tools['timeout_s'] == 90 and joints['timeout_s'] == 90
    assert tools['instruction'] == 'Ready the torso and reach'
    assert [s['skill'] for s in mission if 'skill' in s].count('move_to_pose') == 0
    skills = [s.get('skill') for s in mission]
    assert 'gripper' not in skills and 'go_to_point' not in skills
    assert mission[11]['id'] == 'lift' and mission[11]['replan_from'] == 'reach'
    assert any(line.startswith('step    block reach: move_to_pose') for line in lines)
    assert any(line.startswith('FLAG    block s3: pose grasp_ready has joints and tools') for line in lines)


def test_a_split_step_keeps_the_ids_of_later_steps_and_the_references_to_it(cell):
    run(cell)
    block, outer = steps(cell), steps(cell, 'outer')
    assert block[4]['id'] == 's4' and block[5]['id'] == 's5'          # numbered by position before the split
    assert [s.get('id') for s in outer] == [None, 'after_joints', 'after_tools', 's3'] and \
        outer[3]['replan_from'] == 'after_joints'
    assert outer[1]['args'] == {'pose': 'grasp_ready_joints'} and outer[2]['replan_from'] == 's1_reach'
    assert outer[0]['include'] == 'block' and outer[0]['with'] == {'frame': 'box_1', 'where': 'reach', 'rest': 'home'}


def test_a_reference_into_an_included_split_step_follows_it(cell):
    (cell / 'missions' / 'outer.yaml').write_text(OUTER.replace('replan_from: s1_reach', 'replan_from: s1_s3'))
    run(cell)
    assert steps(cell, 'outer')[2]['replan_from'] == 's1_s3_joints'


def test_the_migrated_missions_load_as_plans_and_the_old_names_are_gone(cell):
    run(cell)
    store = ConfigStore(cell)
    for name in store.mission_names():
        plan = Plan.from_mission(store.expanded_mission(name), name, robot=ROBOT)
        assert not {step.skill for step in plan.steps} & set(ms.OLD_SKILLS)
    assert [step.id for step in Plan.from_mission(store.expanded_mission('outer'), 'outer', robot=ROBOT).steps][-6:] == [
        's1_s14', 's1_s15', 's1_s16', 'after_joints', 'after_tools', 's3']


def test_nothing_held_is_flagged_and_a_block_without_a_reach_is_not(cell):
    (cell / 'missions' / 'point.yaml').write_text("""\
goal: reach, then move a held object that is not held
steps:
- {instruction: reach, skill: go_to_point, args: {point: a/top}}
- {instruction: tuck, skill: move_object, args: {by: [0, 0, 1]}}
""")
    (cell / 'missions' / 'held.yaml').write_text("""\
goal: starts holding something
steps:
- {instruction: down, skill: move_to_pose, args: {pose: home}}
- {instruction: lower, skill: move_object, args: {by: [0, 0, -0.2]}}
""")
    lines = run(cell)
    assert {line for line in lines if line.startswith('FLAG') and 'nothing is held' in line} == {
        'FLAG    point s2: nothing is held at this step (the mission closes no gripper before it): it fails with '
        '"nothing is held" as before; fix it by hand'}


def test_operator_actions_are_rewritten_with_their_arm_args(cell):
    run(cell)
    text = (cell / 'operator.yaml').read_text()
    actions = yaml.safe_load(text)['joy']['actions']
    assert actions == [{'button': 0, 'skill': 'grip', 'args': {'hands': 'all', 'action': 'close'},
                        'arm_args': {'hands': '{arm}'}}, {'button': 11, 'skill': 'move_joints', 'args': {'pose': 'home'}}]
    assert text.count('\n    - {button:') == 2 and '# cross' in text and '# d-pad up' in text and 'topic: /joy' in text


def test_skill_yamls_carry_the_cells_values_and_the_old_files_go(cell):
    run(cell)
    skills = cell / 'skills'
    assert sorted(p.stem for p in skills.glob('*.yaml')) == sorted(
        ['grip', 'move_gripper', 'move_joints', 'move_object', 'navigate_to', 'set_posture', 'wait'])
    gripper = yaml.safe_load((skills / 'move_gripper.yaml').read_text())
    assert gripper['speed'] == {'joint_rad_s': 0.9, 'joint_linear_m_s': 0.2, 'tcp_m_s': 0.03, 'tcp_rad_s': 0.2}
    assert gripper['tolerances'] == {'position': 0.04, 'settle_s': 0.3, 'stall_timeout_s': 6.0, 'state_timeout_s': 7.0}
    assert gripper['motion'] == {'mode': 'free', 'planner': 'auto', 'approach_m': 0.12, 'torso': 'auto'}
    assert gripper['torso_fallback'] == {'joint': 'lift', 'values': [0.0, 0.1]}
    assert [i['class'] for i in gripper['implementations']] == ['taskweave_skills.gripper_motion:MoveGripper'] * 2
    assert '# rad/s, slower than the default' in (skills / 'move_gripper.yaml').read_text()   # its comment goes along
    grip = yaml.safe_load((skills / 'grip.yaml').read_text())
    assert grip['speed'] == {'joint_rad_s': 1.1} and grip['tolerances']['stall_timeout_s'] == 2.0
    assert grip['tolerances']['position'] == 0.02 and grip['tolerances']['state_timeout_s'] == 4.0
    joints = yaml.safe_load((skills / 'move_joints.yaml').read_text())
    assert joints['speed'] == {'joint_rad_s': 0.8, 'joint_linear_m_s': 0.15} and joints['tolerances']['position'] == 0.01
    assert joints['implementations'][0]['class'] == 'taskweave_skills.joint_motion:MoveJoints'
    move_object = yaml.safe_load((skills / 'move_object.yaml').read_text())
    assert move_object['speed'] == {'joint_rad_s': 1.3, 'joint_linear_m_s': 0.3, 'tcp_m_s': 0.04, 'tcp_rad_s': 0.25}
    assert [i['class'] for i in move_object['implementations']] == ['taskweave_skills.object_motion:MoveObject'] * 2
    posture = yaml.safe_load((skills / 'set_posture.yaml').read_text())
    assert posture['speed'] == {'joint_rad_s': 1.3} and posture['elbow_tolerance_deg'] == 15.0
    assert posture['tolerances']['stall_timeout_s'] == 5.5 and 'tcp_m_s' not in posture['speed']
    for name in ('move_gripper', 'grip', 'move_joints', 'move_object', 'set_posture'):   # the store reads them
        ConfigStore(cell).skill_config(name)


def test_a_new_skill_yaml_that_exists_is_kept(cell):
    (cell / 'skills' / 'grip.yaml').write_text('implementations:\n  - {name: mine, type: script, class: "a:B"}\n')
    lines = run(cell)
    assert 'mine' in (cell / 'skills' / 'grip.yaml').read_text() and (cell / 'skills' / 'gripper.yaml').exists()    # kept too, see below
    assert any('skills/grip.yaml  exists already' in line for line in lines)


def test_a_pose_with_joints_and_tools_gets_a_joint_twin_and_keeps_its_comments(cell):
    lines = run(cell)
    text = (cell / 'robots' / ROBOT / 'poses.yaml').read_text()
    assert text.startswith('# poses of the demo bot\n') and '# joints only' in text and '# tools only' in text
    poses = yaml.safe_load(text)
    assert poses['grasp_ready_joints'] == {'lift': 0.3} and 'tools' in poses['grasp_ready']   # j1 is an arm joint
    assert poses['grasp_ready']['joints'] == {'lift': 0.3, 'j1': 0.5} and list(poses)[-1] == 'grasp_ready_joints'
    assert any('grasp_ready has joints and tools: added grasp_ready_joints' in line for line in lines)
    store = ConfigStore(cell)
    assert store.named_pose(ROBOT, 'grasp_ready_joints')['tools'] == {}


def test_a_profile_without_groups_keeps_every_joint_and_says_so(cell):
    (cell / 'robots' / ROBOT / 'profile.yaml').write_text(
        (cell / 'robots' / ROBOT / 'profile.yaml').read_text().replace(PROFILE_JOINTS, ''))
    lines = run(cell)
    poses = yaml.safe_load((cell / 'robots' / ROBOT / 'poses.yaml').read_text())
    assert poses['grasp_ready_joints'] == {'lift': 0.3, 'j1': 0.5}
    assert any(line.startswith('FLAG') and 'keeps every joint' in line for line in lines)


def test_comments_naming_old_skills_are_listed_not_edited(cell):
    (cell / 'objects' / 'box.yaml').write_text('# tested with go_to_point\n' + (cell / 'objects' / 'box.yaml').read_text())
    lines = run(cell)
    assert any(line.startswith('comment objects/box.yaml:1: # tested with go_to_point') for line in lines)
    assert (cell / 'objects' / 'box.yaml').read_text().startswith('# tested with go_to_point')


# ---- the command: dry run, idempotency, the schema chain --------------------------------------------------------------
def tree(root):
    return {str(p.relative_to(root)): p.read_bytes() for p in sorted(root.rglob('*')) if p.is_file()}


def test_dry_run_prints_everything_and_changes_nothing(cell):
    before = tree(cell)
    lines = run(cell, dry_run=True)
    assert tree(cell) == before
    assert any('(dry run: nothing is changed)' in line for line in lines)
    assert any(line.startswith('would write') and 'pack.yaml' in line for line in lines)
    assert any(line.startswith('would write') and 'flagged step' in line for line in lines)


def test_the_migration_is_idempotent_and_bumps_the_schema(cell):
    run(cell)
    assert packs.cell_schema(cell) == 3
    after = tree(cell)
    again = []
    assert migrate_skills_only(cell, False, again.append) == 0
    assert again == [f'{cell.resolve()}: cell pack schema 3, nothing to do'] and tree(cell) == after


def test_converting_migrated_steps_again_changes_nothing(cell):
    run(cell)
    converter_ = converter(cell)
    for step in steps(cell) + steps(cell, 'outer'):
        if 'skill' in step:
            assert converter_.convert(step['skill'], step.get('args'), ROBOT) is None


def test_a_schema_1_cell_is_sent_to_migrate_and_migrate_runs_the_whole_chain(cell, tmp_path):
    (cell / 'pack.yaml').unlink()
    lines = []
    assert migrate_skills_only(cell, False, lines.append) == 2 and 'run taskweave config migrate' in lines[0]
    lines = []
    assert migrate(cell, tmp_path / 'robot_packs', say=lines.append) == 0
    assert packs.cell_schema(cell) == 3
    assert [line for line in lines if line.startswith('write   pack.yaml')] == [
        'write   pack.yaml  (schema_version 2)', 'write   pack.yaml  (schema_version 3)']
    assert not (cell / 'skills' / 'go_to_point.yaml').exists() and steps(cell)[0]['skill'] == 'move_joints'


def test_the_store_refuses_schema_2_and_names_the_command(cell):
    (cell / 'pack.yaml').write_text('name: cell\nversion: 1\nschema_version: 2\n')
    with pytest.raises(ConfigError, match='run taskweave config migrate-skills'):
        ConfigStore(cell)
    assert ConfigStore(cell, check_schema=False).mission_names()


def test_an_unwritable_file_stops_before_anything_is_touched(cell, monkeypatch):
    before = tree(cell)
    blocked = cell / 'operator.yaml'
    real = ms.os.access
    monkeypatch.setattr(ms.os, 'access', lambda path, mode: False if Path(path) == blocked else real(path, mode))
    lines = []
    assert migrate_skills_only(cell, False, lines.append) == 2
    assert 'operator.yaml is not writable' in lines[-1] and tree(cell) == before


def test_a_placeholder_to_is_new_form_unless_a_contact_is_given(cell):
    assert converted(cell, 'move_object', {'to': '{target}'}) is None
    assert converted(cell, 'move_object', {'to': '{target}', 'contact': 'plate'})[0] == [
        ('move_object', {'to': {'onto': '{target}', 'contact': 'plate'}})]
    assert not ms.is_old_move_object({'to': '{target}', 'hands': 'all'}) and ms.is_old_move_object({'to': 'h/hook'})
    run(cell)
    store = ConfigStore(cell)
    plan = Plan.from_mission({'goal': 'g', 'steps': [{'instruction': 'i', 'skill': 'move_object',
                                                       'args': {'to': '{target}'}}]}, 'p', robot=ROBOT)
    problems = validate_plan(plan, {'move_object': {'type': 'object'}}, store)
    assert not any('migrate-skills' in problem for problem in problems), problems


def test_an_old_skill_yaml_is_kept_and_flagged_when_the_new_one_exists(cell):
    (cell / 'skills' / 'grip.yaml').write_text('implementations:\n  - {name: mine, type: script, class: "a:B"}\n')
    lines = run(cell)
    assert (cell / 'skills' / 'gripper.yaml').is_file()                      # its values were not carried: kept
    assert any(line.startswith('FLAG') and 'gripper.yaml' in line and 'grip.yaml' in line for line in lines)
    assert not any(line.startswith('remove  skills/gripper.yaml') for line in lines)
    assert not (cell / 'skills' / 'go_to_point.yaml').exists()               # the others migrated as before
    run_again = []
    assert migrate_skills_only(cell, False, run_again.append) == 0 and run_again[0].endswith('nothing to do')


def test_an_unwritable_pack_yaml_stops_everything_with_a_clear_message(cell, monkeypatch):
    before = tree(cell)
    blocked = cell / 'pack.yaml'
    real = os.access
    monkeypatch.setattr(os, 'access', lambda path, mode, **kw: False if Path(path) == blocked else real(path, mode, **kw))
    lines = []
    assert migrate(cell, None, None, False, lines.append) == 1
    assert any('pack.yaml is not writable' in line and 'nothing was changed' in line for line in lines), lines
    assert tree(cell) == before


def test_an_os_error_while_writing_is_reported_not_raised(cell, monkeypatch):
    def broken(source, target):
        raise PermissionError(13, 'Permission denied', str(target))
    monkeypatch.setattr(ms.os, 'replace', broken)
    lines = []
    assert migrate(cell, None, None, False, lines.append) == 1
    assert any('Permission denied' in line and 'migrate' in line for line in lines), lines


# ---- the old names in validation -----------------------------------------------------------------------------------
@pytest.mark.parametrize('skill, args, text', [
    ('move_to_pose', {'pose': 'home'}, 'the skill move_to_pose was removed, use move_joints'),
    ('go_to_point', {'point': 'a/b'}, 'the skill go_to_point was removed, use move_gripper'),
    ('gripper', {'side': 'both', 'action': 'open'}, 'the skill gripper was removed, use grip'),
    ('move_object', {'by': [0, 0, 1]}, 'move_object takes its target as `to` now'),
    ('move_object', {'elbow': 'up'}, 'the elbow argument is the skill set_posture')])
def test_validation_names_the_migration_command_for_an_old_skill(cell, skill, args, text):
    run(cell)
    store = ConfigStore(cell)
    catalog = {skill: {'type': 'object'}}     # even a catalog that still offers it
    plan = Plan.from_mission({'goal': 'g', 'steps': [{'instruction': 'i', 'skill': skill, 'args': args}]}, 'p',
                             robot=ROBOT)
    problems = validate_plan(plan, catalog, store)
    assert len(problems) == 1 and text in problems[0]
    assert f'run `taskweave config migrate-skills {store.root}`' in problems[0]


def test_a_removed_implementation_class_names_the_command():
    assert 'run `taskweave config migrate-skills' in ms.removed_class_message('taskweave_skills.joints', 'MoveToPose')
    assert ms.removed_class_message('taskweave_skills.gripper_motion', 'Grip') is None


# ---- the templates are the docs examples ---------------------------------------------------------------------------------
def test_the_templates_are_the_docs_examples():
    docs = Path(__file__).resolve().parents[2] / 'docs' / 'examples' / 'skills'
    if not docs.is_dir():
        pytest.skip('no docs next to the code')
    for name, text in TEMPLATES.items():
        assert yaml.safe_load((docs / f'{name}.yaml').read_text()) == yaml.safe_load(text), name
