"""A cell with the old skills, migrated (taskweave config migrate-skills), validates against the real catalog of the new
skills: every step form of the old missions, with the nested arguments checked and the poses split."""

import copy
import shutil
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from taskweave_core.config import ConfigStore, packs  # noqa: E402
from taskweave_core.config.migrate import migrate_skills_only  # noqa: E402
from taskweave_core.config.schemas import HAND_RULES, hands_schema  # noqa: E402
from taskweave_core.plan import Plan  # noqa: E402
from taskweave_core.validation import validate_plan  # noqa: E402
from taskweave_skills.basic import Wait  # noqa: E402
from taskweave_skills.gripper_motion import Grip, MoveGripper  # noqa: E402
from taskweave_skills.joint_motion import MoveJoints  # noqa: E402
from taskweave_skills.navigate import NavigateTo  # noqa: E402
from taskweave_skills.object_motion import MoveObject, SetPosture  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
ROBOT = 'duo_bot'
MISSION = """\
goal: Every old step form
steps:
- {instruction: tuck, skill: move_to_pose, args: {pose: home}}
- {instruction: present one hand, skill: move_to_pose, args: {pose: present_left}}
- {instruction: present both, skill: move_to_pose, args: {pose: present_both}}
- {instruction: open, skill: gripper, args: {side: both, action: open}}
- {instruction: pre-grasp, skill: go_to_point, args: {point: crate_1/rim, offset_m: 0.1}}
- {instruction: one hand, skill: go_to_point, args: {point: crate_1/rim, tools: [left], linear: true}}
- {instruction: onto the rim, skill: go_to_point, args: {point: crate_1/rim, approach_m: 0.0}}
- {instruction: close, skill: gripper, args: {side: left, action: close}}
- {instruction: lift, skill: move_object, args: {by: [0, 0, 0.1], tilt_deg: 5}}
- {instruction: to, skill: move_object, args: {at: [0.5, 0.0, 0.3]}}
- {instruction: carry, skill: move_object, args: {pose: carry}}
- {instruction: elbows, skill: move_object, args: {elbow: out}}
- {instruction: onto, skill: move_object, args: {to: pile_1/top, contact: base, offset_m: 0.05}}
- {instruction: release, skill: gripper, args: {side: left, action: open, place_as: floor_box}}
- {instruction: pause, skill: wait, args: {seconds: 1}}
- {instruction: drive, skill: navigate_to, args: {target: dock}}
"""
OLD_YAMLS = {
    'go_to_point': 'implementations:\n  - {name: ik, type: script, class: "taskweave_skills.joints:GoToPoint"}\n'
                   'max_joint_speed: 2.0\nstall_timeout_s: 1.0\nstate_timeout_s: 3.0\napproach_m: 0.1\n',
    'gripper': 'implementations:\n  - {name: g, type: script, class: "taskweave_skills.joints:Gripper"}\n'
               'max_joint_speed: 2.0\nstall_timeout_s: 1.0\nstate_timeout_s: 3.0\n',
    'move_object': 'implementations:\n  - {name: m, type: script, class: "taskweave_skills.joints:MoveObject"}\n'
                   'max_joint_speed: 2.0\nstall_timeout_s: 1.0\nstate_timeout_s: 3.0\n',
    'move_to_pose': 'implementations:\n  - {name: p, type: script, class: "taskweave_skills.joints:MoveToPose"}\n'
                    'max_joint_speed: 2.0\nstall_timeout_s: 1.0\nstate_timeout_s: 3.0\n',
}
NEW_YAMLS = ('move_gripper', 'grip', 'move_joints', 'move_object', 'set_posture')


@pytest.fixture
def cell(tmp_path, monkeypatch):
    monkeypatch.setattr(packs, 'installed_location', lambda: None)
    root = tmp_path / 'cell'
    shutil.copytree(FIXTURE, root)
    (root / 'pack.yaml').write_text('name: cell\nversion: 1\nschema_version: 2\n')
    (root / 'system.yaml').write_text(f'robot: {ROBOT}\n')
    for name in NEW_YAMLS:
        (root / 'skills' / f'{name}.yaml').unlink()
    for name, text in OLD_YAMLS.items():
        (root / 'skills' / f'{name}.yaml').write_text(text)
    (root / 'missions').mkdir(exist_ok=True)
    (root / 'missions' / 'old.yaml').write_text(MISSION)
    return root


def catalog(store):
    hands = list(store.robot(ROBOT).data.get('grippers', {}))
    out = {}
    for skill in (MoveGripper, Grip, MoveJoints, MoveObject, SetPosture, NavigateTo, Wait):
        schema = copy.deepcopy(skill.args_schema)
        if 'hands' in schema.get('properties', {}):
            schema['properties']['hands'] = hands_schema(hands, ('auto', 'all') if skill is Grip else
                                                         getattr(skill, 'hand_rules', HAND_RULES))
        out[skill.name] = schema
    return out


def test_the_old_mission_is_invalid_and_the_migrated_one_validates(cell):
    store = ConfigStore(cell, check_schema=False)
    plan = Plan.from_mission(store.expanded_mission('old'), 'old', robot=ROBOT)
    before = validate_plan(plan, catalog(store), store)
    assert before and all('taskweave config migrate-skills' in p
                          for p in before if 'was removed' in p or 'takes its' in p)
    assert migrate_skills_only(cell, False, lambda line: None) == 0
    store = ConfigStore(cell)
    plan = Plan.from_mission(store.expanded_mission('old'), 'old', robot=ROBOT)
    assert validate_plan(plan, catalog(store), store) == []
    assert [step.skill for step in plan.steps] == [
        'move_joints', 'move_gripper', 'move_joints', 'move_gripper', 'grip', 'move_gripper', 'move_gripper',
        'move_gripper', 'grip', 'move_object', 'move_object', 'move_object', 'set_posture', 'move_object', 'grip',
        'wait', 'navigate_to']
    assert [step.id for step in plan.steps][2:4] == ['s3_joints', 's3_tools']
    assert plan.steps[2].args == {'pose': 'present_both_joints'}
    assert plan.steps[5].args == {'to': {'point': 'crate_1/rim', 'offset_m': 0.1}}


def test_the_new_skill_yamls_load_into_the_skills(cell):
    migrate_skills_only(cell, False, lambda line: None)
    store = ConfigStore(cell)
    for name in NEW_YAMLS:
        implementation, reasons = store.select_implementation(name, ROBOT)
        assert implementation is not None, (name, reasons)
    gripper = store.skill_params('move_gripper')
    assert gripper['speed']['joint_rad_s'] == 2.0 and gripper['tolerances']['stall_timeout_s'] == 1.0
    assert gripper['motion']['approach_m'] == 0.1
