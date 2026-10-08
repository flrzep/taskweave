import pytest

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_core.embodiment import check_grasp, check_profile, match, robot_card
from taskweave_core.plan import Plan
from taskweave_core.validation import validate_plan

PICK_SCHEMA = {'type': 'object', 'properties': {'grasp': {'type': 'string', 'x-taskweave-point': 'object_point'}}}


def test_embodiment_loads(config_dir):
    embodiment = ConfigStore(config_dir).embodiment('demo_bot')
    assert 'gripper.parallel' in embodiment['capabilities']
    assert embodiment['arms']['left']['payload_kg'] is None  # unknown stays unknown


def test_embodiment_rejects_hardware_names_and_unknown_capabilities(config_dir):
    path = config_dir / 'robots' / 'demo_bot' / 'embodiment.yaml'
    original = path.read_text()
    path.write_text(original + 'cmd_vel: /cmd_vel\n')
    with pytest.raises(ConfigError, match='cmd_vel'):
        ConfigStore(config_dir).embodiment('demo_bot')
    path.write_text(original.replace('camera.head.rgbd', 'camera.laser_eyes'))
    with pytest.raises(ConfigError, match='capabilities'):
        ConfigStore(config_dir).embodiment('demo_bot')


def test_match_explains_every_mismatch(config_dir):
    embodiment = ConfigStore(config_dir).embodiment('demo_bot')
    assert match({'capabilities': ['base.holonomic']}, embodiment) == []
    reasons = match({'capabilities': ['arms.bimanual'], 'action_space': 'joint_pos_bimanual',
                     'cameras': ['left_wrist'], 'embodiments': ['a2d']}, embodiment)
    assert reasons == ['missing capabilities arms.bimanual', "no action space 'joint_pos_bimanual'",
                       'missing cameras left_wrist', 'trained for a2d, not demo_bot']


def test_first_fitting_implementation_is_selected(config_dir):
    store = ConfigStore(config_dir)
    implementation, reasons = store.select_implementation('pick', 'demo_bot')
    assert implementation['name'] == 'pick_scripted'
    assert reasons and reasons[0].startswith('pick_vla: missing capabilities')
    assert store.skill_params('pick', implementation) == {'lift_height': 0.2, 'approach_speed': 0.1}
    assert store.skill_params('pick') == {'lift_height': 0.2}


def test_skill_without_fitting_implementation_is_rejected(config_dir):
    store = ConfigStore(config_dir)
    plan = Plan.from_mission({'goal': 'g', 'robot': 'demo_bot', 'steps': [
        {'instruction': 'fly', 'skill': 'fly'}]}, 'p')
    problems = validate_plan(plan, {'fly': {'type': 'object'}}, store)
    assert problems == ["step 's1' (fly): no implementation fits robot 'demo_bot' "
                        '(fly: missing capabilities arms.bimanual)']


def test_grasp_checked_against_tools(config_dir):
    store = ConfigStore(config_dir)
    embodiment = store.embodiment('demo_bot')
    points = store.object('box')['points']
    assert check_grasp(points['top_grasp'], embodiment) == []
    assert check_grasp(points['wide_grasp'], embodiment) == ["grip width 0.2 m exceeds the 'left' stroke 0.08 m"]
    assert check_grasp(points['suction_top'], embodiment) == ["needs a suction on 'left', robot has parallel_gripper"]
    plan = Plan.from_mission({'goal': 'g', 'robot': 'demo_bot', 'steps': [
        {'instruction': 'pick', 'skill': 'pick', 'args': {'grasp': 'box.wide_grasp'}}]}, 'p')
    problems = validate_plan(plan, {'pick': PICK_SCHEMA}, store)
    assert len(problems) == 1 and 'box.wide_grasp does not fit robot' in problems[0]


def test_profile_and_embodiment_must_agree(config_dir):
    store = ConfigStore(config_dir)
    assert check_profile(store.embodiment('demo_bot'), store.robot('demo_bot').data) == []
    differential = {**store.robot('demo_bot').data, 'base': {**store.robot('demo_bot').base, 'type': 'differential'}}
    problems = check_profile(store.embodiment('demo_bot'), differential)
    assert any("lacks capability 'base.differential'" in p for p in problems)


def test_robot_card_lists_limits_points_and_skills(config_dir):
    store = ConfigStore(config_dir)
    skills = [{'name': 'navigate_to', 'description': 'Drive somewhere.', 'implementation': 'script:navigate_direct',
               'args_schema': {'required': ['target'], 'properties': {'target': {'type': 'string',
                                                                                    'description': 'location'}}},
               'needs': ['path free'], 'achieves': ['at target']}]
    card = robot_card(store.embodiment('demo_bot'), skills, store.point_names('demo_bot'))
    assert card.startswith('Robot: demo_bot\nGeneric test robot')
    assert 'Arm left: 6-DoF, reach 0.80 m, payload unknown, tool left' in card
    assert 'Tool left: parallel_gripper, stroke 0.080 m, finger length unknown' in card
    assert '- locations: dock, home' in card and 'stance:shelf_a' in card and 'box.top_grasp' in card
    assert '- navigate_to(target: location) [script:navigate_direct]: Drive somewhere.' in card
    assert 'needs: path free' in card
