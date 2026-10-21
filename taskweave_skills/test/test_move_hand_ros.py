"""move for hands against the small two-arm simulator of test_gripper_motion (joint targets in, joint states out, a fake
MoveIt IK, static TF for the flanges), on the schema-4 test cell: the rows of the mapping table for move_gripper, the
holds, paths, joints parts, via, the torso, dry runs and the messages. grip is served next to it as in a real cell."""

import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from conftest import TIME_SCALE, RosRig, connect, copy_config, wait_ready, wait_until

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
pytest.importorskip('moveit_msgs')

from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from test_gripper_motion import FLANGES, DuoSim, FakeIk, FakeScene  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from taskweave_core import motion, names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_skills.hold import TOPIC, HoldState  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config_new'
OK, FAILED, REJECTED = RunSkill.Result.SUCCEEDED, RunSkill.Result.FAILED, RunSkill.Result.REJECTED
OVERRIDE = 'move.hand: {motion: {approach_m: 0.2}}\n'   # a robot override of the part's approach_m


@pytest.fixture
def system(tmp_path, request):
    with RosRig() as ros:
        config = tmp_path / 'config'
        copy_config(FIXTURE, config)
        if getattr(request, 'param', None):
            (config / 'robots' / 'duo_bot' / 'skill_overrides.yaml').write_text(request.param)
        sim, ik, scene = ros.add(DuoSim()), ros.add(FakeIk()), ros.add(FakeScene())
        server = SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value='duo_bot'),
            Parameter('skills', value=['move', 'grip'])])
        ros.add(server)
        client = ros.add(rclpy.create_node('test_client'))
        tf, transforms = StaticTransformBroadcaster(client), []
        for parent, child, xyz in [('world', 'base_link', (0.0, 0.0, 0.0)),
                                   *(('base_link', f'{hand}_link7', xyz) for hand, xyz in FLANGES.items())]:
            t = TransformStamped(child_frame_id=child)
            t.header.frame_id = parent
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            transforms.append(t)
        tf.sendTransform(transforms)
        published = []
        client.create_subscription(String, TOPIC, lambda m: published.append(json.loads(m.data)),
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        ros.spin(8)
        hold = HoldState._instances[id(server)]
        move = server.skills['move']
        wait_ready(server, move.modules['hand'].ik.client)

        def flanges_in_tf():
            try:
                hold.tcp_poses(list(FLANGES))
            except LookupError:
                return False
            return True

        wait_until(flanges_in_tf, what='the flange transforms at the skill server')
        wait_until(lambda: published, what='the first holding message')
        yield SimpleNamespace(sim=sim, ik=ik, scene=scene, server=server, client=client, published=published,
                              hold=hold, config=config, move=move)


def run(system, skill, args, timeout_s=None):
    timeout_s = 30.0 * TIME_SCALE if timeout_s is None else timeout_s
    action = connect(system.client, RunSkill, names.skill_action(skill))
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args),
                                                  timeout_s=timeout_s))
    wait_until(future.done, what='the goal to be accepted')
    result = future.result().get_result_async()
    wait_until(result.done, timeout=timeout_s + 30, what='the skill result')
    return result.result().result


def go(system, args, outcome=OK):
    result = run(system, 'move', args)
    assert result.outcome == outcome, result.message
    return result, json.loads(result.data_json)


def at(system, hand, joint1, joint2, tolerance=0.03):
    position = system.sim.position
    assert position[f'{hand}_joint1'] == pytest.approx(joint1, abs=tolerance)
    assert position[f'{hand}_joint2'] == pytest.approx(joint2, abs=tolerance)


# ---- serving ----------------------------------------------------------------------------------------------

def test_move_is_served_with_its_parts_and_the_robots_hands(system):
    skill = system.server.skills['move']
    assert list(system.server.skills) == ['move', 'grip']
    assert sorted(skill.modules) == ['base', 'hand', 'joints', 'object', 'posture', 'straight']
    assert 'hand' in skill.modules and skill.modules['hand'].name == 'move.hand'
    assert skill.modules['hand'].implementation == 'script:ik_planned'            # arms.bimanual comes first
    assert skill.args_schema['properties']['hands']['anyOf'][0]['enum'] == ['auto', 'any_one', 'all', 'left',
                                                                            'right']
    assert skill.args_schema['properties']['hand']['anyOf'][0]['enum'] == ['all', 'left', 'right']
    assert motion.values(motion.resolve(system.server.config, 'duo_bot', 'move.hand'))['torso'] == 'auto'
    assert skill.modules['hand'].cell_frame == 'world'
    assert skill.default_timeout({'to': 'present_left'}) == 60.0


def test_the_old_skill_names_of_the_launch_serve_move_on_a_cell_in_the_targets_layout(tmp_path):
    with RosRig() as ros:
        config = tmp_path / 'config'
        copy_config(FIXTURE, config)
        server = ros.add(SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value='duo_bot'),
            Parameter('skills', value=['move_gripper', 'grip', 'navigate_to', 'move_joints', 'wait'])]))
        assert list(server.skills) == ['move', 'grip', 'wait']


# ---- targets x modes -----------------------------------------------------------------------------------------

def test_a_base_frame_hand_target_free_moves_only_the_hands_it_places(system):
    result, data = go(system, {'to': 'present_left'})
    at(system, 'left', 0.6, 0.4)
    at(system, 'right', 0.0, 0.0)
    assert data['hands'] == ['left'] and data['mode'] == 'free' and data['path'] == 'direct'
    assert data['world_state'] == {'arm_pose': 'present_left'}
    assert data['motion']['elbow'] == 'out'            # the target's own elbow side (target layer)
    assert data['kind'] == 'hand' and data['module'] == 'move.hand' and data['target'] == 'present_left'
    assert result.message.startswith('move hand present_left (left): at present_left (max error')
    assert 'max error' in result.message                # the smoke test parses it
    assert data['speed']['override_avg'] == pytest.approx(100.0, abs=1.0)


def test_a_target_with_joints_and_a_hands_subset(system):
    _, data = go(system, {'to': 'present_both', 'hands': ['right']})
    at(system, 'right', 0.6, 0.4)
    at(system, 'left', 0.0, 0.0)
    assert data['hands'] == ['right'] and system.sim.position['lift'] == pytest.approx(0.1, abs=0.02)
    _, data = go(system, {'to': 'present_both'})
    assert data['hands'] == ['left', 'right']
    at(system, 'left', 0.6, 0.4)
    result, _ = go(system, {'to': 'present_left', 'hands': ['right']}, REJECTED)
    assert result.message == 'move hand: present_left has no tool poses for right'


def test_linear_goes_by_straight_lines_from_where_the_hand_is(system):
    _, data = go(system, {'to': 'present_left', 'motion': {'mode': 'linear'}})
    at(system, 'left', 0.6, 0.4)
    assert data['mode'] == 'linear' and data['path'] == 'linear'
    assert any(0.5 < h.get('left_joint1', 0) < 0.6 for h in system.sim.history)


def test_a_point_free_stops_short_then_goes_straight_in_along_the_approach(system):
    _, data = go(system, {'to': 'box_1/rim'})
    at(system, 'left', 0.6, 0.3)
    at(system, 'right', 0.6, 0.3)
    assert data['path'] == 'approach' and data['hands'] == ['left', 'right']
    assert '(0.1 m straight)' in data['world_state']['arm_pose']        # the point's own approach_m, not the part's
    assert system.hold.reached_hands() == {h: {'point': 'box_1/rim', 'instance': 'box_1'} for h in ('left', 'right')}


def test_a_point_with_an_offset_is_not_a_reach(system):
    _, data = go(system, {'to': 'box_1/rim', 'offset_m': 0.1, 'motion': {'approach_m': 0}})
    at(system, 'left', 0.6, 0.4)
    assert data['path'] == 'direct' and system.hold.reached_hands() == {}
    assert data['world_state']['arm_pose'] == 'box_1/rim (-0.1 m)'


def test_a_point_linear_ignores_approach_m_and_marks_the_point_reached(system):
    _, data = go(system, {'to': 'box_1/rim', 'motion': {'mode': 'linear'}})
    at(system, 'left', 0.6, 0.3)
    assert data['path'] == 'linear' and data['world_state']['arm_pose'] == 'box_1/rim (linear)'
    assert set(system.hold.reached_hands()) == {'left', 'right'}


def test_a_hand_alone_takes_the_alone_pose_else_the_pairs(system):
    _, data = go(system, {'to': 'box_1/rim', 'hands': ['left'], 'motion': {'approach_m': 0}})
    at(system, 'left', 0.6, 0.35)
    at(system, 'right', 0.0, 0.0)
    assert data['hands'] == ['left'] and set(system.hold.reached_hands()) == {'left'}
    go(system, {'to': 'box_1/rim', 'hands': 'right', 'motion': {'approach_m': 0}})
    at(system, 'right', 0.6, 0.3)


def test_a_hand_that_moves_elsewhere_is_no_longer_at_its_point(system):
    go(system, {'to': 'box_1/rim', 'motion': {'approach_m': 0}})
    go(system, {'hand': 'left', 'by': [0, 0, 0.0]})
    assert set(system.hold.reached_hands()) == {'right'}


def test_by_in_the_robot_and_the_tool_frame_with_turn_and_tilt(system):
    go(system, {'hand': 'left', 'by': [0.1, 0, 0.1]})
    at(system, 'left', 0.6, 0.4)
    assert system.ik.requests[-1]['xyz'] == pytest.approx((0.6, 0.2, 0.4))
    go(system, {'hand': 'left', 'by': [0, 0, -0.1], 'frame': 'tool'})
    assert system.ik.requests[-1]['xyz'] == pytest.approx((0.5, 0.2, 0.2))
    go(system, {'hand': 'left', 'by': [0, 0, 0], 'turn_deg': 90})
    w, x, y, z = system.ik.requests[-1]['q']
    assert (w, z) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5))) and abs(x) + abs(y) < 1e-6
    go(system, {'hand': 'left', 'by': [0, 0, 0], 'tilt_deg': 90})
    w, x, y, z = system.ik.requests[-1]['q']
    assert (w, y) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5))) and abs(x) + abs(z) < 1e-6


def test_by_with_all_hands_moves_every_hand_and_linear_goes_straight(system):
    _, data = go(system, {'hand': 'all', 'by': [0.1, 0, 0]})
    assert data['hands'] == ['left', 'right']
    at(system, 'left', 0.6, 0.3)
    at(system, 'right', 0.6, 0.3)
    _, data = go(system, {'hand': ['left'], 'by': [0.2, 0, 0], 'motion': {'mode': 'linear'}})
    assert data['path'] == 'linear'
    at(system, 'left', 0.7, 0.3)


def test_at_places_one_hand_and_keeps_or_sets_the_orientation(system):
    _, data = go(system, {'hand': 'right', 'at': [0.7, -0.1, 0.5]})
    at(system, 'right', 0.7, 0.5)
    assert data['hands'] == ['right'] and system.ik.requests[-1]['q'] == pytest.approx((1.0, 0, 0, 0))
    go(system, {'hand': 'right', 'at': [0.7, -0.1, 0.5], 'rpy_deg': [0, 0, 90]})
    w, x, y, z = system.ik.requests[-1]['q']
    assert (w, z) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))
    result, _ = go(system, {'hand': 'all', 'at': [0.7, 0.1, 0.5]}, REJECTED)
    assert result.message == 'move hand: at needs exactly one hand'


def test_a_hand_that_holds_something_is_refused_in_moves_words(system):
    go(system, {'to': 'box_1/rim'})
    assert run(system, 'grip', {'action': 'close'}).outcome == OK
    result, _ = go(system, {'hand': 'all', 'by': [0, 0, 0.1]}, REJECTED)
    assert result.message == ('move hand: left holds box_1; right holds box_1: move the object (`object: held` or an '
                              'object target), or open the gripper first')


def test_the_free_arm_moves_while_the_other_hand_holds(system):
    go(system, {'to': 'box_1/rim', 'hands': ['left'], 'motion': {'approach_m': 0}})
    assert run(system, 'grip', {'action': 'close', 'hands': ['left']}).outcome == OK
    assert system.hold.hand_holds == {'left': 'box_1'}
    _, data = go(system, {'hand': 'right', 'by': [0.1, 0, 0.1]})
    assert data['hands'] == ['right'] and system.ik.requests[-1]['group'] == 'right_arm'
    _, data = go(system, {'to': 'present_right'})
    assert data['hands'] == ['right']
    result, _ = go(system, {'joints': {'right_arm': [0.3, 0.2]}})
    assert result.message.startswith('move joints right_arm [0.3, 0.2]')
    before = dict(system.sim.position)
    for args in ({'hand': 'left', 'by': [0, 0, 0.1]}, {'hand': 'all', 'by': [0, 0, 0.1]},
                 {'joints': {'left_arm': [0.2, None]}}, {'joints': {'left_arm': [0.2, None], 'torso': 0.1}}):
        result = run(system, 'move', args)
        assert result.outcome == REJECTED and 'left holds box_1' in result.message, (args, result.message)
    assert system.sim.position == before
    go(system, {'joints': {'torso': 0.1}})
    assert system.hold.hand_holds == {'left': 'box_1'}


# ---- the torso, motion options ---------------------------------------------------------------------------------

def test_the_torso_is_tried_at_other_heights_with_torso_auto_only(system):
    system.ik.need_lift = 0.2
    result, _ = go(system, {'to': 'present_left', 'torso': 'keep'}, FAILED)
    assert 'no IK solution' in result.message and system.sim.position['lift'] == 0.0
    assert result.message.startswith('move hand present_left')
    go(system, {'to': 'present_left'})                       # the part's torso: auto
    at(system, 'left', 0.6, 0.4)
    assert system.sim.position['lift'] == pytest.approx(0.2, abs=0.02)


def test_a_torso_value_of_the_step_sets_the_height_and_keeps_it(system):
    go(system, {'to': 'present_left', 'torso': 0.15})
    assert system.sim.position['lift'] == pytest.approx(0.15, abs=0.02)
    assert system.ik.requests[-1]['start']['lift'] == pytest.approx(0.15, abs=0.02)


def test_the_joints_part_of_a_target_moves_the_lift(system):
    _, data = go(system, {'to': 'present_both'})
    assert system.sim.position['lift'] == pytest.approx(0.1, abs=0.02) and data['hands'] == ['left', 'right']
    go(system, {'joints': {'torso': 0.0}})
    go(system, {'to': 'present_both', 'torso': 'keep'})      # keep: the target's lift is not set either
    assert system.sim.position['lift'] == pytest.approx(0.0, abs=0.02)


def with_ik_torso(system, candidates='[0.0, 0.1, 0.2, 0.3]'):
    path = system.config / 'robots' / 'duo_bot' / 'profile.yaml'
    torso = f'  torso: {{joint: lift, candidates: {candidates}}}'
    path.write_text(path.read_text().replace('  timeout_s: 0.2', '  timeout_s: 0.2\n' + torso, 1))


def test_ik_torso_searches_the_height_and_the_result_reports_it_and_the_rejected_ones(system):
    with_ik_torso(system)
    system.ik.need_lift = 0.2
    result, data = go(system, {'to': 'present_left'})
    at(system, 'left', 0.6, 0.4)
    assert system.sim.position['lift'] == pytest.approx(0.2, abs=0.02)
    (leg,) = data['torso']
    assert leg['height'] == pytest.approx(0.2) and 'torso 0.2' in result.message
    assert {r['height'] for r in leg['rejected']} >= {0.0, 0.1}
    system.ik.need_lift = None
    _, data = go(system, {'to': 'present_left', 'torso': 'keep'})
    assert 'torso' not in data


def test_ik_torso_with_no_feasible_height_fails_with_every_height_and_its_reason(system):
    with_ik_torso(system)
    system.ik.blocked = {'left'}
    result, data = go(system, {'to': 'present_left'}, FAILED)
    assert 'no torso height works' in result.message and result.message.startswith('move hand present_left')
    assert {r['height'] for r in data['torso'][0]['rejected']} == {0.0, 0.1, 0.2, 0.3}
    assert system.sim.position['lift'] == 0.0


def test_approach_m_comes_from_the_step_then_the_target_then_the_part(system):
    label = lambda data: data['world_state']['arm_pose']      # noqa: E731
    _, data = go(system, {'to': 'box_1/rim_plain'})
    assert '(0.15 m straight)' in label(data)
    _, data = go(system, {'to': 'box_1/rim_plain', 'motion': {'approach_m': 0.05}})
    assert '(0.05 m straight)' in label(data)
    _, data = go(system, {'to': 'box_1/rim'})
    assert '(0.1 m straight)' in label(data)
    _, data = go(system, {'to': 'box_1/rim', 'motion': {'approach_m': 0.05}})
    assert '(0.05 m straight)' in label(data)


@pytest.mark.parametrize('system', [OVERRIDE], indirect=True)
def test_a_robot_override_of_move_hand_sits_between_the_part_and_the_target(system):
    _, data = go(system, {'to': 'box_1/rim_plain'})
    assert '(0.2 m straight)' in data['world_state']['arm_pose']
    _, data = go(system, {'to': 'box_1/rim'})
    assert '(0.1 m straight)' in data['world_state']['arm_pose']


def test_planner_options_and_speed_options(system):
    result, _ = go(system, {'to': 'present_left', 'motion': {'planner': 'planned'}}, REJECTED)
    assert 'planner planned needs the profile planning section' in result.message
    _, data = go(system, {'to': 'present_left', 'motion': {'planner': 'direct'}})
    assert data['path'] == 'direct' and data['motion']['planner'] == 'direct'
    start = time.monotonic()
    go(system, {'to': 'present_both', 'motion': {'speed_abs': {'joint_rad_s': 0.3}}})
    assert 1.8 < time.monotonic() - start < 20.0


def test_via_goes_through_a_joint_target_or_a_hand_target_first(system):
    go(system, {'to': 'present_left'})
    system.sim.history.clear()
    go(system, {'to': 'present_both', 'motion': {'via': ['home']}})
    at(system, 'left', 0.6, 0.4)
    assert min(h['left_joint1'] for h in system.sim.history if 'left_joint1' in h) < 0.1
    system.ik.requests.clear()
    _, data = go(system, {'to': 'box_1/knob', 'motion': {'via': ['present_right']}})
    assert system.ik.requests[0]['group'] == 'right_arm' and system.ik.requests[0]['xyz'] == pytest.approx(
        (0.6, -0.2, 0.4))                                                     # the via target first, then the point
    assert data['hands'] == ['right']


def test_tolerance_fails_a_hand_that_ends_further_off(system):
    result, _ = go(system, {'to': 'present_left', 'motion': {'tolerance': 0.01}}, FAILED)
    assert 'left is 0.1' in result.message and 'tolerance 0.01 m' in result.message
    go(system, {'to': 'present_left', 'motion': {'tolerance': 0.5}})


def test_a_target_with_a_path_visits_the_waypoints_in_order_and_the_last_is_the_target(system):
    _, data = go(system, {'to': 'box_1/rim_path', 'motion': {'approach_m': 0}})
    heights = [round(r['xyz'][2], 3) for r in system.ik.requests if r['group'] == 'left_arm']
    assert heights[:2] == [0.5, 0.3] and heights[-1] == 0.3
    at(system, 'left', 0.6, 0.3)
    assert data['hands'] == ['left'] and set(system.hold.reached_hands()) == {'left'}
    assert system.hold.reached_hands()['left']['point'] == 'box_1/rim_path'


def test_hands_any_one_takes_the_one_hand_the_target_defines(system):
    _, data = go(system, {'to': 'box_1/knob', 'hands': 'any_one'})
    assert data['hands'] == ['right'] and data['choice']['chosen'] == 'right'


# ---- rejections ----------------------------------------------------------------------------------------------

def test_cell_frame_hand_targets_and_unknown_targets_are_rejected_before_anything_moves(system):
    before = dict(system.sim.position)
    result, _ = go(system, {'to': 'dock_tool'}, REJECTED)
    assert result.message == "move hand: cell-frame hand targets are not supported yet (target 'dock_tool')"
    result, _ = go(system, {'to': 'box_1/rim_nope'}, REJECTED)
    assert result.message.startswith("args.to: object 'box' has no target 'rim_nope'")
    result, _ = go(system, {'to': 'present_left', 'hands': 'left', 'offset_m': 0.1, 'by': [0, 0, 0]}, REJECTED)
    assert 'option by does not apply' in result.message
    result, _ = go(system, {'to': 'dock'}, REJECTED)                    # a base move on a robot with a fixed base
    assert result.message == 'move base: robot base is fixed'
    assert system.sim.position == before


def test_the_schema_rejects_two_forms_and_the_misspelt_target_names_the_path(system):
    assert run(system, 'move', {'to': 'dock', 'hand': 'left', 'by': [0, 0, 0]}).outcome == REJECTED
    result = run(system, 'move', {'to': 'box_1/rimm'})
    assert result.outcome == REJECTED and 'args.to' in result.message
    assert run(system, 'move', {}).outcome == REJECTED


# ---- dry run ---------------------------------------------------------------------------------------------------

def test_a_dry_run_solves_and_returns_the_joint_goal_without_moving(system):
    before, commands = dict(system.sim.position), len(system.sim.history)
    result, data = go(system, {'to': 'present_both', 'dry_run': True})
    assert data['dry_run'] is True and data['kind'] == 'hand' and data['target'] == 'present_both'
    assert data['joint_goal'] == pytest.approx({'lift': 0.1, 'left_joint1': 0.6, 'left_joint2': 0.4,
                                                'right_joint1': 0.6, 'right_joint2': 0.4})
    assert [leg['label'] for leg in data['legs']] == ['present_both']
    assert 'nothing moved' in result.message
    assert system.sim.position == before and len(system.sim.history) == commands


def test_a_dry_run_lists_the_approach_and_the_waypoints_and_reports_an_unreachable_target(system):
    _, data = go(system, {'to': 'box_1/rim', 'dry_run': True})
    assert [leg['label'] for leg in data['legs']] == ['box_1/rim (approach)', 'box_1/rim']
    assert data['legs'][0]['joint_goal']['left_joint2'] == pytest.approx(0.4)      # 0.1 m back along the approach
    assert data['joint_goal']['left_joint2'] == pytest.approx(0.3)
    _, data = go(system, {'to': 'box_1/rim_path', 'dry_run': True})
    assert [leg['label'] for leg in data['legs']][0] == 'box_1/rim_path (1/2)'
    system.ik.blocked = {'left'}
    result, _ = go(system, {'to': 'present_left', 'dry_run': True}, FAILED)
    assert 'no IK solution' in result.message and result.message.startswith('move hand present_left')
    assert system.sim.position['left_joint1'] == 0.0
    result, _ = go(system, {'hand': 'all', 'by': [0, 0, 0.1], 'dry_run': True}, FAILED)    # relative: solved too


# ---- grip on a cell in the targets layout ---------------------------------------------------------------------

def test_grip_uses_the_robots_grip_setting_of_the_target(system):
    go(system, {'to': 'box_1/rim', 'hands': ['left']})
    result = run(system, 'grip', {'action': 'close'})
    data = json.loads(result.data_json)
    assert result.outcome == OK and data['hands'] == ['left']
    assert system.sim.position['left_grip'] == pytest.approx(0.5, abs=0.03)          # grip_m 40 mm of an 80 mm gripper
    assert system.hold.hand_holds == {'left': 'box_1'}
    result = run(system, 'grip', {'action': 'close', 'hands': 'any_one'})
    assert result.outcome == REJECTED
    assert run(system, 'grip', {'action': 'open'}).outcome == OK
    nothing = run(system, 'grip', {'action': 'close', 'hands': 'any_one'})
    assert nothing.outcome == REJECTED and 'no hand is at an object point' in nothing.message
