"""move_gripper and grip against a small two-arm simulator: joint targets in, joint states out, a fake MoveIt IK
(joint1 = the target's x, joint2 = its z) and static TF for the flanges. Targets x modes x hands x motion options."""

import json
import math
import shutil
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from conftest import RosRig, connect, wait_ready, wait_until

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
pytest.importorskip('moveit_msgs')

from geometry_msgs.msg import TransformStamped  # noqa: E402
from moveit_msgs.srv import GetPositionIK  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_core.validation import oneof_problems  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_skills.hold import TOPIC, HoldState  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
OK, FAILED, REJECTED = RunSkill.Result.SUCCEEDED, RunSkill.Result.FAILED, RunSkill.Result.REJECTED
OVERRIDE = 'move_gripper: {motion: {approach_m: 0.2}}\n'   # a robot override of the skill's approach_m
FLANGES = {'left': (0.5, 0.2, 0.3), 'right': (0.5, -0.2, 0.3)}   # where the test's TF puts the hands (identity turn)


class DuoSim(Node):
    """Joints move toward their targets at 3 units/s; a finger joint stops at ``object_at[joint]`` when closing."""

    def __init__(self):
        super().__init__('duo_sim')
        self.position = {'left_joint1': 0.0, 'left_joint2': 0.0, 'right_joint1': 0.0, 'right_joint2': 0.0,
                         'lift': 0.0, 'left_grip': 1.0, 'right_grip': 1.0}
        self.target, self.object_at, self.history = dict(self.position), {}, []
        self.frozen = set()           # joints that do not move (a stalled joint)
        self.pub = self.create_publisher(JointState, '/duo/joint_states', 10)
        self.create_subscription(JointState, '/duo/joint_command', self.command, 10)
        self.create_timer(0.02, self.step)

    def command(self, msg):
        self.target.update(zip(msg.name, msg.position, strict=False))
        self.history.append(dict(zip(msg.name, msg.position, strict=False)))

    def step(self):
        for name, target in self.target.items():
            if name in self.frozen:
                continue
            current = self.position[name]
            new = current + max(-0.06, min(0.06, target - current))
            if name in self.object_at and target < self.object_at[name]:
                new = max(new, self.object_at[name])
            self.position[name] = new
        self.pub.publish(JointState(name=list(self.position), position=list(self.position.values())))


class FakeIk(Node):
    """/compute_ik: <side>_joint1 = the requested flange x, <side>_joint2 = its z. ``need_lift``: only solves when the
    request's start state has this lift (the point is out of reach at the other heights)."""

    def __init__(self):
        super().__init__('fake_ik')
        self.requests, self.need_lift = [], None
        self.blocked = set()          # sides ('left', 'right') whose targets are out of reach
        self.create_service(GetPositionIK, '/compute_ik', self.ik)

    def ik(self, request, response):
        ik = request.ik_request
        start = dict(zip(ik.robot_state.joint_state.name, ik.robot_state.joint_state.position, strict=True))
        pose = ik.pose_stamped.pose
        self.requests.append({'group': ik.group_name, 'xyz': (pose.position.x, pose.position.y, pose.position.z),
                              'q': (pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
                              'start': start})
        side = 'right' if ik.group_name.startswith('right') else 'left'
        response.solution.joint_state.name = [f'{side}_joint1', f'{side}_joint2']
        response.solution.joint_state.position = [pose.position.x, pose.position.z]
        reachable = self.need_lift is None or abs(start.get('lift', -1.0) - self.need_lift) < 1e-6
        response.error_code.val = 1 if reachable and side not in self.blocked else -31
        return response


@pytest.fixture
def system(tmp_path, request):
    with RosRig() as ros:
        config = tmp_path / 'config'
        shutil.copytree(FIXTURE, config)
        if getattr(request, 'param', None):
            (config / 'robots' / 'duo_bot' / 'skill_overrides.yaml').write_text(request.param)
        sim, ik = ros.add(DuoSim()), ros.add(FakeIk())
        server = SkillServer(parameter_overrides=[
            Parameter('config_dir', value=str(config)), Parameter('robot', value='duo_bot'),
            Parameter('skills', value=['move_gripper', 'grip', 'move_object', 'set_posture', 'move_joints'])])
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
        skill = server.skills['move_gripper']
        wait_ready(server, skill.ik.client)

        def flanges_in_tf():  # the static transforms have reached the skill server's buffer
            try:
                hold.tcp_poses(list(FLANGES))
            except LookupError:  # not in the buffer yet
                return False
            return True

        wait_until(flanges_in_tf, what='the flange transforms at the skill server')
        wait_until(lambda: published, what='the first holding message')
        yield SimpleNamespace(sim=sim, ik=ik, server=server, client=client, published=published,
                              hold=hold, config=config)


def run(system, skill, args, timeout_s=30.0):
    action = connect(system.client, RunSkill, names.skill_action(skill))
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args),
                                                  timeout_s=timeout_s))
    wait_until(future.done, what='the goal to be accepted')
    result = future.result().get_result_async()
    wait_until(result.done, timeout=timeout_s + 30, what='the skill result')
    return result.result().result


def go(system, args, outcome=OK):
    result = run(system, 'move_gripper', args)
    assert result.outcome == outcome, result.message
    return result, json.loads(result.data_json)


def at(system, hand, joint1, joint2, tolerance=0.03):
    position = system.sim.position
    assert position[f'{hand}_joint1'] == pytest.approx(joint1, abs=tolerance)
    assert position[f'{hand}_joint2'] == pytest.approx(joint2, abs=tolerance)


def settled(system, text, timeout=30.0):
    """The last published holding payload once ``text`` (a json fragment) is in it."""
    try:
        wait_until(lambda: system.published and text in json.dumps(system.published[-1], sort_keys=True),
                   timeout, 0.05, f'{text!r} in the holding message')
    except AssertionError:
        raise AssertionError(f'{text!r} not in {system.published[-1:]}') from None
    return system.published[-1]


# ---- schemas and registration ---------------------------------------------------------------------------------

def test_the_schemas_pass_the_oneof_rule_and_carry_the_robots_hands(system):
    from taskweave_skills.gripper_motion import Grip, MoveGripper

    for schema in (MoveGripper.args_schema, Grip.args_schema, *(s.args_schema for s in system.server.skills.values())):
        assert oneof_problems(schema, allow_several=True) == []
    skill = system.server.skills['move_gripper']
    move, grip = skill.args_schema['properties'], system.server.skills['grip'].args_schema['properties']
    assert move['hands']['anyOf'][0]['enum'] == ['auto', 'any_one', 'all', 'left', 'right']
    assert grip['hands']['anyOf'][0]['enum'] == ['auto', 'any_one', 'all', 'left', 'right']
    assert [v['required'] for v in move['to']['oneOf']] == [['point'], ['pose'], ['by'], ['at']]
    assert skill.implementation == 'script:ik_planned'                       # arms.bimanual comes first
    assert skill.args_schema['x-taskweave-resources'] == ['hands', 'torso']


def test_an_ambiguous_or_misplaced_target_is_rejected_with_the_nested_path(system):
    assert run(system, 'move_gripper', {'to': {'by': [0, 0, 0], 'at': [0, 0, 0]}}).outcome == REJECTED
    result = run(system, 'move_gripper', {'to': {'point': 'box_1/nope'}})
    assert result.outcome == REJECTED and 'args.to.point' in result.message
    result = run(system, 'move_gripper', {'to': {'pose': 'home'}})          # joint values only: that is move_joints
    assert result.outcome == REJECTED and 'places no tools' in result.message
    assert run(system, 'move_gripper', {'to': {'by': [0, 0, 1]}, 'motion': {'mode': 'sideways'}}).outcome == REJECTED
    assert run(system, 'grip', {'action': 'opening'}).outcome == REJECTED         # opening_mm is required
    result = run(system, 'grip', {'action': 'close', 'hands': 'any_one'})      # no hand is at an object point
    assert result.outcome == REJECTED and 'no hand is at an object point' in result.message


# ---- targets x modes --------------------------------------------------------------------------------------------

def test_pose_free_moves_only_the_hands_the_pose_places(system):
    _, data = go(system, {'to': {'pose': 'present_left'}})
    at(system, 'left', 0.6, 0.4)
    at(system, 'right', 0.0, 0.0)
    assert data['hands'] == ['left'] and data['mode'] == 'free' and data['path'] == 'direct'
    assert data['world_state'] == {'arm_pose': 'present_left'}
    assert data['motion']['elbow'] == 'out'         # the pose's own elbow side (target layer)
    assert data['speed']['override_avg'] == pytest.approx(100.0, abs=1.0)   # the server's record of the override


def test_pose_with_joints_and_a_hands_subset(system):
    _, data = go(system, {'to': {'pose': 'present_both'}, 'hands': ['right']})
    at(system, 'right', 0.6, 0.4)
    at(system, 'left', 0.0, 0.0)
    assert data['hands'] == ['right'] and system.sim.position['lift'] == pytest.approx(0.1, abs=0.02)
    _, data = go(system, {'to': {'pose': 'present_both'}})
    assert data['hands'] == ['left', 'right']
    at(system, 'left', 0.6, 0.4)
    result, _ = go(system, {'to': {'pose': 'present_left'}, 'hands': ['right']}, REJECTED)
    assert 'present_left has no tool poses for right' in result.message


def test_pose_linear_goes_by_straight_lines_from_where_the_hand_is(system):
    _, data = go(system, {'to': {'pose': 'present_left'}, 'motion': {'mode': 'linear'}})
    at(system, 'left', 0.6, 0.4)
    assert data['mode'] == 'linear' and data['path'] == 'linear'
    assert any(0.5 < h.get('left_joint1', 0) < 0.6 for h in system.sim.history)       # in between: a line, not a jump


def test_point_free_stops_short_then_goes_straight_in_along_the_approach(system):
    _, data = go(system, {'to': {'point': 'box_1/rim'}})
    at(system, 'left', 0.6, 0.3)
    at(system, 'right', 0.6, 0.3)
    assert data['path'] == 'approach' and data['hands'] == ['left', 'right']
    assert '(0.1 m straight)' in data['world_state']['arm_pose']          # the point's own approach_m, not the skill's
    assert system.hold.reached_hands() == {h: {'point': 'box_1/rim', 'instance': 'box_1'} for h in ('left', 'right')}


def test_point_with_an_offset_is_not_a_reach(system):
    _, data = go(system, {'to': {'point': 'box_1/rim', 'offset_m': 0.1}, 'motion': {'approach_m': 0}})
    at(system, 'left', 0.6, 0.4)                                           # 10 cm back along the approach (down)
    assert data['path'] == 'direct' and system.hold.reached_hands() == {}
    assert 'box_1/rim (-0.1 m)' == data['world_state']['arm_pose']


def test_point_linear_ignores_approach_m_and_marks_the_point_reached(system):
    _, data = go(system, {'to': {'point': 'box_1/rim'}, 'motion': {'mode': 'linear'}})
    at(system, 'left', 0.6, 0.3)
    assert data['path'] == 'linear' and data['world_state']['arm_pose'] == 'box_1/rim (linear)'
    assert set(system.hold.reached_hands()) == {'left', 'right'}


def test_a_hand_alone_takes_the_points_alone_pose_else_the_pairs(system):
    _, data = go(system, {'to': {'point': 'box_1/rim'}, 'hands': ['left'], 'motion': {'approach_m': 0}})
    at(system, 'left', 0.6, 0.35)                                          # alone: z 0.35 instead of the pair's 0.3
    at(system, 'right', 0.0, 0.0)
    assert data['hands'] == ['left'] and set(system.hold.reached_hands()) == {'left'}
    go(system, {'to': {'point': 'box_1/rim'}, 'hands': 'right', 'motion': {'approach_m': 0}})   # no alone pose: pair's
    at(system, 'right', 0.6, 0.3)
    assert set(system.hold.reached_hands()) == {'left', 'right'}


def test_a_hand_that_moves_elsewhere_is_no_longer_at_its_point(system):
    go(system, {'to': {'point': 'box_1/rim'}, 'motion': {'approach_m': 0}})
    go(system, {'to': {'by': [0, 0, 0.0]}, 'hands': ['left']})                  # left moves (by nothing): off the point
    assert set(system.hold.reached_hands()) == {'right'}


def test_by_in_the_robot_and_the_tool_frame_with_turn_and_tilt(system):
    go(system, {'to': {'by': [0.1, 0, 0.1]}, 'hands': ['left']})
    at(system, 'left', 0.6, 0.4)
    assert system.ik.requests[-1]['xyz'] == pytest.approx((0.6, 0.2, 0.4))
    go(system, {'to': {'by': [0, 0, -0.1], 'frame': 'tool'}, 'hands': ['left']})   # along the TCP z (up): down is -z
    assert system.ik.requests[-1]['xyz'] == pytest.approx((0.5, 0.2, 0.2))   # the TF TCP still stands at 0.5, 0.2, 0.3
    go(system, {'to': {'by': [0, 0, 0], 'turn_deg': 90}, 'hands': ['left']})
    w, x, y, z = system.ik.requests[-1]['q']
    assert (w, z) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5))) and abs(x) + abs(y) < 1e-6
    go(system, {'to': {'by': [0, 0, 0], 'tilt_deg': 90}, 'hands': ['left']})
    w, x, y, z = system.ik.requests[-1]['q']
    assert (w, y) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5))) and abs(x) + abs(z) < 1e-6


def test_by_with_auto_hands_moves_every_hand_and_linear_goes_straight(system):
    _, data = go(system, {'to': {'by': [0.1, 0, 0]}})
    assert data['hands'] == ['left', 'right']
    at(system, 'left', 0.6, 0.3)
    at(system, 'right', 0.6, 0.3)
    _, data = go(system, {'to': {'by': [0.2, 0, 0]}, 'hands': ['left'], 'motion': {'mode': 'linear'}})
    assert data['path'] == 'linear'
    at(system, 'left', 0.7, 0.3)


def test_at_places_one_hand_and_keeps_or_sets_the_orientation(system):
    _, data = go(system, {'to': {'at': [0.7, -0.1, 0.5]}, 'hands': ['right']})
    at(system, 'right', 0.7, 0.5)
    assert data['hands'] == ['right'] and system.ik.requests[-1]['q'] == pytest.approx((1.0, 0, 0, 0))   # kept
    go(system, {'to': {'at': [0.7, -0.1, 0.5], 'rpy_deg': [0, 0, 90]}, 'hands': 'right'})
    w, x, y, z = system.ik.requests[-1]['q']
    assert (w, z) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))
    result, _ = go(system, {'to': {'at': [0.7, 0.1, 0.5]}}, REJECTED)          # auto = both hands
    assert result.message == 'at needs exactly one hand'


def test_a_hand_that_holds_something_is_refused(system):
    go(system, {'to': {'point': 'box_1/rim'}})
    assert run(system, 'grip', {'action': 'close'}).outcome == OK
    result, _ = go(system, {'to': {'by': [0, 0, 0.1]}}, REJECTED)
    assert result.message.startswith('left holds box_1; right holds box_1: the hand and its arm stay with the object')


def test_a_close_that_cannot_register_the_hold_fails(system):
    go(system, {'to': {'point': 'box_1/rim'}})

    def no_tf(_tools):
        raise LookupError('no transform')
    system.hold.tcp_poses = no_tf
    result = run(system, 'grip', {'action': 'close'})
    assert result.outcome == FAILED and 'hold not registered' in result.message
    assert not system.hold.hand_holds


def test_the_free_arm_moves_while_the_other_hand_holds(system):
    go(system, {'to': {'point': 'box_1/rim'}, 'hands': ['left'], 'motion': {'approach_m': 0}})
    assert run(system, 'grip', {'action': 'close', 'hands': ['left']}).outcome == OK
    assert system.hold.hand_holds == {'left': 'box_1'}
    _, data = go(system, {'to': {'by': [0.1, 0, 0.1]}, 'hands': ['right']})              # explicit hand
    assert data['hands'] == ['right'] and system.ik.requests[-1]['group'] == 'right_arm'
    _, data = go(system, {'to': {'pose': 'present_right'}})                              # auto: the pose's one hand
    assert data['hands'] == ['right']
    result = run(system, 'move_joints', {'joints': {'right_joint1': 0.3, 'right_joint2': 0.2}})   # its arm group
    assert result.outcome == OK, result.message
    assert run(system, 'grip', {'action': 'close', 'hands': ['right']}).outcome == OK     # grip with the free hand
    assert run(system, 'grip', {'action': 'open', 'hands': ['right']}).outcome == OK
    assert system.hold.hand_holds == {'left': 'box_1'}
    # the holding hand and its arm stay with the object: rejected, naming the hand, nothing moved
    before = dict(system.sim.position)
    for skill, args in (('move_gripper', {'to': {'by': [0, 0, 0.1]}, 'hands': ['left']}),
                        ('move_gripper', {'to': {'by': [0, 0, 0.1]}}),                  # auto: both hands
                        ('move_joints', {'joints': {'left_joint1': 0.2}}),
                        ('move_joints', {'joints': {'left_joint1': 0.2, 'right_joint1': 0.1, 'lift': 0.1}})):
        result = run(system, skill, args)
        assert result.outcome == REJECTED and 'left holds box_1' in result.message, (skill, result.message)
    assert system.sim.position == before
    # the torso moves, and the hold follows it
    result = run(system, 'move_joints', {'joints': {'lift': 0.1}})
    assert result.outcome == OK, result.message
    assert system.hold.hand_holds == {'left': 'box_1'}


# ---- motion options ---------------------------------------------------------------------------------------------

def test_the_torso_is_tried_at_other_heights_with_torso_auto_only(system):
    system.ik.need_lift = 0.2                                            # out of reach at the current lift
    result, _ = go(system, {'to': {'pose': 'present_left'}, 'motion': {'torso': 'keep'}}, FAILED)
    assert 'no IK solution' in result.message and system.sim.position['lift'] == 0.0
    go(system, {'to': {'pose': 'present_left'}})                         # the skill's torso: auto
    at(system, 'left', 0.6, 0.4)
    assert system.sim.position['lift'] == pytest.approx(0.2, abs=0.02)


def test_approach_m_comes_from_the_step_then_the_point_then_the_skill(system):
    label = lambda data: data['world_state']['arm_pose']      # noqa: E731
    _, data = go(system, {'to': {'point': 'box_1/rim_plain'}})
    assert '(0.15 m straight)' in label(data)                            # the skill yaml's
    _, data = go(system, {'to': {'point': 'box_1/rim_plain'}, 'motion': {'approach_m': 0.05}})
    assert '(0.05 m straight)' in label(data)                            # the step's
    _, data = go(system, {'to': {'point': 'box_1/rim'}})
    assert '(0.1 m straight)' in label(data)                             # the point's beats the skill's
    _, data = go(system, {'to': {'point': 'box_1/rim'}, 'motion': {'approach_m': 0.05}})
    assert '(0.05 m straight)' in label(data)                            # the step beats the point


@pytest.mark.parametrize('system', [OVERRIDE], indirect=True)
def test_a_robot_override_sits_between_the_skill_and_the_point(system):
    _, data = go(system, {'to': {'point': 'box_1/rim_plain'}})
    assert '(0.2 m straight)' in data['world_state']['arm_pose']         # the robot's beats the skill's 0.15
    _, data = go(system, {'to': {'point': 'box_1/rim'}})
    assert '(0.1 m straight)' in data['world_state']['arm_pose']         # the point's beats the robot's


def test_planner_options_and_speed_options(system):
    result, _ = go(system, {'to': {'pose': 'present_left'}, 'motion': {'planner': 'planned'}}, REJECTED)
    assert 'planner planned needs the profile planning section' in result.message
    _, data = go(system, {'to': {'pose': 'present_left'}, 'motion': {'planner': 'direct'}})
    assert data['path'] == 'direct' and data['motion']['planner'] == 'direct'
    start = time.monotonic()                          # 0.6 rad at 0.3 rad/s: 2 s, not the 0.5 s floor
    go(system, {'to': {'pose': 'present_both'}, 'motion': {'speed_abs': {'joint_rad_s': 0.3}}})
    assert 1.8 < time.monotonic() - start < 20.0


def test_via_goes_through_the_waypoint_first(system):
    go(system, {'to': {'pose': 'present_left'}})
    system.sim.history.clear()
    go(system, {'to': {'pose': 'present_both'}, 'motion': {'via': ['pose:home']}})
    at(system, 'left', 0.6, 0.4)
    assert min(h['left_joint1'] for h in system.sim.history if 'left_joint1' in h) < 0.1     # home, then the pose


def test_tolerance_fails_a_hand_that_ends_further_off(system):
    result, _ = go(system, {'to': {'pose': 'present_left'}, 'motion': {'tolerance': 0.01}}, FAILED)   # TF never moves
    assert 'left is 0.1' in result.message and 'tolerance 0.01 m' in result.message
    go(system, {'to': {'pose': 'present_left'}, 'motion': {'tolerance': 0.5}})


# ---- the speed numbers (no ROS) ---------------------------------------------------------------------------------

def stub_skill():
    from taskweave_skills.gripper_motion import MoveGripper
    from taskweave_skills.joints import Params

    skill = MoveGripper.__new__(MoveGripper)
    skill.p, skill.tcp_speed, skill.tcp_turn, skill.plan_scale = Params(), 0.05, 0.3, 1.0
    return skill


def test_speed_is_a_fraction_of_the_standard_or_absolute():
    skill = stub_skill()
    standard = {'joint_rad_s': 2.0, 'tcp_m_s': 0.5, 'tcp_rad_s': 1.0}
    skill._speeds({'standard_speed': standard, 'speed': 0.5, 'accel': 0.5})
    assert (skill.p.max_joint_speed, skill.tcp_speed, skill.tcp_turn) == (1.0, 0.25, 0.5)
    assert skill.p.min_duration_s == 1.0 and skill.plan_scale == 0.5
    skill = stub_skill()
    skill._speeds({'standard_speed': standard, 'speed': 0.5, 'speed_abs': {'tcp_m_s': 0.1}})   # absolute wins
    assert skill.tcp_speed == 0.1 and skill.p.max_joint_speed == 1.0
    skill._speeds({'standard_speed': standard, 'speed_abs': {'joint_rad_s': 9.0}})
    assert skill.plan_scale == 1.0                   # a planned path is never played faster than it was planned


def test_a_planned_path_is_stretched_to_the_speed_fraction():
    from taskweave_skills.motion_plan import Trajectory

    skill = stub_skill()
    path = Trajectory(['a'], [0.0, 1.0, 2.0], [[0.0], [1.0], [2.0]])
    assert skill._slowed(path) is path
    skill.plan_scale = 0.5
    assert skill._slowed(path).times == [0.0, 2.0, 4.0]


def test_relative_targets_move_and_turn_about_the_tcp_in_either_frame():
    from taskweave_skills.points import moved_tcp, target_of

    tcp = np.eye(4)
    tcp[:3, 3] = (1.0, 2.0, 3.0)
    tcp[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]                      # the tool turned 90 deg about the vertical
    robot = moved_tcp(tcp, [0.1, 0, 0], 'robot')
    tool = moved_tcp(tcp, [0.1, 0, 0], 'tool')
    assert robot[:3, 3] == pytest.approx([1.1, 2.0, 3.0]) and tool[:3, 3] == pytest.approx([1.0, 2.1, 3.0])
    turned = moved_tcp(tcp, [0, 0, 0], 'tool', turn_deg=90)               # about the tool z: the same as the vertical
    assert turned[:3, :3] == pytest.approx(np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]]))
    assert target_of(tcp, 0.5)['tcp'] == pytest.approx([1.0, 2.0, 2.5]) and target_of(tcp)['approach'] == [0, 0, 1]


# ---- grip --------------------------------------------------------------------------------------------------------

def test_close_at_a_point_holds_with_the_hands_that_closed(system):
    go(system, {'to': {'point': 'box_1/rim'}})
    system.sim.object_at = {'left_grip': 0.5}
    result = run(system, 'grip', {'action': 'close', 'hands': ['left']})
    data = json.loads(result.data_json)
    assert result.outcome == OK, result.message
    assert data['hands'] == ['left'] and data['holding'] == {'left': True}
    assert system.hold.hand_holds == {'left': 'box_1'} and system.sim.position['right_grip'] == 1.0
    payload = settled(system, '"hands": ["left"]')               # legacy keys of the first hold, plus all holds
    assert payload == {'object': 'box_1', 'point': 'box_1/rim', 'tools': ['left'], 'links': ['left_link7'],
                       'holds': [{'object': 'box_1', 'point': 'box_1/rim', 'hands': ['left'],
                                  'links': ['left_link7']}]}
    assert data['world_state']['held_object'] == 'box_1' and data['world_state']['left_gripper'] == 'holding'
    run(system, 'grip', {'action': 'close', 'hands': ['right']})          # the other hand joins the hold
    assert system.hold.held()['hands'] == ['left', 'right'] and len(system.hold.held_all()) == 1
    assert settled(system, '"hands": ["left", "right"]')['tools'] == ['left', 'right']


def test_close_auto_takes_the_hands_at_a_point_and_uses_the_points_grip(system):
    go(system, {'to': {'point': 'box_1/rim'}, 'hands': ['left']})
    result = run(system, 'grip', {'action': 'close'})                     # auto: only the hand at a point
    data = json.loads(result.data_json)
    assert result.outcome == OK and data['hands'] == ['left']
    assert system.sim.position['right_grip'] == 1.0
    assert system.sim.position['left_grip'] == pytest.approx(0.5, abs=0.03)   # grip_m 40 mm of an 80 mm gripper
    assert data['holding'] == {'left': True} and data['opening_mm']['left'] == pytest.approx(40, abs=3)
    assert system.hold.hand_holds == {'left': 'box_1'}
    run(system, 'grip', {'action': 'close', 'hands': 'all'})              # right is not at a point: it holds nothing
    assert system.hold.hand_holds == {'left': 'box_1'}
    assert system.sim.position['right_grip'] == pytest.approx(0, abs=0.03)


def test_open_takes_hands_out_of_the_hold_and_the_last_one_releases(system):
    go(system, {'to': {'point': 'box_1/rim'}, 'motion': {'approach_m': 0}})
    run(system, 'grip', {'action': 'close'})
    assert set(system.hold.hand_holds) == {'left', 'right'}
    result = run(system, 'grip', {'action': 'open', 'hands': ['left']})
    data = json.loads(result.data_json)
    assert result.outcome == OK and system.hold.hand_holds == {'right': 'box_1'}
    assert data['world_state']['held_object'] == 'box_1' and 'let go of box_1' in result.message
    assert settled(system, '"tools": ["right"]')['holds'][0]['hands'] == ['right']
    result = run(system, 'grip', {'action': 'open'})                      # auto: the holding hand (right)
    data = json.loads(result.data_json)
    assert data['hands'] == ['right'] and system.hold.held() is None and data['world_state']['held_object'] is None
    assert 'released' in result.message
    assert settled(system, '{}') == {}                                    # nothing held: the empty legacy payload
    assert system.sim.position['left_grip'] == pytest.approx(1.0, abs=0.03)


def test_open_with_place_as_records_the_object_only_when_the_last_hand_lets_go(system):
    go(system, {'to': {'point': 'pile_1/rim'}, 'motion': {'approach_m': 0}})     # a stack: closing takes one off
    run(system, 'grip', {'action': 'close'})
    result = run(system, 'grip', {'action': 'open', 'hands': ['left'], 'place_as': 'box_2'})
    assert 'box_2 not placed: the object is still held' in result.message
    result = run(system, 'grip', {'action': 'open', 'place_as': 'box_2'})
    assert result.outcome == OK and 'placed box_2' in result.message, result.message
    assert 'box_2' in system.server.config.object_instances()


def test_opening_sets_the_gap_in_mm_of_a_free_hand_and_rejects_a_holding_one(system):
    go(system, {'to': {'point': 'box_1/rim'}, 'hands': ['left'], 'motion': {'approach_m': 0}})
    run(system, 'grip', {'action': 'close', 'hands': ['left']})
    before = dict(system.sim.position)
    for args in ({'hands': ['left']}, {}):                                  # the holding hand, alone or among all hands
        result = run(system, 'grip', {'action': 'opening', 'opening_mm': 20, **args})
        assert result.outcome == REJECTED
        assert 'left holds box_1: open it with action open, or move the object first' in result.message
    assert system.sim.position == before and system.hold.hand_holds == {'left': 'box_1'}   # no finger commanded
    result = run(system, 'grip', {'action': 'opening', 'opening_mm': 60, 'hands': ['right']})   # a free hand works
    data = json.loads(result.data_json)
    assert result.outcome == OK, result.message
    assert system.sim.position['right_grip'] == pytest.approx(0.75, abs=0.03)
    assert data['opening_mm']['right'] == pytest.approx(60, abs=3) and data['holding'] == {'right': False}
    assert system.hold.hand_holds == {'left': 'box_1'}


def test_open_and_close_with_an_opening_and_a_missing_table(system):
    result = run(system, 'grip', {'action': 'close', 'hands': ['right'], 'opening_mm': 40})
    assert result.outcome == OK and system.sim.position['right_grip'] == pytest.approx(0.5, abs=0.03)
    result = run(system, 'grip', {'action': 'open', 'opening_mm': 80, 'hands': ['right']})
    assert result.outcome == OK and system.sim.position['right_grip'] == pytest.approx(1.0, abs=0.03)
    profile = system.server.robot.data['grippers']['left']
    del profile['opening_m']                                             # a gripper without an opening table
    result = run(system, 'grip', {'action': 'opening', 'opening_mm': 40, 'hands': ['left']})
    assert result.outcome == REJECTED and 'left has no opening table' in result.message


def test_a_sim_reset_ends_every_hold_and_every_reached_point(system):
    go(system, {'to': {'point': 'box_1/rim'}, 'motion': {'approach_m': 0}})
    run(system, 'grip', {'action': 'close', 'hands': ['left']})
    system.hold._on_reset(None)
    assert system.hold.held_all() == [] and system.hold.reached_hands() == {}
    settled(system, '{}')


def test_the_cell_pack_examples_in_docs_load_into_the_skills_parameters():
    import yaml
    from jsonschema import Draft202012Validator

    from taskweave_core.config import schemas
    from taskweave_skills.gripper_motion import translate_params
    from taskweave_skills.joints import Params

    docs = Path(__file__).resolve().parents[2] / 'docs' / 'examples' / 'skills'
    if not docs.is_dir():
        pytest.skip('docs/ not next to the packages (the test runner ships the packages only)')
    for name in ('move_gripper', 'grip'):
        data = yaml.safe_load((docs / f'{name}.yaml').read_text(encoding='utf-8'))
        Draft202012Validator(schemas.SKILL_PARAMS).validate(data)
        flat, extra = translate_params({k: v for k, v in data.items() if k != 'implementations'})
        for key in ('cell_frame', 'lift_joint', 'lift_values', 'tcp_speed', 'tcp_turn'):
            flat.pop(key, None)
        assert Params.from_config(flat).position_tolerance == 0.03 and extra['accel']
    assert flat['max_joint_speed'] == 1.5
    flat, extra = translate_params({'speed': {'tcp_m_s': 0.2}, 'motion': {'mode': 'linear'},
                                    'torso_fallback': {'joint': 'lift', 'values': [0.0]}, 'max_joint_speed': 1.0})
    # old keys stay
    assert flat == {'tcp_speed': 0.2, 'lift_joint': 'lift', 'lift_values': [0.0], 'max_joint_speed': 1.0}


# ---- hands any_one / auto (T3.1) -------------------------------------------------------------------------------
def park(system, hand, joint1, joint2):
    """Put a hand's joints at a place (sim and target) and wait until the skill sees them."""
    sim = system.sim
    for name, value in ((f'{hand}_joint1', joint1), (f'{hand}_joint2', joint2)):
        sim.position[name] = sim.target[name] = value
    joints = system.server.skills['move_gripper'].joints

    def seen():
        state = joints.snapshot(1.0) or {}
        return (abs(state.get(f'{hand}_joint1', 9) - joint1) < 1e-6
                and abs(state.get(f'{hand}_joint2', 9) - joint2) < 1e-6)
    wait_until(seen, what=f'the {hand} joints at {joint1}, {joint2}')


RIM = {'to': {'point': 'box_1/rim'}, 'motion': {'approach_m': 0}}


def test_any_one_takes_the_only_reachable_hand_and_reports_it(system):
    system.ik.blocked = {'left'}
    result, data = go(system, {**RIM, 'hands': 'any_one'})
    assert data['hands'] == ['right'] and data['choice']['chosen'] == 'right'
    assert 'any_one chose right' in result.message and set(system.hold.reached_hands()) == {'right'}
    assert data['choice']['candidates']['left']['feasible'] is False
    at(system, 'right', 0.6, 0.3)                                      # the pair's pose: this point has no right alone
    system.ik.blocked = {'right'}
    park(system, 'right', 0.0, 0.0)
    _, data = go(system, {**RIM, 'hands': 'any_one'})
    assert data['hands'] == ['left']
    at(system, 'left', 0.6, 0.35)                                      # left's alone pose


def test_any_one_takes_the_least_joint_motion_and_ties_go_to_the_profile_order(system):
    park(system, 'right', 0.6, 0.3)                                    # right already stands at its pair pose
    _, data = go(system, {**RIM, 'hands': 'any_one'})
    assert data['hands'] == ['right']
    cands = data['choice']['candidates']
    assert cands['right']['motion_rad'] < cands['left']['motion_rad']
    park(system, 'left', 0.6, 0.35)
    park(system, 'right', 0.0, 0.0)
    assert go(system, {**RIM, 'hands': 'any_one'})[1]['hands'] == ['left']
    for _ in range(2):                                                 # equal motion: always the first hand
        park(system, 'left', 0.0, 0.0)
        park(system, 'right', 0.0, 0.0)
        assert go(system, {'to': {'pose': 'present_both'}, 'hands': 'any_one'})[1]['hands'] == ['left']


def test_any_one_with_no_hand_able_is_rejected_with_a_reason_per_hand(system):
    system.ik.blocked = {'left', 'right'}
    result, _ = go(system, {**RIM, 'hands': 'any_one'}, REJECTED)
    assert 'no hand can take it' in result.message and 'left: no IK solution' in result.message
    assert 'right: no IK solution' in result.message
    system.ik.blocked = {'left'}                                       # present_left: only left has the pose
    result, _ = go(system, {'to': {'pose': 'present_left'}, 'hands': 'any_one'}, REJECTED)
    assert 'left: no IK solution' in result.message
    assert 'right: pose present_left has no tool poses for right' in result.message
    system.ik.blocked = set()
    assert go(system, {'to': {'pose': 'present_left'}, 'hands': 'any_one'})[1]['hands'] == ['left']


def test_a_holding_hand_is_never_chosen(system):
    skill = system.server.skills['move_gripper']
    go(system, {**RIM, 'hands': ['left']})
    assert run(system, 'grip', {'action': 'close', 'hands': 'left'}).outcome == OK
    chosen = skill._target({'to': {'by': [0, 0, 0.1]}, 'hands': 'any_one'})        # the choice alone, not the motion
    assert chosen.hands == ['right']
    system.hold.reach(['right'], 'box_1/knob', 'box_1', np.eye(4))
    system.hold.close_hands(['right'])                                              # both hands in the hold now
    refused = skill._target({'to': {'by': [0, 0, 0.1]}, 'hands': 'any_one'})
    assert refused.outcome.name == 'REJECTED' and 'left: holds box_1' in refused.message
    assert 'right: holds box_1' in refused.message


def test_auto_takes_the_one_hand_a_point_defines_and_grip_any_one_the_hand_at_a_point(system):
    go(system, {**RIM, 'hands': 'auto'})                                                 # both hands at the point
    result = run(system, 'grip', {'action': 'close', 'hands': 'any_one'})
    assert result.outcome == REJECTED and 'ambiguous' in result.message and 'left and right' in result.message
    go(system, {'to': {'by': [0, 0, 0.0]}, 'hands': ['left']})                          # left leaves the point
    _, data = go(system, {'to': {'point': 'box_1/knob'}, 'motion': {'approach_m': 0}})     # only a right_tool
    assert data['hands'] == ['right'] and 'choice' not in data     # auto: the one hand that has a pose
    result = run(system, 'grip', {'action': 'close', 'hands': 'any_one'})
    assert result.outcome == OK and json.loads(result.data_json)['hands'] == ['right']
    assert run(system, 'grip', {'action': 'open', 'hands': 'any_one'}).outcome == OK     # the one holding hand


# ---- P3 review fixes ---------------------------------------------------------------------------------------------
def hold_by_registry(system, hand, instance='box_1', point='rim'):
    """``hand`` holds ``instance`` (the registry, without moving the sim)."""
    system.hold.reach([hand], f'{instance}/{point}', instance, np.eye(4))
    system.hold.close_hands([hand])


def test_grip_close_any_one_never_counts_the_holding_hand_as_reached(system):     # I1
    hold_by_registry(system, 'left')
    system.hold.reach(['right'], 'box_1/knob', 'box_1', np.eye(4))
    result = run(system, 'grip', {'action': 'close', 'hands': 'any_one'})
    assert result.outcome == OK, result.message
    assert json.loads(result.data_json)['hands'] == ['right']
    # open any_one still names the one holding hand
    hold_by_registry(system, 'left', 'pile_1')
    assert system.hold.hand_holds['left'] == 'box_1'


def test_a_holding_gripper_joint_is_refused_like_the_arm_joints(system):          # M9
    hold_by_registry(system, 'left')
    before = dict(system.sim.position)
    result = run(system, 'move_joints', {'joints': {'left_grip': 0.0}})
    assert result.outcome == REJECTED and 'left holds box_1' in result.message, result.message
    assert system.sim.position == before and system.hold.hand_holds == {'left': 'box_1'}
    assert run(system, 'move_joints', {'joints': {'right_grip': 0.5}}).outcome == OK     # the free hand's gripper


def test_place_as_refuses_to_place_two_holds_with_one_name(system):               # M6
    hold_by_registry(system, 'left', 'box_1')
    hold_by_registry(system, 'right', 'pile_1')
    before = dict(system.sim.position)
    result = run(system, 'grip', {'action': 'open', 'hands': 'all', 'place_as': 'box_9'})
    assert result.outcome == REJECTED and 'box_1' in result.message and 'pile_1' in result.message, result.message
    assert system.sim.position == before and set(system.hold.hand_holds) == {'left', 'right'}
    assert 'box_9' not in system.server.config.object_instances()
    result = run(system, 'grip', {'action': 'open', 'hands': ['left'], 'place_as': 'box_9'})   # one hold: fine
    assert result.outcome == OK and 'placed box_9' in result.message, result.message


@pytest.mark.parametrize('hands', [['right'], 'any_one'])
def test_a_via_pose_leg_moves_only_the_target_hands(system, hands):               # M7
    hold_by_registry(system, 'left')
    left_before = {k: v for k, v in system.sim.position.items() if k.startswith('left')}
    system.sim.history.clear()
    go(system, {'to': {'pose': 'present_right'}, 'hands': hands, 'motion': {'via': ['pose:present_both']}})
    assert system.ik.requests and {r['group'] for r in system.ik.requests} == {'right_arm'}
    assert {k: v for k, v in system.sim.position.items() if k.startswith('left')} == left_before
    assert not any(k.startswith('left') for h in system.sim.history for k in h)


def shifted(system, name='box_1', by=1.0):
    """The registry's pose of a held object, moved away from where its hands say it is."""
    pose = np.eye(4)
    pose[0, 3] = by
    system.hold.moved(pose, name)


def object_x(system, name='box_1'):
    return float(system.hold.hold_of(next(iter(system.hold.holds[name]['hands'])))['object_in_base'][0, 3])


def test_the_holds_follow_the_hands_when_a_move_stalls_after_a_lift_change(system):      # I3
    hold_by_registry(system, 'left')
    shifted(system)
    system.sim.frozen = {'lift'}                       # the torso moves nowhere: the move stalls
    result = run(system, 'move_joints', {'joints': {'lift': 0.1}})
    assert result.outcome == FAILED, result.message
    assert object_x(system) == pytest.approx(0.0, abs=1e-6), 'the hold was not followed after the failed move'
    shifted(system)
    system.sim.frozen = {'right_joint1'}
    result = run(system, 'move_gripper', {'to': {'by': [0.1, 0, 0]}, 'hands': ['right']})
    assert result.outcome == FAILED, result.message
    assert object_x(system) == pytest.approx(0.0, abs=1e-6), 'the hold was not followed after the failed move_gripper'


class StubPlanner:
    """Stands in for the MotionPlanner: records the ``keep`` of every plan() call and plans nothing (straight move)."""
    direct = False

    def __init__(self):
        self.keeps = []

    def plan(self, start, goal, should_stop=None, keep=None):
        self.keeps.append(dict(keep or {}))

    def free(self, start, goal, should_stop=None):
        return True


def test_a_planned_move_keeps_the_holding_arms_where_they_are(system):                 # C1
    stub = StubPlanner()
    for name in ('move_gripper', 'move_joints'):
        system.server.skills[name].planner = stub
    planned = {'planner': 'planned'}
    go(system, {'to': {'by': [0.1, 0, 0]}, 'hands': ['right'], 'motion': planned})
    assert stub.keeps[-1] == {}                                     # nothing held: the planner is not constrained
    hold_by_registry(system, 'left')
    go(system, {'to': {'by': [0.1, 0, 0]}, 'hands': ['right'], 'motion': planned})
    assert stub.keeps[-1] == {'left_joint1': 0.0, 'left_joint2': 0.0, 'left_grip': 1.0}     # the left arm and hand
    result = run(system, 'move_joints', {'joints': {'lift': 0.1}, 'motion': planned})       # a torso move
    assert result.outcome == OK, result.message
    assert set(stub.keeps[-1]) == {'left_joint1', 'left_joint2', 'left_grip'}
    hold_by_registry(system, 'right', 'pile_1')
    result = run(system, 'move_joints', {'joints': {'lift': 0.0}, 'motion': planned})
    assert set(stub.keeps[-1]) == {'left_joint1', 'left_joint2', 'left_grip', 'right_joint1', 'right_joint2',
                                   'right_grip'}


def test_move_object_keeps_the_arm_of_another_hold_but_moves_its_own_hands(system):    # C1
    stub = StubPlanner()
    system.server.skills['move_object'].planner = stub
    hold_by_registry(system, 'left', 'box_1')
    hold_by_registry(system, 'right', 'pile_1')
    result = run(system, 'move_object', {'to': {'by': [0.0, 0.0, 0.05]}, 'object': 'box_1',
                                         'motion': {'mode': 'free', 'planner': 'planned'}})
    assert result.outcome == OK, result.message
    assert set(stub.keeps[-1]) == {'right_joint1', 'right_joint2', 'right_grip'}, stub.keeps
