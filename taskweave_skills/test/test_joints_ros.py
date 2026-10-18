"""move_joints and grip against a small joint simulator (targets in, states out)."""

import json
import math
import shutil
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')

from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from taskweave_core import names  # noqa: E402
from taskweave_core.skill_server import SkillServer  # noqa: E402
from taskweave_interfaces.action import RunSkill  # noqa: E402
from taskweave_skills.joints import Params, interpolate, plan_duration  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
J1_LIMIT = 2.0


class JointSim(Node):
    """Joints move toward their targets at 3 units/s; j1 stops at J1_LIMIT; 'grip' can be blocked by an object."""

    def __init__(self):
        super().__init__('joint_sim')
        self.position = {'j1': 0.0, 'j2': 0.0, 'lift': 0.0, 'grip': 1.0}
        self.target = dict(self.position)
        self.object_at = None  # grip stops here when closing on an object
        self.creep = 0.0       # units/s the object gives way while pushed (a sim finger grinding through a collider)
        self.pub = self.create_publisher(JointState, '/arm/joint_states', 10)
        self.create_subscription(JointState, '/arm/joint_command', self.command, 10)
        self.create_timer(0.02, self.step)

    def command(self, msg):
        self.target.update(zip(msg.name, msg.position, strict=False))

    def step(self):
        for name, target in self.target.items():
            current = self.position[name]
            new = current + max(-0.06, min(0.06, target - current))
            if name == 'j1':
                new = min(new, J1_LIMIT)
            if name == 'grip' and self.object_at is not None:
                if target < self.object_at:
                    self.object_at = max(target, self.object_at - self.creep * 0.02)
                new = max(new, self.object_at)
            self.position[name] = new
        self.pub.publish(JointState(name=list(self.position), position=list(self.position.values())))


@pytest.fixture
def system(tmp_path):
    config = tmp_path / 'config'
    shutil.copytree(FIXTURE, config)
    rclpy.init()
    sim = JointSim()
    server = SkillServer(parameter_overrides=[
        Parameter('config_dir', value=str(config)), Parameter('robot', value='arm_bot'),
        Parameter('skills', value=['move_joints', 'grip'])])
    client = rclpy.create_node('test_client')
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (sim, server, client):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    time.sleep(0.3)
    yield sim, client
    executor.shutdown()
    for node in (sim, server, client):
        node.destroy_node()
    rclpy.shutdown()


def run(client, skill, args, timeout_s=20.0):
    action = ActionClient(client, RunSkill, names.skill_action(skill))
    assert action.wait_for_server(timeout_sec=5)
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args),
                                                  timeout_s=timeout_s))
    while not future.done():
        time.sleep(0.02)
    result = future.result().get_result_async()
    while not result.done():
        time.sleep(0.05)
    return result.result().result


def test_plan_duration_uses_the_slowest_joint():
    p = Params(max_joint_speed=1.0, max_linear_speed=0.1, linear_joints=('lift',))
    assert plan_duration({'j1': 0.0, 'lift': 0.0}, {'j1': 0.5, 'lift': 0.2}, p) == pytest.approx(2.0)
    assert interpolate({'a': 0.0}, {'a': 2.0}, 0.5) == {'a': 1.0}


def test_a_blocked_move_swings_the_elbows_first_then_goes_straight(tmp_path):
    """move_joints / move_gripper to an IK goal with the elbows on the other side: the straight joint move is blocked
    (the hands would swing through each other) and a planner's path wanders, so the elbows swing over first, the hands
    staying, then the arms move straight to the goal (instead of planning), if that is free."""
    from types import SimpleNamespace

    import numpy as np
    from test_cartesian import ChainIk
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.cartesian import elbow_deviation_of, elbow_swing_path
    from taskweave_skills.chain import Chain
    from taskweave_skills.joints import _PoseMotion

    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW)
    ik = ChainIk(chain, NAMES)
    ik.joint_pattern = 'j[1-7]'
    swung, _ = elbow_swing_path(ik, ['arm'], {n: START[n] for n in NAMES}, START, 'down', 0.6)
    goal = dict(zip(swung.names, swung.points[-1], strict=True))           # same hand, elbow over on the down side
    goal['j1'] += 0.05                                                     # and the hand moved a little
    blocked_from = []

    def free(start, end, stop):  # blocked unless the elbow is already near the down side
        blocked_from.append(start)
        return elbow_deviation_of(chain, start, 'down') < math.radians(120)

    skill = _PoseMotion.__new__(_PoseMotion)
    skill.ik, skill.p, skill.name = ik, Params(max_joint_speed=1.0, elbow_tolerance_deg=20.0), 'move_joints'
    skill.planner = SimpleNamespace(free=free)
    logged = []
    skill.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=logged.append))
    ctx = SimpleNamespace(should_stop=False)
    trajectory = skill._swing_first(ctx, dict(START), goal, 'down')
    states = [{**START, **dict(zip(trajectory.names, p, strict=True))} for p in trajectory.points]
    assert all(abs(states[-1][n] - goal[n]) < 1e-9 for n in goal)                # ends on the goal
    hand = [np.linalg.norm(chain.fk(s)[:3, 3] - chain.fk(START)[:3, 3]) for s in states]
    turn = next(i for i, h in enumerate(hand) if h > 1e-3)                     # the hands stay while the elbows swing
    assert elbow_deviation_of(chain, states[turn - 1], 'down') < math.radians(120)
    assert all(b >= a - 1e-9 for a, b in zip(trajectory.times, trajectory.times[1:]))
    skill.planner = SimpleNamespace(free=lambda start, end, stop: False)      # blocked from there too: planner's turn
    assert skill._swing_first(ctx, dict(START), goal, 'down') is None
    skill.planner = SimpleNamespace(free=lambda start, end, stop: True)
    ik.valid = lambda joints, group: False                                   # the swing collides at once: stops short
    assert skill._swing_first(ctx, dict(START), goal, 'down') is None and 'stops short' in logged[-1]
    assert 'the next step would collide' in logged[-1]


def test_the_start_of_a_move_is_the_last_goal_while_the_joints_are_within_tolerance():
    """Joint states settle with noise; IK, planning and paths start from the goal the last move reached, so identical
    runs start from identical joints."""
    from types import SimpleNamespace

    from taskweave_skills.joints import JointInterface

    node = SimpleNamespace(create_publisher=lambda *args: None, create_subscription=lambda *args: None)
    joints = JointInterface(node, {'command': '/c', 'state': '/s'})
    joints._state(JointState(name=['a', 'b', 'c'], position=[1.0004, 2.05, 3.0]))
    joints.reached.update({'a': 1.0, 'b': 2.0})
    assert joints.start(1.0, 0.02) == {'a': 1.0, 'b': 2.05, 'c': 3.0}   # b is 0.05 off its goal: as measured
    assert joints.snapshot(1.0)['a'] == pytest.approx(1.0004)


def test_move_to_named_pose(system):
    sim, client = system
    result = run(client, 'move_joints', {'pose': 'ready'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert sim.position['j1'] == pytest.approx(1.0, abs=0.02)
    assert sim.position['lift'] == pytest.approx(0.2, abs=0.02)
    assert json.loads(result.data_json)['world_state'] == {'arm_pose': 'ready'}


def test_unknown_pose_is_rejected(system):
    sim, client = system
    result = run(client, 'move_joints', {'pose': 'dance'})
    assert result.outcome == RunSkill.Result.REJECTED and "unknown pose 'dance'" in result.message
    assert sim.position['j1'] == 0.0


def test_unreachable_pose_fails_and_names_the_joint(system):
    _, client = system
    result = run(client, 'move_joints', {'pose': 'beyond_limit'})
    assert result.outcome == RunSkill.Result.FAILED and 'j1' in result.message


def test_a_pose_that_places_tools_is_refused_by_move_joints(system):
    sim, client = system
    result = run(client, 'move_joints', {'pose': 'reach'})     # arm_bot's reach has a tool target: move_gripper's
    assert result.outcome == RunSkill.Result.REJECTED and 'places tools' in result.message
    assert sim.position['lift'] == 0.0


def test_grip_close_open_and_holding(system):
    sim, client = system
    result = run(client, 'grip', {'hands': ['left'], 'action': 'close'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert json.loads(result.data_json)['holding'] == {'left': False}
    assert run(client, 'grip', {'hands': ['left'], 'action': 'open'}).outcome == RunSkill.Result.SUCCEEDED
    sim.object_at = 0.4  # an object between the fingers
    result = run(client, 'grip', {'hands': ['left'], 'action': 'close'})
    data = json.loads(result.data_json)
    assert result.outcome == RunSkill.Result.SUCCEEDED and data['holding'] == {'left': True}
    assert data['world_state'] == {'left_gripper': 'holding'}


def test_grip_close_on_a_creeping_object_stops_soon(system):
    sim, client = system
    sim.object_at, sim.creep = 0.6, 0.02  # blocked, giving way slower than stall_speed: no progress either
    start = time.monotonic()
    result = run(client, 'grip', {'hands': ['left'], 'action': 'close'})
    took = time.monotonic() - start
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert json.loads(result.data_json)['holding'] == {'left': True}
    assert took < 4.0 and sim.position['grip'] > 0.5  # planned 0.5 s + stall 1 s, not 30 s of creeping


def test_grip_hands_are_the_robots_grippers(system):
    _, client = system
    result = run(client, 'grip', {'hands': ['right'], 'action': 'close'})  # arm_bot has only a left gripper
    assert result.outcome == RunSkill.Result.REJECTED and 'args.hands' in result.message
    result = run(client, 'grip', {'hands': 'all', 'action': 'close'})
    assert result.outcome == RunSkill.Result.SUCCEEDED and json.loads(result.data_json)['holding'] == {'left': False}


def test_the_old_skills_are_not_in_joints_py_any_more():
    """The skills of joints.py were replaced by move_joints, move_gripper, grip, move_object and set_posture; what is
    left there are their bases (no name, no schema, no execute) and the old class names fail with the way out."""
    from taskweave_core.skill import Skill
    from taskweave_core.skill_server import load_skill_class
    from taskweave_skills import joints

    assert [c.__name__ for c in vars(joints).values()
            if isinstance(c, type) and issubclass(c, Skill) and c is not Skill and (c.name or 'execute' in vars(c))] == []
    for old in ('MoveToPose', 'GoToPoint', 'MoveObject', 'Gripper'):
        assert not hasattr(joints, old)
        with pytest.raises(ValueError, match='was removed.*taskweave config migrate-skills'):
            load_skill_class(f'taskweave_skills.joints:{old}')
    assert load_skill_class('taskweave_skills.joint_motion:MoveJoints').name == 'move_joints'
