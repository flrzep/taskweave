"""move_to_pose and gripper against a small joint simulator (targets in, states out)."""

import json
import math
import shutil
import threading
import time
from pathlib import Path

import pytest
from conftest import connect, wait_ready, wait_until

rclpy = pytest.importorskip('rclpy')

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
        Parameter('skills', value=['move_to_pose', 'gripper'])])
    client = rclpy.create_node('test_client')
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (sim, server, client):
        executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    wait_ready(server)
    client.taskweave_server = server  # for tests that wait for what the server has received
    yield sim, client
    executor.shutdown()
    for node in (sim, server, client):
        node.destroy_node()
    rclpy.shutdown()


def run(client, skill, args, timeout_s=20.0):
    action = connect(client, RunSkill, names.skill_action(skill))
    future = action.send_goal_async(RunSkill.Goal(step_id='t', skill=skill, args_json=json.dumps(args),
                                                  timeout_s=timeout_s))
    wait_until(future.done, what='the goal to be accepted')
    result = future.result().get_result_async()
    wait_until(result.done, timeout=timeout_s + 30, what='the skill result')
    return result.result().result


def test_plan_duration_uses_the_slowest_joint():
    p = Params(max_joint_speed=1.0, max_linear_speed=0.1, linear_joints=('lift',))
    assert plan_duration({'j1': 0.0, 'lift': 0.0}, {'j1': 0.5, 'lift': 0.2}, p) == pytest.approx(2.0)
    assert interpolate({'a': 0.0}, {'a': 2.0}, 0.5) == {'a': 1.0}


def test_a_blocked_move_swings_the_elbows_first_then_goes_straight(tmp_path):
    """move_to_pose / go_to_point to an IK goal with the elbows on the other side: the straight joint move is blocked
    (the hands would swing through each other) and a planner's path wanders, so the elbows swing over first, the hands
    staying, then the arms move straight to the goal (instead of planning), if that is free."""
    from types import SimpleNamespace

    import numpy as np
    from test_cartesian import ChainIk
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.cartesian import elbow_deviation_of, elbow_swing_path
    from taskweave_skills.chain import Chain
    from taskweave_skills.joints import MoveToPose

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

    skill = MoveToPose.__new__(MoveToPose)
    skill.ik, skill.p, skill.name = ik, Params(max_joint_speed=1.0, elbow_tolerance_deg=20.0), 'move_to_pose'
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


def test_an_elbow_swing_that_stops_short_fails_before_moving(tmp_path):
    """move_object elbow: elbows that would end further than elbow_tolerance_deg from the side fail the step (where they
    stop and why), instead of a partial swing reported as success."""
    from types import SimpleNamespace

    from test_cartesian import ChainIk
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.chain import Chain
    from taskweave_skills.joints import MoveObject

    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    ik = ChainIk(Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW), NAMES)
    ik.joint_pattern = 'j[1-7]'
    moved = []
    skill = MoveObject.__new__(MoveObject)
    skill.ik, skill.p = ik, Params(max_joint_speed=1.0, elbow_tolerance_deg=20.0)
    skill.joints = SimpleNamespace(start=lambda max_age, tolerance: dict(START))
    skill._move = lambda ctx, goal, label, path=None: moved.append(goal) or (None, goal)
    skill._check_hands = lambda goals, label: None
    ctx = SimpleNamespace(should_stop=False, feedback=lambda fraction, text: None)
    hold = {'tools': ['arm'], 'object': 'box', 'object_in_base': None, 'tcp_in_object': {}}
    result = skill._swing_elbows(ctx, hold, 'down')                          # the wrist reaches its limit halfway
    assert result.outcome.name == 'FAILED' and not moved
    assert result.message == ('elbows of arm down: the elbows would stop more than 20 deg off down: arm 173 -> 53 deg '
                              'off (j5 at its limit); not moved')
    ik.valid = lambda joints, group: False                                   # every step collides
    result = skill._swing_elbows(ctx, hold, 'out')
    assert result.outcome.name == 'FAILED' and not moved and 'the next step would collide' in result.message
    ik.valid = lambda joints, group: True
    result = skill._swing_elbows(ctx, hold, 'out')
    assert result.outcome.name == 'SUCCEEDED' and len(moved) == 1, result.message
    assert result.message == 'elbows of arm out: arm 83 -> 6 deg off out'


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
    result = run(client, 'move_to_pose', {'pose': 'ready'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert sim.position['j1'] == pytest.approx(1.0, abs=0.02)
    assert sim.position['lift'] == pytest.approx(0.2, abs=0.02)
    assert json.loads(result.data_json)['world_state'] == {'arm_pose': 'ready'}


def test_unknown_pose_is_rejected(system):
    sim, client = system
    result = run(client, 'move_to_pose', {'pose': 'dance'})
    assert result.outcome == RunSkill.Result.REJECTED and "unknown pose 'dance'" in result.message
    assert sim.position['j1'] == 0.0


def test_unreachable_pose_fails_and_names_the_joint(system):
    _, client = system
    result = run(client, 'move_to_pose', {'pose': 'beyond_limit'})
    assert result.outcome == RunSkill.Result.FAILED and 'j1' in result.message


def test_gripper_close_open_and_holding(system):
    sim, client = system
    result = run(client, 'gripper', {'side': 'left', 'action': 'close'})
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert json.loads(result.data_json)['holding'] == {'left': False}
    assert run(client, 'gripper', {'side': 'left', 'action': 'open'}).outcome == RunSkill.Result.SUCCEEDED
    sim.object_at = 0.4  # an object between the fingers
    result = run(client, 'gripper', {'side': 'left', 'action': 'close'})
    data = json.loads(result.data_json)
    assert result.outcome == RunSkill.Result.SUCCEEDED and data['holding'] == {'left': True}
    assert data['world_state'] == {'left_gripper': 'holding'}


def test_gripper_close_on_a_creeping_object_stops_soon(system):
    sim, client = system
    sim.object_at, sim.creep = 0.6, 0.02  # blocked, giving way slower than stall_speed: no progress either
    start = time.monotonic()
    result = run(client, 'gripper', {'side': 'left', 'action': 'close'})
    took = time.monotonic() - start
    assert result.outcome == RunSkill.Result.SUCCEEDED, result.message
    assert json.loads(result.data_json)['holding'] == {'left': True}
    assert took < 12.0 and sim.position['grip'] > 0.5  # planned 0.5 s + stall 1 s, not 30 s of creeping


def test_while_holding_a_pose_with_arm_joints_is_refused(system):
    from taskweave_skills.hold import HoldState

    sim, client = system
    holds = list(HoldState._instances.values())  # the server's, and those of earlier tests' servers
    for hold in holds:
        hold.hold = {'object': 'box', 'point': 'box/top', 'tools': ['left'], 'object_in_base': None,
                     'tcp_in_object': {}}
    try:  # arm_bot has no ik section: every joint counts as an arm joint
        result = run(client, 'move_to_pose', {'pose': 'ready'})
    finally:
        for hold in holds:
            hold.hold = None
    assert result.outcome == RunSkill.Result.REJECTED and 'holding box' in result.message
    assert sim.position['j1'] == 0.0


def test_gripper_sides_are_the_robots_grippers(system):
    _, client = system
    result = run(client, 'gripper', {'side': 'right', 'action': 'close'})  # arm_bot has only a left gripper
    assert result.outcome == RunSkill.Result.REJECTED and 'left' in result.message
    result = run(client, 'gripper', {'side': 'both', 'action': 'close'})
    assert result.outcome == RunSkill.Result.SUCCEEDED and json.loads(result.data_json)['holding'] == {'left': False}
