"""move_to_pose and gripper against a small joint simulator (targets in, states out)."""

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
        Parameter('skills', value=['move_to_pose', 'gripper'])])
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
    """move_to_pose / go_to_point to an IK goal with the elbows on the other side: the straight joint move is blocked
    (the hands would swing through each other) and a planner's path wanders, so the elbows swing over first, the hands
    staying, then the arms move straight to the goal (instead of planning), if that is free."""
    from types import SimpleNamespace

    import numpy as np
    from test_cartesian import ChainIk
    from test_chain import NAMES, START, URDF

    from taskweave_skills.cartesian import elbow_deviation_of, elbow_swing_path
    from taskweave_skills.chain import Chain
    from taskweave_skills.joints import MoveToPose

    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange')
    ik = ChainIk(chain, NAMES)
    ik.joint_pattern = 'j[1-7]'
    swung, _ = elbow_swing_path(ik, ['arm'], {n: START[n] for n in NAMES}, START, 'down', 0.6)
    goal = dict(zip(swung.names, swung.points[-1], strict=True))           # same hand, elbow over on the down side
    goal['j1'] += 0.05                                                     # and the hand moved a little
    blocked_from = []

    def free(start, end, stop):  # blocked unless the elbow is already near the down side
        blocked_from.append(start)
        return elbow_deviation_of(chain, NAMES, start, 'down') < math.radians(120)

    skill = MoveToPose.__new__(MoveToPose)
    skill.ik, skill.p, skill.name = ik, Params(max_joint_speed=1.0), 'move_to_pose'
    skill.planner = SimpleNamespace(free=free)
    skill.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=lambda message: None))
    ctx = SimpleNamespace(should_stop=False)
    trajectory = skill._swing_first(ctx, dict(START), goal, 'down')
    states = [{**START, **dict(zip(trajectory.names, p, strict=True))} for p in trajectory.points]
    assert all(abs(states[-1][n] - goal[n]) < 1e-9 for n in goal)                # ends on the goal
    hand = [np.linalg.norm(chain.fk(s)[:3, 3] - chain.fk(START)[:3, 3]) for s in states]
    turn = next(i for i, h in enumerate(hand) if h > 1e-3)                     # the hands stay while the elbows swing
    assert elbow_deviation_of(chain, NAMES, states[turn - 1], 'down') < math.radians(120)
    assert all(b >= a - 1e-9 for a, b in zip(trajectory.times, trajectory.times[1:]))
    skill.planner = SimpleNamespace(free=lambda start, end, stop: False)      # blocked from there too: planner's turn
    assert skill._swing_first(ctx, dict(START), goal, 'down') is None


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
    assert took < 4.0 and sim.position['grip'] > 0.5  # planned 0.5 s + stall 1 s, not 30 s of creeping


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
