"""Straight-line TCP paths (taskweave_skills.cartesian) with a fake IK."""

import math

import numpy as np
import pytest

from taskweave_skills.cartesian import PathError, interpolate, linear_path, matching_flip, rotation_angle, slerp
from taskweave_skills.tcp_jog import rotation


def pose(x, y, z, yaw=0.0):
    m = np.eye(4)
    m[:3, :3] = rotation(np.array([0, 0, yaw]), 1.0)
    m[:3, 3] = [x, y, z]
    return m


class FakeIk:
    """One joint per tool that equals the flange's x; refuses x beyond ``limit``."""
    tools = {'left': {'link': 'l7', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]},
             'right': {'link': 'r7', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}}

    def __init__(self, limit=10.0):
        self.limit, self.calls = limit, 0

    def step(self, tool, flange, current):
        self.calls += 1
        return None if flange[0, 3] > self.limit else {f'{tool}_j': float(flange[0, 3])}


def test_slerp_and_interpolate():
    r0, r1 = np.eye(3), rotation(np.array([0, 0, 1.0]), math.pi / 2)
    assert slerp(r0, r1, 0.5) == pytest.approx(rotation(np.array([0, 0, 1.0]), math.pi / 4))
    assert slerp(r0, r1, 1.0) == pytest.approx(r1)
    mid = interpolate(pose(0, 0, 0), pose(1, 0, 0.2, math.pi / 2), 0.5)
    assert mid[:3, 3] == pytest.approx([0.5, 0, 0.1]) and rotation_angle(mid[:3, :3]) == pytest.approx(math.pi / 4)


def test_both_tools_move_in_lockstep_at_the_tcp_speed():
    ik = FakeIk()
    starts = {'left': pose(0.5, 0.2, 0), 'right': pose(0.5, -0.2, 0)}
    goals = {'left': pose(0.6, 0.2, 0), 'right': pose(0.6, -0.2, 0)}
    path = linear_path(ik, starts, goals, {'left_j': 0.5, 'right_j': 0.5}, speed=0.05, turn=0.3, step_m=0.01)
    assert path.names == ['left_j', 'right_j'] and len(path.points) == 11
    assert path.points[-1] == pytest.approx([0.6, 0.6]) and path.points[5] == pytest.approx([0.55, 0.55])
    assert path.duration == pytest.approx(0.1 / 0.05)


def test_a_step_without_solution_fails_before_moving():
    with pytest.raises(PathError, match='step 6 of 10'):
        linear_path(FakeIk(limit=0.555), {'left': pose(0.5, 0, 0)}, {'left': pose(0.6, 0, 0)}, {'left_j': 0.5},
                    speed=0.05, turn=0.3)


def test_a_flippable_goal_is_turned_the_way_the_tcp_already_is():
    start, goal = np.eye(4), np.eye(4)
    start[:3, :3] = np.diag([-1.0, -1.0, 1.0])  # reached with the grasp turned 180 deg about the approach
    goal[:3, 3] = [0.0, 0.0, -0.1]
    out = matching_flip({'left': goal, 'right': goal}, {'left': start, 'right': start}, {'left': {'flip': True}})
    assert np.allclose(out['left'][:3, :3], start[:3, :3]) and np.allclose(out['left'][:3, 3], goal[:3, 3])
    assert out['right'] is goal  # not flippable: unchanged


class ChainIk:
    """IkClient stand-in with a URDF chain (test_chain's 7-joint arm on a lift) and no collisions."""

    def __init__(self, chain, names):
        self.tools = {'arm': {'link': 'flange', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}}
        self.chains, self.groups, self.choose = {'arm': (chain, names)}, {'arm': 'arm'}, None
        self.limits = {name: (-2.5, 2.5) for name in names}
        self.checked = 0

    def valid(self, joints, group):
        self.checked += 1
        return True


def test_a_straight_line_on_the_chain_moves_joints_away_from_their_limits(tmp_path):
    from test_chain import NAMES, START, URDF

    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange')
    ik = ChainIk(chain, NAMES)
    start_state = {**START, 'j3': 2.45}                      # j3 (upper-arm roll) 0.05 rad from its limit
    start = chain.fk(start_state)
    goal = start.copy()
    goal[:3, 3] += [0.0, 0.05, -0.04]
    trajectory = linear_path(ik, {'arm': start}, {'arm': goal}, {n: start_state[n] for n in NAMES}, 0.05, 0.3,
                             state=start_state)
    for point in trajectory.points:                          # every step on the straight line
        tip = chain.fk({**start_state, **dict(zip(trajectory.names, point, strict=True))})[:3, 3]
        offset = tip - start[:3, 3]
        along = offset @ (goal[:3, 3] - start[:3, 3]) / np.linalg.norm(goal[:3, 3] - start[:3, 3]) ** 2
        assert np.linalg.norm(offset - along * (goal[:3, 3] - start[:3, 3])) < 1e-3
    end = dict(zip(trajectory.names, trajectory.points[-1], strict=True))
    assert np.allclose(chain.fk({**start_state, **end})[:3, 3], goal[:3, 3], atol=1e-3)
    assert end['j3'] < 2.45 - 0.05                           # pushed back from the limit through the null space
    assert ik.checked == len(trajectory.points) - 1          # each step checked for collisions
