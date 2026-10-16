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
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW)
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


def test_the_elbow_swings_toward_a_side_with_the_hand_staying(tmp_path):
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.cartesian import elbow_deviation_of, elbow_swing_path
    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW)
    ik = ChainIk(chain, NAMES)
    arm = {n: START[n] for n in NAMES}
    sides = [s for s in ('down', 'up', 'out', 'in') if elbow_deviation_of(chain, START, s) > 0.6]
    assert len(sides) >= 2                                                          # the elbow is off those sides
    for side in sides:
        before = elbow_deviation_of(chain, START, side)
        trajectory, report = elbow_swing_path(ik, ['arm'], arm, START, side, 0.6)
        end = {**START, **dict(zip(trajectory.names, trajectory.points[-1], strict=True))}
        assert np.allclose(chain.fk(end)[:3, 3], chain.fk(START)[:3, 3], atol=1e-3)     # the hand stays
        assert elbow_deviation_of(chain, end, side) < before - 0.2              # the elbow went that way
        assert report['arm'][1] < report['arm'][0]
        assert all(-2.5 <= end[n] <= 2.5 for n in NAMES)


def test_an_elbow_opposite_its_side_swings_round_the_outside_and_stops_where_asked(tmp_path):
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.cartesian import elbow_deviation_of, elbow_swing_path
    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW)
    ik = ChainIk(chain, NAMES)
    arm = {n: START[n] for n in NAMES}
    assert elbow_deviation_of(chain, START, 'down') > math.radians(150)    # elbow up: nearly opposite down
    trajectory, report = elbow_swing_path(ik, ['arm'], arm, START, 'down', 0.6)
    states = [{**START, **dict(zip(trajectory.names, p, strict=True))} for p in trajectory.points]
    assert min(elbow_deviation_of(chain, s, 'out') for s in states) < math.radians(20)   # past the outside
    assert min(elbow_deviation_of(chain, s, 'in') for s in states) > math.radians(80)    # never the inside
    assert all(np.allclose(chain.fk(s)[:3, 3], chain.fk(START)[:3, 3], atol=1e-3) for s in states)
    assert report['arm'][1] < 90
    trajectory, report = elbow_swing_path(ik, ['arm'], arm, START, 'down', 0.6, settled={'arm': math.radians(110)})
    assert 100 <= report['arm'][1] <= 110                                           # as far as asked, no further


def test_a_swing_that_stops_short_says_where_and_why(tmp_path):
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_skills.cartesian import elbow_swing_path, swing_short
    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW)
    ik = ChainIk(chain, NAMES)
    arm = {n: START[n] for n in NAMES}
    _, report = elbow_swing_path(ik, ['arm'], arm, START, 'out', 0.6)
    assert report['arm'][1] < 20 and report['arm'][2] == '' and swing_short(report, 20) == ''   # got there
    _, report = elbow_swing_path(ik, ['arm'], arm, START, 'down', 0.6)                          # the wrist cannot
    assert report['arm'] == (173, 53, 'j5 at its limit')
    assert swing_short(report, 20) == 'arm 173 -> 53 deg off (j5 at its limit)'
    assert swing_short(report, 20, {'arm': 40}) == ''                     # within 20 deg of where it was to stop
    ik.valid = lambda joints, group: False                                # every step collides
    trajectory, report = elbow_swing_path(ik, ['arm'], arm, START, 'out', 0.6)
    assert trajectory is None and report['arm'] == (83, 83, 'the next step would collide')


def test_an_arm_without_an_elbow_description_has_no_elbow_control(tmp_path):
    from test_chain import NAMES, START, URDF

    from taskweave_core.config.tuning import motion_tuning
    from taskweave_skills.cartesian import elbow_push, elbow_swing_path
    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange')                                 # no elbow links
    ik = ChainIk(chain, NAMES)
    assert not np.any(elbow_push(chain, NAMES, START, 'down', motion_tuning()['elbow']))
    with pytest.raises(PathError, match='no elbow control for arm: the robot profile has no arms.<tool>.elbow'):
        elbow_swing_path(ik, ['arm'], {n: START[n] for n in NAMES}, START, 'down', 0.6)
    start = chain.fk(START)
    goal = start.copy()
    goal[:3, 3] += [0.0, 0.0, -0.03]
    arm = {n: START[n] for n in NAMES}
    with_side = linear_path(ik, {'arm': start}, {'arm': goal}, arm, 0.05, 0.3, state=START, elbow='down')
    plain = linear_path(ik, {'arm': start}, {'arm': goal}, arm, 0.05, 0.3, state=START)
    assert with_side.points == plain.points                                         # the side is simply not used


def test_no_spare_freedom_no_elbow_push(tmp_path):
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_core.config.tuning import motion_tuning
    from taskweave_skills.cartesian import elbow_push
    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    chain = Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW)
    tuning = motion_tuning()['elbow']
    assert np.any(elbow_push(chain, NAMES, START, 'down', tuning))                 # 7 joints: one self-motion
    assert not np.any(elbow_push(chain, NAMES[:6], START, 'down', tuning))         # 6: none


def test_the_profiles_elbow_rate_reaches_the_swing(tmp_path):
    from test_chain import ELBOW, NAMES, START, URDF

    from taskweave_core.config.tuning import motion_tuning
    from taskweave_skills.cartesian import elbow_swing_path
    from taskweave_skills.chain import Chain
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    ik = ChainIk(Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW), NAMES)
    arm = {n: START[n] for n in NAMES}
    usual, _ = elbow_swing_path(ik, ['arm'], arm, START, 'out', 0.6)
    ik.tuning = motion_tuning({'motion_tuning': {'elbow': {'rate_rad': 0.015}}})   # as IkClient has it from a profile
    slow, report = elbow_swing_path(ik, ['arm'], arm, START, 'out', 0.6)
    assert report['arm'][2] == '' and len(slow.points) > 1.5 * len(usual.points)  # half the rate: about twice the steps
    ik.tuning = motion_tuning({'motion_tuning': {'elbow': {'swing_max_steps': 3}}})
    _, report = elbow_swing_path(ik, ['arm'], arm, START, 'out', 0.6)
    assert report['arm'][2] == 'all 3 steps used'
