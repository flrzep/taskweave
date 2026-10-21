import math

import numpy as np
import pytest

from taskweave_skills.chain import Chain, Joint, pose_error

AXES = ['0 0 1', '0 1 0', '0 0 1', '0 1 0', '0 0 1', '0 1 0', '0 0 1']
JOINTS = ''.join(f"""
  <joint name="j{i + 1}" type="revolute"><parent link="l{i}"/><child link="l{i + 1}"/>
    <origin xyz="0 0 {0.0 if i == 0 else 0.2}" rpy="0 0 0"/><axis xyz="{axis}"/>
    <limit lower="-2.5" upper="2.5" effort="1" velocity="1"/></joint>""" for i, axis in enumerate(AXES))
URDF = f"""<robot name="arm">
  <link name="base"/>{''.join(f'<link name="l{i}"/>' for i in range(8))}
  <joint name="lift" type="prismatic"><parent link="base"/><child link="l0"/><origin xyz="0 0 0.5"/>
    <axis xyz="0 0 1"/><limit lower="0" upper="0.5" effort="1" velocity="1"/></joint>{JOINTS}
  <link name="flange"/><joint name="tool" type="fixed"><parent link="l7"/><child link="flange"/>
    <origin xyz="0 0 0.1" rpy="0 1.5708 0"/></joint>
</robot>"""
NAMES = [f'j{i}' for i in range(1, 8)]
ELBOW = ('l2', 'l4', 'l6')  # shoulder, elbow and wrist centres (the frames of j2, j4, j6)
LIMITS = {name: (-2.5, 2.5) for name in NAMES}
START = {'lift': 0.2, 'j1': 0.1, 'j2': 0.5, 'j3': -0.2, 'j4': 1.0, 'j5': 0.3, 'j6': 0.6, 'j7': -0.1}


@pytest.fixture
def chain(tmp_path):
    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    return Chain.from_urdf(path, 'base', 'flange', elbow=ELBOW)


def test_chain_follows_the_urdf_and_its_jacobian_matches_finite_differences(chain):
    assert chain.movable == ['lift', *NAMES]
    zero = chain.fk({})
    assert np.allclose(zero[:3, 3], [0, 0, 0.5 + 6 * 0.2 + 0.1])
    jacobian, tip = chain.jacobian(START, NAMES)
    for column, name in enumerate(NAMES):
        moved = chain.fk({**START, name: START[name] + 1e-6})
        assert np.allclose(pose_error(moved, tip) / 1e-6, jacobian[:, column], atol=1e-4)


def test_a_step_moves_the_tip_toward_the_target(chain):
    tip = chain.fk(START)
    target = tip.copy()
    target[:3, 3] += [0.01, -0.01, 0.005]
    joints, held = chain.solve_step(START, target, NAMES, LIMITS)
    assert held == '' and np.linalg.norm(pose_error(target, chain.fk({**START, **joints}))) < 1e-3


def test_a_joint_at_its_limit_is_held_and_the_others_carry_on(chain):
    start = {**START, 'j4': 2.5 - 1e-3}
    tip = chain.fk(start)
    target = tip.copy()
    target[:3, 3] += [0.0, 0.0, -0.01]
    for _ in range(5):  # a few steps, as the jog does
        joints, held = chain.solve_step(start, target, NAMES, LIMITS)
        start = {**start, **joints}
    assert all(LIMITS[n][0] <= v <= LIMITS[n][1] for n, v in joints.items())
    assert np.linalg.norm(pose_error(target, chain.fk(start))[:3]) < 2e-3


def test_the_elbow_swings_while_the_tip_stays(chain):
    positions = dict(START)
    tip = chain.fk(positions)
    previous = None
    for _ in range(20):
        direction = chain.null_direction(positions, NAMES)
        if previous is not None and direction @ previous < 0:  # the jog keeps the way, as here
            direction = -direction
        previous = direction
        joints, _ = chain.solve_step(positions, tip, NAMES, LIMITS, null=direction * 0.03)
        positions = {**positions, **joints}
    assert abs(positions['j3'] - START['j3']) > 0.05 or abs(positions['j1'] - START['j1']) > 0.05
    assert np.linalg.norm(pose_error(tip, chain.fk(positions))) < 2e-3


def test_link_points_are_where_the_joint_frames_are(chain):
    frames = dict(zip(chain.movable, chain.frames(START)[0], strict=True))
    points = chain.link_points(START, ('l2', 'l4', 'l6', 'flange'))
    assert all(np.array_equal(p, frames[j][:3, 3]) for p, j in zip(points, ('j2', 'j4', 'j6'), strict=False))
    assert np.array_equal(points[3], chain.fk(START)[:3, 3])
    with pytest.raises(KeyError):
        chain.link_points(START, ('nowhere',))


def test_the_step_uses_the_chains_tuning(tmp_path):
    from taskweave_core.config.tuning import motion_tuning

    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    small = Chain.from_urdf(path, 'base', 'flange', tuning={**motion_tuning()['chain'], 'max_step_rad': 0.01})
    target = small.fk(START)
    target[:3, 3] += [0.05, 0.0, 0.0]
    joints, _ = small.solve_step(START, target, NAMES, LIMITS)
    assert max(abs(joints[n] - START[n]) for n in NAMES) == pytest.approx(0.01)   # the step is capped there


def test_a_joint_motion_is_what_the_joint_does_to_everything_it_carries():
    lift = Joint('lift', 'prismatic', np.eye(4), np.array([0.0, 0.0, 1.0]), 'column')
    turn = Joint('turn', 'revolute', np.eye(4), np.array([0.0, 0.0, 1.0]), 'head')
    column = Chain([lift, turn])
    move = column.joint_motion({'lift': 0.5, 'turn': 0.0}, 'lift', 0.1)
    assert np.allclose(move[:3, :3], np.eye(3)) and np.allclose(move[:3, 3], [0.0, 0.0, -0.4])
    spin = column.joint_motion({'lift': 0.5, 'turn': 0.0}, 'turn', math.pi / 2)
    assert np.allclose(spin @ [1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0])
    assert column.joint_motion({}, 'nope', 1.0) is None
