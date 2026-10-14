"""Flange target integration of the TCP jog (no ROS needed)."""

import math

import numpy as np
import pytest

from taskweave_skills.tcp_jog import integrate, rotation


def test_linear_velocity_moves_the_flange_in_the_base_frame():
    pose = np.eye(4)
    pose[:3, :3] = rotation(np.array([0, 0, 1.0]), math.pi / 2)  # flange turned: does not matter
    moved = integrate(pose, (0.1, 0.0, -0.05), (0, 0, 0), 0.5)
    assert moved[:3, 3] == pytest.approx([0.05, 0.0, -0.025]) and moved[:3, :3] == pytest.approx(pose[:3, :3])


def test_angular_velocity_turns_about_base_axes_in_place():
    moved = integrate(np.eye(4), (0, 0, 0), (0.0, 0.0, 1.0), math.pi / 2)
    assert moved[:3, 0] == pytest.approx([0, 1, 0], abs=1e-12)  # x turned onto y about z
    assert moved[:3, 3] == pytest.approx([0, 0, 0])


def test_no_rotation_for_zero_rate():
    assert rotation(np.zeros(3), 1.0) == pytest.approx(np.eye(3))


class FakeIk:
    """ik.step: records the start it got; succeeds unless told to fail."""

    def __init__(self, ok=True):
        self.ok, self.starts = ok, []

    def step(self, tool, flange, start):
        self.starts.append(dict(start))
        return dict(start) if self.ok else None


def swing_node(ik):
    from types import SimpleNamespace

    from taskweave_skills.tcp_jog import TcpJog

    node = TcpJog.__new__(TcpJog)  # only _swing: no ROS node
    node.ik, node.held_tools, node.sent = ik, set(), []
    node.command = SimpleNamespace(publish=lambda msg: node.sent.append(dict(zip(msg.name, msg.position))))
    node.JointState = lambda name, position: SimpleNamespace(name=name, position=position)
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None)
    return node


def test_the_elbow_swing_turns_the_swivel_joint_and_keeps_the_flange_target():
    ik = FakeIk()
    node = swing_node(ik)
    current = {f'left_joint{i}': 0.1 * i for i in range(1, 8)}
    jog = {'swivel': None, 'target': np.eye(4), 'blocked': False}
    node._swing('left', jog, current, 0.05)
    assert ik.starts[0]['left_joint3'] == pytest.approx(0.35)     # default: the third arm joint
    assert ik.starts[0]['left_joint2'] == pytest.approx(0.2) and node.sent
    assert np.array_equal(jog['target'], np.eye(4))               # the TCP stays where it was


def test_a_blocked_elbow_swing_sends_nothing():
    node = swing_node(FakeIk(ok=False))
    jog = {'swivel': 'left_joint5', 'target': np.eye(4), 'blocked': False}
    node._swing('left', jog, {f'left_joint{i}': 0.0 for i in range(1, 8)}, 0.05)
    assert jog['blocked'] and not node.sent
