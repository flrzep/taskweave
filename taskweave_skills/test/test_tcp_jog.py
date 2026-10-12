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
