"""Pad sticks -> flange twist (taskweave_core.jog)."""

import math

import pytest

from taskweave_core.jog import apply, flange_twist, rpy_rotation, stick_velocity

# S1-like tool: TCP 0.28 m along -x of the flange, z (approach) = -x of the flange
TOOL = {'link': 'left_link7', 'xyz': [-0.28, 0.0, 0.0], 'rpy': [math.pi / 2, 0.0, -math.pi / 2]}
IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def test_move_mode_maps_sticks_to_base_axes():
    linear, angular = stick_velocity([0.5, 1.0, 0.0, -1.0, 0.0, 0.0], tilt=False, speed=0.1, turn=0.3)
    assert linear == pytest.approx((0.1, 0.05, -0.1))  # forward, left, down
    assert angular == (0.0, 0.0, 0.0)
    _, angular = stick_velocity([0, 0, 1.0, 0, 0, 0], tilt=False, speed=0.1, turn=0.3)
    assert angular == pytest.approx((0, 0, 0.3))  # right stick: turn about the approach axis


def test_tilt_mode_only_rotates():
    linear, angular = stick_velocity([1.0, 1.0, 0, 1.0, 0, 1.0], tilt=True, speed=0.1, turn=0.3)
    assert linear == (0.0, 0.0, 0.0) and angular == pytest.approx((-0.3, -0.3, 0.0))


def test_translation_stays_in_the_base_frame():
    flange_in_base = rpy_rotation([0.0, 0.0, math.pi / 2])  # the flange's orientation does not matter for it
    v, w = flange_twist((0.0, 0.1, 0.0), (0, 0, 0), flange_in_base, TOOL)
    assert v == pytest.approx((0.0, 0.1, 0.0)) and w == pytest.approx((0, 0, 0))


def test_rotation_is_turned_into_base_axes():
    flange_in_base = rpy_rotation([0.0, 0.0, math.pi / 2])
    _, w = flange_twist((0, 0, 0), (0, 0, 1.0), flange_in_base, TOOL)  # about the approach = -x of the flange
    assert w == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)  # flange x = base y


def test_turning_keeps_the_tcp_in_place():
    v, w = flange_twist((0, 0, 0), (0.0, 0.5, 0.0), IDENTITY, TOOL)  # tilt about the TCP's y axis
    r = TOOL['xyz']
    tcp_velocity = [a + b for a, b in zip(v, (w[1] * r[2] - w[2] * r[1], w[2] * r[0] - w[0] * r[2],
                                                 w[0] * r[1] - w[1] * r[0]), strict=True)]
    assert tcp_velocity == pytest.approx((0, 0, 0), abs=1e-12) and any(abs(c) > 0.01 for c in v)


def test_approach_turn_is_about_the_flange_axis_the_tcp_lies_on():
    v, w = flange_twist((0, 0, 0), (0, 0, 1.0), IDENTITY, TOOL)
    assert w == pytest.approx(apply(rpy_rotation(TOOL['rpy']), (0, 0, 1.0)))
    assert w == pytest.approx((-1.0, 0, 0), abs=1e-9)  # approach = -x of the flange
    assert v == pytest.approx((0, 0, 0), abs=1e-9)  # the TCP is on that axis: no compensation
