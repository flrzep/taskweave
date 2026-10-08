"""The navigate_to control law, simulated without ROS."""

import math

import pytest

pytest.importorskip('geometry_msgs')  # the skill module imports ROS messages

from taskweave_core.config import Pose2D, wrap_angle  # noqa: E402
from taskweave_skills.navigate import Params, command  # noqa: E402


def simulate(start, target, base_type, params=None, steps=2000, dt=0.05):
    p = params or Params()
    pose, cmd = start, (0.0, 0.0, 0.0)
    for _ in range(steps):
        cmd = command(pose, target, base_type, p, cmd, dt)
        vx, vy, wz = cmd
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        pose = Pose2D(pose.x + (c * vx - s * vy) * dt, pose.y + (s * vx + c * vy) * dt, wrap_angle(pose.yaw + wz * dt))
        if base_type == 'differential':
            assert vy == 0.0
        assert abs(vx) <= p.max_linear + 1e-9 and abs(wz) <= p.max_angular + 1e-9
    return pose


@pytest.mark.parametrize('base_type', ['holonomic', 'differential'])
@pytest.mark.parametrize('target', [Pose2D(2.0, 1.0, 1.57), Pose2D(-1.0, -2.0, -3.0), Pose2D(0.0, 0.0, 3.1)])
def test_converges_to_target(base_type, target):
    pose = simulate(Pose2D(0.0, 0.0, 0.0), target, base_type)
    assert math.hypot(pose.x - target.x, pose.y - target.y) < 0.03
    assert abs(wrap_angle(pose.yaw - target.yaw)) < 0.03


def test_acceleration_is_limited():
    p = Params(linear_accel=0.5)
    vx, _, _ = command(Pose2D(0, 0, 0), Pose2D(5, 0, 0), 'holonomic', p, (0.0, 0.0, 0.0), 0.1)
    assert vx == pytest.approx(0.05)


def test_params_capped_by_robot_limits_and_unknown_keys_rejected():
    p = Params.from_config({'max_linear': 2.0}, {'linear': 0.8, 'angular': 0.5})
    assert p.max_linear == 0.8 and p.max_angular == 0.5
    with pytest.raises(ValueError, match='max_speed'):
        Params.from_config({'max_speed': 1.0}, {})


@pytest.mark.parametrize('offset', [0.02, 0.025, 0.029])
def test_differential_settles_from_inside_the_tolerance_band(offset):
    """Regression: starting just inside the position tolerance, the base must turn to the heading and stop."""
    p = Params()
    target = Pose2D(1.0, 0.0, 1.57)
    pose = simulate(Pose2D(1.0 - offset, 0.0, 0.0), target, 'differential', p, steps=400)
    assert math.hypot(pose.x - target.x, pose.y - target.y) <= p.position_tolerance
    assert abs(wrap_angle(pose.yaw - target.yaw)) < p.yaw_tolerance
