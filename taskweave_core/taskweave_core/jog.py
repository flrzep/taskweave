"""Pad sticks -> a tool's TCP velocity (pure math; operator_joy sends it to a twist servo such as MoveIt Servo).

Axes follow sensor_msgs/Joy conventions for a PS4 pad (tools/joy_sender.py): 0 left stick left/right (+left),
1 left stick up/down (+up), 2 right stick left/right (+left), 3 right stick up/down (+up), 5 R2 (0..1).

Move (default): the TCP moves in the robot's base frame (left stick: forward/back, left/right; right stick up/down:
up/down) and right stick left/right turns the gripper about its approach axis. Tilt (R2 held): the sticks turn the
gripper about the TCP: left stick up/down tilts the approach up/down, left/right tilts it left/right.

A servo moves the flange (the tool's link), so the command is the flange's velocity, in the base frame: the TCP's
velocity, the TCP-frame rotation turned into base axes, and the flange velocity that keeps the TCP itself still while
the gripper turns about it (the TCP sits ``r`` from the flange). MoveIt Servo applies the rotation about the flange
(apply_twist_commands_about_ee_frame); it ignores twists given in the flange frame itself, hence the base frame.
"""

from __future__ import annotations

import math

Vec = tuple[float, float, float]


def rpy_rotation(rpy) -> list[list[float]]:
    (cr, cp, cy), (sr, sp, sy) = [math.cos(a) for a in rpy], [math.sin(a) for a in rpy]
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def quaternion_rotation(x, y, z, w) -> list[list[float]]:
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def apply(m, v) -> Vec:
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def cross(a, b) -> Vec:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def stick_velocity(axes, tilt: bool, speed: float, turn: float) -> tuple[Vec, Vec]:
    """(linear velocity in the base frame, angular velocity in the TCP frame) from the pad axes."""
    a = list(axes) + [0.0] * (6 - len(axes))
    if tilt:  # TCP frame: z = approach, y = closing, x = y cross z
        return (0.0, 0.0, 0.0), (-a[0] * turn, -a[1] * turn, a[2] * turn)
    return (a[1] * speed, a[0] * speed, a[3] * speed), (0.0, 0.0, a[2] * turn)


def flange_twist(linear_base: Vec, angular_tcp: Vec, base_to_flange, tool_frame: dict) -> tuple[Vec, Vec]:
    """Flange (linear, angular) velocity in the base frame for a base-frame TCP velocity and a TCP-frame rotation.

    ``base_to_flange``: rotation matrix of the flange in the base frame. ``tool_frame``: profile tool_frames entry
    (xyz, rpy of the TCP relative to the flange)."""
    angular = apply(base_to_flange, apply(rpy_rotation(tool_frame.get('rpy', [0, 0, 0])), angular_tcp))
    r = apply(base_to_flange, tuple(float(v) for v in tool_frame.get('xyz', [0, 0, 0])))  # flange -> TCP, base axes
    # flange velocity = TCP velocity - angular x r (the TCP turns in place)
    turn = cross(angular, r)
    return tuple(v - t for v, t in zip(linear_base, turn, strict=True)), angular
