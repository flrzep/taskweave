"""Pose types used in config. Plain dataclasses: no ROS dependency."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    """Planar pose (metres, radians) in a named frame, e.g. a base location in the cell frame."""

    x: float
    y: float
    yaw: float
    frame: str = 'cell'

    @classmethod
    def from_dict(cls, data: dict) -> Pose2D:
        return cls(float(data['x']), float(data['y']), float(data['yaw']), data.get('frame', 'cell'))

    def to_dict(self) -> dict:
        return {'frame': self.frame, 'x': round(self.x, 4), 'y': round(self.y, 4), 'yaw': round(self.yaw, 4)}

    def compose(self, offset: Pose2D) -> Pose2D:
        """``offset`` (given in this pose's frame) expressed in this pose's parent frame: self * offset."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return Pose2D(self.x + c * offset.x - s * offset.y, self.y + s * offset.x + c * offset.y,
                      wrap_angle(self.yaw + offset.yaw), self.frame)

    def relative_to(self, reference: Pose2D, frame: str | None = None) -> Pose2D:
        """This pose expressed in ``reference``'s frame (the inverse of ``compose``): reference^-1 * self."""
        dx, dy = self.x - reference.x, self.y - reference.y
        c, s = math.cos(reference.yaw), math.sin(reference.yaw)
        return Pose2D(c * dx + s * dy, -s * dx + c * dy, wrap_angle(self.yaw - reference.yaw), frame or reference.frame)


@dataclass(frozen=True)
class ObjectInstance:
    """A placed object in the cell: its type (``objects/<object>.yaml``), planar pose and height. A stack of
    ``stack_count`` nested items ``stack_pitch`` apart starts at ``z``; its points refer to the top item."""

    name: str
    object: str
    pose: Pose2D
    z: float = 0.0
    stack_count: int = 1
    stack_pitch: float = 0.0

    @property
    def top_z(self) -> float:
        """Height of the top item (the one a robot reaches first)."""
        return self.z + max(self.stack_count - 1, 0) * self.stack_pitch

    def relative_pose(self, position, quaternion) -> Pose3D:
        """A cell-frame pose (position, quaternion x, y, z, w) relative to the top item, as object points store it:
        the inverse of how go_to_point places a point (taskweave_skills.points.instance_matrix)."""
        c, s = math.cos(self.pose.yaw), math.sin(self.pose.yaw)
        dx, dy, dz = position[0] - self.pose.x, position[1] - self.pose.y, position[2] - self.top_z
        unturn = quaternion_from_rpy((0.0, 0.0, -self.pose.yaw))
        return Pose3D((c * dx + s * dy, -s * dx + c * dy, dz),
                      rpy_from_quaternion(quaternion_multiply(unturn, quaternion)), self.object)


@dataclass(frozen=True)
class Pose3D:
    """6-DoF pose (metres, radians, roll-pitch-yaw) relative to a named frame, e.g. a tool pose on an object."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]
    frame: str

    @classmethod
    def from_dict(cls, data: dict, default_frame: str) -> Pose3D:
        return cls(tuple(map(float, data['xyz'])), tuple(map(float, data['rpy'])), data.get('frame', default_frame))

    def to_dict(self) -> dict:
        return {'frame': self.frame, 'xyz': [round(v, 5) for v in self.xyz], 'rpy': [round(v, 5) for v in self.rpy]}


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


# Quaternions are (x, y, z, w) as in geometry_msgs; roll-pitch-yaw is R = Rz(yaw) Ry(pitch) Rx(roll) as in URDF.
def quaternion_from_rpy(rpy) -> tuple[float, float, float, float]:
    (cr, cp, cy), (sr, sp, sy) = ((math.cos(a / 2) for a in rpy), (math.sin(a / 2) for a in rpy))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def rpy_from_quaternion(q) -> tuple[float, float, float]:
    x, y, z, w = q
    sin_pitch = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)), math.asin(sin_pitch),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def quaternion_multiply(a, b) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v) -> tuple[float, float, float]:
    """Vector ``v`` rotated by quaternion ``q``."""
    x, y, z, _ = quaternion_multiply(quaternion_multiply(q, (*v, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return x, y, z
