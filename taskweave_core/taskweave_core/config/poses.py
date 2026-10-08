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
    """A placed object in the cell: its type (``objects/<object>.yaml``), planar pose and height."""

    name: str
    object: str
    pose: Pose2D
    z: float = 0.0


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
