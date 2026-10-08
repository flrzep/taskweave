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
