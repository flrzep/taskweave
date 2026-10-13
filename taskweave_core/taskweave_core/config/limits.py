"""Joint limits of a robot, and the checks that keep named poses inside them.

Limits come from the robot's URDF (``robots/<robot>/moveit/robot.urdf``: revolute and prismatic joints with a
``<limit>``) and can be added or overridden in the profile:

    joints:
      limits: {body_joint: [0.0, 0.75]}     # joint -> [lower, upper] in rad or m

A robot with no limits from either source is not checked. Poses are robot-specific, so a pose that names a joint the
robot does not have, or a value outside its range, is a problem: saving it is refused, missions using it fail
validation, and the HMI flags it (e.g. poses copied over from another robot).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ElementTree
from pathlib import Path

from .store import ConfigStore

URDF = 'robots/{robot}/moveit/robot.urdf'
TOLERANCE = 1e-6  # values written with 4 decimals may sit a hair past a limit given with more


def _urdf_limits(path: Path) -> dict[str, tuple[float, float]]:
    limits = {}
    for joint in ElementTree.parse(path).getroot().iter('joint'):
        limit = joint.find('limit')
        if joint.get('type') in ('revolute', 'prismatic') and limit is not None:
            limits[joint.get('name')] = (float(limit.get('lower', '-inf')), float(limit.get('upper', 'inf')))
        elif joint.get('type') == 'continuous':
            limits[joint.get('name')] = (-math.inf, math.inf)
    return limits


def joint_limits(store: ConfigStore, robot: str) -> dict[str, tuple[float, float]]:
    """Joint -> (lower, upper) for the robot: its URDF, then the profile's joints.limits on top. {} if none known."""
    path = store.root / URDF.format(robot=robot)
    limits = _urdf_limits(path) if path.is_file() else {}
    for joint, (lower, upper) in store.robot(robot).data.get('joints', {}).get('limits', {}).items():
        limits[joint] = (float(lower), float(upper))
    return limits


def joint_problems(values: dict[str, float], limits: dict[str, tuple[float, float]],
                   linear: set[str] = frozenset()) -> list[str]:
    """Problems with joint values: joints the robot does not have, values outside their range."""
    if not limits:
        return []
    problems = []
    for joint, value in values.items():
        if joint not in limits:
            problems.append(f'{joint}: this robot has no such joint')
            continue
        lower, upper = limits[joint]
        if not lower - TOLERANCE <= float(value) <= upper + TOLERANCE:
            if joint in linear:
                problems.append(f'{joint}: {float(value):.3f} m is outside {lower:.3f}..{upper:.3f} m')
            else:
                problems.append(f'{joint}: {math.degrees(float(value)):.1f} deg is outside '
                                f'{math.degrees(lower):.1f}..{math.degrees(upper):.1f} deg')
    return problems


def pose_problems(store: ConfigStore, robot: str, name: str) -> list[str]:
    """Problems with a named pose's joints (set directly) for this robot; tool targets are checked by IK."""
    pose = store.named_pose(robot, name)
    linear = set(store.robot(robot).data.get('joints', {}).get('linear', []))
    return joint_problems(pose['joints'], joint_limits(store, robot), linear)
