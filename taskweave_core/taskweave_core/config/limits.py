"""Joint limits of a robot, and the checks that keep named poses inside them.

Limits come from the robot's URDF (``robots/<robot>/moveit/robot.urdf``, usually in its robot pack: revolute and
prismatic joints with a ``<limit>``) and can be added or overridden in the profile:

    joints:
      limits: {body_joint: [0.0, 0.75]}     # joint -> [lower, upper] in rad or m

A robot with no limits from either source is not checked. Poses are robot-specific, so a pose that names a joint the
robot does not have, or a value outside its range, is a problem: saving it is refused, missions using it fail
validation, and the HMI flags it (e.g. poses copied over from another robot).
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ElementTree
from pathlib import Path

import yaml

from .store import ConfigStore

URDF_FILE = 'moveit/robot.urdf'           # a robot file (ConfigStore.robot_file): cell pack, then robot packs
URDF = 'robots/{robot}/' + URDF_FILE       # the cell pack's copy only; use urdf_path()
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


def urdf_path(store: ConfigStore, robot: str) -> Path:
    """The robot's URDF (may not exist)."""
    return store.robot_file(robot, URDF_FILE)


def joint_limits(store: ConfigStore, robot: str) -> dict[str, tuple[float, float]]:
    """Joint -> (lower, upper) for the robot: its URDF, then the profile's joints.limits on top. {} if none known."""
    path = urdf_path(store, robot)
    limits = _urdf_limits(path) if path.is_file() else {}
    for joint, (lower, upper) in store.robot(robot).data.get('joints', {}).get('limits', {}).items():
        limits[joint] = (float(lower), float(upper))
    return limits


def _urdf_velocities(path: Path) -> dict[str, float]:
    return {joint.get('name'): float(limit.get('velocity'))
            for joint in ElementTree.parse(path).getroot().iter('joint')
            if (limit := joint.find('limit')) is not None and limit.get('velocity') is not None
            and float(limit.get('velocity')) > 0}


def velocity_limits(store: ConfigStore, robot: str) -> dict[str, float]:
    """Joint -> max speed (rad/s, m/s for a prismatic joint) for the robot. Source order: the robot pack's
    ``moveit/joint_limits.yaml`` (MoveIt format, ``joint_limits: {joint: {has_velocity_limits, max_velocity}}``; a
    joint with ``has_velocity_limits: false`` or no ``max_velocity`` is skipped), else the URDF ``<limit velocity>``.
    {} if neither says anything."""
    limits: dict[str, float] = {}
    urdf = urdf_path(store, robot)
    if urdf.is_file():
        limits.update(_urdf_velocities(urdf))
    path = store.robot_file(robot, 'moveit/joint_limits.yaml')
    if path.is_file():
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        for joint, entry in (data.get('joint_limits') or {}).items():
            if isinstance(entry, dict) and entry.get('has_velocity_limits', True) and entry.get('max_velocity'):
                limits[joint] = float(entry['max_velocity'])
    return limits


def base_limits(store: ConfigStore, robot: str) -> dict[str, float]:
    """The base's speed limits from the profile's ``base.limits``: ``{linear: m/s, angular: rad/s}`` (keys it lacks are
    left out)."""
    limits = (store.robot(robot).data.get('base') or {}).get('limits') or {}
    return {key: float(value) for key, value in limits.items() if key in ('linear', 'angular')}


def capped_by(wanted: dict[str, float], limits: dict[str, float]) -> dict[str, float]:
    """The entries of ``wanted`` that exceed their limit, as ``{key: limit}`` (the ``motion.capped`` of a step result)."""
    return {key: limits[key] for key, value in wanted.items() if key in limits and value > limits[key] + 1e-12}


def joint_groups(store: ConfigStore, robot: str) -> dict[str, list[str]]:
    """Group -> joints for move_joints ``only``: the profile's ``joints.groups``, plus a ``<tool>_arm`` group for each
    IK tool it does not list (the URDF joints that the ik.joints regex, {tool} = tool name, matches). {} if unknown."""
    data = store.robot(robot).data
    groups = {name: list(joints) for name, joints in data.get('joints', {}).get('groups', {}).items()}
    ik = data.get('ik') or {}
    if ik.get('joints'):
        known = list(joint_limits(store, robot))
        for tool in ik.get('groups', {}):
            if f'{tool}_arm' not in groups:
                pattern = re.compile(ik['joints'].format(tool=tool))
                if members := [joint for joint in known if pattern.fullmatch(joint)]:
                    groups[f'{tool}_arm'] = members
    return groups


def arm_group_names(store: ConfigStore, robot: str) -> set[str]:
    """The groups of ``joint_groups`` that are an arm: ``<tool>_arm`` for each tool of the robot (grippers, tool
    frames, ik groups, arms). Moving their joints moves a hand."""
    data = store.robot(robot).data
    tools = set(data.get('grippers', {})) | set(data.get('tool_frames', {})) | set(data.get('arms', {}))
    return {f'{tool}_arm' for tool in tools | set((data.get('ik') or {}).get('groups', {}))}


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
