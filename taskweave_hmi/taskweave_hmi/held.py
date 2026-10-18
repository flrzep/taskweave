"""Held poses for the HMI Teach tab: where the robot holds an object of a type (robots/<robot>/held_poses.yaml),
used by move_object ``to: {pose: <name>}``. No ROS dependency.

A held pose moves some joints first (e.g. the torso lift; the object moves with the arms), then the held object's
origin in straight lines through the waypoints (base frame: x y z in m, roll pitch yaw in rad). It cannot be captured
by jogging (a held object needs both hands moving together), so it is edited as numbers and tried with Test.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_core.config.limits import joint_limits

SKILL = 'move_object'


def args(name: str) -> dict:
    """The move_object args that take held pose ``name``."""
    return {'to': {'pose': name}}


def pose_of(step_args) -> str | None:
    """The held pose a move_object step names (None: it names none, or a placeholder)."""
    to = (step_args or {}).get('to')
    pose = to.get('pose') if isinstance(to, Mapping) else None
    return pose if isinstance(pose, str) and '{' not in pose else None


class HeldError(ValueError):
    """A held pose cannot be changed; the message says why."""


def used_names(store: ConfigStore) -> dict[str, list[str]]:
    """Held pose name -> missions using it. A step names only the pose (its object type is whichever the robot holds
    then), so a name counts for every object type that has a pose of that name."""
    names: dict[str, list[str]] = {}
    for mission_name in store.mission_names():
        try:
            steps = store.expanded_mission(mission_name).get('steps', [])
        except ConfigError:
            continue
        for step in steps:
            pose = pose_of(step.get('args')) if step.get('skill') == SKILL else None
            if pose is not None and mission_name not in names.setdefault(pose, []):
                names[pose].append(mission_name)
    return names


def overview(store: ConfigStore, robot: str) -> list[dict]:
    """Every held pose of the robot, with the missions using it."""
    used = used_names(store)
    return [{'object_type': obj, 'name': name, 'joints': dict(entry.get('joints') or {}),
             'path': [{'xyz': list(p['xyz']), 'rpy': list(p['rpy'])} for p in entry.get('path') or []],
             'used_by': used.get(name, [])}
            for obj, entries in store.held_poses(robot).items() for name, entry in entries.items()]


def _numbers(values, label: str) -> list[float]:
    try:
        numbers = [float(v) for v in values]
    except (TypeError, ValueError):
        raise HeldError(f'{label}: three numbers') from None
    if len(numbers) != 3 or not all(math.isfinite(v) for v in numbers):
        raise HeldError(f'{label}: three numbers')
    return numbers


def save(store: ConfigStore, robot: str, obj: str, name: str, body: Mapping) -> str:
    """Save the editor's numbers: ``joints`` {joint: value}, ``path`` [{xyz, rpy}] (at least one), ``rename``."""
    store.reload()
    try:
        store.object(obj)
    except ConfigError as error:
        raise HeldError(f'no object type {obj!r} (objects/{obj}.yaml)') from error
    path = []
    for index, waypoint in enumerate(body.get('path') or [], 1):
        if not isinstance(waypoint, Mapping):
            raise HeldError(f'waypoint {index}: x y z and roll pitch yaw')
        path.append({'xyz': _numbers(waypoint.get('xyz'), f'waypoint {index} x y z'),
                     'rpy': _numbers(waypoint.get('rpy'), f'waypoint {index} roll pitch yaw')})
    if not path:
        raise HeldError('a held pose needs at least one waypoint')
    limits = joint_limits(store, robot)
    joints = {}
    for joint, value in (body.get('joints') or {}).items():
        try:
            joints[joint] = float(value)
        except (TypeError, ValueError):
            raise HeldError(f'{joint}: a number') from None
        if limits and joint not in limits:
            raise HeldError(f'{joint}: this robot has no such joint')
        lower, upper = limits.get(joint, (-math.inf, math.inf))
        if not lower - 1e-6 <= joints[joint] <= upper + 1e-6:
            raise HeldError(f'{joint}: {joints[joint]:g} is outside {lower:g}..{upper:g}')
    rename = body.get('rename') or None
    if rename and rename != name:
        users = used_names(store).get(name, [])
        if users:
            raise HeldError(f'{name} is used by {", ".join(users)}; change those missions first')
    try:
        file = store.save_held_pose(robot, obj, name, joints, path, rename)
    except ValueError as error:
        raise HeldError(str(error)) from error
    return str(file.relative_to(store.root))


def delete(store: ConfigStore, robot: str, obj: str, name: str) -> str:
    store.reload()
    users = used_names(store).get(name, [])
    if users:
        raise HeldError(f'{name} is used by {", ".join(users)}; change those missions first')
    try:
        return str(store.delete_point(f'robots/{robot}/held_poses.yaml', [obj, name]).relative_to(store.root))
    except ValueError as error:
        raise HeldError(str(error)) from error
