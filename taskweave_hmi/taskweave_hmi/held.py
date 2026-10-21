"""Held poses for the HMI Teach tab: where the robot holds an object of a type (robots/<robot>/held_poses.yaml),
used by move_object ``to: {pose: <name>}``. No ROS dependency.

A held pose moves some joints first (e.g. the torso lift; the object moves with the arms), then the held object's
origin in straight lines through the waypoints (base frame: x y z in m, roll pitch yaw in rad). It cannot be captured
by jogging (a held object needs both hands moving together), so it is edited as numbers and tried with Test.

On a cell pack in the targets layout (cell schema 4) a held pose is an object target in the base frame in
``robots/<robot>/targets.yaml`` (``moves: object``, ``type: <object type>``, ``path``, joints as joint groups), named
by ``move {to: <name>}``; a target without a type is listed under the object type ``*``. Test runs ``move`` when the
skill catalog has it.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_core.config.limits import joint_groups_of, joint_limits
from taskweave_core.config.targets import GroupError

from . import targets

SKILL = 'move_object'
ANY_TYPE = '*'  # a held pose (object target) for whatever the robot holds


def args(name: str) -> dict:
    """The move_object args that take held pose ``name``."""
    return {'to': {'pose': name}}


def test_request(name: str, move: bool) -> tuple[str, dict]:
    """The skill and args that move the held object to held pose ``name`` now: ``move {to}`` when the skill catalog
    has move, else move_object."""
    return (targets.MOVE, targets.move_args(name)) if move else (SKILL, args(name))


def pose_of(step_args) -> str | None:
    """The held pose a move_object step names (None: it names none, or a placeholder)."""
    to = (step_args or {}).get('to')
    pose = to.get('pose') if isinstance(to, Mapping) else None
    return pose if isinstance(pose, str) and '{' not in pose else None


class HeldError(ValueError):
    """A held pose cannot be changed; the message says why."""


def used_names(store: ConfigStore) -> dict[str, list[str]]:
    """Held pose name -> missions using it. A step names only the pose (its object type is whichever the robot holds
    then), so a name counts for every object type that has a pose of that name. On a targets cell: the plain names
    the steps name (``move {to}``, and move_object's ``to.pose`` until the switch)."""
    if targets.active(store):
        return {name: users for name, users in targets.references(store).items()
                if targets.SEPARATOR not in name and not name.startswith(targets.STANCE)}
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
        if not (obj == ANY_TYPE and targets.active(store)):
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
    if targets.active(store):
        return _save_object_target(store, robot, obj, name, joints, path, rename)
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
    if targets.active(store):
        _held_entry(store, robot, obj, name, exists=True)
        try:
            return store.delete_target(name, robot)[0].relative_to(store.root).as_posix()
        except targets.REFUSED as error:
            raise HeldError(str(error)) from error
    try:
        return str(store.delete_point(f'robots/{robot}/held_poses.yaml', [obj, name]).relative_to(store.root))
    except ValueError as error:
        raise HeldError(str(error)) from error


# ---- the same on a targets cell (cell schema 4) ---------------------------------------------------------------------
def _held_entry(store: ConfigStore, robot: str, obj: str, name: str, exists: bool = False) -> dict:
    """The object target ``name`` of the robot as written ({} if there is none; refused with ``exists``), refused when
    the name is another kind of target or a held pose for another object type."""
    from taskweave_core.config import target_store

    kind = targets.plain_kind(store, robot, name)
    if name in target_store.cell_targets(store) or kind not in (None, 'object'):
        raise HeldError(f'{name!r} is a target that moves {kind}, not a held pose: give the held pose another name')
    if kind is None:
        if exists:
            raise HeldError(f'no held pose {name!r} in robots/{robot}/targets.yaml')
        return {}
    entry = targets.plain_entry(store, robot, name)
    if entry.get('type', ANY_TYPE) != obj:
        raise HeldError(f'{name} is the held pose for {entry.get("type", "any")} objects (names are unique per robot)')
    return entry


def _save_object_target(store: ConfigStore, robot: str, obj: str, name: str, joints: dict, path: list[dict],
                        rename: str | None) -> str:
    """save() on a targets cell: an object target in the base frame in robots/<robot>/targets.yaml. Keys the editor
    does not show (reference, a torso ``auto`` / ``keep``) are kept; a single pose stays one while it is one."""
    entry = _held_entry(store, robot, obj, name)
    try:
        groups = joint_groups_of(store, robot, joints, keep=entry.get('joints'))
    except GroupError as error:
        raise HeldError(str(error)) from error
    data = {k: v for k, v in entry.items() if k not in ('xyz', 'rpy', 'path', 'joints', 'type')}
    data['moves'] = 'object'
    if obj != ANY_TYPE:
        data['type'] = obj
    if len(path) == 1 and 'xyz' in entry:
        data.update(path[0])
    else:
        data['path'] = path
    if groups:
        data['joints'] = groups
    try:
        return store.save_target(name, data, robot, rename=rename)[0].relative_to(store.root).as_posix()
    except targets.REFUSED as error:
        raise HeldError(str(error)) from error
