"""Robot settings for the HMI: named joint poses and navigation clearance. No ROS dependency.

Joint poses live in ``robots/<robot>/poses.yaml`` (joint -> rad, or m for the profile's ``joints.linear``). A pose that
a mission uses cannot be renamed or deleted; which skill arguments name a joint pose comes from the skill catalog
(``x-taskweave-point: joint_pose``). Navigation clearance lives in ``robots/<robot>/navigation.yaml`` (see
``schemas.NAVIGATION``); the server applies saved values to the running path planner.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping

from jsonschema import Draft202012Validator

from taskweave_core.config import ConfigError, ConfigStore, schemas

HMI_NOTE = 'edited in the HMI'
DEFAULT_POSE_ARGS = {('move_to_pose', 'pose')}


class RobotSettingsError(ValueError):
    """A robot setting cannot be changed; the message says why."""


def pose_args(catalog: Iterable[dict] | None) -> set[tuple[str, str]]:
    """(skill, argument) pairs whose value names a joint pose, from the skill catalog."""
    found = {(skill['name'], arg) for skill in catalog or []
             for arg, spec in skill.get('args_schema', {}).get('properties', {}).items()
             if spec.get('x-taskweave-point') == 'joint_pose'}
    return found or DEFAULT_POSE_ARGS


def pose_usage(store: ConfigStore, args: set[tuple[str, str]]) -> dict[str, list[str]]:
    """Joint pose name -> missions that use it."""
    result: dict[str, list[str]] = {}
    for mission_name in store.mission_names():
        try:
            steps = store.mission(mission_name).get('steps', [])
        except ConfigError:
            continue
        used = {step.get('args', {}).get(arg) for step in steps for skill, arg in args if step.get('skill') == skill}
        for name in used - {None}:
            result.setdefault(name, []).append(mission_name)
    return result


def overview(store: ConfigStore, robot: str, args: set[tuple[str, str]]) -> dict:
    store.reload()
    profile = store.robot(robot)
    joints = profile.data.get('joints', {})
    used = pose_usage(store, args)
    return {
        'robot': robot,
        'files': {'poses': f'robots/{robot}/poses.yaml', 'navigation': f'robots/{robot}/navigation.yaml'},
        'linear_joints': list(joints.get('linear', [])),
        'poses': [{'name': name, 'joints': values, 'used_by': used.get(name, [])}
                  for name, values in store.joint_poses(robot).items()],
        'navigation': store.navigation(robot),
        'footprint': profile.base.get('footprint'),
    }


def _check_values(values) -> dict[str, float]:
    if not isinstance(values, Mapping) or not values:
        raise RobotSettingsError('a pose needs at least one joint value')
    try:
        result = {str(joint): float(value) for joint, value in values.items()}
    except (TypeError, ValueError) as error:
        raise RobotSettingsError('joint values must be numbers') from error
    if not all(math.isfinite(v) for v in result.values()):
        raise RobotSettingsError('joint values must be finite')
    return result


def save_pose(store: ConfigStore, robot: str, name: str, body: Mapping, args: set[tuple[str, str]]) -> str:
    """Create or update a joint pose. ``body``: joints {name: value}, optional rename and create (no overwrite)."""
    store.reload()
    exists = name in store.joint_poses(robot)
    if body.get('create') and exists:
        raise RobotSettingsError(f'pose {name!r} already exists')
    rename = body.get('rename') or None
    if rename == name:
        rename = None
    if rename:
        users = pose_usage(store, args).get(name, [])
        if users:
            raise RobotSettingsError(f'pose {name} is used by {", ".join(users)}; change those missions first')
    try:
        path = store.set_joint_pose(robot, name, _check_values(body.get('joints')), rename=rename, note=HMI_NOTE)
    except ValueError as error:
        raise RobotSettingsError(str(error)) from error
    return str(path.relative_to(store.root))


def delete_pose(store: ConfigStore, robot: str, name: str, args: set[tuple[str, str]]) -> str:
    store.reload()
    users = pose_usage(store, args).get(name, [])
    if users:
        raise RobotSettingsError(f'pose {name} is used by {", ".join(users)}; change those missions first')
    try:
        path = store.delete_point(f'robots/{robot}/poses.yaml', [name])
    except ValueError as error:
        raise RobotSettingsError(str(error)) from error
    return str(path.relative_to(store.root))


def check_navigation(settings: Mapping) -> dict:
    errors = sorted(Draft202012Validator(schemas.NAVIGATION).iter_errors(settings), key=lambda e: list(e.absolute_path))
    if errors:
        raise RobotSettingsError('; '.join(f'{".".join(map(str, e.absolute_path)) or "settings"}: {e.message}'
                                           for e in errors))
    return dict(settings)


def save_navigation(store: ConfigStore, robot: str, settings: Mapping) -> str:
    try:
        path = store.save_navigation(robot, check_navigation(settings))
    except ValueError as error:
        raise RobotSettingsError(str(error)) from error
    return str(path.relative_to(store.root))


def footprint(settings: Mapping) -> list[list[float]]:
    """Envelope (front/back/left/right from the base frame) as a polygon in the base frame (x forward, y left)."""
    e = settings['envelope']
    return [[e['front'], e['left']], [e['front'], -e['right']], [-e['back'], -e['right']], [-e['back'], e['left']]]
