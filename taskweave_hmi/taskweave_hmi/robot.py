"""Robot settings for the HMI: named joint poses and navigation clearance. No ROS dependency.

Joint poses live in ``robots/<robot>/poses.yaml`` (joint -> rad, or m for the profile's ``joints.linear``). A pose that
a mission uses cannot be renamed or deleted; which skill arguments name a joint pose comes from the skill catalog
(``x-taskweave-point: joint_pose``). Navigation clearance lives in ``robots/<robot>/navigation.yaml`` (see
``schemas.NAVIGATION``); the server applies saved values to the running path planner. Arm solutions: the profile's
``ik.choose`` (preferred elbow side, how much joint-limit room and move size count); skills read it before every solve.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping

from jsonschema import Draft202012Validator

from taskweave_core.config import ConfigError, ConfigStore, schemas
from taskweave_core.config.limits import joint_limits, joint_problems, pose_problems

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
        'poses': [{'name': name, **store.named_pose(robot, name), 'used_by': used.get(name, []),
                   'problems': pose_problems(store, robot, name)}    # e.g. copied from another robot
                  for name in store.joint_poses(robot)],     # joints set directly, tools solved by IK
        # joint -> [lower, upper] (rad or m; null = unbounded) for the editor's input ranges and checks
        'joint_limits': {joint: [None if math.isinf(v) else v for v in bounds]
                         for joint, bounds in joint_limits(store, robot).items()},
        'tools': list(profile.data.get('ik', {}).get('groups', {})),     # tools a pose can place (IK available)
        'navigation': store.navigation(robot),
        'footprint': profile.base.get('footprint'),
        'base_type': profile.base.get('type'),          # sideways driving only for 'holonomic'
        'limits': profile.base.get('limits', {}),       # linear (m/s), angular (rad/s): caps for the driving speeds
        'ik_choice': ik_choice(profile.data.get('ik', {})),
    }


def ik_choice(ik: Mapping) -> dict | None:
    """The profile's ik.choose with defaults filled in, for the Arm solutions card; None without ik."""
    from taskweave_skills.ik import LIMIT_MARGIN_DEG, WEIGHTS

    if not ik:
        return None
    choose = ik.get('choose') or {}
    elbow = choose.get('elbow')
    return {'enabled': 'choose' in ik, 'elbow': elbow if isinstance(elbow, str) else None,
            'elbow_vector': elbow if isinstance(elbow, list) else None,
            'limit_margin_deg': choose.get('limit_margin_deg', LIMIT_MARGIN_DEG),
            'weights': {**WEIGHTS, **choose.get('weights', {})}, 'elbow_sides': schemas.ELBOW_SIDES}


def save_ik_choice(store: ConfigStore, robot: str, body: Mapping) -> str:
    """Elbow side (or null: no preference), limit margin and weights from the Arm solutions card."""
    from taskweave_skills.ik import WEIGHTS

    choice: dict = {}
    if 'elbow' in body:
        if body['elbow'] not in (None, *schemas.ELBOW_SIDES):
            raise RobotSettingsError(f'elbow: one of {", ".join(schemas.ELBOW_SIDES)} or none')
        choice['elbow'] = body['elbow']
    if 'limit_margin_deg' in body:
        choice['limit_margin_deg'] = _number(body['limit_margin_deg'], 'limit_margin_deg', 0.0, 90.0)
    if 'weights' in body:
        unknown = set(body['weights']) - set(WEIGHTS)
        if unknown:
            raise RobotSettingsError(f'weights: unknown {", ".join(sorted(unknown))}')
        choice['weights'] = {k: _number(v, f'weights.{k}', 0.0, 100.0) for k, v in body['weights'].items()}
    try:
        path = store.save_ik_choice(robot, choice)
    except ValueError as error:
        raise RobotSettingsError(str(error)) from error
    return str(path.relative_to(store.root))


def _number(value, name: str, low: float, high: float) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise RobotSettingsError(f'{name}: not a number') from None
    if not low <= number <= high:
        raise RobotSettingsError(f'{name}: {number:g} is outside {low:g}..{high:g}')
    return number


def _check_tools(tools, allowed: list[str]) -> dict:
    if not isinstance(tools, Mapping):
        raise RobotSettingsError('tools must map a tool name to {tcp, approach, closing}')
    result = {}
    for tool, target in tools.items():
        if tool not in allowed:
            known = ', '.join(allowed) or 'none'
            raise RobotSettingsError(f'tool {tool!r} has no IK in the robot profile (has {known})')
        try:
            vectors = {key: [float(v) for v in target[key]] for key in ('tcp', 'approach', 'closing')}
        except (KeyError, TypeError, ValueError) as error:
            raise RobotSettingsError(f'{tool}: tcp, approach and closing need three numbers each') from error
        if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in vectors.values()):
            raise RobotSettingsError(f'{tool}: tcp, approach and closing need three finite numbers each')
        for key in ('approach', 'closing'):
            if math.hypot(*vectors[key]) < 1e-6:
                raise RobotSettingsError(f'{tool}: {key} must not be zero')
        a, c = vectors['approach'], vectors['closing']
        if abs(sum(x * y for x, y in zip(a, c, strict=True)) / (math.hypot(*a) * math.hypot(*c))) > 0.999:
            raise RobotSettingsError(f'{tool}: approach and closing must not be parallel')
        result[tool] = vectors
    return result


def _check_values(values, allow_empty: bool = False) -> dict[str, float]:
    if not isinstance(values, Mapping) or (not values and not allow_empty):
        raise RobotSettingsError('a pose needs at least one joint value')
    try:
        result = {str(joint): float(value) for joint, value in values.items()}
    except (TypeError, ValueError) as error:
        raise RobotSettingsError('joint values must be numbers') from error
    if not all(math.isfinite(v) for v in result.values()):
        raise RobotSettingsError('joint values must be finite')
    return result


def save_pose(store: ConfigStore, robot: str, name: str, body: Mapping, args: set[tuple[str, str]]) -> str:
    """Create or update a pose. ``body``: joints {name: value}, optional tools {tool: {tcp, approach, closing}}
    (base frame, solved by IK when used), rename and create (no overwrite)."""
    store.reload()
    exists = name in store.joint_poses(robot)
    if body.get('create') and exists:
        raise RobotSettingsError(f'pose {name!r} already exists')
    rename = body.get('rename') or None
    if rename == name:
        rename = None
    joints = _check_values(body.get('joints') or {}, allow_empty=bool(body.get('tools')))
    elbow = body.get('elbow') or None
    if elbow not in (None, *schemas.ELBOW_SIDES):
        raise RobotSettingsError(f'elbow: one of {", ".join(schemas.ELBOW_SIDES)} or the robot default')
    linear = set(store.robot(robot).data.get('joints', {}).get('linear', []))
    problems = joint_problems(joints, joint_limits(store, robot), linear)
    if problems:
        raise RobotSettingsError('outside the joint limits of this robot: ' + '; '.join(problems))
    if rename:
        users = pose_usage(store, args).get(name, [])
        if users:
            raise RobotSettingsError(f'pose {name} is used by {", ".join(users)}; change those missions first')
    try:
        tools = body.get('tools') or None
        if tools:
            allowed = list(store.robot(robot).data.get('ik', {}).get('groups', {}))
            path = store.set_named_pose(robot, name, joints,
                                        _check_tools(tools, allowed), rename=rename, note=HMI_NOTE, elbow=elbow)
        else:
            path = store.set_named_pose(robot, name, joints, rename=rename, note=HMI_NOTE)
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


def check_navigation(settings: Mapping, base: Mapping) -> dict:
    """Schema, then what the robot's base allows: sideways only when holonomic, speeds within base.limits."""
    errors = sorted(Draft202012Validator(schemas.NAVIGATION).iter_errors(settings), key=lambda e: list(e.absolute_path))
    problems = [f'{".".join(map(str, e.absolute_path)) or "settings"}: {e.message}' for e in errors]
    driving = settings.get('driving', {}) if not errors else {}
    if driving.get('sideways') and base.get('type') != 'holonomic':
        problems.append(f'driving.sideways: the robot base is {base.get("type")!r} and cannot drive sideways')
    limits = base.get('limits', {})
    for key, limit in (('max_forward', 'linear'), ('max_reverse', 'linear'), ('max_sideways', 'linear'),
                       ('max_turn', 'angular')):
        if key in driving and limit in limits and driving[key] > limits[limit]:
            problems.append(f'driving.{key}: {driving[key]} is above the base limit {limits[limit]}')
    if problems:
        raise RobotSettingsError('; '.join(problems))
    return dict(settings)


def save_navigation(store: ConfigStore, robot: str, settings: Mapping) -> str:
    try:
        path = store.save_navigation(robot, check_navigation(settings, store.robot(robot).base))
    except ValueError as error:
        raise RobotSettingsError(str(error)) from error
    return str(path.relative_to(store.root))

