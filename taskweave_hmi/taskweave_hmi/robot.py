"""Robot settings for the HMI: named joint poses and navigation clearance. No ROS dependency.

Joint poses live in ``robots/<robot>/poses.yaml`` (joint -> rad, or m for the profile's ``joints.linear``). A pose that
a mission uses cannot be renamed or deleted; which skill arguments name a pose comes from the skill catalog (the point
kinds in POSE_KINDS, at any depth: move_joints ``pose``, move_gripper ``to.pose``; a dotted path names a nested one).
Navigation clearance lives in ``robots/<robot>/navigation.yaml`` (see ``schemas.NAVIGATION``); the server applies
saved values to the running path planner. Arm solutions: the profile's ``ik.choose`` (preferred elbow side, how much
joint-limit room and move size count); skills read it before every solve.

On a cell pack in the targets layout (cell schema 4) a pose is a target in ``robots/<robot>/targets.yaml``: a joint
target (``moves: joints``, joint values as the profile's joint groups) or a hand target in the base frame (``moves:
hand``, each tool as xyz / rpy, plus elbow and a joints part); missions name it as ``move {to: <name>}`` (and the old
skills' args until the switch). The API keeps its shape: joints as {joint: value}, tools as {tcp, approach, closing}.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping

from jsonschema import Draft202012Validator

from taskweave_core.config import ConfigError, ConfigStore, schemas
from taskweave_core.config.limits import joint_groups_of, joint_limits, joint_problems, mimic_joints, pose_problems
from taskweave_core.config.targets import GroupError

from . import targets

HMI_NOTE = 'edited in the HMI'
POSE_KINDS = ('joint_pose', 'joint_pose_only', 'tool_pose')    # x-taskweave-point kinds that name a pose of poses.yaml
DEFAULT_POSE_ARGS = {('move_joints', 'pose'), ('move_gripper', 'to.pose')}


class RobotSettingsError(ValueError):
    """A robot setting cannot be changed; the message says why."""


def _pose_paths(schema: Mapping, prefix: str = '') -> set[str]:
    """Dotted paths of the properties of ``schema`` (also in oneOf / anyOf / allOf branches) that name a pose."""
    found = set()
    for branch in (schema, *(b for key in ('oneOf', 'anyOf', 'allOf') for b in schema.get(key, []))):
        for arg, spec in (branch.get('properties') or {}).items():
            path = f'{prefix}{arg}'
            if spec.get('x-taskweave-point') in POSE_KINDS:
                found.add(path)
            found |= _pose_paths(spec, f'{path}.')
    return found


def pose_args(catalog: Iterable[dict] | None) -> set[tuple[str, str]]:
    """(skill, argument path) pairs whose value names a pose, from the skill catalog."""
    found = {(skill['name'], path) for skill in catalog or [] for path in _pose_paths(skill.get('args_schema', {}))}
    return found or DEFAULT_POSE_ARGS


def _value(args, path: str):
    """The value at the dotted ``path`` in a step's args, None if absent."""
    for key in path.split('.'):
        args = args.get(key) if isinstance(args, Mapping) else None
    return args


def pose_usage(store: ConfigStore, args: set[tuple[str, str]]) -> dict[str, list[str]]:
    """Pose name -> missions that use it. On a targets cell: the plain names the missions' steps name (``move
    {to}``, ``via``, the old skills' args), whatever ``args`` says."""
    if targets.active(store):
        return {name: users for name, users in targets.references(store).items()
                if targets.SEPARATOR not in name and not name.startswith(targets.STANCE)}
    result: dict[str, list[str]] = {}
    for mission_name in store.mission_names():
        try:
            steps = store.expanded_mission(mission_name).get('steps', [])
        except ConfigError:
            continue
        used = {_value(step.get('args', {}), arg) for step in steps
                for skill, arg in args if step.get('skill') == skill}
        for name in used - {None}:
            if isinstance(name, str):
                result.setdefault(name, []).append(mission_name)
    return result


def overview(store: ConfigStore, robot: str, args: set[tuple[str, str]]) -> dict:
    store.reload()
    profile = store.robot(robot)
    joints = profile.data.get('joints', {})
    used = pose_usage(store, args)
    mimics = mimic_joints(store, robot)
    if targets.active(store):
        poses = [{'name': name, 'joints': pose['joints'], 'tools': pose['tools'], 'elbow': pose['elbow'],
                  'used_by': used.get(name, []), 'problems': pose['problems']}
                 for name, pose in targets.robot_poses(store, robot).items()]
    else:
        poses = [{'name': name, **store.named_pose(robot, name), 'used_by': used.get(name, []),
                  'problems': pose_problems(store, robot, name)}    # e.g. copied from another robot
                 for name in store.joint_poses(robot)]     # joints set directly, tools solved by IK
    return {
        'robot': robot,
        'files': {name: store.display_path(store.robot_save_path(robot, f'{file}.yaml'))  # where edits go
                  for name, file in (('poses', poses_file(store)), ('navigation', 'navigation'))},
        'linear_joints': list(joints.get('linear', [])),
        'poses': poses,
        # joint -> [lower, upper] (rad or m; null = unbounded) for the editor's input ranges and checks; mimic joints
        # (they follow another joint) left out: they cannot be set
        'joint_limits': {joint: [None if math.isinf(v) else v for v in bounds]
                         for joint, bounds in joint_limits(store, robot).items() if joint not in mimics},
        'grippers': grippers(profile.data.get('grippers', {})),
        'tools': list(profile.data.get('ik', {}).get('groups', {})),     # tools a pose can place (IK available)
        'navigation': store.navigation(robot),
        'footprint': profile.base.get('footprint'),
        'base_type': profile.base.get('type'),          # sideways driving only for 'holonomic'
        'limits': profile.base.get('limits', {}),       # linear (m/s), angular (rad/s): caps for the driving speeds
        'ik_choice': ik_choice(profile.data.get('ik', {})),
    }


def poses_file(store: ConfigStore) -> str:
    """The robot file the poses are in (without .yaml): targets on a targets cell, else poses."""
    return 'targets' if targets.active(store) else 'poses'


def grippers(spec: Mapping) -> dict:
    """tool -> {joint, open, closed, opening_m} of the profile's joint grippers, for the Robot tab's live view:
    opening_m ([[joint value, finger opening in m], ...]) or null when the profile has no table."""
    return {tool: {'joint': g['joint'], 'open': float(g.get('open', 0.0)), 'closed': float(g['closed']),
                   'opening_m': g.get('opening_m')}
            for tool, g in spec.items() if 'joint' in g and 'closed' in g}


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
    return store.display_path(path)


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
    new = targets.active(store)
    exists = targets.plain_kind(store, robot, name) is not None if new else name in store.joint_poses(robot)
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
    if new:
        return _save_pose_target(store, robot, name, body, joints, elbow, rename)
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
    return store.display_path(path)


def delete_pose(store: ConfigStore, robot: str, name: str, args: set[tuple[str, str]]) -> str:
    store.reload()
    users = pose_usage(store, args).get(name, [])
    if users:
        raise RobotSettingsError(f'pose {name} is used by {", ".join(users)}; change those missions first')
    if targets.active(store):
        _pose_kind(store, robot, name, exists=True)
        try:
            return store.display_path(store.delete_target(name, robot)[0])
        except (ValueError, KeyError) as error:
            raise _problem(error) from error
    try:
        path = store.delete_point(f'robots/{robot}/poses.yaml', [name])
    except ValueError as error:
        raise RobotSettingsError(str(error)) from error
    return store.display_path(path)


# ---- the same on a targets cell (cell schema 4) ---------------------------------------------------------------------
def _problem(error: Exception) -> RobotSettingsError:
    """A store error as the HMI's: TargetError is a KeyError, its message the first argument."""
    return RobotSettingsError(str(error.args[0]) if isinstance(error, KeyError) and error.args else str(error))


def _pose_kind(store: ConfigStore, robot: str, name: str, exists: bool = False) -> str | None:
    """What the target ``name`` moves when it can be a pose (joints, or a hand in the base frame), None if there is
    none (``exists``: refused then too); refused for a target that is not a pose."""
    from taskweave_core.config import target_store

    kind = targets.plain_kind(store, robot, name)
    if name in target_store.cell_targets(store) or kind not in (None, 'joints', 'hand'):
        raise RobotSettingsError(f'{name!r} is a target that moves {kind} (not a robot pose): give the pose another '
                                 'name')
    if kind is None and exists:
        raise RobotSettingsError(f'no pose {name!r} in robots/{robot}/targets.yaml')
    return kind


def _save_pose_target(store: ConfigStore, robot: str, name: str, body: Mapping, joints: dict, elbow: str | None,
                      rename: str | None) -> str:
    """save_pose() on a targets cell: a joint target, or with tools a hand target in the base frame, in
    robots/<robot>/targets.yaml. Keys the editor does not show (approach_m, grip_m, a path's earlier waypoints, a
    torso ``auto`` / ``keep``) are kept; tools and elbow are as given, as poses.yaml had them."""
    kind = _pose_kind(store, robot, name)
    entry = targets.plain_entry(store, robot, name)
    try:
        groups = joint_groups_of(store, robot, joints, keep=entry.get('joints'))
    except GroupError as error:
        raise RobotSettingsError(str(error)) from error
    tools = body.get('tools') or None
    if tools:
        allowed = list(store.robot(robot).data.get('ik', {}).get('groups', {}))
        checked = _check_tools(tools, allowed)
        hands = {tool: body['hands'][tool] for tool in checked if tool in (body.get('hands') or {})}
        hands.update({tool: targets.hand_from_axes(target) for tool, target in checked.items() if tool not in hands})
        data = targets.without_tools({k: v for k, v in entry.items() if k not in ('joints', 'elbow')}) \
            if kind == 'hand' else {}
        data = targets.placed({**data, 'moves': 'hand'}, {targets.tool_key(tool): hand for tool, hand in hands.items()})
        if elbow:
            data['elbow'] = elbow
    else:
        data = {'moves': 'joints'}
    if groups:
        data['joints'] = groups
    try:
        return store.display_path(store.save_target(name, data, robot, rename=rename)[0])
    except (ValueError, KeyError) as error:
        raise _problem(error) from error


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
    return store.display_path(path)

