"""The HMI on a cell pack in the targets layout (cell schema 4). No ROS dependency.

The HMI's API stays as it was (locations, stances, object points, robot poses, held poses); on a schema 4 cell the
modules points, arm, robot and held read and write targets through the store (``save_target``, ``delete_target``,
``rename_target``) instead of the old point files. This module holds what they share: which target an old point is,
the entries as written (so a save keeps the keys the HMI does not edit), joint values as joint groups, the missions
that name a target, and the manual skill requests (``move {to}`` when the skill catalog has ``move``).

On a schema 3 cell none of this is used: the modules keep their old calls.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping

from taskweave_core.config import ConfigError, ConfigStore, Pose2D
from taskweave_core.config import target_store as ts
from taskweave_core.config.limits import joint_groups, torso_joint
from taskweave_core.config.targets import TOOL_SUFFIX, axes_from_rpy, rpy_from_axes

MOVE = 'move'
KIND_GROUPS = {'base': 'base targets', 'hand': 'hand targets', 'object': 'object targets', 'joints': 'joint targets'}
OLD_PREFIXES = ('location:', 'pose:')          # a reference of the old skills: the name follows
STANCE = 'stance:'                             # stance:<instance>[/<variant>]
SEPARATOR = '/'
ROUND = 4


def active(store: ConfigStore) -> bool:
    """The cell pack is in the targets layout: write targets, never the old point files. A stand-in store without
    the method (tests of the old layout) is the old layout."""
    check = getattr(store, 'targets_format', None)
    return bool(check()) if check is not None else False


# ---- names and entries --------------------------------------------------------------------------------------------
def plain_kind(store: ConfigStore, robot: str, name: str) -> str | None:
    """What a plain name (a cell, base-frame or joint target) moves, None when there is no such target."""
    for table in (ts.cell_targets(store), ts.robot_targets(store, robot)):
        if name in table:
            return table[name].get('moves')
    return None


def plain_entry(store: ConfigStore, robot: str, name: str) -> dict:
    """The entry of a plain target as written ({} when there is none), without ``frame`` (the file says it)."""
    for table in (ts.cell_targets(store), ts.robot_targets(store, robot)):
        if name in table:
            return {k: v for k, v in table[name].items() if k != 'frame'}
    return {}


def placed_instance(store: ConfigStore, object_type: str) -> str | None:
    """A placed object of ``object_type`` (the first by name): save_target addresses a type's targets through one."""
    return next((name for name, instance in sorted(store.object_instances().items())
                 if instance.object == object_type), None)


def type_kinds(store: ConfigStore, robot: str | None, object_type: str, instance: str | None = None) -> dict:
    """``{name: moves}`` of the object-frame targets of a type (with the robot's own, and an instance's)."""
    return ts.type_target_kinds(store, robot, object_type, instance)


def robot_layer(store: ConfigStore, robot: str, object_type: str, keys: Iterable[str]) -> dict:
    """The entry at ``keys`` (``targets, <name>`` or ``instances, <instance>, <name>``) of the robot's objects file
    for the type, as written ({} when absent)."""
    node = ts.robot_object_file(store, robot, object_type)
    for key in keys:
        node = node.get(key) if isinstance(node, Mapping) else None
    return dict(node) if isinstance(node, Mapping) else {}


def type_layer(store: ConfigStore, object_type: str, name: str) -> dict:
    """The type's own entry of target ``name`` (objects/<type>.yaml targets), as written ({} when absent)."""
    return dict(ts.object_file(store, object_type).get('targets', {}).get(name) or {})


def placed(entry: Mapping, pose: Mapping) -> dict:
    """``entry`` with its pose keys set from ``pose``; a target given as a path gets them on its last waypoint."""
    data = dict(entry)
    if 'path' in data and data['path']:
        path = [dict(step) for step in data['path']]
        path[-1] = {**path[-1], **pose}
        data['path'] = path
    else:
        data.update(pose)
    return data


def tool_key(tool: str) -> str:
    return f'{tool}{TOOL_SUFFIX}'


def without_tools(entry: Mapping) -> dict:
    """``entry`` without its tool poses (on the entry or, for a path, on the last waypoint)."""
    data = {k: v for k, v in entry.items() if not k.endswith(TOOL_SUFFIX)}
    if data.get('path'):
        path = [dict(step) for step in data['path']]
        path[-1] = {k: v for k, v in path[-1].items() if not k.endswith(TOOL_SUFFIX)}
        data['path'] = path
    return data


def hand_from_axes(target: Mapping) -> dict:
    """{tcp, approach, closing} (base frame) -> {xyz, rpy}, as a hand target writes it."""
    return {'xyz': [round(float(v), ROUND) + 0.0 for v in target['tcp']],
            'rpy': [round(v, ROUND) + 0.0 for v in rpy_from_axes(target['approach'], target['closing'])]}


def axes_from_hand(pose) -> dict:
    """A Pose3D (xyz, rpy) -> {tcp, approach, closing}, the shape the HMI shows a robot pose's tools in."""
    approach, closing = axes_from_rpy(pose.rpy)
    return {'tcp': [round(float(v), ROUND) + 0.0 for v in pose.xyz],
            'approach': [round(v, ROUND) + 0.0 for v in approach], 'closing': [round(v, ROUND) + 0.0 for v in closing]}


# ---- joints: joint values <-> joint groups ------------------------------------------------------------------------
def group_joints(store: ConfigStore, robot: str, values: Mapping[str, float], keep: Mapping | None = None) -> dict:
    """``{joint: value}`` as the ``joints:`` part of a target: per group of the profile a number (a group of one
    joint, or the torso's lift joint alone) or a list in the group's order with None for a joint not given. ``keep``:
    the target's joints part before, whose ``auto`` / ``keep`` words stay for groups the values do not set. Raises
    ValueError for a joint that is in no group."""
    groups, lift = joint_groups(store, robot), torso_joint(store, robot)
    left, result = dict(values), {}
    for group, members in groups.items():
        hit = [joint for joint in members if joint in left]
        if not hit:
            continue
        if len(members) == 1:
            result[group] = round(float(left.pop(members[0])), 5)
        elif group == 'torso' and hit == [lift]:
            result[group] = round(float(left.pop(lift)), 5)
        else:
            result[group] = [round(float(left.pop(joint)), 5) if joint in left else None for joint in members]
    if left:
        raise ValueError(f'not in a joint group of the robot profile (joints.groups): {", ".join(sorted(left))}')
    for group, value in (keep or {}).items():
        if isinstance(value, str) and group not in result:
            result[group] = value
    return result


# ---- robot poses (robots/<robot>/targets.yaml, joint and hand targets) ---------------------------------------------
def robot_poses(store: ConfigStore, robot: str) -> dict[str, dict]:
    """The joint and base-frame hand targets of the robot as the HMI shows a named pose: ``{name: {'joints':
    {joint: value}, 'tools': {tool: {tcp, approach, closing}}, 'hands': {tool: {xyz, rpy}}, 'elbow', 'problems'}}``.
    A target that cannot be read is listed with its problem instead of failing the whole list."""
    from taskweave_core.config.limits import group_values, joint_limits, joint_problems

    limits = joint_limits(store, robot)
    linear = set(store.robot(robot).data.get('joints', {}).get('linear', []))
    result = {}
    for name, entry in ts.robot_targets(store, robot).items():
        if entry.get('moves') not in ('joints', 'hand'):
            continue
        problems, tools = [], {}
        try:
            tools = store.target(name, robot).tools
        except KeyError as error:
            problems.append(str(error))
        try:
            joints = group_values(store, robot, entry.get('joints') or {})
        except ValueError as error:
            joints = {}
            problems.append(str(error))
        problems += joint_problems(joints, limits, linear)
        result[name] = {
            'joints': joints, 'elbow': entry.get('elbow'),
            'tools': {key[:-len(TOOL_SUFFIX)]: axes_from_hand(pose) for key, pose in tools.items()},
            'hands': {key[:-len(TOOL_SUFFIX)]: {'xyz': [round(float(v), ROUND) + 0.0 for v in pose.xyz],
                                                'rpy': [round(float(v), ROUND) + 0.0 for v in pose.rpy]}
                      for key, pose in tools.items()},
            'problems': problems}
    return result


# ---- stances (robots/<robot>/objects/<type>.yaml, base targets) ---------------------------------------------------
def _xyaw(pose: Pose2D) -> dict:
    return {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw}


def stance_offsets(store: ConfigStore, robot: str) -> dict:
    """The robot's base targets on object types in the shape of the old stances.yaml: ``{'objects': {type: {name:
    {x, y, yaw}}}, 'instances': {instance: {name: {x, y, yaw}}}}``, file order kept. An instance override is the
    merged pose (an override may give only some keys); one of an object that is not placed is left out."""
    objects, instances = {}, {}
    placed_objects = store.object_instances()
    for object_type in ts.type_names(store, robot):
        variants = {}
        for name, moves in type_kinds(store, robot, object_type).items():
            if moves == 'base':
                variants[name] = _xyaw(ts.resolve_type(store, robot, object_type, name).pose)
        if variants:
            objects[object_type] = variants
        for instance, table in ts.robot_object_file(store, robot, object_type).get('instances', {}).items():
            if instance not in placed_objects or placed_objects[instance].object != object_type:
                continue
            kinds = type_kinds(store, robot, object_type, instance)
            merged = store.stance_variants(robot, placed_objects[instance])
            poses = {name: _xyaw(merged[name]) for name in table if kinds.get(name) == 'base' and name in merged}
            if poses:
                instances.setdefault(instance, {}).update(poses)
    return {**({'objects': objects} if objects else {}), **({'instances': instances} if instances else {})}


# ---- references in missions ---------------------------------------------------------------------------------------
def _strings(value) -> Iterator[str]:
    if isinstance(value, str):
        yield value
    elif isinstance(value, Mapping):
        for item in value.values():
            yield from _strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from _strings(item)


def normal(text: str) -> str:
    """A reference without the old skills' prefixes: ``location:<n>`` and ``pose:<n>`` are ``<n>``,
    ``stance:<instance>/<variant>`` is ``<instance>/<variant>``; ``stance:<instance>`` (the default variant) stays."""
    for prefix in OLD_PREFIXES:
        if text.startswith(prefix):
            return text[len(prefix):]
    if text.startswith(STANCE) and SEPARATOR in text:
        return text[len(STANCE):]
    return text


def references(store: ConfigStore) -> dict[str, list[str]]:
    """Reference -> the missions that name it, from every string in the steps' args (``move {to}`` and its ``via``,
    and the old skills' args until the switch), normalised (``normal``). Placeholders are left out: they are filled
    at run time. Steps of included missions count too."""
    result: dict[str, list[str]] = {}
    for mission_name in store.mission_names():
        try:
            mission = store.expanded_mission(mission_name)
        except ConfigError:
            continue
        found = set()
        for step in mission.get('steps', []):
            found |= {normal(text) for text in _strings(step.get('args', {})) if '{' not in text}
        for reference in sorted(found):
            result.setdefault(reference, []).append(mission_name)
    return result


def default_stance(store: ConfigStore, robot: str, instance: str) -> str | None:
    """The variant ``stance:<instance>`` meant (the first of the robot's base targets for it), None if none."""
    placed_objects = store.object_instances()
    if instance not in placed_objects:
        return None
    return next(iter(store.stance_variants(robot, placed_objects[instance])), None)


def target_reference(store: ConfigStore, robot: str, text: str) -> str:
    """An old-style reference as ``move`` takes it: no prefixes, ``stance:<instance>`` with its default variant."""
    reference = normal(text)
    if reference.startswith(STANCE):
        instance = reference[len(STANCE):]
        variant = default_stance(store, robot, instance)
        return f'{instance}{SEPARATOR}{variant}' if variant else instance
    return reference


# ---- pickers ------------------------------------------------------------------------------------------------------
def picker_names(store: ConfigStore, robot: str, old: dict[str, list[str]]) -> dict[str, list[str]]:
    """The old pickers' lists (``old``, kept for the old skills' ``x-taskweave-point`` kinds) plus the target groups
    (``base targets``, ``hand targets``, ``object targets``, ``joint targets``, ``placed objects``) that
    ``x-taskweave-target-kinds`` fields pick from."""
    return {**old, **store.target_names(robot)}


# ---- manual skills ------------------------------------------------------------------------------------------------
def move_configured(store: ConfigStore) -> bool:
    """The cell pack configures the move skill (skills/move.yaml): the HMI's fallback when the skill catalog
    cannot be asked."""
    return (store.root / 'skills' / f'{MOVE}.yaml').is_file()


def serves_move(catalog_names: Iterable[str] | None, store: ConfigStore) -> bool:
    """Run ``move`` for the manual skills: the live skill catalog has it (``catalog_names``), or, when it cannot be
    asked (None), the cell pack configures it."""
    if catalog_names is None:
        return move_configured(store)
    return MOVE in set(catalog_names)


def move_args(reference: str, **options) -> dict:
    """``move {to: <reference>}`` with the options given (``hands``, ``offset_m``; None ones left out)."""
    return {'to': reference, **{key: value for key, value in options.items() if value is not None}}


def module_config(store: ConfigStore, old_skill: str, part: str) -> dict:
    """A motion module's skills file: ``skills/move/<part>.yaml`` when the cell has it (the move skill's layout),
    else the old skill's ``skills/<old_skill>.yaml``."""
    if (store.root / 'skills' / MOVE / f'{part}.yaml').is_file():
        return store.skill_config(f'{MOVE}/{part}')
    return store.skill_config(old_skill)


def approach_default(store: ConfigStore) -> float:
    """The hand motion's straight final approach when a point does not set its own (m)."""
    params = {k: v for k, v in module_config(store, 'move_gripper', 'hand').items() if k != 'implementations'}
    return float((params.get('motion') or {}).get('approach_m', 0.0))
