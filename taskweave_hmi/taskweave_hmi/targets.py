"""The HMI on a cell pack in the targets layout (cell schema 4). No ROS dependency.

The HMI's API stays as it was (locations, stances, object points, robot poses, held poses); on a schema 4 cell the
modules points, arm, robot and held read and write targets through the store (``save_target``, ``delete_target``,
``rename_target``) instead of the old point files. This module holds what they share: which target an old point is,
the entries as written (so a save keeps the keys the HMI does not edit), the robot poses as listed, the missions
that name a target, and the manual skill requests (``move {to}`` when the skill catalog has ``move``).

On a schema 3 cell none of this is used: the modules keep their old calls.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_core.config import target_store as ts
from taskweave_core.config.targets import TOOL_SUFFIX, rpy_from_axes

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


# ---- robot poses (robots/<robot>/targets.yaml, joint and hand targets) ---------------------------------------------
def robot_poses(store: ConfigStore, robot: str) -> dict[str, dict]:
    """The joint and base-frame hand targets of the robot as the HMI shows a named pose: ``{name: {'joints':
    {joint: value}, 'tools': {tool: {tcp, approach, closing}}, 'hands': {tool: {xyz, rpy}}, 'elbow', 'problems'}}``,
    file order. The core's reader (``joint_poses``) and, for a target it skips, its problem
    (``target_store.pose_target_problems``): one bad target does not fail the list. ``hands`` are the tools as written
    (xyz / rpy), which the old shape does not give; ``problems`` of a readable pose are its joints outside the limits
    (as ``limits.pose_problems``)."""
    from taskweave_core.config.limits import joint_limits, joint_problems

    bad = ts.pose_target_problems(store, robot)
    poses = store.joint_poses(robot)
    limits = joint_limits(store, robot)
    linear = set(store.robot(robot).data.get('joints', {}).get('linear', []))
    result = {}
    for name, entry in ts.robot_targets(store, robot).items():
        if name in bad and entry.get('moves') in ('joints', 'hand'):
            result[name] = {'joints': {}, 'elbow': entry.get('elbow'), 'tools': {}, 'hands': {},
                            'problems': [bad[name]]}
        elif name in poses:
            pose = poses[name]
            joints = pose.get('joints', {}) if 'tools' in pose else pose
            hands = store.target(name, robot).tools
            result[name] = {
                'joints': joints, 'elbow': pose.get('elbow') if 'tools' in pose else None,
                'tools': pose.get('tools', {}),
                'hands': {key[:-len(TOOL_SUFFIX)]: {'xyz': [round(float(v), ROUND) + 0.0 for v in hand.xyz],
                                                    'rpy': [round(float(v), ROUND) + 0.0 for v in hand.rpy]}
                          for key, hand in hands.items()},
                'problems': joint_problems(joints, limits, linear)}
    return result


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
