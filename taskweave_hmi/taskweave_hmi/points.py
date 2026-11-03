"""Points for the HMI: list, edit, rename and delete locations and stances. No ROS dependency.

The API reads and writes targets (targets.py): a location is a base target in ``cell/targets.yaml`` (cell frame), a
stance variant a base target of the type in ``robots/<robot>/objects/<type>.yaml`` (an offset in the object's frame;
an override for one placed instance: its ``instances`` entry), an object point a hand target in
``objects/<type>.yaml`` with the robot's settings (elbow, grip_m) in the robot's objects file. Placed objects
(``cell/objects.yaml``) are read-only here: the scene is built from the same layout.

Every point lists the missions that refer to it: they name them as ``<name>`` and ``<instance>/<name>`` (``move
{to}``); the old skills' forms count too. A point that a mission needs cannot be renamed or deleted; a stance
override can be deleted when the object type has the same variant to fall back on. Placeholders such as
``{fullest_stack}`` are resolved at run time and are not counted.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

from taskweave_core.config import ConfigStore, Pose2D, TargetError, schemas, target_store
from taskweave_core.config.store import STANCE_PREFIX, TOOL_SUFFIX, VARIANT_SEPARATOR

from . import targets

CELL_TARGETS = 'cell/targets.yaml'
SCOPES = ('objects', 'instances')


class PointError(ValueError):
    """A point cannot be changed; the message says why."""


def robot_objects_file(robot: str, object_type: str = '<type>') -> str:
    return f'robots/{robot}/objects/{object_type}.yaml'


def stance_offsets(store: ConfigStore, robot: str) -> dict:
    """The robot's stances in the shape of the old stances.yaml (the store's reader of its base targets on object
    types: an instance override is merged with the type's layers, so one may give only some keys)."""
    return store.stance_offsets(robot)


def pose_dict(pose: Pose2D) -> dict:
    return {'x': round(pose.x, 4), 'y': round(pose.y, 4), 'yaw': round(pose.yaw, 4)}


def picker_names(store: ConfigStore, robot: str) -> dict[str, list[str]]:
    """Named points for argument pickers (GET /api/points): the store's lists, plus held poses (any object type:
    the held type is known at run time), via targets (``pose:<joint pose>`` or a placed object point) and the target
    groups (base / hand / object / joint targets, placed objects) that fields with ``x-taskweave-target-kinds`` pick
    from."""
    names = store.point_names(robot)
    names['held poses'] = sorted({name for poses in store.held_poses(robot).values() for name in poses})
    names['via targets'] = [f'pose:{pose}' for pose in names.get('joint poses', [])] + \
        list(names.get('placed object points', []))
    return targets.picker_names(store, robot, names)


def usage(store: ConfigStore, robot: str) -> dict[tuple, list[str]]:
    """Which missions use which point: ('location', name), (scope, owner, variant) or ('object_point', type,
    point) -> mission names, from the references the missions name (``<name>``, ``<instance>/<name>``, and the old
    skills' forms)."""
    result: dict[tuple, list[str]] = {}
    cell = target_store.cell_targets(store)
    instances = store.object_instances()
    overrides = stance_offsets(store, robot).get('instances', {})
    for reference, missions in targets.references(store).items():
        if reference.startswith(STANCE_PREFIX):  # stance:<instance>, the old default variant
            owner = reference[len(STANCE_PREFIX):]
            variant = targets.default_stance(store, robot, owner)
        else:
            owner, _, variant = reference.partition(VARIANT_SEPARATOR)
        key = None
        if not variant:
            if cell.get(reference, {}).get('moves') == 'base':
                key = ('location', reference)
        elif owner in instances:
            object_type = instances[owner].object
            moves = targets.type_kinds(store, robot, object_type, owner).get(variant)
            if moves == 'base':
                key = (('instances', owner, variant) if variant in overrides.get(owner, {})
                       else ('objects', object_type, variant))
            elif moves == 'hand':
                key = ('object_point', object_type, variant)
        if key is not None:
            users = result.setdefault(key, [])
            users += [mission for mission in missions if mission not in users]
    return result


def overview(store: ConfigStore, robot: str) -> dict:
    """Everything the points view shows, with the missions using each point."""
    store.reload()
    used = usage(store, robot)
    stances = stance_offsets(store, robot)
    instances = store.object_instances()
    cell_stances = store.stances(robot)
    types = list(dict.fromkeys([i.object for i in instances.values()] + list(stances.get('objects', {}))))
    objects = []
    for object_type in types:
        variants = stances.get('objects', {}).get(object_type, {})
        placed = []
        for instance in (i for i in instances.values() if i.object == object_type):
            overrides = stances.get('instances', {}).get(instance.name, {})
            placed.append({
                'name': instance.name, 'pose': pose_dict(instance.pose), 'z': instance.z,
                'overrides': [{'variant': v, 'offset': pose_dict(Pose2D.from_dict(p)),
                               'used_by': used.get(('instances', instance.name, v), []),
                               'fallback': v in variants} for v, p in overrides.items()],
                'stances': {v: pose_dict(p) for v, p in cell_stances.items()
                            if v.partition(VARIANT_SEPARATOR)[0] == instance.name and VARIANT_SEPARATOR in v},
            })
        objects.append({
            'object': object_type,
            'points': object_points(store, object_type, used, robot),
            'variants': [{'variant': v, 'offset': pose_dict(Pose2D.from_dict(p)),
                          'used_by': used.get(('objects', object_type, v), [])} for v, p in variants.items()],
            'instances': placed,
        })
    return {
        'robot': robot,
        'files': {'locations': CELL_TARGETS, 'stances': robot_objects_file(robot), 'objects': 'cell/objects.yaml'},
        'locations': [{'name': n, 'pose': pose_dict(p), 'used_by': used.get(('location', n), [])}
                      for n, p in store.locations().items()],
        'objects': objects,
        # move_gripper's straight final approach when a point does not set its own
        'approach_default': targets.approach_default(store),
        'elbow_sides': schemas.ELBOW_SIDES,
    }


def object_points(store: ConfigStore, object_type: str, used: dict, robot: str | None = None) -> list[dict]:
    """Points on an object type: its hand targets (tool poses in the object's frame), approach and (with ``robot``)
    that robot's elbow side and gripper openings when closed there (grip_m) for it."""
    result = []
    for name, moves in targets.type_kinds(store, robot, object_type).items():
        if moves != 'hand':
            continue
        target = target_store.resolve_type(store, robot, object_type, name)
        result.append({
            'name': name, 'approach_m': target.get('approach_m'), 'elbow': target.get('elbow'),
            'grip_m': dict(target.get('grip_m') or {}),
            'tools': {key[:-len(TOOL_SUFFIX)]: {'xyz': list(pose.xyz), 'rpy': list(pose.rpy)}
                      for key, pose in target.tools.items()},
            'used_by': used.get(('object_point', object_type, name), []),
        })
    return result


def save_object_point(store: ConfigStore, object_type: str, point: str, body: Mapping, robot: str | None = None) -> str:
    """Approach (m or null = skill default), elbow (side or null = robot default), grip_m ({tool: finger opening in m
    when closed here, or null = close fully}, within the robot's opening_m table; elbow and grip_m are stored for
    ``robot``) and tool poses of a point."""
    from taskweave_core.gripper import opening_range

    settings = {key: body[key] for key in ('approach_m', 'elbow', 'grip_m') if key in body}
    if 'grip_m' in settings:
        if not robot:
            raise PointError('grip: needs the robot')
        if settings['grip_m'] is None:
            settings['grip_m'] = {}
        if not isinstance(settings['grip_m'], Mapping):
            raise PointError('grip: an opening per hand')
        grippers = store.robot(robot).data.get('grippers', {})
        grip = {}
        for tool, value in settings['grip_m'].items():
            if value in (None, ''):
                grip[tool] = None
                continue
            try:
                grip[tool] = float(value)
            except (TypeError, ValueError):
                raise PointError(f'grip {tool}: an opening in mm, or empty to close fully') from None
            span = opening_range(grippers.get(tool, {}))
            if span is None:
                raise PointError(f'grip {tool}: the {robot} profile has no opening_m table for this gripper')
            if not (math.isfinite(grip[tool]) and span[0] - 1e-6 <= grip[tool] <= span[1] + 1e-6):
                raise PointError(f'grip {tool}: {grip[tool] * 1000:g} mm is outside {span[0] * 1000:g}'
                                 f'..{span[1] * 1000:g} mm')
        settings['grip_m'] = grip
    if settings.get('elbow') not in (None, '', *schemas.ELBOW_SIDES):
        raise PointError(f'elbow: one of {", ".join(schemas.ELBOW_SIDES)} or the robot default')
    if settings.get('approach_m') not in (None, ''):
        try:
            if not 0.0 <= float(settings['approach_m']) <= 1.0:
                raise ValueError
        except (TypeError, ValueError):
            raise PointError('approach_m: a distance from 0 to 1 m') from None
    poses = {}
    for tool, pose in (body.get('tools') or {}).items():
        try:
            poses[tool] = {k: [float(v) for v in pose[k]] for k in ('xyz', 'rpy')}
        except (KeyError, TypeError, ValueError):
            raise PointError(f'{tool}: xyz and rpy need three numbers each') from None
        if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in poses[tool].values()):
            raise PointError(f'{tool}: xyz and rpy need three numbers each')
    return _save_hand_target(store, object_type, point, settings, poses, robot)


def parse_pose(body: Mapping, frame: str) -> Pose2D:
    try:
        x, y, yaw = (float(body[key]) for key in ('x', 'y', 'yaw'))
    except (KeyError, TypeError, ValueError) as error:
        raise PointError('x, y and yaw (radians) are required numbers') from error
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise PointError('x, y and yaw must be finite')
    return Pose2D(x, y, yaw, frame)


def _frame(store: ConfigStore, kind: str, path: list[str]) -> str:
    """The frame of a point address: location/<name> or stance/<scope>/<owner>/<variant>."""
    if kind == 'location' and len(path) == 1:
        return 'cell'
    if kind == 'stance' and len(path) == 3 and path[0] in SCOPES:
        scope, owner, _ = path
        if scope == 'instances':
            instance = store.object_instances().get(owner)
            if instance is None:
                raise PointError(f'no object {owner!r} in cell/objects.yaml')
            return instance.object
        return owner
    raise PointError(f'unknown point address {"/".join([kind, *path])!r}')


def _users(store: ConfigStore, robot: str, kind: str, path: list[str]) -> list[str]:
    key = ('location', path[0]) if kind == 'location' else tuple(path)
    return usage(store, robot).get(key, [])


def _exists(store: ConfigStore, robot: str, kind: str, path: list[str]) -> bool:
    if kind == 'location':
        return path[0] in store.locations()
    scope, owner, variant = path
    return variant in stance_offsets(store, robot).get(scope, {}).get(owner, {})


def _type_has_variant(store: ConfigStore, robot: str, path: list[str]) -> bool:
    """An instance override can go when its object type has the same variant to fall back on."""
    instance = store.object_instances()[path[1]]
    stances = stance_offsets(store, robot)
    return path[2] in stances.get('objects', {}).get(instance.object, {})


def _written(store: ConfigStore, paths) -> str:
    return paths[0].relative_to(store.root).as_posix() if paths else ''


def _target_address(store: ConfigStore, robot: str, kind: str, path: list[str]) -> tuple[str, bool, str | None, str]:
    """(reference, instance override, ``type=`` of the store's target calls, frame) of a point address: a location is
    its name, a stance of the type the bare variant with ``type=<type>`` (nothing needs to be placed), an override
    ``<instance>/<variant>``. A name with a ``/`` (an encoded one in the URL) is refused: the teach node would read
    ``<instance>/<name>``."""
    if any('/' in part for part in path):
        raise PointError(f'{"/".join(path)!r}: a name has no "/"')
    frame = _frame(store, kind, path)
    if kind == 'location':
        moves = targets.plain_kind(store, robot, path[0])
        if moves is not None and (moves != 'base' or path[0] not in target_store.cell_targets(store)):
            raise PointError(f'{path[0]!r} is a target that moves {moves} in robots/{robot}/targets.yaml, not a '
                             'location: give the location another name')
        return path[0], False, None, frame
    scope, owner, variant = path
    override = scope == 'instances'
    moves = targets.type_kinds(store, robot, frame, owner if override else None).get(variant)
    if moves not in (None, 'base'):
        raise PointError(f'{variant!r} is a target of {frame} objects that moves {moves}, not a stance: give '
                         'the stance another name')
    if override:
        return f'{owner}{VARIANT_SEPARATOR}{variant}', True, None, frame
    return variant, False, owner, frame


def save(store: ConfigStore, robot: str, kind: str, path: list[str], body: Mapping) -> str:
    """Create or update a point. ``body``: x, y, yaw (radians); optional ``rename`` (new last path element) and
    ``create`` (refuse to overwrite an existing point). A location is a base target in cell/targets.yaml, a stance
    variant a base target of the type in the robot's objects file (an override: its instances entry). Keys the editor
    does not show (joints, a path's earlier waypoints) are kept."""
    store.reload()
    reference, override, on_type, frame = _target_address(store, robot, kind, path)
    if body.get('create') and _exists(store, robot, kind, path):
        raise PointError(f'{"/".join(path)} already exists')
    rename = body.get('rename') or None
    if rename == path[-1]:
        rename = None
    if rename:
        users = _users(store, robot, kind, path)
        if users:
            raise PointError(f'{"/".join(path)} is used by {", ".join(users)}; change those missions first')
        if override and path[2] in targets.type_kinds(store, robot, frame):
            raise PointError(f'{"/".join(path)} overrides the {frame} stance {path[2]} and has its name: '
                             'rename the stance of the type')
    pose = parse_pose(body, frame)
    if kind == 'location':
        entry = targets.plain_entry(store, robot, path[0])
    else:
        keys = ['instances', path[1], path[2]] if override else ['targets', path[2]]
        entry = targets.robot_layer(store, robot, frame, keys)
    data = targets.placed({'moves': 'base', **entry}, pose_dict(pose))
    try:
        paths = store.save_target(reference, data, robot, instance_override=override, rename=rename, type=on_type)
    except targets.REFUSED as error:
        raise PointError(str(error)) from error
    return _written(store, paths)


def teach_reference(store: ConfigStore, robot: str, kind: str, path: list[str], at: str | None,
                    create: bool = False) -> tuple[str, bool, str | None]:
    """What the teach node is asked for to save a point from the robot (kind base): ``(reference, instance_override,
    the placed object measured at)``. A location is its name; a stance of the type is measured at the placed object
    ``at`` (``<at>/<variant>``), an override at its own instance. Refused like save: a name that moves something else,
    ``create`` of one that exists, a stance without a placed object of its type."""
    store.reload()
    reference, override, on_type, frame = _target_address(store, robot, kind, path)
    if create and _exists(store, robot, kind, path):
        raise PointError(f'{"/".join(path)} already exists')
    if kind == 'location':
        return reference, False, None
    if override:
        return reference, True, path[1]
    instance = store.object_instances().get(at or '')
    if instance is None or instance.object != on_type:
        raise PointError(f'pick the placed {on_type} to measure the stance at' if not at
                         else f'{at} is not a placed {on_type}')
    return f'{at}{VARIANT_SEPARATOR}{path[2]}', False, at


def stored_pose(store: ConfigStore, robot: str, kind: str, path: list[str]) -> dict:
    """The pose a point has now ({x, y, yaw}, as save takes it); PointError when there is none."""
    if kind == 'location':
        pose = store.locations().get(path[0])
    else:
        scope, owner, variant = path
        entry = stance_offsets(store, robot).get(scope, {}).get(owner, {}).get(variant)
        pose = None if entry is None else Pose2D.from_dict(entry)
    if pose is None:
        raise PointError(f'{"/".join(path)} is not there')
    return pose_dict(pose)


def delete(store: ConfigStore, robot: str, kind: str, path: list[str]) -> str:
    """Delete a point. A stance of the type that instance overrides with only settings still refer to is refused by
    the store, its message listing them (delete those first: the HMI offers no cascade)."""
    store.reload()
    reference, override, on_type, _ = _target_address(store, robot, kind, path)
    users = _users(store, robot, kind, path)
    if users and not (kind == 'stance' and override and _type_has_variant(store, robot, path)):
        raise PointError(f'{"/".join(path)} is used by {", ".join(users)}; change those missions first')
    try:
        paths = store.delete_target(reference, robot, instance_override=override, type=on_type)
    except targets.REFUSED as error:
        raise PointError(str(error)) from error
    return _written(store, paths)


def _save_hand_target(store: ConfigStore, object_type: str, point: str, settings: dict, poses: dict,
                      robot: str | None) -> str:
    """save_object_point()'s write: the hand target ``point`` of the type, addressed with ``type=`` (nothing
    needs to be placed). The target as the type's and the robot's layers give it (``type_target``) with the editor's
    changes; the store writes each key back to the layer it is in (a new elbow or grip_m to the robot's file), so keys
    the editor does not show and a target defined in the robot's file stay where they are."""
    if not settings and not poses:
        raise PointError('nothing to save')
    if not robot:
        raise PointError('a target of an object type is saved for a robot: needs the robot')
    if targets.type_kinds(store, robot, object_type).get(point) != 'hand':
        raise PointError(f'object {object_type!r} has no point {point!r}')
    try:
        data = {k: v for k, v in store.type_target(object_type, point, robot).data.items() if k != 'frame'}
    except TargetError as error:  # it is there (type_kinds) but cannot be read
        raise PointError(str(error)) from error
    for key in ('approach_m', 'elbow'):
        if key in settings:
            if settings[key] in (None, ''):
                data.pop(key, None)
            else:
                data[key] = round(float(settings[key]), 4) if key == 'approach_m' else settings[key]
    if 'grip_m' in settings:
        grip = dict(data.get('grip_m') or {})
        for tool, value in settings['grip_m'].items():
            if value is None:
                grip.pop(tool, None)
            else:
                grip[tool] = round(value, 4)
        data.pop('grip_m', None)
        if grip:
            data['grip_m'] = grip
    data = targets.placed(data, {targets.tool_key(tool): pose for tool, pose in poses.items()})
    try:
        return _written(store, store.save_target(point, data, robot, type=object_type))
    except targets.REFUSED as error:
        raise PointError(str(error)) from error


def relative_poses(store: ConfigStore, pose: Pose2D) -> dict:
    """The robot pose in the cell and relative to every placed object (for 'set from robot'), nearest first."""
    instances = sorted(store.object_instances().values(),
                       key=lambda i: math.hypot(i.pose.x - pose.x, i.pose.y - pose.y))
    return {'pose': pose_dict(pose),
            'relative': [{'instance': i.name, 'object': i.object,
                          'distance': round(math.hypot(i.pose.x - pose.x, i.pose.y - pose.y), 3),
                          'offset': pose_dict(pose.relative_to(i.pose, i.object))} for i in instances]}
