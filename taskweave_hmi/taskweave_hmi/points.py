"""Points for the HMI: list, edit, rename and delete locations and stances. No ROS dependency.

Locations live in ``cell/locations.yaml`` (cell frame). Stances live in ``robots/<robot>/stances.yaml`` as offsets
in the object's frame, per object type (``objects``) or per placed instance (``instances``, overrides). Placed
objects (``cell/objects.yaml``) are read-only here: the scene is built from the same layout.

Every point lists the missions that refer to it. A point that a mission needs cannot be renamed or deleted; a
stance override can be deleted when the object type has the same variant to fall back on. Placeholders such as
``{fullest_stack}`` are resolved at run time and are not counted.

On a cell pack in the targets layout (cell schema 4) the same API reads and writes targets (targets.py): a location
is a base target in ``cell/targets.yaml``, a stance variant a base target of the type in
``robots/<robot>/objects/<type>.yaml`` (an override: its ``instances`` entry), an object point a hand target in
``objects/<type>.yaml`` with the robot's settings (elbow, grip_m) in the robot's objects file. Missions name them as
``<name>`` and ``<instance>/<name>`` (``move {to}``); the old skills' forms count too.
"""

from __future__ import annotations

import math
from collections.abc import Iterator, Mapping

from taskweave_core.config import ConfigError, ConfigStore, Pose2D, schemas, target_store
from taskweave_core.config.store import LOCATION_PREFIX, STANCE_PREFIX, TOOL_SUFFIX, VARIANT_SEPARATOR
from taskweave_core.config.targets import ROBOT_SETTINGS

from . import targets

LOCATIONS = 'cell/locations.yaml'
CELL_TARGETS = 'cell/targets.yaml'
SCOPES = ('objects', 'instances')


class PointError(ValueError):
    """A point cannot be changed; the message says why."""


def stances_file(robot: str) -> str:
    return f'robots/{robot}/stances.yaml'


def robot_objects_file(robot: str, object_type: str = '<type>') -> str:
    return f'robots/{robot}/objects/{object_type}.yaml'


def stance_offsets(store: ConfigStore, robot: str) -> dict:
    """The robot's stances in the shape of stances.yaml (on a targets cell: its base targets on object types)."""
    return targets.stance_offsets(store, robot) if targets.active(store) else store.stance_offsets(robot)


def pose_dict(pose: Pose2D) -> dict:
    return {'x': round(pose.x, 4), 'y': round(pose.y, 4), 'yaw': round(pose.yaw, 4)}


def _strings(value) -> Iterator[str]:
    if isinstance(value, str):
        yield value
    elif isinstance(value, Mapping):
        for item in value.values():
            yield from _strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from _strings(item)


def _stance_source(store: ConfigStore, robot: str, reference: str) -> tuple[str, str, str] | None:
    """The stances.yaml entry a ``stance:`` reference resolves to: (scope, owner, variant), or None."""
    name, _, variant = reference.partition(VARIANT_SEPARATOR)
    instance = store.object_instances().get(name)
    if instance is None:
        return None
    variants = store.stance_variants(robot, instance)
    variant = variant or next(iter(variants), '')
    if variant not in variants:
        return None
    overrides = stance_offsets(store, robot).get('instances', {})
    if variant in overrides.get(name, {}):
        return 'instances', name, variant
    return 'objects', instance.object, variant


def usage(store: ConfigStore, robot: str) -> dict[tuple, list[str]]:
    """Which missions use which point: ('location', name), (scope, owner, variant) or ('object_point', type,
    point) -> mission names."""
    if targets.active(store):
        return _target_usage(store, robot)
    result: dict[tuple, list[str]] = {}
    locations = store.locations()
    instances = store.object_instances()
    for mission_name in store.mission_names():
        try:
            mission = store.expanded_mission(mission_name)  # steps of included missions count too
        except ConfigError:
            continue
        used = set()
        for step in mission.get('steps', []):
            for text in _strings(step.get('args', {})):
                if '{' in text:
                    continue
                if text.startswith(STANCE_PREFIX):
                    source = _stance_source(store, robot, text[len(STANCE_PREFIX):])
                    if source:
                        used.add(source)
                else:
                    name = text[len(LOCATION_PREFIX):] if text.startswith(LOCATION_PREFIX) else text
                    if name in locations:
                        used.add(('location', name))
                    owner, _, point = text.partition(VARIANT_SEPARATOR)
                    if point and owner in instances:
                        used.add(('object_point', instances[owner].object, point))
        for key in used:
            result.setdefault(key, []).append(mission_name)
    return result


def picker_names(store: ConfigStore, robot: str) -> dict[str, list[str]]:
    """Named points for argument pickers (GET /api/points): the store's lists, plus held poses (any object type:
    the held type is known at run time) and via targets (``pose:<joint pose>`` or a placed object point). On a
    targets cell also the target groups (base / hand / object / joint targets, placed objects) that fields with
    ``x-taskweave-target-kinds`` pick from."""
    names = store.point_names(robot)
    names['held poses'] = sorted({name for poses in store.held_poses(robot).values() for name in poses})
    names['via targets'] = [f'pose:{pose}' for pose in names.get('joint poses', [])] + \
        list(names.get('placed object points', []))
    return targets.picker_names(store, robot, names) if targets.active(store) else names


def _target_usage(store: ConfigStore, robot: str) -> dict[tuple, list[str]]:
    """usage() on a targets cell: the references the missions name (``<name>``, ``<instance>/<name>``, and the old
    skills' forms) as the same keys."""
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
    new = targets.active(store)
    return {
        'robot': robot,
        'files': {'locations': CELL_TARGETS if new else LOCATIONS,
                  'stances': robot_objects_file(robot) if new else stances_file(robot), 'objects': 'cell/objects.yaml'},
        'locations': [{'name': n, 'pose': pose_dict(p), 'used_by': used.get(('location', n), [])}
                      for n, p in store.locations().items()],
        'objects': objects,
        # move_gripper's straight final approach when a point does not set its own
        'approach_default': targets.approach_default(store),
        'elbow_sides': schemas.ELBOW_SIDES,
    }


def object_points(store: ConfigStore, object_type: str, used: dict, robot: str | None = None) -> list[dict]:
    """Points on an object type (objects/<type>.yaml): tool poses in the object's frame, approach and (with
    ``robot``) that robot's elbow side and gripper openings when closed there (grip_m) for it."""
    if targets.active(store):
        return _hand_targets(store, object_type, used, robot)
    try:
        points = store.object(object_type).get('points', {})
    except ConfigError:
        return []
    result = []
    for name in points:
        point = store.object_point(object_type, name, robot)
        result.append({
            'name': name, 'approach_m': point.get('approach_m'), 'elbow': point.get('elbow'),
            'grip_m': dict(point.get('grip_m') or {}),
            'tools': {key[:-len(TOOL_SUFFIX)]: {'xyz': list(pose.xyz), 'rpy': list(pose.rpy)}
                      for key, pose in point.items() if key.endswith(TOOL_SUFFIX) and pose is not None},
            'used_by': used.get(('object_point', object_type, name), []),
        })
    return result


def _hand_targets(store: ConfigStore, object_type: str, used: dict, robot: str | None) -> list[dict]:
    """object_points() on a targets cell: the type's hand targets, with the robot's settings on them."""
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
    if targets.active(store):
        return _save_hand_target(store, object_type, point, settings, poses, robot)
    try:
        path = store.save_object_point_settings(object_type, point, settings, robot) if settings else None
        if poses:
            path = store.save_object_point_poses(object_type, point, poses)
    except ValueError as error:
        raise PointError(str(error)) from error
    if path is None:
        raise PointError('nothing to save')
    return str(path.relative_to(store.root))


def parse_pose(body: Mapping, frame: str) -> Pose2D:
    try:
        x, y, yaw = (float(body[key]) for key in ('x', 'y', 'yaw'))
    except (KeyError, TypeError, ValueError) as error:
        raise PointError('x, y and yaw (radians) are required numbers') from error
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise PointError('x, y and yaw must be finite')
    return Pose2D(x, y, yaw, frame)


def _keys(store: ConfigStore, kind: str, path: list[str]) -> tuple[str, list[str], str]:
    """(file, keys, frame) for a point address: location/<name> or stance/<scope>/<owner>/<variant>."""
    if kind == 'location' and len(path) == 1:
        return LOCATIONS, path, 'cell'
    if kind == 'stance' and len(path) == 3 and path[0] in SCOPES:
        scope, owner, _ = path
        if scope == 'instances':
            instance = store.object_instances().get(owner)
            if instance is None:
                raise PointError(f'no object {owner!r} in cell/objects.yaml')
            frame = instance.object
        else:
            frame = owner
        return '', path, frame
    raise PointError(f'unknown point address {"/".join([kind, *path])!r}')


def _users(store: ConfigStore, robot: str, kind: str, path: list[str]) -> list[str]:
    key = ('location', path[0]) if kind == 'location' else tuple(path)
    return usage(store, robot).get(key, [])


def save(store: ConfigStore, robot: str, kind: str, path: list[str], body: Mapping) -> str:
    """Create or update a point. ``body``: x, y, yaw (radians); optional ``rename`` (new last path element) and
    ``create`` (refuse to overwrite an existing point)."""
    if targets.active(store):
        return _save_base_target(store, robot, kind, path, body)
    file, keys, frame = _keys(store, kind, path)
    file = file or stances_file(robot)
    store.reload()
    if body.get('create') and _exists(store, robot, kind, path):
        raise PointError(f'{"/".join(path)} already exists')
    rename = body.get('rename') or None
    if rename == keys[-1]:
        rename = None
    if rename:
        users = _users(store, robot, kind, path)
        if users:
            raise PointError(f'{"/".join(path)} is used by {", ".join(users)}; change those missions first')
    try:
        store.update_point(file, keys, parse_pose(body, frame), rename=rename)
    except ValueError as error:
        raise PointError(str(error)) from error
    return file


def delete(store: ConfigStore, robot: str, kind: str, path: list[str]) -> str:
    if targets.active(store):
        return _delete_base_target(store, robot, kind, path)
    file, keys, _ = _keys(store, kind, path)
    file = file or stances_file(robot)
    store.reload()
    users = _users(store, robot, kind, path)
    if users and not (kind == 'stance' and path[0] == 'instances' and _type_has_variant(store, robot, path)):
        raise PointError(f'{"/".join(path)} is used by {", ".join(users)}; change those missions first')
    try:
        store.delete_point(file, keys)
    except ValueError as error:
        raise PointError(str(error)) from error
    return file


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


# ---- the same on a targets cell (cell schema 4) ---------------------------------------------------------------------
def _problem(error: Exception) -> PointError:
    """A store error as the HMI's: TargetError is a KeyError, its message the first argument."""
    return PointError(str(error.args[0]) if isinstance(error, KeyError) and error.args else str(error))


def _written(store: ConfigStore, paths) -> str:
    return paths[0].relative_to(store.root).as_posix() if paths else ''


def type_reference(store: ConfigStore, object_type: str, name: str) -> str:
    """``<instance>/<name>`` for a target of the type, through a placed object of it (save_target needs one)."""
    instance = targets.placed_instance(store, object_type)
    if instance is None:
        raise PointError(f'no placed {object_type} in cell/objects.yaml: a target of the type is saved through a '
                         'placed object of it')
    return f'{instance}{VARIANT_SEPARATOR}{name}'


def _target_address(store: ConfigStore, robot: str, kind: str, path: list[str]) -> tuple[str, bool, str, str]:
    """(reference, instance override, object type or '', frame) of a point address on a targets cell."""
    _, _, frame = _keys(store, kind, path)
    if kind == 'location':
        moves = targets.plain_kind(store, robot, path[0])
        if moves is not None and (moves != 'base' or path[0] not in target_store.cell_targets(store)):
            raise PointError(f'{path[0]!r} is a target that moves {moves} in robots/{robot}/targets.yaml, not a '
                             'location: give the location another name')
        return path[0], False, '', frame
    scope, owner, variant = path
    override = scope == 'instances'
    object_type = store.object_instances()[owner].object if override else owner
    reference = f'{owner}{VARIANT_SEPARATOR}{variant}' if override else type_reference(store, owner, variant)
    moves = targets.type_kinds(store, robot, object_type, owner if override else None).get(variant)
    if moves not in (None, 'base'):
        raise PointError(f'{variant!r} is a target of {object_type} objects that moves {moves}, not a stance: give '
                         'the stance another name')
    return reference, override, object_type, frame


def _save_base_target(store: ConfigStore, robot: str, kind: str, path: list[str], body: Mapping) -> str:
    """save() on a targets cell: a location is a base target in cell/targets.yaml, a stance variant a base target of
    the type in the robot's objects file (an override: its instances entry). Keys the editor does not show (joints, a
    path's earlier waypoints) are kept."""
    store.reload()
    reference, override, object_type, frame = _target_address(store, robot, kind, path)
    if body.get('create') and _exists(store, robot, kind, path):
        raise PointError(f'{"/".join(path)} already exists')
    rename = body.get('rename') or None
    if rename == path[-1]:
        rename = None
    if rename:
        users = _users(store, robot, kind, path)
        if users:
            raise PointError(f'{"/".join(path)} is used by {", ".join(users)}; change those missions first')
        if override and path[2] in targets.type_kinds(store, robot, object_type):
            raise PointError(f'{"/".join(path)} overrides the {object_type} stance {path[2]} and has its name: '
                             'rename the stance of the type')
    pose = parse_pose(body, frame)
    if kind == 'location':
        entry = targets.plain_entry(store, robot, path[0])
    else:
        keys = ['instances', path[1], path[2]] if override else ['targets', path[2]]
        entry = targets.robot_layer(store, robot, object_type, keys)
    data = targets.placed({'moves': 'base', **entry}, pose_dict(pose))
    try:
        return _written(store, store.save_target(reference, data, robot, instance_override=override, rename=rename))
    except (ValueError, KeyError) as error:
        raise _problem(error) from error


def _delete_base_target(store: ConfigStore, robot: str, kind: str, path: list[str]) -> str:
    store.reload()
    reference, override, object_type, _ = _target_address(store, robot, kind, path)
    users = _users(store, robot, kind, path)
    if users and not (kind == 'stance' and override and _type_has_variant(store, robot, path)):
        raise PointError(f'{"/".join(path)} is used by {", ".join(users)}; change those missions first')
    if kind == 'stance' and not override:  # an override without its own moves would be left without a target
        table = targets.robot_layer(store, robot, object_type, ['instances'])
        left = sorted(instance for instance, entries in table.items() if isinstance(entries, Mapping)
                      and isinstance(entries.get(path[2]), Mapping) and 'moves' not in entries[path[2]])
        if left:
            raise PointError(f'{"/".join(path)} is overridden for {", ".join(left)}: delete those overrides first')
    try:
        return _written(store, store.delete_target(reference, robot, instance_override=override))
    except (ValueError, KeyError) as error:
        raise _problem(error) from error


def _save_hand_target(store: ConfigStore, object_type: str, point: str, settings: dict, poses: dict,
                      robot: str | None) -> str:
    """save_object_point() on a targets cell: the hand target ``point`` of the type (objects/<type>.yaml), the
    robot's elbow and grip_m in robots/<robot>/objects/<type>.yaml. Keys the editor does not show are kept."""
    if not settings and not poses:
        raise PointError('nothing to save')
    if not robot:
        raise PointError('a target of an object type is saved for a robot: needs the robot')
    if targets.type_kinds(store, robot, object_type).get(point) != 'hand':
        raise PointError(f'object {object_type!r} has no point {point!r}')
    own = targets.type_layer(store, object_type, point)
    if 'moves' not in own:
        raise PointError(f'{object_type} {point} is defined in {robot_objects_file(robot, object_type)}, not in '
                         f'objects/{object_type}.yaml: edit it in the file')
    mine = targets.robot_layer(store, robot, object_type, ['targets', point])
    data = {**own, **{key: mine[key] for key in ROBOT_SETTINGS if key in mine}}
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
        return _written(store, store.save_target(type_reference(store, object_type, point), data, robot))
    except (ValueError, KeyError) as error:
        raise _problem(error) from error


def relative_poses(store: ConfigStore, pose: Pose2D) -> dict:
    """The robot pose in the cell and relative to every placed object (for 'set from robot'), nearest first."""
    instances = sorted(store.object_instances().values(),
                       key=lambda i: math.hypot(i.pose.x - pose.x, i.pose.y - pose.y))
    return {'pose': pose_dict(pose),
            'relative': [{'instance': i.name, 'object': i.object,
                          'distance': round(math.hypot(i.pose.x - pose.x, i.pose.y - pose.y), 3),
                          'offset': pose_dict(pose.relative_to(i.pose, i.object))} for i in instances]}
