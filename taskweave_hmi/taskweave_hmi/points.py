"""Points for the HMI: list, edit, rename and delete locations and stances. No ROS dependency.

Locations live in ``cell/locations.yaml`` (cell frame). Stances live in ``robots/<robot>/stances.yaml`` as offsets
in the object's frame, per object type (``objects``) or per placed instance (``instances``, overrides). Placed
objects (``cell/objects.yaml``) are read-only here: the scene is built from the same layout.

Every point lists the missions that refer to it. A point that a mission needs cannot be renamed or deleted; a
stance override can be deleted when the object type has the same variant to fall back on. Placeholders such as
``{fullest_stack}`` are resolved at run time and are not counted.
"""

from __future__ import annotations

import math
from collections.abc import Iterator, Mapping

from taskweave_core.config import ConfigError, ConfigStore, Pose2D, schemas
from taskweave_core.config.store import LOCATION_PREFIX, STANCE_PREFIX, TOOL_SUFFIX, VARIANT_SEPARATOR

LOCATIONS = 'cell/locations.yaml'
SCOPES = ('objects', 'instances')


class PointError(ValueError):
    """A point cannot be changed; the message says why."""


def stances_file(robot: str) -> str:
    return f'robots/{robot}/stances.yaml'


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
    overrides = store.stance_offsets(robot).get('instances', {})
    if variant in overrides.get(name, {}):
        return 'instances', name, variant
    return 'objects', instance.object, variant


def usage(store: ConfigStore, robot: str) -> dict[tuple, list[str]]:
    """Which missions use which point: ('location', name), (scope, owner, variant) or ('object_point', type,
    point) -> mission names."""
    result: dict[tuple, list[str]] = {}
    locations = store.locations()
    instances = store.object_instances()
    for mission_name in store.mission_names():
        try:
            mission = store.mission(mission_name)
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


def overview(store: ConfigStore, robot: str) -> dict:
    """Everything the points view shows, with the missions using each point."""
    store.reload()
    used = usage(store, robot)
    stances = store.stance_offsets(robot)
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
            'points': object_points(store, object_type, used),
            'variants': [{'variant': v, 'offset': pose_dict(Pose2D.from_dict(p)),
                          'used_by': used.get(('objects', object_type, v), [])} for v, p in variants.items()],
            'instances': placed,
        })
    return {
        'robot': robot,
        'files': {'locations': LOCATIONS, 'stances': stances_file(robot), 'objects': 'cell/objects.yaml'},
        'locations': [{'name': n, 'pose': pose_dict(p), 'used_by': used.get(('location', n), [])}
                      for n, p in store.locations().items()],
        'objects': objects,
        # go_to_point's straight final approach when a point does not set its own
        'approach_default': float(store.skill_params('go_to_point').get('approach_m', 0.0)),
        'elbow_sides': schemas.ELBOW_SIDES,
    }


def object_points(store: ConfigStore, object_type: str, used: dict) -> list[dict]:
    """Points on an object type (objects/<type>.yaml): tool poses in the object's frame, approach and elbow."""
    try:
        points = store.object(object_type).get('points', {})
    except ConfigError:
        return []
    result = []
    for name in points:
        point = store.object_point(object_type, name)
        result.append({
            'name': name, 'approach_m': point.get('approach_m'), 'elbow': point.get('elbow'),
            'tools': {key[:-len(TOOL_SUFFIX)]: {'xyz': list(pose.xyz), 'rpy': list(pose.rpy)}
                      for key, pose in point.items() if key.endswith(TOOL_SUFFIX) and pose is not None},
            'used_by': used.get(('object_point', object_type, name), []),
        })
    return result


def save_object_point(store: ConfigStore, object_type: str, point: str, body: Mapping) -> str:
    """Approach (m or null = skill default), elbow (side or null = robot default) and tool poses of a point."""
    settings = {key: body[key] for key in ('approach_m', 'elbow') if key in body}
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
    try:
        path = store.save_object_point_settings(object_type, point, settings) if settings else None
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
    return variant in store.stance_offsets(robot).get(scope, {}).get(owner, {})


def _type_has_variant(store: ConfigStore, robot: str, path: list[str]) -> bool:
    """An instance override can go when its object type has the same variant to fall back on."""
    instance = store.object_instances()[path[1]]
    stances = store.stance_offsets(robot)
    return path[2] in stances.get('objects', {}).get(instance.object, {})


def relative_poses(store: ConfigStore, pose: Pose2D) -> dict:
    """The robot pose in the cell and relative to every placed object (for 'set from robot'), nearest first."""
    instances = sorted(store.object_instances().values(),
                       key=lambda i: math.hypot(i.pose.x - pose.x, i.pose.y - pose.y))
    return {'pose': pose_dict(pose),
            'relative': [{'instance': i.name, 'object': i.object,
                          'distance': round(math.hypot(i.pose.x - pose.x, i.pose.y - pose.y), 3),
                          'offset': pose_dict(pose.relative_to(i.pose, i.object))} for i in instances]}
