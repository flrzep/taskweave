"""Reading targets from a cell pack in the targets layout (cell schema 4): the lookup of a reference, the layers of an
object type, the names a plan can use, and the shapes the old point readers returned (deprecated, for the callers that
have not moved to ``ConfigStore.target`` yet). ConfigStore's methods call these; the functions take the store first.

Files: ``cell/targets.yaml`` (cell frame), ``robots/<robot>/targets.yaml`` (base frame, joints; robot pack with a cell
override per key), ``objects/<type>.yaml`` ``targets`` and ``instances`` (the object's frame), ``robots/<robot>/objects/
<type>.yaml`` ``targets``, ``instances`` and ``handled_from`` (the same, this robot's). See targets.py for the model.
"""

from __future__ import annotations

import difflib
import logging
import re

from . import schemas
from . import targets as model
from .poses import ObjectInstance, Pose2D, Pose3D
from .targets import Target, TargetError

log = logging.getLogger(__name__)

CELL_FILE = 'cell/targets.yaml'
KIND_LIST = {'base': 'base targets', 'hand': 'hand targets', 'object': 'object targets', 'joints': 'joint targets'}
SEPARATOR = '/'  # <instance>/<name>
OLD_FORM = re.compile(r'[A-Za-z_][A-Za-z0-9_\-]*\.[A-Za-z_][A-Za-z0-9_\-]*')  # <object>.<point>, removed


# ---- the files -----------------------------------------------------------------------------------------------------
def robot_file_label(robot: str) -> str:
    return f'robots/{robot}/targets.yaml'


def cell_targets(store) -> dict:
    return store._load(CELL_FILE, schemas.CELL_TARGETS, required=False, default={})


def robot_targets(store, robot: str) -> dict:
    return store._load(robot_file_label(robot), schemas.ROBOT_TARGETS, required=False, default={})


def object_file(store, type: str) -> dict:
    return store._load(f'objects/{type}.yaml', schemas.OBJECT, required=False, default={})


def robot_object_file(store, robot: str, type: str) -> dict:
    return store._load(f'robots/{robot}/objects/{type}.yaml', schemas.ROBOT_OBJECT_FILE, required=False, default={})


def type_names(store, robot: str | None = None) -> list[str]:
    """Object types with a file: objects/<type>.yaml, and the robot's objects/<type>.yaml."""
    names = {p.stem for p in (store.root / 'objects').glob('*.yaml')}
    if robot:
        names |= {p.stem for p in (store.root / 'robots' / robot / 'objects').glob('*.yaml')}
    return sorted(names)


# ---- the layers of an object type (2.7) ----------------------------------------------------------------------------
def entry_layers(store, robot: str | None, type: str, name: str, instance: str | None = None) -> list[tuple[str, dict]]:
    """The entries that define object-frame target ``name`` of ``type``, weakest first: the type's, the robot's, the
    type's for the instance, the robot's for the instance, as ``[(file label, entry)]``. Without ``robot`` only the
    type's own files count."""
    own, theirs = object_file(store, type), (robot_object_file(store, robot, type) if robot else {})
    own_label, their_label = f'objects/{type}.yaml', f'robots/{robot}/objects/{type}.yaml'
    layers = [(own_label, own.get('targets', {}).get(name)), (their_label, theirs.get('targets', {}).get(name))]
    if instance:
        layers += [
            (own_label, own.get('instances', {}).get(instance, {}).get(name)),
            (their_label, theirs.get('instances', {}).get(instance, {}).get(name)),
        ]
    return [(label, entry) for label, entry in layers if entry is not None]


def type_target_kinds(store, robot: str | None, type: str, instance: str | None = None) -> dict[str, str]:
    """``{name: moves}`` of the object-frame targets of ``type`` (the type's, then the robot's, then the
    instance's), in file order. A name is listed once; ``moves`` is the first layer's that gives it."""
    own, theirs = object_file(store, type), (robot_object_file(store, robot, type) if robot else {})
    tables = [own.get('targets', {}), theirs.get('targets', {})]
    if instance:
        tables += [own.get('instances', {}).get(instance, {}), theirs.get('instances', {}).get(instance, {})]
    kinds: dict[str, str | None] = {}
    for table in tables:
        for name, entry in table.items():
            kinds[name] = kinds.get(name) or entry.get('moves')
    return {name: moves for name, moves in kinds.items() if moves}


def _merged(store, robot, type: str, name: str, reference: str, instance: str | None) -> tuple[dict, tuple]:
    layers = entry_layers(store, robot, type, name, instance)
    if not layers:
        raise _missing_on_type(store, robot, type, name)
    labels = ', '.join(dict.fromkeys(label for label, _ in layers))
    if not any('moves' in entry for _, entry in layers):
        raise TargetError(
            f'settings for an unknown target {name!r} of {type} objects in {labels}: no target of '
            f'that name defines what it moves ({", ".join(sorted(type_target_kinds(store, robot, type)))})'
        )
    merged = model.merge_layers(name, layers)
    problems = model.target_problems(merged, 'object')
    if merged['moves'] == 'joints':
        problems.append('a joints target has no object frame: put it in robots/<robot>/targets.yaml')
    if problems:
        raise TargetError(f'target {reference!r} ({labels}): {problems[0]}')
    return merged, tuple(dict.fromkeys(label for label, _ in layers))


def _missing_on_type(store, robot, type: str, name: str) -> TargetError:
    have = sorted(type_target_kinds(store, robot, type))
    if robot:
        for other in store.robot_names():
            if other != robot and name in robot_object_file(store, other, type).get('targets', {}):
                return TargetError(
                    f'robot {robot!r} has no {name!r} target for {type} objects (robots/{robot}/objects/{type}.yaml)'
                )
    return TargetError(f'object {type!r} has no target {name!r} (it has: {", ".join(have) or "none"})')


# ---- symmetric objects ---------------------------------------------------------------------------------------------
def handled_offset(store, robot: str, instance: ObjectInstance) -> Pose2D | None:
    """The offset of the robot's ``handled_from`` base target for the instance (object frame), None without the key."""
    name = robot_object_file(store, robot, instance.object).get('handled_from')
    if not name:
        return None
    label = f'robots/{robot}/objects/{instance.object}.yaml'
    if name not in type_target_kinds(store, robot, instance.object, instance.name):
        raise TargetError(f'handled_from {name!r} is not a target of {instance.object} objects ({label})')
    merged, _ = _merged(store, robot, instance.object, name, f'{instance.name}/{name}', instance.name)
    if merged['moves'] != 'base':
        raise TargetError(f'handled_from {name!r} moves {merged["moves"]}, it must be a base target ({label})')
    step = (merged.get('path') or [merged])[-1]
    return Pose2D(float(step['x']), float(step['y']), float(step['yaw']), instance.object)


# ---- resolving a reference (3) -------------------------------------------------------------------------------------
def _old_syntax(reference: str) -> str | None:
    for prefix, word in model.OLD_PREFIXES.items():
        if reference.startswith(prefix):
            return f'{reference!r}: the {word}: prefix was removed in schema 4: run taskweave config migrate'
    if SEPARATOR not in reference and OLD_FORM.fullmatch(reference):
        return (
            f'{reference!r}: the <object>.<point> form was removed in schema 4: use <instance>/<name>; run '
            'taskweave config migrate'
        )
    return None


def _instance_error(store, name: str, reference: str) -> KeyError:
    """No such placed object: one taken at run time says so (the existing text), else the instances there are."""
    if name in store.object_instances(state=False):
        return TargetError(store.missing_instance(name, '').args[0])
    return TargetError(
        f'unknown target {reference!r}: no object {name!r} in cell/objects.yaml, instances: '
        f'{", ".join(sorted(store.object_instances())) or "none"}'
    )


def _unknown_plain(store, robot: str, name: str, cell: dict, mine: dict) -> TargetError:
    near = difflib.get_close_matches(name, [*cell, *mine], n=1)
    text = f'unknown target {name!r}'
    on_types = [(type, name) for type in type_names(store, robot) if name in type_target_kinds(store, robot, type)]
    if on_types:
        type = on_types[0][0]
        placed = sorted(i.name for i in store.object_instances().values() if i.object == type)
        example = f', e.g. {placed[0]}/{name}' if placed else ''
        return TargetError(
            f'{text}: it is a target of {type} objects and needs an instance: '
            f'<instance>/{name}{example} (cell: {len(cell)}, robot: {len(mine)})'
        )
    if near:
        text += f'; did you mean {near[0]}?'
    return TargetError(f'{text} (cell: {len(cell)}, robot: {len(mine)}; object targets need <instance>/<name>)')


def resolve(store, reference: str, robot: str, near: Pose2D | None = None, kinds=None, context: str = '') -> Target:
    """``ConfigStore.target``: ``<name>`` (a cell, base-frame or joint target) or ``<instance>/<name>`` (an
    object-frame target of a placed object, as the robot handles it from ``near``)."""
    old = _old_syntax(reference)
    if old:
        raise TargetError(old)
    if SEPARATOR not in reference:
        found = _plain(store, reference, robot)
    else:
        name, _, rest = reference.partition(SEPARATOR)
        if not name or not rest or SEPARATOR in rest:
            raise TargetError(f'{reference!r}: expected <name> or <instance>/<name>')
        found = _on_instance(store, reference, name, rest, robot, near)
    if kinds is not None and found.kind not in ((kinds,) if isinstance(kinds, str) else tuple(kinds)):
        wanted = ' or '.join(model.MOVES_WORD[k] for k in ((kinds,) if isinstance(kinds, str) else kinds))
        raise TargetError(
            f'{reference!r} moves {model.MOVES_WORD[found.kind]}: '
            + (context or f'expected a target that moves {wanted}')
        )
    return found


def _plain(store, name: str, robot: str) -> Target:
    cell, mine = cell_targets(store), robot_targets(store, robot)
    in_cell, in_robot = name in cell, name in mine
    if in_cell and in_robot:
        raise TargetError(f'target {name!r} is in {CELL_FILE} and {robot_file_label(robot)}: rename one')
    if not in_cell and not in_robot:
        raise _unknown_plain(store, robot, name, cell, mine)
    entry, frame, label = (cell[name], 'cell', CELL_FILE) if in_cell else (mine[name], 'base', robot_file_label(robot))
    problems = model.target_problems(entry, frame)
    if problems:
        raise TargetError(f'target {name!r} ({label}): {problems[0]}')
    return model.build_target(
        entry['moves'], 'none' if entry['moves'] == 'joints' else frame, name, name, entry, (label,)
    )


def _on_instance(store, reference: str, name: str, target: str, robot: str, near: Pose2D | None) -> Target:
    instances = store.object_instances()
    if name not in instances:
        raise _instance_error(store, name, reference)
    if instances[name].stack_count == 0:
        raise TargetError(store.missing_instance(name, '').args[0])
    instance = instances[name] if robot is None else store.handling_instance(instances[name], robot, near)
    merged, files = _merged(store, robot, instance.object, target, reference, instance.name)
    frame = 'object'
    return model.build_target(
        merged['moves'], frame, target, reference, merged, files, type=instance.object, instance=instance
    )


def resolve_type(store, robot: str | None, type: str, name: str) -> Target:
    """An object-frame target of a type with no placed object (the pose in the object's own frame): the type's
    own targets, with the robot's settings on top when ``robot`` is given."""
    merged, files = _merged(store, robot, type, name, f'{type}/{name}', None)
    return model.build_target(merged['moves'], 'object', name, f'{type}/{name}', merged, files, type=type)


# ---- names ---------------------------------------------------------------------------------------------------------
def target_names(store, robot: str) -> dict[str, list[str]]:
    """Every name a plan can use, by kind: ``{'base targets': [...], 'hand targets': [...], 'object targets': [...],
    'joint targets': [...], 'placed objects': [...]}``; object-frame names are ``<instance>/<name>`` for the placed
    instances."""
    names: dict[str, list[str]] = {title: [] for title in KIND_LIST.values()}
    for table in (cell_targets(store), robot_targets(store, robot)):
        for name, entry in table.items():
            names[KIND_LIST[entry['moves']]].append(name)
    placed = store.object_instances()
    for instance in placed.values():
        for name, moves in type_target_kinds(store, robot, instance.object, instance.name).items():
            names[KIND_LIST[moves]].append(f'{instance.name}{SEPARATOR}{name}')
    return {**{title: sorted(values) for title, values in names.items()}, 'placed objects': sorted(placed)}


def object_target_names(store, robot: str | None = None) -> dict[str, list[str]]:
    """``{object type: [target names]}`` of the object-frame targets (the robot's too when given), sorted."""
    return {type: sorted(type_target_kinds(store, robot, type)) for type in type_names(store, robot)}


def all_targets(store, robot: str) -> dict[str, Target]:
    """Every resolvable target by reference: the plain ones and, for each placed object, its type's."""
    found = {
        name: resolve(store, name, robot)
        for table in (cell_targets(store), robot_targets(store, robot))
        for name in table
    }
    for instance in store.object_instances().values():
        if instance.stack_count == 0:
            continue
        for name in type_target_kinds(store, robot, instance.object, instance.name):
            reference = f'{instance.name}{SEPARATOR}{name}'
            found[reference] = resolve(store, reference, robot)
    return found


# ---- the shapes of the old readers (deprecated) --------------------------------------------------------------------
def _joint_values(store, robot: str, joints: dict) -> dict[str, float]:
    from .limits import group_values

    return group_values(store, robot, joints)


def legacy_locations(store) -> dict[str, Pose2D]:
    return {name: build_pose(entry) for name, entry in cell_targets(store).items() if entry['moves'] == 'base'}


def build_pose(entry: dict) -> Pose2D:
    step = (entry.get('path') or [entry])[-1]
    return Pose2D(float(step['x']), float(step['y']), float(step['yaw']), 'cell')


def legacy_stance_variants(store, robot: str, instance: ObjectInstance) -> dict[str, Pose2D]:
    result = {}
    for name, moves in type_target_kinds(store, robot, instance.object, instance.name).items():
        if moves == 'base':
            merged, _ = _merged(store, robot, instance.object, name, f'{instance.name}/{name}', instance.name)
            step = (merged.get('path') or [merged])[-1]
            result[name] = Pose2D(float(step['x']), float(step['y']), float(step['yaw']), instance.object)
    return result


def legacy_stance_offsets(store, robot: str) -> dict:
    """``{'objects': {type: {variant: {x, y, yaw}}}, 'instances': {...}}`` from the robot's objects files."""
    objects: dict = {}
    instances: dict = {}
    placed = store.object_instances(state=False)
    for type in type_names(store, robot):
        poses = {}
        for name, moves in type_target_kinds(store, robot, type).items():  # the type's layers with the robot's on top
            if moves == 'base':
                try:
                    poses[name] = _xyaw(_merged(store, robot, type, name, f'{type}/{name}', None)[0])
                except TargetError as error:
                    log.warning('stance %r of %r objects is left out of the old offsets: %s', name, type, error)
        if poses:
            objects[type] = poses
        file = robot_object_file(store, robot, type)
        for instance, table in file.get('instances', {}).items():
            poses = {}
            for name, entry in table.items():
                if 'x' in entry or 'y' in entry or 'yaw' in entry or 'path' in entry:
                    merged = _stance_entry(store, robot, placed.get(instance), name, entry, instance)
                    if merged is not None:
                        poses[name] = _xyaw(merged)
            if poses:
                instances[instance] = poses
    return {**({'objects': objects} if objects else {}), **({'instances': instances} if instances else {})}


def _stance_entry(store, robot: str, placed, name: str, entry: dict, instance: str) -> dict | None:
    """The merged entry of a stance override of ``instance`` (the type's layers first, then the override, per key), None
    when it is not a complete base target. An instance that is not placed has no type: the entry as it stands."""
    merged = model.merge_layers(name, entry_layers(store, robot, placed.object, name, instance)) if placed else entry
    if merged.get('moves', 'base') != 'base':
        return None
    steps = merged.get('path') or [merged]
    return merged if all(k in step for step in steps for k in model.BASE_KEYS) else None


def _xyaw(entry: dict) -> dict:
    step = (entry.get('path') or [entry])[-1]
    return {'x': step['x'], 'y': step['y'], 'yaw': step['yaw']}


def _tools_axes(target: Target) -> dict:
    out = {}
    for key, pose in target.tools.items():
        approach, closing = model.axes_from_rpy(pose.rpy)
        out[key[: -len(model.TOOL_SUFFIX)]] = {
            'tcp': list(pose.xyz),
            'approach': list(approach),
            'closing': list(closing),
        }
    return out


def _robot_pose_targets(store, robot: str) -> dict[str, Target]:
    return _readable(store, robot, ('joints', 'hand'))[0]


def _readable(store, robot: str, kinds: tuple) -> tuple[dict[str, Target], dict[str, Exception]]:
    """The robot's targets of these kinds that resolve and whose joints fit the robot, and the ones that do not
    (``{name: error}``, logged): one bad target does not hide the others."""
    from .limits import group_values

    good: dict[str, Target] = {}
    bad: dict[str, Exception] = {}
    for name, entry in robot_targets(store, robot).items():
        if entry['moves'] not in kinds:
            continue
        try:
            target = resolve(store, name, robot)
            group_values(store, robot, target.joints)
        except (TargetError, model.GroupError) as error:
            bad[name] = error
            log.warning('target %r of robot %r is skipped by the old readers: %s', name, robot, error)
        else:
            good[name] = target
    return good, bad


def pose_target_problems(store, robot: str) -> dict[str, str]:
    """``{name: problem}`` of the robot's joint, hand and held-object targets that cannot be read (a problem in the
    file or joints the robot has no group for); the old pose readers skip these."""
    bad = {**_readable(store, robot, ('joints', 'hand'))[1], **_readable(store, robot, ('object',))[1]}
    return {name: str(error) for name, error in bad.items()}


def legacy_joint_poses(store, robot: str) -> dict[str, dict]:
    """name -> joint map or ``{joints, tools, elbow}`` as poses.yaml stored them."""
    result = {}
    for name, target in _robot_pose_targets(store, robot).items():
        joints = _joint_values(store, robot, target.joints)
        if target.kind == 'joints':
            result[name] = joints
        else:
            result[name] = {
                **({'joints': joints} if joints else {}),
                'tools': _tools_axes(target),
                **({'elbow': target.data['elbow']} if target.data.get('elbow') else {}),
            }
    return result


def legacy_named_pose(store, robot: str, name: str) -> dict:
    targets, bad = _readable(store, robot, ('joints', 'hand'))
    if name in bad:
        raise bad[name]
    if name not in targets:
        raise KeyError(f'unknown pose {name!r} for robot {robot!r}')
    target = targets[name]
    joints = _joint_values(store, robot, target.joints)
    if target.kind == 'joints':
        return {'joints': joints, 'tools': {}, 'elbow': None}
    return {'joints': joints, 'tools': _tools_axes(target), 'elbow': target.data.get('elbow')}


def _held_targets(store, robot: str) -> dict[str, Target]:
    return _readable(store, robot, ('object',))[0]


def legacy_held_poses(store, robot: str) -> dict:
    """``{object type: {name: {joints, path: [{xyz, rpy}]}}}``; a target with no ``type`` is listed under ``'*'``."""
    result: dict = {}
    for name, target in _held_targets(store, robot).items():
        steps = [{'xyz': list(p.xyz), 'rpy': list(p.rpy)} for p in target.poses]
        entry = {'path': steps, **({'joints': _joint_values(store, robot, target.joints)} if target.joints else {})}
        result.setdefault(target.data.get('type', '*'), {})[name] = entry
    return result


def legacy_held_pose(store, robot: str, obj: str, name: str) -> dict:
    bad = _readable(store, robot, ('object',))[1]
    if name in bad:
        raise bad[name]
    target = _held_targets(store, robot).get(name)
    if target is None or target.data.get('type') not in (None, obj):
        known = (
            ', '.join(sorted(n for n, t in _held_targets(store, robot).items() if t.data.get('type') in (None, obj)))
            or 'none'
        )
        raise KeyError(
            f'robot {robot!r} has no held pose {name!r} for {obj!r} objects ({robot_file_label(robot)}; '
            f'it has: {known})'
        )
    return {
        'joints': _joint_values(store, robot, target.joints),
        'path': [Pose3D(p.xyz, p.rpy, 'base') for p in target.poses],
    }


def legacy_robot_object_points(store, robot: str) -> dict:
    """``{type: {point: {elbow, grip_m}}}``: the robot's settings on a type's hand targets."""
    result: dict = {}
    for path in sorted((store.root / 'robots' / robot / 'objects').glob('*.yaml')):
        file = robot_object_file(store, robot, path.stem)
        for name, entry in file.get('targets', {}).items():
            settings = {k: entry[k] for k in model.ROBOT_SETTINGS if k in entry}
            if settings and entry.get('moves') in (None, 'hand'):
                result.setdefault(path.stem, {})[name] = settings
    return result


def legacy_point_names(store, robot: str) -> dict[str, list[str]]:
    from .limits import joint_groups

    cell, mine = cell_targets(store), robot_targets(store, robot)
    placed, contacts, points = [], [], []
    for type in type_names(store):
        points += [f'{type}.{n}' for n, m in type_target_kinds(store, robot, type).items() if m == 'hand']
    for name, instance in sorted(store.object_instances().items()):
        for target, moves in type_target_kinds(store, robot, instance.object).items():
            (placed if moves == 'hand' else contacts if moves == 'object' else []).append(f'{name}{SEPARATOR}{target}')
    poses = sorted(n for n, e in mine.items() if e['moves'] in ('joints', 'hand'))
    stances = []
    for instance in store.object_instances().values():
        for index, variant in enumerate(legacy_stance_variants(store, robot, instance)):
            stances.append('stance:' + (instance.name if index == 0 else f'{instance.name}{SEPARATOR}{variant}'))
    return {
        'locations': sorted(n for n, e in cell.items() if e['moves'] == 'base'),
        'stances': stances,
        'joint poses': poses,
        'tool poses': sorted(n for n, e in mine.items() if e['moves'] == 'hand'),
        'object points': points,
        'joint-only poses': sorted(n for n, e in mine.items() if e['moves'] == 'joints'),
        'joint groups': sorted(joint_groups(store, robot)),
        'placed object points': placed,
        'placed object contacts': contacts,
        'placed objects': sorted(store.object_instances()),
        'held poses': sorted(n for n, e in mine.items() if e['moves'] == 'object'),
        'via targets': [f'pose:{pose}' for pose in poses] + placed,
    }
