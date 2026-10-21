"""Validating config files with structured results: ``validate_file`` (one file), ``check_pack`` (all of them, what
``taskweave check`` prints) and ``dry_run_pack`` (all of them with an edit in place, nothing written).

A problem is a dict ``{path, pack, where, line, col, message, severity}``: ``path``/``pack`` name the file as the
catalogue does, ``where`` is a JSON pointer into it ('' = the whole file), ``line``/``col`` (1-based) are set when
the file's text is known, ``severity`` is ``error`` (it fails at run time) or ``warning`` (it loads, but something
is dangling or unused). Cross-file checks read the rest of the pack as it is on disk; the file's own content is the
data passed in.
"""

from __future__ import annotations

import io
import json
import shutil
import tempfile
from pathlib import Path, PurePosixPath
from types import SimpleNamespace

import yaml
from jsonschema import Draft202012Validator
from jsonschema.exceptions import best_match

from . import catalogue, overlay, packs
from . import target_store as ts
from . import targets as model
from .limits import joint_limits, joint_problems
from .store import ConfigError, ConfigStore

ERROR, WARNING = 'error', 'warning'


def pointer(parts) -> str:
    """JSON pointer of a key path."""
    return ''.join('/' + str(p).replace('~', '~0').replace('/', '~1') for p in parts)


def _problem(path: str, pack: str, parts, message: str, severity: str = ERROR) -> dict:
    return {'path': path, 'pack': pack, 'where': pointer(parts), 'line': None, 'col': None, 'message': message,
            'severity': severity}


def _plain(value):
    return json.loads(json.dumps(value))


def _round_trip(text: str):
    from ruamel.yaml import YAML

    return YAML().load(io.StringIO(text))


def _locate(document, parts) -> tuple[int, int] | None:
    """Line and column (1-based) of the node at ``parts``; of the deepest node that exists if it is missing."""
    node, position = document, None
    for part in parts:
        try:
            if isinstance(node, dict):
                line, col = node.lc.key(part)
            elif isinstance(node, list):
                line, col = node.lc.item(int(part))
            else:
                break
            node = node[part if isinstance(node, dict) else int(part)]
        except (KeyError, IndexError, ValueError, AttributeError, TypeError):
            break
        position = (line + 1, col + 1)
    if position is None and hasattr(document, 'lc'):
        position = (document.lc.line + 1, document.lc.col + 1)
    return position


def _schema_problems(schema: dict, data, path: str, pack: str) -> list[dict]:
    problems = []
    for error in sorted(Draft202012Validator(schema).iter_errors(data), key=lambda e: list(e.absolute_path)):
        shown = best_match(error.context) if error.context else error
        shown = shown or error
        parts = list(shown.absolute_path)
        if shown.validator == 'additionalProperties' and isinstance(shown.instance, dict):
            known = set(shown.schema.get('properties', {})) if isinstance(shown.schema, dict) else set()
            extra = [k for k in shown.instance if k not in known]
            if len(extra) == 1 and shown.message.startswith('Additional properties'):
                parts.append(extra[0])
        problems.append(_problem(path, pack, parts, shown.message))
    return problems


def _context(store: ConfigStore, path: str, pack: str, data) -> SimpleNamespace:
    robot = catalogue.robot_of(path, pack)
    parts = PurePosixPath(path).parts
    rest = '/'.join(parts[2:]) if pack == 'cell' and robot else path
    return SimpleNamespace(path=path, pack=pack, data=data if data is not None else {}, robot=robot, rest=rest,
                           name=PurePosixPath(path).name.split('.')[0])


# ---- cross-file checks: f(store, c) -> [(key path, message, severity)] ----------------------------------------------
def _names_exist(store, names, existing, parts_of, what, severity=WARNING):
    return [(parts_of(n), f'{what} {n!r} does not exist', severity) for n in names if n not in existing]


def _cross_profile(store, c):
    from ..embodiment import check_profile, profile_warnings

    out = []
    if c.robot and c.data.get('name') not in (None, c.robot):
        out.append((['name'], f'name is {c.data["name"]!r}, expected {c.robot!r}', ERROR))
    for joint, bounds in (c.data.get('joints') or {}).get('limits', {}).items():
        if bounds[0] > bounds[1]:
            out.append((['joints', 'limits', joint], f'lower limit {bounds[0]} is above upper {bounds[1]}', ERROR))
    out += [([], m, WARNING) for m in profile_warnings(c.data)]
    if c.robot:
        out += [([], m, ERROR) for m in check_profile(store.embodiment(c.robot), c.data)]
    return out


def _cross_embodiment(store, c):
    from ..embodiment import check_profile

    out = []
    if c.robot and c.data.get('name') not in (None, c.robot):
        out.append((['name'], f'name is {c.data["name"]!r}, expected {c.robot!r}', ERROR))
    return out + [([], m, ERROR) for m in check_profile(c.data, store.robot(c.robot).data)]


def _joint_values(store, robot, values, parts, severity=ERROR):
    limits, linear = joint_limits(store, robot), set(store.robot(robot).data.get('joints', {}).get('linear', []))
    return [(parts + [joint], message, severity) for joint, value in values.items() if isinstance(value, int | float)
            for message in joint_problems({joint: value}, limits, linear)]


def _cross_poses(store, c):
    out = []
    for name, pose in c.data.items():
        joints = pose.get('joints') if 'joints' in pose else pose
        if isinstance(joints, dict):
            out += _joint_values(store, c.robot, joints, [name] + (['joints'] if 'joints' in pose else []))
    return out


def _cross_held_poses(store, c):
    out = _names_exist(store, c.data, catalogue.enum_values(store, 'objects'), lambda n: [n], 'object type')
    for obj, poses in c.data.items():
        for name, held in poses.items():
            out += _joint_values(store, c.robot, held.get('joints', {}), [obj, name, 'joints'])
    return out


def _cross_object_points(store, c):
    objects, tools = catalogue.enum_values(store, 'objects'), store.embodiment(c.robot).get('tools', {})
    out = _names_exist(store, c.data, objects, lambda n: [n], 'object type')
    for obj, points in c.data.items():
        known = store.object(obj).get('points', {}) if obj in objects else None
        for point, settings in points.items():
            if known is not None and point not in known:
                out.append(([obj, point], f'object type {obj!r} has no point {point!r}', WARNING))
            out += [([obj, point, 'grip_m', t], f'robot has no tool {t!r}', WARNING)
                    for t in settings.get('grip_m', {}) if tools and t not in tools]
    return out


def _cross_skill_overrides(store, c):
    keys = catalogue.enum_values(store, 'skill_keys')    # a skill or a part of one: move.hand
    out = []
    for name in c.data:
        if name in keys:
            continue
        head = name.partition('.')[0]
        parts = [k for k in keys if k.startswith(head + '.')]
        hint = f' (the parts of {head} are {", ".join(parts)})' if parts and '.' in name else ''
        out.append(([name], f'skill {name!r} does not exist{hint}', WARNING))
    return out


def _cross_stances(store, c):
    objects, instances = catalogue.enum_values(store, 'objects'), catalogue.enum_values(store, 'instances')
    return (_names_exist(store, c.data.get('objects', {}), objects, lambda n: ['objects', n], 'object type')
            + _names_exist(store, c.data.get('instances', {}), instances, lambda n: ['instances', n],
                           'placed object'))


def _cross_object_instances(store, c):
    objects = catalogue.enum_values(store, 'objects')
    return [([name, 'object'], f'object type {i["object"]!r} does not exist (objects/{i["object"]}.yaml)', ERROR)
            for name, i in c.data.items() if i.get('object') not in objects]


def _cross_object_type(store, c):
    from ..embodiment import check_grasp

    out = [] if c.data.get('name') in (None, c.name) else [
        (['name'], f'name is {c.data["name"]!r} but the file is objects/{c.name}.yaml (it is found by file name)',
         WARNING)]
    for robot in store.robot_names():
        try:
            embodiment = store.embodiment(robot)
        except ConfigError:
            continue
        for point, settings in c.data.get('points', {}).items():
            out += [(['points', point], f'robot {robot!r}: {m}', WARNING) for m in check_grasp(settings, embodiment)]
    return out + (_cross_object_type_targets(store, c) if store.targets_format() else [])


def navigation_problems(data: dict, base: dict) -> list[tuple[list, str]]:
    """What the robot's base allows: sideways only when holonomic, speeds within base.limits."""
    driving, out = data.get('driving', {}), []
    if driving.get('sideways') and base.get('type') != 'holonomic':
        out.append((['driving', 'sideways'], f'the robot base is {base.get("type")!r} and cannot drive sideways'))
    limits = base.get('limits', {})
    for key, limit in (('max_forward', 'linear'), ('max_reverse', 'linear'), ('max_sideways', 'linear'),
                       ('max_turn', 'angular')):
        if key in driving and limit in limits and driving[key] > limits[limit]:
            out.append((['driving', key], f'{driving[key]} is above the base limit {limits[limit]}'))
    return out


def _cross_navigation(store, c):
    return [(parts, message, ERROR) for parts, message in navigation_problems(c.data, store.robot(c.robot).base)]


def _cross_mission(store, c):
    from ..plan import IncludeError, Plan, expand_includes

    steps = c.data.get('steps', [])
    ids = [s.get('id', f's{i}') for i, s in enumerate(steps, start=1)]
    out = [(['steps', i, 'id'], f'step id {s["id"]!r} is used more than once', ERROR)
           for i, s in enumerate(steps) if 'id' in s and ids.count(s['id']) > 1]
    out += [(['steps', i, 'replan_from'], f'replan_from {s["replan_from"]!r} is not a step of this mission', ERROR)
            for i, s in enumerate(steps) if s.get('replan_from') and s['replan_from'] not in ids]
    skills = catalogue.enum_values(store, 'skills')
    out += [(['steps', i, 'skill'], f'no skills/{s["skill"]}.yaml: no implementation is configured', WARNING)
            for i, s in enumerate(steps) if s.get('skill') and s['skill'] not in skills]
    if c.data.get('robot') and c.data['robot'] not in store.robot_names():
        out.append((['robot'], f'robot {c.data["robot"]!r} is not one of this cell\'s robots', WARNING))
    try:
        Plan.from_mission(expand_includes(c.data, lambda n: c.data if n == c.name else store.mission(n)), 'check')
    except (IncludeError, ValueError, KeyError, ConfigError) as error:
        out.append(([], str(error.args[0]) if isinstance(error, KeyError) and error.args else str(error), ERROR))
    return out


def _cross_skill(store, c):
    from ..skill import load_skill_class

    out = []
    for i, implementation in enumerate(c.data.get('implementations', [])):
        try:
            load_skill_class(implementation['class'])
        except ImportError as error:
            if (error.name or '').startswith('taskweave'):
                out.append((['implementations', i, 'class'], f'cannot load {implementation["name"]}: {error}', ERROR))
            # else: a ROS package this machine lacks (message types ...): the class cannot be checked here
        except (AttributeError, TypeError, ValueError) as error:
            out.append((['implementations', i, 'class'], f'cannot load {implementation["name"]}: {error}', ERROR))
    return out


def _cross_operator(store, c):
    skills = catalogue.enum_values(store, 'skills')
    return [(['joy', 'actions', i, 'skill'], f'no skills/{a["skill"]}.yaml', WARNING)
            for i, a in enumerate((c.data.get('joy') or {}).get('actions', [])) if a.get('skill') not in skills]


# ---- targets (cell schema 4) ----------------------------------------------------------------------------------------
def _place_as_names(store) -> set[str]:
    """Names missions give to objects they put down (grip place_as): run-time instances."""
    found = set()
    for name in store.mission_names():
        try:
            steps = store.mission(name).get('steps', [])
        except ConfigError:
            continue
        found |= {s['args']['place_as'] for s in steps if isinstance(s.get('args'), dict)
                  and isinstance(s['args'].get('place_as'), str)}
    return found


def _joints_part(store, robot, joints, parts, severity=ERROR):
    """Problems of a ``joints:`` part for one robot: groups, list lengths, values in range, torso auto without a
    torso to search or lift."""
    from .limits import target_joint_problems

    out = [(parts, message, severity) for message in target_joint_problems(store, robot, joints)]
    data = store.robot(robot).data
    if joints.get('torso') == 'auto' and not (data.get('torso') or {}).get('joint') \
            and not (data.get('ik') or {}).get('torso'):
        out.append((parts + ['torso'], 'torso: auto, but this robot has no torso search or lift', severity))
    return out


def _target_entries(store, robot, table, frame, parts, joints_severity=ERROR):
    """Checks of every complete target of a table: its own rules, joints part, tools against the robot."""
    from ..embodiment import check_grasp

    out = []
    for name, entry in table.items():
        if 'moves' not in entry:
            continue
        out += [(parts + [name], m, ERROR) for m in model.target_problems(entry, frame)]
        if entry['moves'] == 'joints' and frame != 'base':
            out.append((parts + [name], "a joints target has no frame: it goes to the robot's targets.yaml", ERROR))
        if robot and entry.get('joints'):
            out += _joints_part(store, robot, entry['joints'], parts + [name, 'joints'], joints_severity)
        if robot and entry['moves'] == 'hand':
            out += [(parts + [name], m, WARNING)
                    for m in check_grasp({**entry, **entry.get('alone', {})}, store.embodiment(robot))]
            tools = store.embodiment(robot).get('tools', {})
            out += [(parts + [name, 'grip_m', t], f'robot has no tool {t!r}', WARNING)
                    for t in entry.get('grip_m', {}) if tools and t not in tools]
    return out


def _held_references(store, entry_type, reference, parts):
    """An object target's ``type`` and ``reference`` name a type and one of its object targets."""
    out = []
    types = catalogue.enum_values(store, 'objects')
    if entry_type is not None and entry_type not in types:
        out.append((parts + ['type'], f'object type {entry_type!r} does not exist (objects/{entry_type}.yaml)',
                    WARNING))
    if reference is not None:
        names = {t: [n for n, m in ts.type_target_kinds(store, None, t).items() if m == 'object']
                 for t in ([entry_type] if entry_type in types else types)}
        if not any(reference in n for n in names.values()):
            out.append((parts + ['reference'], f'no object target {reference!r}'
                        + (f' on {entry_type}' if entry_type in types else ' on any object type'), ERROR))
    return out


def _loads(store, robot) -> bool:
    try:
        store.robot(robot)
        return True
    except ConfigError:
        return False


def _cross_cell_targets(store, c):
    out = []
    for robot in store.robot_names():   # 2.8: a plain name has one meaning
        try:
            mine = ts.robot_targets(store, robot)
        except ConfigError:
            continue
        out += [([name], f'target {name!r} is in {ts.CELL_FILE} and {ts.robot_file_label(robot)}: rename one', ERROR)
                for name in mine if name in c.data]
    out += _target_entries(store, None, c.data, 'cell', [])
    robots = [r for r in store.robot_names() if _loads(store, r)]
    for name, entry in c.data.items():
        if entry.get('moves') == 'object':
            out += _held_references(store, entry.get('type'), entry.get('reference'), [name])
        if entry.get('joints') and robots:  # a cell target may suit one robot only: the first one tells the groups
            out += _joints_part(store, robots[0], entry['joints'], [name, 'joints'], WARNING)
    return out


def _cross_robot_targets(store, c):
    try:
        cell = ts.cell_targets(store)
    except ConfigError:
        cell = {}
    out = [([name], f'target {name!r} is in {ts.CELL_FILE} and {ts.robot_file_label(c.robot)}: rename one', ERROR)
           for name in c.data if name in cell]
    out += _target_entries(store, c.robot, c.data, 'base', [])
    for name, entry in c.data.items():
        if entry.get('moves') == 'object':
            out += _held_references(store, entry.get('type'), entry.get('reference'), [name])
    return out


def _instance_tables(file_data):
    return [(['instances', i], table) for i, table in file_data.get('instances', {}).items()]


def _type_checks(store, robot, type, own, theirs, here):
    """What the two files of an object type (``own``: objects/<type>.yaml, ``theirs``: the robot's) say together:
    settings need a target, moves agree, instances exist. ``here``: which of them is the one checked."""
    out = []
    own_t, their_t = own.get('targets', {}), theirs.get('targets', {})
    defined = {n for table in (own_t, their_t) for n, e in table.items() if 'moves' in e}
    for data in (own, theirs):
        defined |= {n for _, table in _instance_tables(data) for n, e in table.items() if 'moves' in e}
    checked = own if here == 'own' else theirs
    for parts, table in [(['targets'], checked.get('targets', {})), *_instance_tables(checked)]:
        for name, entry in table.items():
            if 'moves' not in entry and name not in defined:
                out.append((parts + [name], f'settings for an unknown target {name!r} of {type} objects: no target '
                            'of that name says what it moves', ERROR))
    for name in set(own_t) & set(their_t):
        a, b = own_t[name].get('moves'), their_t[name].get('moves')
        if a and b and a != b:
            out.append((['targets', name, 'moves'], f'target {name!r} moves {a} in objects/{type}.yaml but {b} in '
                        f'robots/{robot}/objects/{type}.yaml: they must agree (rename one of them)', ERROR))
    placed, configured = store.object_instances(state=False), _place_as_names(store)
    for inst in checked.get('instances', {}):
        if inst not in placed and inst not in configured:
            out.append((['instances', inst], f'placed object {inst!r} does not exist (cell/objects.yaml; no mission '
                        'puts it down with place_as)', WARNING))
        elif inst in placed and placed[inst].object != type:
            out.append((['instances', inst], f'{inst!r} is a {placed[inst].object}, not a {type}', WARNING))
    return out


def _merged_problems(robot, type, own, theirs):
    """The merged targets of a type (per name, no instance) must be complete and valid."""
    out = []
    tables = ((f'objects/{type}.yaml', own.get('targets', {})), (f'robots/{robot}/objects/{type}.yaml',
                                                                 theirs.get('targets', {})))
    for name in dict.fromkeys([*own.get('targets', {}), *theirs.get('targets', {})]):
        layers = [(label, table[name]) for label, table in tables if name in table]
        if not any('moves' in e for _, e in layers):
            continue
        try:
            merged = model.merge_layers(name, layers)
        except model.TargetError:
            continue    # moves disagree: reported by _type_checks
        out += [(['targets', name], f"merged with the robot's settings: {m}", ERROR)
                for m in model.target_problems(merged, 'object')]
    return out


def _cross_object_type_targets(store, c):
    """The targets part of objects/<type>.yaml (cell schema 4)."""
    from ..embodiment import check_grasp

    out = []
    if not c.data.get('targets') and not c.data.get('instances'):
        return out
    out += _target_entries(store, None, c.data.get('targets', {}), 'object', ['targets'])
    for name, entry in c.data.get('targets', {}).items():
        if entry.get('moves') == 'object':
            out += _held_references(store, entry.get('type'), entry.get('reference'), ['targets', name])
    for robot in store.robot_names():
        try:
            embodiment = store.embodiment(robot)
            theirs = ts.robot_object_file(store, robot, c.name)
        except ConfigError:
            continue
        out += _type_checks(store, robot, c.name, c.data, theirs, 'own')
        out += _merged_problems(robot, c.name, c.data, theirs)
        for name, entry in c.data.get('targets', {}).items():
            if entry.get('moves') == 'hand':
                out += [(['targets', name], f'robot {robot!r}: {m}', WARNING)
                        for m in check_grasp({**entry, **entry.get('alone', {})}, embodiment)]
    return out


def _cross_robot_objects(store, c):
    robot, type = c.robot, c.name
    out = [([], f'object type {type!r} does not exist (objects/{type}.yaml)', WARNING)
           ] if type not in catalogue.enum_values(store, 'objects') else []
    own = ts.object_file(store, type)
    out += _target_entries(store, robot, c.data.get('targets', {}), 'object', ['targets'])
    for parts, table in _instance_tables(c.data):
        out += _target_entries(store, robot, table, 'object', parts)
    out += _type_checks(store, robot, type, own, c.data, 'theirs')
    out += _merged_problems(robot, type, own, c.data)
    kinds = ts.type_target_kinds(store, robot, type)
    kinds.update({n: e['moves'] for n, e in c.data.get('targets', {}).items() if 'moves' in e})
    handled = c.data.get('handled_from')
    if handled is not None:
        if handled not in kinds:
            out.append((['handled_from'], f'handled_from {handled!r} is not a target of {type} objects', ERROR))
        elif kinds[handled] != 'base':
            out.append((['handled_from'], f'handled_from {handled!r} moves {kinds[handled]}, it must be a base '
                        'target', ERROR))
    elif own.get('yaw_symmetry_deg') and any(m == 'base' for m in kinds.values()):
        out.append(([], f'{type} objects have yaw_symmetry_deg, but handled_from names no base target: a turned '
                        'object is handled as placed', WARNING))
    tools = store.embodiment(robot).get('tools', {})
    for name, entry in c.data.get('targets', {}).items():
        out += [(['targets', name, 'grip_m', t], f'robot has no tool {t!r}', WARNING)
                for t in entry.get('grip_m', {}) if tools and t not in tools]
    return out


def _cross_cell_pack(store, c):
    version = c.data.get('schema_version')
    return [] if isinstance(version, int) and packs.CELL_SCHEMA_OLDEST <= version <= packs.CELL_SCHEMA else [
        (['schema_version'], f'is {version}, this taskweave reads {packs.CELL_SCHEMA_OLDEST} to {packs.CELL_SCHEMA}',
         ERROR)]


def _cross_robot_pack(store, c):
    version = c.data.get('schema_version')
    out = [] if isinstance(version, int) and packs.ROBOT_SCHEMA_OLDEST <= version <= packs.ROBOT_SCHEMA else [
        (['schema_version'], f'is {version}, this taskweave reads {packs.ROBOT_SCHEMA_OLDEST} to {packs.ROBOT_SCHEMA}',
         ERROR)]
    if c.robot and c.data.get('name') not in (None, c.robot):
        out.append((['name'], f'name is {c.data["name"]!r}, expected {c.robot!r}', ERROR))
    return out


def _cross_system(store, c):
    return [(['robot_packs', i], f'robot pack location {entry!r} does not exist', ERROR)
            for i, entry in enumerate(c.data.get('robot_packs', [])) if not (store.root / entry).exists()]


CROSS = {'profile': _cross_profile, 'embodiment': _cross_embodiment, 'poses': _cross_poses,
         'held_poses': _cross_held_poses, 'object_points': _cross_object_points,
         'skill_overrides': _cross_skill_overrides, 'stances': _cross_stances,
         'object_instances': _cross_object_instances, 'object_type': _cross_object_type,
         'navigation': _cross_navigation, 'mission': _cross_mission, 'skill': _cross_skill, 'skill_part': _cross_skill,
         'operator': _cross_operator, 'cell_pack': _cross_cell_pack, 'robot_pack': _cross_robot_pack,
         'system': _cross_system, 'cell_targets': _cross_cell_targets, 'robot_targets': _cross_robot_targets,
         'robot_objects': _cross_robot_objects}


# ---- one file --------------------------------------------------------------------------------------------------------
def validate_file(store: ConfigStore, path: str, data_or_text=None, pack: str = 'cell') -> list[dict]:
    """Problems of one file: its schema, then the cross-file checks of its kind against the rest of the pack on
    disk. ``data_or_text``: the content to check (parsed data, or YAML text, which also gives ``line``/``col``);
    None: the file as it is on disk. ``pack``: 'cell' or 'robot:<name>'; a cell pack copy of a robot pack file
    (robots/<robot>/profile.yaml) may be a keyed override, which is checked as laid over the pack's file."""
    path = PurePosixPath(path).as_posix()
    kind = catalogue.kind_of(path, pack)
    if data_or_text is None:
        target = catalogue.file_path(store, path, pack)
        if not target.is_file():
            return [_problem(path, pack, [], 'file not found')]
        data_or_text = target.read_text(encoding='utf-8')
    document = None
    if isinstance(data_or_text, str):
        if not path.endswith(catalogue.YAML_SUFFIXES):
            return []
        try:
            document = _round_trip(data_or_text)
            data = _plain(document)
        except Exception as error:  # noqa: BLE001 - ruamel's errors have no common base worth naming
            mark = getattr(error, 'problem_mark', None)
            problem = _problem(path, pack, [], f'invalid YAML: {getattr(error, "problem", None) or error}')
            if mark is not None:
                problem.update(line=mark.line + 1, col=mark.column + 1)
            return [problem]
    else:
        data = _plain(data_or_text)
    if kind is None or kind.schema is None:
        return []
    if data is None:
        data = {}
    c = _context(store, path, pack, data)
    if c.robot and pack == 'cell' and overlay.is_keyed(data):    # a keyed override: check the merged file
        base = store.override_base(c.robot, c.rest)
        if base is None:
            return [_problem(path, pack, [], f'keyed override ({overlay.MARKER}: {overlay.KEYS}), but no robot pack '
                                             f'has {c.robot}/{c.rest}')]
        c.data = overlay.overlay(store._read_yaml(base), data, c.rest)
    problems = _schema_problems(kind.schema, c.data, path, pack)
    if not problems and kind.kind in CROSS:
        try:
            problems = [_problem(path, pack, parts, message, severity)
                        for parts, message, severity in CROSS[kind.kind](store, c)]
        except (ConfigError, KeyError, ValueError) as error:
            message = str(error.args[0]) if isinstance(error, KeyError) and error.args else str(error)
            problems = [_problem(path, pack, [], f'not checked against the rest of the pack: {message}', WARNING)]
    if document is not None:
        for problem in problems:
            position = _locate(document, [p for p in problem['where'].split('/')[1:]])
            if position:
                problem['line'], problem['col'] = position
    return problems


# ---- the pack --------------------------------------------------------------------------------------------------------
def check_pack(store: ConfigStore) -> list[dict]:
    """The problems of every file of the cell pack and its robots' packs (those with a schema or cross-check)."""
    return [p for _, problems in checked_files(store) for p in problems]


def checked_files(store: ConfigStore) -> list[tuple[dict, list[dict]]]:
    """(catalogue entry, problems) for every file that has a schema; what ``taskweave check`` prints."""
    result = []
    for entry in store.files(hashes=False):
        if entry['schema_id'] is None or not entry['editable'] and entry['kind'] not in ('cell_pack', 'robot_pack'):
            continue
        result.append((entry, validate_file(store, entry['path'], None, entry['pack'])))
    return result


def dry_run_pack(store: ConfigStore, path: str, data_or_text, pack: str = 'cell') -> list[dict]:
    """check_pack with ``data_or_text`` in place of the file ``path`` of ``pack``, on a throwaway copy (nothing is
    written to the pack; meshes are not copied). Shows what else an edit breaks."""
    path = PurePosixPath(path).as_posix()
    text = data_or_text if isinstance(data_or_text, str) else yaml.safe_dump(_plain(data_or_text), sort_keys=False)
    ignore = shutil.ignore_patterns('meshes', '.git', '__pycache__', '*.stl', '*.dae', '*.obj')
    with tempfile.TemporaryDirectory(prefix='tw_dry_') as temp:
        temp = Path(temp)
        root = temp / 'cell'
        shutil.copytree(store.root, root, ignore=ignore)
        try:
            system = store.system()
        except ConfigError:     # system.yaml is invalid: the copy has it too, as it is
            system = {}
        first = []
        for name in {catalogue.robot_of(path, pack)} - {None}:  # the edited robot pack, copied, searched first
            if pack != 'cell':
                original = next(iter(store.robot_packs(name)), None)
                if original is not None:
                    shutil.copytree(original.path, temp / 'packs' / name, ignore=ignore)
                    first.append(temp / 'packs' / name)
        copy_store = ConfigStore(root, first_packs=tuple(first))
        # relative robot pack locations of the real cell must still be found from the copy
        absolute = [str((store.root / entry).resolve()) for entry in system.get('robot_packs', [])]
        if absolute:
            (root / 'system.yaml').write_text(yaml.safe_dump({**system, 'robot_packs': absolute}), encoding='utf-8')
        if pack == 'cell' and path == 'system.yaml':
            try:
                new = yaml.safe_load(text) or {}
                new['robot_packs'] = [str((store.root / e).resolve()) for e in new.get('robot_packs', [])]
                text = yaml.safe_dump(new, sort_keys=False)
            except (yaml.YAMLError, AttributeError, TypeError):
                pass
        target = root / path if pack == 'cell' else temp / 'packs' / catalogue.robot_of(path, pack) / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text, encoding='utf-8')
        # messages name files of the copy: show the real ones, so they compare equal to the pack's own
        names = {str(root): str(store.root), str(root.resolve()): str(store.root)}
        for folder in first:
            orig = next(iter(store.robot_packs(folder.name)), None)
            names.update({str(folder): str(orig.path), str(folder.resolve()): str(orig.path)} if orig else {})

        def real(problems: list[dict]) -> list[dict]:
            for problem in problems:
                for fake, true in sorted(names.items(), key=lambda kv: -len(kv[0])):
                    problem['message'] = problem['message'].replace(fake, true)
            return problems

        if not first and pack != 'cell':    # no pack of that robot yet: a new pack folder
            return real(validate_file(copy_store, path, text, pack))
        copy_store.reload()
        problems = check_pack(copy_store)
        edited = validate_file(copy_store, path, text, pack)   # the edit again, with line and column
        return real([p for p in problems if not (p['path'] == path and p['pack'] == pack)] + edited)
