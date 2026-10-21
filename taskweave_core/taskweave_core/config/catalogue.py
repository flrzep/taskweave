"""The file catalogue: which config files a cell pack and its robot packs hold, which schema each has and what a
change to it needs (reload, restart). One table (KINDS) is the source for ``ConfigStore.files()``, ``file_kinds()``,
``json_schema()`` and the validation (check.py), so an editor needs no knowledge of the layout.

A kind's id is also its schema id. ``reload`` lists what must happen for a change to take effect (RELOAD); a list
means parts of it take effect at different times.
"""

from __future__ import annotations

import copy
import hashlib
import re
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from . import schemas

RELOAD = {
    'live': 'read on every use: effective at once',
    'mission_start': 'read when the next mission starts',
    'reload_service': 'applied on request, between goals: taskweave config reload (a changed implementation class or '
                      'a parameter a connection was made from needs a restart of the skill servers)',
    'launch': 'read at launch (task handler, HMI bridge, operator input, Nav2, MoveIt): restart the task stack',
    'sim': 'read by the simulator application: restart it (not read by taskweave)',
}
TEXT_SUFFIXES = ('.yaml', '.yml', '.json', '.urdf', '.srdf', '.xacro')   # what the listing shows (no meshes)
YAML_SUFFIXES = ('.yaml', '.yml')
SCHEMA_URI = 'https://json-schema.org/draft/2020-12/schema'


@dataclass(frozen=True)
class Kind:
    kind: str                   # also the schema id
    pack: str                   # cell | robot
    pattern: str                # relative to the pack; * = one path part, ** = any depth
    title: str
    reload: tuple
    schema: dict | None = None
    no_schema_reason: str = ''
    editable: bool = True
    editable_reason: str = ''   # why not
    template: dict | None = None    # a start for a new file ({name} = its name)

    @property
    def regex(self) -> re.Pattern:
        return re.compile('^' + re.escape(self.pattern).replace(r'\*\*', '.+').replace(r'\*', '[^/]+') + '$')

    @property
    def creatable(self) -> bool:
        return self.pack == 'cell' and self.editable


MANAGED = 'managed by taskweave config migrate (schema_version): not edited by hand'
_K = Kind
KINDS = (
    # ---- cell pack
    _K('cell_pack', 'cell', 'pack.yaml', 'Cell pack', ('launch',), schemas.PACK, editable=False,
       editable_reason=MANAGED),
    _K('system', 'cell', 'system.yaml', 'Installation', ('launch', 'sim'), schemas.SYSTEM, template={}),
    _K('operator', 'cell', 'operator.yaml', 'Operator inputs', ('launch',), schemas.OPERATOR, template={}),
    _K('locations', 'cell', 'cell/locations.yaml', 'Named base poses', ('live',), schemas.POINTS2D, template={}),
    _K('cell_targets', 'cell', 'cell/targets.yaml', 'Targets in the cell frame', ('live',), schemas.CELL_TARGETS,
       template={}),
    _K('object_instances', 'cell', 'cell/objects.yaml', 'Placed objects', ('live',), schemas.OBJECT_INSTANCES,
       template={}),
    _K('layout', 'cell', 'cell/layout.yaml', 'Cell layout', ('sim',),
       no_schema_reason='application content (fixtures, zones): read by the simulator and cell skills, not by '
                        'taskweave'),
    _K('scene', 'cell', 'sim/scene.yaml', 'Simulator scene', ('sim',),
       no_schema_reason='read by the simulator application, not by taskweave'),
    _K('object_type', 'cell', 'objects/*.yaml', 'Object type', ('live',), schemas.OBJECT, template={'name': '{name}'}),
    _K('skill', 'cell', 'skills/*.yaml', 'Skill implementations and parameters', ('reload_service',),
       schemas.SKILL_PARAMS, template={'implementations': []}),
    _K('skill_part', 'cell', 'skills/*/*.yaml', 'Parameters of a part of a skill', ('reload_service',),
       schemas.SKILL_PARAMS, template={'implementations': []}),
    _K('mission', 'cell', 'missions/*.yaml', 'Mission', ('mission_start',), schemas.MISSION,
       template={'goal': '', 'steps': []}),
    _K('reference_run', 'cell', 'tests/*.reference.yaml', 'Smoke test reference', ('live',),
       no_schema_reason='smoke run record: compared by taskweave smoke, no schema', editable=False,
       editable_reason='written by taskweave smoke run --save-reference'),
    _K('stances', 'cell', 'robots/*/stances.yaml', 'Stances of a robot', ('live',), schemas.STANCES, template={}),
    _K('held_poses', 'cell', 'robots/*/held_poses.yaml', 'Held poses of a robot', ('live',), schemas.HELD_POSES,
       template={}),
    _K('object_points', 'cell', 'robots/*/object_points.yaml', 'Object point settings of a robot', ('live',),
       schemas.ROBOT_OBJECT_POINTS, template={}),
    _K('robot_objects', 'cell', 'robots/*/objects/*.yaml', 'Targets of a robot on an object type', ('live',),
       schemas.ROBOT_OBJECT_FILE, template={}),
    # ---- robot pack (a cell pack copy of these is an override, found as robots/<robot>/<file>)
    _K('robot_pack', 'robot', 'pack.yaml', 'Robot pack', ('launch',), schemas.PACK, editable=False,
       editable_reason=MANAGED),
    _K('profile', 'robot', 'profile.yaml', 'Robot profile', ('launch',), schemas.ROBOT_PROFILE),
    _K('embodiment', 'robot', 'embodiment.yaml', 'Robot embodiment', ('launch',), schemas.EMBODIMENT),
    _K('poses', 'robot', 'poses.yaml', 'Named joint poses', ('live',), schemas.JOINT_POSES),
    _K('robot_targets', 'robot', 'targets.yaml', 'Base-frame and joint targets', ('live',), schemas.ROBOT_TARGETS),
    _K('navigation', 'robot', 'navigation.yaml', 'Navigation envelope and driving', ('live', 'launch'),
       schemas.NAVIGATION),
    _K('skill_overrides', 'robot', 'skill_overrides.yaml', 'Motion options per skill', ('live',),
       schemas.SKILL_OVERRIDES),
    _K('sim', 'robot', 'sim.yaml', 'Simulator inputs', ('sim',),
       no_schema_reason='application-defined simulator input, not read by taskweave'),
    _K('pose_targets', 'robot', 'pose_targets.yaml', 'Pose solver inputs', ('sim',),
       no_schema_reason='input of the application\'s pose solver, not read by taskweave'),
    _K('moveit', 'robot', 'moveit/**', 'MoveIt files', ('launch',),
       no_schema_reason='MoveIt formats (URDF, SRDF, kinematics, joint limits, collision)'),
    _K('nav2_params', 'robot', 'nav2_params.yaml', 'Nav2 parameters', ('launch',),
       no_schema_reason='Nav2 parameter file, laid over the packaged defaults'),
)
BY_KIND = {k.kind: k for k in KINDS}
OTHER = Kind('other', 'cell', '**', 'Other file', ('live',), no_schema_reason='not a file taskweave reads',
             editable=False, editable_reason='not a file taskweave reads')

# dynamic enums an editor fills from the pack (enum_values): kind -> [(path in the exported schema, enum name,
# 'x-taskweave-enum' for a value, 'x-taskweave-enum-keys' for the keys of an object)]
_STEP = ('properties', 'steps', 'items', 'properties')
ENUM_HINTS = {
    'mission': [(('properties', 'robot'), 'robots', 'x-taskweave-enum'),
                (_STEP + ('skill',), 'skills', 'x-taskweave-enum'),
                (_STEP + ('include',), 'missions', 'x-taskweave-enum')],
    'operator': [(('properties', 'joy', 'properties', 'actions', 'items', 'properties', 'skill'), 'skills',
                  'x-taskweave-enum')],
    'object_instances': [(('additionalProperties', 'properties', 'object'), 'objects', 'x-taskweave-enum')],
    'stances': [(('properties', 'objects'), 'objects', 'x-taskweave-enum-keys'),
                (('properties', 'instances'), 'instances', 'x-taskweave-enum-keys')],
    'held_poses': [((), 'objects', 'x-taskweave-enum-keys'),
                   (('additionalProperties', 'additionalProperties', 'properties', 'joints'), 'joints',
                    'x-taskweave-enum-keys')],
    'object_points': [((), 'objects', 'x-taskweave-enum-keys')],
    'skill_overrides': [((), 'skills', 'x-taskweave-enum-keys')],
    'poses': [(('additionalProperties', 'oneOf', 0), 'joints', 'x-taskweave-enum-keys')],
    # targets (cell schema 4): the groups of a joints part, the type of an object target, the target a held one meets
    'cell_targets': [(('additionalProperties', 'properties', 'joints'), 'joint_groups', 'x-taskweave-enum-keys'),
                     (('additionalProperties', 'properties', 'type'), 'target_types', 'x-taskweave-enum'),
                     (('additionalProperties', 'properties', 'reference'), 'object_targets', 'x-taskweave-enum')],
    'robot_targets': [(('additionalProperties', 'properties', 'joints'), 'joint_groups', 'x-taskweave-enum-keys'),
                      (('additionalProperties', 'properties', 'type'), 'target_types', 'x-taskweave-enum'),
                      (('additionalProperties', 'properties', 'reference'), 'object_targets', 'x-taskweave-enum')],
    'object_type': [(('properties', 'targets', 'additionalProperties', 'properties', 'reference'), 'object_targets',
                     'x-taskweave-enum')],
    'robot_objects': [(('properties', 'handled_from'), 'base_targets_of', 'x-taskweave-enum'),
                      (('properties', 'targets', 'additionalProperties', 'properties', 'joints'), 'joint_groups',
                       'x-taskweave-enum-keys')],
}


def kind_of(path: str, pack: str = 'cell') -> Kind | None:
    """The kind of a file by its path relative to its pack (``pack``: 'cell' or 'robot:<name>'); None if no kind
    matches. In the cell pack robots/<robot>/<file> is a cell-specific file or an override of a robot pack file."""
    path = PurePosixPath(path).as_posix()
    if pack == 'cell':
        for kind in KINDS:
            if kind.pack == 'cell' and kind.regex.match(path):
                return kind
        parts = path.split('/')
        path = '/'.join(parts[2:]) if parts[0] == 'robots' and len(parts) > 2 else ''
    for kind in KINDS:
        if kind.pack == 'robot' and path and kind.regex.match(path):
            return kind
    return None


def robot_of(path: str, pack: str) -> str | None:
    """The robot a file belongs to: its robot pack, or robots/<robot>/ in the cell pack."""
    if pack.startswith('robot:'):
        return pack.split(':', 1)[1]
    parts = PurePosixPath(path).parts
    return parts[1] if len(parts) > 2 and parts[0] == 'robots' else None


def robot_arg(pack: str) -> str | None:
    """The robot of ``pack`` ('cell': None, 'robot:<name>': the name); ValueError for anything else, a name that is
    not a plain name included (it becomes a folder name)."""
    if pack == 'cell':
        return None
    name = pack[6:] if pack.startswith('robot:') else ''
    if not re.fullmatch(schemas.NAME['pattern'], name):
        raise ValueError(f"pack is 'cell' or 'robot:<name>' (letters, digits, \"_\", \"-\"), not {pack!r}")
    return name


def guard(base: Path, path: str) -> Path:
    """``base / path``, checked: ``path`` relative without '..', no symlink in it (parents and file), inside ``base``.
    ValueError otherwise."""
    rel = PurePosixPath(path)
    if not rel.parts or rel.is_absolute() or '..' in rel.parts or '\\' in path or ':' in rel.parts[0]:
        raise ValueError(f'{path!r} is not a path inside the pack')
    target = base
    for part in rel.parts:
        target = target / part
        if target.is_symlink():
            raise ValueError(f'{path!r}: {part} is a symbolic link')
    if not target.resolve().is_relative_to(base.resolve()):
        raise ValueError(f'{path!r} is outside the pack')
    return target


def file_path(store, path: str, pack: str = 'cell') -> Path:
    """The file of ``path`` in ``pack`` on disk (existing or not): the cell pack, or the first pack of the robot
    that has it, else the robot's first pack. ValueError for a path or robot name that leaves the pack."""
    robot = robot_arg(pack)
    if robot is None:
        return guard(store.root, path)
    packs = store.robot_packs(robot)
    base = next((p.path for p in packs if (p.path / path).exists()), packs[0].path if packs
                else store.root / 'robots' / robot)
    return guard(base, path)


def _sha(*parts: bytes) -> str:
    return hashlib.sha256(b'\0'.join(parts)).hexdigest()


def file_hash(store, path: str, pack: str = 'cell') -> str | None:
    """sha256 of a file's bytes (None if absent). A robot pack file the cell has an override of (written by a save)
    hashes the pack file and the override together: a change of either is a change of what is in effect."""
    target = file_path(store, path, pack)
    if pack == 'cell':
        return _sha(target.read_bytes()) if target.is_file() else None
    over = guard(store.root, f'robots/{robot_arg(pack)}/{path}')
    if not over.is_file():
        return hashlib.sha256(target.read_bytes()).hexdigest() if target.is_file() else None
    return _sha(target.read_bytes() if target.is_file() else b'', over.read_bytes())


def _schema_fields(kind: Kind) -> dict:
    return {'schema_id': kind.kind if kind.schema is not None else None,
            'no_schema_reason': '' if kind.schema is not None else kind.no_schema_reason}


def _entry(kind: Kind, path: str, pack: str, source: str, robot: str | None, target: Path, hashes: bool) -> dict:
    editable = kind.editable and target.suffix in YAML_SUFFIXES
    reason = '' if editable else kind.editable_reason or 'not a YAML file'
    return {'path': path, 'pack': pack, 'kind': kind.kind, 'title': kind.title, 'robot': robot, 'source': source,
            'editable': editable, 'editable_reason': reason, **_schema_fields(kind), 'reload': list(kind.reload),
            'hash': hashlib.sha256(target.read_bytes()).hexdigest() if hashes and editable else None}


def _walk(folder: Path):
    for target in sorted(folder.rglob('*')):
        parts = target.relative_to(folder).parts
        if target.is_file() and target.suffix in TEXT_SUFFIXES and not any(p.startswith('.') or p == '__pycache__'
                                                                           for p in parts):
            yield '/'.join(parts), target


def files(store, hashes: bool = True) -> list[dict]:
    """Every config file of the cell pack and of its robots' packs: ``{path (relative to its pack), pack ('cell' |
    'robot:<name>'), kind, title, robot, source (cell | override: a cell copy of a robot pack file | robot_pack |
    installed: a read-only robot pack), editable, editable_reason, schema_id, no_schema_reason, reload, hash}``."""
    from .store import ConfigError

    found = []
    for path, target in _walk(store.root):
        kind = kind_of(path) or OTHER
        found.append(_entry(kind, path, 'cell', 'cell' if kind.pack == 'cell' else 'override', robot_of(path, 'cell'),
                            target, hashes))
    try:
        robots = store.robot_names()
    except ConfigError:     # system.yaml does not load: check_pack reports it, the cell's own files stay editable
        robots = []
    for robot in robots:
        seen = set()
        try:
            robot_packs = store.robot_packs(robot)
        except ConfigError:
            continue    # check_pack reports a pack that does not load
        for pack in robot_packs:
            for path, target in _walk(pack.path):
                kind = kind_of(path, f'robot:{robot}')
                if path in seen or kind is None:
                    continue
                seen.add(path)
                source = 'robot_pack' if pack.location.writable else 'installed'
                found.append(_entry(kind, path, f'robot:{robot}', source, robot, target, hashes))
                if found[-1]['hash']:
                    found[-1]['hash'] = file_hash(store, path, f'robot:{robot}')
    return found


def file_kinds() -> list[dict]:
    """The kinds of file a pack can hold (also those not present, for a new file): ``{kind, title, pack ('cell' |
    'robot'), pattern, creatable, template, editable, editable_reason, schema_id, no_schema_reason, reload}``."""
    return [{'kind': k.kind, 'title': k.title, 'pack': k.pack, 'pattern': k.pattern, 'creatable': k.creatable,
             'template': copy.deepcopy(k.template), 'editable': k.editable, 'editable_reason': k.editable_reason,
             **_schema_fields(k), 'reload': list(k.reload)} for k in KINDS]


def new_file(kind: str, name: str = '', robot: str | None = None) -> tuple[str, dict]:
    """The path (in the cell pack) and starting content of a new file of ``kind``: ``name`` fills the ``*`` of
    the pattern (the first one is the robot of a robots/*/ file), the starting content is a template that may not
    validate until it is filled in."""
    entry = BY_KIND.get(kind)
    if entry is None or not entry.creatable:
        raise KeyError(f'cannot create a file of kind {kind!r}; creatable: '
                       f'{", ".join(k.kind for k in KINDS if k.creatable)}')
    path = entry.pattern
    if path.startswith('robots/*/'):
        if not robot:
            raise ValueError(f'{kind} needs a robot')
        path = path.replace('*', robot, 1)
    if '*' in path:
        if not re.fullmatch(schemas.NAME['pattern'], name or ''):
            raise ValueError(f'{kind} needs a name: letters, digits, "_" and "-", starting with a letter')
        path = path.replace('*', name)
    return path, _fill_name(entry.template, name)


def _fill_name(template, name: str):
    if isinstance(template, dict):
        return {k: _fill_name(v, name) for k, v in template.items()}
    if isinstance(template, list):
        return [_fill_name(v, name) for v in template]
    return template.replace('{name}', name) if isinstance(template, str) else template


def json_schema(kind_or_path: str, pack: str = 'cell') -> dict:
    """One self-contained JSON Schema (2020-12) for a kind (``object_type``) or a file path. Dynamic enums are not
    baked in: ``x-taskweave-enum`` names the enum of a value, ``x-taskweave-enum-keys`` that of an object's keys
    (``ConfigStore.enum_values``). Raises KeyError for a kind with no schema or an unknown kind or path."""
    kind = BY_KIND.get(kind_or_path) or kind_of(kind_or_path, pack)
    if kind is None:
        raise KeyError(f'no kind or file {kind_or_path!r}; kinds: {", ".join(BY_KIND)}')
    if kind.schema is None:
        raise KeyError(f'{kind.kind} has no schema: {kind.no_schema_reason}')
    schema = copy.deepcopy(kind.schema)
    schema.setdefault('$schema', SCHEMA_URI)
    schema.setdefault('$id', f'taskweave/{kind.kind}')
    schema.setdefault('title', kind.title)
    schema.setdefault('description', f'{kind.pattern} ({"cell" if kind.pack == "cell" else "robot"} pack); '
                                     + ' / '.join(RELOAD[r] for r in kind.reload))
    for steps, name, keyword in ENUM_HINTS.get(kind.kind, ()):
        parent, node = None, schema
        try:
            for step in steps:     # shared fragments (NAME) are copied, so the mark stays where it was put
                parent, node = node, node[step]
                parent[step] = node = copy.deepcopy(node)
        except (KeyError, IndexError):
            continue
        node[keyword] = name
    return schema


def _plain_target_names(store, robot: str) -> list[str]:
    return [n for key, names in store.target_names(robot).items() if key != 'placed objects' for n in names
            if '/' not in n]


def _kinds_of(store, robot: str, type: str) -> dict[str, str]:
    from .target_store import type_target_kinds

    return type_target_kinds(store, robot, type)


def enum_values(store, name: str, robot: str | None = None) -> list[str]:
    """Values of a dynamic enum in this pack: robots, skills, missions, objects (types), instances, locations,
    stances, object_points, joint_poses, held_poses, hands, joints, tools, and for the targets layout targets (plain
    names), object_targets (names on object types), target_types, base_targets_of (base targets on object types:
    ``handled_from``), joint_groups. Robot-bound ones take ``robot`` (none: the union over the cell's robots)."""
    from .limits import joint_groups, joint_limits

    robots = [robot] if robot else store.robot_names()
    root = store.root

    def stems(folder: str) -> list[str]:
        return sorted(p.stem for p in (root / folder).glob('*.yaml'))

    def per_robot(function) -> list[str]:
        values = set()
        for each in robots:
            try:
                values.update(function(each))
            except Exception:   # noqa: BLE001 - a robot that does not load adds nothing
                continue
        return sorted(values)

    table = {
        'robots': lambda: sorted(store.robot_names()), 'skills': lambda: stems('skills'),
        'missions': lambda: stems('missions'), 'objects': lambda: stems('objects'),
        'instances': lambda: sorted(store.object_instances(state=False)),
        'locations': lambda: sorted(store.locations()),
        'stances': lambda: per_robot(store.stance_names),
        'object_points': lambda: per_robot(lambda r: store.point_names(r)['object points']),
        'joint_poses': lambda: per_robot(lambda r: store.joint_poses(r)),
        'held_poses': lambda: per_robot(lambda r: {n for poses in store.held_poses(r).values() for n in poses}),
        'hands': lambda: per_robot(lambda r: store.robot(r).data.get('grippers', {})),
        'joints': lambda: per_robot(lambda r: joint_limits(store, r)),
        'tools': lambda: per_robot(lambda r: store.embodiment(r).get('tools', {})),
        'targets': lambda: per_robot(lambda r: _plain_target_names(store, r)) if store.targets_format() else [],
        'object_targets': lambda: sorted({n for names in store.object_target_names(robot).values() for n in names})
        if store.targets_format() else [],
        'target_types': lambda: stems('objects'),
        'base_targets_of': lambda: per_robot(lambda r: {
            n for t in store.object_target_names(r) for n, m in _kinds_of(store, r, t).items() if m == 'base'})
        if store.targets_format() else [],
        'joint_groups': lambda: per_robot(lambda r: joint_groups(store, r)),
    }
    if name not in table:
        raise KeyError(f'unknown enum {name!r}; known: {", ".join(table)}')
    return table[name]()
