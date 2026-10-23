"""Loading, validating and querying a taskweave cell pack (a config directory) and the robot packs it uses.

Layout (all files YAML; files marked optional may be absent). robots/<model>/<file> is looked up in the cell pack
first, then in the robot packs (packs.py); the cell pack holds the robot's cell-specific files, and overrides of robot
pack files, keyed (only the keys the cell changes, laid over the pack's file) or whole (overlay.py):

    pack.yaml                      required: name, version, schema_version (packs.CELL_SCHEMA)
    system.yaml                    optional, robot_packs: robot pack locations (relative to the cell pack)
    missions/<name>.yaml           required per mission
    robots/<model>/profile.yaml    required per robot (robot pack): how the hardware is reached (topics, names)
    robots/<model>/embodiment.yaml required per robot (robot pack): what the robot is and can do
    robots/<model>/stances.yaml    optional (cell), robot-specific base poses relative to objects (per type, instance)
    robots/<model>/poses.yaml      optional (robot pack), named joint-space poses (used by move_to_pose)
    robots/<model>/targets.yaml    optional (robot pack, cell schema 4), base-frame and joint targets
    robots/<model>/objects/<type>.yaml optional (cell, schema 4), this robot's targets on an object type
    robots/<model>/skill_overrides.yaml optional (robot pack), per skill: this robot's motion options and speeds
    cell/locations.yaml            optional, named base poses in the cell frame
    cell/targets.yaml              optional (cell schema 4), targets in the cell frame
    cell/objects.yaml              optional, placed object instances: object type and pose in the cell frame
    objects/<name>.yaml            optional per object, taught tool poses
    skills/<skill>.yaml            per served skill: implementations and parameters
    operator.yaml                  optional, operator inputs (e.g. joystick buttons)
"""

from __future__ import annotations

import json
import math
import os
import threading
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from typing import Any

import yaml
from jsonschema import Draft202012Validator

from . import overlay, packs, schemas
from . import target_store as ts
from .poses import ObjectInstance, Pose2D, Pose3D, wrap_angle
from .targets import Target, TargetError

STANCE_PREFIX = 'stance:'
LOCATION_PREFIX = 'location:'
VARIANT_SEPARATOR = '/'     # stance:<instance>/<variant>
TOOL_SUFFIX = '_tool'       # an object point's tool pose key: <tool>_tool
POINT_FILE_SCHEMAS = {  # files that teaching and the HMI may edit (a file name matches any robot folder)
    'navigation.yaml': schemas.NAVIGATION, 'profile.yaml': schemas.ROBOT_PROFILE,  # profile: ik.choose only
    'cell/objects.yaml': schemas.OBJECT_INSTANCES,  # where an object was put down (gripper place_as)
    'cell/targets.yaml': schemas.CELL_TARGETS, 'targets.yaml': schemas.ROBOT_TARGETS,   # targets
}


def skill_file(skill: str) -> str:
    """The file of a skill's configuration, relative to the cell pack: ``skills/<skill>.yaml``; a part of a skill
    (``<skill>.<part>``) has its file in the skill's folder, ``skills/<skill>/<part>.yaml``."""
    return 'skills/' + skill.replace('.', '/') + '.yaml'


class ConfigError(Exception):
    """A config file is missing, unreadable or invalid. The message names the file and the key."""

    def __init__(self, path: Path | str, message: str, key: str = ''):
        self.path, self.key = str(path), key
        super().__init__(f'{path}{f" [{key}]" if key else ""}: {message}')


@dataclass(frozen=True)
class RobotProfile:
    """Maps robot roles (base, arms.left, grippers.right, ...) to hardware names."""

    name: str
    data: dict

    @property
    def base(self) -> dict:
        return self.data['base']

    def role(self, path: str) -> Any:
        """Look up a role by dotted path, e.g. ``role('grippers.left')``. Raises KeyError with the path."""
        node: Any = self.data
        for part in path.split('.'):
            if not isinstance(node, dict) or part not in node:
                raise KeyError(f'robot profile {self.name!r} has no role {path!r}')
            node = node[part]
        return node


STATE_DIR_ENV = 'TASKWEAVE_STATE_DIR'  # where run-time state lives (default: next to the config directory, state/)


class ConfigStore:
    """Read-mostly view of one config directory. Files are validated on first use and cached.

    Placed objects (cell/objects.yaml) are the start state; what happens to them at run time (items taken off a stack,
    objects put down) is kept in <state dir>/objects.yaml and laid over them by object_instances()."""

    def __init__(self, root: str | Path, check_schema: bool = True, first_packs: tuple[Path, ...] = ()):
        self.first_packs = tuple(Path(p) for p in first_packs)    # robot pack folders searched first (dry runs)
        self.root = Path(root).expanduser().resolve()
        if not self.root.is_dir():
            raise ConfigError(self.root, 'config directory does not exist')
        # False: a reader for the migrations (taskweave config migrate): a cell pack and robot packs of an older schema
        self.oldest_robot_pack = packs.ROBOT_SCHEMA_OLDEST if check_schema else packs.MIGRATION_OLDEST
        if check_schema:
            packs.check_cell_schema(self.root)
        self._cache: dict[Path, Any] = {}
        self._lock = threading.Lock()

    # ---- generic loading -------------------------------------------------
    def _load(self, relative: str, schema: dict, required: bool = True, default: Any = None) -> Any:
        """Load and validate a file. Cached per modification time, so edits by other processes are picked up.
        robots/<robot>/<file> is found through the robot file search path (robot_file)."""
        robot_file = self._robot_relative(relative)
        path = self.robot_file(*robot_file) if robot_file else self.root / relative
        base = self.override_base(*robot_file) if robot_file and path.is_relative_to(self.root) else None
        try:
            stamp = (_stamp(path), _stamp(base) if base else None)
        except OSError:
            stamp = None
        with self._lock:
            cached = self._cache.get(path)
            if cached is not None and cached[0] == stamp:
                return cached[1]
        if stamp is None:
            if required:
                raise ConfigError(path, 'file not found' + (f' (looked in {self.robot_search(robot_file[0])})'
                                                            if robot_file else ''))
            return default
        data = self._read_yaml(path)
        keyed = robot_file is not None and overlay.is_keyed(data)
        if keyed:  # a keyed cell override: laid over the robot pack's file, the result is validated
            if base is None:    # not over {}: the delta alone is not a file (and would hide a missing/misnamed pack)
                labels = ', '.join(p.label for p in self.robot_packs(robot_file[0])) or 'no robot pack'
                raise ConfigError(path, f'keyed override ({overlay.MARKER}: {overlay.KEYS}), but no robot pack has '
                                        f'{robot_file[0]}/{robot_file[1]} (looked in '
                                        f'{labels})')
            data = overlay.overlay(self._read_yaml(base), data, robot_file[1])
        if data is None:
            data = default if default is not None else {}
        errors = sorted(Draft202012Validator(schema).iter_errors(data), key=lambda e: list(e.absolute_path))
        if errors:
            first = errors[0]
            key = '.'.join(str(p) for p in first.absolute_path)
            more = f' (and {len(errors) - 1} more)' if len(errors) > 1 else ''
            raise ConfigError(f'{path} (laid over {base})' if keyed and base else path, first.message + more, key)
        with self._lock:
            self._cache[path] = (stamp, data)
        return data

    @staticmethod
    def _read_yaml(path: Path) -> Any:
        try:
            return yaml.safe_load(path.read_text(encoding='utf-8'))
        except yaml.YAMLError as error:
            raise ConfigError(path, f'invalid YAML: {error}') from error

    def reload(self) -> None:
        """Drop all cached files; the next access reads from disk again."""
        with self._lock:
            self._cache.clear()

    # ---- robot files: the cell pack first, then the robot packs (packs.py) ------
    def system(self) -> dict:
        """system.yaml: the installation (robot_packs; other keys are the application's); {} if absent."""
        return self._load('system.yaml', schemas.SYSTEM, required=False, default={})

    def robot_packs(self, robot: str | None = None) -> list:
        """The robot's packs in search order: env TASKWEAVE_ROBOT_PACKS, system.yaml robot_packs, installed.
        Without ``robot``: every pack at every location as ``{name, version, description, location, writable}``."""
        if robot is None:
            return packs.list_packs(self.root, self.system().get('robot_packs'), self.first_packs)
        return packs.robot_packs(self.root, self.system().get('robot_packs'), robot, self.first_packs,
                                 self.oldest_robot_pack)

    def robot_names(self) -> list[str]:
        """Robots this cell has files for (robots/<robot>/), plus system.yaml robot."""
        names = {p.name for p in (self.root / 'robots').glob('*') if p.is_dir()}
        return sorted(names | ({self.system()['robot']} if self.system().get('robot') else set()))

    def robot_file(self, robot: str, file: str) -> Path:
        """Where robots/<robot>/<file> (a file or a folder, e.g. moveit) is read from: the cell pack's copy if it
        has one, else the first robot pack that has it; found nowhere, its place in the cell pack (not existing)."""
        cell = self.root / 'robots' / robot / file
        if cell.exists():
            return cell
        return next((p.path / file for p in self.robot_packs(robot) if (p.path / file).exists()), cell)

    def robot_file_source(self, robot: str, file: str) -> dict:
        """Where a robot file comes from and where an edit of it goes (HMI, error messages):
        ``{'path', 'source': 'cell' | 'override' | 'robot pack' | 'missing', 'pack': robot pack label or None,
        'save_path', 'override': 'keys' | 'file' | None, 'override_keys': [dotted paths], 'override_count',
        'override_deleted': [dotted paths]}``. override: the cell pack's own copy of a file a robot pack has too,
        keyed (only the keys the cell changes, laid over the pack's file; overlay.py) or a whole file. The override
        keys are those whose values differ from the robot pack's (deleted: pack keys the cell removes); None for a
        file that is not YAML or does not load."""
        cell = self.root / 'robots' / robot / file
        found = [p for p in self.robot_packs(robot) if (p.path / file).exists()]
        if cell.exists():
            source, path = 'override' if found else 'cell', cell
        else:
            source, path = ('robot pack', found[0].path / file) if found else ('missing', cell)
        result = {'path': path, 'source': source, 'pack': found[0].label if found else None,
                  'save_path': self.robot_save_path(robot, file), 'override': None, 'override_keys': [],
                  'override_count': 0, 'override_deleted': []}
        if source == 'override' and file in overlay.MERGED_FILES:
            try:
                base, data = self._read_yaml(found[0].path / file) or {}, self._read_yaml(cell)
            except ConfigError:
                return {**result, 'override': 'file', 'override_keys': None, 'override_count': None,
                        'override_deleted': None}
            keyed = overlay.is_keyed(data)
            changed = overlay.delta(base, overlay.overlay(base, data, file) or {}, overlay.DEPTH.get(file))
            depth = overlay.DEPTH.get(file)
            paths = overlay.key_paths(changed, depth)
            result.update({'override': 'keys' if keyed else 'file', 'override_keys': paths,
                           'override_count': len(paths),
                           'override_deleted': overlay.key_paths(changed, depth, deleted=True)})
        return result

    def override_base(self, robot: str, file: str) -> Path | None:
        """The robot pack file a cell pack copy of robots/<robot>/<file> would be laid over (keyed override):
        the first robot pack's that has it; None for cell-specific files and files no robot pack has."""
        if file not in overlay.MERGED_FILES:
            return None
        return next((p.path / file for p in self.robot_packs(robot) if (p.path / file).is_file()), None)

    def robot_save_path(self, robot: str, file: str) -> Path:
        """Where an edit of a robot file is written. Cell-specific files (packs.CELL_ROBOT_FILES) and files the cell
        pack has: the cell pack. Robot pack files: the robot pack they come from, but a read-only one (the
        installed share, overwritten by every build) gets a cell pack override, which holds only the keys the cell
        changes (keyed_override)."""
        cell = self.root / 'robots' / robot / file
        if file in packs.CELL_ROBOT_FILES or file.split('/')[0] in packs.CELL_ROBOT_FOLDERS or cell.exists():
            return cell
        robot_packs = self.robot_packs(robot)
        found = [p for p in robot_packs if (p.path / file).exists()]
        if found:
            return found[0].path / file if found[0].location.writable else cell
        writable = [p for p in robot_packs if p.location.writable]
        return writable[0].path / file if writable else cell

    def robot_search(self, robot: str) -> str:
        """The places a robot's files are looked for, for error messages."""
        return ', '.join([f'cell pack {self.root / "robots" / robot}'] + [p.label for p in self.robot_packs(robot)])

    @staticmethod
    def _robot_relative(relative: str) -> tuple[str, str] | None:
        """robots/<robot>/<file> -> (robot, file); None for any other path."""
        parts = PurePosixPath(relative).parts
        return (parts[1], '/'.join(parts[2:])) if len(parts) > 2 and parts[0] == 'robots' else None

    def display_path(self, path: Path | str) -> str:
        """A file for the operator: relative to the cell pack, or '<robot> robot pack: <file>'."""
        path = Path(path)
        if path.is_relative_to(self.root):
            return str(path.relative_to(self.root))
        for robot in self.robot_names():
            for pack in self.robot_packs(robot):
                if path.is_relative_to(pack.path):
                    return f'{pack.name} robot pack: {path.relative_to(pack.path)}'
        return str(path)

    # ---- catalogue, schema export, validation (catalogue.py, check.py) ---------
    def files(self, hashes: bool = True) -> list[dict]:
        """Every config file of the cell pack and its robot packs (catalogue.files)."""
        from . import catalogue
        return catalogue.files(self, hashes)

    def file_kinds(self) -> list[dict]:
        """The kinds of file a pack can hold, also those not present (catalogue.file_kinds)."""
        from . import catalogue
        return catalogue.file_kinds()

    def json_schema(self, kind_or_path: str, pack: str = 'cell') -> dict:
        """JSON Schema (2020-12) of a kind or of a file path, for a form editor (catalogue.json_schema)."""
        from . import catalogue
        return catalogue.json_schema(kind_or_path, pack)

    def enum_values(self, name: str, robot: str | None = None) -> list[str]:
        """The values of a dynamic enum (``x-taskweave-enum`` in an exported schema) in this pack."""
        from . import catalogue
        return catalogue.enum_values(self, name, robot)

    def file_hash(self, path: str, pack: str = 'cell') -> str | None:
        """sha256 of a file's bytes (None if absent), for an edit that must not overwrite a newer one."""
        from . import catalogue
        return catalogue.file_hash(self, path, pack)

    def validate_file(self, path: str, data_or_text=None, pack: str = 'cell') -> list[dict]:
        """Problems of one file (check.validate_file); the file on disk when no data or text is given."""
        from . import check
        return check.validate_file(self, path, data_or_text, pack)

    def check_pack(self) -> list[dict]:
        """Problems of every file of the pack (check.check_pack)."""
        from . import check
        return check.check_pack(self)

    def dry_run_pack(self, path: str, data_or_text, pack: str = 'cell') -> list[dict]:
        """check_pack with ``data_or_text`` in place of the file, nothing written (check.dry_run_pack)."""
        from . import check
        return check.dry_run_pack(self, path, data_or_text, pack)

    # ---- writing and history (save.py) -------------------------------------------
    def save(self, path: str, content, pack: str = 'cell', base: str | None = None, message: str | None = None,
             author: str | None = None) -> dict:
        """Validate and write a file (text as given, or data merged keeping comments), commit it if the pack is in a
        git work tree. ``base``: the hash read; SaveConflict if the file changed, SaveError on errors (save.py)."""
        from . import save
        return save.save(self, path, content, pack, base, message, author)

    def delete(self, path: str, pack: str = 'cell', base: str | None = None, message: str | None = None,
               author: str | None = None) -> dict:
        """Remove a file (a robot file: its cell override); same lock, conflict and git handling as save."""
        from . import save
        return save.delete(self, path, pack, base, message, author)

    def create(self, kind: str, name: str = '', robot: str | None = None, content=None, message: str | None = None,
               author: str | None = None) -> dict:
        """A new file of a kind (catalogue.new_file), saved without a base."""
        from . import save
        return save.create(self, kind, name, robot, content, message, author)

    def save_mission(self, name: str, content, base: str | None = None, message: str | None = None,
                     author: str | None = None) -> dict:
        """Save missions/<name>.yaml, validated as a mission."""
        from . import save
        return save.save_mission(self, name, content, base, message, author)

    def set_robot(self, robot: str, base: str | None = None, message: str | None = None,
                  author: str | None = None) -> dict:
        """The robot in system.yaml, through save (reload: launch)."""
        from . import save
        return save.set_robot(self, robot, base, message, author)

    def history(self, path: str, pack: str = 'cell', limit: int = 50) -> list[dict]:
        """Commits of a file, newest first: ``{rev, time, author, message}``; [] without a repo."""
        from . import save
        return save.history(self, path, pack, limit)

    def show(self, path: str, rev: str, pack: str = 'cell') -> str:
        """A file's text at a revision."""
        from . import save
        return save.show(self, path, rev, pack)

    def restore(self, path: str, rev: str, pack: str = 'cell', base: str | None = None,
                author: str | None = None) -> dict:
        """A save of a file's text at a revision (conflicts like save)."""
        from . import save
        return save.restore(self, path, rev, pack, base, author)

    # ---- missions, robots, skills, operator ------------------------------
    def mission_names(self) -> list[str]:
        return sorted(p.stem for p in (self.root / 'missions').glob('*.yaml'))

    def mission(self, name: str) -> dict:
        return self._load(f'missions/{name}.yaml', schemas.MISSION)

    def expanded_mission(self, name: str) -> dict:
        """The mission with its include steps replaced by the included missions' steps (plan.expand_includes);
        a bad include is a ConfigError of this mission's file."""
        from ..plan import IncludeError, expand_includes

        mission = self.mission(name)
        try:
            return expand_includes(mission, self.mission)
        except IncludeError as error:
            raise ConfigError(self.root / f'missions/{name}.yaml', str(error)) from error

    def robot(self, name: str) -> RobotProfile:
        data = self._load(f'robots/{name}/profile.yaml', schemas.ROBOT_PROFILE)
        if data['name'] != name:
            raise ConfigError(self.robot_file(name, 'profile.yaml'), f'name is {data["name"]!r}, expected {name!r}',
                              'name')
        return RobotProfile(name, data)

    def skill_config(self, skill: str) -> dict:
        """The whole skills/<skill>.yaml (implementations and shared parameters); {} if absent."""
        return dict(self._load(skill_file(skill), schemas.SKILL_PARAMS, required=False, default={}))

    def skill_params(self, skill: str, implementation: dict | None = None) -> dict:
        """Parameters for a skill: shared keys of skills/<skill>.yaml, overridden by the implementation's params."""
        params = {k: v for k, v in self.skill_config(skill).items() if k != 'implementations'}
        params.update((implementation or {}).get('params', {}))
        return params

    def embodiment(self, robot: str) -> dict:
        """robots/<robot>/embodiment.yaml: what the robot is and can do."""
        data = self._load(f'robots/{robot}/embodiment.yaml', schemas.EMBODIMENT)
        if data['name'] != robot:
            raise ConfigError(self.robot_file(robot, 'embodiment.yaml'),
                              f'name is {data["name"]!r}, expected {robot!r}', 'name')
        return data

    def select_implementation(self, skill: str, robot: str) -> tuple[dict | None, list[str]]:
        """First implementation of ``skill`` whose ``requires`` fits ``robot``, and why the others don't.

        Returns (implementation or None, reasons). With no implementations configured, returns (None, [reason]).
        """
        from ..embodiment import match

        implementations = self.skill_config(skill).get('implementations', [])
        if not implementations:
            return None, [f'no implementations configured in {skill_file(skill)}']
        embodiment = self.embodiment(robot)
        reasons = []
        for implementation in implementations:
            mismatch = match(implementation.get('requires'), embodiment)
            if not mismatch:
                return implementation, reasons
            reasons.append(f'{implementation["name"]}: {"; ".join(mismatch)}')
        return None, reasons

    def operator(self) -> dict:
        return self._load('operator.yaml', schemas.OPERATOR, required=False, default={})

    # ---- points ------------------------------------------------------------
    # The readers below (locations ... base_target) are the deprecated names of the old point model: thin readers over
    # ``target()`` (target_store.legacy_*), until the HMI has moved to targets (P7).
    def targets_format(self) -> bool:
        """Always True: the store reads cell packs in the targets layout only (older ones: taskweave config migrate).
        Kept for the HMI's ``targets.active(store)``."""
        return True

    def locations(self) -> dict[str, Pose2D]:
        return ts.legacy_locations(self)

    def object_instances(self, state: bool = True) -> dict[str, ObjectInstance]:
        """Placed objects: cell/objects.yaml, with the run-time object state on top (``state``: items taken off
        stacks, objects taken away or put down). A perception system can provide the same later."""
        data = dict(self._load('cell/objects.yaml', schemas.OBJECT_INSTANCES, required=False, default={}))
        for name, change in (self.object_state() if state else {}).items():
            if change.get('gone'):
                data.pop(name, None)
            elif 'object' in change:
                data[name] = change
            elif name in data and 'count' in change:
                data[name] = {**data[name],
                              'stack': {'pitch': 0.0, **data[name].get('stack', {}), 'count': change['count']}}
        return {name: ObjectInstance(name, entry['object'], Pose2D.from_dict(entry), float(entry.get('z', 0.0)),
                                     int(entry.get('stack', {}).get('count', 1)),
                                     float(entry.get('stack', {}).get('pitch', 0.0)))
                for name, entry in data.items()}

    # ---- run-time object state ---------------------------------------------
    def state_path(self) -> Path:
        return Path(os.environ.get(STATE_DIR_ENV) or self.root.parent / 'state') / 'objects.yaml'

    def object_state(self) -> dict:
        """<state dir>/objects.yaml: run-time changes to the placed objects ({} if none)."""
        return self._load(str(self.state_path()), schemas.OBJECT_STATE, required=False, default={})

    def _change_object_state(self, edit) -> None:
        state = {k: dict(v) for k, v in self.object_state().items()}
        edit(state)
        errors = list(Draft202012Validator(schemas.OBJECT_STATE).iter_errors(state))
        if errors:
            raise ValueError(f'object state: {errors[0].message}')
        path = self.state_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text('# run-time object state over cell/objects.yaml (taskweave); cleared on a sim reset\n'
                             + yaml.safe_dump(state, sort_keys=False, default_flow_style=None), encoding='utf-8')
        temporary.replace(path)
        with self._lock:
            self._cache.pop(path, None)

    def take_object(self, name: str) -> str:
        """One item of ``name`` was taken: a stack has one less (an empty stack stays, count 0), a single object is
        gone; an object of a ``replenished`` type stays (the next one takes its place). Returns what happened, for a
        log."""
        instance = self.object_instances().get(name)
        if instance is None:
            raise KeyError(f'no placed object {name!r}')
        if self.object(instance.object).get('replenished'):
            return f'{name} stays (the next {instance.object} takes its place)'
        configured = name in self._load('cell/objects.yaml', schemas.OBJECT_INSTANCES, required=False, default={})

        def edit(state):
            if instance.stack_count > 1 or (configured and 'stack' in self._load(
                    'cell/objects.yaml', schemas.OBJECT_INSTANCES, required=False, default={})[name]):
                state[name] = {**({} if 'object' in state.get(name, {}) else state.get(name, {})),
                               'count': max(instance.stack_count - 1, 0)}
            elif configured:
                state[name] = {'gone': True}
            else:
                state.pop(name, None)

        self._change_object_state(edit)
        left = self.object_instances().get(name)
        return f'took one off {name}, {left.stack_count} left' if left else f'took {name}'

    def put_object(self, name: str, obj: str, x: float, y: float, z: float, yaw: float) -> None:
        """An object of type ``obj`` was put down at x, y, z, yaw (cell frame) and is now the placed object ``name``."""
        self._check_name(name)
        self.object(obj)

        def edit(state):
            state[name] = {'object': obj, 'frame': 'cell', 'x': round(x, 4), 'y': round(y, 4), 'z': round(z, 4),
                           'yaw': round(yaw, 4)}

        self._change_object_state(edit)

    def clear_object_state(self) -> None:
        """Back to cell/objects.yaml as configured (e.g. after a simulator reset)."""
        path = self.state_path()
        path.unlink(missing_ok=True)
        with self._lock:
            self._cache.pop(path, None)

    def stance_offsets(self, robot: str) -> dict:
        """robots/<robot>/stances.yaml as stored: ``{'objects': {type: {variant: pose}}, 'instances': {...}}``."""
        return ts.legacy_stance_offsets(self, robot)

    def stance_variants(self, robot: str, instance: ObjectInstance) -> dict[str, Pose2D]:
        """The robot's stance offsets for one object instance, in the object's frame, by variant name: the object
        type's variants, overridden by the instance's own. The first one is the default."""
        return ts.legacy_stance_variants(self, robot, instance)

    def handling_instance(self, instance: ObjectInstance, robot: str, near: Pose2D | None) -> ObjectInstance:
        """The instance as the robot handles it. An object type with ``yaw_symmetry_deg`` is the same when turned
        by that angle: of its equivalent turns, the one whose default stance is nearest to ``near`` (the robot's
        base pose in the cell), so the robot works from its own side, e.g. a frame lying the other way round is
        approached from the aisle and grasped without crossing the arms. Without ``near``, a symmetry or a stance
        of the robot: the instance as placed."""
        step = None if near is None else self._load(f'objects/{instance.object}.yaml', schemas.OBJECT, required=False,
                                                    default={}).get('yaw_symmetry_deg')
        offset = ts.handled_offset(self, robot, instance) if step else None
        if offset is None:
            return instance
        turns = [replace(instance, pose=Pose2D(instance.pose.x, instance.pose.y,
                                               wrap_angle(instance.pose.yaw + math.radians(k * step)),
                                               instance.pose.frame))
                 for k in range(round(360 / step))]

        def distance(turn: ObjectInstance) -> float:
            stance = turn.pose.compose(offset)
            return math.hypot(stance.x - near.x, stance.y - near.y)

        return min(turns, key=distance)

    def stances(self, robot: str) -> dict[str, Pose2D]:
        """Every stance of the robot in the cell frame: ``<instance>`` (default variant), ``<instance>/<variant>``."""
        result = {}
        for instance in self.object_instances().values():
            for index, (variant, offset) in enumerate(self.stance_variants(robot, instance).items()):
                pose = instance.pose.compose(offset)
                if index == 0:
                    result[instance.name] = pose
                result[f'{instance.name}{VARIANT_SEPARATOR}{variant}'] = pose
        return result

    def stance(self, reference: str, robot: str, near: Pose2D | None = None) -> Pose2D:
        """Resolve ``<instance>[/<variant>]`` to a base pose in the cell frame: object pose * robot offset; the
        object as handled from ``near`` (the robot's pose, see handling_instance)."""
        name, _, variant = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances:
            raise KeyError(f'unknown stance {reference!r}: no object {name!r} in cell/objects.yaml')
        instance = self.handling_instance(instances[name], robot, near)
        variants = self.stance_variants(robot, instance)
        if not variants:
            raise KeyError(f'unknown stance {reference!r}: robot {robot!r} has no stance for {instance.object!r} '
                           f'objects (robots/{robot}/stances.yaml)')
        variant = variant or next(iter(variants))
        if variant not in variants:
            raise KeyError(f'unknown stance {reference!r}: robot {robot!r} has no {variant!r} stance for '
                           f'{instance.object!r} (has {", ".join(variants)})')
        return instance.pose.compose(variants[variant])

    def joint_poses(self, robot: str) -> dict[str, dict]:
        """robots/<robot>/poses.yaml as stored: name -> joint map or tool pose (robot-specific by nature)."""
        return ts.legacy_joint_poses(self, robot)

    def named_pose(self, robot: str, name: str) -> dict:
        """A named pose as ``{'joints': {joint: value}, 'tools': {tool: {tcp, approach, closing}}, 'elbow': side}``.
        Joints are set directly; tool targets (TCP in the base frame) are solved by IK when the pose is used, with the
        pose's elbow side if it has one (None: the profile's). A joint-space pose has no tools."""
        return ts.legacy_named_pose(self, robot, name)

    def object(self, name: str) -> dict:
        return self._load(f'objects/{name}.yaml', schemas.OBJECT)

    def point_names(self, robot: str) -> dict[str, list[str]]:
        """Names a plan can refer to, by kind (for planners and the robot card)."""
        return ts.legacy_point_names(self, robot)

    def held_pose(self, robot: str, obj: str, name: str) -> dict:
        """robots/<robot>/held_poses.yaml <obj>.<name>: ``{'joints': {...}, 'path': [Pose3D, ...]}``."""
        return ts.legacy_held_pose(self, robot, obj, name)

    def held_poses(self, robot: str) -> dict:
        """robots/<robot>/held_poses.yaml as written: {object type: {name: {joints, path: [{xyz, rpy}]}}}."""
        return ts.legacy_held_poses(self, robot)

    def robot_object_points(self, robot: str) -> dict:
        """robots/<robot>/object_points.yaml: the robot's settings per object point ({object: {point: {elbow}}})."""
        return ts.legacy_robot_object_points(self, robot)

    def skill_overrides(self, robot: str) -> dict:
        """robots/<robot>/skill_overrides.yaml (robot pack, optional): ``{skill: {motion, speed, accel}}``, this
        robot's own values on top of skills/<skill>.yaml (motion.resolve); {} if the robot has none."""
        return self._load(f'robots/{robot}/skill_overrides.yaml', schemas.SKILL_OVERRIDES, required=False, default={})

    def object_point(self, obj: str, point: str, robot: str | None = None) -> dict:
        """A named point on an object: tool poses (canonical TCP, object-relative) plus its metadata.

        ``{'left_tool': Pose3D, ..., 'approach': {...} | None, 'tool_type': str | None, 'grip_width_m': float | None}``
        With ``robot``, that robot's settings for the point (object_points.yaml, e.g. ``elbow``) come on top.
        """
        found = ts.resolve_type(self, robot, obj, point) if point in ts.type_target_kinds(self, robot, obj) else None
        if found is None or found.kind != 'hand':
            raise KeyError(f'object {obj!r} has no point {point!r}')
        return found.to_point()

    def missing_instance(self, name: str, unknown: str) -> TargetError:
        """The error for an instance that cannot be used: one that is in cell/objects.yaml but taken in the object
        state (gone, or a stack at count 0) says so; any other is ``unknown`` plus the instances that exist."""
        current = self.object_instances()
        if name in self.object_instances(state=False) and (name not in current or current[name].stack_count == 0):
            return TargetError(f'{name!r} was taken in an earlier step or run (object state: state/objects.yaml); '
                               'reset the sim (Stop + Play) or run `taskweave state clear` to put it back')
        return TargetError(f'{unknown}, instances in cell/objects.yaml: {", ".join(sorted(current)) or "none"}')

    def instance_point(self, reference: str, robot: str | None = None,
                       near: Pose2D | None = None) -> tuple[ObjectInstance, dict]:
        """Resolve ``<instance>/<point>``: a placed object (cell/objects.yaml) and a point of its type, as
        object_point() returns it (tool poses relative to the object). With ``robot`` and ``near`` (its base pose),
        the instance as that robot handles it (handling_instance)."""
        name, _, point = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances or not point:
            raise self.missing_instance(name, f'unknown object point {reference!r}: expected <instance>/<point>')
        found = ts.resolve(self, reference, robot, near)
        if found.kind != 'hand':
            raise KeyError(f'object {found.type!r} has no point {point!r}')
        return found.instance, found.to_point()

    def object_contact(self, obj: str, contact: str) -> Pose3D:
        """A contact frame of an object type (an ``object`` target of objects/<object>.yaml), relative to the object."""
        found = ts.resolve_type(self, None, obj, contact) if contact in ts.type_target_kinds(self, None, obj) else None
        if found is None or found.kind != 'object':
            known = sorted(n for n, m in ts.type_target_kinds(self, None, obj).items() if m == 'object')
            raise KeyError(f'object {obj!r} has no contact {contact!r}'
                           + (f' (it has {", ".join(known)})' if known else ''))
        return found.pose

    def instance_contact(self, reference: str) -> tuple[ObjectInstance, Pose3D]:
        """Resolve ``<instance>/<contact>``: a placed object and a contact of its type."""
        name, _, contact = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances or not contact:
            raise self.missing_instance(name, f'unknown object contact {reference!r}: expected <instance>/<contact>')
        return instances[name], self.object_contact(instances[name].object, contact)

    def base_target(self, reference: str, robot: str, near: Pose2D | None = None) -> Pose2D:
        """Resolve a base target: ``stance:<instance>[/<variant>]`` (robot-specific, relative to an object, handled
        from ``near``: the robot's current pose), ``location:<name>`` or a bare location name."""
        if reference.startswith(STANCE_PREFIX):
            return self.stance(reference[len(STANCE_PREFIX):], robot, near)
        name = reference[len(LOCATION_PREFIX):] if reference.startswith(LOCATION_PREFIX) else reference
        locations = self.locations()
        if name not in locations:
            raise KeyError(f'unknown location {name!r}')
        return locations[name]

    # ---- targets (cell schema 4; targets.py, target_store.py, target_edit.py) ------------------------------------
    def target(self, reference: str, robot: str, near: Pose2D | None = None, *, kinds=None,
               context: str = '') -> Target:
        """Resolve a reference to a Target: ``<name>`` (a cell, base-frame or joint target) or ``<instance>/<name>`` (an
        object-frame target of a placed object: the per-key merge of the type's and the robot's layers, a stack
        as its top item, a turned object as the robot handles it from ``near``, its base pose in the cell).
        ``kinds``: the ``moves`` the caller can use (one or several); a target of another kind is refused with
        ``context`` as the reason. Raises TargetError (a KeyError): one line, naming the file to look at."""
        return ts.resolve(self, reference, robot, near, kinds, context)

    def type_target(self, type: str, name: str, robot: str | None = None) -> Target:
        """A target of an object type with no placed object, in the object's own frame: the type's targets, with the
        robot's settings on top when ``robot`` is given (the contact a held object brings to a target). TargetError
        (a KeyError) if there is none."""
        return ts.resolve_type(self, robot, type, name)

    def targets(self, robot: str) -> dict[str, Target]:
        """Every resolvable target by reference: the cell's and the robot's, and ``<instance>/<name>`` for the targets
        of each placed object's type."""
        return ts.all_targets(self, robot)

    def target_names(self, robot: str) -> dict[str, list[str]]:
        """The names a plan can use by kind: ``{'base targets', 'hand targets', 'object targets', 'joint targets',
        'placed objects'}``, object-frame ones as ``<instance>/<name>`` for the placed instances."""
        return ts.target_names(self, robot)

    def object_target_names(self, robot: str | None = None) -> dict[str, list[str]]:
        """``{object type: [target names]}`` of the object-frame targets, with the robot's own when given."""
        return ts.object_target_names(self, robot)

    def save_target(self, reference: str, data: dict, robot: str, *, instance_override: bool = False,
                    rename: str | None = None, type: str | None = None, note: str | None = None) -> list[Path]:
        """Create or update a target and write it where it belongs, comments kept (target_edit.save_target); returns the
        files written. ``instance_override``: for ``<instance>/<name>``, write the instance's override. ``type``: the
        target ``reference`` (a bare name) of an object type, for a type nothing is placed for. ``note``: the entry's
        comment (a new entry gets ``saved <date>``). Refusals are TargetWriteError (a ValueError)."""
        from . import target_edit

        return target_edit.save_target(self, reference, data, robot, instance_override=instance_override,
                                       rename=rename, type=type, note=note)

    def delete_target(self, reference: str, robot: str, *, instance_override: bool = False,
                      type: str | None = None, cascade: bool = False) -> list[Path]:
        """Remove a target from the files that define it (an instance's override only with ``instance_override``;
        ``type`` as for save_target). A type target that instance overrides or other robots' settings still refer to
        is refused, listing them, unless ``cascade=True`` (for callers: the refusal text does not name it): they are
        removed too."""
        from . import target_edit

        return target_edit.delete_target(self, reference, robot, instance_override=instance_override, type=type,
                                         cascade=cascade)

    def rename_target(self, reference: str, new_name: str, robot: str, *, type: str | None = None) -> list[Path]:
        """Rename a target in every file that holds it, position and comments kept (a stance's ``handled_from``
        follows); a name that exists only as one instance's override is renamed in that instance only; missions that
        use the name are not changed."""
        from . import target_edit

        return target_edit.rename_target(self, reference, new_name, robot, type=type)

    # ---- teaching -------------------------------------------------------------
    def save_object_instance(self, name: str, obj: str, x: float, y: float, z: float, yaw: float) -> Path:
        """Place (or move) the object instance ``name`` of type ``obj`` at x, y, z, yaw in the cell frame
        (cell/objects.yaml); other entries and comments are kept, an existing entry keeps its other keys."""
        from ruamel.yaml.comments import CommentedMap

        self._check_name(name)
        self.object(obj)  # the object type must exist

        def edit(document):
            entry = document.get(name) if isinstance(document.get(name), dict) else CommentedMap()
            entry.update({'object': obj, 'frame': entry.get('frame', 'cell'), 'x': round(x, 4), 'y': round(y, 4),
                          'z': round(z, 4), 'yaw': round(yaw, 4)})
            entry.pop('stack', None)  # a single item now
            entry.fa.set_flow_style()
            document[name] = entry

        return self._edit_yaml('cell/objects.yaml', edit)

    def save_ik_choice(self, robot: str, choice: dict) -> Path:
        """Update the profile's ik.choose (elbow, limit_margin_deg, weights); other keys and comments are kept."""
        from ruamel.yaml.comments import CommentedMap

        unknown = set(choice) - {'elbow', 'limit_margin_deg', 'weights'}
        if unknown:
            raise ValueError(f'unknown IK choice settings: {", ".join(sorted(unknown))}')

        def edit(document):
            ik = document.get('ik')
            if not isinstance(ik, dict):
                raise ValueError(f'robot {robot!r} has no ik section')
            choose = ik.get('choose') if isinstance(ik.get('choose'), dict) else CommentedMap()
            for key, value in choice.items():
                if key == 'weights':
                    weights = choose.get('weights') if isinstance(choose.get('weights'), dict) else CommentedMap()
                    weights.update({k: round(float(v), 3) for k, v in value.items()})
                    weights.fa.set_flow_style()
                    choose['weights'] = weights
                elif value is None:
                    choose.pop(key, None)
                else:
                    choose[key] = round(float(value), 2) if key == 'limit_margin_deg' else value
            ik['choose'] = choose

        return self._edit_yaml(f'robots/{robot}/profile.yaml', edit)

    # ---- editing points (teach mode, HMI) ---------------------------------------
    @staticmethod
    def _rename_key(node, name: str, rename: str, label: str, relative: str) -> None:
        """Rename a mapping key in place, keeping its position and its end-of-line comment."""
        if rename in node:
            raise ValueError(f'{label!r} already exists in {relative}')
        position, value, comment = list(node).index(name), node[name], node.ca.items.pop(name, None)
        del node[name]
        node.insert(position, rename, value)
        if comment is not None:
            node.ca.items[rename] = comment

    def navigation(self, robot: str) -> dict:
        """robots/<robot>/navigation.yaml: room kept around the robot while driving; {} if absent."""
        return self._load(f'robots/{robot}/navigation.yaml', schemas.NAVIGATION, required=False, default={})

    def save_navigation(self, robot: str, settings: dict) -> Path:
        """Write the navigation settings, keeping the file's comments; checked against the schema first."""
        from ruamel.yaml.comments import CommentedMap

        def clean(value):
            return value if isinstance(value, bool) else round(float(value), 4)

        def edit(document):
            for key, value in settings.items():
                if isinstance(value, dict):
                    entry = document.get(key) if isinstance(document.get(key), dict) else CommentedMap()
                    entry.update({k: clean(v) for k, v in value.items()})
                    entry.fa.set_flow_style()
                    document[key] = entry
                else:
                    document[key] = clean(value)

        return self._edit_yaml(f'robots/{robot}/navigation.yaml', edit)

    def _edit_yaml(self, relative: str, edit) -> Path:
        """Apply ``edit(document)`` to a point file with comments and order kept; write only if the result is valid.
        A cell pack copy of a robot pack file is edited as the whole file it stands for and written as a keyed
        override (keyed_override)."""
        from ruamel.yaml.comments import CommentedMap

        parts = PurePosixPath(relative).parts
        schema = (POINT_FILE_SCHEMAS.get(relative) or POINT_FILE_SCHEMAS.get(Path(relative).name)
                  or (schemas.OBJECT if Path(relative).parent == Path('objects') else None)
                  or (schemas.ROBOT_OBJECT_FILE if len(parts) == 4 and parts[0] == 'robots' and parts[2] == 'objects'
                      else None))
        if schema is None:
            raise ValueError(f'{relative} is not a point file')
        robot_file = self._robot_relative(relative)
        path = self.robot_save_path(*robot_file) if robot_file else self.root / relative
        handler = _yaml_handler()
        if robot_file and self._keyed(*robot_file, path):
            document = self.keyed_override(*robot_file, edit, schema, relative)[0]
        else:
            start = path if path.is_file() or not robot_file else self.robot_file(*robot_file)  # new override: a copy
            document = handler.load(start.read_text(encoding='utf-8')) if start.is_file() else None
            if document is None:
                document = CommentedMap()
            edit(document)
            _check(schema, _plain(document), relative)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as stream:
            handler.dump(document, stream)
        with self._lock:
            self._cache.pop(path, None)
        return path

    def _keyed(self, robot: str, file: str, path: Path) -> bool:
        """An edit of robots/<robot>/<file> written to ``path`` is a keyed override: the cell pack's copy of a robot
        pack file (or a keyed file whose robot pack file is gone)."""
        cell = self.root / 'robots' / robot / file
        if path != cell or file not in overlay.MERGED_FILES:
            return False
        return self.override_base(robot, file) is not None or (cell.is_file() and overlay.is_keyed(
            self._read_yaml(cell)))

    def keyed_override(self, robot: str, file: str, edit=None, schema: dict | None = None,
                       label: str = '') -> tuple[Any, dict]:
        """The cell pack's robots/<robot>/<file> as a keyed override document (ruamel, comments kept: the cell file's,
        the pack's for keys the cell changes) after ``edit(document)``, which gets the whole file the override stands
        for (the pack's file, the cell's keys laid over it; a whole-file copy as it is). Returns the document and that
        whole file as plain data, checked against ``schema`` (ValueError naming ``label``). Nothing is written."""
        from ruamel.yaml.comments import CommentedMap

        cell, base = self.root / 'robots' / robot / file, self.override_base(robot, file)
        depth, handler = overlay.DEPTH.get(file), _yaml_handler()
        base_document = (handler.load(base.read_text(encoding='utf-8')) if base else None) or CommentedMap()
        base_data = _plain(base_document)
        cell_document = handler.load(cell.read_text(encoding='utf-8')) if cell.is_file() else None
        header, origin = None, None     # origin: the whole-file copy's own keys (missing pack keys are not deletions)
        if overlay.is_keyed(cell_document):
            header = cell_document.ca.comment
            document = _overlay_document(base_document, cell_document, depth)
        else:
            document = cell_document if isinstance(cell_document, dict) else base_document
            origin = _plain(document) if document is cell_document else None
        if edit:
            edit(document)
        data = _plain(document)
        if schema is not None:
            _check(schema, data, label or f'robots/{robot}/{file}')
        _prune_document(document, base_data, depth, origin)
        document.insert(0, overlay.MARKER, overlay.KEYS)
        document.ca.comment = header
        if header is None:
            document.yaml_set_start_comment(OVERRIDE_HEADER.format(file=file))
        return document, data

    @staticmethod
    def _check_name(name: str) -> None:
        if not Draft202012Validator(schemas.NAME).is_valid(name):
            raise ValueError(f'invalid point name {name!r}: use letters, digits, "_" and "-", starting with a letter')


OVERRIDE_HEADER = ("cell override of the robot pack's {file}: only the keys this cell changes, laid over the pack's "
                   'file (~delete removes a pack key; see taskweave config/overlay.py)')


def _stamp(path: Path) -> tuple[int, int]:
    """Change marker of a file for the cache: mtime and size (a coarse clock misses quick rewrites by mtime alone)."""
    info = path.stat()
    return info.st_mtime_ns, info.st_size


def _yaml_handler():
    from ruamel.yaml import YAML  # only needed for writing

    handler = YAML()
    handler.preserve_quotes = True
    handler.width = 4096  # one line per entry, as written by hand and by tools (joint poses are long)
    return handler


def _plain(document) -> Any:
    return json.loads(json.dumps(document))


def _check(schema: dict, data: Any, label: str) -> None:
    errors = sorted(Draft202012Validator(schema).iter_errors(data), key=lambda e: list(e.absolute_path))
    if errors:
        key = '.'.join(str(p) for p in errors[0].absolute_path)
        raise ValueError(f'{label}{f" [{key}]" if key else ""}: {errors[0].message}; nothing written')


def _pop(node, key) -> None:
    node.pop(key, None)
    node.ca.items.pop(key, None)


def _strip_document(value):
    """A ruamel value without overlay.DELETE entries (overlay.strip)."""
    if isinstance(value, dict):
        for key in [k for k, v in value.items() if v == overlay.DELETE]:
            _pop(value, key)
        for item in value.values():
            _strip_document(item)
    return value


def _overlay_document(base, over, depth: int | None):
    """overlay.merge on ruamel documents: ``over``'s keys laid into ``base`` in place, with their comments."""
    for key, value in over.items():
        if key == overlay.MARKER:
            continue
        if value == overlay.DELETE:
            _pop(base, key)
        elif key in base and depth != 1 and isinstance(base[key], dict) and isinstance(value, dict):
            _overlay_document(base[key], value, None if depth is None else depth - 1)
        else:
            base[key] = _strip_document(value)
            if key in over.ca.items:
                base.ca.items[key] = over.ca.items[key]
    return base


def _prune_document(node, base: dict, depth: int | None, origin: dict | None = None) -> None:
    """overlay.delta on a ruamel document, in place: keys equal to ``base`` go, base keys it lacks become DELETE.
    ``origin``: the plain whole-file copy it was loaded from; a base key it never had is not a deletion (the copy
    predates the key), only one the edit removed is. None: a keyed override, every missing key is a deletion."""
    missing = [key for key in base if key not in node and (origin is None or key in origin)]
    for key in list(node):
        if key not in base:
            continue
        if overlay.same(_plain(node[key]), base[key]):
            _pop(node, key)
        elif depth != 1 and isinstance(node[key], dict) and isinstance(base[key], dict):
            inner = origin.get(key) if origin is not None else None
            _prune_document(node[key], base[key], None if depth is None else depth - 1,
                            (inner if isinstance(inner, dict) else {}) if origin is not None else None)
            if not node[key]:   # everything in it equals the pack's (overlay.delta leaves such a mapping out)
                _pop(node, key)
    for key in missing:
        node[key] = overlay.DELETE
