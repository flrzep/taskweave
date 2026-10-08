"""Loading, validating and querying a taskweave config directory.

Layout (all files YAML; files marked optional may be absent):

    missions/<name>.yaml           required per mission
    robots/<model>/profile.yaml    required per robot
    robots/<model>/stances.yaml    optional, robot-specific base poses
    robots/<model>/poses.yaml      optional, named joint-space poses (interpreted by robot-specific skills)
    cell/locations.yaml            optional, named base poses in the cell frame
    objects/<name>.yaml            optional per object, taught tool poses
    skills/<skill>.yaml            optional per skill, parameters
    operator.yaml                  optional, operator inputs (e.g. joystick buttons)
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator

from . import schemas
from .poses import Pose2D, Pose3D

STANCE_PREFIX = 'stance:'
LOCATION_PREFIX = 'location:'


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


class ConfigStore:
    """Read-mostly view of one config directory. Files are validated on first use and cached."""

    def __init__(self, root: str | Path):
        self.root = Path(root).expanduser().resolve()
        if not self.root.is_dir():
            raise ConfigError(self.root, 'config directory does not exist')
        self._cache: dict[Path, Any] = {}
        self._lock = threading.Lock()

    # ---- generic loading -------------------------------------------------
    def _load(self, relative: str, schema: dict, required: bool = True, default: Any = None) -> Any:
        """Load and validate a file. Cached per modification time, so edits by other processes are picked up."""
        path = self.root / relative
        try:
            stamp = path.stat().st_mtime_ns
        except OSError:
            stamp = None
        with self._lock:
            cached = self._cache.get(path)
            if cached is not None and cached[0] == stamp:
                return cached[1]
        if stamp is None:
            if required:
                raise ConfigError(path, 'file not found')
            return default
        try:
            data = yaml.safe_load(path.read_text(encoding='utf-8'))
        except yaml.YAMLError as error:
            raise ConfigError(path, f'invalid YAML: {error}') from error
        if data is None:
            data = default if default is not None else {}
        errors = sorted(Draft202012Validator(schema).iter_errors(data), key=lambda e: list(e.absolute_path))
        if errors:
            first = errors[0]
            key = '.'.join(str(p) for p in first.absolute_path)
            more = f' (and {len(errors) - 1} more)' if len(errors) > 1 else ''
            raise ConfigError(path, first.message + more, key)
        with self._lock:
            self._cache[path] = (stamp, data)
        return data

    def reload(self) -> None:
        """Drop all cached files; the next access reads from disk again."""
        with self._lock:
            self._cache.clear()

    # ---- missions, robots, skills, operator ------------------------------
    def mission_names(self) -> list[str]:
        return sorted(p.stem for p in (self.root / 'missions').glob('*.yaml'))

    def mission(self, name: str) -> dict:
        return self._load(f'missions/{name}.yaml', schemas.MISSION)

    def robot(self, name: str) -> RobotProfile:
        data = self._load(f'robots/{name}/profile.yaml', schemas.ROBOT_PROFILE)
        if data['name'] != name:
            raise ConfigError(self.root / f'robots/{name}/profile.yaml', f'name is {data["name"]!r}, expected {name!r}',
                              'name')
        return RobotProfile(name, data)

    def skill_params(self, skill: str) -> dict:
        return dict(self._load(f'skills/{skill}.yaml', schemas.SKILL_PARAMS, required=False, default={}))

    def operator(self) -> dict:
        return self._load('operator.yaml', schemas.OPERATOR, required=False, default={})

    # ---- points ------------------------------------------------------------
    def locations(self) -> dict[str, Pose2D]:
        data = self._load('cell/locations.yaml', schemas.POINTS2D, required=False, default={})
        return {name: Pose2D.from_dict(pose) for name, pose in data.items()}

    def stances(self, robot: str) -> dict[str, Pose2D]:
        data = self._load(f'robots/{robot}/stances.yaml', schemas.POINTS2D, required=False, default={})
        return {name: Pose2D.from_dict(pose) for name, pose in data.items()}

    def object(self, name: str) -> dict:
        return self._load(f'objects/{name}.yaml', schemas.OBJECT)

    def object_point(self, obj: str, point: str) -> dict:
        """Tool poses of a named point on an object: ``{'left_tool': Pose3D, ..., 'approach': {...}}``."""
        points = self.object(obj).get('points', {})
        if point not in points:
            raise KeyError(f'object {obj!r} has no point {point!r}')
        entry = dict(points[point])
        result: dict[str, Any] = {'approach': entry.pop('approach', None)}
        result.update({tool: Pose3D.from_dict(pose, obj) for tool, pose in entry.items()})
        return result

    def base_target(self, reference: str, robot: str) -> Pose2D:
        """Resolve a base target: ``stance:<name>`` (robot-specific), ``location:<name>`` or a bare location name."""
        if reference.startswith(STANCE_PREFIX):
            name = reference[len(STANCE_PREFIX):]
            table, missing = self.stances(robot), f'unknown stance {name!r} for robot {robot!r}'
        else:
            name = reference[len(LOCATION_PREFIX):] if reference.startswith(LOCATION_PREFIX) else reference
            table, missing = self.locations(), f'unknown location {name!r}'
        if name not in table:
            raise KeyError(missing)
        return table[name]

    # ---- teaching -------------------------------------------------------------
    def save_base_pose(self, name: str, pose: Pose2D, kind: str = 'location', robot: str | None = None) -> Path:
        """Write a taught base pose into ``cell/locations.yaml`` or ``robots/<robot>/stances.yaml``.

        Existing comments and ordering are kept. Returns the file written.
        """
        from ruamel.yaml import YAML  # only needed for teaching
        from ruamel.yaml.comments import CommentedMap

        if kind == 'stance':
            if not robot:
                raise ValueError('a stance needs a robot')
            path = self.root / f'robots/{robot}/stances.yaml'
        elif kind == 'location':
            path = self.root / 'cell/locations.yaml'
        else:
            raise ValueError(f'unknown kind {kind!r}')
        if not Draft202012Validator(schemas.NAME).is_valid(name):
            raise ValueError(f'invalid point name {name!r}: use letters, digits, "_" and "-", starting with a letter')
        handler = YAML()
        handler.preserve_quotes = True
        data = handler.load(path.read_text(encoding='utf-8')) if path.is_file() else None
        if data is None:
            data = {}
        entry = CommentedMap(pose.to_dict())
        entry.fa.set_flow_style()  # one line per point, like hand-written entries
        data[name] = entry
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as stream:
            handler.dump(data, stream)
        with self._lock:
            self._cache.pop(path, None)
        return path
