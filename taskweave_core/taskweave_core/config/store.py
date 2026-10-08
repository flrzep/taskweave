"""Loading, validating and querying a taskweave config directory.

Layout (all files YAML; files marked optional may be absent):

    missions/<name>.yaml           required per mission
    robots/<model>/profile.yaml    required per robot: how the hardware is reached (topics, names)
    robots/<model>/embodiment.yaml required per robot: what the robot is and can do
    robots/<model>/stances.yaml    optional, robot-specific base poses relative to objects (per type, per instance)
    robots/<model>/poses.yaml      optional, named joint-space poses (used by move_to_pose)
    cell/locations.yaml            optional, named base poses in the cell frame
    cell/objects.yaml              optional, placed object instances: object type and pose in the cell frame
    objects/<name>.yaml            optional per object, taught tool poses
    skills/<skill>.yaml            per served skill: implementations and parameters
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
from .poses import ObjectInstance, Pose2D, Pose3D

STANCE_PREFIX = 'stance:'
LOCATION_PREFIX = 'location:'
VARIANT_SEPARATOR = '/'     # stance:<instance>/<variant>
POINT_METADATA = ('approach', 'tool_type', 'grip_width_m')  # keys of an object point that are not tool poses


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

    def skill_config(self, skill: str) -> dict:
        """The whole skills/<skill>.yaml (implementations and shared parameters); {} if absent."""
        return dict(self._load(f'skills/{skill}.yaml', schemas.SKILL_PARAMS, required=False, default={}))

    def skill_params(self, skill: str, implementation: dict | None = None) -> dict:
        """Parameters for a skill: shared keys of skills/<skill>.yaml, overridden by the implementation's params."""
        params = {k: v for k, v in self.skill_config(skill).items() if k != 'implementations'}
        params.update((implementation or {}).get('params', {}))
        return params

    def embodiment(self, robot: str) -> dict:
        """robots/<robot>/embodiment.yaml: what the robot is and can do."""
        data = self._load(f'robots/{robot}/embodiment.yaml', schemas.EMBODIMENT)
        if data['name'] != robot:
            raise ConfigError(self.root / f'robots/{robot}/embodiment.yaml',
                              f'name is {data["name"]!r}, expected {robot!r}', 'name')
        return data

    def select_implementation(self, skill: str, robot: str) -> tuple[dict | None, list[str]]:
        """First implementation of ``skill`` whose ``requires`` fits ``robot``, and why the others don't.

        Returns (implementation or None, reasons). With no implementations configured, returns (None, [reason]).
        """
        from ..embodiment import match

        implementations = self.skill_config(skill).get('implementations', [])
        if not implementations:
            return None, [f'no implementations configured in skills/{skill}.yaml']
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
    def locations(self) -> dict[str, Pose2D]:
        data = self._load('cell/locations.yaml', schemas.POINTS2D, required=False, default={})
        return {name: Pose2D.from_dict(pose) for name, pose in data.items()}

    def object_instances(self) -> dict[str, ObjectInstance]:
        """cell/objects.yaml: placed objects. From config for now; a perception system can provide the same."""
        data = self._load('cell/objects.yaml', schemas.OBJECT_INSTANCES, required=False, default={})
        return {name: ObjectInstance(name, entry['object'], Pose2D.from_dict(entry), float(entry.get('z', 0.0)))
                for name, entry in data.items()}

    def stance_variants(self, robot: str, instance: ObjectInstance) -> dict[str, Pose2D]:
        """The robot's stance offsets for one object instance, in the object's frame, by variant name: the object
        type's variants, overridden by the instance's own. The first one is the default."""
        data = self._load(f'robots/{robot}/stances.yaml', schemas.STANCES, required=False, default={})
        variants = dict(data.get('objects', {}).get(instance.object, {}))
        variants.update(data.get('instances', {}).get(instance.name, {}))
        return {name: Pose2D(float(v['x']), float(v['y']), float(v['yaw']), instance.object)
                for name, v in variants.items()}

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

    def stance_names(self, robot: str) -> list[str]:
        """Stance references for planners and pickers: the default variant without suffix, the others with."""
        names = []
        for instance in self.object_instances().values():
            for index, variant in enumerate(self.stance_variants(robot, instance)):
                names.append(STANCE_PREFIX + (instance.name if index == 0 else
                                              f'{instance.name}{VARIANT_SEPARATOR}{variant}'))
        return names

    def stance(self, reference: str, robot: str) -> Pose2D:
        """Resolve ``<instance>[/<variant>]`` to a base pose in the cell frame: object pose * robot offset."""
        name, _, variant = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances:
            raise KeyError(f'unknown stance {reference!r}: no object {name!r} in cell/objects.yaml')
        instance = instances[name]
        variants = self.stance_variants(robot, instance)
        if not variants:
            raise KeyError(f'unknown stance {reference!r}: robot {robot!r} has no stance for {instance.object!r} '
                           f'objects (robots/{robot}/stances.yaml)')
        variant = variant or next(iter(variants))
        if variant not in variants:
            raise KeyError(f'unknown stance {reference!r}: robot {robot!r} has no {variant!r} stance for '
                           f'{instance.object!r} (has {", ".join(variants)})')
        return instance.pose.compose(variants[variant])

    def joint_poses(self, robot: str) -> dict[str, dict[str, float]]:
        """robots/<robot>/poses.yaml: named joint-space poses (robot-specific by nature)."""
        return self._load(f'robots/{robot}/poses.yaml', schemas.JOINT_POSES, required=False, default={})

    def joint_pose(self, robot: str, name: str) -> dict[str, float]:
        poses = self.joint_poses(robot)
        if name not in poses:
            raise KeyError(f'unknown pose {name!r} for robot {robot!r}')
        return {joint: float(value) for joint, value in poses[name].items()}

    def object(self, name: str) -> dict:
        return self._load(f'objects/{name}.yaml', schemas.OBJECT)

    def point_names(self, robot: str) -> dict[str, list[str]]:
        """Names a plan can refer to, by kind (for planners and the robot card)."""
        objects = []
        for path in sorted((self.root / 'objects').glob('*.yaml')):
            objects += [f'{path.stem}.{name}' for name in self.object(path.stem).get('points', {})]
        return {'locations': sorted(self.locations()), 'stances': self.stance_names(robot),
                'joint poses': sorted(self.joint_poses(robot)), 'object points': objects}

    def object_point(self, obj: str, point: str) -> dict:
        """A named point on an object: tool poses (canonical TCP, object-relative) plus its metadata.

        ``{'left_tool': Pose3D, ..., 'approach': {...} | None, 'tool_type': str | None, 'grip_width_m': float | None}``
        """
        points = self.object(obj).get('points', {})
        if point not in points:
            raise KeyError(f'object {obj!r} has no point {point!r}')
        entry = dict(points[point])
        result: dict[str, Any] = {key: entry.pop(key, None) for key in POINT_METADATA}
        result.update({tool: Pose3D.from_dict(pose, obj) for tool, pose in entry.items()})
        return result

    def base_target(self, reference: str, robot: str) -> Pose2D:
        """Resolve a base target: ``stance:<instance>[/<variant>]`` (robot-specific, relative to an object),
        ``location:<name>`` or a bare location name."""
        if reference.startswith(STANCE_PREFIX):
            return self.stance(reference[len(STANCE_PREFIX):], robot)
        name = reference[len(LOCATION_PREFIX):] if reference.startswith(LOCATION_PREFIX) else reference
        locations = self.locations()
        if name not in locations:
            raise KeyError(f'unknown location {name!r}')
        return locations[name]

    # ---- teaching -------------------------------------------------------------
    def save_base_pose(self, name: str, pose: Pose2D, kind: str = 'location', robot: str | None = None) -> Path:
        """Write a taught base pose. Returns the file written; existing comments and ordering are kept.

        ``location``: ``name`` -> ``cell/locations.yaml``. ``stance``: ``name`` = ``<instance>[/<variant>]``; the
        pose is stored relative to that object, for its object type (all instances of the type follow).
        ``stance_instance``: the same, as an override for this instance only.
        """
        if kind in ('stance', 'stance_instance'):
            if not robot:
                raise ValueError('a stance needs a robot')
            return self.save_stance(name, pose, robot, per_instance=kind == 'stance_instance')[0]
        if kind != 'location':
            raise ValueError(f'unknown kind {kind!r}')
        self._check_name(name)
        return self._write_point(self.root / 'cell/locations.yaml', [name], pose)

    def save_stance(self, reference: str, pose: Pose2D, robot: str, per_instance: bool = False) -> tuple[Path, Pose2D]:
        """Store ``pose`` (cell frame) as the robot's stance at ``<instance>[/<variant>]``; returns file and offset."""
        name, _, variant = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances:
            raise ValueError(f'no object {name!r} in cell/objects.yaml')
        instance = instances[name]
        variant = variant or next(iter(self.stance_variants(robot, instance)), 'default')
        self._check_name(variant)
        offset = pose.relative_to(instance.pose, instance.object)
        keys = ['instances', instance.name, variant] if per_instance else ['objects', instance.object, variant]
        return self._write_point(self.root / f'robots/{robot}/stances.yaml', keys, offset), offset

    @staticmethod
    def _check_name(name: str) -> None:
        if not Draft202012Validator(schemas.NAME).is_valid(name):
            raise ValueError(f'invalid point name {name!r}: use letters, digits, "_" and "-", starting with a letter')

    def _write_point(self, path: Path, keys: list[str], pose: Pose2D) -> Path:
        """Set ``data[keys...] = pose`` in a YAML file, keeping comments and order."""
        from ruamel.yaml import YAML  # only needed for teaching
        from ruamel.yaml.comments import CommentedMap

        handler = YAML()
        handler.preserve_quotes = True
        data = handler.load(path.read_text(encoding='utf-8')) if path.is_file() else None
        node = data = data if data is not None else CommentedMap()
        for key in keys[:-1]:
            if not isinstance(node.get(key), dict):
                node[key] = CommentedMap()
            node = node[key]
        entry = CommentedMap(pose.to_dict())
        entry.fa.set_flow_style()  # one line per point, like hand-written entries
        node[keys[-1]] = entry
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as stream:
            handler.dump(data, stream)
        with self._lock:
            self._cache.pop(path, None)
        return path
