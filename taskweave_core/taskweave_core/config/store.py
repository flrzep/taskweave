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

import json
import math
import threading
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator

from . import schemas
from .poses import ObjectInstance, Pose2D, Pose3D, wrap_angle

STANCE_PREFIX = 'stance:'
LOCATION_PREFIX = 'location:'
VARIANT_SEPARATOR = '/'     # stance:<instance>/<variant>
POINT_METADATA = ('approach', 'tool_type', 'grip_width_m', 'approach_m', 'elbow')  # object point keys besides tool poses
TOOL_SUFFIX = '_tool'       # an object point's tool pose key: <tool>_tool
POINT_FILE_SCHEMAS = {  # files that teaching and the HMI may edit (a file name matches any robot folder)
    'cell/locations.yaml': schemas.POINTS2D, 'stances.yaml': schemas.STANCES, 'poses.yaml': schemas.JOINT_POSES,
    'navigation.yaml': schemas.NAVIGATION, 'profile.yaml': schemas.ROBOT_PROFILE,  # profile: ik.choose only
    'cell/objects.yaml': schemas.OBJECT_INSTANCES,  # where an object was put down (gripper place_as)
    'object_points.yaml': schemas.ROBOT_OBJECT_POINTS,  # a robot's settings for object points (elbow side)
    'held_poses.yaml': schemas.HELD_POSES,  # where a robot holds an object of a type (move_object pose)
}


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
        return {name: ObjectInstance(name, entry['object'], Pose2D.from_dict(entry), float(entry.get('z', 0.0)),
                                     int(entry.get('stack', {}).get('count', 1)),
                                     float(entry.get('stack', {}).get('pitch', 0.0)))
                for name, entry in data.items()}

    def stance_offsets(self, robot: str) -> dict:
        """robots/<robot>/stances.yaml as stored: ``{'objects': {type: {variant: pose}}, 'instances': {...}}``."""
        return self._load(f'robots/{robot}/stances.yaml', schemas.STANCES, required=False, default={})

    def stance_variants(self, robot: str, instance: ObjectInstance) -> dict[str, Pose2D]:
        """The robot's stance offsets for one object instance, in the object's frame, by variant name: the object
        type's variants, overridden by the instance's own. The first one is the default."""
        data = self.stance_offsets(robot)
        variants = dict(data.get('objects', {}).get(instance.object, {}))
        variants.update(data.get('instances', {}).get(instance.name, {}))
        return {name: Pose2D(float(v['x']), float(v['y']), float(v['yaw']), instance.object)
                for name, v in variants.items()}

    def handling_instance(self, instance: ObjectInstance, robot: str, near: Pose2D | None) -> ObjectInstance:
        """The instance as the robot handles it. An object type with ``yaw_symmetry_deg`` is the same when turned
        by that angle: of its equivalent turns, the one whose default stance is nearest to ``near`` (the robot's
        base pose in the cell), so the robot works from its own side, e.g. a frame lying the other way round is
        approached from the aisle and grasped without crossing the arms. Without ``near``, a symmetry or a stance
        of the robot: the instance as placed."""
        step = None if near is None else self._load(f'objects/{instance.object}.yaml', schemas.OBJECT, required=False,
                                                    default={}).get('yaw_symmetry_deg')
        offset = next(iter(self.stance_variants(robot, instance).values()), None) if step else None
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

    def stance_names(self, robot: str) -> list[str]:
        """Stance references for planners and pickers: the default variant without suffix, the others with."""
        names = []
        for instance in self.object_instances().values():
            for index, variant in enumerate(self.stance_variants(robot, instance)):
                names.append(STANCE_PREFIX + (instance.name if index == 0 else
                                              f'{instance.name}{VARIANT_SEPARATOR}{variant}'))
        return names

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
        return self._load(f'robots/{robot}/poses.yaml', schemas.JOINT_POSES, required=False, default={})

    def named_pose(self, robot: str, name: str) -> dict:
        """A named pose as ``{'joints': {joint: value}, 'tools': {tool: {tcp, approach, closing}}, 'elbow': side}``.
        Joints are set directly; tool targets (TCP in the base frame) are solved by IK when the pose is used, with the
        pose's elbow side if it has one (None: the profile's). A joint-space pose has no tools."""
        poses = self.joint_poses(robot)
        if name not in poses:
            raise KeyError(f'unknown pose {name!r} for robot {robot!r}')
        entry = poses[name]
        if 'tools' in entry:
            return {'joints': {j: float(v) for j, v in entry.get('joints', {}).items()}, 'tools': dict(entry['tools']),
                    'elbow': entry.get('elbow')}
        return {'joints': {j: float(v) for j, v in entry.items()}, 'tools': {}, 'elbow': None}

    def joint_pose(self, robot: str, name: str) -> dict[str, float]:
        """A joint-space pose; a pose given as tool targets has no fixed joint values (use named_pose)."""
        pose = self.named_pose(robot, name)
        if pose['tools']:
            raise KeyError(f'pose {name!r} of robot {robot!r} is given as tool targets; it needs IK (named_pose)')
        return pose['joints']

    def object(self, name: str) -> dict:
        return self._load(f'objects/{name}.yaml', schemas.OBJECT)

    def point_names(self, robot: str) -> dict[str, list[str]]:
        """Names a plan can refer to, by kind (for planners and the robot card)."""
        objects = []
        for path in sorted((self.root / 'objects').glob('*.yaml')):
            objects += [f'{path.stem}.{name}' for name in self.object(path.stem).get('points', {})]
        placed = []  # <instance>/<point>: a point on one placed object (go_to_point)
        for name, instance in sorted(self.object_instances().items()):
            try:
                placed += [f'{name}{VARIANT_SEPARATOR}{point}' for point in self.object(instance.object).get('points', {})]
            except ConfigError:
                continue
        contacts = []  # <instance>/<contact>: where a held object can be put (move_object to)
        for name, instance in sorted(self.object_instances().items()):
            try:
                contacts += [f'{name}{VARIANT_SEPARATOR}{c}' for c in self.object(instance.object).get('contacts', {})]
            except ConfigError:
                continue
        return {'locations': sorted(self.locations()), 'stances': self.stance_names(robot),
                'joint poses': sorted(self.joint_poses(robot)), 'object points': objects,
                'placed object points': placed, 'placed object contacts': contacts,
                'placed objects': sorted(self.object_instances())}

    def held_pose(self, robot: str, obj: str, name: str) -> dict:
        """robots/<robot>/held_poses.yaml <obj>.<name>: ``{'joints': {...}, 'path': [Pose3D, ...]}``."""
        poses = self._load(f'robots/{robot}/held_poses.yaml', schemas.HELD_POSES, required=False, default={})
        if name not in poses.get(obj, {}):
            known = ', '.join(sorted(poses.get(obj, {}))) or 'none'
            raise KeyError(f'robot {robot!r} has no held pose {name!r} for {obj!r} objects (robots/{robot}/'
                           f'held_poses.yaml; it has: {known})')
        entry = poses[obj][name]
        return {'joints': dict(entry.get('joints', {})),
                'path': [Pose3D.from_dict(p, 'base') for p in entry.get('path', [])]}

    def robot_object_points(self, robot: str) -> dict:
        """robots/<robot>/object_points.yaml: the robot's settings per object point ({object: {point: {elbow}}})."""
        return self._load(f'robots/{robot}/object_points.yaml', schemas.ROBOT_OBJECT_POINTS, required=False, default={})

    def object_point(self, obj: str, point: str, robot: str | None = None) -> dict:
        """A named point on an object: tool poses (canonical TCP, object-relative) plus its metadata.

        ``{'left_tool': Pose3D, ..., 'approach': {...} | None, 'tool_type': str | None, 'grip_width_m': float | None}``
        With ``robot``, that robot's settings for the point (object_points.yaml, e.g. ``elbow``) come on top.
        """
        points = self.object(obj).get('points', {})
        if point not in points:
            raise KeyError(f'object {obj!r} has no point {point!r}')
        entry = dict(points[point])
        result: dict[str, Any] = {key: entry.pop(key, None) for key in POINT_METADATA}
        result.update({tool: Pose3D.from_dict(pose, obj) for tool, pose in entry.items()})
        if robot:
            result.update(self.robot_object_points(robot).get(obj, {}).get(point, {}))
        return result

    def instance_point(self, reference: str, robot: str | None = None,
                       near: Pose2D | None = None) -> tuple[ObjectInstance, dict]:
        """Resolve ``<instance>/<point>``: a placed object (cell/objects.yaml) and a point of its type, as
        object_point() returns it (tool poses relative to the object). With ``robot`` and ``near`` (its base pose),
        the instance as that robot handles it (handling_instance)."""
        name, _, point = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances or not point:
            raise KeyError(f'unknown object point {reference!r}: expected <instance>/<point>, instances in '
                           f'cell/objects.yaml: {", ".join(sorted(instances)) or "none"}')
        instance = instances[name] if robot is None else self.handling_instance(instances[name], robot, near)
        return instance, self.object_point(instance.object, point, robot)

    def object_contact(self, obj: str, contact: str) -> Pose3D:
        """A contact frame of an object type (objects/<object>.yaml contacts), relative to the object."""
        contacts = self.object(obj).get('contacts', {})
        if contact not in contacts:
            raise KeyError(f'object {obj!r} has no contact {contact!r}'
                           + (f' (it has {", ".join(sorted(contacts))})' if contacts else ''))
        return Pose3D.from_dict(contacts[contact], obj)

    def instance_contact(self, reference: str) -> tuple[ObjectInstance, Pose3D]:
        """Resolve ``<instance>/<contact>``: a placed object and a contact of its type."""
        name, _, contact = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances or not contact:
            raise KeyError(f'unknown object contact {reference!r}: expected <instance>/<contact>, instances in '
                           f'cell/objects.yaml: {", ".join(sorted(instances)) or "none"}')
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
        return self.update_point('cell/locations.yaml', [name], pose)

    def save_stance(self, reference: str, pose: Pose2D, robot: str, per_instance: bool = False) -> tuple[Path, Pose2D]:
        """Store ``pose`` (cell frame) as the robot's stance at ``<instance>[/<variant>]``; returns file and offset."""
        name, _, variant = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances:
            raise ValueError(f'no object {name!r} in cell/objects.yaml')
        instance = self.handling_instance(instances[name], robot, pose)  # a symmetric object: as the robot sees it
        variant = variant or next(iter(self.stance_variants(robot, instance)), 'default')
        self._check_name(variant)
        offset = pose.relative_to(instance.pose, instance.object)
        keys = ['instances', instance.name, variant] if per_instance else ['objects', instance.object, variant]
        return self.update_point(f'robots/{robot}/stances.yaml', keys, offset), offset

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

    def save_object_point(self, reference: str, tool_poses: dict, note: str | None = None, robot: str | None = None,
                          near: Pose2D | None = None) -> tuple[Path, dict]:
        """Store tool TCP poses (cell frame: ``{tool: (position, quaternion xyzw)}``) as the point ``<point>`` of
        ``<instance>``'s object type, relative to its top item (as ``robot`` at ``near`` handles it, see
        handling_instance). Other tools and the point's metadata are kept. Returns the file and the stored
        ``{tool: Pose3D}``."""
        from ruamel.yaml.comments import CommentedMap, CommentedSeq

        name, _, point = reference.partition(VARIANT_SEPARATOR)
        instances = self.object_instances()
        if name not in instances or not point:
            raise ValueError(f'expected <instance>/<point> with an instance from cell/objects.yaml, got {reference!r}')
        if not tool_poses:
            raise ValueError('no tool poses to store')
        self._check_name(point)
        instance = instances[name] if robot is None else self.handling_instance(instances[name], robot, near)
        stored ={tool: instance.relative_pose(*pose) for tool, pose in tool_poses.items()}

        def flow(values) -> CommentedSeq:
            seq = CommentedSeq(round(float(v), 4) + 0.0 for v in values)  # + 0.0: no "-0.0"
            seq.fa.set_flow_style()
            return seq

        def edit(document):
            points = document.setdefault('points', CommentedMap())
            entry = points.get(point)
            if not isinstance(entry, dict):
                entry = points[point] = CommentedMap()
            for tool, pose in stored.items():
                key = f'{tool}{TOOL_SUFFIX}'
                value = CommentedMap(xyz=flow(pose.xyz), rpy=flow(pose.rpy))
                value.fa.set_flow_style()
                entry[key] = value
                entry.ca.items.pop(key, None)  # a hand-written comment no longer describes the taught pose
                if note:
                    entry.yaml_add_eol_comment(note, key, column=4)

        return self._edit_yaml(f'objects/{instance.object}.yaml', edit), stored

    def save_object_point_settings(self, obj: str, point: str, settings: dict, robot: str | None = None) -> Path:
        """Set or clear (None) an object point's ``approach_m`` (objects/<obj>.yaml) and ``elbow`` (the robot's:
        robots/<robot>/object_points.yaml; without ``robot`` the object file, as before); tool poses are kept."""
        unknown = set(settings) - {'approach_m', 'elbow'}
        if unknown:
            raise ValueError(f'unknown point settings: {", ".join(sorted(unknown))}')
        if robot and 'elbow' in settings:
            if point not in self.object(obj).get('points', {}):
                raise ValueError(f'object {obj!r} has no point {point!r}')
            elbow = settings['elbow']
            settings = {k: v for k, v in settings.items() if k != 'elbow'}

            def robot_edit(document):
                from ruamel.yaml.comments import CommentedMap

                points = document.setdefault(obj, CommentedMap())
                entry = points.get(point) if isinstance(points.get(point), dict) else CommentedMap()
                if elbow in (None, ''):
                    entry.pop('elbow', None)
                else:
                    entry['elbow'] = elbow
                if entry:
                    entry.fa.set_flow_style()
                    points[point] = entry
                else:
                    points.pop(point, None)
                if not points:
                    document.pop(obj, None)

            path = self._edit_yaml(f'robots/{robot}/object_points.yaml', robot_edit)
            if not settings:
                return path

        def edit(document):
            entry = document.get('points', {}).get(point)
            if not isinstance(entry, dict):
                raise ValueError(f'object {obj!r} has no point {point!r}')
            for key, value in settings.items():
                if value is None or value == '':
                    entry.pop(key, None)
                else:
                    entry[key] = round(float(value), 4) if key == 'approach_m' else value

        return self._edit_yaml(f'objects/{obj}.yaml', edit)

    def save_object_point_poses(self, obj: str, point: str, poses: dict) -> Path:
        """Replace tool poses of an object point: ``{tool: {'xyz': [...], 'rpy': [...]}}`` in the object's frame (its
        CAD coordinates, as taught; the point and its other tools and settings are kept)."""
        from ruamel.yaml.comments import CommentedMap, CommentedSeq

        def flow(values) -> CommentedSeq:
            seq = CommentedSeq(round(float(v), 4) + 0.0 for v in values)
            seq.fa.set_flow_style()
            return seq

        def edit(document):
            entry = document.get('points', {}).get(point)
            if not isinstance(entry, dict):
                raise ValueError(f'object {obj!r} has no point {point!r}')
            for tool, pose in poses.items():
                key = f'{tool}{TOOL_SUFFIX}'
                value = CommentedMap(xyz=flow(pose['xyz']), rpy=flow(pose['rpy']))
                value.fa.set_flow_style()
                entry[key] = value
                entry.ca.items.pop(key, None)

        return self._edit_yaml(f'objects/{obj}.yaml', edit)

    def rename_object_point(self, obj: str, point: str, rename: str) -> Path:
        """Rename an object point in place (position, settings and comment kept)."""
        self._check_name(rename)
        relative = f'objects/{obj}.yaml'

        def edit(document):
            points = document.get('points')
            if not isinstance(points, dict) or point not in points:
                raise ValueError(f'object {obj!r} has no point {point!r}')
            self._rename_key(points, point, rename, f'points/{rename}', relative)

        path = self._edit_yaml(relative, edit)
        for robot_file in sorted((self.root / 'robots').glob('*/object_points.yaml')):  # robots' settings follow
            hints = f'robots/{robot_file.parent.name}/object_points.yaml'
            if point in self.robot_object_points(robot_file.parent.name).get(obj, {}):
                self._edit_yaml(hints, lambda document, h=hints: self._rename_key(document[obj], point, rename,
                                                                                    rename, h))
        return path

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
    def update_point(self, relative: str, keys: list[str], pose: Pose2D | None = None,
                     rename: str | None = None) -> Path:
        """Set and/or rename the planar point at ``keys`` in a point file (``cell/locations.yaml`` or
        ``robots/<robot>/stances.yaml``). An existing entry keeps its place, its comment and its frame key; a new
        one is appended on one line. The result is checked against the file's schema before anything is written."""
        from ruamel.yaml.comments import CommentedMap

        if pose is None and rename is None:
            raise ValueError('nothing to change')
        self._check_name(keys[-1])
        if rename is not None:
            self._check_name(rename)

        def edit(document):
            node = document
            for key in keys[:-1]:
                if not isinstance(node.get(key), dict):
                    node[key] = CommentedMap()
                node.fa.set_block_style()
                node = node[key]
            node.fa.set_block_style()
            name = keys[-1]
            if pose is None and name not in node:
                raise ValueError(f'no point {"/".join(keys)!r} in {relative}')
            if pose is not None:
                values = pose.to_dict()
                if isinstance(node.get(name), dict):
                    node[name].update({key: values[key] for key in ('x', 'y', 'yaw')})
                else:
                    entry = CommentedMap(values)
                    entry.fa.set_flow_style()  # one line per point, like hand-written entries
                    node[name] = entry
            if rename is not None and rename != name:
                self._rename_key(node, name, rename, '/'.join(keys[:-1] + [rename]), relative)

        return self._edit_yaml(relative, edit)

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

    def set_joint_pose(self, robot: str, name: str, values: dict[str, float] | None = None,
                       rename: str | None = None, note: str | None = None) -> Path:
        """A joint-space pose; see set_named_pose."""
        return self.set_named_pose(robot, name, values, rename=rename, note=note)

    def set_named_pose(self, robot: str, name: str, joints: dict[str, float] | None = None, tools: dict | None = None,
                       rename: str | None = None, note: str | None = None, elbow: str | None = None) -> Path:
        """Set and/or rename a named pose in robots/<robot>/poses.yaml. Without ``tools`` it is a joint map on one line;
        with ``tools`` ({tool: {tcp, approach, closing}}) it is written as joints + tool targets, one line each, and
        ``elbow`` (preferred elbow side; None: the profile's).
        ``note`` replaces the entry's end-of-line comment (e.g. an outdated IK error)."""
        from ruamel.yaml.comments import CommentedMap, CommentedSeq

        relative = f'robots/{robot}/poses.yaml'
        if joints is None and tools is None and rename is None:
            raise ValueError('nothing to change')
        self._check_name(name)
        if rename is not None:
            self._check_name(rename)

        def flow(mapping: dict) -> CommentedMap:
            entry = CommentedMap()
            for key, value in mapping.items():
                if isinstance(value, (list, tuple)):
                    value = CommentedSeq(round(float(v), 4) for v in value)
                    value.fa.set_flow_style()
                else:
                    value = round(float(value), 4)
                entry[key] = value
            entry.fa.set_flow_style()
            return entry

        def edit(document):
            if joints is None and tools is None and name not in document:
                raise ValueError(f'no pose {name!r} in {relative}')
            if tools:
                entry = CommentedMap(joints=flow(joints or {}), tools=CommentedMap(
                    (tool, flow({key: target[key] for key in ('tcp', 'approach', 'closing')}))
                    for tool, target in tools.items()))
                if elbow:
                    entry['elbow'] = elbow
                document[name] = entry
            elif joints is not None:
                document[name] = flow(joints)
            if note and (joints is not None or tools):
                document.ca.items.pop(name, None)
                if not tools:  # a block entry has no end-of-line comment
                    document.yaml_add_eol_comment(note, name, column=2)  # right after the entry
            if rename is not None and rename != name:
                self._rename_key(document, name, rename, rename, relative)

        return self._edit_yaml(relative, edit)

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

    def delete_point(self, relative: str, keys: list[str]) -> Path:
        """Remove the point at ``keys``; containers below the top level that become empty are removed too."""

        def edit(document):
            path = [document]
            for key in keys[:-1]:
                if not isinstance(path[-1].get(key), dict):
                    raise ValueError(f'no point {"/".join(keys)!r} in {relative}')
                path.append(path[-1][key])
            if keys[-1] not in path[-1]:
                raise ValueError(f'no point {"/".join(keys)!r} in {relative}')
            del path[-1][keys[-1]]
            for depth in range(len(keys) - 1, 1, -1):  # e.g. instances.<name> once its last variant is gone
                if not path[depth]:
                    del path[depth - 1][keys[depth - 1]]
            for container in path[1:]:
                if not container:
                    container.fa.set_flow_style()  # an emptied top-level key reads `{}`

        return self._edit_yaml(relative, edit)

    def _edit_yaml(self, relative: str, edit) -> Path:
        """Apply ``edit(document)`` to a point file with comments and order kept; write only if the result is valid."""
        from ruamel.yaml import YAML  # only needed for writing
        from ruamel.yaml.comments import CommentedMap

        schema = (POINT_FILE_SCHEMAS.get(relative) or POINT_FILE_SCHEMAS.get(Path(relative).name)
                  or (schemas.OBJECT if Path(relative).parent == Path('objects') else None))
        if schema is None:
            raise ValueError(f'{relative} is not a point file')
        path = self.root / relative
        handler = YAML()
        handler.preserve_quotes = True
        handler.width = 4096  # one line per entry, as written by hand and by tools (joint poses are long)
        document = handler.load(path.read_text(encoding='utf-8')) if path.is_file() else None
        if document is None:
            document = CommentedMap()
        edit(document)
        errors = sorted(Draft202012Validator(schema).iter_errors(json.loads(json.dumps(document))),
                        key=lambda e: list(e.absolute_path))
        if errors:
            key = '.'.join(str(p) for p in errors[0].absolute_path)
            raise ValueError(f'{relative}{f" [{key}]" if key else ""}: {errors[0].message}; nothing written')
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', encoding='utf-8') as stream:
            handler.dump(document, stream)
        with self._lock:
            self._cache.pop(path, None)
        return path

    @staticmethod
    def _check_name(name: str) -> None:
        if not Draft202012Validator(schemas.NAME).is_valid(name):
            raise ValueError(f'invalid point name {name!r}: use letters, digits, "_" and "-", starting with a letter')
