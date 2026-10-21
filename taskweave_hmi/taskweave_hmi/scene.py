"""The HMI's 3D view: the robot model, the planning scene and the taught points as JSON. No ROS dependency.

The browser draws the robot from its URDF (``robots/<robot>/moveit/robot.urdf``, the file MoveIt plans with): mesh
file names are rewritten to the server's ``/robot/meshes/<file>`` and the STL files are served from the URDF's
``meshes/`` folder. The scene combines what MoveIt avoids (the world collision objects of the planning scene, as
the server last fetched them) with the config the robot works from (placed objects, locations, object points with
their tool poses in the object frame; on a cell pack in the targets layout: the cell's base targets and the types' hand
targets, points.object_points). Object types have no served mesh yet: ``mesh`` is null and the browser draws
a marker.

Quaternions in the JSON are [w, x, y, z] (three.js order is x, y, z, w: the viewer reorders); internally they are
(x, y, z, w) as in geometry_msgs and config.poses.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ElementTree
from collections.abc import Callable, Sequence
from pathlib import Path

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_core.config.limits import urdf_path
from taskweave_core.config.poses import quaternion_multiply, rotate

from . import points

MESH_URL = '/robot/meshes/'
# a mesh file name in the URDF: meshes/<file> relative to it (also package:// or file:// paths ending in meshes/)
MESH_FILENAME = re.compile(r'filename="(?:[^"]*/)?meshes/([^"/]+)"')
PRIMITIVES = {1: 'box', 2: 'sphere', 3: 'cylinder', 4: 'cone', 5: 'prism'}   # shape_msgs/SolidPrimitive types
IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

Transform = tuple[tuple[float, float, float], tuple[float, float, float, float]]   # position, quaternion x, y, z, w


# ---- robot model ---------------------------------------------------------------------
def rewrite_urdf(text: str) -> str:
    """The URDF with every mesh file name pointing at the server (``/robot/meshes/<file>``)."""
    return MESH_FILENAME.sub(lambda match: f'filename="{MESH_URL}{match.group(1)}"', text)


def root_link(text: str) -> str | None:
    """The URDF's root link: the one link no joint has as its child."""
    root = ElementTree.fromstring(text)
    children = {child.get('link') for joint in root.iter('joint') if (child := joint.find('child')) is not None}
    return next((link.get('name') for link in root.iter('link') if link.get('name') not in children), None)


def robot_model(store: ConfigStore, robot: str) -> dict | None:
    """URDF (mesh paths rewritten), root link, tool frames and grippers of the robot; None if it has no URDF."""
    path = urdf_path(store, robot)
    if not path.is_file():
        return None
    text = path.read_text(encoding='utf-8')
    try:
        root = root_link(text)
    except ElementTree.ParseError as error:
        raise ConfigError(path, f'not a valid URDF: {error}') from error
    profile = store.robot(robot).data
    return {'robot': robot, 'urdf': rewrite_urdf(text), 'root_link': root,
            'tool_frames': {tool: {'link': frame['link'], 'xyz': list(frame['xyz']), 'rpy': list(frame['rpy'])}
                            for tool, frame in profile.get('tool_frames', {}).items()},
            'grippers': {tool: {key: gripper.get(key) for key in ('joint', 'open', 'closed')}
                         for tool, gripper in profile.get('grippers', {}).items()}}


def mesh_file(store: ConfigStore, robot: str, name: str) -> Path | None:
    """The STL file ``name`` from the URDF's meshes/ folder; None unless it is a plain .stl file name found there."""
    if not name or '/' in name or '\\' in name or '..' in name or not name.lower().endswith('.stl'):
        return None
    folder = urdf_path(store, robot).parent / 'meshes'
    return _inside(folder, folder / name)


def vendor_file(root: Path, parts: Sequence[str]) -> Path | None:
    """The file at path ``parts`` below ``root`` (the viewer's vendored libraries); None if it is not a file there,
    or the path leaves ``root`` (``..``, an absolute or drive path, a symlink pointing out)."""
    if not parts or any(part in ('', '.', '..') or '/' in part or '\\' in part or ':' in part for part in parts):
        return None
    return _inside(root, root.joinpath(*parts))


def _inside(root: Path, path: Path) -> Path | None:
    try:
        resolved = path.resolve()
        resolved.relative_to(root.resolve())
    except (OSError, ValueError):
        return None
    return resolved if resolved.is_file() else None


# ---- planning scene ------------------------------------------------------------------
def _compose(a: Transform, b: Transform) -> Transform:
    offset = rotate(a[1], b[0])
    return (a[0][0] + offset[0], a[0][1] + offset[1], a[0][2] + offset[2]), quaternion_multiply(a[1], b[1])


def _pose(pose) -> Transform:
    """A geometry_msgs Pose as a transform; an all-zero quaternion (a pose left unset) is no rotation."""
    q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    norm = math.sqrt(sum(v * v for v in q))
    return ((pose.position.x, pose.position.y, pose.position.z),
            tuple(v / norm for v in q) if norm > 1e-9 else IDENTITY[1])


def _rounded(values, digits: int = 5) -> list[float]:
    return [round(float(v), digits) for v in values]


def collision_objects(objects, cell_frame: str = 'world',
                      transform: Callable[[str], Transform | None] | None = None) -> tuple[list[dict], int]:
    """World collision objects of a planning scene (``scene.world.collision_objects``) as primitives in the cell
    frame: (shapes, skipped). Each primitive pose is composed with its object's pose and, for an object in another
    frame, that frame's pose in the cell (``transform(frame)``, None: unknown). A box has ``size`` [x, y, z], other
    primitives their ``dimensions`` as SolidPrimitive gives them (sphere [r], cylinder and cone [height, r]).
    Meshes, planes and objects in a frame that cannot be placed are left out and counted in ``skipped``."""
    shapes, skipped = [], 0
    for obj in objects:
        frame = obj.header.frame_id.lstrip('/')
        placed = IDENTITY if frame in ('', cell_frame) else (transform(frame) if transform else None)
        skipped += len(obj.meshes) + len(obj.planes)
        if placed is None:
            skipped += len(obj.primitives)
            continue
        origin = _compose(placed, _pose(obj.pose))
        many = len(obj.primitives) > 1
        for index, (primitive, pose) in enumerate(zip(obj.primitives, obj.primitive_poses, strict=False)):
            (x, y, z), (qx, qy, qz, qw) = _compose(origin, _pose(pose))
            kind = PRIMITIVES.get(primitive.type, f'type {primitive.type}')
            shape = {'id': f'{obj.id}/{index}' if many else obj.id, 'type': kind}
            shape['size' if kind == 'box' else 'dimensions'] = _rounded(primitive.dimensions)
            shapes.append({**shape, 'xyz': _rounded((x, y, z)), 'quat': _rounded((qw, qx, qy, qz))})
    return shapes, skipped


# ---- config --------------------------------------------------------------------------
def object_types(store: ConfigStore) -> list[str]:
    """Object types with a file in objects/, plus the types of placed objects."""
    files = {path.stem for path in (store.root / 'objects').glob('*.yaml')}
    return sorted(files | {instance.object for instance in store.object_instances().values()})


def config_scene(store: ConfigStore) -> dict:
    """Placed objects (with the run-time object state), locations and object points from the config. A point's
    tool poses are in the object frame: at the placed object's top item (``top_z`` for a stack)."""
    placed = [{'name': name, 'object': instance.object, 'x': round(instance.pose.x, 4),
               'y': round(instance.pose.y, 4), 'z': round(instance.z, 4), 'yaw': round(instance.pose.yaw, 4),
               'top_z': round(instance.top_z, 4),
               'mesh': None}   # object meshes are not served yet
              for name, instance in sorted(store.object_instances().items())]
    locations = [{'name': name, **points.pose_dict(pose)} for name, pose in sorted(store.locations().items())]
    object_points = []
    for object_type in object_types(store):
        try:
            entries = points.object_points(store, object_type, {})
        except (ConfigError, KeyError):   # a type file taskweave cannot read (e.g. sim settings only): no points
            continue
        object_points += [{'object_type': object_type, 'name': point['name'], 'tools': point['tools']}
                          for point in entries]
    return {'placed': placed, 'locations': locations, 'points': object_points}
