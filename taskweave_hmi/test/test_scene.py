import math
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from taskweave_core.config import ConfigError, ConfigStore
from taskweave_hmi import scene

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'
URDF = """<robot name="demo_bot">
  <link name="base_link"><visual><geometry><mesh filename="meshes/base.stl"/></geometry></visual></link>
  <link name="arm"><collision><geometry><mesh filename="package://demo/moveit/meshes/arm.stl"/></geometry></collision>
  </link>
  <link name="tool"/>
  <joint name="j1" type="revolute"><parent link="base_link"/><child link="arm"/></joint>
  <joint name="j2" type="fixed"><parent link="arm"/><child link="tool"/></joint>
</robot>
"""
TOOL_FRAMES = """tool_frames:
  left: {link: tool, xyz: [0.0, 0.0, 0.1], rpy: [0.0, 0.0, 0.0]}
"""


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


def with_urdf(store: ConfigStore) -> Path:
    moveit = store.root / 'robots' / ROBOT / 'moveit'
    (moveit / 'meshes').mkdir(parents=True)
    (moveit / 'robot.urdf').write_text(URDF)
    (moveit / 'meshes' / 'base.stl').write_bytes(b'solid base\n')
    profile = store.root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text() + TOOL_FRAMES)
    return moveit


def test_urdf_mesh_paths_point_at_the_server_and_the_root_is_found():
    text = scene.rewrite_urdf(URDF)
    assert 'filename="/robot/meshes/base.stl"' in text and 'filename="/robot/meshes/arm.stl"' in text
    assert scene.root_link(URDF) == 'base_link'


def test_robot_model_has_the_urdf_tool_frames_and_grippers(store):
    assert scene.robot_model(store, ROBOT) is None   # no URDF: the server answers 404
    with_urdf(store)
    model = scene.robot_model(store, ROBOT)
    assert model['robot'] == ROBOT and model['root_link'] == 'base_link'
    assert '/robot/meshes/base.stl' in model['urdf']
    assert model['tool_frames'] == {'left': {'link': 'tool', 'xyz': [0.0, 0.0, 0.1], 'rpy': [0.0, 0.0, 0.0]}}
    assert model['grippers'] == {'left': {'joint': None, 'open': 0.0, 'closed': 1.0}}


def test_a_broken_urdf_is_a_config_error(store):
    moveit = with_urdf(store)
    (moveit / 'robot.urdf').write_text('<robot name="demo_bot">')
    with pytest.raises(ConfigError, match='not a valid URDF'):
        scene.robot_model(store, ROBOT)


def test_only_plain_stl_names_from_the_meshes_folder_are_served(store):
    moveit = with_urdf(store)
    (moveit / 'robot.srdf').write_text('<robot/>')
    assert scene.mesh_file(store, ROBOT, 'base.stl') == (moveit / 'meshes' / 'base.stl').resolve()
    for name in ('missing.stl', '../robot.urdf', '..', 'sub/base.stl', 'sub\\base.stl', '../robot.srdf', '',
                 'robot.urdf', '..base.stl'):
        assert scene.mesh_file(store, ROBOT, name) is None, name


def test_vendor_files_are_served_below_the_folder_only(tmp_path):
    vendor = tmp_path / 'web' / 'vendor'
    (vendor / 'three' / 'addons' / 'controls').mkdir(parents=True)
    (vendor / 'three' / 'addons' / 'controls' / 'OrbitControls.js').write_text('export {};\n')
    (tmp_path / 'web' / 'secret.txt').write_text('no')
    (vendor / 'three' / 'out.js').symlink_to(tmp_path / 'web' / 'secret.txt')
    found = scene.vendor_file(vendor, ['three', 'addons', 'controls', 'OrbitControls.js'])
    assert found == (vendor / 'three' / 'addons' / 'controls' / 'OrbitControls.js').resolve()
    for parts in ([], ['..', 'secret.txt'], ['three', '..', '..', 'secret.txt'], ['three', 'out.js'], ['three'],
                  [str(tmp_path / 'web' / 'secret.txt')], ['three/../../secret.txt'], ['C:', 'x.js'], ['', 'x.js']):
        assert scene.vendor_file(vendor, parts) is None, parts


def pose(xyz=(0.0, 0.0, 0.0), yaw=0.0, unset=False):
    q = (0.0, 0.0, 0.0, 0.0) if unset else (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return SimpleNamespace(position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
                           orientation=SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))


def collision_object(id, frame, at, primitives=(), meshes=0):
    return SimpleNamespace(id=id, header=SimpleNamespace(frame_id=frame), pose=at,
                           primitives=[SimpleNamespace(type=kind, dimensions=dims) for kind, dims, _ in primitives],
                           primitive_poses=[offset for _, _, offset in primitives], meshes=[object()] * meshes,
                           planes=[])


def test_collision_objects_are_placed_in_the_cell_frame():
    objects = [
        # a table turned 90 deg: its box sits 0.5 m along the object's x, i.e. along the cell's y
        collision_object('table', 'world', pose((1.0, 2.0, 0.0), math.pi / 2),
                         [(1, [1.0, 0.6, 0.7], pose((0.5, 0, 0.35)))]),
        # two primitives on the robot's base, 1 m ahead of the cell origin (from TF); a mesh is left out
        collision_object('rack', 'base_link', pose(unset=True), [(1, [0.1, 0.1, 0.1], pose((0, 0, 1))),
                                                                  (2, [0.05], pose((0, 0, 2)))], meshes=1),
        collision_object('ghost', 'camera', pose(), [(1, [1, 1, 1], pose())]),   # no transform: left out
    ]
    frames = {'base_link': ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))}
    shapes, skipped = scene.collision_objects(objects, 'world', frames.get)
    table, box, ball = shapes
    assert table['id'] == 'table' and table['type'] == 'box' and table['size'] == [1.0, 0.6, 0.7]
    assert table['xyz'] == pytest.approx([1.0, 2.5, 0.35])
    assert table['quat'] == pytest.approx([math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)], abs=1e-5)
    assert (box['id'], box['xyz'], box['quat']) == ('rack/0', [1.0, 0.0, 1.0], [1.0, 0.0, 0.0, 0.0])
    assert (ball['id'], ball['type'], ball['dimensions'], ball['xyz']) == ('rack/1', 'sphere', [0.05], [1.0, 0.0, 2.0])
    assert 'size' not in ball and skipped == 2   # the mesh and the ghost's box
    assert scene.collision_objects([], 'world') == ([], 0)


def test_config_scene_lists_placed_objects_locations_and_points(store):
    (store.root / 'objects' / 'part_box.yaml').write_text('name: part_box\nsim: {size: [0.4, 0.3, 0.2]}\n')
    (store.root / 'objects' / 'odd.yaml').write_text('sim: {size: [0.4, 0.3, 0.2]}\n')   # not a taskweave object
    data = scene.config_scene(store)
    crate = next(p for p in data['placed'] if p['name'] == 'crate_1')
    assert crate == {'name': 'crate_1', 'object': 'crate', 'x': 1.0, 'y': -2.0, 'z': 0.4, 'yaw': 1.5708, 'top_z': 0.4,
                     'mesh': None}
    assert [p['name'] for p in data['placed']] == ['crate_1', 'shelf_a', 'shelf_b']
    assert data['locations'] == [{'name': 'dock', 'x': 1.5, 'y': -1.0, 'yaw': 3.1416},
                                 {'name': 'home', 'x': 0.0, 'y': 0.0, 'yaw': 0.0}]
    top = next(p for p in data['points'] if p['name'] == 'top_grasp')
    assert top == {'object_type': 'box', 'name': 'top_grasp',
                   'tools': {'left': {'xyz': [0.0, 0.1, 0.2], 'rpy': [0.0, 1.5708, 0.0]}}}
    assert {p['object_type'] for p in data['points']} == {'box'}   # part_box, crate, shelf: no points
