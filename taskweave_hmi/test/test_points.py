import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore, Pose2D
from taskweave_hmi import points

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
ROBOT = 'demo_bot'


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


def test_overview_lists_points_with_their_missions(store):
    data = points.overview(store, ROBOT)
    locations = {loc['name']: loc for loc in data['locations']}
    assert locations['home']['used_by'] == ['patrol'] and locations['dock']['used_by'] == []
    shelf = next(o for o in data['objects'] if o['object'] == 'shelf')
    variants = {v['variant']: v for v in shelf['variants']}
    assert variants['front']['used_by'] == ['patrol'] and variants['side']['used_by'] == []   # stance:shelf_a = front
    shelf_b = next(i for i in shelf['instances'] if i['name'] == 'shelf_b')
    assert shelf_b['overrides'][0]['variant'] == 'front' and shelf_b['overrides'][0]['fallback']
    assert shelf_b['stances']['shelf_b/front']['y'] == pytest.approx(0.1)
    assert any(o['object'] == 'crate' and o['variants'] == [] for o in data['objects'])        # placed, no stance yet


def test_used_points_cannot_be_renamed_or_deleted(store):
    with pytest.raises(points.PointError, match='used by patrol'):
        points.delete(store, ROBOT, 'location', ['home'])
    with pytest.raises(points.PointError, match='used by patrol'):
        points.save(store, ROBOT, 'location', ['home'], {'x': 0, 'y': 0, 'yaw': 0, 'rename': 'start'})
    with pytest.raises(points.PointError, match='used by patrol'):
        points.delete(store, ROBOT, 'stance', ['objects', 'shelf', 'front'])
    points.save(store, ROBOT, 'location', ['home'], {'x': 0.1, 'y': 0, 'yaw': 0})               # moving it is fine
    assert store.locations()['home'].x == pytest.approx(0.1)
    points.save(store, ROBOT, 'location', ['dock'], {'x': 1.5, 'y': -1, 'yaw': 3.1416, 'rename': 'charger'})
    points.delete(store, ROBOT, 'location', ['charger'])
    assert list(store.locations()) == ['home']


def test_robot_pose_relative_to_objects_nearest_first(store):
    data = points.relative_poses(store, Pose2D(3.0, 0.25, 1.5708))
    assert data['relative'][0]['instance'] == 'shelf_a'
    offset = data['relative'][0]['offset']
    assert (offset['x'], offset['y'], offset['yaw']) == pytest.approx((0.0, -0.75, 1.5708), abs=1e-4)


def test_a_points_grip_is_saved_for_the_robot(store):
    profile = store.root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text().replace(
        'left: {action: /left_gripper/command,',
        'left: {joint: left_finger, opening_m: [[0.0, 0.08], [1.0, 0.0]], action: /left_gripper/command,'))
    store.reload()
    points.save_object_point(store, 'box', 'top_grasp', {'grip_m': {'left': '0.012'}}, ROBOT)
    top = next(p for p in points.object_points(store, 'box', {}, ROBOT) if p['name'] == 'top_grasp')
    assert top['grip_m'] == {'left': 0.012}
    points.save_object_point(store, 'box', 'top_grasp', {'grip_m': {'left': ''}}, ROBOT)   # empty: close fully
    assert next(p for p in points.object_points(store, 'box', {}, ROBOT) if p['name'] == 'top_grasp')['grip_m'] == {}
    with pytest.raises(points.PointError, match='grip left'):
        points.save_object_point(store, 'box', 'top_grasp', {'grip_m': {'left': 'tight'}}, ROBOT)
    with pytest.raises(points.PointError, match='outside 0..80 mm'):
        points.save_object_point(store, 'box', 'top_grasp', {'grip_m': {'left': 0.09}}, ROBOT)
    with pytest.raises(points.PointError, match='no opening_m table'):
        points.save_object_point(store, 'box', 'top_grasp', {'grip_m': {'right': 0.01}}, ROBOT)
    with pytest.raises(points.PointError, match='robot'):
        points.save_object_point(store, 'box', 'top_grasp', {'grip_m': {'left': 0.01}})
