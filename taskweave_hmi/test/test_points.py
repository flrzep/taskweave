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


def test_override_with_a_fallback_can_be_deleted_even_if_used(store):
    path = store.root / 'missions' / 'patrol.yaml'
    path.write_text(path.read_text().replace('stance:shelf_a', 'stance:shelf_b'))
    assert points.overview(store, ROBOT)['objects'][0]['instances'][1]['overrides'][0]['used_by'] == ['patrol']
    points.delete(store, ROBOT, 'stance', ['instances', 'shelf_b', 'front'])
    assert store.base_target('stance:shelf_b', ROBOT).y == pytest.approx(0.25)


def test_new_points_are_created_once_and_checked(store):
    points.save(store, ROBOT, 'stance', ['objects', 'crate', 'load'], {'x': -0.5, 'y': 0, 'yaw': 0, 'create': True})
    assert store.base_target('stance:crate_1', ROBOT).y == pytest.approx(-2.5)
    with pytest.raises(points.PointError, match='already exists'):
        points.save(store, ROBOT, 'stance', ['objects', 'crate', 'load'], {'x': 0, 'y': 0, 'yaw': 0, 'create': True})
    with pytest.raises(points.PointError, match='required numbers'):
        points.save(store, ROBOT, 'location', ['spot'], {'x': 'left', 'y': 0, 'yaw': 0})
    with pytest.raises(points.PointError, match="no object 'ghost'"):
        points.save(store, ROBOT, 'stance', ['instances', 'ghost', 'front'], {'x': 0, 'y': 0, 'yaw': 0})
    with pytest.raises(points.PointError, match='unknown point address'):
        points.save(store, ROBOT, 'stance', ['everywhere', 'shelf', 'front'], {'x': 0, 'y': 0, 'yaw': 0})


def test_robot_pose_relative_to_objects_nearest_first(store):
    data = points.relative_poses(store, Pose2D(3.0, 0.25, 1.5708))
    assert data['relative'][0]['instance'] == 'shelf_a'
    offset = data['relative'][0]['offset']
    assert (offset['x'], offset['y'], offset['yaw']) == pytest.approx((0.0, -0.75, 1.5708), abs=1e-4)


def test_object_points_are_listed_and_their_settings_saved(store):
    top = next(p for p in points.object_points(store, 'box', {}) if p['name'] == 'top_grasp')
    assert top['approach_m'] is None and top['elbow'] is None and top['tools']['left']['xyz'] == [0.0, 0.1, 0.2]
    points.save_object_point(store, 'box', 'top_grasp', {'approach_m': 0.15, 'elbow': 'down',
                                                         'tools': {'left': {'xyz': [0, 0.1, 0.22], 'rpy': [0, 1.5, 0]}}})
    top = next(p for p in points.object_points(store, 'box', {}) if p['name'] == 'top_grasp')
    assert top['approach_m'] == 0.15 and top['elbow'] == 'down' and top['tools']['left']['xyz'] == [0.0, 0.1, 0.22]
    with pytest.raises(points.PointError, match='elbow'):
        points.save_object_point(store, 'box', 'top_grasp', {'elbow': 'sideways'})
    with pytest.raises(points.PointError, match='approach_m'):
        points.save_object_point(store, 'box', 'top_grasp', {'approach_m': 3})
