"""The live event points without a ROS graph: a save, delete or capture over HTTP tells every browser (the mission
editor's argument pickers fetch /api/points again, which lists the new point at once); a refused one does not."""

import shutil
import threading
from pathlib import Path

import pytest

pytest.importorskip('rclpy')

from test_speed import FakeLogger, serve  # noqa: E402, F401 - serve: the fixture

from taskweave_core.config import ConfigStore  # noqa: E402
from taskweave_hmi.server import Bridge  # noqa: E402

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'


def points_bridge(config):
    """A Bridge with a real config store and nothing of ROS: the events it sends are kept in ``sent``."""
    bridge = Bridge.__new__(Bridge)  # no node
    bridge.lock, bridge.listeners, bridge.sent = threading.Lock(), [], []
    bridge.config, bridge.robot, bridge.manual = config, 'demo_bot', None
    bridge.check_idle = lambda: None
    logger = FakeLogger()
    bridge.get_logger = lambda: logger
    bridge._broadcast = lambda kind, data: bridge.sent.append((kind, data))
    return bridge


@pytest.fixture
def config(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


def test_a_saved_location_is_announced_and_in_the_picker_list_at_once(serve, config):
    bridge = points_bridge(config)
    request = serve(bridge)
    status, names = request('GET', '/api/points')
    assert status == 200 and 'spot' not in names['locations']
    status, _ = request('PUT', '/api/points/location/spot', {'x': 0.6899, 'y': -1.5708, 'yaw': 0, 'create': True})
    assert status == 200 and bridge.sent == [('points', None)]
    assert 'spot' in request('GET', '/api/points')[1]['locations']  # no reload needed: the store reads the new file
    status, _ = request('DELETE', '/api/points/location/spot')
    assert status == 200 and bridge.sent == [('points', None)] * 2
    assert 'spot' not in request('GET', '/api/points')[1]['locations']


def test_a_refused_save_announces_nothing(serve, config):
    bridge = points_bridge(config)
    request = serve(bridge)
    status, data = request('DELETE', '/api/points/location/home')  # used by a mission
    assert status == 422 and 'used by' in data['problems'][0]
    status, _ = request('PUT', '/api/points/location/spot', {'x': 'left', 'y': 0, 'yaw': 0})
    assert status == 422 and bridge.sent == []


def test_a_pad_teach_press_announces_the_new_point():
    from types import SimpleNamespace

    bridge = points_bridge(None)
    bridge.teach_target = {'scope': 'robot', 'name': 'look_down'}
    bridge.capture = lambda scope, name, body: {'file': 'robots/demo_bot/poses.yaml', 'message': ''}
    notices = []
    bridge.notice_pub = SimpleNamespace(publish=notices.append)
    bridge._teach_request(SimpleNamespace(data='{}'))
    assert bridge.sent == [('points', None)] and 'taught robot look_down' in notices[0].data
    bridge.sent.clear()
    bridge.capture = lambda *_: (_ for _ in ()).throw(TimeoutError('no joint states'))
    bridge._teach_request(SimpleNamespace(data='{}'))
    assert bridge.sent == [] and 'failed' in notices[1].data
