"""POST /api/missions/<name>/edit over HTTP against a bridge without a ROS graph: the ops land in the file with an
undo, a refused edit is a 422 with its places, the undo puts the file back, and GET /api/schema/mission serves the
step schema the editor reads its blocks from."""

import json
import shutil
import threading
from http.server import ThreadingHTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

pytest.importorskip('rclpy')

from test_missions_edit import CATALOG, FIXTURE, WAIT  # noqa: E402

from taskweave_core.config import ConfigStore  # noqa: E402
from taskweave_hmi import undo  # noqa: E402
from taskweave_hmi.server import Bridge, Handler  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)


@pytest.fixture
def served(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    bridge = Bridge.__new__(Bridge)  # no node
    bridge.config = ConfigStore(tmp_path / 'config')
    bridge.robot = 'demo_bot'
    bridge.schemas = lambda: CATALOG
    bridge.undos = undo.Undo()
    bridge.lock, bridge.status = threading.Lock(), {'state': 'idle'}  # the undo's idle check
    bridge.points_changed = lambda: None
    logger = FakeLogger()
    bridge.get_logger = lambda: logger
    Handler.bridge = bridge
    httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{httpd.server_address[1]}'

    def request(method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        try:
            with urlopen(Request(base + path, data=data, method=method), timeout=10) as response:
                answer = json.loads(response.read())
                if response.headers.get('ETag'):
                    answer['_etag'] = response.headers['ETag']
                return response.status, answer
        except HTTPError as error:
            return error.code, json.loads(error.read())
    yield request, bridge
    httpd.shutdown()
    httpd.server_close()
    Handler.bridge = None


def test_edit_saves_with_an_undo_and_refuses_a_bad_step(served):
    request, bridge = served
    path = bridge.config.root / 'missions' / 'patrol.yaml'
    before = path.read_text()
    status, answer = request('POST', '/api/missions/patrol/edit',
                             {'ops': [{'op': 'insert', 'at': 'steps/3', 'step': WAIT}]})
    assert status == 200 and answer['file'] == 'missions/patrol.yaml'
    assert [s['instruction'] for s in answer['mission']['steps']][-1] == 'Wait a little'
    assert answer['undo'] and answer['undo']['files'] == ['missions/patrol.yaml']
    assert len(bridge.config.mission('patrol')['steps']) == 4
    status, answer = request('POST', '/api/missions/patrol/edit',
                             {'ops': [{'op': 'set', 'at': 'steps/0', 'key': 'args', 'value': {'to': 'nowhere'}}]})
    assert status == 422 and any('nowhere' in p for p in answer['problems']) and answer['places'][0]['step'] == 0
    status, answer = request('POST', '/api/missions/patrol/edit', {'ops': 'not a list'})
    assert status == 400
    # the ETag of a GET is the base of the next edit: stale after the edit above, fresh from the answer's hash
    status, opened = request('GET', '/api/missions/patrol')
    assert status == 200 and opened['_etag']
    status, answer = request('POST', '/api/missions/patrol/edit',
                             {'ops': [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'Again'}], 'base': 'stale'})
    assert status == 409 and 'changed since' in answer['problems'][0]
    status, answer = request('POST', '/api/missions/patrol/edit',
                             {'ops': [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'Again'}],
                              'base': opened['_etag']})
    assert status == 200 and answer['hash'] and answer['mission']['goal'] == 'Again'
    status, answer = request('POST', '/api/missions/patrol/edit', {'ops': [], 'create': True})
    assert status == 422 and 'already exists' in answer['problems'][0]
    # the undos put the file back as it was before the first edit, newest first (an older save's file changed since)
    assert len(bridge.undos.saves) == 2
    for key in reversed(list(bridge.undos.saves)):
        status, answer = request('POST', '/api/teach/undo', {'id': key})
        assert status == 200, answer
    assert path.read_text() == before


def test_schema_route_serves_the_step_schema(served):
    request, _bridge = served
    status, schema = request('GET', '/api/schema/mission')
    assert status == 200 and 'steps' in schema['properties']
    assert 'skill' in schema['properties']['steps']['items']['properties']
