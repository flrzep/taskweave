"""The speed override control without a ROS graph: the request a POST /api/speed body makes, what the browser is
shown, live updates only on change, and the endpoint over HTTP against a fake task handler service."""

import json
import threading
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

pytest.importorskip('rclpy')

from taskweave_hmi.server import Bridge, Handler, speed_dict, speed_request  # noqa: E402
from taskweave_interfaces.msg import SpeedOverride as SpeedOverrideMsg  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)


def speed_bridge(answer=None, ready=True):
    """A Bridge with only the speed parts; ``answer(request)`` plays the task handler's set_speed_override."""
    bridge = Bridge.__new__(Bridge)  # no node
    bridge.lock, bridge.listeners, bridge.sent = threading.Lock(), [], []
    bridge.speed_msg, bridge.speed_state = None, speed_dict(None, False)
    bridge.speed_client = SimpleNamespace(service_is_ready=lambda: ready, srv_name='/set_speed_override')
    logger = FakeLogger()
    bridge.get_logger = lambda: logger

    def call(_client, request):
        bridge.sent.append(request)
        if answer is None:
            raise TimeoutError('/set_speed_override not available - is the task handler running?')
        return answer(request)
    bridge.call = call
    bridge._broadcast = lambda kind, data: bridge.listeners.append((kind, data))
    return bridge


def test_a_body_sets_only_what_it_names():
    request = speed_request({'percent': 30})
    assert request.percent == 30.0 and request.test_percent == 0.0 and not request.set_test_mode
    request = speed_request({'test_mode': True})
    assert request.percent == 0.0 and request.set_test_mode and request.test_mode   # percent 0: unchanged
    request = speed_request({'percent': 10.5, 'test_mode': False, 'test_percent': 20})
    assert request.percent == pytest.approx(10.5) and request.test_percent == 20.0
    assert request.set_test_mode and not request.test_mode
    for body in ({'percent': '50'}, {'percent': True}, {'percent': 0}, {'percent': -5}, {'test_mode': 'yes'}):
        with pytest.raises(ValueError):
            speed_request(body)
    assert speed_request({'percent': 150}).percent == 150.0  # the range is the handler's to refuse, with its message


def test_the_browser_sees_rounded_percents_and_nothing_before_the_first_message():
    assert speed_dict(None, False) == {'percent': None, 'test_percent': None, 'cap_percent': None, 'test_mode': False,
                                       'presets': [10, 50, 100], 'min': 1.0, 'max': 100.0, 'available': False}
    msg = SpeedOverrideMsg(percent=33.3, test_mode=True, test_percent=10.0, cap_percent=50.0)
    shown = speed_dict(msg, True)
    assert shown['percent'] == 33.3 and shown['test_mode'] and shown['cap_percent'] == 50.0 and shown['available']


def test_a_speed_event_goes_out_when_the_override_or_the_handler_changes():
    ready = {'now': False}
    bridge = speed_bridge()
    bridge.speed_client = SimpleNamespace(service_is_ready=lambda: ready['now'])
    bridge._speed_update()                         # timer: no handler yet, nothing new to show
    assert bridge.listeners == []
    ready['now'] = True
    bridge._speed_update(SpeedOverrideMsg(percent=50.0, test_percent=10.0, cap_percent=50.0))
    bridge._speed_update()                         # timer: unchanged
    assert [(kind, data['percent'], data['available']) for kind, data in bridge.listeners] == [('speed', 50.0, True)]
    ready['now'] = False                           # the task handler is gone: the control greys out, value kept
    bridge._speed_update()
    assert bridge.listeners[-1][1]['available'] is False and bridge.listeners[-1][1]['percent'] == 50.0
    assert bridge.speed_state is bridge.listeners[-1][1]


@pytest.fixture
def serve():
    """Run the HMI's request handler on a free port with the given bridge; post(path, body) -> (status, json)."""
    servers = []

    def start(bridge):
        Handler.bridge = bridge
        httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        servers.append(httpd)
        base = f'http://127.0.0.1:{httpd.server_address[1]}'

        def request(method, path, body=None):
            data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
            try:
                with urlopen(Request(base + path, data=data, method=method), timeout=10) as response:
                    return response.status, json.loads(response.read())
            except HTTPError as error:
                return error.code, json.loads(error.read())
        return request
    yield start
    for httpd in servers:
        httpd.shutdown()
        httpd.server_close()
    Handler.bridge = None


def test_post_speed_forwards_to_the_task_handler_and_returns_its_message(serve):
    def handler(request):  # test mode on, cap 50: the handler clamps and says so
        percent = min(request.percent, 50.0)
        note = '; clamped to the test mode cap 50 %' if request.percent > 50 else ''
        return SimpleNamespace(ok=True, message=f'speed {percent:g} %{note}', percent=percent)
    bridge = speed_bridge(handler)
    request = serve(bridge)
    assert request('POST', '/api/speed', {'percent': 80}) == (
        200, {'ok': True, 'message': 'speed 50 %; clamped to the test mode cap 50 %', 'percent': 50.0})
    assert bridge.sent[-1].percent == 80.0 and not bridge.sent[-1].set_test_mode
    status, _ = request('POST', '/api/speed', {'test_mode': True})
    assert status == 200 and bridge.sent[-1].set_test_mode and bridge.sent[-1].test_mode
    assert bridge.get_logger().lines[0] == 'HMI speed override: speed 50 %; clamped to the test mode cap 50 %'
    status, data = request('POST', '/api/speed', {'percent': 'fast'})
    assert status == 400 and 'percent' in data['error'] and len(bridge.sent) == 2  # never reached the handler
    assert request('POST', '/api/speed', b'{not json')[0] == 400
    assert request('GET', '/api/speed')[1]['presets'] == [10, 50, 100]


def test_post_speed_reports_a_refusal_and_a_missing_task_handler(serve):
    refused = SimpleNamespace(ok=False, message='speed override 150 % is outside 1-100 %', percent=100.0)
    request = serve(speed_bridge(lambda _request: refused))
    assert request('POST', '/api/speed', {'percent': 150}) == (
        422, {'ok': False, 'message': 'speed override 150 % is outside 1-100 %', 'percent': 100.0})
    request = serve(speed_bridge(None))
    status, data = request('POST', '/api/speed', {'percent': 50})
    assert status == 503 and 'task handler' in data['error']
