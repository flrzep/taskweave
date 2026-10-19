"""The Nav2 skill's speed override mechanism (nav2.py _limit_speed / _scale_accel): SpeedLimit percentage plus MPPI
accelerations at nominal x o^2 through fake publishers and parameter clients. No ROS needed."""

from types import SimpleNamespace

from taskweave_skills.nav2 import NAV2_DEFAULTS, Nav2NavigateTo

NOMINAL = {'ax_max': 2.0, 'ax_min': -2.0, 'az_max': 2.0}


class FakeFuture:
    def __init__(self, ok=True):
        self.ok = ok

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return SimpleNamespace(results=[SimpleNamespace(successful=self.ok, reason='' if self.ok else 'rejected')])


class FakeClient:
    def __init__(self, ready=True, ok=True):
        self.ready, self.ok, self.requests = ready, ok, []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append({p.name: p.value.double_value for p in request.parameters})
        return FakeFuture(self.ok)


class FakeLog:
    def __init__(self):
        self.warnings = []

    def warning(self, text):
        self.warnings.append(text)


def skill(client=None, **nav):
    sent, log = [], FakeLog()
    s = SimpleNamespace(nav={**NAV2_DEFAULTS, **nav}, _speed_sent=None, _accel_sent=None, _accel_warned=False,
                        speed_limit=SimpleNamespace(publish=sent.append), SpeedLimit=lambda **kw: kw, sent=sent,
                        accel_client=client, node=SimpleNamespace(get_logger=lambda: log), log=log,
                        SetParameters=__import__('rcl_interfaces.srv', fromlist=['SetParameters']).SetParameters)
    s._scale_accel = lambda percent: Nav2NavigateTo._scale_accel(s, percent)
    s._accel_done = lambda future: Nav2NavigateTo._accel_done(s, future)
    return s


def limit(s, o):
    Nav2NavigateTo._limit_speed(s, o)


def test_accelerations_follow_the_override_squared():
    client = FakeClient()
    s = skill(client, nominal_accel=NOMINAL)
    for o in (1.0, 0.5, 0.1):
        limit(s, o)
    prefix = 'FollowPath.'
    assert client.requests[0] == {prefix + 'ax_max': 2.0, prefix + 'ax_min': -2.0, prefix + 'az_max': 2.0}
    assert client.requests[1] == {prefix + 'ax_max': 0.5, prefix + 'ax_min': -0.5, prefix + 'az_max': 0.5}
    assert all(abs(client.requests[2][prefix + k] - v * 0.01) < 1e-9 for k, v in NOMINAL.items())
    assert [x['speed_limit'] for x in s.sent] == [0.0, 50.0, 10.0]


def test_nominal_is_set_explicitly_at_100_percent_once():
    client = FakeClient()
    s = skill(client, nominal_accel=NOMINAL)
    limit(s, 1.0)
    limit(s, 1.0)
    assert len(client.requests) == 1 and client.requests[0]['FollowPath.ax_max'] == 2.0


def test_restore_on_stop_and_on_failure():
    client = FakeClient()
    s = skill(client, nominal_accel=NOMINAL)
    limit(s, 0.5)
    limit(s, 1.0)   # what stop() and the failure paths of execute() call
    assert client.requests[-1]['FollowPath.ax_max'] == 2.0 and s.sent[-1]['speed_limit'] == 0.0


def test_nothing_is_touched_without_nominal_accel():
    client = FakeClient()
    s = skill(client)
    limit(s, 0.5)
    assert client.requests == []


def test_unavailable_parameter_service_warns_once_and_retries():
    client = FakeClient(ready=False)
    s = skill(client, nominal_accel=NOMINAL)
    limit(s, 0.5)
    limit(s, 0.5)
    assert client.requests == [] and len(s.log.warnings) == 1
    client.ready = True
    limit(s, 0.5)
    assert len(client.requests) == 1


def test_rejected_parameters_warn_and_retry():
    client = FakeClient(ok=False)
    s = skill(client, nominal_accel=NOMINAL)
    limit(s, 0.5)
    limit(s, 0.5)
    assert len(client.requests) == 2 and s.log.warnings


def test_speed_limit_off_sends_only_nominal():
    client = FakeClient()
    s = skill(client, speed_limit=False, nominal_accel=NOMINAL)
    limit(s, 0.5)
    assert client.requests == [{'FollowPath.ax_max': 2.0, 'FollowPath.ax_min': -2.0, 'FollowPath.az_max': 2.0}]
    assert s.sent == [{'percentage': True, 'speed_limit': 0.0}]


def test_the_override_goes_to_nav2_as_a_percentage():
    s = skill()
    Nav2NavigateTo._limit_speed(s, 0.5)
    Nav2NavigateTo._limit_speed(s, 1.0)
    assert s.sent == [{'percentage': True, 'speed_limit': 50.0}, {'percentage': True, 'speed_limit': 0.0}]


def test_time_allowances_stretch_by_one_over_the_override():
    client = FakeClient()   # at 10 % a drive takes 10x as long: its stuck check must wait 10x as long
    s = skill(client, nominal_time={'progress_checker.movement_time_allowance': 20.0})
    for o in (1.0, 0.5, 0.1):
        limit(s, o)
    assert [r['progress_checker.movement_time_allowance'] for r in client.requests] == [20.0, 40.0, 200.0]


def test_time_allowances_alone_create_the_parameter_requests():
    client = FakeClient()
    s = skill(client, nominal_time={'progress_checker.movement_time_allowance': 20.0}, nominal_accel={})
    limit(s, 0.25)
    assert client.requests == [{'progress_checker.movement_time_allowance': 80.0}]
