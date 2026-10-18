"""Mission smoke test (taskweave_core.smoke): events -> step results, comparison, the run loop against a fake HMI."""

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest
import yaml

from taskweave_core import cli, smoke


def episode(kind, step_id, time, cycle=1, attempt=1, plan_id='frame_cycle-0001', **extra):
    skill = {'s1': 'navigate_to', 's2': 'move_joints', 's3': 'move_gripper'}.get(step_id, 'wait')
    return {'event': kind, 'time': time, 'plan_id': plan_id, 'cycle': cycle, 'step_id': step_id, 'skill': skill,
            'instruction': f'do {step_id}', 'args': {}, 'attempt': attempt, **extra}


def done(step_id, time, outcome='SUCCEEDED', message='', **kw):
    return episode('step_finished', step_id, time, outcome=outcome, message=message, **kw)


# /api/state as the HMI serves it (status, plan, outcomes, events), mid-mission
STATE = {
    'robot': 'galbot_s1', 'pad_mode': 'drive',
    'status': {'state': 'running', 'plan_id': 'frame_cycle-0001', 'goal': '', 'cycle': 1, 'step_index': 2,
               'step_count': 3, 'step': None, 'attempt': 1, 'step_status': 'dispatched', 'step_progress': -1.0,
               'step_elapsed_s': 0.4, 'last_error': ''},
    'plan': {'plan_id': 'frame_cycle-0001', 'goal': '', 'robot': 'galbot_s1', 'loop': '', 'steps': []},
    'outcomes': {},
    'events': [
        episode('step_started', 's1', 10.0),
        done('s1', 22.5, message='reached bay_2'),
        episode('step_started', 's2', 22.6),
        done('s2', 23.4, message='at ready (max error 0.005)'),
        episode('step_started', 's3', 23.5),
        done('s3', 25.0, outcome='FAILED', message='no plan', attempt=1),
        episode('step_started', 's3', 25.1, attempt=2),
        done('s3', 27.0, message='moved left/rails_top (max error 1.2e-3)', attempt=2),
    ]}


def test_step_results_from_state_events():
    steps = smoke.step_results(STATE['events'])
    assert [s['step_id'] for s in steps] == ['s1', 's2', 's3']
    assert steps[0] == {'cycle': 1, 'step_id': 's1', 'skill': 'navigate_to', 'instruction': 'do s1',
                        'outcome': 'SUCCEEDED', 'duration_s': 12.5, 'max_error': None, 'attempts': 1,
                        'message': 'reached bay_2'}
    assert steps[1]['duration_s'] == pytest.approx(0.8) and steps[1]['max_error'] == 0.005
    assert steps[2]['attempts'] == 2 and steps[2]['outcome'] == 'SUCCEEDED'  # the last attempt counts
    assert steps[2]['duration_s'] == pytest.approx(3.5) and steps[2]['max_error'] == pytest.approx(0.0012)


def test_step_results_skip_later_cycles_and_unfinished_starts():
    events = [done('s0', 5.0)] + STATE['events'][:2] + [episode('step_started', 's1', 30.0, cycle=2)]
    steps = smoke.step_results(events, cycles=1)
    assert [(s['cycle'], s['step_id']) for s in steps] == [(1, 's1')]  # s0 started before the run
    running = smoke.step_results(STATE['events'][:3])
    assert running[-1]['outcome'] == 'RUNNING' and running[-1]['duration_s'] is None


def ref_run(**changes):
    steps = [{'cycle': 1, 'step_id': 's1', 'skill': 'navigate_to', 'instruction': 'drive', 'outcome': 'SUCCEEDED',
              'duration_s': 12.5, 'max_error': None, 'attempts': 1, 'message': ''},
             {'cycle': 1, 'step_id': 's2', 'skill': 'move_joints', 'instruction': 'arms up', 'outcome': 'SUCCEEDED',
              'duration_s': 0.8, 'max_error': 0.005, 'attempts': 1, 'message': ''},
             {'cycle': 1, 'step_id': 's3', 'skill': 'move_gripper', 'instruction': 'reach', 'outcome': 'SUCCEEDED',
              'duration_s': 4.0, 'max_error': 0.002, 'attempts': 1, 'message': ''}]
    for step_id, values in changes.items():
        next(s for s in steps if s['step_id'] == step_id).update(values)
    return {'mission': 'frame_cycle', 'outcome': '1 cycle done', 'steps': steps}


def results(comparison):
    return [(row['step_id'], row['result']) for row in comparison['rows']]


def test_compare_pass_within_tolerance_and_slack():
    run = ref_run(s1={'duration_s': 13.6}, s2={'duration_s': 1.3, 'max_error': 0.0065}, s3={'duration_s': 3.7})
    comparison = smoke.compare(ref_run(), run)
    assert comparison['passed'] and results(comparison) == [('s1', 'PASS'), ('s2', 'PASS'), ('s3', 'PASS')]
    assert comparison['rows'][0]['delta_pct'] == pytest.approx(8.8)


def test_compare_slow_and_fast_steps_fail():
    comparison = smoke.compare(ref_run(), ref_run(s1={'duration_s': 15.0}, s3={'duration_s': 2.0}))
    assert not comparison['passed'] and results(comparison) == [('s1', 'FAIL'), ('s2', 'PASS'), ('s3', 'FAIL')]
    assert 'slower' in comparison['rows'][0]['note'] and 'faster' in comparison['rows'][2]['note']
    assert smoke.compare(ref_run(), ref_run(s1={'duration_s': 15.0}), tolerance=0.25)['passed']


def test_compare_failed_step():
    comparison = smoke.compare(ref_run(), ref_run(s2={'outcome': 'FAILED', 'message': 'joint limit'}))
    assert results(comparison)[1] == ('s2', 'FAIL') and 'FAILED: joint limit' in comparison['rows'][1]['note']
    assert comparison['failed'] == 1


def test_compare_missing_and_extra_steps():
    run = ref_run()
    run['steps'] = [run['steps'][0], {**run['steps'][1], 'step_id': 'recover'}, run['steps'][2]]
    comparison = smoke.compare(ref_run(), run)
    assert results(comparison) == [('s1', 'PASS'), ('s2', 'MISSING'), ('recover', 'EXTRA'), ('s3', 'PASS')]
    assert not comparison['passed']
    allowed = smoke.compare(ref_run(), run, allow_extra=True)
    assert results(allowed)[2] == ('recover', 'PASS') and not allowed['passed']  # s2 is still missing
    short = ref_run()
    short['steps'] = short['steps'][:1]
    assert results(smoke.compare(ref_run(), short))[1:] == [('s2', 'MISSING'), ('s3', 'MISSING')]


def test_compare_error_growth():
    assert smoke.compare(ref_run(), ref_run(s3={'max_error': 0.0041}))['passed']  # <= 0.002 * 1.1 + 0.002
    comparison = smoke.compare(ref_run(), ref_run(s3={'max_error': 0.006}))
    assert results(comparison)[2] == ('s3', 'FAIL') and 'error 0.006 > 0.002' in comparison['rows'][2]['note']


def test_table_and_compare_command(tmp_path, capsys):
    reference, run = tmp_path / 'cycle.reference.yaml', tmp_path / 'run.yaml'
    smoke.save(str(reference), ref_run())
    smoke.save(str(run), ref_run(s1={'duration_s': 15.0}))
    assert cli.main(['smoke', 'compare', str(reference), str(run)]) == 1
    out = capsys.readouterr().out
    assert 'FAILED: 1 FAIL, 2 PASS' in out and '+20.0' in out
    report = yaml.safe_load((tmp_path / 'cycle.report.yaml').read_text())
    assert report['failed'] == 1 and report['run']['steps'][0]['duration_s'] == 15.0
    # a report can stand in for a run
    assert cli.main(['smoke', 'compare', str(reference), str(tmp_path / 'cycle.report.yaml'),
                     '--tolerance', '0.3', '--report', str(tmp_path / 'r2.yaml')]) == 0


# ---- the run loop against a fake HMI --------------------------------------------------
class FakeHmi:
    """Imitates /api/state, /api/recordings and /api/recordings/start: each recordings poll is one tick of a script
    (events to add, recording phase); the latest 50 events are shown, like the HMI."""

    def __init__(self, script, old_events=()):
        self.script, self.tick, self.started = script, -1, None
        self.events = list(old_events)
        self.recording = {'active': False}
        self.posts = []
        self.meta = None  # the recording's meta.json (newer HMIs), None: 404 like older ones

    def handle(self, method, path, body):
        if method == 'POST':
            self.posts.append((path, body))
        if method == 'GET' and path == '/api/state':
            return 200, {**STATE, 'events': self.events[-50:]}
        if method == 'POST' and path == '/api/recordings/start':
            if body.get('mission') != 'frame_cycle':
                return 400, {'error': f'no mission {body.get("mission")!r}'}
            self.started = body
            self.recording = {'id': '20261016_120000_smoke_frame_cycle', 'options': body, 'phase': 'starting',
                              'active': True, 'outcome': '', 'problems': []}
            return 200, {'id': self.recording['id'], 'deleted': []}
        if method == 'GET' and path == '/api/recordings':
            if self.started and self.tick + 1 < len(self.script):
                self.tick += 1
                events, phase = self.script[self.tick]
                self.events.extend(events)
                self.recording['phase'] = phase
                if phase == 'done':
                    self.recording.update(active=False, outcome='1 cycle done')
            return 200, {'status': self.recording, 'recordings': []}
        if method == 'POST' and path in ('/api/recordings/stop', '/api/abort'):
            return 200, {}
        if method == 'GET' and self.meta is not None and path == f'/recordings/{self.recording.get("id")}/meta.json':
            return 200, self.meta
        return 404, {'error': 'not found'}


@pytest.fixture
def fake_hmi():
    servers = []

    def start(fake):
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _reply(self, method):
                length = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(length) or b'{}') if method == 'POST' else None
                status, data = fake.handle(method, self.path, body)
                payload = json.dumps(data).encode()
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(payload)))
                self.end_headers()
                self.wfile.write(payload)

            def do_GET(self):  # noqa: N802
                self._reply('GET')

            def do_POST(self):  # noqa: N802
                self._reply('POST')

        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        servers.append(server)
        return f'http://127.0.0.1:{server.server_address[1]}'

    yield start
    for server in servers:
        server.shutdown()
        server.server_close()


SCRIPT = [
    ([], 'resetting the sim'),
    ([], 'starting frame_cycle'),
    ([], 'recording'),
    (STATE['events'][:3], 'recording'),
    (STATE['events'][3:], 'recording'),
    ([episode('step_started', 's1', 40.0, cycle=2, plan_id='frame_cycle-0002'),
      done('s1', 41.0, outcome='CANCELED', cycle=2, plan_id='frame_cycle-0002')], 'finishing'),
    ([], 'done'),
]


def test_run_follows_the_recording(fake_hmi):
    old = [episode('step_started', 's1', 3.0, plan_id='frame_cycle-0000'), done('s1', 9.0, plan_id='frame_cycle-0000')]
    fake = FakeHmi(SCRIPT, old_events=old)
    lines = []
    run = smoke.run(smoke.Hmi(fake_hmi(fake)), 'frame_cycle', poll_s=0.0, log=lines.append)
    assert fake.started == {'mission': 'frame_cycle', 'name': 'smoke', 'cycles': 1, 'reset': True, 'bag': False,
                            'camera': False, 'video': False, 'max_min': 60.0}
    assert run['outcome'] == '1 cycle done' and run['recording'] == '20261016_120000_smoke_frame_cycle'
    assert [(s['step_id'], s['duration_s']) for s in run['steps']] == [('s1', 12.5), ('s2', 0.8), ('s3', 3.5)]
    assert '  resetting the sim' in lines and smoke.run_ok(run)


def test_run_takes_the_recordings_own_events_when_it_has_them(fake_hmi):
    fake = FakeHmi(SCRIPT)
    fake.meta = {'events': STATE['events'][:2]}  # e.g. events the 50-event window lost: the recording's list wins
    run = smoke.run(smoke.Hmi(fake_hmi(fake)), 'frame_cycle', poll_s=0.0, log=lambda line: None)
    assert [s['step_id'] for s in run['steps']] == ['s1']


def test_run_command_saves_reference_and_compares(fake_hmi, tmp_path, capsys):
    url, reference = fake_hmi(FakeHmi(SCRIPT)), tmp_path / 'frame_cycle.reference.yaml'
    assert cli.main(['smoke', 'run', 'frame_cycle', '--hmi', url, '--poll', '0', '--save-reference',
                     str(reference)]) == 0
    assert [s['step_id'] for s in yaml.safe_load(reference.read_text())['steps']] == ['s1', 's2', 's3']
    url = fake_hmi(FakeHmi(SCRIPT))
    assert cli.main(['smoke', 'run', 'frame_cycle', '--hmi', url, '--poll', '0', '--reference', str(reference),
                     '--no-reset']) == 0
    out = capsys.readouterr().out
    assert 'PASSED: 3 PASS' in out and (tmp_path / 'frame_cycle.report.yaml').exists()


def test_run_reports_a_rejected_start(fake_hmi, capsys):
    url = fake_hmi(FakeHmi(SCRIPT))
    assert cli.main(['smoke', 'run', 'nope', '--hmi', url, '--poll', '0']) == 2
    assert "no mission 'nope'" in capsys.readouterr().out
    assert cli.main(['smoke', 'run', 'x', '--hmi', 'http://127.0.0.1:9', '--poll', '0']) == 2
