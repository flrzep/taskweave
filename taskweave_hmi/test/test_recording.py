"""Record tab backend: options, folders, files and byte ranges (no ROS, no simulator)."""

import json
import threading
from datetime import datetime

import pytest

from taskweave_hmi import recording
from taskweave_hmi.recording import Recorder, RecordingError, byte_range, options, recording_id


class FakeBridge:
    def __init__(self, state='idle'):
        self.lock = threading.Lock()
        self.status = {'state': state, 'cycle': 0}
        self.plan = None

    def check_idle(self):
        if self.status['state'] in ('running', 'paused'):
            raise RuntimeError('busy')


def test_options_defaults_and_checks():
    opts = options({}, ['frame_cycle'])
    assert opts == {'mission': '', 'name': '', 'cycles': None, 'reset': True, 'bag': True, 'camera': True,
                    'video': True, 'quality': 'ray_trace', 'max_min': 60.0}
    assert options({'mission': 'frame_cycle', 'cycles': '2'}, ['frame_cycle'])['cycles'] == 2
    for body, text in (({'mission': 'nope'}, 'no mission'), ({'cycles': 1}, 'needs a mission'),
                       ({'mission': 'frame_cycle', 'cycles': 0.5}, None), ({'name': 'a b'}, 'name'),
                       ({'quality': 'ultra'}, 'quality'), ({'max_min': 0}, None), ({'colour': 1}, 'unknown')):
        if text is None:
            continue
        with pytest.raises(RecordingError, match=text):
            options(body, ['frame_cycle'])
    with pytest.raises(RecordingError, match='time limit'):
        options({'max_min': 100000}, [])


def test_recording_id():
    now = datetime(2026, 10, 16, 13, 4, 24)
    assert recording_id(options({'mission': 'm', 'name': 'one'}, ['m']), now) == '20261016_130424_one_m'
    assert recording_id(options({}, []), now) == '20261016_130424'


@pytest.mark.parametrize('header, expected', [
    (None, None), ('bytes=0-99', (0, 99)), ('bytes=100-', (100, 999)), ('bytes=-10', (990, 999)),
    ('bytes=900-5000', (900, 999)), ('items=0-1', None)])
def test_byte_range(header, expected):
    assert byte_range(header, 1000) == expected


def test_byte_range_outside_the_file():
    with pytest.raises(ValueError):
        byte_range('bytes=2000-', 1000)


def make(tmp_path, rec_id, meta=None, video=None, files=()):
    folder = tmp_path / rec_id
    folder.mkdir()
    if meta is not None:
        (folder / 'meta.json').write_text(json.dumps(meta))
    if video is not None:
        (folder / 'status.json').write_text(json.dumps(video))
    for name in files:
        (folder / name).write_bytes(b'x' * 10)
    return folder


def test_list_file_and_delete(tmp_path):
    make(tmp_path, '20261016_120000_a', {'mission': 'm', 'outcome': 'mission succeeded'},
         {'state': 'done', 'frames': 30, 'first_time': 0.0, 'last_time': 1.0}, ['fixed.mp4'])
    make(tmp_path, '20261016_130000_b')
    (tmp_path / 'control.json').write_text('{}')
    recorder = Recorder(FakeBridge(), str(tmp_path))
    listed = recorder.list()
    assert [r['id'] for r in listed] == ['20261016_130000_b', '20261016_120000_a']  # newest first, folders only
    assert listed[1]['files'] == ['fixed.mp4', 'meta.json', 'status.json'] and listed[1]['frames'] == 30
    assert recorder.file('20261016_120000_a', 'fixed.mp4').read_bytes() == b'x' * 10
    for rec_id, name in (('20261016_120000_a', '../meta.json'), ('..', 'fixed.mp4'), ('20261016_130000_b', 'fixed.mp4'),
                         ('20261016_120000_a', 'bag')):
        with pytest.raises(RecordingError):
            recorder.file(rec_id, name)
    recorder.delete('20261016_130000_b')
    assert [r['id'] for r in recorder.list()] == ['20261016_120000_a']
    with pytest.raises(RecordingError):
        recorder.delete('20261016_130000_b')


def test_start_refuses_a_second_recording_and_a_running_mission(tmp_path, monkeypatch):
    started = []
    monkeypatch.setattr(Recorder, '_run', lambda self, rec, folder: started.append(folder))
    recorder = Recorder(FakeBridge(), str(tmp_path / 'recordings'))
    rec_id = recorder.start({'name': 'x', 'reset': False}, [])['id']
    assert started and started[0].name == rec_id and started[0].is_dir()
    with pytest.raises(RecordingError, match='running'):
        recorder.start({'reset': False}, [])
    with pytest.raises(RecordingError, match='stop the recording first'):
        recorder.delete(rec_id)
    assert recorder.stop() == {'id': rec_id} and recorder.active['stop']
    busy = Recorder(FakeBridge('running'), str(tmp_path / 'other'))
    with pytest.raises(RuntimeError, match='busy'):
        busy.start({'reset': True}, [])


def test_wait_stops_after_the_cycles(tmp_path, monkeypatch):
    bridge = FakeBridge('running')
    recorder = Recorder(bridge, str(tmp_path))
    aborted = []
    monkeypatch.setattr(Recorder, '_abort_mission', lambda self: aborted.append(True))
    monkeypatch.setattr(recording.time, 'sleep', lambda _s: bridge.status.update(cycle=bridge.status['cycle'] + 1))
    rec = {'id': 'r', 'stop': False, 'video': False,
           'options': options({'mission': 'm', 'cycles': 2, 'video': False}, ['m'])}
    assert recorder._wait(rec) == '2 cycles done' and aborted == [True]
    bridge.status = {'state': 'running', 'cycle': 1}
    monkeypatch.setattr(recording.time, 'sleep', lambda _s: bridge.status.update(state='failed'))
    assert recorder._wait({**rec, 'options': {**rec['options'], 'cycles': None}}) == 'mission failed'
