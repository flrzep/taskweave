"""Record tab backend: options, folders, files and byte ranges (no ROS, no simulator)."""

import json
import threading
from datetime import datetime

import pytest

from taskweave_hmi import recording
from taskweave_hmi.recording import Recorder, RecordingError, byte_range, options, recording_id


class FakeLogger:
    def info(self, _text):
        pass

    warning = info


class FakeBridge:
    def __init__(self, state='idle'):
        self.lock = threading.Lock()
        self.status = {'state': state, 'cycle': 0}
        self.plan = None

    def get_logger(self):
        return FakeLogger()

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
         {'state': 'done', 'frames': 30, 'first_time': 0.0, 'last_time': 1.0},
         ['20261016_120000_a_follow.mp4', '20261016_120000_a_fixed.mp4', 'notes.txt'])
    make(tmp_path, '20261016_130000_b')
    (tmp_path / 'control.json').write_text('{}')
    recorder = Recorder(FakeBridge(), str(tmp_path))
    listed = recorder.recordings()
    assert [r['id'] for r in listed] == ['20261016_130000_b', '20261016_120000_a']  # newest first, folders only
    assert listed[1]['files'] == ['20261016_120000_a_fixed.mp4', '20261016_120000_a_follow.mp4', 'meta.json',
                                  'status.json'] and listed[1]['frames'] == 30
    assert recorder.file('20261016_120000_a', '20261016_120000_a_fixed.mp4').read_bytes() == b'x' * 10
    for rec_id, name in (('20261016_120000_a', '../meta.json'), ('..', 'fixed.mp4'), ('20261016_130000_b', 'fixed.mp4'),
                         ('20261016_120000_a', 'bag'), ('20261016_120000_a', 'notes.txt'),
                         ('20261016_120000_a', 'a/b_fixed.mp4')):
        with pytest.raises(RecordingError):
            recorder.file(rec_id, name)
    recorder.delete('20261016_130000_b')
    assert [r['id'] for r in recorder.recordings()] == ['20261016_120000_a']
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


def test_a_new_recording_replaces_the_old_ones_except_kept(tmp_path, monkeypatch):
    monkeypatch.setattr(Recorder, '_run', lambda self, rec, folder: None)
    for rec_id in ('20251016_100000_a', '20251016_110000_b', '20251016_120000_c'):
        make(tmp_path, rec_id, files=['x_fixed.mp4'])
    recorder = Recorder(FakeBridge(), str(tmp_path), keep=2)
    assert recorder.set_keep('20251016_100000_a', True) is True
    started = recorder.start({'reset': False}, [])
    assert started['deleted'] == ['20251016_110000_b']  # newest kept (keep 2 = it and the new one), marked one stays
    names = [r['id'] for r in recorder.recordings()]
    assert names == [started['id'], '20251016_120000_c', '20251016_100000_a']
    assert [r['keep'] for r in recorder.recordings()] == [False, False, True]
    recorder.active = None
    recorder.keep = 1
    started = recorder.start({'reset': False, 'name': 'z'}, [])
    assert set(started['deleted']) == {'20251016_120000_c', names[0]}
    recorder.active = None
    recorder.set_keep('20251016_100000_a', False)
    recorder.keep = 0  # keep all
    assert recorder.start({'reset': False, 'name': 'y'}, [])['deleted'] == []


def test_no_start_without_free_disk(tmp_path, monkeypatch):
    monkeypatch.setattr(Recorder, '_run', lambda self, rec, folder: None)
    recorder = Recorder(FakeBridge(), str(tmp_path), min_free_gb=1e9)
    with pytest.raises(RecordingError, match='GB free'):
        recorder.start({'reset': False}, [])
    assert recorder.active is None


def test_files_lists_videos_first_and_only_known_kinds(tmp_path):
    names = ['20261016_120000_a_head_camera_color.mp4', '20261016_120000_a_follow.mp4', '20261016_120000_a_fixed.mp4',
             'cam.yaml', 'post.log', 'raw.bin', '.keep']
    folder = make(tmp_path, '20261016_120000_a', {}, {}, names)
    assert recording.files(folder) == ['20261016_120000_a_fixed.mp4', '20261016_120000_a_follow.mp4',
                                       '20261016_120000_a_head_camera_color.mp4', 'cam.yaml', 'meta.json',
                                       'post.log', 'status.json']


def test_post_processing_runs_on_the_folder(tmp_path):
    folder = make(tmp_path, '20261016_120000_a')
    recorder = Recorder(FakeBridge(), str(tmp_path), post='sh -c "echo done > {dir}/out.csv"')
    rec = {'id': folder.name, 'problems': []}
    recorder._post_process(rec, folder)
    assert (folder / 'out.csv').read_text() == 'done\n' and rec['problems'] == []
    recorder.post = 'sh -c "exit 3"'
    recorder._post_process(rec, folder)
    assert 'exit 3' in rec['problems'][0]


def test_a_light_recording_deletes_nothing(tmp_path, monkeypatch):
    monkeypatch.setattr(Recorder, '_run', lambda self, rec, folder: None)
    make(tmp_path, '20251016_100000_a', files=['x_fixed.mp4'])
    recorder = Recorder(FakeBridge(), str(tmp_path), keep=1)
    assert recorder.start({'reset': False, 'bag': False, 'video': False}, [])['deleted'] == []
    assert len(recorder.recordings()) == 2


def test_step_events_and_plan_ids_go_into_the_status_and_meta(tmp_path, monkeypatch):
    monkeypatch.setattr(Recorder, '_run', lambda self, rec, folder: None)
    recorder = Recorder(FakeBridge(), str(tmp_path))
    recorder.episode({'event': 'step_started', 'plan_id': 'm-0000'})  # before the recording: not kept
    rec_id = recorder.start({'reset': False, 'bag': False, 'video': False, 'mission': 'm'}, ['m'])['id']
    events = [{'event': 'step_started', 'plan_id': 'm-0001', 'step_id': 's1', 'seq': 7},
              {'event': 'step_finished', 'plan_id': 'm-0001', 'step_id': 's1', 'seq': 8},
              {'event': 'step_started', 'plan_id': 'm-0002', 'step_id': 's1', 'seq': 9}]
    for event in events:
        recorder.episode(event)
    status = recorder.status()
    assert status['plan_ids'] == ['m-0001', 'm-0002'] and status['events'] == 3
    recorder._finish(recorder.active, tmp_path / rec_id, None)
    meta = json.loads((tmp_path / rec_id / 'meta.json').read_text())
    assert meta['plan_ids'] == ['m-0001', 'm-0002'] and meta['events'] == events
    recorder.episode({'event': 'step_started', 'plan_id': 'm-0003'})  # after it: not kept
    assert recorder.status()['plan_ids'] == ['m-0001', 'm-0002']
