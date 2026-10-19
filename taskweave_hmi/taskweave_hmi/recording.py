"""Recordings from the HMI (Record tab): a ROS bag and, when the simulator has one, videos, optionally of a mission.

One recording at a time, in its own folder ``<recordings_dir>/<id>/``:
  bag/          ROS bag (mcap, sim time), bag.log
  <id>_fixed.mp4, <id>_follow.mp4, frames.csv, status.json   from the video recorder, if one answers
  meta.json     what was recorded: mission, options, outcome, times, the plan, plan_ids and the step events
                (step_started / step_finished, as on /api/state) from the recording's start to its end
  ...           whatever the post-processing command writes (e.g. camera videos from the bag), post.log

Recordings are big (camera images: ~0.15 GB per sim second), so they do not pile up: starting one deletes the older
ones beyond the newest ``keep - 1`` (keep 1: each recording replaces the last), except those marked keep (a ``.keep``
file in the folder). It does not start with less than ``min_free_gb`` free. A light recording (no bag, no video: only
meta.json, e.g. a smoke test) deletes only older light ones beyond the newest ``LIGHT_KEEP - 1``; marked ones always
stay.

Video recorder contract (frame-sim: scripts/recorder.py): the HMI writes ``<recordings_dir>/control.json``
``{"id", "cmd": "start"|"stop", "dir", "options"}``; the recorder answers in ``<dir>/status.json`` with ``state``
capturing | stopping | encoding | done | error (and frames, first_time, last_time in sim seconds). It steps the sim
itself while capturing, so the sim runs slower than real time. Without an answer the recording is a bag only.

"From the start" resets the simulation (simulation_interfaces SetSimulationState stop, play) and waits for the
navigation stack to come back before the bag and the mission start.
"""

from __future__ import annotations

import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

ID_RE = re.compile(r'^[A-Za-z0-9_.-]+$')
SUFFIXES = ('.mp4', '.csv', '.json', '.yaml', '.log')  # files of a recording the HMI lists and serves
VIEWS = ('fixed', 'follow')  # the simulator's videos (<id>_fixed.mp4), listed first
KEEP_FILE = '.keep'
POST_TIMEOUT_S = 3600.0
LIGHT_KEEP = 20  # light recordings kept (a few KB each): they only clutter the list
END_STATES = ('succeeded', 'failed', 'aborted', 'idle')
QUALITY = {'ray_trace': 'RAY_TRACE', 'path_trace': 'PATH_TRACE'}
BAG_EXCLUDE = (r'.*/_action/.*|/rosout|/parameter_events|.*/transition_event|.*/bond|.*costmap_raw.*|.*_updates'
               r'|.*static_layer.*|/diagnostics|/behavior_tree_log|.*/preview/.*')
CAMERA_EXCLUDE = r'.*image_raw.*|.*/compressed.*|.*/points'  # camera images and point clouds: most of a bag's size
NAV_ACTIVE = '/lifecycle_manager_navigation/is_active'
SIM_STATE = '/set_simulation_state'
VIDEO_ANSWER_S = 30.0
VIDEO_FINISH_S = 1800.0  # encoding a long path-traced video
BAG_START_S = 20.0
NAV_BACK_S = 120.0
NAV_DOWN_S = 20.0  # after a sim reset Nav2 goes inactive within this, or it was not reset


class RecordingError(Exception):
    """A recording request that cannot be done (bad options, one already running, unknown recording)."""


def check_id(name: str) -> str:
    if not ID_RE.match(name or '') or name.startswith('.'):
        raise RecordingError(f'invalid recording name {name!r}')
    return name


def options(body: dict, missions: list[str]) -> dict:
    """The options of a start request, checked; defaults for the rest."""
    known = {'mission', 'name', 'cycles', 'reset', 'bag', 'camera', 'video', 'quality', 'max_min'}
    unknown = set(body) - known
    if unknown:
        raise RecordingError(f'unknown options: {", ".join(sorted(unknown))}')
    mission = str(body.get('mission') or '')
    if mission and mission not in missions:
        raise RecordingError(f'no mission {mission!r}')
    name = str(body.get('name') or '').strip()
    if name and not ID_RE.match(name):
        raise RecordingError('name: letters, digits, _ . - only')
    cycles = body.get('cycles')
    if cycles in (None, '', 0):
        cycles = None
    else:
        try:
            cycles = int(cycles)
        except (TypeError, ValueError):
            raise RecordingError('cycles must be a whole number') from None
        if cycles < 1:
            raise RecordingError('cycles must be at least 1')
        if not mission:
            raise RecordingError('cycles needs a mission')
    quality = str(body.get('quality') or 'ray_trace')
    if quality not in QUALITY:
        raise RecordingError(f'quality: one of {", ".join(QUALITY)}')
    try:
        max_min = float(body.get('max_min') or 60)
    except (TypeError, ValueError):
        raise RecordingError('time limit must be a number') from None
    if not 0.5 <= max_min <= 24 * 60:
        raise RecordingError('time limit: 0.5 to 1440 minutes')
    flag = lambda key, default: bool(body.get(key, default))  # noqa: E731
    return {'mission': mission, 'name': name, 'cycles': cycles, 'reset': flag('reset', True), 'bag': flag('bag', True),
            'camera': flag('camera', True), 'video': flag('video', True), 'quality': quality, 'max_min': max_min}


def recording_id(opts: dict, now: datetime) -> str:
    return '_'.join(part for part in (now.strftime('%Y%m%d_%H%M%S'), opts['name'], opts['mission']) if part)


def folder_size(path: Path) -> int:
    total = 0
    for root, _dirs, files in os.walk(path):
        for file in files:
            try:
                total += os.stat(os.path.join(root, file)).st_size
            except OSError:
                pass
    return total


def files(folder: Path) -> list[str]:
    """The recording's videos (the simulator's fixed and follow views first) and data files."""
    def order(name: str):
        stem = name.rsplit('.', 1)[0]
        view = next((i for i, v in enumerate(VIEWS) if stem == v or stem.endswith(f'_{v}')), len(VIEWS))
        return (not name.endswith('.mp4'), view, name)
    return sorted((p.name for p in folder.iterdir() if p.is_file() and p.suffix in SUFFIXES and ID_RE.match(p.name)),
                  key=order)


def is_light(folder: Path) -> bool:
    """A finished recording without bag and video (its meta.json says so)."""
    opts = (read_json(folder / 'meta.json') or {}).get('options') or {}
    return opts.get('bag') is False and opts.get('video') is False


def read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def byte_range(header: str | None, size: int) -> tuple[int, int] | None:
    """(first, last) byte of an HTTP Range header ``bytes=a-b`` / ``bytes=a-`` / ``bytes=-n``; None: the whole file.
    ValueError when it cannot be satisfied."""
    if not header:
        return None
    match = re.fullmatch(r'bytes=(\d*)-(\d*)', header.strip())
    if not match or match.groups() == ('', ''):
        return None
    first, last = match.groups()
    if first == '':
        start, end = max(size - int(last), 0), size - 1
    else:
        start, end = int(first), min(int(last), size - 1) if last else size - 1
    if start > end or start >= size:
        raise ValueError(f'range {header} outside {size} bytes')
    return start, end


class Recorder:
    """Runs one recording at a time in a thread; ``bridge`` is the HMI's ROS node (server.Bridge)."""

    def __init__(self, bridge, directory: str, bag_exclude: str = BAG_EXCLUDE, keep: int = 1,
                 min_free_gb: float = 30.0, post: str = ''):
        self.bridge, self.dir, self.bag_exclude = bridge, Path(directory), bag_exclude
        self.keep, self.min_free_gb, self.post = int(keep), float(min_free_gb), post
        self.lock = threading.Lock()
        self.active: dict | None = None
        self.last: dict | None = None  # the latest finished recording, shown until the next one starts
        self.sim_client = self.nav_client = None

    # ---- requests ---------------------------------------------------------------------
    def start(self, body: dict, missions: list[str]) -> dict:
        opts = options(body, missions)
        with self.lock:
            if self.active:
                raise RecordingError(f'recording {self.active["id"]} is running')
            if opts['mission'] or opts['reset']:
                self.bridge.check_idle()
            rec_id = recording_id(opts, datetime.now())
            folder = self.dir / rec_id
            if folder.exists():
                raise RecordingError(f'{rec_id} exists already')
            self.dir.mkdir(parents=True, exist_ok=True)
            deleted = self._prune() if opts['bag'] or opts['video'] else self._prune(light=True)
            free = shutil.disk_usage(self.dir).free / 1e9
            if free < self.min_free_gb:
                raise RecordingError(f'only {free:.0f} GB free (at least {self.min_free_gb:g} GB needed): delete '
                                     f'recordings marked keep')
            folder.mkdir(mode=0o2775)
            folder.chmod(0o2775)  # the host's group can delete it
            self.active = {'id': rec_id, 'options': opts, 'phase': 'starting', 'started': time.time(), 'stop': False,
                           'outcome': '', 'problems': [], 'video': None, 'bag': None, 'plan_ids': [], 'events': []}
            self.last = None
        threading.Thread(target=self._run, args=(self.active, folder), daemon=True).start()
        return {'id': rec_id, 'deleted': deleted}

    def episode(self, event: dict) -> None:
        """A step event of the task handler (server.Bridge): kept with the running recording."""
        with self.lock:
            rec = self.active
            if rec is None:
                return
            rec['events'].append(event)
            plan_id = event.get('plan_id')
            if plan_id and plan_id not in rec['plan_ids']:
                rec['plan_ids'].append(plan_id)

    def _prune(self, light: bool = False) -> list[str]:
        """Make room for a new recording: delete the older ones beyond the newest keep - 1, except marked ones.
        ``light``: only light recordings, beyond the newest LIGHT_KEEP - 1."""
        keep = LIGHT_KEEP if light else self.keep
        if keep <= 0:
            return []
        folders = [folder for folder in self._folders() if not (folder / KEEP_FILE).exists()]
        if light:
            folders = [folder for folder in folders if is_light(folder)]
        deleted = []
        for folder in folders[keep - 1:]:
            try:
                shutil.rmtree(folder)
                deleted.append(folder.name)
            except OSError as error:
                self.bridge.get_logger().warning(f'recording {folder.name} not deleted: {error}')
        if deleted:
            self.bridge.get_logger().info(f'recordings deleted to make room: {", ".join(deleted)}')
        return deleted

    def _folders(self) -> list[Path]:
        """Recording folders, newest first."""
        if not self.dir.is_dir():
            return []
        return sorted((p for p in self.dir.iterdir()
                       if p.is_dir() and ID_RE.match(p.name) and not p.name.startswith('.')), reverse=True)

    def set_keep(self, rec_id: str, keep: bool) -> bool:
        folder = self.dir / check_id(rec_id)
        if not folder.is_dir():
            raise RecordingError(f'no recording {rec_id}')
        marker = folder / KEEP_FILE
        if keep:
            marker.touch()
        elif marker.exists():
            marker.unlink()
        return keep

    def stop(self) -> dict:
        with self.lock:
            if not self.active:
                raise RecordingError('no recording is running')
            self.active['stop'] = True
            return {'id': self.active['id']}

    def status(self) -> dict:
        with self.lock:
            rec = self.active or self.last
            if rec is None:
                return {'active': False}
            data = {key: rec[key] for key in ('id', 'options', 'phase', 'started', 'outcome', 'problems')}
            data['plan_ids'], data['events'] = list(rec['plan_ids']), len(rec['events'])
            data['active'] = self.active is not None
            data['ended'] = rec.get('ended')
        folder = self.dir / data['id']
        data['video'] = read_json(folder / 'status.json')
        data['bytes'] = folder_size(folder)
        data['wall_s'] = round((data['ended'] or time.time()) - data['started'], 1)
        return data

    def recordings(self) -> list[dict]:
        result, active = [], self.active['id'] if self.active else None
        for folder in self._folders():
            meta = read_json(folder / 'meta.json') or {}
            video = read_json(folder / 'status.json') or {}
            result.append({'id': folder.name, 'mission': meta.get('mission', ''), 'outcome': meta.get('outcome', ''),
                           'options': meta.get('options'), 'started': meta.get('started'), 'ended': meta.get('ended'),
                           'frames': video.get('frames'), 'first_time': video.get('first_time'),
                           'last_time': video.get('last_time'), 'bytes': folder_size(folder),
                           'files': files(folder), 'bag': (folder / 'bag').is_dir(), 'active': folder.name == active,
                           'keep': (folder / KEEP_FILE).exists()})
        return result

    def file(self, rec_id: str, name: str) -> Path:
        check_id(rec_id)
        if not ID_RE.match(name) or Path(name).suffix not in SUFFIXES:
            raise RecordingError(f'no file {name!r}')
        path = self.dir / rec_id / name
        if not path.is_file():
            raise RecordingError(f'{rec_id} has no {name}')
        return path

    def archive(self, rec_id: str) -> tuple[Path, list[tuple[Path, str]]]:
        """A finished recording's folder and its files as (path, name in the archive), every file below it (bag
        included) under ``<id>/``, without the keep marker: what the Record tab's Download all packs into one tar."""
        check_id(rec_id)
        with self.lock:
            if self.active and self.active['id'] == rec_id:
                raise RecordingError('the recording is still running: download it when it has finished')
        folder = self.dir / rec_id
        if not folder.is_dir():
            raise RecordingError(f'no recording {rec_id}')
        members = [(path, f'{rec_id}/{path.relative_to(folder).as_posix()}') for path in sorted(folder.rglob('*'))
                   if path.is_file() and not path.is_symlink() and path.name != KEEP_FILE]
        return folder, members

    def delete(self, rec_id: str) -> None:
        check_id(rec_id)
        with self.lock:
            if self.active and self.active['id'] == rec_id:
                raise RecordingError('stop the recording first')
        folder = self.dir / rec_id
        if not folder.is_dir():
            raise RecordingError(f'no recording {rec_id}')
        shutil.rmtree(folder)
        with self.lock:
            if self.last and self.last['id'] == rec_id:
                self.last = None

    # ---- the recording ------------------------------------------------------------------
    def _phase(self, rec: dict, phase: str) -> None:
        with self.lock:
            rec['phase'] = phase
        self.bridge.get_logger().info(f'recording {rec["id"]}: {phase}')

    def _problem(self, rec: dict, text: str) -> None:
        with self.lock:
            rec['problems'].append(text)
        self.bridge.get_logger().warning(f'recording {rec["id"]}: {text}')

    def _run(self, rec: dict, folder: Path) -> None:
        opts, bag = rec['options'], None
        try:
            if opts['reset']:
                self._phase(rec, 'resetting the sim')
                self._reset_sim()
            if opts['bag']:
                self._phase(rec, 'starting the bag')
                bag = self._start_bag(folder, opts['camera'])
                rec['bag'] = True
            if opts['video']:
                self._phase(rec, 'starting the video')
                rec['video'] = self._start_video(rec, folder)
            if opts['mission']:
                self._phase(rec, f'starting {opts["mission"]}')
                accepted, text = self._start_mission(opts['mission'])
                if not accepted:
                    raise RecordingError(f'{opts["mission"]} not started: {text}')
            self._phase(rec, 'recording')
            rec['outcome'] = self._wait(rec)
        except Exception as error:  # noqa: BLE001 - whatever failed, the recording is closed below
            rec['outcome'] = rec['outcome'] or f'error: {error}'
            self._problem(rec, str(error))
        finally:
            self._finish(rec, folder, bag)

    def _wait(self, rec: dict) -> str:
        opts = rec['options']
        end = time.monotonic() + 60.0 * opts['max_min']
        seen_running = False
        while time.monotonic() < end:
            if rec['stop']:
                return 'stopped by hand'
            if rec['video'] and (read_json(self.dir / rec['id'] / 'status.json') or {}).get('state') == 'error':
                return 'video recorder error'
            if opts['mission']:
                with self.bridge.lock:
                    status = dict(self.bridge.status)
                seen_running = seen_running or status['state'] in ('running', 'paused')
                if seen_running and status['state'] in END_STATES:
                    return f'mission {status["state"]}'
                if seen_running and opts['cycles'] and status.get('cycle', 0) > opts['cycles']:
                    self._abort_mission()
                    return f'{opts["cycles"]} cycle{"s" if opts["cycles"] > 1 else ""} done'
            time.sleep(1.0)
        return 'time limit reached'

    def _finish(self, rec: dict, folder: Path, bag: subprocess.Popen | None) -> None:
        self._phase(rec, 'finishing')
        if rec['video']:
            self._stop_video(rec, folder)
        if bag is not None:
            self._stop_bag(bag)
            if self.post:
                self._phase(rec, 'post-processing')
                self._post_process(rec, folder)
        rec['ended'] = time.time()
        with self.bridge.lock:
            plan = self.bridge.plan
        meta = {'id': rec['id'], 'mission': rec['options']['mission'], 'outcome': rec['outcome'],
                'options': rec['options'], 'problems': rec['problems'],
                'started': datetime.fromtimestamp(rec['started']).isoformat(timespec='seconds'),
                'ended': datetime.fromtimestamp(rec['ended']).isoformat(timespec='seconds'),
                'video': read_json(folder / 'status.json'), 'plan': plan if rec['options']['mission'] else None}
        with self.lock:
            meta['plan_ids'], meta['events'] = list(rec['plan_ids']), list(rec['events'])
        try:
            (folder / 'meta.json').write_text(json.dumps(meta, indent=1))
        except OSError as error:
            self._problem(rec, f'meta.json not written: {error}')
        self._phase(rec, 'done')
        with self.lock:
            self.last, self.active = rec, None

    def _post_process(self, rec: dict, folder: Path) -> None:
        """The post-processing command (parameter recording_post, ``{dir}`` = the folder), e.g. camera videos."""
        command = shlex.split(self.post.replace('{dir}', str(folder)))
        try:
            with open(folder / 'post.log', 'w') as log:
                result = subprocess.run(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                        timeout=POST_TIMEOUT_S, preexec_fn=lambda: os.umask(0o002), check=False)
            if result.returncode:
                self._problem(rec, f'post-processing failed (exit {result.returncode}, post.log)')
        except (OSError, subprocess.TimeoutExpired) as error:
            self._problem(rec, f'post-processing: {error}')

    # ---- sim reset ------------------------------------------------------------------------
    def _reset_sim(self) -> None:
        from simulation_interfaces.msg import SimulationState
        from simulation_interfaces.srv import SetSimulationState
        from std_srvs.srv import Trigger

        node = self.bridge
        if self.sim_client is None:
            self.sim_client = node.create_client(SetSimulationState, SIM_STATE, callback_group=node.group)
            self.nav_client = node.create_client(Trigger, NAV_ACTIVE, callback_group=node.group)
        was_active = self._nav_active()
        for state in (SimulationState.STATE_STOPPED, SimulationState.STATE_PLAYING):
            node.call(self.sim_client, SetSimulationState.Request(state=SimulationState(state=state)))
        if was_active is None:  # no Nav2 lifecycle manager: give the stack a moment
            time.sleep(10.0)
            return
        # the sim's clock jumps back: Nav2 is reset (inactive) and brought up again on the new clock
        start, went_down = time.monotonic(), False
        while time.monotonic() - start < NAV_BACK_S:
            active = self._nav_active()
            went_down = went_down or active is False
            if went_down and active:
                time.sleep(5.0)  # move_group respawns with it
                return
            if not went_down and time.monotonic() - start > NAV_DOWN_S:
                return  # never went down: nothing to wait for
            time.sleep(1.0)
        raise RecordingError(f'navigation not back {NAV_BACK_S:g} s after the sim reset')

    def _nav_active(self) -> bool | None:
        from std_srvs.srv import Trigger

        if self.nav_client is None or not self.nav_client.wait_for_service(timeout_sec=1.0):
            return None
        try:
            return bool(self.bridge.call(self.nav_client, Trigger.Request()).success)
        except TimeoutError:
            return False

    # ---- bag ----------------------------------------------------------------------------
    def _start_bag(self, folder: Path, camera: bool) -> subprocess.Popen:
        exclude = self.bag_exclude if camera else f'{self.bag_exclude}|{CAMERA_EXCLUDE}'
        log = open(folder / 'bag.log', 'w')  # noqa: SIM115 - kept open by the child
        process = subprocess.Popen(
            ['ros2', 'bag', 'record', '-a', '--exclude-regex', f'({exclude})', '--use-sim-time', '-s', 'mcap',
             '--storage-preset-profile', 'zstd_fast', '-o', str(folder / 'bag')],
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
            preexec_fn=lambda: os.umask(0o002))
        log.close()
        deadline = time.monotonic() + BAG_START_S
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RecordingError(f'ros2 bag record ended at once (bag.log): {self._tail(folder / "bag.log")}')
            if 'Recording...' in self._tail(folder / 'bag.log', 4000):
                return process
            time.sleep(0.5)
        return process  # slow to start (waiting for /clock): it records once the clock runs

    def _stop_bag(self, process: subprocess.Popen) -> None:
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    @staticmethod
    def _tail(path: Path, size: int = 400) -> str:
        try:
            return path.read_text(errors='replace')[-size:]
        except OSError:
            return ''

    # ---- video --------------------------------------------------------------------------
    def _control(self, rec: dict, folder: Path, cmd: str) -> None:
        opts = rec['options']
        command = {'id': f'{rec["id"]}-{cmd}', 'cmd': cmd, 'dir': str(folder),
                   'options': {'render_preset': QUALITY[opts['quality']], 'max_minutes': opts['max_min']}}
        temp = self.dir / '.control.tmp'
        temp.write_text(json.dumps(command))
        temp.replace(self.dir / 'control.json')

    def _start_video(self, rec: dict, folder: Path) -> bool:
        self._control(rec, folder, 'start')
        deadline = time.monotonic() + VIDEO_ANSWER_S
        while time.monotonic() < deadline:
            status = read_json(folder / 'status.json') or {}
            if status.get('state') == 'capturing':
                return True
            if status.get('state') == 'error':
                raise RecordingError(f'video recorder: {status.get("error")}')
            time.sleep(0.5)
        self._problem(rec, f'no video recorder answered in {VIDEO_ANSWER_S:g} s: bag only')
        return False

    def _stop_video(self, rec: dict, folder: Path) -> None:
        self._control(rec, folder, 'stop')
        deadline = time.monotonic() + VIDEO_FINISH_S
        while time.monotonic() < deadline:
            state = (read_json(folder / 'status.json') or {}).get('state')
            if state in ('done', 'error'):
                return
            time.sleep(1.0)
        self._problem(rec, 'the video recorder did not finish')

    # ---- mission ------------------------------------------------------------------------
    def _start_mission(self, mission: str) -> tuple[bool, str]:
        from taskweave_interfaces.srv import StartMission

        response = self.bridge.call(self.bridge.start_client, StartMission.Request(mission=mission, start_step=''))
        return response.accepted, response.message

    def _abort_mission(self) -> None:
        from std_srvs.srv import Trigger

        try:
            self.bridge.call(self.bridge.triggers['abort'], Trigger.Request())
        except TimeoutError as error:
            self.bridge.get_logger().warning(f'recording: abort after the last cycle failed: {error}')
