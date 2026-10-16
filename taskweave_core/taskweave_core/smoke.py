"""Mission smoke test: run a mission in the simulator from a clean start, compare each step with a reference run.

The run goes through the HMI's HTTP API only (no ROS here): ``POST /api/recordings/start`` resets the sim, starts the
mission and ends it after the requested cycles; ``GET /api/recordings`` gives the recording's status (``active``
false: it ended) and its ``meta.json`` all step events of the run (step_started / step_finished, time = sim time).
``GET /api/state`` is followed during the run for progress and as the fallback for older HMIs.

A run (and a reference, which is a saved run) is YAML::

    mission: frame_cycle
    outcome: 1 cycle done          # the recording's outcome
    steps:
    - {cycle: 1, step_id: s1, skill: navigate_to, instruction: ..., outcome: SUCCEEDED, duration_s: 12.4,
       max_error: 0.005, attempts: 1, message: ...}

Comparison: every reference step must come in the same order with outcome SUCCEEDED, its duration within
±tolerance (relative) plus an absolute slack, its max error not above reference × (1 + tolerance) plus a slack.
"""

from __future__ import annotations

import argparse
import json
import re
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path

import yaml

ERROR_RE = re.compile(r'max error (-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)')
DEFAULT_HMI = 'http://localhost:8090'
STATE_EVENTS = 50  # /api/state shows the latest 50 events
END_MARGIN_S = 300.0  # beyond the mission time limit: sim reset before, recording finish after


class SmokeError(Exception):
    """The smoke test could not run (HMI unreachable, recording rejected, timeout)."""


# ---- events -> results ----------------------------------------------------------------
def event_key(event: dict) -> tuple:
    return tuple(event.get(k) for k in ('event', 'plan_id', 'cycle', 'step_id', 'attempt', 'time'))


def max_error(message: str) -> float | None:
    match = ERROR_RE.search(message or '')
    return float(match.group(1)) if match else None


def step_results(events: list[dict], cycles: int | None = None) -> list[dict]:
    """One result per (cycle, step id), in the order the steps started: the last attempt's outcome and message,
    duration from the first start to the last finish (sim time). Steps of cycles beyond ``cycles`` are left out
    (the recording aborts the mission once the next cycle has begun)."""
    results: dict[tuple, dict] = {}
    for event in events:
        kind, cycle = event.get('event'), event.get('cycle', 0)
        if kind not in ('step_started', 'step_finished') or (cycles and cycle > cycles):
            continue
        key = (cycle, event.get('step_id'))
        result = results.get(key)
        if result is None:
            if kind != 'step_started':
                continue  # its start was before this run
            result = results[key] = {'cycle': cycle, 'step_id': event.get('step_id'), 'skill': event.get('skill', ''),
                                     'instruction': event.get('instruction', ''), 'outcome': 'RUNNING',
                                     'duration_s': None, 'max_error': None, 'attempts': 0, 'message': '',
                                     '_start': event.get('time')}
        if kind == 'step_started':
            result['attempts'] = max(result['attempts'], int(event.get('attempt') or 1))
            continue
        result['outcome'], result['message'] = event.get('outcome', ''), event.get('message', '')
        result['max_error'] = max_error(result['message'])
        if result['_start'] is not None and event.get('time') is not None:
            result['duration_s'] = round(float(event['time']) - float(result['_start']), 3)
    return [{k: v for k, v in r.items() if not k.startswith('_')} for r in results.values()]


# ---- comparison ---------------------------------------------------------------------------
def compare(reference: dict, run: dict, tolerance: float = 0.10, slack_s: float = 1.0, error_slack: float = 0.002,
            allow_extra: bool = False) -> dict:
    """Rows (one per reference step, plus EXTRA run steps) and ``passed``."""
    ref_steps, run_steps = reference.get('steps') or [], run.get('steps') or []
    rows, at = [], 0
    for ref in ref_steps:
        key = (ref.get('cycle'), ref.get('step_id'))
        found = next((i for i in range(at, len(run_steps))
                      if (run_steps[i].get('cycle'), run_steps[i].get('step_id')) == key), None)
        if found is None:
            rows.append(_row(ref, None, 'MISSING', 'not run'))
            continue
        rows.extend(_row(None, extra, 'PASS' if allow_extra else 'EXTRA', 'not in the reference')
                    for extra in run_steps[at:found])
        rows.append(_judge(ref, run_steps[found], tolerance, slack_s, error_slack))
        at = found + 1
    rows.extend(_row(None, extra, 'PASS' if allow_extra else 'EXTRA', 'not in the reference')
                for extra in run_steps[at:])
    failed = sum(row['result'] != 'PASS' for row in rows)
    return {'passed': failed == 0, 'failed': failed, 'rows': rows,
            'limits': {'tolerance': tolerance, 'slack_s': slack_s, 'error_slack': error_slack,
                       'allow_extra': allow_extra}}


def _row(ref: dict | None, now: dict | None, result: str, note: str = '') -> dict:
    step = now or ref
    return {'cycle': step.get('cycle'), 'step_id': step.get('step_id'), 'skill': step.get('skill', ''),
            'instruction': step.get('instruction', ''),
            'ref_s': ref.get('duration_s') if ref else None, 'now_s': now.get('duration_s') if now else None,
            'delta_pct': None, 'ref_error': ref.get('max_error') if ref else None,
            'now_error': now.get('max_error') if now else None, 'outcome': now.get('outcome') if now else None,
            'result': result, 'note': note}


def _judge(ref: dict, now: dict, tolerance: float, slack_s: float, error_slack: float) -> dict:
    row, problems = _row(ref, now, 'PASS'), []
    if now.get('outcome') != 'SUCCEEDED':
        problems.append(f'{now.get("outcome")}: {now.get("message", "")}'.rstrip(': '))
    ref_s, now_s = ref.get('duration_s'), now.get('duration_s')
    if ref_s is not None and now_s is not None:
        row['delta_pct'] = round(100.0 * (now_s - ref_s) / ref_s, 1) if ref_s else None
        if abs(now_s - ref_s) > tolerance * ref_s + slack_s:
            problems.append(f'{"slower" if now_s > ref_s else "faster"} than {ref_s:g} s ±{tolerance:.0%}')
    elif ref_s is not None:
        problems.append('no duration')
    ref_e, now_e = ref.get('max_error'), now.get('max_error')
    if ref_e is not None and now_e is not None and now_e > ref_e * (1 + tolerance) + error_slack:
        problems.append(f'error {now_e:g} > {ref_e:g}')
    if now.get('attempts', 1) > ref.get('attempts', 1):
        row['note'] = f'{now["attempts"]} attempts (reference {ref.get("attempts", 1)})'
    if problems:
        row['result'], row['note'] = 'FAIL', '; '.join(problems + ([row['note']] if row['note'] else []))
    return row


def table(comparison: dict) -> str:
    def num(value, fmt):
        return '-' if value is None else format(value, fmt)

    def error(row):
        if row['now_error'] is None and row['ref_error'] is None:
            return ''
        return f'{num(row["now_error"], ".4f")} ({num(row["ref_error"], ".4f")})'

    lines = [f'{"#":>3} {"cy":>2} {"step":<14} {"instruction":<36} {"ref s":>7} {"now s":>7} {"Δ%":>6} '
             f'{"error (ref)":<17} {"result":<7} note']
    for i, row in enumerate(comparison['rows'], 1):
        delta = '-' if row['delta_pct'] is None else f'{row["delta_pct"]:+.1f}'
        lines.append(f'{i:>3} {row["cycle"]!s:>2} {str(row["step_id"])[:14]:<14} {row["instruction"][:36]:<36} '
                     f'{num(row["ref_s"], ".1f"):>7} {num(row["now_s"], ".1f"):>7} {delta:>6} {error(row):<17} '
                     f'{row["result"]:<7} {row["note"]}'.rstrip())
    counts = {}
    for row in comparison['rows']:
        counts[row['result']] = counts.get(row['result'], 0) + 1
    summary = ', '.join(f'{n} {k}' for k, n in sorted(counts.items()))
    lines.append(f'{"PASSED" if comparison["passed"] else "FAILED"}: {summary or "no steps"}')
    return '\n'.join(lines)


def run_table(run: dict) -> str:
    lines = [f'{"#":>3} {"cy":>2} {"step":<14} {"instruction":<36} {"s":>7} {"error":>8} outcome']
    for i, step in enumerate(run.get('steps') or [], 1):
        duration = '-' if step['duration_s'] is None else f'{step["duration_s"]:.1f}'
        err = '' if step['max_error'] is None else f'{step["max_error"]:.4f}'
        lines.append(f'{i:>3} {step["cycle"]!s:>2} {str(step["step_id"])[:14]:<14} {step["instruction"][:36]:<36} '
                     f'{duration:>7} {err:>8} {step["outcome"]}')
    lines.append(f'{run.get("mission")}: {run.get("outcome")}')
    return '\n'.join(lines)


def run_ok(run: dict) -> bool:
    """Without a reference: every step succeeded and the mission did not fail."""
    outcome = str(run.get('outcome', ''))
    bad = ('failed', 'aborted', 'error', 'time limit', 'stopped', 'timeout')
    return bool(run.get('steps')) and all(s['outcome'] == 'SUCCEEDED' for s in run['steps']) \
        and not any(word in outcome for word in bad)


# ---- files -----------------------------------------------------------------------------
def load(path: str) -> dict:
    """A run or reference file; a report file gives its run."""
    data = yaml.safe_load(Path(path).read_text()) or {}
    return data['run'] if isinstance(data.get('run'), dict) else data


def save(path: str, data: dict) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True, width=120))


def report_path(reference: str) -> str:
    path = Path(reference)
    name = path.name.replace('.reference.yaml', '') if path.name.endswith('.reference.yaml') else path.stem
    return str(path.with_name(name + '.report.yaml'))


# ---- the run ---------------------------------------------------------------------------
class Hmi:
    """The HMI's HTTP API (stdlib only)."""

    def __init__(self, url: str, timeout_s: float = 15.0):
        self.url, self.timeout_s = url.rstrip('/'), timeout_s

    def request(self, method: str, path: str, body: dict | None = None):
        data = json.dumps(body).encode() if body is not None else None
        request = urllib.request.Request(self.url + path, data=data, method=method,
                                         headers={'Content-Type': 'application/json'} if data else {})
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
                return json.loads(response.read() or b'null')
        except urllib.error.HTTPError as error:
            try:
                text = json.loads(error.read()).get('error') or error.reason
            except (ValueError, AttributeError):
                text = error.reason
            raise SmokeError(f'{method} {path}: {error.code} {text}') from None
        except (urllib.error.URLError, OSError) as error:
            raise SmokeError(f'{method} {path}: HMI at {self.url} not reachable ({error})') from None

    def get(self, path: str):
        return self.request('GET', path)

    def post(self, path: str, body: dict | None = None):
        return self.request('POST', path, body or {})


def run(hmi: Hmi, mission: str, cycles: int = 1, reset: bool = True, bag: bool = False, video: bool = False,
        timeout_min: float = 60.0, poll_s: float = 1.0, log=print) -> dict:
    """Start ``mission`` through the recording API, follow it to its end, return the run."""
    status = hmi.get('/api/recordings').get('status') or {}
    if status.get('active'):
        raise SmokeError(f'recording {status.get("id")} is running')
    seen = {event_key(e) for e in hmi.get('/api/state').get('events') or []}  # events of earlier runs
    body = {'mission': mission, 'name': 'smoke', 'cycles': cycles, 'reset': reset, 'bag': bag, 'camera': False,
            'video': video, 'max_min': min(max(timeout_min, 0.5), 1440.0)}
    rec_id = hmi.post('/api/recordings/start', body)['id']
    log(f'recording {rec_id}: {mission}, {cycles} cycle(s){", sim reset" if reset else ""}')
    events, phase, deadline = [], '', time.monotonic() + 60.0 * body['max_min'] + END_MARGIN_S
    while True:
        if _collect(hmi, seen, events):
            log('  warning: /api/state showed only new events - some may have been missed (poll faster)')
        status = hmi.get('/api/recordings').get('status') or {}
        if status.get('id') != rec_id:
            raise SmokeError(f'recording {rec_id} is gone (status shows {status.get("id")})')
        if status.get('phase') != phase:
            phase = status.get('phase', '')
            log(f'  {phase}')
        if not status.get('active'):
            break
        if time.monotonic() > deadline:
            log('  timeout: stopping the recording and aborting the mission')
            for path in ('/api/recordings/stop', '/api/abort'):
                try:
                    hmi.post(path)
                except SmokeError as error:
                    log(f'  {error}')
            status = {**status, 'outcome': 'timeout'}
            break
        time.sleep(poll_s)
    _collect(hmi, seen, events)
    try:  # exact: the recording's meta.json holds every step event of the run (HMI 9bc6a5f on); polled ones otherwise
        recorded = hmi.get(f'/recordings/{rec_id}/meta.json').get('events')
        if isinstance(recorded, list) and recorded:
            events = recorded
    except (SmokeError, AttributeError):
        pass
    return {'mission': mission, 'recorded': datetime.now().isoformat(timespec='seconds'), 'recording': rec_id,
            'outcome': status.get('outcome', ''), 'problems': status.get('problems') or [], 'cycles': cycles,
            'reset': reset, 'steps': step_results(events, cycles)}


def _collect(hmi: Hmi, seen: set, events: list) -> bool:
    """Add the new step events from /api/state; True when its whole window (the latest 50) was new, so events
    before it may be lost (polled every second or so, none are)."""
    window = hmi.get('/api/state').get('events') or []
    new = [event for event in window if event_key(event) not in seen]
    seen.update(event_key(event) for event in new)
    gap = bool(events) and len(window) >= STATE_EVENTS and len(new) == len(window)
    events.extend(new)
    return gap


# ---- command line (taskweave smoke ...) --------------------------------------------------
def add_arguments(parser) -> None:
    sub = parser.add_subparsers(dest='smoke_command', required=True)
    run_parser = sub.add_parser('run', help='run a mission from a clean start, compare it with a reference run')
    run_parser.add_argument('mission')
    run_parser.add_argument('--hmi', default=DEFAULT_HMI, help=f'HMI address (default {DEFAULT_HMI})')
    run_parser.add_argument('--cycles', type=int, default=1)
    run_parser.add_argument('--reference', help='compare with this reference run (YAML)')
    run_parser.add_argument('--save-reference', metavar='FILE', help='save this run as the reference')
    run_parser.add_argument('--save-run', metavar='FILE', help='save this run (for a later compare)')
    run_parser.add_argument('--no-reset', action='store_true', help='start where the sim is (no sim reset)')
    run_parser.add_argument('--bag', action='store_true', help='record a ROS bag (without camera images)')
    run_parser.add_argument('--video', action='store_true', help='record the simulator videos')
    run_parser.add_argument('--timeout', type=float, default=60.0, metavar='MIN', help='mission time limit (min)')
    run_parser.add_argument('--poll', type=float, default=1.0, metavar='S', help=argparse.SUPPRESS)
    compare_parser = sub.add_parser('compare', help='compare a saved run (or report) with a reference, offline')
    compare_parser.add_argument('reference')
    compare_parser.add_argument('run')
    for p in (run_parser, compare_parser):
        p.add_argument('--tolerance', type=float, default=0.10, help='relative duration/error tolerance (0.10)')
        p.add_argument('--slack', type=float, default=1.0, metavar='S',  # path computing varies ~1 s with CPU load
                       help='absolute duration slack (1 s)')
        p.add_argument('--error-slack', type=float, default=0.002, help='absolute max error slack (0.002)')
        p.add_argument('--allow-extra', action='store_true', help='steps not in the reference do not fail')
        p.add_argument('--report', metavar='FILE', help='report file (default: <reference>.report.yaml)')


def main(options) -> int:
    """0: passed, 1: failed, 2: could not run."""
    limits = {'tolerance': options.tolerance, 'slack_s': options.slack, 'error_slack': options.error_slack,
              'allow_extra': options.allow_extra}
    try:
        if options.smoke_command == 'compare':
            reference, run_data = load(options.reference), load(options.run)
        else:
            reference = load(options.reference) if options.reference else None  # a bad file fails before the run
            run_data = run(Hmi(options.hmi), options.mission, options.cycles, not options.no_reset, options.bag,
                           options.video, options.timeout, options.poll, log=lambda text: print(text, flush=True))
            if options.save_run:
                save(options.save_run, run_data)
            if options.save_reference:
                save(options.save_reference, run_data)
                print(f'reference saved: {options.save_reference}')
    except (SmokeError, OSError, yaml.YAMLError) as error:
        print(f'smoke: {error}')
        return 2
    if reference is None:
        print(run_table(run_data))
        if options.report:
            save(options.report, {'run': run_data, 'passed': run_ok(run_data)})
        return 0 if run_ok(run_data) else 1
    comparison = compare(reference, run_data, **limits)
    print(f'{run_data.get("mission")} ({run_data.get("outcome", "")}) against {options.reference}')
    print(table(comparison))
    report = options.report or report_path(options.reference)
    save(report, {'reference': options.reference, 'checked': datetime.now().isoformat(timespec='seconds'),
                  **comparison, 'run': run_data})
    print(f'report: {report}')
    return 0 if comparison['passed'] else 1
