"""The smoke comparison across the move skill: a reference recorded with the six motion skills (navigate_to,
move_gripper, ...) is compared with a run of ``move`` steps (and the other way round). Steps are matched by step id; the
duration tolerance follows the kind of motion, so a base drive keeps its 25 % whichever name the step has.

The reference is a real recording of the frame cycle (26 steps, 21 Oct 2026); the runs are made from it."""

import copy
from pathlib import Path

import pytest
import yaml

from taskweave_core import cli, smoke

REFERENCE = Path(__file__).parent / 'fixtures' / 'smoke' / 'frame_cycle.reference.yaml'
KIND = {'navigate_to': 'base', 'drive_straight': 'straight', 'move_gripper': 'hand', 'move_object': 'object',
        'set_posture': 'posture', 'move_joints': 'joints'}


@pytest.fixture
def old() -> dict:
    return yaml.safe_load(REFERENCE.read_text())


def as_move(run: dict) -> dict:
    """The same run as ``move`` steps: skill move, data.kind and module of the kind of motion, ids unchanged."""
    new = copy.deepcopy(run)
    for step in new['steps']:
        kind = KIND.get(step['skill'])
        if kind:
            step['skill'] = 'move'
            step['data'] = {**step.get('data', {}), 'kind': kind, 'module': f'move.{kind}'}
    return new


def is_drive(step: dict) -> bool:
    return step['skill'] == 'navigate_to' or (step['skill'] == 'move' and step['data']['kind'] == 'base')


def scaled(run: dict, base: float, rest: float) -> dict:
    """Durations scaled: base drives by ``base``, the other steps by ``rest``."""
    new = copy.deepcopy(run)
    for step in new['steps']:
        step['duration_s'] = round(step['duration_s'] * (base if is_drive(step) else rest), 3)
    return new


def results(comparison: dict) -> dict:
    return {row['step_id']: row['result'] for row in comparison['rows']}


def step_of(run: dict, step_id: str) -> dict:
    return next(s for s in run['steps'] if s['step_id'] == step_id)


def test_the_reference_is_the_old_skills(old):
    skills = {step['skill'] for step in old['steps']}
    assert len(old['steps']) == 26 and skills >= set(KIND) | {'grip'}
    assert 'move' not in skills


def test_an_old_reference_passes_against_a_run_of_move_steps(old):
    run = scaled(as_move(old), base=1.20, rest=1.05)     # drives 20 % slower (limit 25 %), the rest 5 %
    assert {s['skill'] for s in run['steps']} == {'move', 'grip'}
    comparison = smoke.compare(old, run)
    assert comparison['passed'], smoke.table(comparison)
    assert len(comparison['rows']) == 26 and all(r['result'] == 'PASS' for r in comparison['rows'])
    limits = comparison['limits']['skill_tolerance']
    assert limits['navigate_to'] == limits['move.base'] == 0.25
    assert limits['move_gripper'] == limits['move.hand'] == 0.10


def test_a_non_drive_step_is_held_to_ten_percent_whatever_it_is_called(old):
    slow = next(s for s in old['steps'] if s['skill'] == 'move_object' and s['duration_s'] > 10)
    run = as_move(old)
    step_of(run, slow['step_id'])['duration_s'] = round(slow['duration_s'] * 1.2, 3)
    comparison = smoke.compare(old, run)
    assert [k for k, v in results(comparison).items() if v == 'FAIL'] == [slow['step_id']]
    assert '10%' in next(r['note'] for r in comparison['rows'] if r['step_id'] == slow['step_id'])


def test_a_drive_is_held_to_twenty_five_percent(old):
    drive = next(s for s in old['steps'] if s['skill'] == 'navigate_to')
    run = as_move(old)
    step = step_of(run, drive['step_id'])
    step['duration_s'] = round(drive['duration_s'] * 1.22, 3)
    assert smoke.compare(old, run)['passed']
    step['duration_s'] = round(drive['duration_s'] * 1.40, 3)
    comparison = smoke.compare(old, run)
    assert results(comparison)[drive['step_id']] == 'FAIL'
    assert '25%' in next(r['note'] for r in comparison['rows'] if r['step_id'] == drive['step_id'])


def test_straight_drives_are_held_to_the_general_tolerance(old):
    straight = next(s for s in old['steps'] if s['skill'] == 'drive_straight' and s['duration_s'] > 3.5)
    run = as_move(old)
    # 3.7 s * 1.4: 1.5 s over, more than 10 % + 1 s slack, less than the 25 % of a Nav2 drive would allow
    step_of(run, straight['step_id'])['duration_s'] = round(straight['duration_s'] * 1.4, 3)
    assert results(smoke.compare(old, run))[straight['step_id']] == 'FAIL'
    assert smoke.compare(old, run, skill_tolerance={'move.straight': 0.25})['passed']


def test_a_run_of_old_steps_passes_against_a_reference_of_move_steps(old):
    comparison = smoke.compare(as_move(old), scaled(old, base=1.2, rest=1.05))
    assert comparison['passed'], smoke.table(comparison)


def test_limits_written_with_either_name_mean_the_same(old):
    drive = next(s for s in old['steps'] if s['skill'] == 'navigate_to')
    run = as_move(old)
    step_of(run, drive['step_id'])['duration_s'] = round(drive['duration_s'] * 1.22, 3)
    tight = copy.deepcopy(old)
    tight['limits'] = {'tolerance_by_skill': {'navigate_to': 0.05}}          # the old key, in the reference file
    assert results(smoke.compare(tight, run))[drive['step_id']] == 'FAIL'
    assert smoke.compare(old, run, skill_tolerance={'navigate_to': 0.25})['passed']
    assert not smoke.compare(old, run, skill_tolerance={'move.base': 0.05})['passed']   # the new key
    assert smoke.compare(old, run, skill_tolerance={'move': 0.5})['passed']       # the whole skill
    # a limit of the whole skill does not loosen a kind that has its own
    both = smoke.compare(old, run, skill_tolerance={'move': 0.5, 'move.base': 0.05})
    assert results(both)[drive['step_id']] == 'FAIL'


def test_the_command_line_takes_both_spellings(old, tmp_path, capsys):
    reference, run, report = tmp_path / 'cycle.reference.yaml', tmp_path / 'run.yaml', tmp_path / 'r.yaml'
    smoke.save(str(reference), old)
    drive = next(s for s in old['steps'] if s['skill'] == 'navigate_to')
    data = as_move(old)
    step_of(data, drive['step_id'])['duration_s'] = round(drive['duration_s'] * 1.22, 3)
    smoke.save(str(run), data)
    args = ['smoke', 'compare', str(reference), str(run), '--report', str(report)]
    assert cli.main(args) == 0
    assert cli.main(args + ['--skill-tolerance', 'move.base=0.05']) == 1
    assert cli.main(args + ['--skill-tolerance', 'navigate_to=0.05']) == 1
    capsys.readouterr()


def test_the_run_keeps_the_kind_of_a_move_step():
    events = [{'event': 'step_started', 'cycle': 1, 'step_id': 's1', 'skill': 'move', 'time': 1.0, 'attempt': 1},
              {'event': 'step_finished', 'cycle': 1, 'step_id': 's1', 'skill': 'move', 'time': 3.0,
               'outcome': 'SUCCEEDED', 'message': 'move base to waiting (nav2): at the target (13 mm, 0.3 deg)',
               'data': {'kind': 'base', 'module': 'move.base', 'target': 'waiting', 'path': 'nav2', 'extra': 1}}]
    [step] = smoke.step_results(events)
    assert step['data'] == {'kind': 'base', 'module': 'move.base', 'path': 'nav2'}
    assert smoke.step_label(step) == 'move.base' and smoke.tolerance_key('navigate_to') == 'move.base'
    assert smoke.tolerance_key('grip') == 'grip' and smoke.tolerance_key('move') == 'move'
