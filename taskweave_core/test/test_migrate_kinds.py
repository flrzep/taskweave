"""Cell pack migration 4 -> 5 (config/migrate_kinds.py): the held-object form of a move step becomes ``hand: holding``,
the part files and robot overrides of navigate and carry get their names, smoke references their kind ids. On the
schema 4 fixture targets_new with a mission of every old form: comments and flow style kept, idempotent, dry run, all
or nothing, the result validates."""

import os
import shutil
import sys
from pathlib import Path

import pytest
import yaml

from taskweave_core.config import ConfigStore, packs
from taskweave_core.config.migrate import migrate
from taskweave_core.step_info import move_problems, step_info

pytest.importorskip('ruamel.yaml')

sys.path.insert(0, str(Path(__file__).parent))
import move_forms  # noqa: E402

FIXTURES = Path(__file__).parent / 'fixtures'
ROBOT = 'bot_two_arm'
FORMS = '''\
goal: Every form of schema 4   # the steps below
robot: bot_two_arm
steps:
  - {id: lift, instruction: Lift, skill: move, args: {object: held, by: [0, 0, 0.2], torso: keep}}   # 20 cm
  - id: tilt
    instruction: Tilt the frame
    skill: move
    args:
      object: held           # the frame
      by: [0, 0, 0]
      turn_deg: 10
      tilt_deg: 30
      motion: {speed: 0.5}
  - {id: at, instruction: To a point, skill: move, args: {object: held, at: [0.6, 0, 1.0]}}
  - {id: named, instruction: Which one, skill: move, args: {object: bay_2_stack_2_front, by: [0, 0, 0.1], hands: all}}
  - {id: param, instruction: A parameter, skill: move, args: {object: '{held_one}', at: [0.6, 0, 1.0]}}
  - {id: target, instruction: Object target, skill: move, args: {to: carry, object: bay_2_stack_2_front}}
  - {id: hand, instruction: A hand, skill: move, args: {hand: left, by: [0, 0, 0.1]}}
  - {id: grip, instruction: Grip, skill: grip, args: {hands: all, action: close}}
'''
REFERENCE = '''\
mission: forms
limits:
  tolerance_by_skill: {move.base: 0.3, move.object: 0.2, grip: 0.5}   # per kind
steps:
- {cycle: 1, step_id: lift, skill: move, duration_s: 2.0, data: {kind: object, module: move.object, path: linear}}
- {cycle: 1, step_id: drive, skill: move, duration_s: 9.0, data: {kind: base, module: move.base}}
- {cycle: 1, step_id: old, skill: navigate_to, duration_s: 9.0}
'''


@pytest.fixture(autouse=True)
def only_this_cell(tmp_path, monkeypatch):
    monkeypatch.setenv('TASKWEAVE_STATE_DIR', str(tmp_path / 'state'))
    monkeypatch.delenv(packs.ROBOT_PACKS_ENV, raising=False)
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8', newline='')


@pytest.fixture
def cell(tmp_path) -> Path:
    """targets_new (schema 4) with a mission of every old form, part files, a robot override, a smoke reference and
    an operator button."""
    root = tmp_path / 'cell'
    shutil.copytree(FIXTURES / 'targets_new', root)
    write(root / 'missions' / 'forms.yaml', FORMS)
    write(root / 'skills' / 'move' / 'base.yaml', '# the controller\nimplementations: [{name: a, type: script, '
                                                  'class: "taskweave_skills.navigate:NavigateTo"}]\nmax_linear: 0.4\n')
    write(root / 'skills' / 'move' / 'object.yaml', 'implementations: [{name: b, type: script, '
                                                    'class: "taskweave_skills.object_motion:MoveObject"}]\n')
    write(root / 'robots' / ROBOT / 'skill_overrides.yaml', '# this robot\nmove.object: {motion: {planner: direct}}\n'
                                                            'move.hand: {motion: {elbow: down}}\n')
    write(root / 'tests' / 'forms.reference.yaml', REFERENCE)
    write(root / 'operator.yaml', 'joy:\n  actions:\n    - {button: 3, skill: move, args: {object: held, by: [0, 0, '
                                  '0.05]}}   # nudge up\n')
    return root


def run(cell: Path, **kwargs):
    lines: list[str] = []
    return migrate(cell, say=lines.append, **kwargs), lines


def tree(root: Path) -> dict:
    return {p.relative_to(root).as_posix(): p.read_bytes() for p in sorted(root.rglob('*')) if p.is_file()}


def steps(cell: Path) -> dict:
    return {s['id']: s['args'] for s in yaml.safe_load((cell / 'missions' / 'forms.yaml').read_text())['steps']}


def test_every_old_form_becomes_hand_holding_and_the_rest_stays(cell):
    code, lines = run(cell)
    assert code == 0, '\n'.join(lines)
    assert packs.cell_schema(cell) == packs.CELL_SCHEMA == 5
    got = steps(cell)
    assert got['lift'] == {'hand': 'holding', 'by': [0, 0, 0.2], 'torso': 'keep'}
    assert got['tilt'] == {'hand': 'holding', 'by': [0, 0, 0], 'turn_deg': 10, 'tilt_deg': 30,
                           'motion': {'speed': 0.5}}
    assert got['at'] == {'hand': 'holding', 'at': [0.6, 0, 1.0]}
    assert got['named'] == {'hand': 'holding', 'by': [0, 0, 0.1], 'hands': 'all', 'object': 'bay_2_stack_2_front'}
    assert got['param'] == {'hand': 'holding', 'at': [0.6, 0, 1.0], 'object': '{held_one}'}
    assert got['target'] == {'to': 'carry', 'object': 'bay_2_stack_2_front'}       # an object target: unchanged
    assert got['hand'] == {'hand': 'left', 'by': [0, 0, 0.1]} and got['grip'] == {'hands': 'all', 'action': 'close'}
    assert sum(line.startswith('step    forms ') for line in lines) == 5


def test_the_order_comments_and_flow_style_of_the_mission_are_kept(cell):
    run(cell)
    text = (cell / 'missions' / 'forms.yaml').read_text(encoding='utf-8')
    assert text.startswith('goal: Every form of schema 4   # the steps below\nrobot: bot_two_arm\nsteps:\n'
                           '  - {id: lift')
    assert "args: {hand: holding, by: [0, 0, 0.2], torso: keep}}  # 20 cm" in text
    assert "args: {hand: holding, at: [0.6, 0, 1.0], object: '{held_one}'}}" in text
    assert '      hand: holding          # the frame\n      by: [0, 0, 0]\n' in text
    assert 'args: {hand: holding, by: [0, 0, 0.1], hands: all, object: bay_2_stack_2_front}}' in text
    assert (cell / 'operator.yaml').read_text().endswith('args: {hand: holding, by: [0, 0, 0.05]}}  # nudge up\n')


def test_the_part_files_overrides_and_references_get_the_names_of_schema_5(cell):
    before = (cell / 'skills' / 'move' / 'base.yaml').read_bytes()
    run(cell)
    move = cell / 'skills' / 'move'
    assert not (move / 'base.yaml').exists() and not (move / 'object.yaml').exists()
    assert (move / 'navigate.yaml').read_bytes() == before and (move / 'carry.yaml').is_file()
    overrides = (cell / 'robots' / ROBOT / 'skill_overrides.yaml').read_text()
    assert overrides == '# this robot\nmove.carry: {motion: {planner: direct}}\nmove.hand: {motion: {elbow: down}}\n'
    reference = (cell / 'tests' / 'forms.reference.yaml').read_text()
    assert 'tolerance_by_skill: {move.navigate: 0.3, move.hand: 0.2, grip: 0.5} # per kind' in reference
    data = yaml.safe_load(reference)
    assert [s.get('data') for s in data['steps']] == [
        {'kind': 'hand', 'module': 'move.carry', 'path': 'linear'}, {'kind': 'navigate', 'module': 'move.navigate'},
        None]
    store = ConfigStore(cell)
    assert store.skill_params('move.navigate')['max_linear'] == 0.4


def test_a_second_run_changes_nothing_and_a_dry_run_changes_nothing(cell):
    before = tree(cell)
    code, lines = run(cell, dry_run=True)
    assert code == 0 and tree(cell) == before
    assert any(line.startswith('would write') for line in lines)
    assert any(line.startswith('move    skills/move/base.yaml -> skills/move/navigate.yaml') for line in lines)
    assert run(cell)[0] == 0
    after = tree(cell)
    code, lines = run(cell)
    assert code == 0 and tree(cell) == after and 'nothing to do' in lines[-1]
    from taskweave_core.config.migrate_kinds import migrate_kinds
    said: list[str] = []
    migrate_kinds(cell, None, False, said.append)                # the step itself on a migrated cell: nothing
    assert tree(cell) == after and said[-1].startswith('wrote 0 file(s), remove 0')


def test_a_robot_folder_linked_out_of_the_cell_is_not_touched(cell, tmp_path):
    shared = tmp_path / 'shared' / ROBOT
    shared.parent.mkdir()
    (cell / 'robots' / ROBOT).rename(shared)
    try:
        os.symlink(shared, cell / 'robots' / ROBOT, target_is_directory=True)
    except OSError:
        pytest.skip('no symbolic links here')
    before = (shared / 'skill_overrides.yaml').read_text()
    code, lines = run(cell)
    assert code == 0 and (shared / 'skill_overrides.yaml').read_text() == before
    assert any(line.startswith('FLAG') and 'link out of the cell pack' in line for line in lines)


def test_check_takes_the_old_override_keys_as_the_parts_they_were(cell):
    run(cell)
    path = cell / 'robots' / ROBOT / 'skill_overrides.yaml'
    path.write_text('move.base: {motion: {speed: 0.5}}\nmove.bogus: {}\n')   # e.g. a robot pack not migrated
    found = [p['message'] for p in ConfigStore(cell).check_pack() if p['path'].endswith('skill_overrides.yaml')]
    assert len(found) == 1 and "'move.bogus' does not exist" in found[0]


def test_an_existing_new_part_file_is_kept_and_flagged(cell):
    write(cell / 'skills' / 'move' / 'navigate.yaml', 'implementations: [{name: c, type: script, class: "x:Y"}]\n')
    code, lines = run(cell)
    assert code == 0
    assert (cell / 'skills' / 'move' / 'base.yaml').is_file()
    assert any(line.startswith('FLAG') and 'skills/move/base.yaml kept' in line for line in lines)


def test_an_unwritable_file_stops_before_anything_is_touched(cell, monkeypatch):
    from taskweave_core.config import migrate_targets as mt

    before = tree(cell)
    blocked = cell / 'operator.yaml'
    real = os.access
    monkeypatch.setattr(mt.os, 'access', lambda path, mode: False if Path(path) == blocked else real(path, mode))
    code, lines = run(cell)
    assert code == 2 and 'operator.yaml is not writable' in lines[-1] and tree(cell) == before
    assert packs.cell_schema(cell) == 4


def test_the_migrated_cell_validates_and_its_steps_have_the_five_kinds(cell):
    schema = {'type': 'object', 'x-taskweave-resources': ['base', 'hands', 'torso', 'joints']}
    old = ConfigStore(cell)
    assert move_problems(steps(cell)['lift'], old, ROBOT) == []            # schema 4: the old form runs
    run(cell)
    store = ConfigStore(cell)
    assert move_forms.mission_problems(store, 'forms') == []
    for args in steps(cell).values():
        assert move_problems(args, store, ROBOT) == [], args
    assert step_info('move', steps(cell)['lift'], schema, store, ROBOT)[0] == 'hand'
    [(_, text)] = move_problems({'object': 'held', 'by': [0, 0, 0.1]}, store, ROBOT)     # schema 5: the old form
    assert 'taskweave config migrate' in text


def test_check_notes_the_old_form_on_schema_4_and_fails_it_on_schema_5(cell):
    def found(store):
        return [p for p in store.check_pack() if p['path'] == 'missions/forms.yaml' and 'hand: holding' in p['message']]

    notes = found(ConfigStore(cell))
    assert len(notes) == 5 and {p['severity'] for p in notes} == {'warning'}
    assert 'taskweave config migrate' in notes[0]['message']
    pack = cell / 'pack.yaml'
    pack.write_text(pack.read_text().replace('schema_version: 4', 'schema_version: 5'))
    errors = found(ConfigStore(cell))
    assert len(errors) == 5 and {p['severity'] for p in errors} == {'error'}
    old_files = [p for p in ConfigStore(cell).check_pack() if p['path'] == 'skills/move/base.yaml']
    assert old_files and 'skills/move/navigate.yaml' in old_files[0]['message']
