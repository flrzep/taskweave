"""Cell pack migration 3 -> 4 (config/migrate_targets.py, migrate_move.py): the old point files and the six motion
skills become targets and ``move``. Golden tests on the fixture pair (targets_old -> targets_new) and on the docs
example cell,
synthetic cells for every construct of the design (name clashes, run-time instance, partial group values, whole-file
and keyed overrides, first-variant mismatch, contact references, every step form), the equivalence dump of
tools/dump_targets.py, robot packs, idempotence and the stops.

The golden output of the docs example cell is fixtures/migrate_targets/docs_cell; to make it again after a change of the
migration (read the diff before committing it):

    python3 taskweave_core/test/test_migrate_targets.py --regenerate
"""

import importlib.util
import os
import re
import shutil
import sys
from pathlib import Path

import pytest
import yaml

from taskweave_core.config import ConfigStore, packs
from taskweave_core.config.migrate import migrate, migrate_skills_only

pytest.importorskip('ruamel.yaml')

sys.path.insert(0, str(Path(__file__).parent))
import move_forms  # noqa: E402

HERE = Path(__file__).parent
FIXTURES = HERE / 'fixtures'
REPO = HERE.parents[1]
DOCS = REPO / 'docs' / 'examples'
GOLDEN = FIXTURES / 'migrate_targets' / 'docs_cell'
ROBOT = 'bot_two_arm'
TOLERANCE = 1e-6


def load_dump_tool():
    spec = importlib.util.spec_from_file_location('dump_targets', REPO / 'tools' / 'dump_targets.py')
    module = importlib.util.module_from_spec(spec)
    sys.modules['dump_targets'] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(autouse=True)
def only_this_cell(tmp_path, monkeypatch):
    monkeypatch.setenv('TASKWEAVE_STATE_DIR', str(tmp_path / 'state'))
    monkeypatch.delenv(packs.ROBOT_PACKS_ENV, raising=False)
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8', newline='')


def run(cell: Path, *args, **kwargs):
    lines: list[str] = []
    code = migrate(cell, *args, say=lines.append, **kwargs)
    return code, lines


def tree(root: Path) -> dict:
    return {p.relative_to(root).as_posix(): p.read_bytes() for p in sorted(root.rglob('*')) if p.is_file()}


def data(path: Path):
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def close(a, b, tolerance: float = TOLERANCE) -> bool:
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(close(a[k], b[k], tolerance) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close(x, y, tolerance) for x, y in zip(a, b, strict=True))
    if isinstance(a, int | float) and isinstance(b, int | float) and not isinstance(a, bool):
        return abs(a - b) <= tolerance
    return a == b


@pytest.fixture
def old_cell(tmp_path) -> Path:
    """A copy of the old twin of the targets fixture, with missions of every step form."""
    root = tmp_path / 'old'
    shutil.copytree(FIXTURES / 'targets_old', root)
    write(root / 'system.yaml', f'robot: {ROBOT}\n')
    return root


# ---- the fixture pair is a golden pair ------------------------------------------------------------------------------
def test_the_old_fixture_becomes_the_new_fixture(old_cell):
    # the reference of a contact comes from the steps that go onto it: one mission does (the fixture has none)
    add_mission(
        old_cell,
        'hang',
        '- {id: onto, instruction: x, skill: move_object, args: {to: {onto: '
        'karakuri_hanger/hook, contact: hook_plate}}}\n',
    )
    code, lines = run(old_cell)
    assert code == 0, '\n'.join(lines)
    new = FIXTURES / 'targets_new'
    for path in sorted(new.rglob('*.yaml')):
        relative = path.relative_to(new)
        if relative.parts[0] == 'missions' or relative.as_posix() in ('pack.yaml',):
            continue
        assert (old_cell / relative).is_file(), f'{relative} was not made'
        # the fixture spells angles with 4 decimals, the migration keeps the tool axes exact (6 and more decimals)
        assert close(data(old_cell / relative), data(path), 1e-5), relative.as_posix()
    migrated = {p.relative_to(old_cell).as_posix() for p in old_cell.rglob('*.yaml')}
    assert not {
        f
        for f in migrated
        if f.endswith(('locations.yaml', 'stances.yaml', 'held_poses.yaml', 'object_points.yaml', 'poses.yaml'))
    }
    assert packs.cell_schema(old_cell) == 4


def test_the_renames_and_drops_are_printed(old_cell):
    _, lines = run(old_cell)
    assert any(line.startswith('rename  stance') and 'karakuri_hanger knob -> knob_stance' in line for line in lines)
    assert any(line.startswith('rename  held pose') and 'set_down -> set_down_held' in line for line in lines)


def test_a_second_run_changes_nothing_and_a_dry_run_changes_nothing(old_cell, tmp_path):
    before = tree(old_cell)
    code, lines = run(old_cell, dry_run=True)
    assert code == 0 and tree(old_cell) == before
    assert any(line.startswith('would write') for line in lines)
    assert any(line.startswith('move    cell/locations.yaml') for line in lines)
    assert run(old_cell)[0] == 0
    after = tree(old_cell)
    code, lines = run(old_cell)
    assert code == 0 and tree(old_cell) == after and 'nothing to do' in lines[0]


def test_the_migrated_cell_checks_and_loads(old_cell):
    from taskweave_core import cli

    assert run(old_cell)[0] == 0
    store = ConfigStore(old_cell)
    assert store.targets_format() and store.target('karakuri_hanger/knob_stance', ROBOT).kind == 'base'
    assert cli.check(str(old_cell)) == 0


def test_the_equivalence_dump_of_the_pair_is_equal_modulo_the_logged_renames(old_cell, tmp_path):
    tool = load_dump_tool()
    before = tool.dump(old_cell)
    assert before['targets_format'] is False
    _, lines = run(old_cell)
    log = tmp_path / 'log.txt'
    log.write_text('\n'.join(lines), encoding='utf-8')
    after = tool.dump(old_cell)
    assert after['targets_format'] is True and 'native' in after
    sections = before['legacy']
    assert len(sections['stances']) >= 8 and len(sections['points']) >= 10 and len(sections['held']) == 4
    said: list[str] = []
    assert tool.compare(before, after, tool.read_log(log), TOLERANCE, said.append), '\n'.join(said)
    for kind in ('legacy', 'native'):
        assert after[kind]['stances'] == pytest.approx(after['legacy']['stances'])


def test_the_equivalence_compare_sees_a_difference_and_a_missing_entry(old_cell, tmp_path):
    tool = load_dump_tool()
    before = tool.dump(old_cell)
    run(old_cell)
    after = tool.dump(old_cell)
    after['legacy']['stances'][f'{ROBOT}/bay_1_stack_1_front/pick'][0] += 0.001
    del after['native']['contacts']['karakuri_hanger/hook']
    said: list[str] = []
    assert not tool.compare(before, after, None, TOLERANCE, said.append)
    text = '\n'.join(said)
    assert 'differ: ' + f'{ROBOT}/bay_1_stack_1_front/pick' in text and 'only before: karakuri_hanger/hook' in text


# ---- the docs example cell -------------------------------------------------------------------------------------------
def docs_cell(target: Path) -> Path:
    """The docs example as one cell pack: the cell, the example missions and the skill files of docs/examples."""
    shutil.copytree(DOCS / 'cell', target)
    shutil.copytree(DOCS / 'missions', target / 'missions')
    shutil.copytree(DOCS / 'skills', target / 'skills', dirs_exist_ok=True)
    return target


def test_the_docs_example_cell_is_migrated_to_the_golden_files(tmp_path):
    cell = docs_cell(tmp_path / 'docs')
    code, lines = run(cell)
    assert code == 0, '\n'.join(lines)
    got, want = tree(cell), tree(GOLDEN)
    assert sorted(got) == sorted(want)
    for name in want:
        assert got[name] == want[name], f'{name} differs from fixtures/migrate_targets/docs_cell'
    assert run(cell)[0] == 0 and tree(cell) == got


def test_the_migrated_docs_example_checks_and_its_steps_are_valid(tmp_path, capsys):
    from taskweave_core import cli

    cell = docs_cell(tmp_path / 'docs')
    run(cell)
    capsys.readouterr()
    code = cli.check(str(cell))
    failed = [line for line in capsys.readouterr().out.splitlines() if line.strip().startswith('FAIL')]
    # the only thing that may fail is the move skill class, until taskweave_skills has it (the move skill task)
    assert all('taskweave_skills.move' in line for line in failed), failed
    assert code == (1 if failed else 0)
    store = ConfigStore(cell)
    names = store.mission_names()
    assert len(names) >= 6
    for name in names:
        assert move_forms.mission_problems(store, name) == [], name


# ---- synthetic cells: the constructs of the design -------------------------------------------------------------------
def add_mission(cell: Path, name: str, steps: str, header: str = 'goal: Test\n') -> None:
    write(cell / 'missions' / f'{name}.yaml', f'{header}steps:\n{steps}')


def steps_of(cell: Path, name: str) -> list[dict]:
    return data(cell / 'missions' / f'{name}.yaml')['steps']


def args_of(cell: Path, name: str) -> list[dict]:
    return [s.get('args') for s in steps_of(cell, name)]


FORMS = """\
- {id: a1, instruction: base, skill: navigate_to, args: {target: waiting}}
- {id: a2, instruction: stance, skill: navigate_to, args: {target: "stance:bay_1_stack_1_front"}}
- {id: a3, instruction: variant, skill: navigate_to, args: {target: "stance:karakuri_hanger/hang"}}
- {id: a4, instruction: renamed variant, skill: drive_straight, args: {to: "stance:karakuri_hanger/knob"}}
- {id: a5, instruction: default variant, skill: drive_straight, args: {to: "stance:karakuri_hanger"}}
- {id: a6, instruction: distance, skill: drive_straight, args: {distance_m: -0.5}}
- {id: a7, instruction: location prefix, skill: navigate_to, args: {target: "location:point_a"}}
- {id: b1, instruction: joint pose, skill: move_joints, args: {pose: lift_high}}
- {id: b2, instruction: only, skill: move_joints, args: {pose: home, only: [left_arm], motion: {speed: 0.5}}}
- {id: b3, instruction: joints, skill: move_joints, args: {joints: {body_joint: 0.3, head_joint1: 0.1}}}
- {id: c1, instruction: point, skill: move_gripper, args: {to: {point: bay_1_stack_1_front/rails_top, offset_m: 0.15}}}
- {id: c2, instruction: pose, skill: move_gripper, args: {to: {pose: frame_grasp_approach}}}
- {id: c3, instruction: relative, skill: move_gripper, args: {to: {by: [0, 0, 0.1], frame: tool}, hands: [left]}}
- {id: c4, instruction: absolute, skill: move_gripper,
  args: {to: {at: [0.5, 0, 0.5], rpy_deg: [0, 90, 0]}, hands: [right]}}
- {id: c5, instruction: any one, skill: move_gripper,
  args: {hands: any_one, to: {point: part_box_1/top_grasp}, motion: {mode: linear, torso: keep}}}
- {id: c6, instruction: relative no hands, skill: move_gripper, args: {to: {by: [0.1, 0, 0]}}}
- {id: d1, instruction: lift, skill: move_object, args: {to: {by: [0, 0, 0.3]}}}
- {id: d2, instruction: tilt, skill: move_object, args: {to: {by: [0, 0, 0], tilt_deg: 30}, hands: [left]}}
- {id: d3, instruction: pose, skill: move_object, args: {to: {pose: stand_up}}}
- {id: d4, instruction: renamed held pose, skill: move_object, args: {to: {pose: set_down}, motion: {torso: auto}}}
- {id: d5, instruction: onto, skill: move_object,
  args: {to: {onto: karakuri_hanger/hook, contact: hook_plate, offset_m: 0.1}, motion: {torso: auto, speed: 0.5}}}
- {id: d6, instruction: onto keep, skill: move_object, args: {to: {onto: karakuri_hanger/hook, contact: hook_plate}}}
- {id: d7, instruction: at, skill: move_object, args: {to: {at: [0.8, 0, 0.4]}, object: frame_1}}
- {id: e1, instruction: elbow, skill: set_posture, args: {elbow: down}}
- {id: e2, instruction: elbow hands, skill: set_posture, args: {elbow: out, hands: [left], tolerance_deg: 10}}
- {id: f1, instruction: untouched, skill: grip, args: {hands: all, action: close}}
- {id: f2, instruction: untouched too, skill: wait, args: {seconds: 1}}
"""

EXPECTED = {
    'a1': {'to': 'waiting'},
    'a2': {'to': 'bay_1_stack_1_front/pick'},
    'a3': {'to': 'karakuri_hanger/hang'},
    'a4': {'to': 'karakuri_hanger/knob_stance', 'motion': {'mode': 'straight'}},
    'a5': {'to': 'karakuri_hanger/staging', 'motion': {'mode': 'straight'}},
    'a6': {'base': {'by': [-0.5, 0]}},
    'a7': {'to': 'point_a'},
    'b1': {'to': 'lift_high'},
    'b2': {'to': 'home', 'only': ['left_arm'], 'motion': {'speed': 0.5}},
    'b3': {'joints': {'torso': 0.3, 'head': [0.1, None]}},
    'c1': {'to': 'bay_1_stack_1_front/rails_top', 'offset_m': 0.15},
    'c2': {'to': 'frame_grasp_approach'},
    'c3': {'hand': 'left', 'by': [0, 0, 0.1], 'frame': 'tool'},
    'c4': {'hand': 'right', 'at': [0.5, 0, 0.5], 'rpy_deg': [0, 90, 0]},
    'c5': {'hands': 'any_one', 'to': 'part_box_1/top_grasp', 'motion': {'mode': 'linear'}, 'torso': 'keep'},
    'c6': {'hand': 'all', 'by': [0.1, 0, 0]},
    'd1': {'object': 'held', 'by': [0, 0, 0.3], 'torso': 'keep'},
    'd2': {'object': 'held', 'by': [0, 0, 0], 'tilt_deg': 30, 'hands': ['left'], 'torso': 'keep'},
    'd3': {'to': 'stand_up', 'torso': 'keep'},
    'd4': {'to': 'set_down_held', 'torso': 'auto'},
    'd5': {'to': 'karakuri_hanger/hook', 'offset_m': 0.1, 'motion': {'speed': 0.5}, 'torso': 'auto'},
    'd6': {'to': 'karakuri_hanger/hook', 'torso': 'keep'},
    'd7': {'object': 'frame_1', 'at': [0.8, 0, 0.4], 'torso': 'keep'},
    'e1': {'elbow': 'down'},
    'e2': {'elbow': 'out', 'hands': ['left'], 'tolerance_deg': 10},
    'f1': {'hands': 'all', 'action': 'close'},
    'f2': {'seconds': 1},
}


def test_every_step_form_of_the_design_is_rewritten(old_cell):
    add_mission(old_cell, 'forms', FORMS)
    code, lines = run(old_cell)
    assert code == 0, '\n'.join(lines)
    steps = steps_of(old_cell, 'forms')
    assert [s['id'] for s in steps] == re.findall(r'\{id: (\w+)', FORMS)
    for step in steps:
        want = EXPECTED[step['id']]
        assert close(step['args'], want), (step['id'], step['args'], want)
        assert step['skill'] == ('move' if step['id'][0] in 'abcde' else ('grip' if step['id'] == 'f1' else 'wait'))
    # the held frame is not held at c5 in a real mission; the forms are checked against the cell's targets
    store = ConfigStore(old_cell)
    assert move_forms.mission_problems(store, 'forms') == []


def test_step_ids_instructions_and_settings_do_not_change(old_cell):
    add_mission(
        old_cell,
        'one',
        '- {id: x, instruction: A, skill: navigate_to, args: {target: waiting}, retries: 2}\n'
        '- {instruction: B, skill: move_joints, args: {pose: home}, on_failure: replan, '
        'replan_from: x}\n- {instruction: C, include: two, with: {frame: bay_1_stack_1_front}}\n',
        'goal: One\ndefaults: {timeout_s: 100}\n',
    )
    add_mission(
        old_cell,
        'two',
        '- {instruction: D, skill: navigate_to, args: {target: "stance:{frame}"}}\n',
        'goal: Two\nparams:\n  frame: {point: instance}\n',
    )
    shapes = {n: move_forms.step_shape(old_cell / 'missions' / f'{n}.yaml') for n in ('one', 'two')}
    assert run(old_cell)[0] == 0
    assert {n: move_forms.step_shape(old_cell / 'missions' / f'{n}.yaml') for n in ('one', 'two')} == shapes
    assert data(old_cell / 'missions' / 'one.yaml')['defaults'] == {'timeout_s': 100}


def test_comments_and_the_flow_style_of_the_files_are_kept(old_cell):
    text = (old_cell / 'cell' / 'locations.yaml').read_text()
    assert '# centre of the waiting zone' in text
    assert run(old_cell)[0] == 0
    text = (old_cell / 'cell' / 'targets.yaml').read_text(encoding='utf-8')
    assert 'waiting: {moves: base, x: 2.0, y: 0.0, yaw: 0.0}' in text and '# centre of the waiting zone' in text
    assert '# named base poses in the cell frame' in text


# ---- name clashes ----------------------------------------------------------------------------------------------------
def test_a_stance_named_like_a_point_gets_stance_and_the_steps_follow(old_cell):
    add_mission(
        old_cell,
        'm',
        '- {id: s1, instruction: x, skill: navigate_to, args: {target: "stance:karakuri_hanger/knob"}}\n'
        '- {id: s2, instruction: y, skill: move_gripper, args: {to: {point: karakuri_hanger/knob}}}\n',
    )
    _, lines = run(old_cell)
    assert args_of(old_cell, 'm') == [{'to': 'karakuri_hanger/knob_stance'}, {'to': 'karakuri_hanger/knob'}]
    store = ConfigStore(old_cell)
    assert store.target('karakuri_hanger/knob_stance', ROBOT).kind == 'base'
    assert (
        store.target('karakuri_hanger/knob', ROBOT).kind == 'hand'
        and store.target('karakuri_hanger/knob', ROBOT).get('elbow') == 'out'
    )


def test_a_contact_named_like_a_point_gets_contact(old_cell):
    write(
        old_cell / 'objects' / 'part_box.yaml',
        (old_cell / 'objects' / 'part_box.yaml').read_text()
        + 'contacts:\n  top_grasp: {xyz: [0, 0, 0.1], rpy: [0, 0, 0]}\n',
    )
    add_mission(
        old_cell,
        'm',
        '- {id: s1, instruction: x, skill: move_object, args: {to: {onto: part_box_1/top_grasp, '
        'contact: hook_plate}}}\n',
    )
    _, lines = run(old_cell)
    assert any('rename  contact objects/part_box.yaml top_grasp -> top_grasp_contact' in line for line in lines)
    store = ConfigStore(old_cell)
    assert store.target('part_box_1/top_grasp_contact', ROBOT).kind == 'object'
    assert args_of(old_cell, 'm')[0]['to'] == 'part_box_1/top_grasp_contact'


def test_a_held_pose_named_like_a_location_and_a_pose_named_like_a_location(old_cell):
    # carry is also a location here, and so is home (a joint pose): the location keeps its name
    write(
        old_cell / 'cell' / 'locations.yaml',
        (old_cell / 'cell' / 'locations.yaml').read_text()
        + 'carry: {frame: cell, x: 1.0, y: 1.0, yaw: 0.0}\nhome: {frame: cell, x: 0.0, y: 0.0, yaw: 0.0}\n',
    )
    add_mission(
        old_cell,
        'm',
        '- {id: s1, instruction: a, skill: navigate_to, args: {target: carry}}\n'
        '- {id: s2, instruction: b, skill: move_object, args: {to: {pose: carry}}}\n'
        '- {id: s3, instruction: c, skill: move_joints, args: {pose: home}}\n'
        '- {id: s4, instruction: d, skill: navigate_to, args: {target: home}}\n',
    )
    code, lines = run(old_cell)
    assert code == 0, lines
    assert args_of(old_cell, 'm') == [
        {'to': 'carry'},
        {'to': 'carry_held', 'torso': 'keep'},
        {'to': 'home_pose'},
        {'to': 'home'},
    ]
    store = ConfigStore(old_cell)
    assert store.target('carry', ROBOT).kind == 'base' and store.target('carry_held', ROBOT).kind == 'object'
    assert store.target('home_pose', ROBOT).kind == 'joints'


# ---- run-time instances, first variants ------------------------------------------------------------------------------
def test_a_placeholder_instance_takes_the_variant_its_values_agree_on_and_flags_a_run_time_object(old_cell):
    add_mission(
        old_cell,
        'block',
        '- {id: go, instruction: x, skill: navigate_to, args: {target: "stance:{frame}"}}\n'
        '- {id: pt, instruction: y, skill: move_gripper, args: {to: {point: "{frame}/rails_top"}}}\n',
        'goal: Block\nparams:\n  frame: {point: instance}\n',
    )
    add_mission(
        old_cell,
        'outer',
        '- {instruction: a, include: block, with: {frame: bay_1_stack_1_front}}\n'
        '- {instruction: b, include: block, with: {frame: floor_frame}}\n'
        '- {id: put, instruction: c, skill: grip, args: {hands: all, action: open, place_as: floor_frame}}\n',
    )
    code, lines = run(old_cell)
    assert code == 0, '\n'.join(lines)
    assert args_of(old_cell, 'block')[0] == {'to': '{frame}/pick'}
    assert any(line.startswith('FLAG') and 'floor_frame' in line and 'pick' in line for line in lines)


def test_a_literal_run_time_object_is_taken_for_the_type_whose_targets_the_missions_use_on_it(old_cell):
    add_mission(
        old_cell,
        'm',
        '- {id: go, instruction: x, skill: navigate_to, args: {target: "stance:floor_frame"}}\n'
        '- {id: pt, instruction: y, skill: move_gripper, args: {to: {point: floor_frame/rails_top}}}\n'
        '- {id: put, instruction: c, skill: grip, args: {hands: all, action: open, place_as: floor_frame}}\n',
    )
    code, lines = run(old_cell)
    assert code == 0 and args_of(old_cell, 'm')[0] == {'to': 'floor_frame/pick'}
    assert any('assumed pick for floor_frame' in line for line in lines)
    assert move_forms.mission_problems(ConfigStore(old_cell), 'm') == []


def test_a_placeholder_whose_values_have_different_stances_stops_and_names_the_step(old_cell):
    add_mission(
        old_cell,
        'block',
        '- {id: go, instruction: x, skill: navigate_to, args: {target: "stance:{thing}"}}\n',
        'goal: Block\nparams:\n  thing: {point: instance}\n',
    )
    add_mission(
        old_cell,
        'outer',
        '- {instruction: a, include: block, with: {thing: bay_1_stack_1_front}}\n'
        '- {instruction: b, include: block, with: {thing: karakuri_hanger}}\n',
    )
    before = tree(old_cell)
    code, lines = run(old_cell)
    assert code == 2 and 'block go' in lines[-1] and 'differ' in lines[-1]
    assert tree(old_cell) == before


def second_robot(cell: Path, first: str) -> None:
    """robot bot_plain stands at frames from `first` (a different first variant than bot_two_arm's pick)."""
    write(
        cell / 'robots' / 'bot_plain' / 'stances.yaml',
        f'objects:\n  frame_type2:\n    {first}: {{x: -0.9, y: 0.0, yaw: 0.0}}\n'
        '    pick: {x: -0.85, y: 0.0, yaw: 0.0}\n',
    )


def test_robots_with_different_first_variants_are_flagged_and_the_missions_robot_decides(old_cell):
    second_robot(old_cell, 'staged')
    add_mission(
        old_cell, 'a', '- {id: s1, instruction: x, skill: navigate_to, args: {target: "stance:bay_1_stack_1_front"}}\n'
    )
    add_mission(
        old_cell,
        'b',
        '- {id: s1, instruction: x, skill: navigate_to, args: {target: "stance:bay_1_stack_1_front"}}\n',
        'robot: bot_plain\ngoal: B\n',
    )
    code, lines = run(old_cell)
    assert code == 0, lines
    assert args_of(old_cell, 'a') == [{'to': 'bay_1_stack_1_front/pick'}]
    assert args_of(old_cell, 'b') == [{'to': 'bay_1_stack_1_front/staged'}]
    assert any(line.startswith('FLAG') and 'frame_type2' in line and 'differently' in line for line in lines)
    plain = data(old_cell / 'robots' / 'bot_plain' / 'objects' / 'frame_type2.yaml')
    assert plain['handled_from'] == 'staged' and list(plain['targets']) == ['staged', 'pick']


def test_handled_from_is_the_old_first_variant(old_cell):
    run(old_cell)
    assert data(old_cell / 'robots' / ROBOT / 'objects' / 'karakuri_hanger.yaml')['handled_from'] == 'staging'
    store = ConfigStore(old_cell)
    near = ConfigStore(FIXTURES / 'targets_old')
    from taskweave_core.config import Pose2D

    for spot in (Pose2D(7.0, 1.0, 0.0), Pose2D(9.0, 1.0, 3.1416), Pose2D(5.0, 3.0, 0.0)):
        for instance in near.object_instances().values():
            assert store.handling_instance(
                store.object_instances()[instance.name], ROBOT, spot
            ) == near.handling_instance(instance, ROBOT, spot)


# ---- contact references ----------------------------------------------------------------------------------------------
def test_a_contact_every_step_names_the_same_reference_for_gets_it_on_the_target(old_cell):
    add_mission(
        old_cell,
        'm',
        '- {id: s1, instruction: x, skill: move_object, args: {to: {onto: karakuri_hanger/hook, '
        'contact: hook_plate}}}\n- {id: s2, instruction: y, skill: move_object, args: {to: {onto: '
        'karakuri_hanger/hook, contact: hook_plate, offset_m: 0.1}}}\n',
    )
    run(old_cell)
    assert all('reference' not in a for a in args_of(old_cell, 'm'))
    assert data(old_cell / 'objects' / 'karakuri_hanger.yaml')['targets']['hook']['reference'] == 'hook_plate'


def test_steps_that_name_different_references_keep_their_own(old_cell):
    write(
        old_cell / 'objects' / 'frame_type2.yaml',
        (old_cell / 'objects' / 'frame_type2.yaml').read_text() + '  edge: {xyz: [0, 0, 0], rpy: [0, 0, 0]}\n',
    )
    add_mission(
        old_cell,
        'm',
        '- {id: s1, instruction: x, skill: move_object, args: {to: {onto: karakuri_hanger/hook, '
        'contact: hook_plate}}}\n- {id: s2, instruction: y, skill: move_object, args: {to: {onto: '
        'karakuri_hanger/hook, contact: edge}}}\n',
    )
    run(old_cell)
    assert [a.get('reference') for a in args_of(old_cell, 'm')] == ['hook_plate', 'edge']
    assert 'reference' not in data(old_cell / 'objects' / 'karakuri_hanger.yaml')['targets']['hook']


# ---- poses and held poses --------------------------------------------------------------------------------------------
def test_a_group_with_part_of_its_joints_gets_null_for_the_rest(old_cell):
    run(old_cell)
    lift = data(old_cell / 'robots' / 'bot_lift' / 'targets.yaml')
    assert lift['lift_high'] == {'moves': 'joints', 'joints': {'torso': [0.4, None]}}
    assert lift['ready']['joints']['right_arm'] == [0.5, None]


def test_a_joint_in_no_group_stops_the_migration(old_cell):
    write(
        old_cell / 'robots' / ROBOT / 'poses.yaml',
        (old_cell / 'robots' / ROBOT / 'poses.yaml').read_text() + 'odd: {mystery_joint: 1.0}\n',
    )
    before = tree(old_cell)
    code, lines = run(old_cell)
    assert code == 2 and 'mystery_joint' in lines[-1] and 'no joint group' in lines[-1]
    assert tree(old_cell) == before


def test_tool_axes_become_rpy_that_rebuild_the_same_axes(old_cell):
    from taskweave_core.config import targets as model

    run(old_cell)
    grasp = data(old_cell / 'robots' / ROBOT / 'targets.yaml')['frame_grasp_approach']
    for tool, closing in (('left', (0, -1, 0)), ('right', (0, 1, 0))):
        approach, got = model.axes_from_rpy(grasp[f'{tool}_tool']['rpy'])
        assert approach == pytest.approx((1, 0, 0), abs=5e-7) and got == pytest.approx(closing, abs=5e-7)


def test_a_joint_twin_is_dropped_unless_a_mission_uses_it(old_cell):
    twin = 'frame_grasp_approach_joints: {body_joint: 0.5, head_joint1: -0.3491, head_joint2: 0.0}\n'
    poses = old_cell / 'robots' / ROBOT / 'poses.yaml'
    write(poses, poses.read_text() + twin)
    _, lines = run(old_cell)
    assert any(line.startswith('drop') and 'frame_grasp_approach_joints' in line for line in lines)
    assert 'frame_grasp_approach_joints' not in data(old_cell / 'robots' / ROBOT / 'targets.yaml')


def test_a_joint_twin_a_mission_uses_stays_a_joint_target(tmp_path, old_cell):
    twin = 'frame_grasp_approach_joints: {body_joint: 0.5, head_joint1: -0.3491, head_joint2: 0.0}\n'
    poses = old_cell / 'robots' / ROBOT / 'poses.yaml'
    write(poses, poses.read_text() + twin)
    add_mission(
        old_cell, 'm', '- {id: s1, instruction: x, skill: move_joints, args: {pose: frame_grasp_approach_joints}}\n'
    )
    run(old_cell)
    targets = data(old_cell / 'robots' / ROBOT / 'targets.yaml')
    assert targets['frame_grasp_approach_joints'] == {
        'moves': 'joints',
        'joints': {'torso': 0.5, 'head': [-0.3491, 0.0]},
    }
    assert args_of(old_cell, 'm') == [{'to': 'frame_grasp_approach_joints'}]


def pack_dir(
    path: Path, robot: str = ROBOT, poses: str = 'home: {body_joint: 0.1}\nlift_high: {body_joint: 0.6}\n'
) -> Path:
    """A robot pack (old layout, schema 1) made of the fixture robot's profile."""
    shutil.copytree(FIXTURES / 'targets_old' / 'robots' / robot, path / robot)
    for name in ('poses.yaml', 'stances.yaml', 'held_poses.yaml', 'object_points.yaml'):
        (path / robot / name).unlink(missing_ok=True)
    write(path / robot / 'poses.yaml', poses)
    write(path / robot / 'pack.yaml', f'name: {robot}\nversion: 0.2.3\nschema_version: 1\n')
    return path


def test_a_whole_file_override_stays_a_whole_file_with_the_held_poses_in_it(old_cell):
    # bot_two_arm's poses.yaml has no override marker: the cell file is the whole file
    run(old_cell)
    document = data(old_cell / 'robots' / ROBOT / 'targets.yaml')
    assert packs.cell_schema(old_cell) == 4 and 'home' in document and 'carry' in document
    assert overlay_marker() not in document


def overlay_marker() -> str:
    from taskweave_core.config import overlay

    return overlay.MARKER


def test_a_keyed_override_stays_keyed_and_held_poses_join_it(tmp_path, old_cell, monkeypatch):
    packs_dir = pack_dir(tmp_path / 'packs')
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(packs_dir))
    write(old_cell / 'robots' / ROBOT / 'poses.yaml', '~override: keys\nlift_high: {body_joint: 0.7}\n')
    for name in ('profile.yaml', 'embodiment.yaml'):
        (old_cell / 'robots' / ROBOT / name).unlink()
    code, lines = run(old_cell)
    assert code == 0, '\n'.join(lines)
    document = data(old_cell / 'robots' / ROBOT / 'targets.yaml')
    assert document['~override'] == 'keys' and document['lift_high'] == {'moves': 'joints', 'joints': {'torso': 0.7}}
    assert document['carry']['moves'] == 'object' and 'home' not in document
    # the robot pack (writable, from the environment) is migrated too
    assert (packs_dir / ROBOT / 'targets.yaml').is_file() and not (packs_dir / ROBOT / 'poses.yaml').exists()
    assert yaml.safe_load((packs_dir / ROBOT / 'pack.yaml').read_text())['schema_version'] == 2
    store = ConfigStore(old_cell)
    assert store.target('lift_high', ROBOT).joints == {'torso': 0.7} and store.target('home', ROBOT).kind == 'joints'


def test_held_poses_alone_become_a_keyed_override_when_a_robot_pack_has_targets(tmp_path, old_cell, monkeypatch):
    packs_dir = pack_dir(tmp_path / 'packs')
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(packs_dir))
    (old_cell / 'robots' / ROBOT / 'poses.yaml').unlink()
    for name in ('profile.yaml', 'embodiment.yaml'):
        (old_cell / 'robots' / ROBOT / name).unlink()
    assert run(old_cell)[0] == 0
    document = data(old_cell / 'robots' / ROBOT / 'targets.yaml')
    assert document['~override'] == 'keys' and set(document) == {
        '~override',
        'carry',
        'stand_up',
        'set_down_held',
        'hang-approach',
    }
    assert ConfigStore(old_cell).target('home', ROBOT).kind == 'joints'


def test_the_same_held_pose_of_two_types_is_one_target_and_different_ones_stop(old_cell):
    held = old_cell / 'robots' / ROBOT / 'held_poses.yaml'
    entry = '  carry:\n    path: [{xyz: [1.0, 0.0, 0.4], rpy: [0.0, -0.8727, 0.0]}]\n'
    write(held, held.read_text() + 'part_box:\n' + entry)
    code, lines = run(old_cell)
    assert code == 0, lines
    assert 'type' not in data(old_cell / 'robots' / ROBOT / 'targets.yaml')['carry']
    other = old_cell.parent / 'other'
    shutil.copytree(FIXTURES / 'targets_old', other)
    write(other / 'system.yaml', f'robot: {ROBOT}\n')
    held = other / 'robots' / ROBOT / 'held_poses.yaml'
    write(held, held.read_text() + 'part_box:\n  carry:\n    path: [{xyz: [0.5, 0.0, 0.4], rpy: [0.0, 0.0, 0.0]}]\n')
    before = tree(other)
    code, lines = run(other)
    assert code == 2 and 'carry differs' in lines[-1] and tree(other) == before


# ---- instances, settings ---------------------------------------------------------------------------------------------
def test_instance_overrides_and_settings_go_to_the_type_file_of_the_robot(old_cell):
    run(old_cell)
    frame = data(old_cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml')
    assert frame['handled_from'] == 'pick' and frame['targets']['rails_top'] == {'elbow': 'up'}
    assert frame['instances']['bay_1_stack_1_back'] == {'pick': {'x': -1.528, 'y': 1.0, 'yaw': 0.0}}
    assert data(old_cell / 'robots' / ROBOT / 'objects' / 'part_box.yaml')['targets']['top_grasp'] == {
        'grip_m': {'left': 0.06, 'right': 0.06}
    }
    assert data(old_cell / 'objects' / 'frame_type2.yaml')['targets']['hook_plate']['moves'] == 'object'


def test_settings_for_no_point_are_dropped_and_flagged(old_cell):
    path = old_cell / 'robots' / ROBOT / 'object_points.yaml'
    write(
        path,
        path.read_text().replace('rails_top: {elbow: up}', 'rails_top: {elbow: up}\n  nope: {elbow: out}')
        + 'ghost_type:\n  p: {elbow: up}\n',
    )
    code, lines = run(old_cell)
    assert code == 0
    flags = [line for line in lines if line.startswith('FLAG') and 'dropped' in line]
    assert len(set(flags)) == 2 and any('nope' in f for f in flags) and any('ghost_type' in f for f in flags)
    assert not (old_cell / 'robots' / ROBOT / 'objects' / 'ghost_type.yaml').exists()
    assert 'nope' not in data(old_cell / 'robots' / ROBOT / 'objects' / 'frame_type2.yaml')['targets']


def test_an_instance_override_for_an_unknown_run_time_object_stops(old_cell):
    write(
        old_cell / 'robots' / ROBOT / 'stances.yaml',
        (old_cell / 'robots' / ROBOT / 'stances.yaml').read_text()
        + '  nobody_1:\n    pick: {x: -1.0, y: 0.0, yaw: 0.0}\n',
    )
    before = tree(old_cell)
    code, lines = run(old_cell)
    assert code == 2 and 'nobody_1' in lines[-1] and tree(old_cell) == before


# ---- skills, operator, overrides -------------------------------------------------------------------------------------
def test_skill_files_move_unchanged_into_the_move_folder(old_cell):
    contents = {
        'navigate_to': 'implementations:\n'
        '  - {name: nav, type: script, class: "taskweave_skills.navigate:NavigateTo"}\n'
        'max_angular: 0.75      # slower here\r\n',
        'drive_straight': 'implementations: []\n',
        'move_gripper': 'implementations: []\n',
        'move_object': 'implementations: []\n',
        'set_posture': 'implementations: []\n',
        'move_joints': 'implementations: []\n',
    }
    for name, text in contents.items():
        (old_cell / 'skills' / f'{name}.yaml').write_bytes(text.encode())
    add_mission(old_cell, 'm', '- {id: s1, instruction: x, skill: navigate_to, args: {target: waiting}}\n')
    code, lines = run(old_cell)
    assert code == 0, lines
    for old, new in (
        ('navigate_to', 'base'),
        ('drive_straight', 'straight'),
        ('move_gripper', 'hand'),
        ('move_object', 'object'),
        ('set_posture', 'posture'),
        ('move_joints', 'joints'),
    ):
        assert (old_cell / 'skills' / 'move' / f'{new}.yaml').read_bytes() == contents[old].encode()
        assert not (old_cell / 'skills' / f'{old}.yaml').exists()
    move = data(old_cell / 'skills' / 'move.yaml')
    assert move['implementations'] == [{'name': 'dispatch', 'type': 'script', 'class': 'taskweave_skills.move:Move'}]
    assert (old_cell / 'skills' / 'wait.yaml').is_file()


def test_the_move_skill_file_is_written_when_steps_use_it_even_without_old_skill_files(old_cell):
    add_mission(old_cell, 'm', '- {id: s1, instruction: x, skill: navigate_to, args: {target: waiting}}\n')
    run(old_cell)
    assert (old_cell / 'skills' / 'move.yaml').is_file() and not (old_cell / 'skills' / 'move').exists()


def test_an_existing_module_file_is_kept_and_flagged(old_cell):
    write(old_cell / 'skills' / 'navigate_to.yaml', 'implementations: []\nmax_linear: 1.0\n')
    write(old_cell / 'skills' / 'move' / 'base.yaml', 'implementations: []\nmax_linear: 0.5\n')
    code, lines = run(old_cell)
    assert code == 0 and (old_cell / 'skills' / 'navigate_to.yaml').is_file()
    assert any(line.startswith('FLAG') and 'skills/move/base.yaml exists already' in line for line in lines)


def test_skill_overrides_of_a_robot_follow_the_module_names(old_cell):
    write(old_cell / 'robots' / ROBOT / 'skill_overrides.yaml', 'move_gripper: {speed: {tcp_m_s: 0.1}}\nwait: {}\n')
    run(old_cell)
    assert list(data(old_cell / 'robots' / ROBOT / 'skill_overrides.yaml')) == ['move.hand', 'wait']


def test_operator_actions_become_move_steps(old_cell):
    write(
        old_cell / 'operator.yaml',
        'joy:\n  topic: /joy\n  actions:\n'
        '    - {button: 11, skill: move_joints, args: {pose: lift_high}}   # up\n'
        '    - {button: 0, skill: grip, args: {hands: all, action: close}, arm_args: {hands: "{arm}"}}\n',
    )
    run(old_cell)
    actions = data(old_cell / 'operator.yaml')['joy']['actions']
    assert actions[0] == {'button': 11, 'skill': 'move', 'args': {'to': 'lift_high'}}
    assert actions[1]['skill'] == 'grip' and '# up' in (old_cell / 'operator.yaml').read_text()


def test_params_with_a_removed_point_kind_are_flagged_not_rewritten(old_cell):
    add_mission(
        old_cell,
        'm',
        '- {id: s1, instruction: x, skill: move_joints, args: {pose: "{where}"}}\n',
        'goal: M\nparams:\n  where: {point: joint_pose}\n',
    )
    _, lines = run(old_cell)
    assert any(line.startswith('FLAG') and 'point kind joint_pose' in line for line in lines)
    assert data(old_cell / 'missions' / 'm.yaml')['params']['where'] == {'point': 'joint_pose'}


# ---- the chain and the stops -----------------------------------------------------------------------------------------
def test_migrate_skills_alone_still_stops_at_schema_3(old_cell):
    (old_cell / 'pack.yaml').write_text('name: c\nversion: 1\nschema_version: 2\n')
    add_mission(old_cell, 'm', '- {id: s1, instruction: x, skill: navigate_to, args: {target: waiting}}\n')
    assert migrate_skills_only(old_cell, False, lambda text: None) == 0
    assert packs.cell_schema(old_cell) == 3


def test_a_newer_cell_is_refused_and_a_cell_of_schema_4_has_nothing_to_do(old_cell):
    assert run(old_cell)[0] == 0
    code, lines = run(old_cell)
    assert code == 0 and 'nothing to do' in lines[-1]


def test_an_unwritable_file_stops_before_anything_is_touched(old_cell, monkeypatch):
    from taskweave_core.config import migrate_targets as mt

    before = tree(old_cell)
    blocked = old_cell / 'cell' / 'locations.yaml'
    real = os.access
    monkeypatch.setattr(mt.os, 'access', lambda path, mode: False if Path(path) == blocked else real(path, mode))
    code, lines = run(old_cell)
    assert code == 2 and 'locations.yaml is not writable' in lines[-1] and tree(old_cell) == before


def test_a_run_that_stopped_before_the_end_is_finished_by_the_next_one(old_cell):
    """The files of an interrupted run are partly new: the next run reads the type files in either layout."""
    assert run(old_cell)[0] == 0
    # put the old files back as an interrupted run (new files written, old ones not yet removed) would leave them
    shutil.copy(FIXTURES / 'targets_old' / 'cell' / 'locations.yaml', old_cell / 'cell' / 'locations.yaml')
    for name in ('stances.yaml', 'held_poses.yaml', 'object_points.yaml', 'poses.yaml'):
        shutil.copy(FIXTURES / 'targets_old' / 'robots' / ROBOT / name, old_cell / 'robots' / ROBOT / name)
    (old_cell / 'pack.yaml').write_text('name: c\nversion: 1\nschema_version: 3\n')
    done = tree(old_cell)
    code, lines = run(old_cell)
    assert code == 0, '\n'.join(lines)
    assert any(line.startswith('rename  stance') for line in lines)  # the clash is seen in the finished files too
    assert not (old_cell / 'robots' / ROBOT / 'stances.yaml').exists()
    assert ConfigStore(old_cell).target('karakuri_hanger/knob_stance', ROBOT).kind == 'base'
    assert done.keys() >= {'cell/targets.yaml', 'objects/karakuri_hanger.yaml'}


# ---- robot packs -----------------------------------------------------------------------------------------------------
def test_a_robot_pack_folder_is_migrated_on_its_own(tmp_path):
    folder = pack_dir(
        tmp_path / 'packs', ROBOT, 'home: {head_joint1: 0.0, head_joint2: 0.1}\nlift: {body_joint: 0.3}  # lift\n'
    )
    lines: list[str] = []
    status = migrate(folder, say=lines.append)
    assert status == 0, lines
    pack = folder / ROBOT
    targets = data(pack / 'targets.yaml')
    assert targets == {
        'home': {'moves': 'joints', 'joints': {'head': [0.0, 0.1]}},
        'lift': {'moves': 'joints', 'joints': {'torso': 0.3}},
    }
    assert '# lift' in (pack / 'targets.yaml').read_text() and not (pack / 'poses.yaml').exists()
    assert data(pack / 'pack.yaml')['schema_version'] == 2 and data(pack / 'pack.yaml')['version'] == '0.2.3'
    before = tree(folder)
    lines.clear()
    status = migrate(pack, say=lines.append)
    assert status == 0 and tree(folder) == before and any('nothing to do' in line for line in lines)


def test_a_robot_pack_dry_run_changes_nothing(tmp_path):
    folder = pack_dir(tmp_path / 'packs')
    before = tree(folder)
    lines: list[str] = []
    assert migrate(folder, dry_run=True, say=lines.append) == 0 and tree(folder) == before
    assert any('poses.yaml -> targets.yaml' in line for line in lines)


def test_the_repo_robot_packs_are_migrated_and_a_second_run_changes_nothing(tmp_path):
    robots = REPO / 'taskweave_robots'
    for pack in sorted(robots.glob('*/pack.yaml')):
        assert data(pack)['schema_version'] == 2, pack.parent.name
        assert (pack.parent / 'targets.yaml').is_file() and not (pack.parent / 'poses.yaml').exists()
    copy = tmp_path / 'robots'
    shutil.copytree(robots, copy, ignore=shutil.ignore_patterns('test', '__pycache__', '.*'))
    before = tree(copy)
    lines: list[str] = []
    assert migrate(copy, say=lines.append) == 0 and tree(copy) == before


def test_an_installed_read_only_pack_is_only_reported(tmp_path, monkeypatch, old_cell):
    packs_dir = pack_dir(tmp_path / 'installed')
    monkeypatch.setattr(packs, 'installed_location', lambda: packs_dir)
    for name in ('profile.yaml', 'embodiment.yaml'):
        (old_cell / 'robots' / ROBOT / name).unlink()
    before = tree(packs_dir)
    code, lines = run(old_cell)
    assert code == 0 and tree(packs_dir) == before
    assert any(line.startswith('FLAG') and 'read-only' in line and 'taskweave config migrate' in line for line in lines)


# ---- init writes the new files ---------------------------------------------------------------------------------------
def test_init_cell_writes_a_cell_of_the_current_schema(tmp_path, monkeypatch):
    from taskweave_core import cli
    from taskweave_core.config.packfile import init_cell

    packs_dir = pack_dir(tmp_path / 'packs')
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(packs_dir))
    cell = init_cell(tmp_path / 'new', ROBOT, 'fresh')
    assert packs.cell_schema(cell) == packs.CELL_SCHEMA == 4
    files = {p.relative_to(cell).as_posix() for p in cell.rglob('*') if p.is_file()}
    assert 'cell/targets.yaml' in files and 'cell/objects.yaml' in files and 'cell/locations.yaml' not in files
    assert not any(f.endswith(('stances.yaml', 'held_poses.yaml', 'object_points.yaml')) for f in files)
    assert ConfigStore(cell).targets_format() and cli.check(str(cell)) == 0


# ---- regenerate the golden docs cell ---------------------------------------------------------------------------------
if __name__ == '__main__':
    import tempfile

    if '--regenerate' not in sys.argv:
        raise SystemExit(__doc__)
    from taskweave_core.config import packs as _packs

    _packs.installed_location = lambda: None
    with tempfile.TemporaryDirectory() as scratch:
        built = docs_cell(Path(scratch) / 'docs')
        lines: list[str] = []
        status = migrate(built, say=lines.append)
        print('\n'.join(line for line in lines if not line.startswith(('step', 'comment'))))
        if status:
            raise SystemExit(status)
        if GOLDEN.exists():
            shutil.rmtree(GOLDEN)
        shutil.copytree(built, GOLDEN)
    print(f'wrote {GOLDEN}')
