"""Targets in the pack check, the catalogue and the pack schema numbers."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigError, ConfigStore, catalogue, packs

FIXTURES = Path(__file__).parent / 'fixtures'
ROBOT = 'bot_two_arm'
CELL = 'cell/targets.yaml'
ROBOT_TARGETS = f'robots/{ROBOT}/targets.yaml'
ROBOT_FRAME = f'robots/{ROBOT}/objects/frame_type2.yaml'
OBJECT_FRAME = 'objects/frame_type2.yaml'


@pytest.fixture
def cell(tmp_path) -> Path:
    target = tmp_path / 'cell'
    shutil.copytree(FIXTURES / 'targets_new', target)
    return target


def append(path: Path, text: str) -> None:
    path.write_text(path.read_text(encoding='utf-8') + text, encoding='utf-8')


def replace(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding='utf-8')
    assert old in text, old
    path.write_text(text.replace(old, new), encoding='utf-8')


def problems(cell: Path, path: str, pack: str = 'cell') -> list[tuple[str, str, str]]:
    """(severity, where, message) of one file."""
    return [(p['severity'], p['where'], p['message']) for p in ConfigStore(cell).validate_file(path, None, pack)]


def messages(cell: Path, path: str) -> list[str]:
    return [m for _, _, m in problems(cell, path)]


def errors(cell: Path, path: str) -> list[str]:
    return [m for s, _, m in problems(cell, path) if s == 'error']


def warnings(cell: Path, path: str) -> list[str]:
    return [m for s, _, m in problems(cell, path) if s == 'warning']


# ---- a cell in the new layout passes --------------------------------------------------------------
def test_check_passes_on_the_new_format_cell():
    store = ConfigStore(FIXTURES / 'targets_new')
    assert store.check_pack() == []
    from taskweave_core.config.check import checked_files

    kinds = {entry['kind'] for entry, _ in checked_files(store)}
    assert {'cell_targets', 'robot_objects', 'robot_targets', 'object_type', 'cell_pack'} <= kinds


def test_check_passes_on_the_old_format_twin():
    assert ConfigStore(FIXTURES / 'targets_old').check_pack() == []


def test_the_command_line_check_passes_on_the_new_format_cell(capsys):
    from taskweave_core.cli import main

    assert main(['check', str(FIXTURES / 'targets_new')]) == 0


# ---- the cross rules ------------------------------------------------------------------------------
def test_a_name_in_the_cell_and_the_robots_targets_is_an_error_in_both_files(cell):
    append(cell / ROBOT_TARGETS, 'set_down: {moves: base, x: 0.0, y: 0.0, yaw: 0.0}\n')
    expected = "target 'set_down' is in cell/targets.yaml and robots/bot_two_arm/targets.yaml: rename one"
    assert expected in errors(cell, CELL) and expected in errors(cell, ROBOT_TARGETS)


def test_a_base_and_a_hand_target_of_one_name_on_a_type_is_an_error(cell):
    replace(cell / ROBOT_FRAME, '  rails_top: {elbow: up}', '  rails_top: {moves: base, x: 0.0, y: 0.0, yaw: 0.0}')
    wanted = (
        "target 'rails_top' moves hand in objects/frame_type2.yaml but base in robots/bot_two_arm/objects/"
        'frame_type2.yaml: they must agree'
    )
    assert any(wanted in m for m in errors(cell, ROBOT_FRAME)) and any(wanted in m for m in errors(cell, OBJECT_FRAME))


def test_settings_for_an_unknown_target_are_an_error(cell):
    replace(cell / ROBOT_FRAME, '  rails_top: {elbow: up}', '  rails_top: {elbow: up}\n  ghost: {elbow: up}')
    assert any("settings for an unknown target 'ghost'" in m for m in errors(cell, ROBOT_FRAME))
    replace(cell / ROBOT_FRAME, 'bay_1_stack_1_back: {pick:', 'bay_1_stack_1_back: {nothing: {elbow: up}, pick:')
    assert any("settings for an unknown target 'nothing'" in m for m in errors(cell, ROBOT_FRAME))


def test_handled_from_must_name_a_base_target_of_the_type(cell):
    replace(cell / ROBOT_FRAME, 'handled_from: pick', 'handled_from: rails_top')
    assert any("handled_from 'rails_top' moves hand, it must be a base target" in m for m in errors(cell, ROBOT_FRAME))
    replace(cell / ROBOT_FRAME, 'handled_from: rails_top', 'handled_from: nowhere')
    assert any("handled_from 'nowhere' is not a target of frame_type2 objects" in m for m in errors(cell, ROBOT_FRAME))


def test_a_symmetric_type_without_handled_from_is_a_warning(cell):
    replace(cell / ROBOT_FRAME, 'handled_from: pick', '# nothing')
    assert any('yaw_symmetry_deg, but handled_from names no base target' in m for m in warnings(cell, ROBOT_FRAME))
    assert errors(cell, ROBOT_FRAME) == []


def test_the_type_and_reference_of_an_object_target_must_exist(cell):
    replace(cell / 'objects' / 'karakuri_hanger.yaml', 'reference: hook_plate', 'reference: nothing')
    assert any(
        "no object target 'nothing' on any object type" in m for m in errors(cell, 'objects/karakuri_hanger.yaml')
    )
    replace(
        cell / ROBOT_TARGETS,
        '  type: frame_type2\n  path:\n  - {xyz: [1.0, 0.0, 0.4]',
        '  type: crate\n  path:\n  - {xyz: [1.0, 0.0, 0.4]',
    )
    assert any("object type 'crate' does not exist" in m for m in warnings(cell, ROBOT_TARGETS))
    append(
        cell / ROBOT_TARGETS,
        'reaching: {moves: object, type: frame_type2, reference: hook_plate, xyz: [1, 0, 0], '
        'rpy: [0, 0, 0]}\nbad_ref: {moves: object, type: frame_type2, reference: knob, '
        'xyz: [1, 0, 0], rpy: [0, 0, 0]}\n',
    )
    found = errors(cell, ROBOT_TARGETS)
    assert any("no object target 'knob' on frame_type2" in m for m in found)
    assert not any("'hook_plate'" in m for m in found)


def test_joint_groups_lengths_and_ranges_are_checked_against_the_robot(cell):
    append(
        cell / ROBOT_TARGETS,
        'a: {moves: joints, joints: {neck: [0, 0]}}\n'
        'b: {moves: joints, joints: {head: [0.1]}}\n'
        'c: {moves: joints, joints: {torso: 0.9}}\n'
        'd: {moves: joints, joints: {left_arm: 0.1}}\n',
    )
    found = errors(cell, ROBOT_TARGETS)
    assert any("joint group 'neck': this robot has no such group" in m for m in found)
    assert any("joint group 'head' has 2 joints" in m for m in found)
    assert any('body_joint' in m and 'outside' in m for m in found)
    assert any("joint group 'left_arm' has 7 joints" in m for m in found)


def test_null_entries_are_checked_only_where_they_set_a_value(cell):
    append(
        cell / ROBOT_TARGETS,
        'part: {moves: joints, joints: {head: [null, 0.5], left_arm: [null, null, null, null, null, null, 1.0]}}\n',
    )
    assert errors(cell, ROBOT_TARGETS) == []
    append(cell / ROBOT_TARGETS, 'worse: {moves: joints, joints: {head: [null, 1.5]}}\n')
    assert any('head_joint2' in m and 'outside' in m for m in errors(cell, ROBOT_TARGETS))


def test_auto_is_for_the_torso_with_a_pose_to_search_for(cell):
    append(cell / ROBOT_TARGETS, 'only_auto: {moves: joints, joints: {torso: auto}}\n')
    assert any('auto needs a hand or object pose' in m for m in errors(cell, ROBOT_TARGETS))
    append(cell / ROBOT_TARGETS, 'head_auto: {moves: object, xyz: [1, 0, 0], rpy: [0, 0, 0], joints: {head: auto}}\n')
    assert any('joints.head: auto is for the torso only' in m for m in errors(cell, ROBOT_TARGETS))


def test_auto_without_any_torso_search_or_lift_is_an_error(cell):
    path = cell / 'robots' / 'bot_plain' / 'targets.yaml'
    path.write_text('pose: {moves: hand, left_tool: {xyz: [0, 0, 0], rpy: [0, 0, 0]}, joints: {torso: auto}}\n')
    assert any('no torso search or lift' in m for m in errors(cell, 'robots/bot_plain/targets.yaml'))
    path.write_text('pose: {moves: hand, left_tool: {xyz: [0, 0, 0], rpy: [0, 0, 0]}, joints: {torso: keep}}\n')
    assert errors(cell, 'robots/bot_plain/targets.yaml') == []
    path.write_text('pose: {moves: hand, left_tool: {xyz: [0, 0, 0], rpy: [0, 0, 0]}, joints: {torso: auto}}\n')
    assert errors(cell, 'robots/bot_two_arm/targets.yaml') == []  # a lift joint: the fallback heights


def test_a_target_has_the_keys_of_its_kind(cell):
    append(
        cell / CELL,
        'a: {moves: base, xyz: [0, 0, 0], rpy: [0, 0, 0]}\n'
        'b: {moves: hand, x: 1.0, y: 0.0, yaw: 0.0, left_tool: {xyz: [0, 0, 0], rpy: [0, 0, 0]}}\n'
        'c: {moves: base, x: 1.0, y: 0.0, yaw: 0.0, path: [{x: 1, y: 1, yaw: 0}]}\n'
        'd: {moves: base, x: 1.0}\n'
        'e: {moves: base, frame: base, x: 1.0, y: 0.0, yaw: 0.0}\n'
        'f: {moves: joints, joints: {torso: 0.1}, x: 0.0}\n'
        'g: {moves: object, xyz: [0, 0, 0], rpy: [0, 0, 0], elbow: up}\n',
    )
    found = errors(cell, CELL)
    for part in (
        'does not apply to a target that moves the base',
        'x does not apply to a target that moves a hand',
        'path and a single pose',
        'pose needs y, yaw',
        "frame 'base' does not fit this file",
        'x does not apply to a target that moves joints',
        'elbow does not apply to a target that moves a held object',
    ):
        assert any(part in m for m in found), part


def test_a_joints_target_has_no_frame_in_the_cell_file_or_in_an_object_frame(cell):
    append(cell / CELL, 'lift: {moves: joints, joints: {torso: 0.1}}\n')
    assert any('has no frame' in m for m in errors(cell, CELL))
    replace(
        cell / ROBOT_FRAME,
        '  rails_top: {elbow: up}',
        '  rails_top: {elbow: up}\n  lift: {moves: joints, joints: {torso: 0.1}}',
    )
    assert any('has no object frame' in m or 'has no frame' in m for m in errors(cell, ROBOT_FRAME))


def test_waypoints_have_the_pose_form_of_the_kind(cell):
    append(cell / CELL, 'a: {moves: base, path: [{x: 1, y: 1}]}\n')
    assert any('path[1] needs yaw' in m for m in errors(cell, CELL))
    append(
        cell / ROBOT_TARGETS,
        'b: {moves: object, path: [{xyz: [0, 0, 0]}]}\nc: {moves: hand, path: [{xyz: [0, 0, 0]}]}\n',
    )
    found = errors(cell, ROBOT_TARGETS)
    assert any('path[1] needs rpy' in m for m in found)
    assert any('path[1] needs a tool pose' in m for m in found)


def test_the_look_at_key_is_reserved_and_not_accepted_yet(cell):
    append(cell / ROBOT_TARGETS, 'gaze: {moves: joints, joints: {head: [0, 0]}, look_at: waiting}\n')
    assert errors(cell, ROBOT_TARGETS) and 'look_at' in errors(cell, ROBOT_TARGETS)[0]


def test_instance_names_must_exist_or_be_put_down_by_a_mission(cell):
    append(cell / ROBOT_FRAME, '  bay_9_stack_1_front: {pick: {x: -1.0}}\n  floor_frame: {pick: {x: -1.0}}\n')
    found = warnings(cell, ROBOT_FRAME)
    assert any("placed object 'bay_9_stack_1_front' does not exist" in m for m in found)
    assert any("placed object 'floor_frame' does not exist" in m for m in found)
    (cell / 'missions' / 'drop.yaml').write_text(
        'goal: put one down\nsteps:\n  - {id: put, instruction: put it down, skill: wait, '
        'args: {seconds: 0, place_as: floor_frame}}\n'
    )
    found = warnings(cell, ROBOT_FRAME)
    assert not any("'floor_frame'" in m for m in found) and any('bay_9_stack_1_front' in m for m in found)
    append(cell / ROBOT_FRAME, '  part_box_1: {pick: {x: -1.0}}\n')
    assert any("'part_box_1' is a part_box, not a frame_type2" in m for m in warnings(cell, ROBOT_FRAME))


def test_tools_and_gripper_settings_are_checked_against_the_robot(cell):
    replace(
        cell / 'robots' / ROBOT / 'objects' / 'part_box.yaml',
        'grip_m: {left: 0.06, right: 0.06}',
        'grip_m: {left: 0.06, middle: 0.06}',
    )
    assert any("robot has no tool 'middle'" in m for m in warnings(cell, 'robots/bot_two_arm/objects/part_box.yaml'))
    replace(cell / 'objects' / 'part_box.yaml', 'grip_width_m: 0.06', 'grip_width_m: 0.3')
    assert any('exceeds the' in m and 'stroke' in m for m in warnings(cell, 'objects/part_box.yaml'))
    replace(cell / 'objects' / 'part_box.yaml', 'tool_type: parallel_gripper', 'tool_type: suction')
    assert any('needs a suction' in m for m in warnings(cell, 'objects/part_box.yaml'))


def test_a_robot_objects_file_of_an_unknown_type_is_a_warning(cell):
    shutil.copy(cell / ROBOT_FRAME, cell / 'robots' / ROBOT / 'objects' / 'crate.yaml')
    assert any("object type 'crate' does not exist" in m for m in warnings(cell, f'robots/{ROBOT}/objects/crate.yaml'))


# ---- the catalogue --------------------------------------------------------------------------------
def test_the_catalogue_knows_the_four_target_files():
    assert catalogue.kind_of('cell/targets.yaml').kind == 'cell_targets'
    assert catalogue.kind_of('objects/frame_type2.yaml').kind == 'object_type'
    assert catalogue.kind_of('robots/bot_two_arm/objects/frame_type2.yaml').kind == 'robot_objects'
    assert catalogue.kind_of('robots/bot_two_arm/targets.yaml').kind == 'robot_targets'
    assert catalogue.kind_of('targets.yaml', 'robot:bot_two_arm').kind == 'robot_targets'
    assert catalogue.kind_of('cell/locations.yaml').kind == 'locations'  # the old kinds stay for the old layout
    kinds = {k['kind'] for k in ConfigStore(FIXTURES / 'targets_new').file_kinds()}
    assert {'cell_targets', 'robot_objects', 'robot_targets', 'object_type', 'locations', 'stances'} <= kinds


def test_new_files_of_the_new_kinds_can_be_created(cell):
    assert catalogue.new_file('cell_targets') == ('cell/targets.yaml', {})
    assert catalogue.new_file('robot_objects', 'crate', ROBOT) == (f'robots/{ROBOT}/objects/crate.yaml', {})


def test_files_lists_the_target_files_with_their_kinds():
    files = {(f['path'], f['pack']): f for f in ConfigStore(FIXTURES / 'targets_new').files(hashes=False)}
    assert files[('cell/targets.yaml', 'cell')]['kind'] == 'cell_targets'
    assert files[(ROBOT_FRAME, 'cell')]['schema_id'] == 'robot_objects' and files[(ROBOT_FRAME, 'cell')]['editable']
    assert (
        files[(ROBOT_TARGETS, 'cell')]['kind'] == 'robot_targets'
        and files[(ROBOT_TARGETS, 'cell')]['source'] == 'override'
    )


@pytest.mark.parametrize('kind', ['cell_targets', 'robot_targets', 'robot_objects', 'object_type'])
def test_the_json_schemas_of_the_target_files_export(kind):
    schema = ConfigStore(FIXTURES / 'targets_new').json_schema(kind)
    assert schema['$id'] == f'taskweave/{kind}'


def test_the_schemas_carry_the_dynamic_enum_hints():
    store = ConfigStore(FIXTURES / 'targets_new')
    cell = store.json_schema('cell_targets')['additionalProperties']['properties']
    assert (
        cell['joints']['x-taskweave-enum-keys'] == 'joint_groups' and cell['type']['x-taskweave-enum'] == 'target_types'
    )
    assert cell['reference']['x-taskweave-enum'] == 'object_targets'
    assert store.json_schema('robot_objects')['properties']['handled_from']['x-taskweave-enum'] == 'base_targets_of'


def test_the_new_enums_have_values():
    store = ConfigStore(FIXTURES / 'targets_new')
    assert 'waiting' in store.enum_values('targets') and 'carry' in store.enum_values('targets')
    assert 'bay_1_stack_1_front/pick' not in store.enum_values('targets')
    assert {'hook_plate', 'pick', 'rails_top'} <= set(store.enum_values('object_targets', ROBOT))
    assert store.enum_values('target_types') == ['frame_type2', 'karakuri_hanger', 'part_box']
    assert set(store.enum_values('base_targets_of', ROBOT)) == {'pick', 'staging', 'hang', 'knob_stance'}
    assert {'torso', 'head', 'left_arm', 'right_arm'} <= set(store.enum_values('joint_groups', ROBOT))
    old = ConfigStore(FIXTURES / 'targets_old')
    assert old.enum_values('targets') == [] and old.enum_values('stances', ROBOT)


# ---- the schema numbers ---------------------------------------------------------------------------
def test_the_schema_numbers():
    assert (packs.CELL_SCHEMA, packs.ROBOT_SCHEMA) == (4, 2)
    assert (packs.CELL_SCHEMA_OLDEST, packs.ROBOT_SCHEMA_OLDEST) == (3, 1)  # until the migration to 4 exists


def write_pack(folder: Path, schema: int, name='cell') -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'pack.yaml').write_text(f'name: {name}\nversion: 1\nschema_version: {schema}\n')
    (folder / 'cell').mkdir(exist_ok=True)
    return folder


@pytest.mark.parametrize('schema', [3, 4])
def test_cell_schemas_three_and_four_are_read(tmp_path, schema):
    assert ConfigStore(write_pack(tmp_path / 'c', schema)).targets_format() == (schema == 4)


def test_cell_schemas_below_and_above_are_refused(tmp_path):
    with pytest.raises(
        ConfigError, match='cell pack schema 2, this taskweave needs 3: run taskweave config migrate-skills'
    ):
        ConfigStore(write_pack(tmp_path / 'a', 2))
    with pytest.raises(ConfigError, match='cell pack schema 5, this taskweave reads schema 4: update taskweave'):
        ConfigStore(write_pack(tmp_path / 'b', 5))


def test_the_pack_check_accepts_the_cell_and_robot_schemas_that_are_read(tmp_path):
    cell = write_pack(tmp_path / 'c', 4)
    assert ConfigStore(cell).validate_file('pack.yaml') == []
    for schema, ok in ((1, True), (2, True), (3, False)):
        pack = write_pack(tmp_path / f'robot{schema}', schema, 'bot')
        result = ConfigStore(cell).validate_file('pack.yaml', (pack / 'pack.yaml').read_text(), 'robot:bot')
        assert (result == []) == ok, (schema, result)
    assert (
        'reads 3 to 4'
        in ConfigStore(cell).validate_file('pack.yaml', 'name: c\nversion: 1\nschema_version: 5\n')[0]['message']
    )


@pytest.mark.parametrize('schema,ok', [(1, True), (2, True), (3, False)])
def test_robot_pack_schemas_one_and_two_are_read(tmp_path, schema, ok):
    cell = copy_with_pack(tmp_path, schema)
    if ok:
        assert ConfigStore(cell).robot_packs('bot_a')
    else:
        with pytest.raises(
            ConfigError, match='robot pack schema 3, this taskweave reads schema 1 to 2: update taskweave'
        ):
            ConfigStore(cell).robot_packs('bot_a')


def copy_with_pack(tmp_path: Path, schema: int) -> Path:
    cell = write_pack(tmp_path / 'c', 4)
    pack = tmp_path / 'packs' / 'bot_a'
    pack.mkdir(parents=True)
    (pack / 'pack.yaml').write_text(f'name: bot_a\nversion: 1\nschema_version: {schema}\n')
    (cell / 'system.yaml').write_text('robot_packs: [../packs]\n')
    return cell


def test_robot_pack_files_include_the_targets_file():
    assert 'targets.yaml' in packs.ROBOT_PACK_FILES and 'poses.yaml' in packs.ROBOT_PACK_FILES
    assert packs.CELL_ROBOT_FOLDERS == ('objects',)


def test_the_migration_chain_stops_where_it_can_go(tmp_path, capsys):
    from taskweave_core.config import migrate

    cell = write_pack(tmp_path / 'c', 3)
    assert migrate.migrate(cell, say=lambda *a: None) == 0 and migrate.LATEST == 3
