"""The file catalogue, schema export and structured validation (config/catalogue.py, config/check.py)."""

import json
import shutil
from pathlib import Path

import pytest
import yaml
from jsonschema import Draft202012Validator

from taskweave_core import cli
from taskweave_core.config import ConfigStore, catalogue, schemas

REPO = Path(__file__).resolve().parents[2]
EXAMPLE = REPO / 'docs' / 'examples' / 'cell'
ROBOTS = REPO / 'taskweave_robots'
S1 = ROBOTS / 'galbot_s1'


@pytest.fixture
def example(tmp_path) -> Path:
    target = tmp_path / 'cell'
    shutil.copytree(EXAMPLE, target)
    return target


@pytest.fixture
def with_packs(config_dir, monkeypatch) -> ConfigStore:
    """The generic test cell with the repo's robot packs as an extra location."""
    (config_dir / 'system.yaml').write_text(yaml.safe_dump({'robot_packs': [str(ROBOTS)]}))
    monkeypatch.delenv('TASKWEAVE_ROBOT_PACKS', raising=False)
    (config_dir / 'robots' / 'galbot_s1').mkdir()       # the cell has a folder for this robot: it is one of its robots
    return ConfigStore(config_dir)


def by_path(store, pack='cell'):
    return {e['path']: e for e in store.files() if e['pack'] == pack}


def load(path):
    return yaml.safe_load(Path(path).read_text())


# ---- files() and file_kinds() ----------------------------------------------------------------------------------------
def test_files_lists_the_cell_pack_with_kind_schema_reload_and_hash(example):
    store = ConfigStore(example)
    files = by_path(store)
    entry = files['objects/part.yaml']
    assert (entry['kind'], entry['source'], entry['schema_id'], entry['reload'], entry['editable']) == (
        'object_type', 'cell', 'object_type', ['live'], True)
    assert entry['hash'] == store.file_hash('objects/part.yaml') and len(entry['hash']) == 64
    assert files['skills/wait.yaml']['reload'] == ['reload_service']
    pack = files['pack.yaml']
    assert not pack['editable'] and 'migrate' in pack['editable_reason'] and pack['hash'] is None
    # a cell copy of a robot pack file is an override, a cell-specific file is cell
    assert files['robots/duo_bot/profile.yaml']['source'] == 'override'
    assert files['robots/duo_bot/profile.yaml']['robot'] == 'duo_bot'
    assert files['robots/duo_bot/stances.yaml']['kind'] == 'stances'


def test_a_mission_reloads_at_mission_start(config_dir):
    assert by_path(ConfigStore(config_dir))['missions/patrol.yaml']['reload'] == ['mission_start']


def test_files_has_the_robot_packs_and_marks_what_has_no_schema(with_packs):
    pack_files = by_path(with_packs, 'robot:galbot_s1')
    assert pack_files['profile.yaml']['kind'] == 'profile' and pack_files['profile.yaml']['source'] == 'robot_pack'
    assert pack_files['profile.yaml']['reload'] == ['launch']
    assert 'moveit/robot.urdf' in pack_files and 'moveit/meshes/Arms_base_link_0.stl' not in pack_files
    for path in ('moveit/robot.urdf', 'sim.yaml', 'pose_targets.yaml'):
        assert pack_files[path]['schema_id'] is None and pack_files[path]['no_schema_reason']
    assert pack_files['moveit/robot.urdf']['editable'] is False
    assert pack_files['moveit/kinematics.yaml']['editable'] is True
    assert pack_files['navigation.yaml']['reload'] == ['live', 'launch']
    assert pack_files['sim.yaml']['reload'] == ['sim']
    assert not pack_files['pack.yaml']['editable']


def test_a_read_only_robot_pack_is_installed(config_dir, monkeypatch):
    from taskweave_core.config import packs

    monkeypatch.setenv('TASKWEAVE_ROBOT_PACKS', '')
    monkeypatch.setattr(packs, 'installed_location', lambda: ROBOTS)
    (config_dir / 'robots' / 'galbot_s1').mkdir(parents=True)
    assert by_path(ConfigStore(config_dir), 'robot:galbot_s1')['profile.yaml']['source'] == 'installed'


def test_file_kinds_include_files_that_do_not_exist_yet(example):
    kinds = {k['kind']: k for k in ConfigStore(example).file_kinds()}
    assert kinds['object_type']['pattern'] == 'objects/*.yaml' and kinds['object_type']['creatable']
    assert kinds['mission']['reload'] == ['mission_start']
    assert not kinds['profile']['creatable'] and kinds['profile']['pack'] == 'robot'
    assert kinds['layout']['schema_id'] is None and kinds['layout']['no_schema_reason']
    assert set(kinds) >= {'object_type', 'mission', 'skill', 'stances', 'profile', 'embodiment', 'poses', 'navigation'}
    for kind in catalogue.KINDS:    # every kind has a schema or says why not
        assert (kind.schema is None) == bool(kind.no_schema_reason)
        assert kind.reload and set(kind.reload) <= set(catalogue.RELOAD)


def test_new_file_gives_a_path_and_a_start():
    assert catalogue.new_file('object_type', 'crate') == ('objects/crate.yaml', {'name': 'crate'})
    assert catalogue.new_file('stances', robot='duo_bot')[0] == 'robots/duo_bot/stances.yaml'
    with pytest.raises(ValueError):
        catalogue.new_file('mission', 'not a name')
    with pytest.raises(KeyError):
        catalogue.new_file('profile', 'x')


def test_kind_of_maps_paths_in_both_packs():
    assert catalogue.kind_of('missions/a.yaml').kind == 'mission'
    assert catalogue.kind_of('robots/r/profile.yaml').kind == 'profile'
    assert catalogue.kind_of('robots/r/stances.yaml').kind == 'stances'
    assert catalogue.kind_of('profile.yaml', 'robot:r').kind == 'profile'
    assert catalogue.kind_of('moveit/meshes/x.urdf', 'robot:r').kind == 'moveit'
    assert catalogue.kind_of('nothing.yaml') is None


# ---- json_schema() ---------------------------------------------------------------------------------------------------
def test_every_schema_exports_as_a_valid_document(example):
    store = ConfigStore(example)
    for kind in catalogue.KINDS:
        if kind.schema is None:
            with pytest.raises(KeyError, match='no schema'):
                store.json_schema(kind.kind)
            continue
        schema = store.json_schema(kind.kind)
        Draft202012Validator.check_schema(schema)
        assert schema['$schema'].endswith('2020-12/schema') and schema['title']
        json.dumps(schema)


def test_the_schema_of_a_path_is_the_schema_of_its_kind(example):
    store = ConfigStore(example)
    assert store.json_schema('objects/part.yaml') == store.json_schema('object_type')
    assert store.json_schema('profile.yaml', 'robot:x') == store.json_schema('profile')
    with pytest.raises(KeyError):
        store.json_schema('nonsense')


def test_the_latest_keys_are_in_the_exported_schemas(example):
    store = ConfigStore(example)
    obj = store.json_schema('object_type')['properties']
    assert 'collision' in obj and 'sim' in obj
    profile = store.json_schema('profile')['properties']
    assert 'links' in profile['grippers']['additionalProperties']['properties']
    assert {'scene_service', 'scene_timeout_s', 'hold_tolerance_rad'} <= set(profile['planning']['properties'])
    assert 'limits' in profile['joints']['properties']


def test_dynamic_enums_are_marked_not_baked_in(example):
    store = ConfigStore(example)
    mission = store.json_schema('mission')
    skill = mission['properties']['steps']['items']['properties']['skill']
    assert skill['x-taskweave-enum'] == 'skills' and 'enum' not in skill
    assert mission['properties']['robot']['x-taskweave-enum'] == 'robots'
    assert store.json_schema('stances')['properties']['objects']['x-taskweave-enum-keys'] == 'objects'
    assert 'x-taskweave-enum' not in schemas.MISSION['properties']['robot']    # the shared schema is not touched
    assert store.enum_values('skills') == ['wait'] and store.enum_values('robots') == ['duo_bot']
    assert {'part', 'bin'} <= set(store.enum_values('objects'))
    assert store.enum_values('hands', 'duo_bot')
    assert isinstance(store.enum_values('joints', 'duo_bot'), list)
    with pytest.raises(KeyError):
        store.enum_values('nope')


def test_joint_limits_are_in_the_profile_schema():
    valid = {'name': 'r', 'base': {'type': 'fixed', 'cmd_vel': 'a', 'odom': 'b', 'frame': 'c'},
             'joints': {'command': 'a', 'state': 'b', 'limits': {'j': [0.0, 1.0]}}}
    validator = Draft202012Validator(schemas.ROBOT_PROFILE)
    assert validator.is_valid(valid)
    valid['joints']['limits'] = {'j': [0.0]}
    assert not validator.is_valid(valid)
    valid['joints']['limits'] = {'j': 'wide'}
    assert not validator.is_valid(valid)


# ---- validate_file() -------------------------------------------------------------------------------------------------
def test_a_valid_file_has_no_problems(example):
    assert ConfigStore(example).validate_file('objects/part.yaml') == []


def test_schema_problems_have_a_json_pointer(example):
    data = load(example / 'objects' / 'part.yaml')
    data['yaw_symmetry_deg'] = 45
    data['bogus'] = 1
    problems = ConfigStore(example).validate_file('objects/part.yaml', data)
    assert {p['where'] for p in problems} >= {'/yaw_symmetry_deg'}
    assert all(set(p) == {'path', 'pack', 'where', 'line', 'col', 'message', 'severity'} for p in problems)
    assert all(p['line'] is None and p['severity'] == 'error' and p['path'] == 'objects/part.yaml' for p in problems)


def test_an_unknown_key_is_named_in_where(example):
    problems = ConfigStore(example).validate_file('robots/duo_bot/object_points.yaml',
                                                  {'part': {'p': {'bogus': 1}}})
    assert [p['where'] for p in problems] == ['/part/p/bogus']


def test_text_gives_lines_and_yaml_errors(example):
    store = ConfigStore(example)
    (problem,) = store.validate_file('objects/part.yaml', 'name: part\n\nyaw_symmetry_deg: 45\n')
    assert (problem['where'], problem['line'], problem['col']) == ('/yaw_symmetry_deg', 3, 1)
    (broken,) = store.validate_file('objects/part.yaml', 'name: [part\n')
    assert 'invalid YAML' in broken['message'] and broken['line']
    assert store.validate_file('objects/ghost.yaml')[0]['message'] == 'file not found'


def test_validation_writes_nothing(example):
    store = ConfigStore(example)
    before = {p: p.read_bytes() for p in example.rglob('*') if p.is_file()}
    store.validate_file('objects/part.yaml', {'name': 'x', 'bogus': 1})
    store.dry_run_pack('objects/part.yaml', {'name': 'x', 'bogus': 1})
    assert {p: p.read_bytes() for p in example.rglob('*') if p.is_file()} == before


def test_cross_file_checks_use_the_pack_on_disk(example):
    store = ConfigStore(example)
    instances = load(example / 'cell' / 'objects.yaml')
    name = next(iter(instances))
    instances[name]['object'] = 'missing_type'
    (problem,) = store.validate_file('cell/objects.yaml', instances)
    assert problem['where'] == f'/{name}/object' and 'missing_type' in problem['message']
    pose = {'x': 0, 'y': 0, 'yaw': 0}
    stances = {'objects': {'missing_type': {'front': pose}}, 'instances': {'no_such_instance': {'front': pose}}}
    problems = store.validate_file('robots/duo_bot/stances.yaml', stances)
    assert {(p['where'], p['severity']) for p in problems} == {('/objects/missing_type', 'warning'),
                                                              ('/instances/no_such_instance', 'warning')}


def test_object_type_checks(example):
    store = ConfigStore(example)
    data = load(example / 'objects' / 'part.yaml')
    data['name'] = 'other'
    assert [p['where'] for p in store.validate_file('objects/part.yaml', data)] == ['/name']
    data = load(example / 'objects' / 'part.yaml')
    point = next(iter(data['points']))
    data['points'][point]['grip_width_m'] = 9.0
    problems = store.validate_file('objects/part.yaml', data)
    assert problems and all(p['severity'] == 'warning' and p['where'] == f'/points/{point}' for p in problems)
    assert any('stroke' in p['message'] for p in problems)


def test_profile_and_embodiment_cross_checks(example):
    store = ConfigStore(example)
    profile = load(example / 'robots' / 'duo_bot' / 'profile.yaml')
    profile['base']['type'] = 'holonomic'
    problems = store.validate_file('robots/duo_bot/profile.yaml', profile)
    assert any('capability' in p['message'] and p['severity'] == 'error' for p in problems)
    embodiment = load(example / 'robots' / 'duo_bot' / 'embodiment.yaml')
    embodiment['capabilities'] = [c for c in embodiment['capabilities'] if not c.startswith('base.')]
    assert store.validate_file('robots/duo_bot/embodiment.yaml', embodiment)
    profile = load(example / 'robots' / 'duo_bot' / 'profile.yaml')
    profile['name'] = 'someone'
    profile.setdefault('joints', {}).update(command='a', state='b', limits={'j1': [1.0, 0.0]})
    where = {p['where'] for p in store.validate_file('robots/duo_bot/profile.yaml', profile)}
    assert {'/name', '/joints/limits/j1'} <= where


def test_navigation_and_poses_cross_checks(with_packs):
    nav = load(S1 / 'navigation.yaml')
    nav['driving'] = {**nav.get('driving', {}), 'sideways': True, 'max_forward': 99.0}
    problems = with_packs.validate_file('navigation.yaml', nav, 'robot:galbot_s1')
    assert {p['where'] for p in problems} >= {'/driving/sideways', '/driving/max_forward'}
    poses = load(S1 / 'poses.yaml')
    name = next(n for n, p in poses.items() if 'joints' not in p and p)
    joint = next(iter(poses[name]))
    poses[name][joint] = 99.0
    problems = with_packs.validate_file('poses.yaml', poses, 'robot:galbot_s1')
    assert any(p['where'] == f'/{name}/{joint}' and 'outside' in p['message'] for p in problems)


def test_held_poses_object_points_and_skill_overrides(example):
    store = ConfigStore(example)
    problems = store.validate_file('robots/duo_bot/held_poses.yaml', {'ghost': {'carry': {'joints': {'nope': 1.0}}}})
    assert '/ghost' in [p['where'] for p in problems]
    problems = store.validate_file('robots/duo_bot/object_points.yaml', {'part': {'no_point': {'elbow': 'up'}}})
    assert [(p['where'], p['severity']) for p in problems] == [('/part/no_point', 'warning')]
    problems = store.validate_file('robots/duo_bot/skill_overrides.yaml',
                                   {'ghost': {'speed': {'tcp_m_s': 0.1}}, 'wait': {}})
    assert [(p['where'], p['severity']) for p in problems] == [('/ghost', 'warning')]


def test_a_mission_is_checked_for_ids_includes_and_skills(config_dir):
    store = ConfigStore(config_dir)
    mission = {'goal': 'g', 'robot': 'demo_bot', 'steps': [
        {'id': 'a', 'instruction': 'i', 'skill': 'wait', 'replan_from': 'zzz'},
        {'id': 'a', 'instruction': 'i', 'skill': 'ghost_skill'},
        {'instruction': 'i', 'include': 'no_such_mission'}]}
    problems = {(p['where'], p['severity']) for p in store.validate_file('missions/new.yaml', mission)}
    assert ('/steps/0/replan_from', 'error') in problems and ('/steps/1/id', 'error') in problems
    assert ('/steps/1/skill', 'warning') in problems and ('', 'error') in problems
    assert store.validate_file('missions/patrol.yaml') == []


def test_a_keyed_override_is_checked_merged(with_packs, config_dir):
    robot = config_dir / 'robots' / 'galbot_s1'
    robot.mkdir(parents=True, exist_ok=True)
    (robot / 'navigation.yaml').write_text('~override: keys\ndriving: {max_forward: -1}\n')
    store = ConfigStore(config_dir)
    assert [p['where'] for p in store.validate_file('robots/galbot_s1/navigation.yaml')] == ['/driving/max_forward']
    (robot / 'navigation.yaml').write_text('~override: keys\ndriving: {max_forward: 0.2}\n')
    assert store.validate_file('robots/galbot_s1/navigation.yaml') == []


# ---- check_pack() and dry_run_pack() ---------------------------------------------------------------------------------
def test_check_pack_of_the_example_cell_is_clean(example):
    assert [p for p in ConfigStore(example).check_pack() if p['severity'] == 'error'] == []


def test_check_pack_covers_the_files_the_old_check_skipped(example):
    poses = example / 'robots' / 'duo_bot' / 'poses.yaml'
    poses.write_text(poses.read_text() + '\nbroken: {no_joint: not_a_number}\n')
    (example / 'robots' / 'duo_bot' / 'held_poses.yaml').write_text('ghost: {carry: {bogus: 1}}\n')
    paths = {p['path'] for p in ConfigStore(example).check_pack() if p['severity'] == 'error'}
    assert {'robots/duo_bot/poses.yaml', 'robots/duo_bot/held_poses.yaml'} <= paths


def test_the_check_command_prints_and_returns_the_result(example, capsys):
    assert cli.check(str(example)) == 0
    out = capsys.readouterr().out
    assert 'objects/part.yaml' in out and 'config ok' in out
    (example / 'robots' / 'duo_bot' / 'navigation.yaml').write_text('envelope: {front: 1}\n')
    assert cli.check(str(example)) == 1
    assert 'FAIL' in capsys.readouterr().out


def test_dry_run_shows_what_else_an_edit_breaks(example):
    store = ConfigStore(example)
    problems = store.dry_run_pack('objects/part.yaml', {'name': 'part', 'yaw_symmetry_deg': 45})
    assert [p for p in problems if p['path'] == 'objects/part.yaml' and p['where'] == '/yaw_symmetry_deg']
    assert not [p for p in store.check_pack() if p['path'] == 'objects/part.yaml']     # still fine on disk
    text = 'x:\n  object: nothing\n  x: 0\n  y: 0\n  yaw: 0\n  z: 0\n'
    hit = [p for p in store.dry_run_pack('cell/objects.yaml', text) if p['path'] == 'cell/objects.yaml']
    assert hit and hit[0]['line'] == 2 and 'nothing' in hit[0]['message']


def test_dry_run_of_a_robot_pack_file(with_packs):
    nav = load(S1 / 'navigation.yaml')
    nav['driving'] = {'sideways': True}
    problems = with_packs.dry_run_pack('navigation.yaml', nav, 'robot:galbot_s1')
    assert any(p['where'] == '/driving/sideways' and p['pack'] == 'robot:galbot_s1' for p in problems)
    assert with_packs.validate_file('navigation.yaml', None, 'robot:galbot_s1') == []


def test_robot_packs_are_listed(with_packs):
    listed = {p['location']: p for p in with_packs.robot_packs()}
    assert {str(ROBOTS / n) for n in ('galbot_s1', 'galbot_g1', 'a2d')} <= set(listed)
    assert set(listed[str(S1)]) == {'name', 'version', 'description', 'location', 'writable'}
    assert listed[str(S1)]['name'] == 'galbot_s1' and listed[str(S1)]['writable'] is True
    assert with_packs.robot_packs('galbot_s1')[0].name == 'galbot_s1'    # by robot: the packs, as before


# ---- command line ----------------------------------------------------------------------------------------------------
def test_cli_files_schema_and_validate(example, capsys):
    assert cli.main(['config', 'files', str(example), '--json']) == 0
    assert any(e['path'] == 'objects/part.yaml' for e in json.loads(capsys.readouterr().out))
    assert cli.main(['config', 'schema', 'object_type']) == 0
    assert json.loads(capsys.readouterr().out)['title']
    assert cli.main(['config', 'schema', 'layout']) == 2
    part = example / 'objects' / 'part.yaml'
    assert cli.main(['config', 'validate', str(part), '--json']) == 0
    assert json.loads(capsys.readouterr().out) == []
    part.write_text(part.read_text() + 'yaw_symmetry_deg: 45\n')
    assert cli.main(['config', 'validate', str(part), '--json']) == 1
    (problem,) = json.loads(capsys.readouterr().out)
    assert problem['where'] == '/yaw_symmetry_deg' and problem['path'] == 'objects/part.yaml'
    assert cli.main(['config', 'validate', 'objects/part.yaml', '--cell', str(example)]) == 1
    assert 'yaw_symmetry_deg' in capsys.readouterr().out
