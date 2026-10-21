"""Cell pack and robot packs: where robot files are read from and written to, schema checks, the migration."""

import os
import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigError, ConfigStore, packs
from taskweave_core.config.migrate import migrate as migrate_chain

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'


def migrate(*args, **kwargs):
    """The 1 -> 2 and 2 -> 3 migrations these tests are about (3 -> 4 has its own tests)."""
    return migrate_chain(*args, latest=3, **kwargs)


ROBOT = 'demo_bot'
POSES = '# robot pack poses\nhome: {lift: 0.1}   # pack comment\n'


@pytest.fixture(autouse=True)
def no_outside_packs(monkeypatch):
    """Only the packs a test sets up: no env, no installed share."""
    monkeypatch.delenv(packs.ROBOT_PACKS_ENV, raising=False)
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


def robot_pack(folder: Path, robot: str = ROBOT, files: dict | None = None, schema: int = packs.ROBOT_SCHEMA) -> Path:
    """A robot pack <folder>/<robot>/ with pack.yaml and ``files``."""
    pack = folder / robot
    pack.mkdir(parents=True)
    (pack / 'pack.yaml').write_text(f'name: {robot}\nversion: 0.1.0\nschema_version: {schema}\n')
    for name, text in (files or {}).items():
        (pack / name).parent.mkdir(parents=True, exist_ok=True)
        (pack / name).write_text(text)
    return pack


@pytest.fixture
def split(tmp_path):
    """The test config with the robot's profile, embodiment and a poses file in a robot pack (system.yaml)."""
    cell = tmp_path / 'cell'
    shutil.copytree(FIXTURE, cell)
    pack = robot_pack(tmp_path / 'packs', files={'poses.yaml': POSES})
    for name in ('profile.yaml', 'embodiment.yaml'):
        shutil.move(cell / 'robots' / ROBOT / name, pack / name)
    (cell / 'system.yaml').write_text('robot: demo_bot\nrobot_packs: [../packs]\n')
    return ConfigStore(cell), pack


def test_robot_files_come_from_the_robot_pack(split):
    store, pack = split
    assert store.robot(ROBOT).base['type'] == 'holonomic'
    assert store.robot_file(ROBOT, 'profile.yaml') == pack / 'profile.yaml'
    assert store.robot_file(ROBOT, 'stances.yaml') == store.root / 'robots' / ROBOT / 'stances.yaml'
    assert store.stance('shelf_a', ROBOT).yaw == pytest.approx(1.5708)
    source = store.robot_file_source(ROBOT, 'profile.yaml')
    assert source['source'] == 'robot pack' and 'system.yaml' in source['pack']
    assert store.display_path(pack / 'poses.yaml') == 'demo_bot robot pack: poses.yaml'


def test_search_order_cell_env_system_installed(split, tmp_path, monkeypatch):
    store, pack = split
    installed = robot_pack(tmp_path / 'installed', files={'poses.yaml': 'home: {lift: 0.3}\n'})
    monkeypatch.setattr(packs, 'installed_location', lambda: installed.parent)
    assert store.named_pose(ROBOT, 'home')['joints'] == {'lift': 0.1}              # system.yaml before installed
    (pack / 'poses.yaml').unlink()
    assert store.named_pose(ROBOT, 'home')['joints'] == {'lift': 0.3}              # file by file: then installed
    env = robot_pack(tmp_path / 'env', files={'poses.yaml': 'home: {lift: 0.2}\n'})
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(env))                            # a robot pack itself
    assert store.named_pose(ROBOT, 'home')['joints'] == {'lift': 0.2}
    (store.root / 'robots' / ROBOT / 'poses.yaml').write_text('home: {lift: 0.4}\n')
    assert store.named_pose(ROBOT, 'home')['joints'] == {'lift': 0.4}              # cell override first
    assert store.robot_file_source(ROBOT, 'poses.yaml')['source'] == 'override'
    assert [p.location.origin for p in store.robot_packs(ROBOT)] == ['env', 'system.yaml', 'installed']


def test_saves_cell_files_to_the_cell_and_pack_files_to_the_pack(split):
    store, pack = split
    path = store.set_joint_pose(ROBOT, 'ready', {'lift': 0.2})
    assert path == pack / 'poses.yaml' and '# pack comment' in path.read_text()
    assert not (store.root / 'robots' / ROBOT / 'poses.yaml').exists()
    path = store.save_held_pose(ROBOT, 'box', 'carry', {'lift': 0.1}, [])         # cell-specific: the cell pack
    assert path == store.root / 'robots' / ROBOT / 'held_poses.yaml'
    assert store.robot_save_path(ROBOT, 'navigation.yaml') == pack / 'navigation.yaml'  # new file: the writable pack


def test_read_only_pack_edits_become_a_cell_override(split, tmp_path, monkeypatch):
    store, pack = split
    (store.root / 'system.yaml').write_text('robot: demo_bot\n')                   # only the installed share
    shutil.move(pack, tmp_path / 'installed' / ROBOT)
    monkeypatch.setattr(packs, 'installed_location', lambda: tmp_path / 'installed')
    installed = tmp_path / 'installed' / ROBOT / 'poses.yaml'
    path = store.set_joint_pose(ROBOT, 'ready', {'lift': 0.2})
    assert path == store.root / 'robots' / ROBOT / 'poses.yaml'
    assert 'ready' in path.read_text() and 'home' not in path.read_text()         # only what the cell changed
    assert installed.read_text() == POSES
    assert store.named_pose(ROBOT, 'home')['joints'] == {'lift': 0.1}             # the rest from the pack's file
    assert store.robot_file_source(ROBOT, 'poses.yaml')['source'] == 'override'
    assert store.display_path(store.robot_save_path(ROBOT, 'profile.yaml')) == f'robots/{ROBOT}/profile.yaml'


def test_missing_robot_file_names_the_places_searched(split):
    store, _ = split
    (store.root.parent / 'empty').mkdir()    # a location without the robot's pack (a missing one is an error)
    (store.root / 'system.yaml').write_text('robot_packs: [../empty]\n')
    with pytest.raises(ConfigError, match='file not found .looked in cell pack'):
        store.robot(ROBOT)


def test_schema_version_is_checked(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'cell')
    (tmp_path / 'cell' / 'pack.yaml').unlink()
    with pytest.raises(ConfigError, match='cell pack schema 1, this taskweave needs 3: run taskweave config migrate '):
        ConfigStore(tmp_path / 'cell')
    (tmp_path / 'cell' / 'pack.yaml').write_text('name: cell\nversion: 1\nschema_version: 2\n')
    with pytest.raises(ConfigError, match='cell pack schema 2, this taskweave needs 3: run taskweave config '
                                          'migrate-skills'):
        ConfigStore(tmp_path / 'cell')
    shutil.copytree(tmp_path / 'cell', tmp_path / 'newer')   # (a pack.yaml is read again only when its mtime changes)
    (tmp_path / 'newer' / 'pack.yaml').write_text('name: cell\nversion: 1\nschema_version: 5\n')
    with pytest.raises(ConfigError, match='update taskweave'):
        ConfigStore(tmp_path / 'newer')


def test_robot_pack_needs_pack_yaml_and_its_schema(split):
    store, pack = split
    (pack / 'pack.yaml').write_text('name: demo_bot\nversion: 0.1.0\nschema_version: 9\n')
    with pytest.raises(ConfigError, match='robot pack schema 9'):
        store.robot(ROBOT)
    (pack / 'pack.yaml').unlink()
    with pytest.raises(ConfigError, match='has no pack.yaml'):
        store.robot(ROBOT)


@pytest.fixture
def old_config(tmp_path):
    """A config directory from before packs (schema 1): whole robot folders in robots/."""
    cell = tmp_path / 'config'
    shutil.copytree(FIXTURE, cell)
    (cell / 'pack.yaml').unlink()
    folder = cell / 'robots' / ROBOT
    (folder / 'moveit').mkdir()
    (folder / 'moveit' / 'robot.urdf').write_text('<robot name="demo_bot"/>\n')
    (folder / 'poses.yaml').write_text(POSES)
    return cell


def test_migration_dry_run_changes_nothing(old_config, tmp_path):
    before = sorted(str(p.relative_to(tmp_path)) for p in tmp_path.rglob('*'))
    lines = []
    assert migrate(old_config, tmp_path / 'robot_packs', dry_run=True, say=lines.append) == 0
    assert sorted(str(p.relative_to(tmp_path)) for p in tmp_path.rglob('*')) == before
    assert any(line.startswith('would move    robots/demo_bot/profile.yaml') for line in lines)
    assert any(line.startswith('would write   pack.yaml') for line in lines)


def test_migration_splits_and_is_idempotent(old_config, tmp_path):
    lines = []
    assert migrate(old_config, tmp_path / 'robot_packs', name='test cell', say=lines.append) == 0
    pack = tmp_path / 'robot_packs' / ROBOT
    assert sorted(p.name for p in pack.iterdir()) == ['embodiment.yaml', 'moveit', 'pack.yaml', 'poses.yaml',
                                                      'profile.yaml']
    assert sorted(p.name for p in (old_config / 'robots' / ROBOT).iterdir()) == ['stances.yaml']
    assert 'robot_packs:\n- ../robot_packs' in (old_config / 'system.yaml').read_text()
    assert 'name: test_cell' in (old_config / 'pack.yaml').read_text()
    store = ConfigStore(old_config)
    assert store.robot(ROBOT).name == ROBOT and store.stances(ROBOT)
    assert store.robot_file(ROBOT, 'moveit/robot.urdf') == pack / 'moveit' / 'robot.urdf'
    again = []
    assert migrate(old_config, tmp_path / 'robot_packs', say=again.append) == 0
    assert again == [f'{old_config.resolve()}: cell pack schema 3, nothing to do']


def test_migration_removes_copies_and_keeps_differences(old_config, tmp_path, monkeypatch):
    """Robot files already in a robot pack (e.g. the installed share): the same are removed, others kept."""
    folder = old_config / 'robots' / ROBOT
    installed = robot_pack(tmp_path / 'installed', files={name: (folder / name).read_text()
                                                          for name in ('profile.yaml', 'embodiment.yaml')})
    shutil.copytree(folder / 'moveit', installed / 'moveit')
    (installed / 'poses.yaml').write_text('home: {lift: 0.3}\n')
    monkeypatch.setattr(packs, 'installed_location', lambda: installed.parent)
    lines = []
    assert migrate(old_config, say=lines.append) == 0
    assert sorted(p.name for p in folder.iterdir()) == ['poses.yaml', 'stances.yaml']
    assert any('keep    robots/demo_bot/poses.yaml  as a cell override' in line for line in lines)
    assert any(line.startswith('remove  robots/demo_bot/moveit/') for line in lines)
    assert not (old_config / 'system.yaml').exists()    # nothing moved: no robot_packs entry needed
    store = ConfigStore(old_config)
    assert store.robot_file_source(ROBOT, 'poses.yaml')['source'] == 'override'
    assert store.robot_file(ROBOT, 'profile.yaml') == installed / 'profile.yaml'
    assert os.path.samefile(store.robot_file(ROBOT, 'moveit'), installed / 'moveit')


# ---- P1 review fix 3: a configured robot pack location that does not exist is an error ----
def test_a_missing_env_location_is_an_error_naming_it(split, tmp_path, monkeypatch):
    store, _ = split
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(tmp_path / 'typo'))
    with pytest.raises(ConfigError, match=r'typo.*does not exist.*env TASKWEAVE_ROBOT_PACKS'):
        store.robot(ROBOT)


def test_a_missing_system_yaml_location_is_an_error_naming_it(split):
    store, _ = split
    (store.root / 'system.yaml').write_text('robot: demo_bot\nrobot_packs: [../packs, ../nowhere]\n')
    with pytest.raises(ConfigError, match=r'nowhere.*does not exist.*system.yaml robot_packs'):
        ConfigStore(store.root).robot(ROBOT)


def test_migration_writes_an_absolute_path_across_windows_drives(old_config, tmp_path, monkeypatch):
    def other_drive(path, start=os.curdir):
        raise ValueError('path is on mount D:, start on mount C:')

    monkeypatch.setattr(os.path, 'relpath', other_drive)
    assert migrate(old_config, tmp_path / 'robot_packs', say=lambda line: None) == 0
    entry = (old_config / 'system.yaml').read_text()
    assert (tmp_path / 'robot_packs').resolve().as_posix() in entry
    assert ConfigStore(old_config).robot(ROBOT).name == ROBOT


def test_migration_does_not_delete_robot_pack_files_through_a_linked_robot_folder(old_config, tmp_path, monkeypatch):
    folder = old_config / 'robots' / ROBOT
    pack = robot_pack(tmp_path / 'linked', files={name: (folder / name).read_text()
                                                  for name in ('profile.yaml', 'embodiment.yaml', 'stances.yaml')})
    shutil.copytree(folder / 'moveit', pack / 'moveit')
    shutil.rmtree(folder)
    try:
        folder.symlink_to(pack, target_is_directory=True)
    except (OSError, NotImplementedError):
        pytest.skip('no symlink support here')
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(tmp_path / 'linked'))
    before = {str(p.relative_to(pack)): p.read_bytes() for p in pack.rglob('*') if p.is_file()}
    lines = []
    assert migrate(old_config, say=lines.append) == 0
    assert {str(p.relative_to(pack)): p.read_bytes() for p in pack.rglob('*') if p.is_file()} == before
    assert any(line.startswith('skip') and 'robot pack' in line for line in lines)


# ---- F1: migrate removes a unit completely or not at all; a half-removed unit is recognised on the rerun ----
MOVEIT = {'robot.urdf': '<robot name="demo_bot"/>\n', 'a.srdf': 'a\n', 'b.yaml': 'b: 1\n', 'meshes/m.stl': 'solid\n'}


@pytest.fixture
def same_as_pack(old_config, tmp_path, monkeypatch):
    """The cell's robot files all equal the installed robot pack's; moveit/ has a meshes/ subfolder."""
    folder = old_config / 'robots' / ROBOT
    for name, text in MOVEIT.items():
        (folder / 'moveit' / name).parent.mkdir(exist_ok=True)
        (folder / 'moveit' / name).write_text(text)
    installed = robot_pack(tmp_path / 'installed', files={name: (folder / name).read_text()
                                                          for name in ('profile.yaml', 'embodiment.yaml',
                                                                       'poses.yaml')})
    shutil.copytree(folder / 'moveit', installed / 'moveit')
    monkeypatch.setattr(packs, 'installed_location', lambda: installed.parent)
    return old_config


def tree(folder: Path) -> list[str]:
    return sorted(str(p.relative_to(folder)) for p in folder.rglob('*'))


def test_migration_removes_nothing_of_a_unit_it_cannot_remove_completely(same_as_pack):
    moveit = same_as_pack / 'robots' / ROBOT / 'moveit'
    before = tree(moveit)
    (moveit / 'meshes').chmod(0o555)
    try:
        if os.access(moveit / 'meshes', os.W_OK):
            pytest.skip('permissions are not enforced here (root?)')
        lines = []
        assert migrate(same_as_pack, say=lines.append) == 1
        assert tree(moveit) == before                          # not even the 3 files that could go
        assert any(line.startswith('cannot remove ') and 'meshes' in line and line.endswith(
            f'(unit robots/{ROBOT}/moveit/ left as it is; fix permissions or remove it as its owner)')
            for line in lines)
        assert not (same_as_pack / 'robots' / ROBOT / 'profile.yaml').exists()    # the other units went
        assert not (same_as_pack / 'pack.yaml').exists()
        assert any('pack.yaml not written' in line and 'moveit' in line for line in lines)
    finally:
        (moveit / 'meshes').chmod(0o755)
    again = []
    assert migrate(same_as_pack, say=again.append) == 0         # fixed permissions: the rerun finishes the job
    assert not moveit.exists() and (same_as_pack / 'pack.yaml').is_file()


def test_migration_blocked_unit_with_a_stubbed_permission_check(same_as_pack, monkeypatch):
    """Same as above where chmod cannot block (root): the check is one function, forced to say no."""
    from taskweave_core.config import migrate as module

    moveit = same_as_pack / 'robots' / ROBOT / 'moveit'
    before = tree(moveit)
    real = module._entry_blocker
    monkeypatch.setattr(module, '_entry_blocker',
                        lambda path: 'Permission denied' if path.name == 'meshes' else real(path))
    lines = []
    assert migrate(same_as_pack, say=lines.append) == 1
    assert tree(moveit) == before
    assert f'cannot remove {moveit / "meshes"}: Permission denied (unit robots/{ROBOT}/moveit/ left as it is; ' \
           'fix permissions or remove it as its owner)' in lines
    assert not (same_as_pack / 'robots' / ROBOT / 'profile.yaml').exists()
    assert not (same_as_pack / 'pack.yaml').exists() and any('pack.yaml not written' in line for line in lines)
    monkeypatch.setattr(module, '_entry_blocker', real)
    assert migrate(same_as_pack, say=lambda line: None) == 0
    assert not moveit.exists() and (same_as_pack / 'pack.yaml').is_file()


def test_migration_dry_run_reports_what_it_cannot_remove(same_as_pack, monkeypatch):
    from taskweave_core.config import migrate as module

    monkeypatch.setattr(module, '_entry_blocker', lambda path: 'Permission denied' if path.name == 'meshes' else None)
    lines = []
    assert migrate(same_as_pack, dry_run=True, say=lines.append) == 1
    assert any(line.startswith('cannot remove ') for line in lines)


def test_migration_rerun_finishes_a_half_removed_unit(same_as_pack):
    moveit = same_as_pack / 'robots' / ROBOT / 'moveit'
    for name in ('robot.urdf', 'a.srdf', 'b.yaml'):            # what the incident left behind
        (moveit / name).unlink()
    lines = []
    assert migrate(same_as_pack, say=lines.append) == 0
    assert not moveit.exists()
    assert any('robots/demo_bot/moveit/' in line and 'partly removed' in line for line in lines)
    assert not any('cell override' in line and 'moveit' in line for line in lines)
    assert (same_as_pack / 'pack.yaml').is_file()


def test_migration_keeps_a_subset_unit_whose_files_differ(same_as_pack):
    moveit = same_as_pack / 'robots' / ROBOT / 'moveit'
    (moveit / 'a.srdf').unlink()
    (moveit / 'b.yaml').write_text('b: 2\n')
    lines = []
    assert migrate(same_as_pack, say=lines.append) == 0
    assert (moveit / 'b.yaml').is_file()
    assert any('keep    robots/demo_bot/moveit/  as a cell override' in line for line in lines)


def test_migration_keeps_a_unit_with_a_file_the_pack_lacks(same_as_pack):
    moveit = same_as_pack / 'robots' / ROBOT / 'moveit'
    (moveit / 'extra.yaml').write_text('x: 1\n')
    assert migrate(same_as_pack, say=lambda line: None) == 0
    assert (moveit / 'extra.yaml').is_file()


def test_slim_does_not_remove_a_file_it_cannot_remove(same_as_pack, monkeypatch):
    from taskweave_core.config import migrate as module
    from taskweave_core.config.slim import slim

    assert migrate(same_as_pack, say=lambda line: None) == 0
    profile = same_as_pack / 'robots' / ROBOT / 'profile.yaml'
    shutil.copy(ConfigStore(same_as_pack).robot_file(ROBOT, 'profile.yaml'), profile)   # an old whole-file copy
    monkeypatch.setattr(module, '_entry_blocker', lambda path: 'Permission denied')
    lines = []
    assert slim(same_as_pack, say=lines.append) == 1
    assert profile.is_file()
    assert any(line.startswith('cannot remove ') and 'profile.yaml' in line for line in lines)
