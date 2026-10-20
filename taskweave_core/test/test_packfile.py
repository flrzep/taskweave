"""config init / export / import: a cell pack from nothing, as one file, between folders."""

import hashlib
import io
import os
import shutil
import stat
import subprocess
import tarfile
from pathlib import Path

import pytest
import yaml

from taskweave_core import cli
from taskweave_core.config import ConfigStore, Pose2D, packfile, packs
from taskweave_core.config.packfile import PackfileError, export_cell, import_cell, init_cell

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
ROBOT = 'demo_bot'
MISSION = {'goal': 'Wait once', 'robot': ROBOT,
           'steps': [{'instruction': 'Wait', 'skill': 'wait', 'args': {'seconds': 1}}]}


@pytest.fixture(autouse=True)
def robot_pack(tmp_path, monkeypatch):
    """The fixture robot as the only robot pack."""
    pack = tmp_path / 'robot_packs' / ROBOT
    pack.mkdir(parents=True)
    (pack / 'pack.yaml').write_text(f'name: {ROBOT}\nversion: 1\nschema_version: {packs.ROBOT_SCHEMA}\n')
    for name in ('profile.yaml', 'embodiment.yaml'):
        shutil.copy(FIXTURE / 'robots' / ROBOT / name, pack / name)
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(tmp_path / 'robot_packs'))
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


def check_ok(cell: Path) -> None:
    assert cli.check(str(cell)) == 0


def build_cell(cell: Path) -> None:
    """A cell from nothing: init, then the store's save APIs (and the two files without one)."""
    init_cell(cell, ROBOT, 'built cell')
    store = ConfigStore(cell)
    store._edit_yaml('objects/crate.yaml', lambda d: d.update({'name': 'crate'}))      # API gap: object type
    (cell / 'missions' / 'once.yaml').write_text(yaml.safe_dump(MISSION))              # API gap: mission
    store.save_object_instance('crate_1', 'crate', 1.0, -2.0, 0.4, 1.5708)
    store.save_object_point('crate_1/top', {'left': ((0.1, 0.0, 0.2), (0.0, 0.0, 0.0, 1.0))})
    store.save_base_pose('home', Pose2D(0.5, 0.5, 0.0))
    store.save_stance('crate_1/front', Pose2D(1.0, -1.2, 1.5708), ROBOT)


def tree(folder: Path) -> dict:
    return {p.relative_to(folder).as_posix(): p.read_bytes() for p in sorted(folder.rglob('*')) if p.is_file()}


def test_init_is_valid_and_empty(tmp_path):
    cell = tmp_path / 'my cell'
    init_cell(cell, ROBOT)
    check_ok(cell)
    store = ConfigStore(cell)
    assert store.object_instances() == {} and store.mission_names() == [] and store.locations() == {}
    assert yaml.safe_load((cell / 'pack.yaml').read_text()) == {'name': 'my_cell', 'version': 1,
                                                                 'schema_version': packs.CELL_SCHEMA}
    with pytest.raises(PackfileError, match='not an empty'):
        init_cell(cell, ROBOT)
    with pytest.raises(PackfileError, match='no robot pack'):
        init_cell(tmp_path / 'other', 'nobody')
    assert not (tmp_path / 'other').exists()


def test_build_export_import_roundtrip(tmp_path):
    cell = tmp_path / 'a'
    build_cell(cell)
    check_ok(cell)
    assert (tmp_path / 'a' / 'cell' / 'objects.yaml').read_text().startswith('crate_1')
    (cell / '__pycache__').mkdir()
    (cell / '__pycache__' / 'x.pyc').write_bytes(b'x')
    (cell / 'state').mkdir()
    (cell / 'state' / 'objects.yaml').write_text('crate_1: {gone: true}\n')
    manifest = export_cell(cell, tmp_path / 'out' / 'a.tar.gz')
    assert manifest['name'] == 'built_cell' and manifest['git_rev'] is None
    assert manifest['schema_version'] == packs.CELL_SCHEMA and manifest['date'] and manifest['taskweave_version']
    listed = {f['path'] for f in manifest['files']}
    assert {'pack.yaml', 'objects/crate.yaml', 'missions/once.yaml', 'robots/demo_bot/stances.yaml'} <= listed
    assert not any('pycache' in f or f.startswith('state') for f in listed)
    report = import_cell(tmp_path / 'out' / 'a.tar.gz', tmp_path / 'b')
    assert report['mode'] == 'new' and report['files'] == len(listed)
    check_ok(tmp_path / 'b')
    assert tree(tmp_path / 'b') == {k: v for k, v in tree(cell).items() if k in listed}
    assert ConfigStore(tmp_path / 'b').object_point('crate', 'top')['left_tool']


def test_import_swaps_and_keeps_backup(tmp_path):
    build_cell(tmp_path / 'a')
    export_cell(tmp_path / 'a', tmp_path / 'a.tar.gz')
    init_cell(tmp_path / 'b', ROBOT, 'old')
    with pytest.raises(PackfileError, match='--force'):
        import_cell(tmp_path / 'a.tar.gz', tmp_path / 'b')
    report = import_cell(tmp_path / 'a.tar.gz', tmp_path / 'b', force=True)
    backup = Path(report['backup'])
    assert report['mode'] == 'swap' and backup.name.startswith('b.bak-')
    assert yaml.safe_load((backup / 'pack.yaml').read_text())['name'] == 'old'
    assert yaml.safe_load((tmp_path / 'b' / 'pack.yaml').read_text())['name'] == 'built_cell'
    assert [p.name for p in tmp_path.iterdir() if p.name.startswith('.b.')] == []     # no temp dir left


@pytest.mark.skipif(os.name != 'posix', reason='group and mode bits')
def test_import_keeps_group_and_mode(tmp_path):
    build_cell(tmp_path / 'a')
    export_cell(tmp_path / 'a', tmp_path / 'a.tar.gz')
    old = tmp_path / 'b'
    init_cell(old, ROBOT)
    os.chmod(old, 0o2775)
    report = import_cell(tmp_path / 'a.tar.gz', old, force=True)
    assert not report['warnings']
    assert stat.S_IMODE(old.stat().st_mode) == 0o2775 and old.stat().st_gid == Path(report['backup']).stat().st_gid
    assert stat.S_IMODE((old / 'objects').stat().st_mode) == 0o2775
    assert (old / 'pack.yaml').stat().st_mode & stat.S_IWGRP


def make_archive(path: Path, members: dict, manifest: dict | None = None, extra=()) -> Path:
    """An archive of ``members`` {name: bytes} with a matching manifest, keys overridden by ``manifest``."""
    base = {'name': 'x', 'schema_version': packs.CELL_SCHEMA, 'git_rev': None,
            'files': [{'path': n, 'sha256': hashlib.sha256(b).hexdigest()} for n, b in members.items()]}
    base.update(manifest or {})
    with tarfile.open(path, 'w:gz') as archive:
        for name, data in {packfile.MANIFEST: yaml.safe_dump(base).encode(), **members}.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
        for info in extra:
            archive.addfile(info)
    return path


@pytest.mark.parametrize('case,message', [
    ('traversal', 'unsafe path'), ('absolute', 'unsafe path'), ('symlink', 'link'), ('hash', 'checksum'),
    ('schema', 'incompatible cell pack: schema 2'), ('extra', 'differ'), ('invalid', 'pack check failed'),
    ('garbage', 'not a readable'),
])
def test_bad_archive_leaves_pack_untouched(tmp_path, case, message):
    init_cell(tmp_path / 'cell', ROBOT)
    before = tree(tmp_path / 'cell')
    good = {'pack.yaml': b'name: x\nversion: 1\nschema_version: 3\n'}
    archive = tmp_path / 'bad.tar.gz'
    if case == 'traversal':
        make_archive(archive, {'../evil.yaml': b'x: 1'})
    elif case == 'absolute':
        make_archive(archive, {'/tmp/evil.yaml': b'x: 1'})
    elif case == 'symlink':
        link = tarfile.TarInfo('link')
        link.type, link.linkname = tarfile.SYMTYPE, '/etc/passwd'
        make_archive(archive, good, extra=[link])
    elif case == 'hash':
        make_archive(archive, good, {'files': [{'path': 'pack.yaml', 'sha256': '0' * 64}]})
    elif case == 'schema':
        make_archive(archive, {'pack.yaml': b'name: x\nversion: 1\nschema_version: 2\n'}, {'schema_version': 2})
    elif case == 'extra':
        make_archive(archive, good, {'files': []})
    elif case == 'invalid':
        make_archive(archive, {**good, 'cell/objects.yaml': b'a: {object: [1]}\n'})
    else:
        archive.write_bytes(b'not a tar')
    with pytest.raises(PackfileError, match=message):
        import_cell(archive, tmp_path / 'cell', force=True)
    assert tree(tmp_path / 'cell') == before
    assert not (tmp_path / 'evil.yaml').exists()
    assert [p.name for p in tmp_path.iterdir() if p.name.startswith('.cell.')] == []


def git(cwd, *args):
    return subprocess.run(['git', '-c', 'user.name=t', '-c', 'user.email=t@t', *args], cwd=cwd, check=True,
                          capture_output=True, text=True).stdout.strip()


@pytest.mark.skipif(shutil.which('git') is None, reason='git')
def test_git_target_gets_a_commit(tmp_path):
    build_cell(tmp_path / 'a')
    git(tmp_path / 'a', 'init', '-q')
    (tmp_path / 'a' / '.gitignore').write_text('ignored.txt\n')
    (tmp_path / 'a' / 'ignored.txt').write_text('x')
    git(tmp_path / 'a', 'add', '-A')
    git(tmp_path / 'a', 'commit', '-qm', 'one')
    rev = git(tmp_path / 'a', 'rev-parse', 'HEAD')
    manifest = export_cell(tmp_path / 'a', tmp_path / 'a.tar.gz')
    assert manifest['git_rev'] == rev and 'ignored.txt' not in {f['path'] for f in manifest['files']}
    # the repo root is above the pack folder; the old pack differs; other files around it
    repo = tmp_path / 'repo'
    init_cell(repo / 'config', ROBOT, 'old')
    (repo / 'config' / 'missions' / 'stale.yaml').write_text(yaml.safe_dump(MISSION))
    (repo / 'scripts.py').write_text('x = 1\n')
    git(repo, 'init', '-q')
    git(repo, 'add', '-A')
    git(repo, 'commit', '-qm', 'base')
    (repo / 'scripts.py').write_text('x = 2\n')     # unrelated dirty file outside the pack
    report = import_cell(tmp_path / 'a.tar.gz', repo / 'config')      # clean pack: the commit is the backup
    assert report['mode'] == 'git' and report['commit']
    assert git(repo, 'log', '-1', '--format=%s') == f'import built_cell {rev}'
    assert 'scripts.py' not in git(repo, 'show', '--name-only', '--format=', 'HEAD')
    assert git(repo, 'status', '--porcelain') == 'M scripts.py'
    assert not (repo / 'config' / 'missions' / 'stale.yaml').exists()
    assert not list(tmp_path.glob('repo*.bak-*')) and not list(repo.glob('*.bak-*'))
    check_ok(repo / 'config')
    assert import_cell(tmp_path / 'a.tar.gz', repo / 'config')['commit'] is None   # nothing changed
    # uncommitted pack edits: refused without --force, snapshotted with it
    (repo / 'config' / 'cell' / 'locations.yaml').write_text('mine: {x: 1, y: 2, yaw: 0}\n')
    with pytest.raises(PackfileError, match='uncommitted'):
        import_cell(tmp_path / 'a.tar.gz', repo / 'config')
    import_cell(tmp_path / 'a.tar.gz', repo / 'config', force=True)
    assert 'snapshot before import' in git(repo, 'log', '-3', '--format=%s')


def test_cli(tmp_path, capsys):
    assert cli.main(['config', 'init', str(tmp_path / 'c'), '--robot', ROBOT]) == 0
    assert cli.main(['config', 'export', str(tmp_path / 'c'), str(tmp_path / 'c.tar.gz')]) == 0
    assert cli.main(['config', 'import', str(tmp_path / 'c.tar.gz'), str(tmp_path / 'd')]) == 0
    assert cli.main(['config', 'import', str(tmp_path / 'c.tar.gz'), str(tmp_path / 'd')]) == 1
    assert 'error' in capsys.readouterr().err
    assert cli.main(['config', 'import', str(tmp_path / 'c.tar.gz'), str(tmp_path / 'd'), '--force']) == 0
