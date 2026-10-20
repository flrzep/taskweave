"""ConfigStore.save / delete / create / save_mission / set_robot, history / show / restore, config save|history|restore
(config/save.py). The cell pack sits inside a git repo whose root is above it, as /data/frame-sim/config does."""

import multiprocessing
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

from taskweave_core import cli
from taskweave_core.config import ConfigStore, SaveConflict, SaveError, packs

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
LOCATIONS = 'cell/locations.yaml'
posix_only = pytest.mark.skipif(os.name != 'posix', reason='flock, groups and modes need a POSIX system')


def git(repo, *args) -> str:
    return subprocess.run(['git', *args], cwd=repo, capture_output=True, text=True, check=True).stdout.strip()


@pytest.fixture(autouse=True)
def no_robot_packs(monkeypatch):
    monkeypatch.delenv(packs.ROBOT_PACKS_ENV, raising=False)
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


@pytest.fixture
def repo(tmp_path) -> Path:
    root = tmp_path / 'repo'
    shutil.copytree(FIXTURE, root / 'config')
    git(root, 'init', '-q')
    git(root, 'config', 'user.name', 'Init')
    git(root, 'config', 'user.email', 'init@example.org')
    git(root, 'add', '.')
    git(root, 'commit', '-q', '-m', 'start')
    return root


@pytest.fixture
def store(repo) -> ConfigStore:
    return ConfigStore(repo / 'config')


def revs(repo) -> list[str]:
    return git(repo, 'log', '--format=%H').split()


def text(store, path=LOCATIONS) -> str:
    return (store.root / path).read_text()


# ---- text and data ---------------------------------------------------------------------------------------------------
def test_a_text_save_writes_it_as_given_and_commits_that_path(store, repo):
    new = text(store).replace('x: 1.5', 'x: 2.5').replace('3.1416}', '3.1416}   # moved')
    result = store.save(LOCATIONS, new, base=store.file_hash(LOCATIONS))
    assert text(store) == new and '# moved' in text(store) and '# start position' in text(store)
    assert result['hash'] == store.file_hash(LOCATIONS) and result['reload'] == ['live'] and result['override'] is None
    assert result['rev'] == git(repo, 'rev-parse', 'HEAD') and len(revs(repo)) == 2
    assert git(repo, 'log', '-1', '--format=%s') == 'HMI: edit cell/locations.yaml'
    assert git(repo, 'log', '-1', '--format=%an <%ae>').startswith('taskweave HMI <hmi@')
    assert store.locations()['dock'].x == 2.5       # not stale


def test_the_caller_gives_message_and_author(store, repo):
    store.save(LOCATIONS, text(store) + '\n', base=store.file_hash(LOCATIONS), message='tidy', author='Ann <ann@x.org>')
    assert git(repo, 'log', '-1', '--format=%s|%an <%ae>|%cn <%ce>') == 'tidy|Ann <ann@x.org>|Ann <ann@x.org>'


def test_data_is_merged_into_the_file_keeping_comments_and_order(store):
    data = yaml.safe_load(text(store))
    data['dock']['x'] = 2.0
    data['bay'] = {'frame': 'cell', 'x': 3.0, 'y': 0.0, 'yaw': 0.0}
    store.save(LOCATIONS, data, base=store.file_hash(LOCATIONS))
    after = text(store)
    assert after.startswith('# Named base poses in the cell frame.') and '# start position' in after
    assert after.index('home:') < after.index('dock:') < after.index('bay:')
    assert yaml.safe_load(after) == data
    del data['home']
    store.save(LOCATIONS, data, base=store.file_hash(LOCATIONS))
    assert 'home' not in yaml.safe_load(text(store))


def test_identical_content_is_no_commit(store, repo):
    result = store.save(LOCATIONS, text(store), base=store.file_hash(LOCATIONS))
    assert len(revs(repo)) == 1 and result['rev'] == revs(repo)[0]


# ---- conflicts and the base ------------------------------------------------------------------------------------------
def test_a_stale_base_conflicts_with_the_current_hash_and_text(store, repo):
    base = store.file_hash(LOCATIONS)
    theirs = text(store).replace('x: 1.5', 'x: 9.5')
    (store.root / LOCATIONS).write_text(theirs)
    with pytest.raises(SaveConflict) as error:
        store.save(LOCATIONS, text(store).replace('y: -1.0', 'y: -2.0'), base=base)
    assert error.value.current_text == theirs and error.value.current_hash == store.file_hash(LOCATIONS)
    assert text(store) == theirs and len(revs(repo)) == 1


def test_no_base_for_a_file_that_exists_is_refused_as_a_conflict(store, repo):
    before = text(store)
    with pytest.raises(SaveConflict, match='needs its hash as base') as error:
        store.save(LOCATIONS, before + '\n')
    assert error.value.current_text == before and text(store) == before and len(revs(repo)) == 1


def test_a_base_for_a_file_that_is_gone_conflicts(store):
    with pytest.raises(SaveConflict) as error:
        store.save('objects/cup.yaml', 'name: cup\n', base='0' * 64)
    assert error.value.current_hash is None and error.value.current_text is None


def test_a_new_file_needs_no_base(store, repo):
    result = store.save('objects/cup.yaml', 'name: cup\n')
    assert text(store, 'objects/cup.yaml') == 'name: cup\n' and result['rev'] and len(revs(repo)) == 2
    assert 'objects/cup.yaml' in git(repo, 'show', '--stat', '--format=', 'HEAD')


# ---- validation ------------------------------------------------------------------------------------------------------
def test_an_invalid_save_leaves_the_file_and_the_repo_alone(store, repo):
    before = text(store)
    for bad in (before.replace('x: 1.5', 'x: far'), 'home: [unclosed\n'):
        with pytest.raises(SaveError) as error:
            store.save(LOCATIONS, bad, base=store.file_hash(LOCATIONS))
        assert error.value.problems and all('severity' in p for p in error.value.problems)
    assert text(store) == before and len(revs(repo)) == 1 and git(repo, 'status', '--porcelain') == ''


def test_a_problem_the_pack_already_had_does_not_block_another_file(store):
    assert any(p['severity'] == 'error' for p in store.check_pack())    # missions/broken.yaml
    store.save(LOCATIONS, text(store) + '# note\n', base=store.file_hash(LOCATIONS))


def test_a_delete_returns_the_problems_it_causes_as_warnings(store):
    made = store.save('objects/shelf.yaml', 'name: shelf\n')
    done = store.delete('objects/shelf.yaml', base=made['hash'])
    assert any(p['path'] == 'cell/objects.yaml' and 'shelf' in p['message'] for p in done['warnings']
               if isinstance(p, dict))


def test_warnings_pass_and_are_returned(store):
    mission = {'goal': 'x', 'steps': [{'instruction': 'go', 'skill': 'not_a_skill'}]}
    result = store.save_mission('odd', mission)
    assert any('not_a_skill' in w['message'] for w in result['warnings'] if isinstance(w, dict))
    assert (store.root / 'missions/odd.yaml').is_file() and result['reload'] == ['mission_start']


def test_not_editable_files_are_refused(store):
    for path in ('pack.yaml', 'sim/unknown.txt', 'tests/x.reference.yaml'):
        with pytest.raises(SaveError, match='cannot be edited'):
            store.save(path, 'a: 1\n')
    for path in ('../outside.yaml', '/etc/x.yaml'):
        with pytest.raises(SaveError):
            store.save(path, 'a: 1\n')


# ---- files: owner, group, mode ---------------------------------------------------------------------------------------
@posix_only
def test_new_files_and_dirs_take_the_groups_and_modes(store):
    gids = [g for g in os.getgroups() + [os.getgid()] if g != os.stat(store.root).st_gid]
    gid = gids[0] if gids else (1234 if os.getuid() == 0 else os.getgid())     # a second group when there is one
    (store.root / 'robots').chmod(0o2775)
    os.chown(store.root / 'robots', -1, gid)
    old = os.umask(0o022)
    try:
        store.save('robots/new_bot/stances.yaml', {'objects': {}})
        store.save('objects/cup.yaml', 'name: cup\n')
    finally:
        os.umask(old)
    folder, made = store.root / 'robots/new_bot', store.root / 'robots/new_bot/stances.yaml'
    assert folder.stat().st_gid == gid and folder.stat().st_mode & 0o2000 and folder.stat().st_mode & 0o020
    assert made.stat().st_gid == gid and made.stat().st_mode & 0o777 == 0o664
    assert (store.root / 'objects/cup.yaml').stat().st_mode & 0o777 == 0o664


@posix_only
def test_an_existing_file_keeps_its_mode(store):
    (store.root / LOCATIONS).chmod(0o660)
    store.save(LOCATIONS, text(store) + '# x\n', base=store.file_hash(LOCATIONS))
    assert (store.root / LOCATIONS).stat().st_mode & 0o777 == 0o660
    assert not list(store.root.rglob('*.tmp'))


# ---- git -------------------------------------------------------------------------------------------------------------
def test_each_save_is_one_commit_of_that_path_only(store, repo):
    (store.root / 'objects/box.yaml').write_text('name: box\n# dirty\n')
    (store.root / 'untracked.txt').write_text('x')
    git(repo, 'add', 'config/objects/box.yaml')     # even staged, it stays out of the commit
    store.save(LOCATIONS, text(store) + '# one\n', base=store.file_hash(LOCATIONS))
    store.save(LOCATIONS, text(store) + '# two\n', base=store.file_hash(LOCATIONS))
    assert len(revs(repo)) == 3
    for rev in revs(repo)[:2]:
        assert git(repo, 'show', '--name-only', '--format=', rev) == 'config/cell/locations.yaml'
    assert 'objects/box.yaml' in git(repo, 'status', '--porcelain') and 'untracked.txt' in git(
        repo, 'status', '--porcelain')


def test_a_git_failure_after_the_write_is_a_warning(store, repo):
    (repo / '.gitignore').write_text('config/objects/cup.yaml\n')
    result = store.save('objects/cup.yaml', 'name: cup\n')
    assert result['rev'] is None and any('not committed' in w for w in result['warnings'])
    assert text(store, 'objects/cup.yaml') == 'name: cup\n'


def test_the_lock_file_stays_out_of_git_status(store, repo):
    store.save(LOCATIONS, text(store) + '# x\n', base=store.file_hash(LOCATIONS))
    assert (store.root / '.taskweave.lock').exists() and 'lock' not in git(repo, 'status', '--porcelain')


def test_without_a_repo_history_is_empty_and_save_works(config_dir):
    store = ConfigStore(config_dir)
    result = store.save(LOCATIONS, text(store) + '# x\n', base=store.file_hash(LOCATIONS))
    assert result['rev'] is None and result['warnings'] == []
    assert store.history(LOCATIONS) == []
    with pytest.raises(ValueError, match='not in a git repository'):
        store.show(LOCATIONS, 'HEAD')


# ---- the lock --------------------------------------------------------------------------------------------------------
def _save_in_child(cell, base, marker, queue):
    try:
        ConfigStore(cell).save(LOCATIONS, (Path(cell) / LOCATIONS).read_text() + f'# {marker}\n', base=base)
        queue.put('saved')
    except SaveConflict:
        queue.put('conflict')


@posix_only
def test_two_processes_saving_from_the_same_base_are_serialized(store):
    context = multiprocessing.get_context('fork')
    queue = context.Queue()
    base = store.file_hash(LOCATIONS)
    children = [context.Process(target=_save_in_child, args=(str(store.root), base, n, queue)) for n in 'ab']
    for child in children:
        child.start()
    results = sorted(queue.get(timeout=60) for _ in children)
    for child in children:
        child.join(30)
    assert results == ['conflict', 'saved']
    assert text(store).count('# ') >= 2 and text(store).count('# a') + text(store).count('# b') == 1


HOLDER = """
import sys

from taskweave_core.config.save import pack_lock

with pack_lock(sys.argv[1]):
    print('locked', flush=True)
    sys.stdin.readline()
"""


@posix_only
def test_a_save_waits_for_the_lock_another_process_holds(store):
    """The holder is a separate program (a forked child would inherit the lock's descriptor), ended by a line."""
    holder = subprocess.Popen([sys.executable, '-c', HOLDER, str(store.root)], stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE, text=True)
    assert holder.stdout.readline().strip() == 'locked'
    context = multiprocessing.get_context('fork')
    queue = context.Queue()
    child = context.Process(target=_save_in_child, args=(str(store.root), store.file_hash(LOCATIONS), 'w', queue))
    child.start()
    time.sleep(1.0)
    assert queue.empty() and '# w' not in text(store)
    holder.stdin.write('release\n')
    holder.stdin.flush()
    assert queue.get(timeout=60) == 'saved'
    child.join(30)
    holder.wait(30)


# ---- keyed overrides of robot pack files -----------------------------------------------------------------------------
@pytest.fixture
def with_pack(repo, tmp_path, monkeypatch) -> ConfigStore:
    """The fixture robot as a robot pack (outside the repo); the cell has no copy of its files."""
    pack = tmp_path / 'robot_packs' / 'demo_bot'
    pack.mkdir(parents=True)
    (pack / 'pack.yaml').write_text(f'name: demo_bot\nversion: 1\nschema_version: {packs.ROBOT_SCHEMA}\n')
    for name in ('profile.yaml', 'embodiment.yaml'):
        shutil.move(repo / 'config/robots/demo_bot' / name, pack / name)
    git(repo, 'add', '-A')
    git(repo, 'commit', '-q', '-m', 'robot files to a pack')
    monkeypatch.setenv(packs.ROBOT_PACKS_ENV, str(tmp_path / 'robot_packs'))
    return ConfigStore(repo / 'config')


def hash_of(store) -> str:
    return store.file_hash('profile.yaml', 'robot:demo_bot')


def test_a_robot_pack_file_is_saved_as_a_keyed_override_in_the_cell(with_pack, repo, tmp_path):
    store, pack_file = with_pack, tmp_path / 'robot_packs/demo_bot/profile.yaml'
    original, base = pack_file.read_bytes(), with_pack.file_hash('profile.yaml', 'robot:demo_bot')
    data = yaml.safe_load(original)
    data['base']['limits']['linear'] = 0.8
    assert store.robot('demo_bot').base['limits']['linear'] == 1.0      # cached
    result = store.save('profile.yaml', data, 'robot:demo_bot', base=base)
    override = store.root / 'robots/demo_bot/profile.yaml'
    assert pack_file.read_bytes() == original
    assert yaml.safe_load(override.read_text()) == {'~override': 'keys', 'base': {'limits': {'linear': 0.8}}}
    assert result['override'] == 'robots/demo_bot/profile.yaml' and result['reload'] == ['launch']
    assert result['hash'] == store.file_hash('profile.yaml', 'robot:demo_bot') != base
    assert git(repo, 'show', '--name-only', '--format=', 'HEAD') == 'config/robots/demo_bot/profile.yaml'
    assert store.robot('demo_bot').base['limits']['linear'] == 0.8     # no stale cache
    entry = next(e for e in store.files() if e['path'] == 'profile.yaml' and e['pack'] == 'robot:demo_bot')
    assert entry['hash'] == result['hash']
    with pytest.raises(SaveConflict):
        store.save('profile.yaml', data, 'robot:demo_bot', base=base)
    # back to the pack's value: nothing left to override
    data['base']['limits']['linear'] = 1.0
    store.save('profile.yaml', data, 'robot:demo_bot', base=result['hash'])
    assert yaml.safe_load(override.read_text()) == {'~override': 'keys'}
    assert store.robot('demo_bot').base['limits']['linear'] == 1.0


def test_an_invalid_robot_pack_edit_writes_nothing(with_pack, repo):
    data = yaml.safe_load((with_pack.robot_file('demo_bot', 'profile.yaml')).read_text())
    data['base']['limits'] = 'fast'
    with pytest.raises(SaveError):
        with_pack.save('profile.yaml', data, 'robot:demo_bot', base=hash_of(with_pack))
    assert not (with_pack.root / 'robots/demo_bot/profile.yaml').exists() and len(revs(repo)) == 2


def test_delete_of_an_override_brings_the_pack_file_back(with_pack, repo):
    data = yaml.safe_load(with_pack.robot_file('demo_bot', 'profile.yaml').read_text())
    data['base']['limits']['linear'] = 0.5
    done = with_pack.save('profile.yaml', data, 'robot:demo_bot', base=hash_of(with_pack))
    with_pack.delete('profile.yaml', 'robot:demo_bot', base=done['hash'])
    assert with_pack.robot('demo_bot').base['limits']['linear'] == 1.0
    with pytest.raises(SaveError, match='nothing to delete'):
        with_pack.delete('profile.yaml', 'robot:demo_bot', base=with_pack.file_hash('profile.yaml', 'robot:demo_bot'))


# ---- delete, create, mission, robot ----------------------------------------------------------------------------------
def test_delete_needs_the_base_and_commits_the_removal(store, repo):
    with pytest.raises(SaveConflict):
        store.delete('objects/box.yaml')
    with pytest.raises(SaveConflict):
        store.delete('objects/box.yaml', base='1' * 64)
    result = store.delete('objects/box.yaml', base=store.file_hash('objects/box.yaml'))
    assert not (store.root / 'objects/box.yaml').exists() and result['hash'] is None and result['rev']
    assert git(repo, 'show', '--name-status', '--format=', 'HEAD') == 'D\tconfig/objects/box.yaml'
    assert git(repo, 'log', '-1', '--format=%s') == 'HMI: delete objects/box.yaml'
    with pytest.raises(SaveError, match='no such file'):
        store.delete('objects/box.yaml', base='x')


def test_create_a_file_of_a_kind(store):
    result = store.create('object_type', 'cup')
    assert yaml.safe_load(text(store, 'objects/cup.yaml')) == {'name': 'cup'} and result['rev']
    with pytest.raises(SaveConflict):
        store.create('object_type', 'cup')
    with pytest.raises(ValueError):
        store.create('object_type', 'not a name')


def test_save_mission_validates_with_the_mission_checks(store):
    data = {'goal': 'Wait', 'robot': 'demo_bot', 'steps': [{'id': 'a', 'instruction': 'w', 'skill': 'wait'}]}
    result = store.save_mission('new_one', data)
    assert yaml.safe_load(text(store, 'missions/new_one.yaml')) == data
    data['steps'].append({'id': 'a', 'instruction': 'again', 'skill': 'wait'})
    with pytest.raises(SaveError, match='more than once'):
        store.save_mission('new_one', data, base=result['hash'])
    with pytest.raises(SaveError):
        store.save_mission('bad name', data)
    assert store.mission('new_one')['steps'][0]['id'] == 'a'


def test_set_robot_goes_through_save(store, repo):
    (store.root / 'system.yaml').write_text('# the installation\nrobot_packs: []\n')
    git(repo, 'add', '-A')
    git(repo, 'commit', '-q', '-m', 'system')
    result = store.set_robot('demo_bot')
    assert yaml.safe_load(text(store, 'system.yaml')) == {'robot_packs': [], 'robot': 'demo_bot'}
    assert text(store, 'system.yaml').startswith('# the installation') and 'launch' in result['reload']
    assert result['rev'] and store.system()['robot'] == 'demo_bot'
    with pytest.raises(SaveError, match='no robot'):
        store.set_robot('ghost')


# ---- caches ----------------------------------------------------------------------------------------------------------
def test_pack_yaml_is_re_read_when_only_its_size_changed(tmp_path):
    folder = tmp_path / 'p'
    folder.mkdir()
    file = folder / 'pack.yaml'
    file.write_text(f'name: p\nversion: 1\nschema_version: {packs.ROBOT_SCHEMA}\n')
    stamp = file.stat().st_mtime_ns
    assert packs.read_pack(folder)['version'] == 1
    file.write_text(f'name: p\nversion: 22\nschema_version: {packs.ROBOT_SCHEMA}\n')
    os.utime(file, ns=(stamp, stamp))
    assert packs.read_pack(folder)['version'] == 22


def test_a_save_in_the_same_clock_tick_is_not_served_stale(store):
    file = store.root / LOCATIONS
    assert store.locations()['dock'].x == 1.5
    stamp = file.stat().st_mtime_ns
    store.save(LOCATIONS, text(store).replace('x: 1.5', 'x: 2.5'), base=store.file_hash(LOCATIONS))
    os.utime(file, ns=(stamp, stamp))
    assert store.locations()['dock'].x == 2.5


# ---- history, show, restore ------------------------------------------------------------------------------------------
def test_history_show_and_restore(store, repo):
    original = text(store)
    store.save(LOCATIONS, original.replace('x: 1.5', 'x: 2.5'), base=store.file_hash(LOCATIONS), message='first')
    store.save(LOCATIONS, text(store).replace('x: 2.5', 'x: 3.5'), base=store.file_hash(LOCATIONS),
               author='Bo <bo@x.org>')
    rows = store.history(LOCATIONS)
    assert [r['message'] for r in rows] == ['HMI: edit cell/locations.yaml', 'first', 'start']
    assert rows[0]['author'] == 'Bo <bo@x.org>' and rows[0]['rev'] == git(repo, 'rev-parse', 'HEAD')
    assert rows[0]['time'].startswith('20') and 'T' in rows[0]['time'] and len(store.history(LOCATIONS, limit=2)) == 2
    assert store.show(LOCATIONS, rows[2]['rev']) == original and store.show(LOCATIONS, 'HEAD~1') != original
    done = store.restore(LOCATIONS, rows[2]['rev'], base=store.file_hash(LOCATIONS), author='Cy <cy@x.org>')
    assert text(store) == original and done['rev']
    assert git(repo, 'log', '-1', '--format=%s|%an') == f'restore cell/locations.yaml to {rows[2]["rev"][:8]}|Cy'
    assert len(store.history(LOCATIONS)) == 4
    with pytest.raises(ValueError):
        store.show(LOCATIONS, 'nonsense')
    with pytest.raises(ValueError):
        store.show(LOCATIONS, '--output=x')


def test_restore_conflicts_on_a_stale_base(store):
    base = store.file_hash(LOCATIONS)
    store.save(LOCATIONS, text(store).replace('x: 1.5', 'x: 2.5'), base=base)
    start = store.history(LOCATIONS)[-1]['rev']
    with pytest.raises(SaveConflict) as error:
        store.restore(LOCATIONS, start, base=base)
    assert error.value.current_text == text(store) and 'x: 2.5' in text(store)


def test_history_follows_a_rename(store, repo):
    git(repo, 'mv', 'config/objects/box.yaml', 'config/objects/crate.yaml')
    git(repo, 'commit', '-q', '-m', 'rename')
    assert [r['message'] for r in store.history('objects/crate.yaml')] == ['rename', 'start']


# ---- command line ----------------------------------------------------------------------------------------------------
def test_cli_save_history_restore(store, repo, tmp_path, capsys):
    new = tmp_path / 'new.yaml'
    new.write_text(text(store).replace('x: 1.5', 'x: 2.5'))
    cell, target = str(store.root), str(store.root / LOCATIONS)
    assert cli.main(['config', 'save', target, '--from', str(new)]) == 3        # exists: needs --base
    assert 'conflict' in capsys.readouterr().err
    assert cli.main(['config', 'save', target, '--from', str(new), '--base', store.file_hash(LOCATIONS),
                     '-m', 'by cli']) == 0
    assert 'saved' in capsys.readouterr().out and 'x: 2.5' in text(store)
    new.write_text('home: [open\n')
    assert cli.main(['config', 'save', LOCATIONS, '--cell', cell, '--from', str(new), '--base',
                     store.file_hash(LOCATIONS)]) == 1
    assert 'nothing written' in capsys.readouterr().err
    assert cli.main(['config', 'history', target]) == 0
    out = capsys.readouterr().out
    assert 'by cli' in out and 'start' in out
    start = store.history(LOCATIONS)[-1]['rev'][:8]
    assert cli.main(['config', 'restore', target, start, '--base', store.file_hash(LOCATIONS)]) == 0
    assert 'x: 1.5' in text(store)


# ---- P4 review: paths and robot names stay inside the pack ----------------------------------------------------------
EVIL = ('robot:../../../tmp/x', 'robot:..', 'robot:a/b', 'robot:/abs', 'robot:1bad', 'robot:')


def test_a_robot_name_stays_inside_the_pack(store, repo, tmp_path):
    outside = tmp_path / 'tmp' / 'x'
    for pack in EVIL:
        with pytest.raises((SaveError, ValueError)):
            store.save('profile.yaml', 'a: 1\n', pack=pack)
        with pytest.raises((SaveError, ValueError)):
            store.delete('profile.yaml', pack=pack, base='x')
        with pytest.raises((SaveError, ValueError)):
            store.show('profile.yaml', 'HEAD', pack=pack)
        with pytest.raises((SaveError, ValueError)):
            store.history('profile.yaml', pack=pack)
    assert not outside.exists() and not any(tmp_path.glob('**/x/profile.yaml'))


def test_a_conflict_stays_inside_the_pack(store, tmp_path):
    (tmp_path / 'secret').mkdir()
    (tmp_path / 'secret' / 'profile.yaml').write_text('secret: 1\n')
    with pytest.raises((SaveError, ValueError)) as error:
        store.save('profile.yaml', 'a: 1\n', pack='robot:../../../secret')
    assert 'secret: 1' not in str(error.value)


@posix_only
def test_symlinks_stays_inside_the_pack(store, tmp_path):
    outside = tmp_path / 'outside'
    outside.mkdir()
    (store.root / 'linked').symlink_to(outside)
    with pytest.raises(SaveError):
        store.save('missions/../linked/x.yaml', 'a: 1\n')
    (store.root / 'missions' / 'ln.yaml').symlink_to(outside / 'f.yaml')
    with pytest.raises(SaveError):
        store.save('missions/ln.yaml', 'goal: x\nsteps: []\n')
    (store.root / 'robots' / 'link_bot').symlink_to(outside)
    with pytest.raises(SaveError):
        store.save('profile.yaml', 'a: 1\n', pack='robot:link_bot')
    assert not list(outside.iterdir())


# ---- P4 review: old errors are not new ones ------------------------------------------------------------------------
def test_a_cross_file_error_the_pack_had_does_not_block_a_valid_save(store):
    (store.root / 'missions' / 'outer.yaml').write_text(
        'goal: g\nsteps:\n  - instruction: x\n    include: gone_mission\n')
    errors = [p for p in store.check_pack() if p['path'] == 'missions/outer.yaml' and p['severity'] == 'error']
    assert errors and str(store.root) in errors[0]['message']    # a message with the real path
    store.reload()
    store.save(LOCATIONS, text(store) + '# note\n', base=store.file_hash(LOCATIONS))
    store.save('missions/ok.yaml', 'goal: g\nsteps:\n  - instruction: w\n    skill: wait\n')


# ---- P4 review: the file is hashed again right before the write ----------------------------------------------------
def test_a_change_during_validation_is_a_conflict(store, monkeypatch):
    from taskweave_core.config import save as save_module

    real = save_module._validate
    other = text(store) + '# by an ssh editor\n'

    def slow(*args, **kwargs):
        result = real(*args, **kwargs)
        (store.root / LOCATIONS).write_text(other)     # nobody holds the flock
        return result

    monkeypatch.setattr(save_module, '_validate', slow)
    base = store.file_hash(LOCATIONS)
    with pytest.raises(SaveConflict) as error:
        store.save(LOCATIONS, text(store).replace('x: 1.5', 'x: 2.5'), base=base)
    assert text(store) == other and error.value.current_text == other


def test_a_change_during_the_delete_check_is_a_conflict(store, monkeypatch):
    from taskweave_core.config import check

    made = store.save('objects/shelf.yaml', 'name: shelf\n')
    real = check.check_pack
    calls = []

    def slow(s):
        calls.append(1)
        if len(calls) == 1:
            (s.root / 'objects/shelf.yaml').write_text('name: shelf\n# changed\n')
        return real(s)

    monkeypatch.setattr(check, 'check_pack', slow)
    with pytest.raises(SaveConflict):
        store.delete('objects/shelf.yaml', base=made['hash'])
    assert (store.root / 'objects/shelf.yaml').is_file()


# ---- P4 review: an invalid system.yaml can be fixed through the API ------------------------------------------------
@pytest.mark.parametrize('broken', ['robot_packs: 5\n', 'robot: [unclosed\n'])
def test_an_invalid_system_yaml_is_a_problem_and_can_be_fixed_by_a_save(store, broken, capsys):
    (store.root / 'system.yaml').write_text(broken)
    store.reload()
    problems = store.check_pack()
    assert any(p['path'] == 'system.yaml' and p['severity'] == 'error' for p in problems)
    assert cli.check(str(store.root)) == 1
    assert 'FAIL  system.yaml' in capsys.readouterr().out
    store.save('missions/ok.yaml', 'goal: g\nsteps:\n  - instruction: w\n    skill: wait\n')   # other files still work
    store.save('system.yaml', 'robot: demo_bot\n', base=store.file_hash('system.yaml'))
    assert text(store, 'system.yaml') == 'robot: demo_bot\n' and not any(
        p['path'] == 'system.yaml' for p in store.check_pack())
    store.delete('missions/ok.yaml', base=store.file_hash('missions/ok.yaml'))


# ---- P4 review: shared git repository ------------------------------------------------------------------------------
def _object_dirs(repo) -> set:
    return {p for p in (repo / '.git' / 'objects').iterdir() if p.is_dir() and len(p.name) == 2}


@posix_only
def test_the_shared_repo_commit_runs_with_umask_002(store, repo):
    old = os.umask(0o022)
    try:
        before = _object_dirs(repo)
        store.save(LOCATIONS, text(store) + '# note\n', base=store.file_hash(LOCATIONS))
    finally:
        os.umask(old)
    made = _object_dirs(repo) - before
    assert made and all(d.stat().st_mode & 0o020 for d in made)


@posix_only
def test_a_shared_repo_without_shared_repository_warns(store, repo):
    objects = repo / '.git' / 'objects'
    save = lambda: store.save(LOCATIONS, text(store) + '# note\n', base=store.file_hash(LOCATIONS))   # noqa: E731
    assert not any('sharedRepository' in str(w) for w in save()['warnings'])    # no setgid: nothing to say
    objects.chmod(objects.stat().st_mode | 0o2000)
    assert any('core.sharedRepository group' in str(w) for w in save()['warnings'])
    git(repo, 'config', 'core.sharedRepository', 'group')
    assert not any('sharedRepository' in str(w) for w in save()['warnings'])
