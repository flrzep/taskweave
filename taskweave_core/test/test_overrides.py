"""Per-key cell overrides of robot pack files (config/overlay.py, slim.py): merge rules, saving, compatibility."""

import shutil
from pathlib import Path

import pytest
import yaml

from taskweave_core.config import ConfigError, ConfigStore, overlay, packs
from taskweave_core.config.slim import slim

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
ROBOT = 'demo_bot'
NAV = ('# robot pack navigation\nenvelope: {front: 0.5, back: 0.4, left: 0.3, right: 0.3}\npadding: 0.1\n'
       'driving:\n  max_forward: 0.5\n  max_turn: 1.0\n  sideways: false\n')
POSES = '# robot pack poses\nhome: {lift: 0.1}\nready: {lift: 0.2}\n'


@pytest.fixture(autouse=True)
def no_outside_packs(monkeypatch):
    monkeypatch.delenv(packs.ROBOT_PACKS_ENV, raising=False)
    monkeypatch.setattr(packs, 'installed_location', lambda: None)


@pytest.fixture
def cell(tmp_path):
    """Cell pack + a robot pack (profile, embodiment, navigation, poses) the cell can override."""
    cell = tmp_path / 'cell'
    shutil.copytree(FIXTURE, cell)
    pack = tmp_path / 'packs' / ROBOT
    pack.mkdir(parents=True)
    (pack / 'pack.yaml').write_text(f'name: {ROBOT}\nversion: 0.1.0\nschema_version: {packs.ROBOT_SCHEMA}\n')
    for name in ('profile.yaml', 'embodiment.yaml'):
        shutil.move(cell / 'robots' / ROBOT / name, pack / name)
    (pack / 'navigation.yaml').write_text(NAV)
    (pack / 'poses.yaml').write_text(POSES)
    (cell / 'system.yaml').write_text('robot: demo_bot\nrobot_packs: [../packs]\n')
    return cell, pack


def override(cell: Path, file: str, text: str) -> Path:
    path = cell / 'robots' / ROBOT / file
    path.write_text(text)
    return path


def installed_only(root, pack, tmp_path, monkeypatch):
    """Move the robot pack to a read-only 'installed' location: edits become cell overrides."""
    (root / 'system.yaml').write_text('robot: demo_bot\n')
    shutil.move(pack, tmp_path / 'installed' / ROBOT)
    monkeypatch.setattr(packs, 'installed_location', lambda: tmp_path / 'installed')
    return tmp_path / 'installed' / ROBOT


def test_merge_rules():
    base = {'a': {'b': 1, 'c': {'d': 2, 'e': 3}}, 'list': [1, 2, 3], 's': 'x'}
    over = {'a': {'c': {'d': 9}}, 'list': [7], 'new': {'k': 1}}
    assert overlay.merge(base, over) == {'a': {'b': 1, 'c': {'d': 9, 'e': 3}}, 'list': [7], 's': 'x',
                                         'new': {'k': 1}}
    assert base['a']['c']['d'] == 2                                     # the pack's data is not touched
    assert overlay.merge(base, {'a': {'c': {'e': overlay.DELETE}}, 's': overlay.DELETE}) == {
        'a': {'b': 1, 'c': {'d': 2}}, 'list': [1, 2, 3]}
    assert overlay.merge(base, {'ghost': overlay.DELETE}) == base       # deleting what the pack lacks: nothing
    assert overlay.merge({'a': {'b': 1}}, {'a': 5}) == {'a': 5}          # scalar replaces a mapping
    assert overlay.merge({'a': 1}, {'a': {'b': 2}}) == {'a': {'b': 2}}
    assert overlay.merge({'x': {'j': 1, 'k': 2}}, {'x': {'j': 5}}, depth=1) == {'x': {'j': 5}}   # replaced whole
    assert overlay.merge({'x': {'j': 1, 'k': 2}}, {'x': {'j': 5}}, depth=2) == {'x': {'j': 5, 'k': 2}}


def test_delta_is_the_inverse_of_merge():
    base = {'a': {'b': 1, 'c': 2}, 'l': [1, 2], 'gone': 1, 'f': 1.0}
    data = {'a': {'b': 1, 'c': 3}, 'l': [1, 2, 3], 'f': 1, 'new': 0}
    changed = overlay.delta(base, data)
    assert changed == {'a': {'c': 3}, 'l': [1, 2, 3], 'f': 1, 'new': 0, 'gone': overlay.DELETE}  # 1 is not 1.0
    assert overlay.merge(base, changed) == data
    assert overlay.delta(base, base) == {}


def test_keyed_override_loads_over_the_pack_file(cell):
    root, _ = cell
    override(root, 'navigation.yaml', '~override: keys\ndriving: {max_forward: 0.9}\npadding: ~delete\n')
    nav = ConfigStore(root).navigation(ROBOT)
    assert nav['driving'] == {'max_forward': 0.9, 'max_turn': 1.0, 'sideways': False}
    assert 'padding' not in nav and nav['envelope']['front'] == 0.5


def test_poses_merge_one_level_a_pose_is_replaced_whole(cell):
    root, _ = cell
    override(root, 'poses.yaml', '~override: keys\nhome: {lift: 0.5, tilt: 0.1}\nready: ~delete\nextra: {lift: 0.3}\n')
    poses = ConfigStore(root).joint_poses(ROBOT)
    assert poses == {'home': {'lift': 0.5, 'tilt': 0.1}, 'extra': {'lift': 0.3}}


def test_the_merged_result_is_validated(cell):
    root, _ = cell
    override(root, 'navigation.yaml', '~override: keys\nenvelope: ~delete\n')       # envelope is required
    with pytest.raises(ConfigError, match='envelope'):
        ConfigStore(root).navigation(ROBOT)
    override(root, 'navigation.yaml', '~override: keys\ndriving: {max_forward: -1}\n')
    with pytest.raises(ConfigError, match='max_forward'):
        ConfigStore(root).navigation(ROBOT)


def test_the_cache_follows_the_pack_file_and_the_override(cell):
    root, pack = cell
    override(root, 'navigation.yaml', '~override: keys\npadding: 0.2\n')
    store = ConfigStore(root)
    assert store.navigation(ROBOT)['driving']['max_turn'] == 1.0
    (pack / 'navigation.yaml').write_text(NAV.replace('max_turn: 1.0', 'max_turn: 2.0') + '# longer file\n')
    assert store.navigation(ROBOT)['driving']['max_turn'] == 2.0 and store.navigation(ROBOT)['padding'] == 0.2


def test_save_writes_only_the_delta(cell):
    root, pack = cell
    store = ConfigStore(root)
    override(root, 'navigation.yaml', '~override: keys\npadding: 0.2\n')              # a cell override exists: edits go there
    path = store.save_navigation(ROBOT, {'driving': {'max_forward': 0.7}})
    assert path == root / 'robots' / ROBOT / 'navigation.yaml'
    assert yaml.safe_load(path.read_text()) == {'~override': 'keys', 'padding': 0.2, 'driving': {'max_forward': 0.7}}
    assert (pack / 'navigation.yaml').read_text() == NAV
    assert store.navigation(ROBOT)['driving'] == {'max_forward': 0.7, 'max_turn': 1.0, 'sideways': False}
    store.save_navigation(ROBOT, {'padding': 0.1})                                      # back to the pack's value
    assert yaml.safe_load(path.read_text()) == {'~override': 'keys', 'driving': {'max_forward': 0.7}}


def test_read_only_pack_edit_creates_a_keyed_override(cell, tmp_path, monkeypatch):
    root, pack = cell
    installed = installed_only(root, pack, tmp_path, monkeypatch)
    store = ConfigStore(root)
    path = store.set_joint_pose(ROBOT, 'ready', {'lift': 0.25})
    assert path == root / 'robots' / ROBOT / 'poses.yaml'
    assert yaml.safe_load(path.read_text()) == {'~override': 'keys', 'ready': {'lift': 0.25}}
    assert store.joint_poses(ROBOT) == {'home': {'lift': 0.1}, 'ready': {'lift': 0.25}}
    store.set_joint_pose(ROBOT, 'home', rename='base')                                  # a pack key goes by ~delete
    assert yaml.safe_load(path.read_text())['home'] == overlay.DELETE
    assert set(store.joint_poses(ROBOT)) == {'base', 'ready'}
    assert (installed / 'poses.yaml').read_text() == POSES


def test_a_pack_update_to_another_key_reaches_the_cell(cell, tmp_path, monkeypatch):
    root, pack = cell
    installed = installed_only(root, pack, tmp_path, monkeypatch)
    store = ConfigStore(root)
    store.set_joint_pose(ROBOT, 'ready', {'lift': 0.25})                                # the cell changes `ready`
    (installed / 'poses.yaml').write_text(POSES.replace('home: {lift: 0.1}', 'home: {lift: 0.15}')
                                          + 'stow: {lift: 0.0}\n')
    poses = store.joint_poses(ROBOT)
    assert poses['home'] == {'lift': 0.15} and poses['stow'] == {'lift': 0.0}           # the update is not shadowed
    assert poses['ready'] == {'lift': 0.25}                                             # the cell's change stays


def test_a_full_copy_override_loads_the_same(cell):
    root, _ = cell
    full = NAV.replace('max_turn: 1.0', 'max_turn: 3.0')
    override(root, 'navigation.yaml', full)
    store = ConfigStore(root)
    assert store.navigation(ROBOT) == yaml.safe_load(full)
    assert store.robot_file_source(ROBOT, 'navigation.yaml')['override'] == 'file'


def test_edit_of_a_full_copy_becomes_a_delta(cell):
    root, _ = cell
    path = override(root, 'navigation.yaml', NAV.replace('max_turn: 1.0', 'max_turn: 3.0'))
    store = ConfigStore(root)
    store.save_navigation(ROBOT, {'padding': 0.3})
    assert yaml.safe_load(path.read_text()) == {'~override': 'keys', 'driving': {'max_turn': 3.0}, 'padding': 0.3}
    assert store.navigation(ROBOT)['padding'] == 0.3 and store.navigation(ROBOT)['driving']['max_forward'] == 0.5


def test_cell_only_files_are_unchanged(cell):
    root, _ = cell
    store = ConfigStore(root)
    path = store.save_held_pose(ROBOT, 'box', 'carry', {'lift': 0.1}, [])
    assert '~override' not in path.read_text()


def test_source_info_counts_the_keys(cell):
    root, _ = cell
    store = ConfigStore(root)
    source = store.robot_file_source(ROBOT, 'navigation.yaml')
    assert source['source'] == 'robot pack' and source['override'] is None and source['override_count'] == 0
    override(root, 'navigation.yaml', '~override: keys\ndriving: {max_forward: 0.9, sideways: true}\npadding: ~delete\n')
    source = store.robot_file_source(ROBOT, 'navigation.yaml')
    assert source['source'] == 'override' and source['override'] == 'keys' and source['pack']      # old keys stay
    assert source['override_keys'] == ['driving.max_forward', 'driving.sideways', 'padding']
    assert source['override_count'] == 3 and source['override_deleted'] == ['padding']
    override(root, 'poses.yaml', 'home: {lift: 0.1}\nready: {lift: 0.9}\nnew: {lift: 0}\n')   # whole file: what differs
    source = store.robot_file_source(ROBOT, 'poses.yaml')
    assert source['override'] == 'file' and sorted(source['override_keys']) == ['new', 'ready']
    assert source['override_count'] == 2 and source['override_deleted'] == []


def test_slim_dry_run_real_and_idempotent(cell):
    root, _ = cell
    nav = override(root, 'navigation.yaml', NAV.replace('max_turn: 1.0', 'max_turn: 3.0'))
    poses = override(root, 'poses.yaml', POSES)                                          # same as the pack
    store = ConfigStore(root)
    before = (store.navigation(ROBOT), store.joint_poses(ROBOT))
    text = nav.read_text()
    lines = []
    assert slim(root, dry_run=True, say=lines.append) == 0
    assert nav.read_text() == text and poses.exists()
    assert any('slim' in line and 'driving.max_turn' in line for line in lines)
    assert any('remove' in line for line in lines)
    lines.clear()
    assert slim(root, say=lines.append) == 0
    assert yaml.safe_load(nav.read_text()) == {'~override': 'keys', 'driving': {'max_turn': 3.0}}
    assert not poses.exists()
    assert any('drops' in line and 'envelope' in line for line in lines)
    fresh = ConfigStore(root)
    assert (fresh.navigation(ROBOT), fresh.joint_poses(ROBOT)) == before
    slimmed, again = nav.read_text(), []
    assert slim(root, say=again.append) == 0
    assert nav.read_text() == slimmed and again[-1] == 'nothing to slim'


# ---- P1 review fix 1: a whole-file copy lacking pack keys is not a deletion of them ----
def test_slim_of_an_old_copy_takes_new_pack_keys_from_the_pack(cell):
    """An old full copy predates the pack's `padding` and `driving.sideways`: missing means never had, not deleted."""
    root, _ = cell
    old = override(root, 'navigation.yaml', 'envelope: {front: 0.5, back: 0.4, left: 0.3, right: 0.3}\n'
                                            'driving:\n  max_forward: 0.9\n  max_turn: 1.0\n')
    assert 'padding' not in ConfigStore(root).navigation(ROBOT)     # as a whole file it replaces the pack's
    assert slim(root, say=lambda line: None) == 0
    assert yaml.safe_load(old.read_text()) == {'~override': 'keys', 'driving': {'max_forward': 0.9}}
    nav = ConfigStore(root).navigation(ROBOT)
    assert nav['padding'] == 0.1 and nav['driving']['sideways'] is False and nav['driving']['max_forward'] == 0.9


def test_slim_leaves_an_explicit_delete_alone(cell):
    root, _ = cell
    keyed = override(root, 'navigation.yaml', '~override: keys\npadding: ~delete\n')
    text = keyed.read_text()
    assert slim(root, say=lambda line: None) == 0
    assert keyed.read_text() == text
    assert 'padding' not in ConfigStore(root).navigation(ROBOT)


def test_editing_a_whole_copy_writes_delete_only_for_what_the_edit_removes(cell):
    root, _ = cell
    path = override(root, 'navigation.yaml', 'envelope: {front: 0.5, back: 0.4, left: 0.3, right: 0.3}\n'
                                             'driving:\n  max_forward: 0.5\n  max_turn: 1.0\npadding: 0.1\n')
    ConfigStore(root).save_navigation(ROBOT, {})
    assert yaml.safe_load(path.read_text()) == {'~override': 'keys'}    # sideways: never had it, not deleted
    document, _ = ConfigStore(root).keyed_override(ROBOT, 'navigation.yaml', edit=lambda doc: doc.pop('padding'))
    assert yaml.safe_load(yaml.dump(dict(document))) == {'~override': 'keys', 'padding': overlay.DELETE}


# ---- P1 review fix 4: a keyed override needs a robot pack file to lie over ----
def test_a_keyed_override_without_a_robot_pack_file_is_an_error(cell):
    root, pack = cell
    (pack / 'navigation.yaml').unlink()
    override(root, 'navigation.yaml', '~override: keys\ndriving: {max_turn: 2.0}\n')
    with pytest.raises(ConfigError, match=r'no robot pack has demo_bot/navigation.yaml \(looked in .*packs'):
        ConfigStore(root).navigation(ROBOT)


# ---- P1 review fix 7: slim never touches what a cell entry links to outside the cell pack ----
def _symlink_or_skip(link: Path, target: Path) -> None:
    try:
        link.symlink_to(target, target_is_directory=target.is_dir())
    except (OSError, NotImplementedError):
        pytest.skip('no symlink support here')


def test_slim_does_not_delete_robot_pack_files_through_a_linked_robot_folder(cell):
    root, pack = cell
    shutil.rmtree(root / 'robots' / ROBOT)
    _symlink_or_skip(root / 'robots' / ROBOT, pack)                 # the cell's robot folder IS the pack's
    before = {p.name: p.read_text() for p in pack.iterdir() if p.is_file()}
    lines = []
    assert slim(root, say=lines.append) == 0
    assert {p.name: p.read_text() for p in pack.iterdir() if p.is_file()} == before
    assert any('skip' in line and 'robot pack' in line for line in lines)


def test_slim_skips_an_entry_linked_to_the_pack_file(cell):
    root, pack = cell
    link = root / 'robots' / ROBOT / 'navigation.yaml'
    _symlink_or_skip(link, pack / 'navigation.yaml')
    lines = []
    assert slim(root, say=lines.append) == 0
    assert link.is_symlink() and (pack / 'navigation.yaml').read_text() == NAV
    assert any(line.startswith('skip') and 'navigation.yaml' in line for line in lines)


def test_slim_does_not_write_outside_the_cell_pack(cell, tmp_path):
    root, _ = cell
    outside = tmp_path / 'elsewhere' / 'navigation.yaml'
    outside.parent.mkdir()
    text = NAV.replace('max_turn: 1.0', 'max_turn: 3.0')
    outside.write_text(text)
    _symlink_or_skip(root / 'robots' / ROBOT / 'navigation.yaml', outside)
    assert slim(root, say=lambda line: None) == 0
    assert outside.read_text() == text
