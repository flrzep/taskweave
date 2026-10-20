"""pack.yaml requires.taskweave: a pack that does not admit this taskweave is refused, naming both versions."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigError, ConfigStore, packs

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
ROBOT = 'demo_bot'


@pytest.fixture(autouse=True)
def fixed_version(monkeypatch):
    monkeypatch.delenv(packs.ROBOT_PACKS_ENV, raising=False)
    monkeypatch.setattr(packs, 'installed_location', lambda: None)
    monkeypatch.setattr(packs, 'taskweave_version', lambda: '0.2.3')


def cell(tmp_path, name: str, requires: str | None) -> Path:
    folder = tmp_path / name
    shutil.copytree(FIXTURE, folder)
    extra = '' if requires is None else f'requires:\n  taskweave: "{requires}"\n'
    (folder / 'pack.yaml').write_text(f'name: test_cell\nversion: 7\nschema_version: 3\n{extra}')
    return folder


def test_compatible_cell(tmp_path):
    ConfigStore(cell(tmp_path, 'a', '>=0.2,<0.3'))
    ConfigStore(cell(tmp_path, 'b', '~=0.2.1'))
    ConfigStore(cell(tmp_path, 'c', '==0.2.3'))


def test_incompatible_cell_names_both_versions(tmp_path):
    with pytest.raises(ConfigError, match=r'cell pack test_cell 7 requires taskweave >=0\.3,<0\.4, this is '
                                          r'taskweave 0\.2\.3'):
        ConfigStore(cell(tmp_path, 'a', '>=0.3,<0.4'))
    with pytest.raises(ConfigError, match='requires taskweave !=0.2.3'):
        ConfigStore(cell(tmp_path, 'b', '!=0.2.3'))


def test_missing_requires_is_allowed(tmp_path):
    ConfigStore(cell(tmp_path, 'a', None))


def test_malformed_spec(tmp_path):
    with pytest.raises(ConfigError, match='not a version specifier'):
        ConfigStore(cell(tmp_path, 'a', 'newer than 1'))


def test_robot_pack_is_checked_too(tmp_path):
    store = ConfigStore(cell(tmp_path, 'a', None))
    pack = tmp_path / 'packs' / ROBOT
    pack.mkdir(parents=True)
    (pack / 'pack.yaml').write_text(f'name: {ROBOT}\nversion: 0.1.0\nschema_version: {packs.ROBOT_SCHEMA}\n'
                                    'requires:\n  taskweave: ">=1.0"\n')
    location = packs.Location(tmp_path / 'packs', 'env', True)
    with pytest.raises(ConfigError, match=r'robot pack demo_bot 0\.1\.0 requires taskweave >=1\.0, this is '
                                          r'taskweave 0\.2\.3'):
        packs.robot_pack(location, ROBOT)
    ok = tmp_path / 'ok' / ROBOT    # (a pack.yaml is read again only when its mtime changes: a folder of its own)
    ok.mkdir(parents=True)
    (ok / 'pack.yaml').write_text(f'name: {ROBOT}\nversion: 0.1.0\nschema_version: {packs.ROBOT_SCHEMA}\n'
                                  'requires:\n  taskweave: ">=0.2,<0.3"\n')
    assert packs.robot_pack(packs.Location(tmp_path / 'ok', 'env', True), ROBOT) is not None
    assert store


def test_shipped_packs_admit_this_taskweave(monkeypatch):
    """The example cell and the bundled robot packs require a range that contains the version of the package."""
    import re

    import yaml
    from packaging.specifiers import SpecifierSet

    monkeypatch.undo()
    root = Path(__file__).resolve().parents[2]
    version = re.search(r'<version>([^<]+)</version>', (root / 'taskweave_core' / 'package.xml').read_text()).group(1)
    found = sorted([*root.glob('taskweave_robots/*/pack.yaml'), root / 'docs' / 'examples' / 'cell' / 'pack.yaml'])
    assert len(found) >= 4  # the bundled robot packs (one more with each new robot) and the example cell
    for path in found:
        spec = yaml.safe_load(path.read_text())['requires']['taskweave']
        assert SpecifierSet(spec).contains(version), path
