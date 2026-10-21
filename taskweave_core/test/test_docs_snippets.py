"""The YAML snippets of the docs are what the config accepts. Every ```yaml block of docs/targets.md and
docs/migration_targets.md parses. A block whose first line is ``# example: <path>`` is a whole file of the example cell
(docs/examples/cell, its missions and skill files): it is laid in place of that file and the pack is checked, so the
snippet must be valid and leave the rest of the cell valid. Mission blocks are checked by test_docs_examples in
taskweave_skills (the docs show those files verbatim)."""

import re
import shutil
from pathlib import Path

import pytest
import yaml

from taskweave_core.config import ConfigStore, packs

REPO = Path(__file__).resolve().parents[2]
DOCS = REPO / 'docs'
EXAMPLES = DOCS / 'examples'
PARSED = ('targets.md', 'migration_targets.md')
CHECKED = ('targets.md', 'configuration.md', 'missions.md', 'migration_targets.md')
BLOCK = re.compile(r'```yaml\n(.*?)\n```', re.S)
HEADER = re.compile(r'# example: (\S+)\n')

pytestmark = pytest.mark.skipif(not EXAMPLES.is_dir(), reason='docs/examples is not shipped with the package')


def blocks(names):
    return [(name, index, text) for name in names
            for index, text in enumerate(BLOCK.findall((DOCS / name).read_text(encoding='utf-8')))]


def ids(items):
    return [f'{name}#{index}' for name, index, _ in items]


@pytest.fixture
def cell(tmp_path, monkeypatch):
    monkeypatch.setattr(packs, 'installed_location', lambda: None)
    monkeypatch.setenv('TASKWEAVE_STATE_DIR', str(tmp_path / 'state'))
    root = tmp_path / 'cell'
    shutil.copytree(EXAMPLES / 'cell', root)
    shutil.copytree(EXAMPLES / 'missions', root / 'missions')
    shutil.copytree(EXAMPLES / 'skills', root / 'skills', dirs_exist_ok=True)
    return root


@pytest.mark.parametrize('name,index,text', blocks(PARSED), ids=ids(blocks(PARSED)))
def test_the_snippet_is_yaml(name, index, text):
    if text.startswith('cell pack schema') or text.startswith('rename'):
        pytest.skip('program output, not config')
    yaml.safe_load(text)


EXAMPLE_BLOCKS = [item for item in blocks(CHECKED) if HEADER.match(item[2])]


def test_the_docs_have_checked_examples():
    assert len({name for name, _, _ in EXAMPLE_BLOCKS}) >= 1 and len(EXAMPLE_BLOCKS) >= 8


@pytest.mark.parametrize('name,index,text', EXAMPLE_BLOCKS, ids=ids(EXAMPLE_BLOCKS))
def test_a_checked_example_is_valid_in_place(cell, name, index, text):
    path = HEADER.match(text).group(1)
    assert (cell / path).exists(), f'{name}: no such file in the example cell: {path}'
    data = yaml.safe_load(text)
    store = ConfigStore(cell)
    problems = [p for p in store.dry_run_pack(path, text) if p['path'] == path]
    assert problems == [], f'{name} block {index} ({path}): {problems}'
    assert isinstance(data, dict) and data
