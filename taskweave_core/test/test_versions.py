"""Every package of the repo carries one version: package.xml and setup.py agree (and the bundled robot packs)."""

import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


def package_dirs():
    return sorted(p.parent for p in ROOT.glob('*/package.xml'))


def test_one_version_everywhere():
    dirs = package_dirs()
    if len(dirs) < 6:
        pytest.skip('not a full checkout')
    found = {}
    for d in dirs:
        xml = (d / 'package.xml').read_text()
        found[f'{d.name}/package.xml'] = re.search(r'<version>([^<]+)</version>', xml).group(1)
        setup = d / 'setup.py'
        if setup.exists():
            found[f'{d.name}/setup.py'] = re.search(r"version='([^']+)'", setup.read_text()).group(1)
    for pack in sorted(ROOT.glob('taskweave_robots/*/pack.yaml')):
        found[f'robots/{pack.parent.name}/pack.yaml'] = str(yaml.safe_load(pack.read_text())['version'])
    assert len(set(found.values())) == 1, found
