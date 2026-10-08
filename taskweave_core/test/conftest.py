import shutil
from pathlib import Path

import pytest

FIXTURE_CONFIG = Path(__file__).parent / 'fixtures' / 'config'


@pytest.fixture
def config_dir(tmp_path) -> Path:
    """A writable copy of the generic test config."""
    target = tmp_path / 'config'
    shutil.copytree(FIXTURE_CONFIG, target)
    return target
