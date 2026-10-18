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


def wait_until(condition, timeout=30.0, interval=0.02, what='condition'):
    """Poll ``condition()`` (while a background executor spins the nodes) until it is truthy, and return its value.
    The deadline is generous on purpose: it only matters when something is really wrong, a loaded machine (several
    suites in parallel) just takes longer. Raises AssertionError naming ``what`` on timeout."""
    import time

    deadline = time.monotonic() + timeout
    while True:
        value = condition()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise AssertionError(f'timed out after {timeout:g} s waiting for {what}')
        time.sleep(interval)
