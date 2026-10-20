"""taskweave state show / clear: the run-time object state (state/objects.yaml) from the command line."""

from taskweave_core import cli
from taskweave_core.config import ConfigStore

CELL = ('pile: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0, stack: {count: 3, pitch: 0.05}}\n'
        'single: {object: box, frame: cell, x: 3.0, y: 0.0, yaw: 0.0}\n')


def taken(config_dir) -> ConfigStore:
    (config_dir / 'cell' / 'objects.yaml').write_text(CELL)
    store = ConfigStore(config_dir)
    store.take_object('single')
    store.take_object('pile')
    store.put_object('floor_box', 'box', 0.5, -0.2, 0.02, 0.0)
    return store


def test_show_lists_what_is_taken_counts_and_placed(config_dir, capsys):
    taken(config_dir)
    assert cli.main(['state', 'show', str(config_dir)]) == 0
    out = capsys.readouterr().out
    assert 'single: taken' in out and 'pile: 2 left' in out and 'floor_box: placed' in out and 'objects.yaml' in out


def test_show_without_state(config_dir, capsys):
    assert cli.main(['state', 'show', str(config_dir)]) == 0
    assert 'no object state' in capsys.readouterr().out


def test_clear_puts_everything_back(config_dir, capsys):
    store = taken(config_dir)
    assert cli.main(['state', 'clear', str(config_dir)]) == 0
    assert not store.state_path().exists() and 'cleared' in capsys.readouterr().out
    assert 'single' in store.object_instances() and store.object_instances()['pile'].stack_count == 3
    assert cli.main(['state', 'clear', str(config_dir)]) == 0          # nothing to clear is not an error
