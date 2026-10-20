"""config/reload.py without ROS: what a change needs, per the catalogue."""

from taskweave_core.config import ConfigStore
from taskweave_core.config.reload import Reloader


def test_changes_are_sorted_by_the_catalogue_and_restarts_stick(config_dir):
    store = ConfigStore(config_dir)
    reloader = Reloader(store, 'demo_bot', {'live', 'mission_start'}, {'operator': 'read at launch'})
    assert reloader.reload() == ([], [])
    (config_dir / 'missions' / 'fresh.yaml').write_text('goal: g\nrobot: demo_bot\nsteps: []\n')
    (config_dir / 'operator.yaml').write_text('joy: {topic: /other}\n')
    assert reloader.reload() == (['missions/fresh.yaml'], [('operator.yaml', 'read at launch')])
    assert reloader.reload() == ([], [('operator.yaml', 'read at launch')])
    assert reloader.reload(['missions/fresh.yaml']) == ([], [])    # limited to the files named


def test_a_failing_apply_changes_nothing(config_dir):
    store = ConfigStore(config_dir)

    def apply(entries):
        raise ValueError('does not load')

    reloader = Reloader(store, 'demo_bot', {'mission_start'}, {}, apply=apply)
    (config_dir / 'missions' / 'fresh.yaml').write_text('goal: g\nrobot: demo_bot\nsteps: []\n')
    try:
        reloader.reload()
    except ValueError:
        pass
    else:
        raise AssertionError('apply did not raise')
    assert ('cell', 'missions/fresh.yaml') not in reloader.seen
