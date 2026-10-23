"""instance_point (a deprecated reader over target()): a point on one placed object (<instance>/<point>); the run-time
object state."""

from taskweave_core.config import ConfigStore
from taskweave_core.validation import check_point

BOX = 'box_1: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0}'


def placed(config_dir) -> ConfigStore:
    (config_dir / 'cell' / 'objects.yaml').write_text(BOX + '\n')
    return ConfigStore(config_dir)


def test_placed_object_points_are_listed_per_instance(config_dir):
    names = placed(config_dir).point_names('demo_bot')
    assert 'box_1/top_grasp' in names['placed object points'] and 'box.top_grasp' in names['object points']


def test_contacts_of_placed_objects(config_dir):
    box = config_dir / 'objects' / 'box.yaml'
    box.write_text(box.read_text() + '  lid: {moves: object, xyz: [0.0, 0.0, 0.3], rpy: [3.14159, 0.0, 0.0]}\n')
    store = placed(config_dir)
    assert 'box_1/lid' in store.point_names('demo_bot')['placed object contacts']
    instance, lid = store.instance_contact('box_1/lid')
    assert instance.name == 'box_1' and lid.xyz == (0.0, 0.0, 0.3)


def test_an_object_put_down_is_saved_as_a_placed_object(config_dir):
    objects = config_dir / 'cell' / 'objects.yaml'
    objects.write_text('# placed objects\nbox_1: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0, '
                       'stack: {count: 3, pitch: 0.1}}\n')
    store = ConfigStore(config_dir)
    store.save_object_instance('box_floor', 'box', 0.5, -0.25, 0.02, 3.1416)
    store.save_object_instance('box_1', 'box', 1.5, 2.0, 0.3, 0.0)   # moved: a single item now
    text = objects.read_text()
    assert text.startswith('# placed objects') and 'box_floor: {object: box, frame: cell, x: 0.5, y: -0.25' in text
    instances = ConfigStore(config_dir).object_instances()
    assert instances['box_floor'].pose.yaw == 3.1416 and instances['box_1'].pose.x == 1.5
    assert instances['box_1'].stack_count == 1 and instances['box_1'].top_z == 0.3


def test_object_state_takes_items_off_stacks_and_puts_objects_down(config_dir):
    import pytest
    (config_dir / 'cell' / 'objects.yaml').write_text(
        'pile: {object: box, frame: cell, x: 1.0, y: 2.0, z: 0.1, yaw: 0.0, stack: {count: 2, pitch: 0.05}}\n'
        'single: {object: box, frame: cell, x: 3.0, y: 0.0, yaw: 0.0}\n')
    store = ConfigStore(config_dir)
    assert store.take_object('pile') == 'took one off pile, 1 left'
    assert store.object_instances()['pile'].top_z == pytest.approx(0.1)        # the top item is the bottom one now
    assert store.take_object('pile') == 'took one off pile, 0 left'
    with pytest.raises(KeyError, match="'pile' was taken"):
        store.instance_point('pile/top_grasp')
    store.take_object('single')
    assert 'single' not in store.object_instances() and 'single' in store.object_instances(state=False)
    store.put_object('on_floor', 'box', 0.5, -0.2, 0.02, 3.1416)
    assert ConfigStore(config_dir).object_instances()['on_floor'].pose.yaw == 3.1416   # other processes see it
    assert 'on_floor' not in (config_dir / 'cell' / 'objects.yaml').read_text()        # config stays the start state
    store.take_object('on_floor')
    assert 'on_floor' not in store.object_instances()
    store.clear_object_state()
    assert store.object_instances()['pile'].stack_count == 2 and 'single' in store.object_instances()


def test_a_replenished_object_stays_when_taken(config_dir):
    (config_dir / 'objects' / 'hanger.yaml').write_text('name: hanger\nreplenished: true\n')
    (config_dir / 'cell' / 'objects.yaml').write_text(
        'waiting: {object: hanger, frame: cell, x: 1.0, y: 0.0, yaw: 0.0}\n')
    store = ConfigStore(config_dir)
    assert store.take_object('waiting') == 'waiting stays (the next hanger takes its place)'
    assert 'waiting' in store.object_instances()


TAKEN = 'was taken in an earlier step or run (object state: state/objects.yaml)'


def test_a_taken_object_says_so_not_unknown(config_dir):
    import re

    import pytest
    (config_dir / 'cell' / 'objects.yaml').write_text(
        'pile: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0, stack: {count: 1, pitch: 0.05}}\n'
        'single: {object: box, frame: cell, x: 3.0, y: 0.0, yaw: 0.0}\n')
    store = ConfigStore(config_dir)
    store.take_object('single')
    store.take_object('pile')
    for call, name in ((lambda: store.instance_point('single/top_grasp'), 'single'),
                       (lambda: store.instance_point('pile/top_grasp'), 'pile'),
                       (lambda: store.instance_contact('single/lid'), 'single')):
        with pytest.raises(KeyError, match=f"'{name}' {re.escape(TAKEN)}; reset the sim .* `taskweave state clear`"):
            call()
    assert TAKEN in check_point('target', 'single/top_grasp', store, 'demo_bot')
    assert TAKEN in check_point('held_object', 'single', store, 'demo_bot')


def test_a_truly_unknown_instance_keeps_the_old_message(config_dir):
    import pytest
    store = placed(config_dir)
    with pytest.raises(KeyError, match='unknown object point .nope/top_grasp.: expected <instance>/<point>'):
        store.instance_point('nope/top_grasp')
    assert 'unknown placed object' in check_point('held_object', 'nope', store, 'demo_bot')
