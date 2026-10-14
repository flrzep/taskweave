"""instance_point: a point on one placed object (<instance>/<point>), as go_to_point needs it."""

from taskweave_core.config import ConfigStore
from taskweave_core.validation import check_point

BOX = 'box_1: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0}'


def placed(config_dir) -> ConfigStore:
    (config_dir / 'cell' / 'objects.yaml').write_text(BOX + '\n')
    return ConfigStore(config_dir)


def test_placed_object_points_are_listed_per_instance(config_dir):
    names = placed(config_dir).point_names('demo_bot')
    assert 'box_1/top_grasp' in names['placed object points'] and 'box.top_grasp' in names['object points']


def test_an_instance_point_must_name_a_placed_object(config_dir):
    store = placed(config_dir)
    assert check_point('instance_point', 'box_1/top_grasp', store, 'demo_bot') is None
    problem = check_point('instance_point', 'box.top_grasp', store, 'demo_bot')
    assert 'names no placed object' in problem and 'box_1/' in problem       # with an example to pick
    assert check_point('object_point', 'box.top_grasp', store, 'demo_bot') is None   # a type's point is fine there


def test_contacts_of_placed_objects(config_dir):
    box = config_dir / 'objects' / 'box.yaml'
    box.write_text(box.read_text() + 'contacts:\n  lid: {xyz: [0.0, 0.0, 0.3], rpy: [3.14159, 0.0, 0.0]}\n')
    store = placed(config_dir)
    assert 'box_1/lid' in store.point_names('demo_bot')['placed object contacts']
    instance, lid = store.instance_contact('box_1/lid')
    assert instance.name == 'box_1' and lid.xyz == (0.0, 0.0, 0.3)
    assert check_point('instance_contact', 'box_1/lid', store, 'demo_bot') is None
    assert 'no contact' in check_point('instance_contact', 'box_1/hinge', store, 'demo_bot')


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
