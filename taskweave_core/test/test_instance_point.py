"""instance_point: a point on one placed object (<instance>/<point>), as move_gripper needs it."""

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


def test_the_elbow_side_of_a_point_is_the_robots(config_dir):
    store = placed(config_dir)
    store.save_object_point_settings('box', 'top_grasp', {'elbow': 'up', 'approach_m': 0.05}, robot='demo_bot')
    store = ConfigStore(config_dir)
    assert 'elbow' not in (config_dir / 'objects' / 'box.yaml').read_text()        # the object stays robot-free
    assert store.object('box')['points']['top_grasp']['approach_m'] == 0.05
    assert store.object_point('box', 'top_grasp', 'demo_bot')['elbow'] == 'up'
    assert store.object_point('box', 'top_grasp')['elbow'] is None                   # without a robot: none
    assert store.instance_point('box_1/top_grasp', 'demo_bot')[1]['elbow'] == 'up'
    store.rename_object_point('box', 'top_grasp', 'lid_grasp')
    assert ConfigStore(config_dir).robot_object_points('demo_bot') == {'box': {'lid_grasp': {'elbow': 'up'}}}
    store.save_object_point_settings('box', 'lid_grasp', {'elbow': None}, robot='demo_bot')
    assert ConfigStore(config_dir).robot_object_points('demo_bot') == {}


def test_how_far_the_grippers_close_at_a_point_is_the_robots(config_dir):
    import pytest
    store = placed(config_dir)
    store.save_object_point_settings('box', 'top_grasp', {'elbow': 'up'}, robot='demo_bot')
    store.save_object_point_settings('box', 'top_grasp', {'grip_m': {'left': 0.012, 'right': 0.0125}}, robot='demo_bot')
    store.save_object_point_settings('box', 'top_grasp', {'grip_m': {'right': None}}, robot='demo_bot')  # merged
    store = ConfigStore(config_dir)
    assert store.robot_object_points('demo_bot') == {'box': {'top_grasp': {'elbow': 'up', 'grip_m': {'left': 0.012}}}}
    assert store.instance_point('box_1/top_grasp', 'demo_bot')[1]['grip_m'] == {'left': 0.012}
    assert 'grip_m' not in (config_dir / 'objects' / 'box.yaml').read_text()
    with pytest.raises(ValueError, match='robot setting'):
        store.save_object_point_settings('box', 'top_grasp', {'grip_m': {'left': 0.01}})
    store.save_object_point_settings('box', 'top_grasp', {'grip_m': {'left': None}, 'elbow': None}, robot='demo_bot')
    assert ConfigStore(config_dir).robot_object_points('demo_bot') == {}


def test_held_poses_are_the_robots(config_dir):
    import pytest
    (config_dir / 'robots' / 'demo_bot' / 'held_poses.yaml').write_text(
        'box:\n  carry:\n    joints: {lift: 0.5}\n    path:\n    - {xyz: [0.8, 0.0, 0.1], rpy: [0.0, 0.0, 0.0]}\n'
        '    - {xyz: [0.8, 0.0, 0.3], rpy: [0.0, -1.5708, 0.0]}\n')
    store = ConfigStore(config_dir)
    pose = store.held_pose('demo_bot', 'box', 'carry')
    assert pose['joints'] == {'lift': 0.5} and [p.xyz for p in pose['path']] == [(0.8, 0.0, 0.1), (0.8, 0.0, 0.3)]
    with pytest.raises(KeyError, match="no held pose 'stand_up'.*it has: carry"):
        store.held_pose('demo_bot', 'box', 'stand_up')


def test_object_state_takes_items_off_stacks_and_puts_objects_down(config_dir):
    import pytest
    (config_dir / 'cell' / 'objects.yaml').write_text(
        'pile: {object: box, frame: cell, x: 1.0, y: 2.0, z: 0.1, yaw: 0.0, stack: {count: 2, pitch: 0.05}}\n'
        'single: {object: box, frame: cell, x: 3.0, y: 0.0, yaw: 0.0}\n')
    store = ConfigStore(config_dir)
    assert store.take_object('pile') == 'took one off pile, 1 left'
    assert store.object_instances()['pile'].top_z == pytest.approx(0.1)        # the top item is the bottom one now
    assert store.take_object('pile') == 'took one off pile, 0 left'
    with pytest.raises(KeyError, match='pile is empty'):
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


def test_held_poses_and_via_targets_are_listed(config_dir):
    (config_dir / 'robots' / 'demo_bot' / 'poses.yaml').write_text('home: {j1: 0.0, j2: 0.0}\nready: {j1: 0.3}\n')
    (config_dir / 'robots' / 'demo_bot' / 'held_poses.yaml').write_text(
        'box:\n  carry:\n    joints: {lift: 0.5}\n    path: []\n'
        'cup:\n  stand_up:\n    joints: {lift: 0.2}\n    path: []\n  carry:\n    joints: {lift: 0.4}\n    path: []\n')
    names = placed(config_dir).point_names('demo_bot')
    assert names['held poses'] == ['carry', 'stand_up']                    # the union over object types, sorted
    assert names['joint poses'] == ['home', 'ready']
    assert names['via targets'] == ['pose:home', 'pose:ready'] + names['placed object points']
    assert 'box_1/top_grasp' in names['via targets']
