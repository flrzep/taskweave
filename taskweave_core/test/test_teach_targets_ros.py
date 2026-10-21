"""Teaching on a cell of schema 4 (targets): the teach node writes the right file for each kind, through save_target.

The node is called directly (``_teach``) with its odometry, joint states and TF fed by hand, so no other node is needed
(the pad's button path is in test_teach_ros.py)."""

import math
import shutil
from pathlib import Path

import pytest
import yaml
from conftest import RosRig

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')

from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from taskweave_core.config import ConfigStore  # noqa: E402
from taskweave_interfaces.srv import TeachPoint  # noqa: E402

FIXTURES = Path(__file__).parent / 'fixtures'
ROBOT = 'bot_two_arm'
# TCP 0.1 m along the flange's x axis; the flange points straight down (rotated pi about y), so it lies 0.1 m back
TOOL_FRAMES = ("tool_frames:\n  left: {link: left_link7, xyz: [0.1, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}\n"
               "  right: {link: right_link7, xyz: [0.1, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}\n")
BASE_AT = (1.0, 0.5)    # base_link in the cell (world) frame, no turn
FLANGE = {'left_link7': (0.3, 0.2, 0.9), 'right_link7': (0.3, -0.2, 0.9)}    # in base_link


@pytest.fixture
def cell(tmp_path) -> Path:
    root = tmp_path / 'cell'
    shutil.copytree(FIXTURES / 'targets_new', root)
    profile = root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text() + TOOL_FRAMES)
    return root


@pytest.fixture
def bare(cell):
    from taskweave_core.teach import Teach

    parameters = [Parameter('config_dir', value=str(cell)), Parameter('robot', value=ROBOT),
                  Parameter('max_tf_age_s', value=5.0)]
    with RosRig() as ros:
        yield ros.add(Teach(parameter_overrides=parameters))


@pytest.fixture
def node(bare):
    feed_tf(bare)    # static transforms: always fresh
    return bare


def transform(node, parent: str, child: str, xyz, rotated: bool = False, stamp=None) -> None:
    """A transform of the test robot; ``stamp`` (a Time message) makes it a dynamic one, else it is static."""
    t = TransformStamped(child_frame_id=child)
    t.header.frame_id = parent
    if stamp is not None:
        t.header.stamp = stamp
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    if rotated:
        t.transform.rotation.y, t.transform.rotation.w = 1.0, 0.0   # pi about y
    else:
        t.transform.rotation.w = 1.0
    (node.tf.set_transform if stamp is not None else node.tf.set_transform_static)(t, 'test')


def feed_tf(node, age_s=None) -> None:
    """The test robot's TF: static, or dynamic ``age_s`` seconds old (all with one stamp)."""
    stamp = None if age_s is None else (node.get_clock().now() - rclpy.duration.Duration(seconds=age_s)).to_msg()
    transform(node, 'world', 'base_link', (*BASE_AT, 0.0), stamp=stamp)
    for link, xyz in FLANGE.items():
        transform(node, 'base_link', link, xyz, rotated=True, stamp=stamp)


def at(node, x: float, y: float, yaw: float) -> None:
    odom = Odometry()
    odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    node._odom(odom)


def joints(node, values: dict) -> None:
    node._joint_state(JointState(name=list(values), position=[float(v) for v in values.values()]))


def teach(node, name: str, kind: str, tools=(), override: bool = False):
    request = TeachPoint.Request(name=name, kind=kind, tools=list(tools))
    request.instance_override = override
    return node._teach(request, TeachPoint.Response())


def load(cell: Path, relative: str) -> dict:
    return yaml.safe_load((cell / relative).read_text()) or {}


# ---- base ----------------------------------------------------------------------------------------------------------
def test_base_of_a_name_goes_to_the_cells_targets(node, cell):
    at(node, 4.25, 3.0, 0.5)
    answer = teach(node, 'bay_dock', 'base')
    assert answer.success, answer.message
    assert answer.file.endswith('cell/targets.yaml') or answer.file.endswith('cell\\targets.yaml')
    assert load(cell, 'cell/targets.yaml')['bay_dock'] == {'moves': 'base', 'x': 4.25, 'y': 3.0, 'yaw': 0.5}
    assert 'taught' in (cell / 'cell' / 'targets.yaml').read_text()
    assert load(cell, 'cell/targets.yaml')['waiting']['x'] == 2.0    # the neighbours stay


def test_base_of_an_existing_name_moves_it_and_keeps_the_other_keys(node, cell):
    at(node, 2.5, 0.25, 0.1)
    assert teach(node, 'waiting', 'base').success
    entry = load(cell, 'cell/targets.yaml')['waiting']
    assert entry == {'moves': 'base', 'x': 2.5, 'y': 0.25, 'yaw': 0.1}
    assert '# centre of the waiting zone' in (cell / 'cell' / 'targets.yaml').read_text()


def test_base_of_an_instance_name_is_a_stance_of_the_type(node, cell):
    at(node, 7.0, 1.0, 0.0)    # in front of the bay: the stance is relative to the frame
    answer = teach(node, 'bay_2_stack_2_front/pick', 'base')
    assert answer.success, answer.message
    assert 'robots/bot_two_arm/objects/frame_type2.yaml' in answer.file.replace('\\', '/')
    store = ConfigStore(cell)
    again = store.target('bay_2_stack_2_front/pick', ROBOT, near=node.pose)
    assert (again.pose.x, again.pose.y) == pytest.approx((7.0, 1.0), abs=1e-3)
    table = load(cell, f'robots/{ROBOT}/objects/frame_type2.yaml')
    assert table['targets']['pick']['moves'] == 'base' and table['handled_from'] == 'pick'
    assert 'rails_top' in table['targets']                  # the robot's other entries stay
    assert 'bay_1_stack_1_back' in table['instances']       # and so do the instances


def test_base_of_an_instance_with_the_override_flag_writes_that_instance_only(node, cell):
    at(node, 7.1, 1.0, 0.0)
    answer = teach(node, 'bay_1_stack_1_front/pick', 'base', override=True)
    assert answer.success, answer.message
    table = load(cell, f'robots/{ROBOT}/objects/frame_type2.yaml')
    assert 'bay_1_stack_1_front' in table['instances']
    assert table['targets']['pick'] == {'moves': 'base', 'x': -1.0, 'y': 0.0, 'yaw': 0.0}    # the type's: unchanged


def test_base_refuses_an_unknown_object_and_a_hand_target_name(node, cell):
    at(node, 1.0, 1.0, 0.0)
    answer = teach(node, 'nowhere/pick', 'base')
    assert not answer.success and 'nowhere' in answer.message
    answer = teach(node, 'frame_grasp_approach', 'base')
    assert not answer.success and 'moves a hand' in answer.message


def test_base_needs_a_pose(node):
    answer = teach(node, 'somewhere', 'base')
    assert not answer.success and 'no current pose' in answer.message


# ---- hand ----------------------------------------------------------------------------------------------------------
def test_hand_of_an_instance_name_is_stored_relative_to_the_object_in_the_types_file(node, cell):
    at(node, 1.0, 0.5, 0.0)
    answer = teach(node, 'part_box_1/top_grasp', 'hand', tools=['left'])
    assert answer.success, answer.message
    assert answer.file.replace('\\', '/').endswith('objects/part_box.yaml')
    point = load(cell, 'objects/part_box.yaml')['targets']['top_grasp']
    # TCP in the cell: base (1.0, 0.5) + flange (0.3, 0.2, 0.9) + 0.1 along flange x (= -x): (1.2, 0.7, 0.9);
    # the box is at (3, -1) turned 0.5 rad, on the floor
    assert point['left_tool']['xyz'] == pytest.approx((-0.7646, 2.3549, 0.9), abs=1e-3)
    assert point['alone']['right_tool']['xyz'] == [0.0, 0.0, 0.07]     # the one-hand variants stay
    assert point['tool_type'] == 'parallel_gripper' and point['moves'] == 'hand'
    assert 'right_tool' not in point                                   # only the tool asked for


def test_hand_of_an_instance_name_without_tools_stores_every_tool(node, cell):
    at(node, 1.0, 0.5, 0.0)
    assert teach(node, 'part_box_1/side_grasp', 'hand').success
    point = load(cell, 'objects/part_box.yaml')['targets']['side_grasp']
    assert point['moves'] == 'hand' and set(point) >= {'left_tool', 'right_tool'}


def test_hand_of_a_name_is_in_the_base_frame_in_the_robots_targets(node, cell):
    answer = teach(node, 'reach_out', 'hand')
    assert answer.success, answer.message
    assert answer.file.replace('\\', '/').endswith(f'robots/{ROBOT}/targets.yaml') or 'targets.yaml' in answer.file
    entry = ConfigStore(cell).target('reach_out', ROBOT)
    assert entry.kind == 'hand' and entry.frame == 'base'
    assert entry.tool_poses['left'].xyz == pytest.approx((0.2, 0.2, 0.9), abs=1e-4)
    assert entry.tool_poses['right'].xyz == pytest.approx((0.2, -0.2, 0.9), abs=1e-4)
    from taskweave_core.config.poses import quaternion_from_rpy

    q = quaternion_from_rpy(entry.tool_poses['left'].rpy)    # the flange is turned pi about y
    assert abs(q[1]) == pytest.approx(1.0, abs=1e-4)


def test_hand_of_an_existing_base_target_keeps_its_other_keys(node, cell):
    answer = teach(node, 'frame_grasp_approach', 'hand', tools=['left'])
    assert answer.success, answer.message
    found = ConfigStore(cell).target('frame_grasp_approach', ROBOT)
    assert found.tool_poses['left'].xyz == pytest.approx((0.2, 0.2, 0.9), abs=1e-4)
    assert found.tool_poses['right'].xyz == pytest.approx((0.48, 0.295, 0.6))     # not taught: as it was
    assert found.data['elbow'] == 'up' and found.joints['torso'] == 0.5


def test_hand_refuses_a_name_that_moves_something_else(node):
    answer = teach(node, 'home', 'hand')
    assert not answer.success and 'moves joints' in answer.message


def test_hand_refuses_stale_tf(bare):
    feed_tf(bare, age_s=30.0)
    answer = teach(bare, 'reach_out', 'hand', tools=['left'])
    assert not answer.success and 'stale' in answer.message
    feed_tf(bare)
    assert teach(bare, 'reach_out', 'hand', tools=['left']).success    # fresh data is taught again


# ---- joints --------------------------------------------------------------------------------------------------------
ARM = [f'left_arm_joint{i}' for i in range(1, 8)]
POSITIONS = {'body_joint': 0.4, 'head_joint1': 0.1, 'head_joint2': -0.2,
             **{j: 0.1 * i for i, j in enumerate(ARM, 1)}, **{f'right_arm_joint{i}': 0.0 for i in range(1, 8)}}


def test_joints_of_a_new_name_takes_every_group_from_the_joint_states(node, cell):
    joints(node, POSITIONS)
    answer = teach(node, 'look_around', 'joints')
    assert answer.success, answer.message
    assert answer.file.replace('\\', '/').endswith(f'robots/{ROBOT}/targets.yaml')
    entry = load(cell, f'robots/{ROBOT}/targets.yaml')['look_around']
    assert entry['moves'] == 'joints'
    assert entry['joints']['torso'] == 0.4 and entry['joints']['head'] == [0.1, -0.2]
    assert entry['joints']['left_arm'] == pytest.approx([0.1 * i for i in range(1, 8)])
    assert 'lift_high' in load(cell, f'robots/{ROBOT}/targets.yaml')     # the neighbours stay


def test_joints_of_an_existing_name_keeps_its_groups_and_updates_their_values(node, cell):
    joints(node, POSITIONS)
    answer = teach(node, 'home', 'joints')
    assert answer.success, answer.message
    entry = load(cell, f'robots/{ROBOT}/targets.yaml')['home']
    assert set(entry['joints']) == {'head', 'left_arm', 'right_arm'}      # no torso: home had none
    assert entry['joints']['head'] == [0.1, -0.2]


def test_joints_of_named_groups_only(node, cell):
    joints(node, POSITIONS)
    assert teach(node, 'home', 'joints', tools=['torso']).success
    entry = load(cell, f'robots/{ROBOT}/targets.yaml')['home']['joints']
    assert entry['torso'] == 0.4 and entry['head'] == [0.0, 0.0]          # head as it was


def test_joints_refuse_without_states_or_with_an_unknown_group_or_an_instance_name(node):
    assert 'no joint states' in teach(node, 'x', 'joints').message
    joints(node, POSITIONS)
    assert 'no joint group tail' in teach(node, 'x', 'joints', tools=['tail']).message
    assert 'plain name' in teach(node, 'part_box_1/x', 'joints').message
    assert 'moves a hand' in teach(node, 'frame_grasp_approach', 'joints').message


# ---- kinds ---------------------------------------------------------------------------------------------------------
def test_the_old_kinds_are_taken_for_the_new_ones_on_a_targets_cell(node, cell):
    at(node, 4.0, 2.0, 0.0)
    assert teach(node, 'old_spot', 'location').success
    assert load(cell, 'cell/targets.yaml')['old_spot']['moves'] == 'base'
    assert teach(node, 'part_box_1/pick', 'stance_instance').success
    assert 'part_box_1' in load(cell, f'robots/{ROBOT}/objects/part_box.yaml')['instances']
    assert teach(node, 'part_box_1/side', 'object_point', tools=['left']).success
    assert load(cell, 'objects/part_box.yaml')['targets']['side']['moves'] == 'hand'


def test_a_held_object_is_not_built_and_unknown_kinds_are_refused(node):
    answer = teach(node, 'carry2', 'object')
    assert not answer.success and 'cannot be taught yet' in answer.message
    answer = teach(node, 'carry2', 'tcp')
    assert not answer.success and 'unknown kind' in answer.message
    assert not teach(node, 'part_box_1/x', 'base', override=False).success    # no pose yet


def test_the_override_flag_is_for_instance_references(node):
    at(node, 1.0, 1.0, 0.0)
    answer = teach(node, 'plain', 'base', override=True)
    assert not answer.success and 'instance_override' in answer.message


# ---- the pad ---------------------------------------------------------------------------------------------------------
OPERATOR = """joy:
  topic: /joy
  drive_button: 8
  arms:
    left: {button: 13, twist: /taskweave/jog/left}
  teach: {button: 10, point: part_box_1/top_grasp}
"""


def test_the_pad_teaches_a_hand_target_on_a_targets_cell(cell):
    import time

    from conftest import wait_until
    from sensor_msgs.msg import Joy

    from taskweave_core.operator_joy import OperatorJoy
    from taskweave_core.teach import Teach

    (cell / 'operator.yaml').write_text(OPERATOR)
    parameters = [Parameter('config_dir', value=str(cell)), Parameter('robot', value=ROBOT),
                  Parameter('max_tf_age_s', value=30.0)]
    with RosRig() as ros:
        operator = ros.add(OperatorJoy(parameter_overrides=parameters))
        teach_node = ros.add(Teach(parameter_overrides=parameters))
        pad = ros.add(rclpy.create_node('pad_test'))
        ros.spin(4)
        feed_tf(teach_node)    # static: always fresh
        joy = pad.create_publisher(Joy, '/joy', 10)
        wait_until(lambda: joy.get_subscription_count() > 0, what='the operator node to subscribe to /joy')
        wait_until(lambda: operator.teach_client.service_is_ready(), what='the teach service')

        def send(buttons=()):
            joy.publish(Joy(axes=[0.0] * 6, buttons=[1 if i in buttons else 0 for i in range(15)]))
            time.sleep(0.15)

        def left_mode():
            send(buttons=(13,))
            send()
            return operator.mode == 'left'

        wait_until(left_mode, what="the pad's left arm mode", interval=0.0)

        def taught():
            send(buttons=(4,))
            send(buttons=(4, 10))
            send()
            store = ConfigStore(cell)
            return 'left_tool' in store.type_target('part_box', 'top_grasp', ROBOT).data

        wait_until(taught, what='the taught hand target in objects/part_box.yaml', interval=0.0)
        point = load(cell, 'objects/part_box.yaml')['targets']['top_grasp']
        assert point['left_tool']['xyz'] == pytest.approx((-0.7646, 2.3549, 0.9), abs=1e-3)
        assert 'right_tool' not in point and point['moves'] == 'hand'
