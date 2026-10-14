"""IkClient with ``choose``: the best of several IK solutions wins, not the first (fake /compute_ik, /compute_fk)."""

import threading
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from geometry_msgs.msg import PoseStamped  # noqa: E402
from moveit_msgs.srv import GetPositionFK, GetPositionIK  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

from taskweave_skills.ik import IkClient  # noqa: E402

TARGET = {'tcp': [0.8, 0.3, 0.1], 'approach': [0, 0, -1], 'closing': [0, 1, 0]}


class FakeMoveIt(Node):
    """IK converges to wherever it starts (<side>_joint1 = the start's value, 0 from the current state); FK puts the
    elbow above the shoulder-wrist line for <side>_joint1 > 0 and below it for < 0 (right arm: mirrored in y)."""

    def __init__(self):
        super().__init__('fake_moveit')
        self.ik_requests = []
        self.create_service(GetPositionIK, '/compute_ik', self.ik)
        self.create_service(GetPositionFK, '/compute_fk', self.fk)

    def ik(self, request, response):
        self.ik_requests.append(request.ik_request)
        start = dict(zip(request.ik_request.robot_state.joint_state.name,
                         request.ik_request.robot_state.joint_state.position, strict=True))
        side = 'right' if request.ik_request.group_name.startswith('right') else 'left'
        response.solution.joint_state.name = [f'{side}_joint1', f'{side}_joint2']
        response.solution.joint_state.position = [start.get(f'{side}_joint1', 0.0), 0.0]
        response.error_code.val = 1
        return response

    def fk(self, request, response):
        joints = dict(zip(request.robot_state.joint_state.name, request.robot_state.joint_state.position, strict=True))
        for link in request.fk_link_names:
            side = link.split('_')[0]
            y = 0.3 if side == 'left' else -0.3
            points = {'link2': (0.0, y, 0.5), 'link4': (0.4, y, 0.31 + joints.get(f'{side}_joint1', 0.0)),
                      'link6': (0.8, y, 0.1)}
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = points[link.split('_')[1]]
            response.pose_stamped.append(pose)
        response.error_code.val = 1
        return response


def robot(choose=None, flip=False):
    ik = {'groups': {'left': 'left_arm'}, 'joints': '{tool}_joint[1-7]', 'timeout_s': 0.1,
          'seeds': [{'left_joint1': 0.5}, {'left_joint1': -0.5}]}
    if choose is not None:
        ik['choose'] = choose
    return SimpleNamespace(base={'frame': 'base_link'}, data={
        'ik': ik, 'tool_frames': {'left': {'link': 'left_link7', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'flip': flip}}})


@pytest.fixture
def moveit():
    rclpy.init()
    fake, client = FakeMoveIt(), rclpy.create_node('ik_client_test')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fake)
    executor.add_node(client)
    threading.Thread(target=executor.spin, daemon=True).start()
    yield fake, client
    executor.shutdown()
    fake.destroy_node()
    client.destroy_node()
    rclpy.shutdown()


def test_without_choose_the_first_seed_wins(moveit):
    fake, node = moveit
    assert IkClient(node, robot()).solve({'left': TARGET}, {})['left_joint1'] == pytest.approx(0.5)
    assert len(fake.ik_requests) == 1


def test_the_preferred_elbow_side_wins_over_the_first_solution(moveit):
    _, node = moveit
    ik = IkClient(node, robot({'elbow': 'down'}))
    assert ik.solve({'left': TARGET}, {})['left_joint1'] == pytest.approx(-0.5)  # elbow below the line
    assert ik.last['left']['elbow_deg'] < 10 and ik.last['left']['candidates'] == 3


def test_room_to_the_joint_limits_can_outweigh_the_elbow(moveit):
    _, node = moveit
    limits = {'left_joint1': (-0.55, 1.0)}  # the elbow-down solution is 3 degrees from a limit
    ik = IkClient(node, robot({'elbow': 'down', 'weights': {'limits': 2.0}}), limits=limits)
    assert ik.solve({'left': TARGET}, {})['left_joint1'] != pytest.approx(-0.5)


def test_the_size_of_the_move_breaks_ties(moveit):
    _, node = moveit
    ik = IkClient(node, robot({}))  # no elbow preference: only limits (none) and the move from the current joints
    assert ik.solve({'left': TARGET}, {}, current={'left_joint1': -0.4, 'left_joint2': 0.0})['left_joint1'] == \
        pytest.approx(-0.5)


def test_a_flippable_grasp_is_also_solved_turned_about_the_approach(moveit):
    fake, node = moveit
    IkClient(node, robot({'elbow': 'down'}, flip=True)).solve({'left': TARGET}, {})
    orientations = {(round(r.pose_stamped.pose.orientation.x, 3), round(r.pose_stamped.pose.orientation.y, 3))
                    for r in fake.ik_requests}
    assert len(fake.ik_requests) == 6 and len(orientations) == 2  # 3 starts x (target, flipped target)


def test_a_mirroring_arm_starts_from_and_prefers_the_mirror_image(moveit):
    fake, node = moveit
    profile = robot({'elbow': 'down'})
    profile.data['ik']['mirror'] = {'right': 'left'}
    profile.data['ik']['groups']['right'] = 'right_arm'
    profile.data['tool_frames']['right'] = {'link': 'right_link7', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    ik = IkClient(node, profile)
    right = {**TARGET, 'tcp': [0.8, -0.3, 0.1]}
    solution = ik.solve({'right': right, 'left': TARGET}, {})  # right listed first: still solved after left
    assert solution['left_joint1'] == pytest.approx(-0.5)
    starts = [dict(zip(r.robot_state.joint_state.name, r.robot_state.joint_state.position, strict=True))
              for r in fake.ik_requests if r.group_name == 'right_arm']
    assert starts[0]['right_joint1'] == pytest.approx(-0.5)  # the first right start is the mirror image
    assert solution['right_joint1'] == pytest.approx(-0.5)


def test_a_point_can_ask_for_another_elbow_side_than_the_profile(moveit):
    _, node = moveit
    ik = IkClient(node, robot({'elbow': 'down'}))
    assert ik.solve({'left': TARGET}, {}, elbow='up')['left_joint1'] == pytest.approx(0.5)   # elbow above the line
    assert ik.solve({'left': TARGET}, {})['left_joint1'] == pytest.approx(-0.5)               # the profile again


def test_the_choice_can_change_after_start(moveit):
    _, node = moveit
    ik = IkClient(node, robot())  # no choose: the first seed wins
    ik.set_choice({'elbow': 'down'})
    assert ik.solve({'left': TARGET}, {})['left_joint1'] == pytest.approx(-0.5)
