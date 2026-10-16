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
    """IK converges to wherever it starts (<side>_joint1 = the start's value, ``measured()`` from MoveIt's own current
    state, default 0); FK puts the elbow above the shoulder-wrist line for <side>_joint1 > 0 and below it for < 0 (right
    arm: mirrored in y)."""

    def __init__(self):
        super().__init__('fake_moveit')
        self.ik_requests = []
        self.measured = lambda: 0.0
        self.solution = None
        self.create_service(GetPositionIK, '/compute_ik', self.ik)
        self.create_service(GetPositionFK, '/compute_fk', self.fk)

    def ik(self, request, response):
        self.ik_requests.append(request.ik_request)
        start = dict(zip(request.ik_request.robot_state.joint_state.name,
                         request.ik_request.robot_state.joint_state.position, strict=True))
        side = 'right' if request.ik_request.group_name.startswith('right') else 'left'
        response.solution.joint_state.name = [f'{side}_joint1', f'{side}_joint2']
        response.solution.joint_state.position = [start.get(f'{side}_joint1', self.measured()), 0.0]
        if self.solution:  # a fixed answer, whatever the start
            response.solution.joint_state.name = list(self.solution)
            response.solution.joint_state.position = list(self.solution.values())
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
        'ik': ik, 'tool_frames': {'left': {'link': 'left_link7', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0], 'flip': flip}},
        'arms': {'left': {'elbow': {'shoulder': 'left_link2', 'elbow': 'left_link4', 'wrist': 'left_link6'}}}})


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
    assert ik.solve({'left': TARGET}, {})['left_joint1'] == pytest.approx(0.5)      # no current joints: the first
    fake, _ = moveit
    fake.measured = lambda: 2.0   # a descent from MoveIt's own state would end far away
    current = {'left_joint1': -0.4, 'left_joint2': 0.0}
    assert ik.solve({'left': TARGET}, {}, current=current)['left_joint1'] == pytest.approx(-0.4)  # from those joints

def test_identical_inputs_give_the_same_solution_whatever_moveit_measures(moveit):
    """The current state start is the joints the skill passes, not MoveIt's measured state (noisy while the arm
    settles): a local descent from a start a milliradian off can end in another configuration, or fail."""
    import random

    fake, node = moveit
    fake.measured = lambda: random.uniform(-1.0, 1.0)   # a descent from MoveIt's own state ends anywhere
    ik = IkClient(node, robot({}))
    current = {'left_joint1': -0.4, 'left_joint2': 0.1, 'other_joint': 3.0}
    solutions = [ik.solve({'left': TARGET}, {}, current=current) for _ in range(5)]
    assert all(s == solutions[0] for s in solutions) and solutions[0]['left_joint1'] == pytest.approx(-0.4)
    starts = [dict(zip(r.robot_state.joint_state.name, r.robot_state.joint_state.position, strict=True))
              for r in fake.ik_requests]
    assert starts[2] == {'left_joint1': -0.4, 'left_joint2': pytest.approx(0.1)}  # after the 2 seeds: the arm as given


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
    profile.data['arms']['right'] = {'elbow': {'shoulder': 'right_link2', 'elbow': 'right_link4',
                                               'wrist': 'right_link6'}}
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


def chain_robot(tmp_path, node, choose=None, **data):
    """IkClient for test_chain's 7-joint arm on a lift, with its URDF chain attached (``data``: more profile keys)."""
    from test_chain import NAMES, URDF

    path = tmp_path / 'arm.urdf'
    path.write_text(URDF)
    profile = SimpleNamespace(base={'frame': 'base'}, data={
        'ik': {'groups': {'arm': 'arm'}, 'joints': 'j[1-7]', 'timeout_s': 0.1, 'seeds': [{'j1': 0.0}],
               **({'choose': choose} if choose is not None else {})},
        'tool_frames': {'arm': {'link': 'flange', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}},
        'arms': {'arm': {'elbow': {'shoulder': 'l2', 'elbow': 'l4', 'wrist': 'l6'}}}, **data})
    ik = IkClient(node, profile, limits={name: (-2.5, 2.5) for name in NAMES})
    assert ik.attach_chains(path) == ['arm']
    return ik


def test_a_short_move_continues_the_arm_configuration_instead_of_a_seed_branch(moveit, tmp_path):
    """s1_s5-like: the target is 5 cm from the flange; a seed answers with the wrist turned over (another branch of the
    7-joint arm). The continuation (the flange walked there on the chain from the joints now) wins, whatever the
    seeds say, and the same start within 5 mrad of noise gives the same configuration."""
    import numpy as np
    from test_chain import NAMES, START

    from taskweave_skills.ik import tool_target

    fake, node = moveit
    ik = chain_robot(tmp_path, node)                          # no choose: the first seed would win
    chain = ik.chains['arm'][0]
    flange = chain.fk(START)
    flange[:3, 3] += [0.0, 0.03, -0.04]
    target = tool_target(flange, {})
    fake.solution = {**{n: START[n] for n in NAMES}, 'j5': START['j5'] + 3.0, 'j7': START['j7'] - 3.0}  # turned over
    solution = ik.solve({'arm': target}, {}, current=dict(START))
    assert not fake.ik_requests and ik.last['arm']['continued']                     # no IK search needed
    assert np.allclose(chain.fk({**START, **solution})[:3, 3], flange[:3, 3], atol=1e-3)
    assert max(abs(solution[n] - START[n]) for n in NAMES) < 0.3                   # continuous: no branch flip
    rng = np.random.default_rng(0)
    for _ in range(10):
        noisy = {n: v + rng.normal(0, 0.005) for n, v in START.items()}
        other = ik.solve({'arm': target}, {}, current=noisy)
        assert max(abs(other[n] - solution[n]) for n in NAMES) < 0.05              # the same configuration
    far = chain.fk(START)
    far[:3, 3] += [0.4, 0.0, 0.0]
    assert ik.solve({'arm': tool_target(far, {})}, {}, current=dict(START))['j5'] == \
        pytest.approx(fake.solution['j5'])                                         # too far to continue: IK decides
    assert 'continued' not in ik.last['arm']


def test_a_configuration_jump_without_a_continuation_is_logged(moveit, tmp_path):
    from test_chain import NAMES, START

    from taskweave_skills.ik import tool_target

    fake, node = moveit
    ik = chain_robot(tmp_path, node)
    flange = ik.chains['arm'][0].fk(START)
    flange[:3, 3] += [0.0, 0.0, -0.04]
    ik.limits = {name: (START[name] - 1e-3, START[name] + 1e-3) for name in NAMES}  # the arm cannot walk there
    fake.solution = {**{n: START[n] for n in NAMES}, 'j5': START['j5'] + 3.0}
    warnings = []
    ik.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=warnings.append))
    assert ik.solve({'arm': tool_target(flange, {})}, {}, current=dict(START))['j5'] == pytest.approx(START['j5'] + 3)
    assert len(warnings) == 1 and 'configuration jump, j5 turns 172 deg' in warnings[0]


def test_the_profile_gives_the_chains_their_elbow_and_tuning(moveit, tmp_path):
    from taskweave_core.config.tuning import motion_tuning

    _, node = moveit
    chain = chain_robot(tmp_path, node).chains['arm'][0]
    assert chain.elbow == ('l2', 'l4', 'l6') and chain.tuning == motion_tuning()['chain']
    ik = chain_robot(tmp_path, node, motion_tuning={'chain': {'max_step_rad': 0.02}, 'continuation': {'max_m': 0.1}})
    assert ik.chains['arm'][0].tuning['max_step_rad'] == 0.02 and ik.tuning['continuation']['max_m'] == 0.1
    ik = chain_robot(tmp_path, node, arms={'arm': {'elbow': {'shoulder': 'l2', 'elbow': 'l4', 'wrist': 'nowhere'}}})
    assert ik.chains['arm'][0].elbow is None                                       # not on the chain: no elbow control
    assert chain_robot(tmp_path, node, arms={}).chains['arm'][0].elbow is None


def test_without_an_elbow_description_the_elbow_side_is_ignored_with_a_warning(moveit):
    _, node = moveit
    profile = robot({'elbow': 'down'})
    del profile.data['arms']
    ik = IkClient(node, profile)
    warnings = []
    ik.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warning=warnings.append))
    assert ik.elbow_links('left') is None
    assert ik.solve({'left': TARGET}, {})['left_joint1'] == pytest.approx(0.5)    # no elbow cost: the first seed
    ik.solve({'left': TARGET}, {})
    assert len(warnings) == 1 and 'no arms.left.elbow' in warnings[0]               # said once
