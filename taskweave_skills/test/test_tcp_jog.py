"""Flange target integration of the TCP jog (no ROS needed)."""

import math

import numpy as np
import pytest

from taskweave_skills.tcp_jog import integrate, rotation, swivel_joint


def test_linear_velocity_moves_the_flange_in_the_base_frame():
    pose = np.eye(4)
    pose[:3, :3] = rotation(np.array([0, 0, 1.0]), math.pi / 2)  # flange turned: does not matter
    moved = integrate(pose, (0.1, 0.0, -0.05), (0, 0, 0), 0.5)
    assert moved[:3, 3] == pytest.approx([0.05, 0.0, -0.025]) and moved[:3, :3] == pytest.approx(pose[:3, :3])


def test_angular_velocity_turns_about_base_axes_in_place():
    moved = integrate(np.eye(4), (0, 0, 0), (0.0, 0.0, 1.0), math.pi / 2)
    assert moved[:3, 0] == pytest.approx([0, 1, 0], abs=1e-12)  # x turned onto y about z
    assert moved[:3, 3] == pytest.approx([0, 0, 0])


def test_no_rotation_for_zero_rate():
    assert rotation(np.zeros(3), 1.0) == pytest.approx(np.eye(3))


class FakeIk:
    """ik.step: records the start it got; succeeds unless told to fail."""

    def __init__(self, ok=True):
        self.ok, self.starts = ok, []

    def step(self, tool, flange, start):
        self.starts.append(dict(start))
        return dict(start) if self.ok else None


def swing_node(ik):
    from types import SimpleNamespace

    from taskweave_skills.tcp_jog import TcpJog

    node = TcpJog.__new__(TcpJog)  # only _swing: no ROS node
    node.ik, node.held_tools, node.sent = ik, set(), []
    node.command = SimpleNamespace(publish=lambda msg: node.sent.append(dict(zip(msg.name, msg.position, strict=True))))
    node.JointState = lambda name, position: SimpleNamespace(name=name, position=position)
    node.warnings = []
    node.get_logger = lambda: SimpleNamespace(warning=lambda text, **k: node.warnings.append(text))
    return node


def test_the_elbow_swing_turns_the_swivel_joint_and_keeps_the_flange_target():
    ik = FakeIk()
    node = swing_node(ik)
    current = {f'left_joint{i}': 0.1 * i for i in range(1, 8)}
    jog = {'swivel': 'left_joint3', 'target': np.eye(4), 'blocked': False}
    node._swing('left', jog, current, 0.05)
    assert ik.starts[0]['left_joint3'] == pytest.approx(0.35)     # the profile's swivel joint
    assert ik.starts[0]['left_joint2'] == pytest.approx(0.2) and node.sent
    assert np.array_equal(jog['target'], np.eye(4))               # the TCP stays where it was


def test_without_a_swivel_joint_the_ik_swing_says_what_is_missing():
    ik = FakeIk()
    node = swing_node(ik)
    jog = {'swivel': None, 'target': np.eye(4), 'blocked': False}
    node._swing('left', jog, {f'left_joint{i}': 0.0 for i in range(1, 8)}, 0.05)
    assert not ik.starts and not node.sent                         # no arm joint is guessed
    assert 'arms.left.swivel' in node.warnings[0]


def test_the_swivel_joint_comes_from_the_arm_description():
    assert swivel_joint({'arms': {'left': {'swivel': 'l3'}}}, 'left') == 'l3'
    assert swivel_joint({'ik': {'swivel_joint': '{tool}_j3'}}, 'right') == 'right_j3'   # deprecated key
    assert swivel_joint({'arms': {'left': {'elbow': {}}}, 'ik': {'groups': {}}}, 'left') is None


def test_a_blocked_elbow_swing_sends_nothing():
    node = swing_node(FakeIk(ok=False))
    jog = {'swivel': 'left_joint5', 'target': np.eye(4), 'blocked': False}
    node._swing('left', jog, {f'left_joint{i}': 0.0 for i in range(1, 8)}, 0.05)
    assert jog['blocked'] and not node.sent


# ---- object jog -----------------------------------------------------------------------------------------------

class FakeChain:
    """A chain whose joints are the tip position (<hand>_x, _y, _z): a step reaches the target unless ``stuck``."""

    def __init__(self, hand, stuck=False):
        self.hand, self.stuck, self.rotation = hand, stuck, np.eye(3)

    def solve_step(self, positions, target, names, limits, null=None):
        if self.stuck:
            return {n: positions[n] for n in names}, ''
        self.rotation = target[:3, :3]
        return dict(zip(names, (float(v) for v in target[:3, 3]), strict=True)), ''

    def fk(self, positions):
        pose = np.eye(4)
        pose[:3, :3] = np.eye(3) if self.stuck else self.rotation
        pose[:3, 3] = [positions[f'{self.hand}_{a}'] for a in 'xyz']
        return pose


def object_node(hands=('left', 'right'), stuck=(), valid=True):
    import re
    import threading
    from types import SimpleNamespace

    from taskweave_skills.tcp_jog import TcpJog

    node = TcpJog.__new__(TcpJog)
    node.sent, node.notices, node.checks = [], [], []
    node.String = lambda data: SimpleNamespace(data=data)
    node.JointState = lambda name, position: SimpleNamespace(name=name, position=position)
    node.command = SimpleNamespace(publish=lambda m: node.sent.append(dict(zip(m.name, m.position, strict=True))))
    node.notice = SimpleNamespace(publish=lambda m: node.notices.append(m.data))
    node.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None, info=lambda *a, **k: None)
    node.lock, node.limits, node.objects, node.shape_cache = threading.Lock(), {}, {}, {None: []}
    node.ik = SimpleNamespace(tools={h: {'link': f'{h}_link7', 'xyz': [0, 0, 0.1]} for h in hands},
                              groups={h: f'{h}_arm' for h in hands})
    node.tools = {h: {'chain': FakeChain(h, h in stuck), 'names': [f'{h}_{a}' for a in 'xyz'], 'held': '',
                      'joints': re.compile(f'{h}_.*'), 'blocked': False, 'target': None, 'twist': None,
                      'received': -1e9, 'elbow': 0.0, 'elbow_received': -1e9, 'swivel': None} for h in hands}
    # the flange of each hand: x 0.5, y +-0.2, the TCP 0.1 ahead on z
    node.flanges = {h: np.array([[1, 0, 0, 0.5], [0, 1, 0, 0.2 if h == 'left' else -0.2], [0, 0, 1, 0.0],
                                 [0, 0, 0, 1]]) for h in hands}
    node.positions = {f'{h}_{a}': float(node.flanges[h][i, 3]) for h in hands for i, a in enumerate('xyz')}
    node._flange = lambda tool: node.flanges[tool]
    node._valid = lambda group, joints, attached=(): node.checks.append((group, dict(joints))) or valid
    node._held_notice = lambda *a: None
    obj = np.eye(4)
    obj[:3, 3] = [0.5, 0.0, 0.1]      # between the hands, the TCPs 0.1 ahead of the flanges
    tcp = np.eye(4)
    tcp[2, 3] = 0.1
    grips = {h: np.linalg.inv(obj) @ node.flanges[h] @ tcp for h in hands}
    node.holds = [{'object': 'crate', 'type': None, 'hands': list(hands), 'grips': grips}]
    return node


def test_one_hand_object_jog_moves_the_object_and_the_hand():
    node = object_node(hands=('left',))
    node._object_step(node.holds[0], ((0.1, 0, 0), (0, 0, 0)), 0.5)
    assert node.sent[0]['left_x'] == pytest.approx(0.55)          # the flange went with the object
    assert node.sent[0]['left_y'] == pytest.approx(0.2)
    assert node.checks[0][0] == 'left_arm'                          # one hand: its own group
    assert node.objects['crate']['target'][0, 3] == pytest.approx(0.55)


def test_two_hands_step_together_in_one_message_and_one_check():
    node = object_node()
    node._object_step(node.holds[0], ((0.0, 0.0, 0.1), (0, 0, 0)), 0.5)
    assert len(node.sent) == 1 and len(node.checks) == 1
    assert node.checks[0][0] == ''                                   # whole robot: both arms in one request
    assert set(node.sent[0]) == {f'{h}_{a}' for h in ('left', 'right') for a in 'xyz'}
    assert node.sent[0]['left_z'] == pytest.approx(0.05) and node.sent[0]['right_z'] == pytest.approx(0.05)


def test_turning_the_object_moves_the_hands_about_its_origin():
    node = object_node()
    node._object_step(node.holds[0], ((0, 0, 0), (0, 0, 1.0)), math.pi / 2)    # 90 deg about z
    # the object origin is (0.5, 0): the left hand (y +0.2) goes to x 0.3, the right hand to x 0.7
    assert node.sent[0]['left_x'] == pytest.approx(0.3) and node.sent[0]['left_y'] == pytest.approx(0.0, abs=1e-9)
    assert node.sent[0]['right_x'] == pytest.approx(0.7)


def test_when_one_hand_cannot_follow_no_hand_moves():
    node = object_node(stuck=('right',))
    node._object_step(node.holds[0], ((0.1, 0, 0), (0, 0, 0)), 0.5)
    assert not node.sent and not node.checks
    assert 'right cannot follow' in node.notices[0]
    node._object_step(node.holds[0], ((0.1, 0, 0), (0, 0, 0)), 0.5)
    assert len(node.notices) == 1                                    # said once


def test_a_collision_stops_both_hands():
    node = object_node(valid=False)
    node._object_step(node.holds[0], ((0.1, 0, 0), (0, 0, 0)), 0.5)
    assert not node.sent and 'collision' in node.notices[0]
    assert node.objects['crate']['target'][0, 3] == pytest.approx(0.5)   # the target did not run off


def test_a_held_tool_is_jogged_not_refused():
    import time
    from types import SimpleNamespace
    node = object_node()
    node.held_tools = {'left', 'right'}
    node.last_time = None
    clock = [0.0]
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0] * 1e9)))
    node.tools['left']['twist'] = ((0.1, 0, 0), (0, 0, 0))
    node.tools['left']['received'] = time.monotonic()
    node._step()          # the first step has dt 0 and sets the clock
    clock[0] = 0.05
    node._step()
    assert node.sent and node.sent[-1]['right_x'] == pytest.approx(0.505)   # the other hand went with it
    node.tools['left']['received'] = -1e9
    node._step()
    assert 'crate' not in node.objects                                # stopped: the next jog starts from the object


def test_the_elbow_swing_still_works_while_holding():
    node = object_node(hands=('left',))
    node.held_tools = {'left'}
    node.ik.step = lambda tool, flange, start: dict(start)
    jog = node.tools['left']
    jog.update(swivel='left_x', chain=None, target=np.eye(4))
    node._swing('left', jog, {'left_x': 0.0, 'left_y': 0.1}, 0.05)
    assert node.sent == [{'left_x': 0.05, 'left_y': 0.1}]


def test_the_jog_state_names_the_tools_that_jog_an_object():
    import json
    node = object_node()
    published = []
    node.mode_state = type('P', (), {'publish': staticmethod(lambda m: published.append(json.loads(m.data)))})()
    node._publish_state()
    assert published[-1] == {'left': {'object': 'crate', 'hands': ['left', 'right']},
                             'right': {'object': 'crate', 'hands': ['left', 'right']}}
    node.holds = []
    node._publish_state()
    assert published[-1] == {}


def test_the_holding_message_gives_the_grips():
    import json
    from types import SimpleNamespace
    node = object_node()
    node.mode_state = SimpleNamespace(publish=lambda m: None)
    node._holding(SimpleNamespace(data=json.dumps({'object': 'crate', 'tools': ['left'], 'holds': [
        {'object': 'crate', 'type': 'bin', 'hands': ['left'], 'grips': {'left': np.eye(4).tolist()}}]})))
    assert node.holds[0]['type'] == 'bin' and node.holds[0]['grips']['left'] == pytest.approx(np.eye(4))
    node._holding(SimpleNamespace(data='{}'))
    assert node.holds == [] and node.objects == {}


def test_the_validity_request_carries_the_object_shapes_attached_to_the_first_hand():
    pytest.importorskip('moveit_msgs')
    node = object_node()
    node.robot_data = {'tool_frames': node.ik.tools}
    node.store = type('S', (), {'object': staticmethod(lambda kind: {'collision': {'box': [0.4, 0.3, 0.2]}})})()
    hold = {**node.holds[0], 'type': 'crate'}
    attached = node._attached(hold)
    assert len(attached) == 1 and attached[0].link_name == 'left_link7' and attached[0].object.id == 'crate'
    assert attached[0].touch_links == ['left_link7', 'right_link7']
    pose = attached[0].object.primitive_poses[0].position           # object origin in the left flange frame
    assert (pose.x, pose.y, pose.z) == pytest.approx((0.0, -0.2, 0.1))
    assert node._attached({**hold, 'type': None}) == []             # a type without collision shapes: nothing
