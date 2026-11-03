"""Teach saves over HTTP: a location or stance from the robot goes through the teach node (its checks, its refusal);
a capture can be previewed against what is saved; every Teach save answers with an undo, and POST /api/teach/undo
puts the files back."""

import math

import pytest

pytest.importorskip('rclpy')

from test_speed import serve  # noqa: E402, F401 - serve: the fixture
from test_targets_bridge import ROBOT, config, teach_bridge  # noqa: E402, F401 - config: the fixture

from taskweave_core.config import Pose2D  # noqa: E402

STALE = "base 'spawn': the sim clock jumped less than 2 s ago (clock jump); wait and try again"
TOOL_FRAMES = """tool_frames:
  left: {link: left_flange, xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}
  right: {link: right_flange, xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]}
"""


def node_writes(config, x=2.2):
    """The teach node's answer to a base request: it saves the robot's pose (here x, y 0, yaw 0) as asked."""
    from taskweave_interfaces.srv import TeachPoint

    def answer(request):
        paths = config.save_target(request.name, {'moves': 'base', 'x': x, 'y': 0.0, 'yaw': 0.0}, ROBOT,
                                   instance_override=request.instance_override)
        return TeachPoint.Response(success=True, message=f"base {request.name!r}: {{'x': {x}}}",
                                   file=', '.join(str(p) for p in paths))
    return answer


def test_a_location_from_the_robot_is_taught_by_the_teach_node_and_can_be_undone(serve, config):  # noqa: F811
    bridge = teach_bridge(config, node_writes(config))
    bridge.robot_pose = lambda: Pose2D(2.2, 0.0, 0.0)
    request = serve(bridge)
    before = (config.root / 'cell/targets.yaml').read_text()
    status, data = request('POST', '/api/points/location/spawn/teach', {})
    sent = bridge.sent_requests[0]
    assert status == 200 and (sent.name, sent.kind, sent.instance_override) == ('spawn', 'base', False)
    assert data['file'] == 'cell/targets.yaml' and data['message'].startswith("base 'spawn'") and data['at'] is None
    assert data['undo']['files'] == ['cell/targets.yaml'] and ('points', None) in bridge.sent
    config.reload()
    assert config.locations()['spawn'].x == pytest.approx(2.2)
    status, undone = request('POST', '/api/teach/undo', {'id': data['undo']['id']})
    assert status == 200 and undone['files'] == ['cell/targets.yaml']
    assert (config.root / 'cell/targets.yaml').read_text() == before
    status, again = request('POST', '/api/teach/undo', {'id': data['undo']['id']})
    assert status == 409 and 'nothing to undo' in again['problems'][0]


def test_a_stance_from_the_robot_is_measured_at_the_chosen_object(serve, config):  # noqa: F811
    bridge = teach_bridge(config, node_writes(config, -1.1))
    bridge.robot_pose = lambda: Pose2D(7.0, 1.0, 0.0)
    request = serve(bridge)
    status, data = request('POST', '/api/points/stance/objects/frame_type2/pick/teach', {'at': 'bay_2_stack_2_front'})
    sent = bridge.sent_requests[0]
    assert status == 200 and (sent.name, sent.instance_override) == ('bay_2_stack_2_front/pick', False)
    assert data['at'] == 'bay_2_stack_2_front' and data['distance_m'] == 1.0
    assert data['undo']['files'] == [f'robots/{ROBOT}/objects/frame_type2.yaml']
    status, data = request('POST', '/api/points/stance/instances/bay_1_stack_1_back/pick/teach', {})
    assert status == 200 and bridge.sent_requests[1].instance_override is True
    status, data = request('POST', '/api/points/stance/objects/frame_type2/pick/teach', {})
    assert status == 422 and 'pick the placed frame_type2' in data['problems'][0] and len(bridge.sent_requests) == 2


def test_a_refused_teach_is_the_nodes_answer_and_writes_nothing(serve, config):  # noqa: F811
    from taskweave_interfaces.srv import TeachPoint

    bridge = teach_bridge(config, lambda request: TeachPoint.Response(success=False, message=STALE))
    request = serve(bridge)
    before = (config.root / 'cell/targets.yaml').read_text()
    assert request('POST', '/api/points/location/spawn/teach', {}) == (422, {'problems': [STALE]})
    status, data = request('POST', '/api/points/location/spawn/teach', {'create': True})
    assert status == 422 and data['problems'] == ['spawn already exists'] and len(bridge.sent_requests) == 1
    assert (config.root / 'cell/targets.yaml').read_text() == before


def test_a_rename_with_a_save_from_the_robot_renames_first_then_teaches_the_new_name(serve, config):  # noqa: F811
    bridge = teach_bridge(config, node_writes(config, 2.3))
    request = serve(bridge)
    status, data = request('POST', '/api/points/location/spawn/teach', {'rename': 'start'})
    assert status == 200 and bridge.sent_requests[0].name == 'start'
    config.reload()
    assert 'spawn' not in config.locations() and config.locations()['start'].x == pytest.approx(2.3)
    request('POST', '/api/teach/undo', {'id': data['undo']['id']})       # one undo for both
    config.reload()
    assert 'start' not in config.locations() and config.locations()['spawn'].x == 2.0


def test_typed_saves_answer_with_an_undo_and_a_file_changed_since_is_refused(serve, config):  # noqa: F811
    bridge = teach_bridge(config, node_writes(config))
    request = serve(bridge)
    status, data = request('PUT', '/api/points/location/waiting', {'x': 2.1, 'y': 0.0, 'yaw': 0.0})
    assert status == 200 and data['undo']['label'] == 'location waiting'
    request('PUT', '/api/points/location/spawn', {'x': 2.1, 'y': 0.0, 'yaw': 0.0})
    status, refused = request('POST', '/api/teach/undo', {'id': data['undo']['id']})
    assert status == 409 and 'cell/targets.yaml changed since that save' in refused['problems'][0]


def test_a_capture_answers_where_it_was_taught_and_the_distance(config):  # noqa: F811
    from taskweave_interfaces.srv import TeachPoint

    bridge = teach_bridge(config, lambda request: TeachPoint.Response(success=True, message='ok', file=''))
    bridge.robot_pose = lambda: Pose2D(6.0, 0.0, 0.0)
    result = bridge.capture('frame_type2', 'rails_top', {'at': 'bay_1_stack_1_front', 'instance_override': True})
    assert bridge.sent_requests[0].instance_override is True
    assert (result['at'], result['distance_m'], result['undo']) == ('bay_1_stack_1_front', 1.0, None)


def test_a_capture_preview_compares_the_robot_now_with_the_saved_pose(serve, config):  # noqa: F811
    bridge = teach_bridge(config, None)
    saved = {'tcp': [0.48, -0.295, 0.6], 'approach': [1.0, 0.0, 0.0], 'closing': [0.0, -1.0, 0.0]}
    bridge.tool_targets = lambda: {'left': {**saved, 'tcp': [0.48, -0.295, 0.63]},
                                   'right': {'tcp': [0.48, 0.295, 0.6], 'approach': [1, 0, 0], 'closing': [0, 1, 0]}}
    bridge.joint_positions = lambda: {'body_joint': 0.52, 'head_joint1': -0.3491, 'head_joint2': 0.0}
    request = serve(bridge)
    status, data = request('POST', '/api/teach/arm/robot/frame_grasp_approach/preview', {})
    assert status == 200 and data['exists'] is True
    hands = {h['tool']: h for h in data['hands']}
    assert hands['left']['moved_mm'] == pytest.approx(30.0) and hands['left']['turned_deg'] == pytest.approx(0, abs=0.1)
    assert hands['right']['moved_mm'] == pytest.approx(0.0)
    status, data = request('POST', '/api/teach/arm/robot/no_such_pose/preview', {'form': 'hands'})
    assert status == 200 and data == {'exists': False, 'at': None}


def test_a_point_preview_takes_each_tcp_relative_to_the_object_from_tf(serve, config):  # noqa: F811
    from taskweave_core.config.poses import quaternion_from_rpy, quaternion_multiply

    profile = config.root / 'robots' / ROBOT / 'profile.yaml'
    profile.write_text(profile.read_text() + TOOL_FRAMES)
    config.reload()
    bridge = teach_bridge(config, None)
    bridge.robot_pose = lambda: Pose2D(5.0, 1.0, 0.0)
    bridge.cell_frame = 'world'
    point = config.type_target('frame_type2', 'rails_top', ROBOT)
    handled = config.handling_instance(config.object_instances()['bay_1_stack_1_front'], ROBOT, Pose2D(5.0, 1.0, 0.0))
    c, s = math.cos(handled.pose.yaw), math.sin(handled.pose.yaw)

    def frame_pose(link):  # each flange (= TCP) where the saved point puts it, the left one 20 mm higher
        tool = link.split('_')[0]
        hand = point.tools[f'{tool}_tool']
        x, y, z = hand.xyz
        position = (handled.pose.x + c * x - s * y, handled.pose.y + s * x + c * y,
                    handled.top_z + z + (0.02 if tool == 'left' else 0.0))
        return position, quaternion_multiply(quaternion_from_rpy((0.0, 0.0, handled.pose.yaw)),
                                             quaternion_from_rpy(hand.rpy))
    bridge.frame_pose = frame_pose
    status, data = serve(bridge)('POST', '/api/teach/arm/frame_type2/rails_top/preview', {'at': 'bay_1_stack_1_front'})
    assert status == 200 and data['at'] == 'bay_1_stack_1_front'
    hands = {h['tool']: h for h in data['hands']}
    assert hands['left']['moved_mm'] == pytest.approx(20.0, abs=0.2)
    assert hands['right']['moved_mm'] == pytest.approx(0.0, abs=0.2)
    assert hands['left']['turned_deg'] == pytest.approx(0.0, abs=0.1)
