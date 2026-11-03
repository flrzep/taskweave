"""The Bridge's manual skills on a targets cell (cell schema 4): ``move {to}`` when the skill catalog has move, the old
skills otherwise; the picker lists over HTTP carry the target groups."""

import shutil
import threading
from pathlib import Path

import pytest

pytest.importorskip('rclpy')

from test_speed import FakeLogger, serve  # noqa: E402, F401 - serve: the fixture

from taskweave_core.config import ConfigStore  # noqa: E402
from taskweave_hmi import arm  # noqa: E402
from taskweave_hmi.server import Bridge  # noqa: E402

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'targets_new'
ROBOT = 'bot_two_arm'


@pytest.fixture
def config(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


def target_bridge(config, served):
    """A Bridge with a real store and nothing of ROS; ``served``: the skill names of the last catalog answer."""
    bridge = Bridge.__new__(Bridge)  # no node
    bridge.lock, bridge.listeners, bridge.sent, bridge.runs = threading.Lock(), [], [], []
    bridge.config, bridge.robot, bridge.manual, bridge.served_skills = config, ROBOT, None, served
    bridge.check_idle = lambda: None
    logger = FakeLogger()
    bridge.get_logger = lambda: logger
    bridge._broadcast = lambda kind, data: bridge.sent.append((kind, data))
    bridge.run_skill = lambda skill, args, **_: bridge.runs.append((skill, args)) or {'outcome': 'succeeded'}
    return bridge


def test_manual_skills_run_move_to_the_target_when_the_catalog_has_move(config):
    bridge = target_bridge(config, ['wait', 'move', 'grip'])
    bridge.drive('stance:karakuri_hanger')
    bridge.drive('stance:bay_1_stack_1_back/pick')
    bridge.drive('waiting')
    bridge.go_to_pose('home')
    bridge.go_to_pose('frame_grasp_approach')
    bridge.test_position('frame_type2', 'rails_top', {'at': 'bay_1_stack_1_front', 'hands': 'left'})
    bridge.test_held('frame_type2', 'carry')
    assert bridge.runs == [('move', {'to': 'karakuri_hanger/staging'}), ('move', {'to': 'bay_1_stack_1_back/pick'}),
                           ('move', {'to': 'waiting'}), ('move', {'to': 'home'}),
                           ('move', {'to': 'frame_grasp_approach'}),
                           ('move', {'to': 'bay_1_stack_1_front/rails_top', 'hands': 'left'}),
                           ('move', {'to': 'carry'})]
    with pytest.raises(arm.ArmError, match='moves the base'):
        bridge.go_to_pose('waiting')


def test_manual_skills_run_the_old_skills_until_move_is_served(config):
    bridge = target_bridge(config, ['navigate_to', 'move_gripper', 'move_joints', 'move_object'])
    bridge.drive('stance:karakuri_hanger')
    bridge.go_to_pose('home')
    bridge.go_to_pose('frame_grasp_approach')
    bridge.test_position('frame_type2', 'rails_top', {'at': 'bay_1_stack_1_front'})
    bridge.test_held('frame_type2', 'carry')
    assert [run[0] for run in bridge.runs] == ['navigate_to', 'move_joints', 'move_gripper', 'move_gripper',
                                               'move_object']
    assert bridge.runs[0][1] == {'target': 'stance:karakuri_hanger'}


def test_the_points_list_over_http_has_the_target_groups(serve, config):  # noqa: F811
    request = serve(target_bridge(config, None))
    status, names = request('GET', '/api/points')
    assert status == 200 and 'karakuri_hanger/hook' in names['object targets'] and 'locations' in names
    status, _ = request('PUT', '/api/points/location/spot', {'x': 0.5, 'y': -1.0, 'yaw': 0, 'create': True})
    assert status == 200 and 'spot' in request('GET', '/api/points')[1]['base targets']


def test_a_refused_target_write_is_422_and_a_missing_target_404_over_http(serve, config):  # noqa: F811
    request = serve(target_bridge(config, None))
    status, data = request('DELETE', '/api/points/stance/objects/frame_type2/pick')   # overrides refer to it
    assert status == 422 and 'instances.bay_1_stack_1_back.pick' in data['problems'][0]
    status, data = request('PUT', '/api/points/stance/objects/nothing/front', {'x': 0, 'y': 0, 'yaw': 0})
    assert status == 404 and "no object type 'nothing'" in data['problems'][0]
    status, data = request('PUT', '/api/points/stance/objects/frame_type2/fresh', {'x': -0.9, 'y': 0, 'yaw': 0})
    assert status == 200 and data['file'] == f'robots/{ROBOT}/objects/frame_type2.yaml'


# ---- P6-4: the plan's kind and resources, the teach request ----------------------------------------------------------
def test_the_live_plan_json_has_each_steps_kind_and_resources():
    from taskweave_hmi.server import step_dict
    from taskweave_interfaces.msg import PlanStep

    step = step_dict(PlanStep(step_id='s1', instruction='Drive', skill='move', args_json='{"to": "waiting"}',
                              kind='navigate', resources=['base', 'torso'], notes=['args.object: migrate']))
    assert step['kind'] == 'navigate' and step['resources'] == ['base', 'torso'] and step['args'] == {'to': 'waiting'}
    assert step['notes'] == ['args.object: migrate']
    assert step_dict(PlanStep(step_id='s2', skill='wait'))['kind'] == ''


def test_the_live_plan_json_has_each_steps_origin_and_replan_rules():
    from types import SimpleNamespace

    from taskweave_hmi.server import step_dict
    from taskweave_interfaces.msg import PlanStep

    step = step_dict(PlanStep(step_id='s2_s3', skill='wait', origin_json='{"include": ["pick"], "step": 2, "top": 1}',
                              replan_from='s2_s1', max_replans=3))
    assert step['origin'] == {'include': ['pick'], 'step': 2, 'top': 1}
    assert (step['replan_from'], step['max_replans']) == ('s2_s1', 3)
    assert step_dict(PlanStep(step_id='s1', skill='wait'))['origin'] is None   # not from a mission
    old = SimpleNamespace(step_id='s1', instruction='', skill='wait', args_json='', timeout_s=0.0, retries=-1,
                          on_failure='')                                        # an older message: no such fields
    assert {k: step_dict(old)[k] for k in ('origin', 'replan_from', 'max_replans')} == {
        'origin': None, 'replan_from': '', 'max_replans': 0}


def test_the_plan_preview_over_http_has_kind_and_resources(serve, config):  # noqa: F811
    (config.root / 'missions' / 'drive.yaml').write_text(
        'goal: d\nsteps:\n- {instruction: Drive, skill: move, args: {to: waiting}}\n'
        '- {instruction: Rest, skill: wait, args: {seconds: 1}}\n')
    bridge = target_bridge(config, None)
    bridge.schemas = lambda: (_ for _ in ()).throw(TimeoutError('no skill server'))   # the kinds need none
    status, plan = serve(bridge)('GET', '/api/missions/drive/plan')
    assert status == 200 and [(s['kind'], s['resources'][:1]) for s in plan['steps']] == [('navigate', ['base']),
                                                                                           ('', [])]
    assert [s['notes'] for s in plan['steps']] == [[], []]


def test_a_check_over_http_has_the_notes(serve, config):  # noqa: F811
    bridge = target_bridge(config, None)
    bridge.schemas = lambda: {}
    up = {'object': 'held', 'by': [0, 0, 1]}
    mission = {'goal': 'g', 'steps': [{'instruction': 'up', 'skill': 'move', 'args': up}]}
    status, data = serve(bridge)('POST', '/api/missions/up/validate', mission)
    assert status == 200 and [(n['step'], n['path']) for n in data['notes']] == [(0, ['args', 'object'])]


def teach_bridge(config, answer):
    bridge = target_bridge(config, None)
    bridge.teach_client, bridge.sent_requests = object(), []
    bridge.joint_positions = lambda: {}

    def call(_client, request):
        bridge.sent_requests.append(request)
        return answer(request)
    bridge.call = call
    return bridge


def test_a_capture_on_a_targets_cell_teaches_a_hand_target_and_shows_the_answer(config):
    from taskweave_interfaces.srv import TeachPoint

    def answer(request):
        return TeachPoint.Response(success=True, message=f"hand {request.name!r}: left_tool saved",
                                   file=str(config.root / 'objects' / 'frame_type2.yaml'))
    bridge = teach_bridge(config, answer)
    result = bridge.capture('frame_type2', 'rails_top', {'at': 'bay_1_stack_1_front', 'tools': ['left']})
    request = bridge.sent_requests[0]
    assert (request.name, request.kind, list(request.tools), request.instance_override) == (
        'bay_1_stack_1_front/rails_top', 'hand', ['left'], False)
    assert result == {'file': 'objects/frame_type2.yaml',
                      'message': "hand 'bay_1_stack_1_front/rails_top': left_tool saved"}


def test_a_held_pose_capture_shows_the_cores_cannot_be_taught_yet(serve, config):  # noqa: F811
    from taskweave_interfaces.srv import TeachPoint

    text = "object 'carry': a held object in the base frame cannot be taught yet: write the pose ..."
    bridge = teach_bridge(config, lambda request: TeachPoint.Response(success=False, message=text))
    status, data = serve(bridge)('POST', '/api/teach/held/frame_type2/carry/capture', {})
    assert status == 422 and data['problems'] == [text]
    assert bridge.sent_requests[0].kind == 'object' and bridge.sent_requests[0].name == 'carry'

