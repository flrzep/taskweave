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
