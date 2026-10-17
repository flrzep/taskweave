"""The tool_pose point kind (move_gripper to.pose), single-hand ``alone`` poses and the shared hands fragment."""

import pytest
from jsonschema import Draft202012Validator

from taskweave_core.config import ConfigStore, schemas
from taskweave_core.validation import check_point, oneof_problems

POSES = """
home: {j1: 0.0, j2: 0.0}
present: {tools: {left: {tcp: [0.5, 0.2, 0.4], approach: [1, 0, 0], closing: [0, 0, 1]}}, elbow: out}
"""


@pytest.fixture
def store(config_dir):
    (config_dir / 'robots' / 'demo_bot' / 'poses.yaml').write_text(POSES)
    return ConfigStore(config_dir)


def test_tool_poses_are_the_poses_that_place_tools(store):
    assert store.tool_pose_names('demo_bot') == ['present']
    assert store.point_names('demo_bot')['tool poses'] == ['present']
    assert 'home' in store.point_names('demo_bot')['joint poses']      # joint poses stay as they were


def test_check_point_tool_pose(store):
    assert check_point('tool_pose', 'present', store, 'demo_bot') is None
    assert 'places no tools' in check_point('tool_pose', 'home', store, 'demo_bot')
    assert "unknown pose 'nope'" in check_point('tool_pose', 'nope', store, 'demo_bot')


def test_a_point_can_have_single_hand_poses(config_dir):
    (config_dir / 'objects' / 'box.yaml').write_text(
        'name: box\npoints:\n  rim:\n    left_tool: {xyz: [0, 0.2, 0.3], rpy: [0, 0, 0]}\n'
        '    alone:\n      left_tool: {xyz: [0, 0.1, 0.3], rpy: [0, 0, 0]}\n')
    point = ConfigStore(config_dir).object_point('box', 'rim')
    assert point['left_tool'].xyz == (0.0, 0.2, 0.3) and point['alone']['left_tool'].xyz == (0.0, 0.1, 0.3)
    schema = {'type': 'object', 'properties': {'points': schemas.OBJECT['properties']['points']}}
    pose = {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    assert Draft202012Validator(schema).is_valid({'points': {'rim': {'alone': {'left_tool': pose}}}})


def test_hands_schema_takes_rules_names_and_lists():
    validator = Draft202012Validator(schemas.hands_schema(['left', 'right']))
    assert all(validator.is_valid(v) for v in ('auto', 'any_one', 'all', 'left', ['left', 'right']))
    assert not any(validator.is_valid(v) for v in ('middle', [], ['left', 'left'], ['up']))
    grip = Draft202012Validator(schemas.hands_schema(['left'], ('auto', 'all')))
    assert grip.is_valid('all') and not grip.is_valid('any_one')
    assert oneof_problems(schemas.hands_schema(['left']), allow_several=True) == []
