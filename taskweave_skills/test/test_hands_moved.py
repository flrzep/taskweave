"""Which hands a joint motion moves, on a robot whose profile has joint groups but no arm groups and no ik."""

from types import SimpleNamespace

import pytest

pytest.importorskip('sensor_msgs')

from taskweave_skills.joint_motion import MoveJoints  # noqa: E402


def skill(groups):
    data = {'grippers': {'left': {'joint': 'left_grip'}, 'right': {'joint': 'right_grip'}},
            'joints': {'groups': groups}}
    config = SimpleNamespace(robot=lambda _name: SimpleNamespace(data=data))
    made = object.__new__(MoveJoints)
    made.config, made.robot, made.ik = config, SimpleNamespace(name='r', data=data), None
    return made


def test_an_arm_joint_outside_every_group_moves_every_hand():
    made = skill({'torso': ['lift']})
    assert made._hands_moved({'left_joint1': 0.0}) == {'left', 'right'}
    assert made._hands_moved({'lift': 0.0}) == set()
    assert made._hands_moved({'left_grip': 0.0}) == {'left'}
