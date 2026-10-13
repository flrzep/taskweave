"""Object point -> tool targets in the base frame (taskweave_skills.points)."""

import math

import numpy as np
import pytest

from taskweave_core.config.poses import ObjectInstance, Pose2D, Pose3D
from taskweave_skills.points import point_targets

DOWN = (0.0, math.pi, 0.0)  # canonical TCP pointing down: approach = -z, closing = +y


def base_at(x, y, z, yaw):
    m = np.eye(4)
    m[:3, :3] = [[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]]
    m[:3, 3] = [x, y, z]
    return m


def test_point_on_a_turned_object_seen_from_the_robot():
    crate = ObjectInstance('crate_1', 'crate', Pose2D(2.0, 1.0, math.pi / 2), z=0.4)
    point = {'left_tool': Pose3D((0.1, 0.0, 0.2), DOWN, 'crate'), 'right_tool': None, 'tool_type': None}
    targets = point_targets(point, crate, base_at(1.0, 1.0, 0.5, 0.0))
    left = targets['left']
    assert left['tcp'] == pytest.approx([1.0, 0.1, 0.1])  # object x (+0.1) is cell +y after the 90 deg turn
    assert left['approach'] == pytest.approx([0, 0, -1], abs=1e-9)
    assert left['closing'] == pytest.approx([-1, 0, 0], abs=1e-9)  # object y = cell -x
    assert set(targets) == {'left'}


def test_offset_backs_off_along_the_approach_and_tools_can_be_chosen():
    box = ObjectInstance('box_1', 'box', Pose2D(0.0, 0.0, 0.0))
    point = {'left_tool': Pose3D((0.5, 0.2, 0.3), DOWN, 'box'), 'right_tool': Pose3D((0.5, -0.2, 0.3), DOWN, 'box')}
    targets = point_targets(point, box, base_at(0, 0, 0, 0), offset_m=0.1, tools=['right'])
    assert set(targets) == {'right'} and targets['right']['tcp'] == pytest.approx([0.5, -0.2, 0.4])
    with pytest.raises(KeyError, match='no left'):
        point_targets({'right_tool': point['right_tool']}, box, base_at(0, 0, 0, 0), tools=['left'])


def test_points_of_a_stack_refer_to_its_top_item():
    stack = ObjectInstance('stack_1', 'box', Pose2D(0.0, 0.0, 0.0), z=0.1, stack_count=8, stack_pitch=0.04)
    point = {'left_tool': Pose3D((0.0, 0.0, 0.05), DOWN, 'box')}
    assert stack.top_z == pytest.approx(0.38)
    assert point_targets(point, stack, base_at(0, 0, 0, 0))['left']['tcp'] == pytest.approx([0, 0, 0.43])


def test_a_taught_tcp_comes_back_as_the_same_target():
    """ObjectInstance.relative_pose (teaching) is the inverse of point_targets (go_to_point)."""
    from taskweave_core.config.poses import quaternion_from_rpy

    stack = ObjectInstance('stack_1', 'box', Pose2D(8.8, 1.1, 1.5708), z=0.1, stack_count=8, stack_pitch=0.04)
    position, rpy = (8.6, 1.3, 0.55), (0.3, 2.8, -1.0)  # some TCP pose in the cell frame
    taught = stack.relative_pose(position, quaternion_from_rpy(rpy))
    base = base_at(7.9, 0.4, 0.0, 0.7)
    left = point_targets({'left_tool': taught}, stack, base)['left']
    from taskweave_skills.ik import rpy_matrix
    expected = np.linalg.inv(base) @ rpy_matrix(position, rpy)
    assert left['tcp'] == pytest.approx(expected[:3, 3], abs=1e-5)
    assert left['approach'] == pytest.approx(expected[:3, 2], abs=1e-5)
    assert left['closing'] == pytest.approx(expected[:3, 1], abs=1e-5)
