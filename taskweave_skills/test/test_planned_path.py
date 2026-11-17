"""planned_path: the states along a move, the TCP points of the hands that travel, and the latched topic."""

import numpy as np
import pytest

from taskweave_skills.planned_path import MIN_LENGTH_M, sample_states, tool_points


class Trajectory:
    def __init__(self, names, times, points):
        self.names, self.times, self.points = names, times, points


def fk_along_x(state, hands):
    """A hand's TCP sits at x = its joint value (y tells the hands apart)."""
    poses = {}
    for index, hand in enumerate(hands):
        pose = np.eye(4)
        pose[:3, 3] = [state[f'{hand}_joint'], float(index), 0.0]
        poses[hand] = pose
    return poses


START = {'a_joint': 0.0, 'b_joint': 1.0, 'other': 5.0}


def test_a_straight_move_goes_from_start_to_goal_with_every_joint_in_each_state():
    states = sample_states(START, {'a_joint': 1.0}, samples=4)
    assert [s['a_joint'] for s in states] == [0.0, 0.25, 0.5, 0.75, 1.0]
    assert all(s['b_joint'] == 1.0 and s['other'] == 5.0 for s in states)


def test_the_straight_part_follows_the_time_law_of_the_move():
    states = sample_states(START, {'a_joint': 1.0}, samples=4, ease=lambda f: f * f)
    assert [s['a_joint'] for s in states] == [0.0, 0.0625, 0.25, 0.5625, 1.0]


def test_a_planned_move_takes_the_planners_points_and_the_other_goal_joints_go_alongside():
    path = Trajectory(['a_joint'], [0.0, 1.0, 2.0], [[0.0], [0.7], [1.0]])
    states = sample_states(START, {'a_joint': 1.0, 'b_joint': 3.0}, path)
    assert [s['a_joint'] for s in states] == [0.0, 0.7, 1.0]
    assert [s['b_joint'] for s in states] == [1.0, 2.0, 3.0]


def test_a_long_trajectory_is_thinned_and_keeps_both_ends():
    count = 500
    path = Trajectory(['a_joint'], [float(i) for i in range(count)], [[i / (count - 1)] for i in range(count)])
    states = sample_states(START, {'a_joint': 1.0}, path, samples=40)
    assert 40 <= len(states) <= 43
    assert states[0]['a_joint'] == 0.0 and states[-1]['a_joint'] == 1.0


def test_a_goal_joint_without_a_state_is_left_out():
    states = sample_states(START, {'a_joint': 1.0, 'unknown': 2.0}, samples=2)
    assert 'unknown' not in states[-1]


def test_only_the_hands_that_travel_are_shown():
    states = sample_states(START, {'a_joint': 0.5}, samples=5)
    points = tool_points(states, fk_along_x, ['a', 'b'])
    assert list(points) == ['a']
    assert points['a'][0] == (0.0, 0.0, 0.0) and points['a'][-1] == (0.5, 0.0, 0.0)
    assert len(points['a']) == 6


def test_a_move_shorter_than_the_minimum_shows_nothing():
    states = sample_states(START, {'a_joint': MIN_LENGTH_M / 2}, samples=5)
    assert tool_points(states, fk_along_x, ['a', 'b']) == {}


def test_no_kinematics_shows_nothing():
    states = sample_states(START, {'a_joint': 1.0}, samples=5)
    assert tool_points(states, lambda state, hands: None, ['a']) == {}


def test_the_topic_carries_the_path_in_the_base_frame_and_is_emptied_after_the_move():
    rclpy = pytest.importorskip('rclpy')
    from conftest import RosRig, wait_until
    from nav_msgs.msg import Path
    from rclpy.qos import DurabilityPolicy, QoSProfile

    from taskweave_core import names
    from taskweave_skills.planned_path import PlannedPath

    with RosRig() as ros:
        node = ros.add(rclpy.create_node('planned_path_test'))
        seen = []
        node.create_subscription(Path, f'{names.PLANNED_PATH}/a', seen.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        ros.spin(2)
        planned = PlannedPath.for_node(node, 'base')
        assert PlannedPath.for_node(node, 'base') is planned
        planned.show({'a': [(0.0, 0.0, 0.0), (0.5, 0.0, 0.2)]})
        wait_until(lambda: seen, what='the planned path')
        assert seen[-1].header.frame_id == 'base'
        assert [(p.pose.position.x, p.pose.position.z) for p in seen[-1].poses] == [(0.0, 0.0), (0.5, 0.2)]
        planned.clear()
        wait_until(lambda: len(seen) > 1 and not seen[-1].poses, what='the emptied path')
