"""Tool target <-> flange pose math (no ROS needed)."""

import numpy as np
import pytest

from taskweave_skills.ik import IkError, flange_target, quaternion, tcp_matrix, tool_target

TOOL = {'link': 'left_link7', 'xyz': [-0.28, 0.0, 0.0], 'rpy': [1.5708, 0.0, -1.5708]}


def test_tcp_frame_is_approach_z_and_closing_y():
    pose = tcp_matrix({'tcp': [0.8, 0.2, 0.4], 'approach': [2, 0, 0], 'closing': [0.1, 1, 0]})  # made normal
    assert pose[:3, 2] == pytest.approx([1, 0, 0]) and pose[:3, 1] == pytest.approx([0, 1, 0])
    assert pose[:3, 0] == pytest.approx([0, 0, -1]) and pose[:3, 3] == pytest.approx([0.8, 0.2, 0.4])
    with pytest.raises(IkError, match='parallel'):
        tcp_matrix({'tcp': [0, 0, 0], 'approach': [1, 0, 0], 'closing': [2, 0, 0]})


def test_flange_target_and_tool_target_are_inverse():
    target = {'tcp': [0.85, 0.25, 0.42], 'approach': [1, 0, 0], 'closing': [0, 1, 0]}
    flange = flange_target(target, TOOL)
    assert flange[:3, 3] == pytest.approx([0.57, 0.25, 0.42], abs=1e-4)        # 0.28 m behind the TCP
    back = tool_target(flange, TOOL)
    assert back['tcp'] == pytest.approx(target['tcp'], abs=1e-4)
    assert back['approach'] == pytest.approx(target['approach'], abs=1e-4)
    assert back['closing'] == pytest.approx(target['closing'], abs=1e-4)


@pytest.mark.parametrize('angles', [(0.1, 0.2, 0.3), (np.pi, 0, 0), (0, np.pi, 0), (0, 0, np.pi), (2.5, -1.2, 0.7)])
def test_quaternion_round_trip(angles):
    from taskweave_skills.ik import rpy_matrix
    rotation = rpy_matrix([0, 0, 0], angles)[:3, :3]
    w, x, y, z = quaternion(rotation)
    rebuilt = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    assert rebuilt == pytest.approx(rotation, abs=1e-9)


def test_backward_approach_is_flagged():
    from taskweave_skills.ik import target_warnings
    assert target_warnings({'tcp': [0.85, -0.1, 0.4], 'approach': [-1, 0, 0], 'closing': [0, 1, 0]})
    assert not target_warnings({'tcp': [0.85, -0.1, 0.4], 'approach': [1, 0, 0], 'closing': [0, 1, 0]})
    assert not target_warnings({'tcp': [0.5, 0.2, 0.2], 'approach': [0, 0, -1], 'closing': [1, 0, 0]})  # from above


def test_reach_search_tries_the_smallest_shifts_first():
    from taskweave_skills.ik import shifts
    order = list(shifts(0.05, 0.1))
    assert len(order) == 12 and order[:2] == [[0.05, 0, 0], [-0.05, 0, 0]]
    assert max(abs(v) for v in order[5]) == 0.05 and max(abs(v) for v in order[6]) == 0.1
