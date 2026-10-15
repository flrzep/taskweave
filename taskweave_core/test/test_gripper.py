import pytest

from taskweave_core.gripper import joint_for_opening, opening_for_joint, opening_range

S1 = {'joint': 'left_gripper_joint', 'open': 0.0, 'closed': 1.63,
      'opening_m': [[0.0, 0.077], [0.8, 0.037], [1.55, 0.0]]}


def test_openings_and_joints_are_interpolated_from_the_table():
    assert opening_range(S1) == (0.0, 0.077)
    assert joint_for_opening(S1, 0.077) == 0.0 and joint_for_opening(S1, 0.0) == 1.55
    assert joint_for_opening(S1, 0.057) == pytest.approx(0.4)
    assert opening_for_joint(S1, 0.4) == pytest.approx(0.057)
    assert opening_for_joint(S1, 1.63) == 0.0 and joint_for_opening(S1, 0.2) == 0.0   # clamped to the table


def test_a_gripper_without_a_table_has_no_opening_scale():
    assert opening_range({'joint': 'g'}) is None
    with pytest.raises(ValueError, match='no opening_m table'):
        joint_for_opening({'joint': 'g'}, 0.01)
