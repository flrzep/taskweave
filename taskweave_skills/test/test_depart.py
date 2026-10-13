import math
from types import SimpleNamespace

import numpy as np
import pytest

from taskweave_skills.depart import departure, footprint_box

INFO = SimpleNamespace(resolution=0.05, origin_x=0.0, origin_y=0.0)
BOX = (1.0, 0.4, 0.6, 0.7)  # front, back, left, right (padding included)


def grid_with(*boxes):
    """10 x 10 m map with occupied rectangles (x0, y0, x1, y1) in metres."""
    grid = np.zeros((200, 200), dtype=np.int16)
    for x0, y0, x1, y1 in boxes:
        grid[round(y0 / 0.05):round(y1 / 0.05), round(x0 / 0.05):round(x1 / 0.05)] = 100
    return grid


def test_the_footprint_box_is_the_envelope_plus_padding():
    navigation = {'envelope': {'front': 0.9, 'back': 0.3, 'left': 0.5, 'right': 0.6}, 'padding': 0.1}
    assert footprint_box(navigation, {}) == pytest.approx(BOX)
    assert footprint_box({}, {'footprint': [0.8, 0.6]}) == pytest.approx((0.41, 0.41, 0.31, 0.31))


def test_a_clear_footprint_does_not_back_out():
    assert departure(grid_with((8, 0, 9, 10)), INFO, (5.0, 5.0, 0.0), BOX, 1.0, 0.1) == 0.0


def test_a_bin_in_the_footprint_means_backing_out_until_clear_plus_margin():
    # robot at x=5 facing +x, bin from x=5.6: the footprint (front 1.0) reaches 0.4 m into it
    distance = departure(grid_with((5.6, 3, 7, 7)), INFO, (5.0, 5.0, 0.0), BOX, 1.0, 0.1)
    assert distance == pytest.approx(0.5, abs=0.06)  # 0.4 to clear + 0.1 margin
    # the same facing +y: the check follows the heading
    rotated = departure(grid_with((3, 5.6, 7, 7)), INFO, (5.0, 5.0, math.pi / 2), BOX, 1.0, 0.1)
    assert rotated == pytest.approx(distance)


def test_a_wall_behind_or_too_deep_an_overlap_is_refused():
    with pytest.raises(ValueError, match='way back is blocked'):
        departure(grid_with((5.6, 3, 7, 7), (4.2, 3, 4.5, 7)), INFO, (5.0, 5.0, 0.0), BOX, 1.0, 0.1)
    with pytest.raises(ValueError, match='does not clear'):
        departure(grid_with((5.2, 3, 7, 7)), INFO, (5.0, 5.0, 0.0), BOX, 0.5, 0.1)
