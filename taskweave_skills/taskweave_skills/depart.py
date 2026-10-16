"""Backing out of a tight spot before Nav2 takes over (no ROS needed).

At a pick stance the robot's footprint (envelope + padding, robots/<robot>/navigation.yaml) can overlap a bin it is
reaching into: the arms reach over it, which a 2D footprint cannot express. Nav2 then sees the robot in collision
already, and every part of it refuses to move it (the controller finds no valid trajectory, the BackUp behaviour
sees a collision on the first step). ``departure`` says how far the robot must back straight out until its
footprint is clear of the map's occupied cells, and checks that the strip it backs into is free.
"""

from __future__ import annotations

import math

import numpy as np

OCCUPIED = 50       # occupancy (0-100) from which a map cell counts as an obstacle
SAMPLE = 0.5        # footprint sample spacing, in map cells


def envelope(navigation: dict, base: dict, carrying: bool = False) -> tuple[float, float, float, float]:
    """(front, back, left, right) around the base frame, without padding: the navigation envelope, the carry envelope
    while the robot carries an object (if there is one), else the chassis footprint centred on the base frame."""
    e = (navigation.get('carry_envelope') if carrying else None) or navigation.get('envelope')
    if e:
        return e['front'], e['back'], e['left'], e['right']
    length, width = base['footprint']
    return length / 2, length / 2, width / 2, width / 2


def padding(navigation: dict, nav2_default_m: float = 0.01) -> float:
    """navigation.yaml ``padding`` (m), grows the envelope like Nav2's footprint_padding; unset: Nav2's default."""
    return float(navigation.get('padding', nav2_default_m))


def footprint_box(navigation: dict, base: dict, carrying: bool = False) -> tuple[float, float, float, float]:
    """(front, back, left, right) around the base frame, padding included: the box Nav2 checks."""
    front, back, left, right = envelope(navigation, base, carrying)
    pad = padding(navigation)
    return front + pad, back + pad, left + pad, right + pad


def occupied(grid: np.ndarray, info, pose, x_range, y_range) -> int:
    """Occupied cells of ``grid`` (rows = y) in the rectangle x_range x y_range of the robot frame at ``pose``
    (x, y, yaw in the map frame). Cells outside the map count as free."""
    res = info.resolution
    step = res * SAMPLE
    xs, ys = np.meshgrid(np.arange(x_range[0], x_range[1] + 1e-9, step), np.arange(y_range[0], y_range[1] + 1e-9, step))
    c, s = math.cos(pose[2]), math.sin(pose[2])
    col = np.floor((pose[0] + xs * c - ys * s - info.origin_x) / res).astype(int)
    row = np.floor((pose[1] + xs * s + ys * c - info.origin_y) / res).astype(int)
    inside = (col >= 0) & (col < grid.shape[1]) & (row >= 0) & (row < grid.shape[0])
    return int((grid[row[inside], col[inside]] >= OCCUPIED).sum())


def departure(grid: np.ndarray, info, pose, box, max_m: float, margin_m: float, step_m: float = 0.05):
    """How far to back straight out: 0.0 if the footprint is clear where it stands, the distance (+ margin) at which
    it is clear, or raises ValueError (nothing helps within max_m, or the way back is blocked)."""
    front, back, left, right = box
    if not occupied(grid, info, pose, (-back, front), (-right, left)):
        return 0.0
    for d in np.arange(step_m, max_m + 1e-9, step_m):
        if occupied(grid, info, pose, (-back - d, -back), (-right, left)):  # the strip the robot backs into
            raise ValueError(f'the robot overlaps an obstacle and the way back is blocked within {d:.2f} m')
        if not occupied(grid, info, pose, (-back - d, front - d), (-right, left)):
            distance = min(float(d) + margin_m, max_m)
            if occupied(grid, info, pose, (-back - distance, -back), (-right, left)):
                raise ValueError(f'the robot overlaps an obstacle and the way back is blocked within {distance:.2f} m')
            return distance
    raise ValueError(f'the robot overlaps an obstacle that backing up {max_m:.2f} m does not clear')
