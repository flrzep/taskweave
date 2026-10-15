"""Gripper opening <-> gripper joint, from the robot profile's table.

A gripper in the profile can list how far its fingers are open at some joint values:

    grippers:
      left: {joint: left_gripper_joint, open: 0.0, closed: 1.63,
             opening_m: [[0.0, 0.077], [0.8, 0.037], [1.55, 0.0]]}   # [joint, opening in m], measured

Openings (a point's ``grip_m``) are robot-independent numbers a person can check with a ruler; the table turns them
into this robot's joint value (linear between the rows). Without a table a gripper has no opening scale.
"""

from __future__ import annotations

from collections.abc import Mapping


def opening_table(gripper: Mapping) -> list[tuple[float, float]] | None:
    """[(joint, opening m)] sorted by opening (ascending), or None without ``opening_m``."""
    rows = gripper.get('opening_m')
    if not rows:
        return None
    return sorted(((float(joint), float(opening)) for joint, opening in rows), key=lambda row: row[1])


def opening_range(gripper: Mapping) -> tuple[float, float] | None:
    """(smallest, largest) opening in m the table covers, or None."""
    table = opening_table(gripper)
    return None if table is None else (table[0][1], table[-1][1])


def _interpolate(table: list[tuple[float, float]], value: float, source: int) -> float:
    rows = sorted(table, key=lambda row: row[source])
    target = 1 - source
    if value <= rows[0][source]:
        return rows[0][target]
    for (a, b) in zip(rows, rows[1:]):
        if value <= b[source]:
            span = b[source] - a[source]
            fraction = 0.0 if span == 0 else (value - a[source]) / span
            return a[target] + fraction * (b[target] - a[target])
    return rows[-1][target]


def joint_for_opening(gripper: Mapping, opening_m: float) -> float:
    """The joint value that opens the fingers ``opening_m`` (clamped to the table). Raises ValueError without one."""
    table = opening_table(gripper)
    if table is None:
        raise ValueError(f'gripper {gripper.get("joint", "?")} has no opening_m table in the profile')
    return _interpolate(table, float(opening_m), source=1)


def opening_for_joint(gripper: Mapping, joint: float) -> float:
    """How far (m) the fingers are open at ``joint`` (clamped to the table). Raises ValueError without one."""
    table = opening_table(gripper)
    if table is None:
        raise ValueError(f'gripper {gripper.get("joint", "?")} has no opening_m table in the profile')
    return _interpolate(table, float(joint), source=0)
