"""Object points -> tool targets in the robot's base frame, and contacts -> object poses (pure math; used by
go_to_point and move_object).

An object point gives each tool's canonical TCP pose relative to its object (``<tool>_tool: {xyz, rpy}``, z =
approach, y = closing; objects/<object>.yaml). A placed object instance stands at x, y, z, yaw in the cell frame
(cell/objects.yaml). The robot's base pose in the cell comes from TF.
"""

from __future__ import annotations

import math

import numpy as np

from .ik import rpy_matrix

TOOL_SUFFIX = '_tool'


def instance_matrix(instance) -> np.ndarray:
    """4x4 pose of a placed object (taskweave_core ObjectInstance) in the cell frame; of a stack's top item."""
    c, s = math.cos(instance.pose.yaw), math.sin(instance.pose.yaw)
    m = np.eye(4)
    m[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    m[:3, 3] = [instance.pose.x, instance.pose.y, instance.top_z]  # a stack: its top item
    return m


def point_targets(point: dict, instance, base_in_cell: np.ndarray, offset_m: float = 0.0,
                  tools=None) -> dict[str, dict]:
    """Tool targets {tool: {tcp, approach, closing}} in the base frame for an object point.

    ``point``: ConfigStore.object_point() result (Pose3D per ``<tool>_tool``). ``offset_m``: stop this far back
    along the approach (a pre-grasp pose). ``tools``: only these tools (default: every tool the point has)."""
    to_base = np.linalg.inv(base_in_cell) @ instance_matrix(instance)
    targets = {}
    for key, pose in point.items():
        if not key.endswith(TOOL_SUFFIX) or pose is None:
            continue
        tool = key[:-len(TOOL_SUFFIX)]
        if tools is not None and tool not in tools:
            continue
        tcp = to_base @ rpy_matrix(pose.xyz, pose.rpy)
        approach, closing = tcp[:3, 2], tcp[:3, 1]
        position = tcp[:3, 3] - offset_m * approach
        targets[tool] = {'tcp': [round(float(v), 5) for v in position],
                         'approach': [round(float(v), 6) for v in approach],
                         'closing': [round(float(v), 6) for v in closing]}
    if tools is not None and set(tools) - set(targets):
        raise KeyError(f'the point has no {", ".join(sorted(set(tools) - set(targets)))} tool pose')
    return targets


def onto_pose(target, target_contact, held_contact, base_in_cell: np.ndarray, offset_m: float = 0.0) -> np.ndarray:
    """4x4 pose (base frame) of a held object whose contact ``held_contact`` (Pose3D, relative to the held object)
    lies on ``target_contact`` (Pose3D) of the placed object ``target``, ``offset_m`` back against that contact's z
    (the direction the held object moves to engage, e.g. down onto a hook)."""
    back = np.eye(4)
    back[2, 3] = -offset_m
    at = instance_matrix(target) @ rpy_matrix(target_contact.xyz, target_contact.rpy) @ back  # cell frame
    return np.linalg.inv(base_in_cell) @ at @ np.linalg.inv(rpy_matrix(held_contact.xyz, held_contact.rpy))


def moved_object(obj: np.ndarray, by=None, at=None, turn_deg: float = 0.0, tilt_deg: float = 0.0) -> np.ndarray:
    """A held object's pose (4x4, base frame) after move_object: turned about the vertical through its origin by
    ``turn_deg``, tilted about the base's y axis through it by ``tilt_deg`` (far side up), then shifted ``by`` [x, y, z]
    or so that its origin lands ``at`` [x, y, z]."""
    turn, tilt = math.radians(turn_deg), -math.radians(tilt_deg)  # about +y a positive angle lowers the far side
    c, s, ct, st = math.cos(turn), math.sin(turn), math.cos(tilt), math.sin(tilt)
    rotate = np.eye(4)
    rotate[:3, :3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
    pivot, back = np.eye(4), np.eye(4)
    pivot[:3, 3], back[:3, 3] = obj[:3, 3], -obj[:3, 3]
    shift = np.eye(4)
    shift[:3, 3] = (np.array(at, dtype=float) - obj[:3, 3]) if at is not None else np.array(by or (0, 0, 0), dtype=float)
    return shift @ pivot @ rotate @ back @ obj
