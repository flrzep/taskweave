"""Object points -> tool targets in the robot's base frame (pure math; used by go_to_point).

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
