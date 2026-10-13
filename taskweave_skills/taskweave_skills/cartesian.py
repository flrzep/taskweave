"""Straight-line TCP motion for one or more tools at once (linear approach, moving a held object).

The TCPs move from their current poses to their goals in lockstep: positions on straight lines, orientations
interpolated (slerp), split into steps of at most ``step_m`` / ``step_rad``. Each step is solved by one local IK
descent from the previous step's joints (taskweave_skills.ik IkClient.step: the profile's joint limits and collision
checks), so the whole path is known to be feasible before anything moves. The joint waypoints are timed so no TCP
exceeds ``speed`` (m/s) / ``turn`` (rad/s) and returned as a motion_plan.Trajectory for the joint skills to stream.
"""

from __future__ import annotations

import math

import numpy as np

from .ik import rpy_matrix, tcp_matrix
from .motion_plan import Trajectory


class PathError(RuntimeError):
    """No straight-line path; the message says where it fails."""


def rotation_angle(r: np.ndarray) -> float:
    return math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))


def slerp(r0: np.ndarray, r1: np.ndarray, s: float) -> np.ndarray:
    """Rotation a fraction ``s`` of the way from r0 to r1 (about the fixed axis of r0^T r1)."""
    delta = r0.T @ r1
    angle = rotation_angle(delta)
    if angle < 1e-9:
        return r0.copy()
    if abs(angle - math.pi) < 1e-6:  # axis from the symmetric part
        axis = np.sqrt(np.maximum((np.diag(delta) + 1) / 2, 0))
        axis = axis / np.linalg.norm(axis)
    else:
        axis = np.array([delta[2, 1] - delta[1, 2], delta[0, 2] - delta[2, 0], delta[1, 0] - delta[0, 1]])
        axis = axis / (2 * math.sin(angle))
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    a = angle * s
    return r0 @ (np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k)


def interpolate(t0: np.ndarray, t1: np.ndarray, s: float) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, :3] = slerp(t0[:3, :3], t1[:3, :3], s)
    pose[:3, 3] = t0[:3, 3] + (t1[:3, 3] - t0[:3, 3]) * s
    return pose


def steps_needed(starts: dict, goals: dict, step_m: float, step_rad: float) -> int:
    worst = 1
    for tool, start in starts.items():
        goal = goals[tool]
        worst = max(worst, math.ceil(np.linalg.norm(goal[:3, 3] - start[:3, 3]) / step_m),
                    math.ceil(rotation_angle(start[:3, :3].T @ goal[:3, :3]) / step_rad))
    return worst


def tcp_targets(tools: dict) -> dict[str, np.ndarray]:
    """{tool: {tcp, approach, closing}} -> {tool: 4x4 TCP pose}."""
    return {tool: tcp_matrix(target) for tool, target in tools.items()}


def matching_flip(goals: dict, starts: dict, tool_frames: dict) -> dict:
    """For a tool whose grasp may be turned 180 deg about the approach (tool frame ``flip``), the goal turned the way
    the TCP already is: IK may have reached the start with the flipped grasp, and a straight line must not turn it."""
    out = dict(goals)
    for tool, goal in goals.items():
        if tool_frames.get(tool, {}).get('flip') and tool in starts and starts[tool][:3, 1] @ goal[:3, 1] < 0:
            out[tool] = goal @ np.diag([-1.0, -1.0, 1.0, 1.0])
    return out


def linear_path(ik, starts: dict, goals: dict, current: dict, speed: float, turn: float,
                step_m: float = 0.01, step_rad: float = 0.05) -> Trajectory:
    """Joint trajectory moving each tool's TCP from ``starts`` to ``goals`` (4x4 TCP poses in the base frame) in
    lockstep. ``current``: the arm joints now (the first step's seed). Raises PathError."""
    count = steps_needed(starts, goals, step_m, step_rad)
    joints = dict(current)
    names = sorted(current)
    times, points, elapsed = [0.0], [[current[n] for n in names]], 0.0
    previous = starts
    tcp_to_flange = {tool: np.linalg.inv(rpy_matrix(ik.tools[tool].get('xyz', [0, 0, 0]),
                                                    ik.tools[tool].get('rpy', [0, 0, 0]))) for tool in starts}
    for i in range(1, count + 1):
        step = {tool: interpolate(starts[tool], goals[tool], i / count) for tool in starts}
        for tool, tcp in step.items():
            solved = ik.step(tool, tcp @ tcp_to_flange[tool], joints)
            if solved is None:
                raise PathError(f'no straight-line path for {tool}: step {i} of {count} '
                                f'(TCP {np.round(tcp[:3, 3], 3).tolist()}) is out of reach, at a joint limit '
                                f'or in collision')
            joints.update(solved)
        duration = max(max(np.linalg.norm(step[t][:3, 3] - previous[t][:3, 3]) / speed,
                           rotation_angle(previous[t][:3, :3].T @ step[t][:3, :3]) / turn) for t in step)
        elapsed += max(duration, 1e-3)
        times.append(elapsed)
        points.append([joints[n] for n in names])
        previous = step
    return Trajectory(names, times, points)
