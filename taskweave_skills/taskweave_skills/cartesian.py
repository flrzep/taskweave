"""Straight-line TCP motion for one or more tools at once (linear approach, moving a held object).

The TCPs move from their current poses to their goals in lockstep: positions on straight lines, orientations
interpolated (slerp), split into steps of at most ``step_m`` / ``step_rad``, so the whole path is known to be feasible
before anything moves. With the robot's URDF chains (IkClient.attach_chains) each step is solved on the chain (chain.py
damped least squares, iterated until the flange is on the line) while the arm's spare freedom (the elbow, its null
space) moves joints that are within the limit margin back toward mid-range, so a straight line does not end up pinned
at a joint limit; each step is then checked for collisions. Without chains, each step is one local IK descent from the
previous step's joints (IkClient.step), which keeps the configuration it starts in. The joint waypoints are timed so no TCP
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


LIMIT_PUSH_RAD = 0.02       # the most a step moves a joint back from a limit through the null space
ON_LINE_M, ON_LINE_RAD = 5e-4, 5e-3   # a chain step is done when the flange is this close to the line point
STEP_ITERATIONS = 8
PUSH_ITERATIONS = 3         # while a joint is within the margin, at least this many iterations per step (null-space push)


def limit_push(positions: dict, names: list[str], limits: dict, margin: float) -> np.ndarray:
    """Joint step toward mid-range for the joints within ``margin`` of a limit (more the closer), 0 for the others."""
    push = []
    for name in names:
        lower, upper = limits.get(name, (-math.inf, math.inf))
        value = positions[name]
        if value - lower < margin:
            push.append(LIMIT_PUSH_RAD * (1 - max(value - lower, 0.0) / margin))
        elif upper - value < margin:
            push.append(-LIMIT_PUSH_RAD * (1 - max(upper - value, 0.0) / margin))
        else:
            push.append(0.0)
    return np.array(push)


ELBOW_RATE_RAD = 0.03      # the most a step swings the elbow toward the preferred side (null space)
ELBOW_SETTLED_RAD = 0.1    # the elbow is on its side when it is this close (deviation, rad)


def elbow_joints(names: list[str]) -> list[str]:
    """Shoulder, elbow and wrist centres of a 7-joint arm: the frames of its joints 2, 4 and 6."""
    return [names[1], names[3], names[5]]


def elbow_deviation_of(chain, names: list[str], positions: dict, side) -> float:
    """How far (rad) the elbow is from ``side`` (ik.preferred_elbow), seen along the shoulder-wrist line."""
    from .ik import elbow_deviation, preferred_elbow

    frames = dict(zip(chain.movable, chain.frames(positions)[0], strict=True))
    shoulder, elbow, wrist = (frames[name][:3, 3] for name in elbow_joints(names))
    return elbow_deviation(shoulder, elbow, wrist, preferred_elbow(side, shoulder))


def elbow_push(chain, names: list[str], positions: dict, side) -> np.ndarray:
    """Joint step swinging the elbow toward ``side`` without moving the tip (along the arm's self-motion), more the
    further it is off; 0 once it is there or where the arm has no elbow freedom."""
    if len(names) < 7:
        return np.zeros(len(names))
    deviation = elbow_deviation_of(chain, names, positions, side)
    if deviation < ELBOW_SETTLED_RAD:
        return np.zeros(len(names))
    direction = chain.null_direction(positions, names)
    probe = {**positions, **{n: positions[n] + 0.01 * d for n, d in zip(names, direction, strict=True)}}
    slope = (elbow_deviation_of(chain, names, probe, side) - deviation) / 0.01
    if abs(slope) < 1e-6:
        return np.zeros(len(names))
    return -math.copysign(ELBOW_RATE_RAD * min(1.0, deviation / 0.5), slope) * direction


def chain_step(chain, names: list[str], positions: dict, flange: np.ndarray, limits: dict, margin: float,
               elbow=None):
    """The arm joints that put the chain's tip on ``flange`` (4x4) from ``positions`` (every chain joint), pushed away
    from limits through the null space and, with ``elbow`` (a side, ik.preferred_elbow), swung toward that side;
    (joints, '') or (None, why not)."""
    from .chain import pose_error

    state = dict(positions)
    held = ''
    for iteration in range(1, STEP_ITERATIONS + 1):
        push = limit_push(state, names, limits, margin)
        if elbow is not None and iteration == 1:
            push = push + elbow_push(chain, names, state, elbow)
        new, held = chain.solve_step(state, flange, names, limits, null=push)
        state.update(new)
        error = pose_error(flange, chain.fk(state))
        on_line = np.linalg.norm(error[:3]) < ON_LINE_M and np.linalg.norm(error[3:]) < ON_LINE_RAD
        if on_line and (iteration >= PUSH_ITERATIONS or not np.any(push)):
            return {name: state[name] for name in names}, ''
    return None, f'{held} at its limit' if held else 'out of reach'


def linear_path(ik, starts: dict, goals: dict, current: dict, speed: float, turn: float,
                step_m: float = 0.01, step_rad: float = 0.05, state: dict | None = None, elbow=None) -> Trajectory:
    """Joint trajectory moving each tool's TCP from ``starts`` to ``goals`` (4x4 TCP poses in the base frame) in
    lockstep. ``current``: the arm joints now (the first step's seed); ``state``: every joint now (the chains need
    the joints between base and arm too, e.g. a torso lift); ``elbow``: a side the elbows swing toward on the way
    (chains only). Raises PathError."""
    chains = getattr(ik, 'chains', {})
    if state is not None and chains and all(tool in chains for tool in starts):
        return _chain_path(ik, starts, goals, current, state, speed, turn, step_m, step_rad, elbow)
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


def _chain_path(ik, starts: dict, goals: dict, current: dict, state: dict, speed: float, turn: float,
                step_m: float, step_rad: float, elbow=None) -> Trajectory:
    """linear_path on the URDF chains (see the module docstring)."""
    from .ik import IkError, LIMIT_MARGIN_DEG

    margin = math.radians(float((getattr(ik, 'choose', None) or {}).get('limit_margin_deg', LIMIT_MARGIN_DEG)))
    count = steps_needed(starts, goals, step_m, step_rad)
    positions = {**state, **current}
    names = sorted(current)
    times, points, elapsed = [0.0], [[current[n] for n in names]], 0.0
    previous = starts
    tcp_to_flange = {tool: np.linalg.inv(rpy_matrix(ik.tools[tool].get('xyz', [0, 0, 0]),
                                                    ik.tools[tool].get('rpy', [0, 0, 0]))) for tool in starts}
    for i in range(1, count + 1):
        step = {tool: interpolate(starts[tool], goals[tool], i / count) for tool in starts}
        for tool, tcp in step.items():
            chain, arm = ik.chains[tool]
            solved, why = chain_step(chain, arm, positions, tcp @ tcp_to_flange[tool], ik.limits, margin, elbow)
            if solved is None:
                raise PathError(f'no straight-line path for {tool}: step {i} of {count} '
                                f'(TCP {np.round(tcp[:3, 3], 3).tolist()}): {why}')
            positions.update(solved)
        moved = {name: positions[name] for name in names}
        try:
            free = ik.valid(moved, ik.groups[next(iter(step))])
        except IkError as error:
            raise PathError(str(error)) from error
        if not free:
            raise PathError(f'no straight-line path: step {i} of {count} is in collision')
        duration = max(max(np.linalg.norm(step[t][:3, 3] - previous[t][:3, 3]) / speed,
                           rotation_angle(previous[t][:3, :3].T @ step[t][:3, :3]) / turn) for t in step)
        elapsed += max(duration, 1e-3)
        times.append(elapsed)
        points.append([positions[n] for n in names])
        previous = step
    return Trajectory(names, times, points)


SWING_MAX_STEPS = 200


def elbow_swing_path(ik, tools: list[str], current: dict, state: dict, side, joint_speed: float):
    """Joint trajectory swinging the elbows of ``tools`` toward ``side`` (ik.preferred_elbow) along each arm's
    self-motion, the hands staying where they are (chains only). It stops when every elbow is there, or before a step
    that would put a joint at its limit (margin) or the robot in collision. Returns (Trajectory or None if nothing
    moved, {tool: (deviation before, after) in degrees}). Raises PathError without chains."""
    from .ik import IkError, LIMIT_MARGIN_DEG

    chains = getattr(ik, 'chains', {})
    if not all(tool in chains for tool in tools):
        raise PathError('swinging the elbow needs the URDF chains (IkClient.attach_chains)')
    margin = math.radians(float((getattr(ik, 'choose', None) or {}).get('limit_margin_deg', LIMIT_MARGIN_DEG)))
    positions = {**state, **current}
    names = sorted(current)
    flanges = {tool: chains[tool][0].fk(positions) for tool in tools}
    before = {tool: elbow_deviation_of(chains[tool][0], chains[tool][1], positions, side) for tool in tools}
    times, points, elapsed = [0.0], [[current[n] for n in names]], 0.0
    for _ in range(SWING_MAX_STEPS):
        trial, moving = dict(positions), False
        for tool in tools:
            chain, arm = chains[tool]
            if elbow_deviation_of(chain, arm, trial, side) < ELBOW_SETTLED_RAD:
                continue
            solved, _ = chain_step(chain, arm, trial, flanges[tool], {}, margin, side)
            if solved is None:
                continue
            edge = margin / 5   # closer than this to a limit, a joint may only move away from it
            near_limit = any((v < ik.limits[n][0] + edge and v < trial[n]) or (v > ik.limits[n][1] - edge and v > trial[n])
                             for n, v in solved.items() if n in ik.limits)
            if near_limit or elbow_deviation_of(chain, arm, {**trial, **solved}, side) > \
                    elbow_deviation_of(chain, arm, trial, side) - 1e-4:
                continue  # at a limit, or no longer getting closer: this elbow stays
            trial.update(solved)
            moving = True
        if not moving:
            break
        moved = {name: trial[name] for name in names}
        try:
            if not ik.valid(moved, ik.groups[tools[0]]):
                break
        except IkError as error:
            raise PathError(str(error)) from error
        step = max(abs(trial[n] - positions[n]) for n in names)
        positions = trial
        elapsed += max(step / joint_speed, 1e-3)
        times.append(elapsed)
        points.append([positions[n] for n in names])
    after = {tool: elbow_deviation_of(chains[tool][0], chains[tool][1], positions, side) for tool in tools}
    report = {tool: (round(math.degrees(before[tool])), round(math.degrees(after[tool]))) for tool in tools}
    return (Trajectory(names, times, points) if len(points) > 1 else None), report
