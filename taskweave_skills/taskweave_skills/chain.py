"""A kinematic chain from the robot's URDF: forward kinematics and the Jacobian, for velocity-level arm control.

tcp_jog uses it to jog a tool and to swing an elbow smoothly: one damped least-squares step per tick instead of an
IK search per tick, so the arm slows down near a singularity instead of stopping, holds a joint at its limit while the
others carry on, and swings the elbow in the arm's null space (the hand stays where it is).

    chain = Chain.from_urdf(path, 'base_link', 'left_link7')
    flange = chain.fk(positions)                    # 4x4 in the base frame; every chain joint from ``positions``
    step = chain.solve_step(positions, target, ['left_joint1', ...], limits)
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ElementTree
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from taskweave_core.config.tuning import motion_tuning

from .ik import rpy_matrix


@dataclass
class Joint:
    name: str
    kind: str              # revolute, continuous, prismatic or fixed
    origin: np.ndarray     # 4x4 parent -> joint frame at zero
    axis: np.ndarray       # unit axis in the joint frame
    child: str = ''        # the link it moves


def _transform(kind: str, axis: np.ndarray, value: float) -> np.ndarray:
    pose = np.eye(4)
    if kind == 'prismatic':
        pose[:3, 3] = axis * value
    elif kind in ('revolute', 'continuous'):
        k = axis
        kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        pose[:3, :3] = np.eye(3) + math.sin(value) * kx + (1 - math.cos(value)) * kx @ kx
    return pose


def pose_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """6-vector (position, rotation vector) that moves ``current`` onto ``target``, both 4x4 in the same frame."""
    rotation = target[:3, :3] @ current[:3, :3].T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1) / 2)))
    if angle < 1e-9:
        omega = np.zeros(3)
    else:
        vector = np.array([rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0],
                           rotation[1, 0] - rotation[0, 1]])
        omega = vector / (2 * math.sin(angle)) * angle if angle < math.pi - 1e-6 else _axis_at_pi(rotation) * angle
    return np.concatenate([target[:3, 3] - current[:3, 3], omega])


def _axis_at_pi(rotation: np.ndarray) -> np.ndarray:
    values, vectors = np.linalg.eigh((rotation + rotation.T) / 2)
    return vectors[:, int(np.argmax(values))]


class Chain:
    """``tuning``: the step's motion_tuning.chain values (default: the schema's); ``elbow``: the links at the arm's
    shoulder, elbow and wrist centres (profile arms.<tool>.elbow), None for an arm without elbow control."""

    def __init__(self, joints: list[Joint], tuning: dict | None = None, elbow: tuple[str, str, str] | None = None):
        self.joints = joints
        self.movable = [j.name for j in joints if j.kind != 'fixed']
        self.links = [j.child for j in joints]
        self.tuning = tuning or motion_tuning()['chain']
        self.elbow = tuple(elbow) if elbow else None

    @classmethod
    def from_urdf(cls, path: Path | str, base: str, tip: str, tuning: dict | None = None,
                  elbow: tuple[str, str, str] | None = None) -> Chain:
        root = ElementTree.parse(path).getroot()
        by_child = {j.find('child').get('link'): j for j in root.iter('joint')}
        joints, link = [], tip
        while link != base:
            element = by_child.get(link)
            if element is None:
                raise ValueError(f'{path}: no chain from {base} to {tip} (stuck at {link})')
            origin = element.find('origin')
            xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis_element = element.find('axis')
            axis_text = axis_element.get('xyz') if axis_element is not None else '1 0 0'
            axis = np.array([float(v) for v in axis_text.split()])
            norm = np.linalg.norm(axis)
            joints.append(Joint(element.get('name'), element.get('type'), rpy_matrix(xyz, rpy),
                                axis / norm if norm > 0 else axis, link))
            link = element.find('parent').get('link')
        return cls(joints[::-1], tuning, elbow)

    def frames(self, positions: dict[str, float]) -> tuple[list[np.ndarray], np.ndarray]:
        """Each movable joint's frame (4x4, base frame, after its origin) and the tip pose."""
        pose, frames = np.eye(4), []
        for joint in self.joints:
            pose = pose @ joint.origin
            if joint.kind != 'fixed':
                frames.append(pose)
                pose = pose @ _transform(joint.kind, joint.axis, float(positions.get(joint.name, 0.0)))
        return frames, pose

    def fk(self, positions: dict[str, float]) -> np.ndarray:
        return self.frames(positions)[1]

    def joint_motion(self, positions: dict[str, float], joint: str, new: float) -> np.ndarray | None:
        """4x4 (base frame) of what moving ``joint`` from its value in ``positions`` to ``new`` does to everything it
        carries (a lift: a shift along its axis); None if the joint is not on the chain."""
        movable = [j for j in self.joints if j.kind != 'fixed']
        found = next((i for i, j in enumerate(movable) if j.name == joint), None)
        if found is None:
            return None
        frame, one = self.frames(positions)[0][found], movable[found]
        step = _transform(one.kind, one.axis, new) @ np.linalg.inv(_transform(one.kind, one.axis,
                                                                              float(positions.get(joint, 0.0))))
        return frame @ step @ np.linalg.inv(frame)

    def link_points(self, positions: dict[str, float], links) -> list[np.ndarray]:
        """Origins (base frame) of the named links of the chain; KeyError for a link not on it."""
        pose, found = np.eye(4), {}
        for joint in self.joints:
            pose = pose @ joint.origin
            if joint.kind != 'fixed':
                pose = pose @ _transform(joint.kind, joint.axis, float(positions.get(joint.name, 0.0)))
            found[joint.child] = pose[:3, 3]
        return [found[link] for link in links]

    def jacobian(self, positions: dict[str, float], names: list[str]) -> tuple[np.ndarray, np.ndarray]:
        """Geometric Jacobian (6 x len(names): linear velocity of the tip origin, angular velocity; base frame) of
        the named joints, and the tip pose. Chain joints not named are held."""
        frames, tip = self.frames(positions)
        movable = [j for j in self.joints if j.kind != 'fixed']
        columns = {}
        for joint, frame in zip(movable, frames, strict=True):
            axis = frame[:3, :3] @ joint.axis
            if joint.kind == 'prismatic':
                columns[joint.name] = np.concatenate([axis, np.zeros(3)])
            else:
                columns[joint.name] = np.concatenate([np.cross(axis, tip[:3, 3] - frame[:3, 3]), axis])
        return np.column_stack([columns[name] for name in names]), tip

    def solve_step(self, positions: dict[str, float], target: np.ndarray, names: list[str],
                   limits: dict[str, tuple[float, float]], null: np.ndarray | None = None) -> tuple[dict, str]:
        """One damped least-squares step of the named joints toward ``target`` (tip, 4x4), plus ``null`` (a joint
        step projected so the tip stays). Joints that would pass a limit are held there and the rest re-solved.
        Returns the new joint values and '' or the joints held at a limit (a reason to show)."""
        max_step, damping_max = self.tuning['max_step_rad'], self.tuning['damping']
        singular, margin = self.tuning['singular'], self.tuning['limit_margin']
        q = np.array([float(positions[name]) for name in names])
        free = np.ones(len(names), bool)
        held = []
        for _ in range(len(names)):
            jacobian, tip = self.jacobian(positions, names)
            jacobian = jacobian * free
            error = pose_error(target, tip)
            u, sigma, vt = np.linalg.svd(jacobian, full_matrices=False)
            smallest = sigma[-1] if len(sigma) else 0.0
            damping = 0.0 if smallest >= singular else (1 - (smallest / singular) ** 2) * damping_max ** 2
            pinv = vt.T @ np.diag(sigma / (sigma ** 2 + damping)) @ u.T
            step = pinv @ error
            if null is not None:
                step += (np.eye(len(names)) - pinv @ jacobian) @ (null * free)
            step *= free
            largest = float(np.max(np.abs(step))) if len(step) else 0.0
            if largest > max_step:
                step *= max_step / largest
            new = q + step
            over = [i for i, name in enumerate(names) if free[i] and name in limits and not
                    limits[name][0] + margin <= new[i] <= limits[name][1] - margin]
            if not over:
                return {name: float(v) for name, v in zip(names, new, strict=True)}, ', '.join(held)
            for i in over:  # hold it where it is (or just inside its range) and let the others do the work
                lower, upper = limits[names[i]]
                q[i] = min(max(q[i], lower + margin), upper - margin)
                free[i] = False
                held.append(names[i])
            positions = {**positions, **{name: float(v) for name, v in zip(names, q, strict=True)}}
        return {name: float(v) for name, v in zip(names, q, strict=True)}, ', '.join(held)

    def null_direction(self, positions: dict[str, float], names: list[str]) -> np.ndarray:
        """Unit joint step that moves the elbow but not the tip (the arm's self-motion; a 7-joint arm has one)."""
        jacobian, _ = self.jacobian(positions, names)
        return np.linalg.svd(jacobian)[2][-1]
