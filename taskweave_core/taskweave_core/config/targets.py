"""Targets: the one thing a robot moves to. A target says what it moves (``moves``: the base, a hand, a held object or
joints) and, from the file it is in, the frame it is given in. This module is the model: parsing and checking one
target, the per-key merge of an object type's layers, joint group values, the per-kind option table of the move skill
and the conversion between tool axes and roll-pitch-yaw. Reading the files is ConfigStore's (target_store.py).

Frames, by file: ``cell/targets.yaml`` the cell, a robot's ``targets.yaml`` the robot's base, the target tables of an
object type (``objects/<type>.yaml``, ``robots/<robot>/objects/<type>.yaml``) the object, joints none.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from .poses import ObjectInstance, Pose2D, Pose3D

MOVES = ('base', 'hand', 'object', 'joints')
FRAMES = ('cell', 'base', 'object', 'none')
TOOL_SUFFIX = '_tool'  # a hand target's tool pose key: <tool>_tool
BASE_KEYS = ('x', 'y', 'yaw')
OBJECT_KEYS = ('xyz', 'rpy')
HAND_ATTRIBUTES = ('alone', 'tool_type', 'grip_width_m', 'approach_m', 'elbow', 'grip_m')
OBJECT_ATTRIBUTES = ('type', 'reference')
ROBOT_SETTINGS = ('elbow', 'grip_m')  # what a robot adds to a target of an object type (the rest is the object's)
MERGE_PER_KEY = ('grip_m', 'alone', 'joints')  # merged per hand or per group; every other key replaces as a whole
TORSO = 'torso'
TORSO_WORDS = ('auto', 'keep')
FRAME_OF_FILE = {'cell': 'cell', 'base': 'base', 'object': 'object', 'joints': 'none'}
MOVES_WORD = {'base': 'the base', 'hand': 'a hand', 'object': 'a held object', 'joints': 'joints'}
OLD_PREFIXES = {'stance:': 'stance', 'location:': 'location', 'pose:': 'pose'}  # removed in cell schema 4


class TargetError(KeyError):
    """A target cannot be found or used. KeyError for the callers of the old readers; the message is one line that
    names the file to look at (str() is the message, not its repr)."""

    def __str__(self) -> str:
        return str(self.args[0]) if self.args else ''


class TargetWriteError(ValueError):
    """A target cannot be written (saved, deleted or renamed): the data does not fit its kind or file, the name is
    taken or in two places, or other targets depend on it. ValueError (the old writers raised ValueError); a target
    that cannot be found is a TargetError."""


def is_tool_key(key: str) -> bool:
    return key.endswith(TOOL_SUFFIX) and len(key) > len(TOOL_SUFFIX)


def tool_keys(data: dict) -> list[str]:
    return [key for key in data if is_tool_key(key)]


# ---- one target ---------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Target:
    """A resolved target. ``data`` is the merged entry as written (plain data); the typed parts are parsed from it.

    ``poses``: base targets, the poses to drive to in the cell frame (an object-frame target composed with the
    object's pose; a base-frame one stays relative to the base, frame 'base'); hand targets, one ``{<tool>_tool:
    Pose3D}`` per leg; object targets, one Pose3D per waypoint. The last one is the target, ``pose`` / ``tools``.
    ``instance``: the placed object as handled (stacks: its top item, ``instance.top_z``), for object-frame targets.
    ``files``: the files that gave it, ``joints`` the group values as written (a number, a list, 'auto' or 'keep')."""

    kind: str
    frame: str
    name: str
    reference: str
    data: dict
    files: tuple = ()
    type: str | None = None
    instance: ObjectInstance | None = None
    poses: tuple = ()
    alone: dict = field(default_factory=dict)
    joints: dict = field(default_factory=dict)

    @property
    def moves(self) -> str:
        return self.kind

    @property
    def pose(self):
        return self.poses[-1] if self.poses else None

    @property
    def tools(self) -> dict[str, Pose3D]:
        """The last leg of a hand target: ``{<tool>_tool: Pose3D}``."""
        return self.poses[-1] if self.kind == 'hand' and self.poses else {}

    @property
    def tool_poses(self) -> dict[str, Pose3D]:
        """The same by tool name (``left``, not ``left_tool``)."""
        return {key[: -len(TOOL_SUFFIX)]: pose for key, pose in self.tools.items()}

    @property
    def is_path(self) -> bool:
        return 'path' in self.data

    @property
    def held_type(self) -> str | None:
        """An object target's ``type``: it is for a held object of that type (None: any)."""
        return self.data.get('type')

    @property
    def reference_target(self) -> str | None:
        """An object target's ``reference``: the target of the held object's type that goes here."""
        return self.data.get('reference')

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)

    def torso(self) -> Any:
        """The torso entry of the joints part: a value, 'auto', 'keep', or None when it names none."""
        return self.joints.get(TORSO)

    def needs_held(self, held_type: str | None) -> str | None:
        """Why this object target cannot be used for what is held (``held_type``: its object type, None: nothing)
        or None. A target that names a ``type`` is for a held object of that type."""
        if self.kind != 'object' or not self.data.get('type') or self.data['type'] == held_type:
            return None
        what = f'a held {self.data["type"]}' if self.data['type'] else 'a held object'
        return f'{self.name!r} is a {self.frame}-frame object target for {self.data["type"]}: {what} is needed, ' + (
            'nothing is held' if held_type is None else f'a {held_type} is held'
        )

    def to_point(self) -> dict:
        """The shape the old object point readers return: ``{<tool>_tool: Pose3D, alone: {...} | None,
        tool_type, grip_width_m, approach_m, elbow, approach}`` (hand targets)."""
        result: dict[str, Any] = {
            key: self.data.get(key) for key in ('approach', 'tool_type', 'grip_width_m', 'approach_m', 'elbow')
        }
        result.update(self.tools)
        result['alone'] = dict(self.alone) or None
        if 'grip_m' in self.data:
            result['grip_m'] = self.data['grip_m']
        return result


def _pose3d(entry: dict, frame: str) -> Pose3D:
    return Pose3D(tuple(float(v) for v in entry['xyz']), tuple(float(v) for v in entry['rpy']), frame)


def _tool_leg(entry: dict, frame: str) -> dict[str, Pose3D]:
    return {key: _pose3d(entry[key], frame) for key in tool_keys(entry)}


def build_target(
    kind: str,
    frame: str,
    name: str,
    reference: str,
    data: dict,
    files=(),
    *,
    type: str | None = None,
    instance: ObjectInstance | None = None,
) -> Target:
    """A Target from checked merged ``data``. An object-frame base target is composed with ``instance``'s pose;
    hand and object poses keep the object's (or the base's) frame, ``type`` names the pose frame of the former."""
    where = type if frame == 'object' else frame
    poses: tuple = ()
    alone: dict = {}
    if kind == 'base':
        steps = data.get('path') or [data]
        poses = tuple(_pose2d(step, instance, type, frame) for step in steps)
    elif kind == 'hand':
        poses = tuple(_tool_leg(step, where) for step in (data.get('path') or [data]))
        alone = {key: _pose3d(pose, where) for key, pose in data.get('alone', {}).items()}
    elif kind == 'object':
        poses = tuple(_pose3d(step, where) for step in (data.get('path') or [data]))
    return Target(
        kind, frame, name, reference, data, tuple(files), type, instance, poses, alone, dict(data.get('joints') or {})
    )


def _pose2d(step: dict, instance: ObjectInstance | None, type: str | None, frame: str) -> Pose2D:
    offset = Pose2D(float(step['x']), float(step['y']), float(step['yaw']), type if frame == 'object' else frame)
    return instance.pose.compose(offset) if frame == 'object' and instance is not None else offset


# ---- checking one target ------------------------------------------------------------------------------------------
_FORBIDDEN = {  # keys a kind of target must not have
    'base': set(OBJECT_KEYS) | set(HAND_ATTRIBUTES) | set(OBJECT_ATTRIBUTES),
    'hand': set(BASE_KEYS) | set(OBJECT_KEYS) | set(OBJECT_ATTRIBUTES),
    'object': set(BASE_KEYS) | set(HAND_ATTRIBUTES),
    'joints': set(BASE_KEYS) | set(OBJECT_KEYS) | set(HAND_ATTRIBUTES) | set(OBJECT_ATTRIBUTES) | {'path'},
}


def spatial_keys(data: dict) -> list[str]:
    """The keys of ``data`` that make the single pose (x/y/yaw, xyz/rpy, tool poses; ``alone`` is not one)."""
    return [key for key in data if key in BASE_KEYS or key in OBJECT_KEYS or is_tool_key(key)]


def target_problems(data: dict, frame: str | None = None) -> list[str]:
    """What is wrong with one complete target (``moves`` given), as short sentences; [] if it is fine. ``frame``:
    the frame of the file it is in; an explicit ``frame`` key must say the same."""
    moves = data.get('moves')
    if moves not in MOVES:
        return [f'moves must be one of {", ".join(MOVES)}']
    problems = []
    implied = 'none' if moves == 'joints' else frame
    if implied is not None and 'frame' in data and data['frame'] != implied:
        problems.append(
            f'frame {data["frame"]!r} does not fit this file (its {moves} targets are in the {implied} frame)'
        )
    for key in sorted(set(data) & _FORBIDDEN[moves]):
        problems.append(f'{key} does not apply to a target that moves {MOVES_WORD[moves]}')
    for key in tool_keys(data):
        if moves != 'hand':
            problems.append(f'{key} does not apply to a target that moves {MOVES_WORD[moves]}')
    if moves == 'joints':
        if not data.get('joints'):
            problems.append('a joints target needs joints')
        return problems + joints_problems(data.get('joints') or {}, spatial=False)
    has_path, single = 'path' in data, spatial_keys(data)
    if has_path and single:
        problems.append(f'path and a single pose ({", ".join(single)}) together: use one of them')
    steps = data.get('path') if has_path else [data]
    if has_path and not steps:
        problems.append('path has no waypoints')
    for number, step in enumerate(steps or [], start=1):
        label = f'path[{number}]' if has_path else 'pose'
        problems += _pose_problems(moves, step, label)
    if moves == 'hand' and not has_path and not tool_keys(data) and not data.get('alone'):
        problems.append('a hand target needs a tool pose (<tool>_tool), alone or path')
    return problems + joints_problems(data.get('joints') or {}, spatial=True)


def _pose_problems(moves: str, step: Any, label: str) -> list[str]:
    if not isinstance(step, dict):
        return [f'{label} must be a mapping']
    if moves == 'base':
        wanted = [key for key in BASE_KEYS if key not in step]
        extra = [key for key in step if key not in BASE_KEYS and (label != 'pose')]
        return ([f'{label} needs {", ".join(wanted)}'] if wanted else []) + [
            f'{label}: {key} does not apply to a base waypoint' for key in extra
        ]
    if moves == 'object':
        wanted = [key for key in OBJECT_KEYS if key not in step]
        extra = [key for key in step if key not in OBJECT_KEYS and label != 'pose']
        return ([f'{label} needs {", ".join(wanted)}'] if wanted else []) + [
            f'{label}: {key} does not apply to an object waypoint' for key in extra
        ]
    if label == 'pose':  # a hand target's own poses are checked by their schema
        return []
    if not tool_keys(step):
        return [f'{label} needs a tool pose (<tool>_tool)']
    return [f'{label}: {key} does not apply to a hand waypoint' for key in step if not is_tool_key(key)]


def joints_problems(joints: dict, spatial: bool) -> list[str]:
    """Group-independent problems of a ``joints:`` part: ``auto`` and ``keep`` are for the torso only, and ``auto``
    needs a hand or object pose to search for."""
    problems = []
    for group, value in joints.items():
        if isinstance(value, str) and value not in TORSO_WORDS:
            problems.append(f'joints.{group}: {value!r} is not a number, a list or one of {", ".join(TORSO_WORDS)}')
        elif isinstance(value, str) and group != TORSO:
            problems.append(f'joints.{group}: {value} is for the torso only')
        elif value == 'auto' and not spatial:
            problems.append('joints.torso: auto needs a hand or object pose to search a height for')
    return problems


# ---- merging an object type's layers ------------------------------------------------------------------------------
def merge_layers(name: str, layers: list[tuple[str, dict]]) -> dict:
    """The per-key merge of the layers of one object-frame target, later wins (``layers``: ``[(file label, entry)]``,
    weakest first). ``left_tool`` / ``right_tool`` / ``x`` / ``y`` / ``yaw`` / ``xyz`` / ``rpy`` and any other key
    replace as a whole; ``grip_m``, ``alone`` and ``joints`` merge per hand or per group. A layer that gives a
    ``path`` replaces a single pose of the layers before it, and the other way round. ``moves`` must agree where
    several layers give it."""
    merged: dict[str, Any] = {}
    moves_from: tuple[str, str] | None = None
    for label, entry in layers:
        if 'moves' in entry:
            if moves_from and entry['moves'] != moves_from[1]:
                raise TargetError(
                    f'target {name!r} moves {moves_from[1]} in {moves_from[0]} but {entry["moves"]} in '
                    f'{label}: they must agree (rename one of them)'
                )
            moves_from = moves_from or (label, entry['moves'])
        if 'path' in entry:
            merged = {k: v for k, v in merged.items() if k not in spatial_keys(merged)}
        elif spatial_keys(entry):
            merged.pop('path', None)
        for key, value in entry.items():
            if key in MERGE_PER_KEY and isinstance(value, dict) and isinstance(merged.get(key), dict):
                merged[key] = {**merged[key], **value}
            else:
                merged[key] = value
    return merged


# ---- joint groups -------------------------------------------------------------------------------------------------
class GroupError(ValueError):
    """A joint group value that does not fit the robot's groups."""


def group_joint_values(
    joints: dict, groups: dict[str, list[str]], torso_joint: str | None = None, only: list[str] | None = None
) -> dict[str, float]:
    """The joint values of a ``joints:`` part: ``{joint: value}`` for every joint it sets. A group value is a number
    (a group of one joint, or the torso's lift joint ``torso_joint``), or a list in the group's joint order where
    ``null`` leaves that joint where it is. ``auto`` and ``keep`` set no joint (torso_setting). ``only``: just these
    groups. Raises GroupError naming the group."""
    result: dict[str, float] = {}
    for group, value in joints.items():
        if only is not None and group not in only:
            continue
        if group not in groups:
            known = ', '.join(sorted(groups)) or 'none'
            raise GroupError(f'joint group {group!r}: this robot has no such group (it has: {known})')
        members = groups[group]
        if isinstance(value, str):
            if group != TORSO or value not in TORSO_WORDS:
                raise GroupError(f'joint group {group!r}: {value!r} is not a number or a list')
            continue
        if isinstance(value, bool):
            raise GroupError(f'joint group {group!r}: {value!r} is not a number or a list')
        if isinstance(value, int | float):
            if len(members) == 1:
                result[members[0]] = float(value)
            elif group == TORSO and torso_joint in members:
                result[torso_joint] = float(value)
            else:
                raise GroupError(
                    f'joint group {group!r} has {len(members)} joints ({", ".join(members)}): give a '
                    f'list of {len(members)} values (null leaves a joint where it is)'
                )
            continue
        if not isinstance(value, list | tuple) or len(value) != len(members):
            raise GroupError(
                f'joint group {group!r} has {len(members)} joints ({", ".join(members)}): give a '
                f'list of {len(members)} values, not {value!r}'
            )
        for joint, each in zip(members, value, strict=True):
            if each is None:
                continue
            if isinstance(each, bool) or not isinstance(each, int | float):
                raise GroupError(f'joint group {group!r}: {each!r} is not a number or null')
            result[joint] = float(each)
    return result


def torso_setting(joints: dict) -> str | float | list | None:
    """What the joints part says about the torso: ``auto``, ``keep``, a number or a list; None if it names none."""
    return joints.get(TORSO)


# ---- tool axes and roll-pitch-yaw ---------------------------------------------------------------------------------
GIMBAL = 1e-9  # 1 - |sin pitch| below this: roll and yaw are not unique, roll is fixed at 0


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _unit(v):
    norm = math.sqrt(sum(c * c for c in v))
    if norm == 0:
        raise ValueError('a zero vector has no direction')
    return tuple(c / norm for c in v)


def rpy_from_axes(approach, closing) -> tuple[float, float, float]:
    """Roll-pitch-yaw (R = Rz(yaw) Ry(pitch) Rx(roll)) of the tool frame with z along ``approach`` and y along
    ``closing`` (x = y cross z). At pitch +-90 deg roll is 0. ``closing`` is made square to ``approach``."""
    z = _unit(approach)
    y = _unit(closing)
    y = _unit(tuple(c - sum(a * b for a, b in zip(y, z, strict=True)) * zc for c, zc in zip(y, z, strict=True)))
    x = _cross(y, z)
    sin_pitch = -x[2]
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    if 1.0 - abs(sin_pitch) < GIMBAL:
        return (0.0, pitch, math.atan2(-y[0], y[1]))
    return (math.atan2(y[2], z[2]), pitch, math.atan2(x[1], x[0]))


def axes_from_rpy(rpy) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """``(approach, closing)``: the z and y axes of the frame with these roll-pitch-yaw."""
    (cr, cp, cy), (sr, sp, sy) = ((math.cos(a) for a in rpy), (math.sin(a) for a in rpy))
    approach = (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr)
    closing = (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr)
    return approach, closing


# ---- the move skill's options per kind of motion ------------------------------------------------------------------
MOVE_FORMS = ('base', 'straight', 'hand', 'object', 'elbow', 'joints')  # what a move step resolves to
ALL_FORMS = MOVE_FORMS
MOVE_OPTIONS: dict[str, tuple[str, ...]] = {  # option -> the forms it applies to (an error on the others)
    'hands': ('hand', 'object', 'elbow'),
    'offset_m': ('hand', 'object'),
    'reference': ('object',),
    'only': ('joints',),
    'torso': ('base', 'straight', 'hand', 'object', 'elbow'),
    'tolerance_deg': ('elbow',),
    'motion': ALL_FORMS,
}
MOTION_OPTIONS: dict[str, tuple[str, ...]] = {  # motion.<key> -> the forms it applies to
    'mode': ('base', 'straight', 'hand', 'object'),
    'speed': ALL_FORMS,
    'speed_abs': ALL_FORMS,
    'accel': ALL_FORMS,
    'planner': ('hand', 'object', 'joints'),
    'via': ('hand', 'joints'),
    'tolerance': ('hand', 'object', 'elbow', 'joints'),
    'elbow': ('hand', 'object'),
    'approach_m': ('hand',),
}
MOTION_MODES = {'base': ('nav',), 'straight': ('straight',), 'hand': ('free', 'linear'), 'object': ('linear', 'free')}
TORSO_VALUES = {
    'base': ('keep', 'number'),
    'straight': ('keep', 'number'),
    'hand': ('auto', 'keep', 'number'),
    'object': ('auto', 'keep', 'number'),
    'elbow': ('keep',),
}


def move_option_problems(form: str, options: dict) -> list[str]:
    """What is wrong with the options of a move step that resolves to ``form`` (one of MOVE_FORMS): an option that
    does not apply to it, a ``motion`` key or ``mode`` or ``torso`` value it does not take."""
    problems = []
    for option, value in options.items():
        forms = MOVE_OPTIONS.get(option)
        if forms is not None and form not in forms:
            problems.append(f'move {form}: option {option} does not apply')
            continue
        if option == 'motion' and isinstance(value, dict):
            for key, each in value.items():
                if key in MOTION_OPTIONS and form not in MOTION_OPTIONS[key]:
                    problems.append(f'move {form}: motion.{key} does not apply')
                elif key == 'mode' and each not in MOTION_MODES.get(form, ()):
                    problems.append(
                        f'move {form}: motion.mode {each!r} does not apply (it takes '
                        f'{", ".join(MOTION_MODES.get(form, ()))})'
                    )
        elif option == 'torso':
            word = value if isinstance(value, str) else 'number'
            if word not in TORSO_VALUES.get(form, ()):
                problems.append(
                    f'move {form}: torso {value!r} does not apply (it takes {", ".join(TORSO_VALUES.get(form, ()))})'
                )
    return problems
