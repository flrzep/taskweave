"""Arm positions for the HMI: one list for everywhere the arms are taught to go. No ROS dependency.

A position is relative to the robot (a named pose in ``robots/<robot>/poses.yaml``: hand targets in the base frame
solved by IK, or joint values) or to an object type (a point in ``objects/<type>.yaml``: hand targets in the object's
frame, e.g. a grasp). The files and the skills that use them (move_joints, move_gripper) stay as they are; this module
shows both the same way: each hand as x y z (m) and roll pitch yaw (rad) of its canonical TCP (z = approach,
y = closing), plus joints (robot positions), elbow side and straight approach (object positions).

Scopes in addresses: ``robot`` or an object type name.

On a cell pack in the targets layout (cell schema 4) a robot position is a joint or hand target in
``robots/<robot>/targets.yaml`` (robot.py) and an object position a hand target of the type (points.py); renames and
deletes go through the store's rename_target / delete_target. The editor's shape stays.
"""

from __future__ import annotations

import math
from collections.abc import Mapping

import numpy as np

from taskweave_core.config import ConfigError, ConfigStore, schemas
from taskweave_core.config.limits import joint_limits, mimic_joints, pose_problems
from taskweave_skills.ik import rpy_matrix, tcp_matrix

from . import held, points, robot
from . import targets as layout

ROBOT = 'robot'


class ArmError(ValueError):
    """An arm position cannot be changed; the message says why."""


def hand_from_target(target: Mapping) -> dict:
    """{tcp, approach, closing} (a named pose's tool target) -> {xyz, rpy}."""
    m = tcp_matrix(target)
    rpy = [math.atan2(m[2, 1], m[2, 2]), math.asin(max(-1.0, min(1.0, -m[2, 0]))), math.atan2(m[1, 0], m[0, 0])]
    return {'xyz': [round(float(v), 4) for v in m[:3, 3]], 'rpy': [round(v, 4) + 0.0 for v in rpy]}


def target_from_hand(hand: Mapping) -> dict:
    """{xyz, rpy} -> {tcp, approach, closing}."""
    m = rpy_matrix(np.asarray(hand['xyz'], float), np.asarray(hand['rpy'], float))
    return {'tcp': [round(float(v), 4) for v in m[:3, 3]], 'approach': [round(float(v), 4) for v in m[:3, 2]],
            'closing': [round(float(v), 4) for v in m[:3, 1]]}


def object_types(store: ConfigStore) -> list[str]:
    return sorted(path.stem for path in (store.root / 'objects').glob('*.yaml'))


def overview(store: ConfigStore, robot_name: str, pose_args: set) -> dict:
    """Every arm position, with the missions using it, and what the editor needs to show them."""
    store.reload()
    profile = store.robot(robot_name)
    pose_used = robot.pose_usage(store, pose_args)
    point_used = points.usage(store, robot_name)
    positions = []
    if layout.active(store):   # hand targets keep their xyz / rpy as written
        for name, pose in layout.robot_poses(store, robot_name).items():
            positions.append({
                'scope': ROBOT, 'name': name, 'joints': pose['joints'], 'elbow': pose['elbow'], 'approach_m': None,
                'hands': pose['hands'], 'used_by': pose_used.get(name, []), 'problems': pose['problems']})
    else:
        for name in store.joint_poses(robot_name):
            pose = store.named_pose(robot_name, name)
            positions.append({
                'scope': ROBOT, 'name': name, 'joints': pose['joints'], 'elbow': pose['elbow'], 'approach_m': None,
                'hands': {tool: hand_from_target(target) for tool, target in pose['tools'].items()},
                'used_by': pose_used.get(name, []), 'problems': pose_problems(store, robot_name, name)})
    types = object_types(store)
    for object_type in types:
        for point in points.object_points(store, object_type, point_used, robot_name):
            positions.append({
                'scope': object_type, 'name': point['name'], 'joints': {}, 'elbow': point['elbow'],
                'approach_m': point['approach_m'], 'grip_m': point['grip_m'], 'hands': point['tools'],
                'used_by': point['used_by'], 'problems': []})
    instances = store.object_instances()
    mimics = mimic_joints(store, robot_name)
    return {
        'robot': robot_name,
        'targets_layout': layout.active(store),   # cell schema 4: points are hand targets, held poses object targets
        'positions': positions,
        'held': held.overview(store, robot_name),   # held poses (move_object pose:), edited as numbers
        'grip_range_m': grip_ranges(store, robot_name),   # finger openings each gripper can be set to (grip_m)
        'object_types': types,
        'placed': {t: sorted(n for n, i in instances.items() if i.object == t) for t in types},
        'tools': list(profile.data.get('ik', {}).get('groups', {})),   # hands with IK
        'linear_joints': list(profile.data.get('joints', {}).get('linear', [])),
        'joint_limits': {joint: [None if math.isinf(v) else v for v in bounds]   # mimic joints left out: not settable
                         for joint, bounds in joint_limits(store, robot_name).items() if joint not in mimics},
        'elbow_sides': schemas.ELBOW_SIDES,
        'approach_default': layout.approach_default(store),
        'files': {'poses': store.display_path(store.robot_save_path(robot_name, f'{robot.poses_file(store)}.yaml')),
                  'objects': 'objects/<type>.yaml'},
    }


def _hands(body: Mapping) -> dict:
    hands = {}
    for tool, hand in (body.get('hands') or {}).items():
        try:
            hands[tool] = {key: [float(v) for v in hand[key]] for key in ('xyz', 'rpy')}
        except (KeyError, TypeError, ValueError):
            raise ArmError(f'{tool}: x y z and roll pitch yaw need three numbers each') from None
        if any(len(v) != 3 or not all(math.isfinite(x) for x in v) for v in hands[tool].values()):
            raise ArmError(f'{tool}: x y z and roll pitch yaw need three numbers each')
    return hands


def save(store: ConfigStore, robot_name: str, scope: str, name: str, body: Mapping, pose_args: set) -> str:
    """Update a position from the editor's numbers: ``hands`` {tool: {xyz, rpy}}, ``joints`` (robot), ``elbow``,
    ``approach_m`` (object), optional ``rename``. New positions are made by capturing the robot (capture)."""
    store.reload()
    hands = _hands(body)
    rename = body.get('rename') or None
    if rename == name:
        rename = None
    if scope == ROBOT:
        request = {'joints': body.get('joints') or {}, 'rename': rename}
        if hands:
            request.update(tools={tool: target_from_hand(hand) for tool, hand in hands.items()},
                           elbow=body.get('elbow') or None)
            if layout.active(store):   # written as xyz / rpy: the editor's numbers as they are
                request['hands'] = hands
        try:
            return robot.save_pose(store, robot_name, name, request, pose_args)
        except robot.RobotSettingsError as error:
            raise ArmError(str(error)) from error
    _object_type(store, scope)
    settings = {key: body[key] for key in ('approach_m', 'elbow', 'grip_m') if key in body}
    try:
        file = points.save_object_point(store, scope, name, {**settings, 'tools': hands}, robot_name)
    except points.PointError as error:
        raise ArmError(str(error)) from error
    if rename:
        users = points.usage(store, robot_name).get(('object_point', scope, name), [])
        if users:
            raise ArmError(f'{scope} {name} is used by {", ".join(users)}; change those missions first '
                           '(the numbers were saved)')
        if layout.active(store):
            return _target_change(
                store, robot_name, lambda target: store.rename_target(target, rename, robot_name, type=scope), scope,
                name)
        try:
            file = str(store.rename_object_point(scope, name, rename).relative_to(store.root))
        except ValueError as error:
            raise ArmError(str(error)) from error
    return file


def _target_change(store: ConfigStore, robot_name: str, change, object_type: str, name: str) -> str:
    """Rename or delete (``change(name)``, the store's call with ``type=object_type``: nothing needs to be placed)
    the hand target ``name`` of a type on a targets cell; the first file written."""
    if layout.type_kinds(store, robot_name, object_type).get(name) != 'hand':
        raise ArmError(f'object {object_type!r} has no point {name!r}')
    try:
        paths = change(name)
    except layout.REFUSED as error:
        raise ArmError(str(error)) from error
    return paths[0].relative_to(store.root).as_posix() if paths else ''


def delete(store: ConfigStore, robot_name: str, scope: str, name: str, pose_args: set) -> str:
    store.reload()
    if scope == ROBOT:
        try:
            return robot.delete_pose(store, robot_name, name, pose_args)
        except robot.RobotSettingsError as error:
            raise ArmError(str(error)) from error
    _object_type(store, scope)
    users = points.usage(store, robot_name).get(('object_point', scope, name), [])
    if users:
        raise ArmError(f'{scope} {name} is used by {", ".join(users)}; change those missions first')
    if layout.active(store):
        return _target_change(store, robot_name, lambda target: store.delete_target(target, robot_name, type=scope),
                              scope, name)
    try:
        return str(store.delete_point(f'objects/{scope}.yaml', ['points', name]).relative_to(store.root))
    except ValueError as error:
        raise ArmError(str(error)) from error


def _object_type(store: ConfigStore, scope: str) -> None:
    try:
        store.object(scope)
    except ConfigError as error:
        raise ArmError(f'no object type {scope!r} (objects/{scope}.yaml)') from error


def capture_pose(store: ConfigStore, robot_name: str, name: str, body: Mapping, joints: dict[str, float],
                 targets: dict | None, pose_args: set) -> str:
    """Save the robot's current arm as the robot position ``name``. ``joints``: current joint states; ``targets``:
    current tool targets (forward kinematics), None if unavailable. An existing position keeps its form (hands or
    joints) and its joint set; ``body.tools`` limits a hand capture to those hands (the others are kept). A new one
    takes ``body.form`` ('hands' or 'joints')."""
    store.reload()
    if layout.active(store):   # read as the Robot tab lists them (a target that cannot be read does not stop it)
        named = layout.robot_poses(store, robot_name)
        poses, pose_of = list(named), named.__getitem__
    else:
        poses, pose_of = store.joint_poses(robot_name), lambda n: store.named_pose(robot_name, n)
    existing = pose_of(name) if name in poses else None
    profile_tools = list(store.robot(robot_name).data.get('ik', {}).get('groups', {}))
    if existing is not None:
        hands = bool(existing['tools'])
        keys = list(existing['joints'])
    else:
        hands = body.get('form', 'hands' if profile_tools else 'joints') == 'hands'
        others = [pose_of(n) for n in poses]
        keys = list(dict.fromkeys(j for p in others if bool(p['tools']) == hands for j in p['joints']))
        if not keys and not hands:
            raise ArmError('no joint pose to copy the joint list from: make the first one by hand in poses.yaml')
    missing = [joint for joint in keys if joint not in joints]
    if missing:
        raise ArmError(f'not in the joint states: {", ".join(missing)}')
    request: dict = {'joints': {joint: joints[joint] for joint in keys}, 'create': existing is None}
    if hands:
        if targets is None:
            raise ArmError('no current hand positions (forward kinematics unavailable: is MoveIt running?)')
        tools = list(existing['tools']) if existing else profile_tools
        only = [tool for tool in body.get('tools') or [] if tool in tools] or tools
        kept = {tool: target for tool, target in (existing or {}).get('tools', {}).items() if tool not in only}
        request.update(tools={**kept, **{tool: targets[tool] for tool in only}},
                       elbow=(existing or {}).get('elbow'))
    try:
        return robot.save_pose(store, robot_name, name, request, pose_args)
    except robot.RobotSettingsError as error:
        raise ArmError(str(error)) from error


def grip_ranges(store: ConfigStore, robot_name: str) -> dict[str, list[float]]:
    """{tool: [smallest, largest] finger opening in m} for the grippers with an opening_m table."""
    from taskweave_core.gripper import opening_range

    ranges = {}
    for tool, gripper in store.robot(robot_name).data.get('grippers', {}).items():
        span = opening_range(gripper)
        if span is not None:
            ranges[tool] = [span[0], span[1]]
    return ranges


def closed_grips(store: ConfigStore, robot_name: str, tools: list[str], joints: Mapping) -> dict[str, float]:
    """The grippers of ``tools`` (all if empty) that are closed now (further from open toward closed than the
    profile's grippers.<tool>.closed_from): {tool: finger opening in m (the profile's opening_m table)}, the
    point's grip_m. Open grippers (also a little past open) and grippers without a table are left out, so their grip
    stays as it was."""
    from taskweave_core.gripper import opening_for_joint, opening_table

    grippers = store.robot(robot_name).data.get('grippers', {})
    grip = {}
    for tool, gripper in grippers.items():
        if tools and tool not in tools or gripper.get('joint') not in joints or 'closed' not in gripper:
            continue
        if opening_table(gripper) is None:
            continue
        opened, closed = float(gripper.get('open', 0.0)), float(gripper['closed'])
        value = float(joints[gripper['joint']])
        closed_from = float(gripper.get('closed_from', schemas.default(schemas.ROBOT_PROFILE, 'grippers', '*',
                                                                       'closed_from')))
        if (value - opened) * (1.0 if closed >= opened else -1.0) > closed_from:
            grip[tool] = round(opening_for_joint(gripper, value), 4)
    return grip


def nearest_placed(store: ConfigStore, object_type: str, relative: list[dict]) -> str:
    """The placed object of this type nearest the robot (relative: points.relative_poses()['relative'])."""
    for entry in relative:
        if entry['object'] == object_type:
            return entry['instance']
    raise ArmError(f'no placed {object_type} in cell/objects.yaml to teach at')
