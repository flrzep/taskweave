"""Teach node: saves where the robot is now as a target: its base pose, its tools' TCP poses, or its joint values.

The request names a target reference and a kind, and the node writes it with
``ConfigStore.save_target`` (comments and neighbours kept, the file chosen by the target):

    base    ``<name>``             the base pose in the cell frame            -> cell/targets.yaml
    base    ``<instance>/<name>``  a stance: the pose relative to the object  -> robots/<robot>/objects/<type>.yaml
    hand    ``<instance>/<name>``  each tool's TCP relative to the object     -> objects/<type>.yaml
    hand    ``<name>``             each tool's TCP in the base frame (the cell frame for a target of
                                   cell/targets.yaml)                         -> robots/<robot>/targets.yaml
    joints  ``<name>``             the joint values of the groups             -> robots/<robot>/targets.yaml
    object  ``<name>``             the held object's origin in the base frame (a carry pose), from its
                                   holding hand's TCP and the grip measured at the close
                                                                              -> robots/<robot>/targets.yaml

``instance_override`` writes the override for that object instance only (base, hand). ``tools``: the tools of a hand
teach (default every tool of the profile's ``tool_frames``), the groups of a joints teach (default the target's own
groups, else every group of the profile), the hand whose hold an object teach takes (needed when several objects are
held). An existing target keeps every key the teach does not set (joints, elbow, the other tools, the way there of a
path: an object teach replaces a path's last pose), a new one gets the pose only (an object one also its ``type``).

The kinds of the old point model are still taken as aliases: ``location`` and ``stance`` are ``base``,
``stance_instance`` is ``base`` with ``instance_override``, ``object_point`` is ``hand``.

The base pose comes from the robot profile's ``base.odom`` topic, which must report the pose in the cell frame (ground
truth in simulation, the localisation output on a real robot); the TCP poses come from TF (profile ``tool_frames``:
flange link + offset), the joint values from the profile's ``joints.state`` topic, what the hands hold and how they
grip it from the skill server's holds (names.HOLDING). Jog the tools onto the object with
the pad, then teach (pad ``teach`` button or the CLI).

    ros2 service call /taskweave/teach_point taskweave_interfaces/srv/TeachPoint "{name: bin_2, kind: base}"
    taskweave teach base bin_2
    taskweave teach base bin_2/pick [--instance]
    taskweave teach hand bin_2/top_grasp [--tool left]
    taskweave teach joints home [--tool left_arm]
    taskweave teach object carry [--tool left]
"""

from __future__ import annotations

import datetime
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from taskweave_interfaces.srv import TeachPoint

from . import clock_jump, names, tf_util
from .config import ConfigError, ConfigStore, Pose2D
from .config import target_store as ts
from .config import targets as model
from .config.limits import joint_groups, joint_groups_of
from .config.poses import (
    Pose3D,
    quaternion_from_matrix,
    quaternion_from_rpy,
    quaternion_multiply,
    rotate,
    rpy_from_quaternion,
)

MAX_POSE_AGE_S = 1.0  # default of max_tf_age_s (the HMI imports it)
OLD_KINDS = {'location': ('base', False), 'stance': ('base', False), 'stance_instance': ('base', True),
             'object_point': ('hand', False)}      # the kinds of the old point model -> (kind, instance_override)
NEW_KINDS = ('base', 'hand', 'joints', 'object')


def yaw_from_quaternion(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Refused(Exception):
    """A teach that cannot be done: the message is the answer."""


class Teach(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'teach'), **kwargs)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('robot', '')
        self.declare_parameter('cell_frame', '')  # default: the hand move's cell_frame, else 'world'
        self.config = ConfigStore(self.get_parameter('config_dir').value)
        self.robot = self.config.robot(self.get_parameter('robot').value)
        self.cell_frame = self.get_parameter('cell_frame').value or self._skill_cell_frame()
        self.pose: Pose2D | None = None
        self.stamp = 0.0
        self.joints: dict[str, float] = {}   # the latest joint states
        self.joints_stamp = 0.0
        self.holds: list[dict] = []           # the latest holds (names.HOLDING)
        self.declare_parameter('max_tf_age_s', MAX_POSE_AGE_S)   # refuse to save from TF or a base pose older than this
        self.declare_parameter('after_jump_s', 2.0)   # ... and for this long after a clock jump
        self.max_age_s = self.get_parameter('max_tf_age_s').value
        self.after_jump_s = self.get_parameter('after_jump_s').value
        self.tf, self.tf_listener = tf_util.buffer(self)  # cleared when sim time jumps back
        self.clock_jump = clock_jump.watch(self)
        self.clock_jump.callbacks.append(self._forget_pose)
        self.create_subscription(Odometry, self.robot.base['odom'], self._odom, 10)
        topic = (self.robot.data.get('joints') or {}).get('state')
        if topic:
            self.create_subscription(JointState, topic, self._joint_state, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, names.HOLDING, self._holding, latched)
        self.create_service(TeachPoint, names.TEACH_POINT, self._teach)
        self.get_logger().info(f'teach: base pose from {self.robot.base["odom"]}, tool TCPs in {self.cell_frame}'
                               + (f', joints from {topic}' if topic else ''))

    def _skill_cell_frame(self) -> str:
        try:
            frame = self.config.skill_config('move.hand').get('cell_frame')
        except (ConfigError, KeyError):
            frame = None
        return frame or 'world'

    def _forget_pose(self) -> None:
        self.pose = None  # the base pose of the old run
        self.joints = {}

    def _stale(self, tf_stamp: float | None = None, joints: bool = False) -> str:
        """Why what teach would save cannot be trusted, or '' when it can. ``tf_stamp``: the oldest TF stamp used;
        ``joints``: the joint states are used."""
        if self.clock_jump.recent(self.after_jump_s):
            return f'the sim clock jumped less than {self.after_jump_s:g} s ago (clock jump); wait and try again'
        now = self._now()
        if self.pose is not None and abs(now - self.stamp) > self.max_age_s:
            return f'the base pose on {self.robot.base["odom"]} is stale ({now - self.stamp:.1f} s old)'
        if joints and abs(now - self.joints_stamp) > self.max_age_s:
            return f'the joint states are stale ({now - self.joints_stamp:.1f} s old)'
        if tf_stamp:
            age = now - tf_stamp
            if abs(age) > self.max_age_s:
                return f'TF is stale ({age:.1f} s old, limit {self.max_age_s:g} s); is the robot publishing TF?'
        return ''

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = Pose2D(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.stamp = self._now()

    def _joint_state(self, msg: JointState) -> None:
        self.joints.update(zip(msg.name, msg.position, strict=False))
        self.joints_stamp = self._now()

    def _holding(self, msg: String) -> None:
        try:
            self.holds = list((json.loads(msg.data or '{}') or {}).get('holds', []))
        except (ValueError, AttributeError):
            self.get_logger().warning(f'unreadable {names.HOLDING} message ignored')

    def tcp_pose(self, tool: str, stamps: list | None = None, frame: str | None = None):
        """The tool's canonical TCP in ``frame`` (default the cell frame): (position, quaternion xyzw); raises if TF
        lacks it. The TF stamp (seconds) is appended to ``stamps``."""
        tool_frame = self.robot.data.get('tool_frames', {}).get(tool)
        if tool_frame is None:
            raise ValueError(f'no tool_frames.{tool} in the robot profile')
        found = self.tf.lookup_transform(frame or self.cell_frame, tool_frame['link'], rclpy.time.Time())
        t, stamp = found.transform, found.header.stamp
        if stamps is not None:
            stamps.append(stamp.sec + stamp.nanosec * 1e-9)
        flange_q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        offset = rotate(flange_q, tool_frame.get('xyz', [0.0, 0.0, 0.0]))
        position = (t.translation.x + offset[0], t.translation.y + offset[1], t.translation.z + offset[2])
        return position, quaternion_multiply(flange_q, quaternion_from_rpy(tool_frame.get('rpy', [0.0, 0.0, 0.0])))

    def _tcp_poses(self, tools: list[str], frame: str | None = None) -> dict:
        """``{tool: (position, quaternion)}`` of the tools in ``frame`` from TF, refusing stale data."""
        stamps: list = []
        poses = {tool: self.tcp_pose(tool, stamps, frame) for tool in tools}
        if reason := self._stale(min((x for x in stamps if x), default=None)):  # 0: a static transform
            raise Refused(reason)
        return poses

    # ---- the request ------------------------------------------------------------------------------------------
    def _teach(self, request, response):
        override = bool(getattr(request, 'instance_override', False))   # absent in an older interface build
        asked = request.kind or 'base'
        kind = asked
        try:
            if asked in OLD_KINDS:
                kind, old_override = OLD_KINDS[asked]
                override = override or old_override
            elif asked not in NEW_KINDS:
                raise Refused(f'unknown kind {asked!r} (it is one of {", ".join(NEW_KINDS)})')
            text, written = getattr(self, f'_teach_{kind}')(request, override)
        except Refused as error:
            response.success, response.message = False, f'{kind} {request.name!r}: {error}'
            return response
        except (ValueError, KeyError, OSError, ConfigError) as error:
            message = error.args[0] if error.args else error
            response.success, response.message = False, f'{kind} {request.name!r}: {message}'
            return response
        except Exception as error:  # noqa: BLE001 - tf2 raises several types; the caller gets the message
            response.success, response.message = False, f'{kind} {request.name!r}: {error}'
            return response
        response.success, response.file = True, ', '.join(str(path) for path in written)
        response.message = f'{kind} {request.name!r}: {text}'
        self.get_logger().info(f'taught {response.message} -> {response.file}')
        return response

    def _note(self) -> str:
        return f'taught {datetime.date.today().isoformat()}'

    def _base_pose(self) -> Pose2D:
        if reason := self._stale():
            raise Refused(reason)
        if self.pose is None:
            raise Refused(f'no current pose on {self.robot.base["odom"]}')
        return self.pose

    def _instance_of(self, reference: str, near: Pose2D | None):
        """``(placed object as handled, instance name, target name)`` of ``<instance>/<name>``, or ``(None, None,
        name)`` for a plain name."""
        instance, separator, name = reference.rpartition(ts.SEPARATOR)
        if not separator:
            return None, None, reference
        placed = self.config.object_instances()
        if instance not in placed:
            raise Refused(str(self.config.missing_instance(instance, f'no object {instance!r}').args[0]))
        return self.config.handling_instance(placed[instance], self.robot.name, near), instance, name

    def _existing(self, reference: str, handled, instance: str | None, name: str, override: bool, moves: str) -> dict:
        """The entry of the target as it is written now, to keep the keys a teach does not set; {} for a new one.
        ``override``: the instance's own entry. A target that moves something else is refused."""
        robot = self.robot.name
        if handled is None:
            try:
                found = self.config.target(reference, robot)
            except KeyError:
                return {}
            kind, data = found.kind, found.data
        else:
            if override:
                tables = [(f'objects/{handled.object}.yaml', ts.object_file(self.config, handled.object)),
                          (f'robots/{robot}/objects/{handled.object}.yaml',
                           ts.robot_object_file(self.config, robot, handled.object))]
                layers = [(label, entry) for label, table in tables
                          for entry in [table.get('instances', {}).get(instance, {}).get(name)] if entry is not None]
            else:
                layers = ts.entry_layers(self.config, robot, handled.object, name)
            data = model.merge_layers(name, layers) if layers else {}
            kind = data.get('moves', moves)
        if kind != moves:
            raise Refused(f'{reference!r} moves {model.MOVES_WORD[kind]}, not {model.MOVES_WORD[moves]}: '
                          'give another name')
        return dict(data)

    # ---- base -------------------------------------------------------------------------------------------------
    def _teach_base(self, request, override: bool):
        pose = self._base_pose()
        handled, instance, name = self._instance_of(request.name, pose)
        if handled is None:
            if override:
                raise Refused('instance_override is for <instance>/<name> references')
            existing = self._existing(request.name, None, None, name, False, 'base')
            if existing and self.config.target(request.name, self.robot.name).frame != 'cell':
                raise Refused('this base target is in the base frame: a base pose is taught in the cell frame')
            entry = pose
        else:
            entry = pose.relative_to(handled.pose, handled.object)
            existing = self._existing(request.name, handled, instance, name, override, 'base')
        data = {**{k: v for k, v in existing.items() if k not in ('x', 'y', 'yaw', 'path')}, 'moves': 'base',
                'x': entry.x, 'y': entry.y, 'yaw': entry.yaw}
        written = self.config.save_target(request.name, data, self.robot.name, instance_override=override,
                                          note=None if existing else self._note())
        stored = {key: round(value, 4) for key, value in (('x', entry.x), ('y', entry.y), ('yaw', entry.yaw))}
        text = f'{stored}' if handled is None else f'offset {stored} (base at {pose.to_dict()})'
        return text, written

    # ---- hand -------------------------------------------------------------------------------------------------
    def _teach_hand(self, request, override: bool):
        tools = list(request.tools) or list(self.robot.data.get('tool_frames', {}))
        if not tools:
            raise Refused('the robot profile has no tool_frames')
        handled, instance, name = self._instance_of(request.name, self.pose)
        frame = None    # the cell frame
        if handled is None:
            if override:
                raise Refused('instance_override is for <instance>/<name> references')
            existing = self._existing(request.name, None, None, name, False, 'hand')
            in_cell = bool(existing) and self.config.target(request.name, self.robot.name).frame == 'cell'
            frame = None if in_cell else self.robot.base['frame']
        else:
            existing = self._existing(request.name, handled, instance, name, override, 'hand')
        stored: dict[str, Pose3D] = {}
        for tool, (position, quaternion) in self._tcp_poses(tools, frame).items():
            stored[tool] = (handled.relative_pose(position, quaternion) if handled is not None else
                            Pose3D(tuple(position), rpy_from_quaternion(quaternion), frame or self.cell_frame))
        data = {k: v for k, v in existing.items() if k != 'path'}
        data['moves'] = 'hand'
        for tool, pose in stored.items():
            data[f'{tool}{model.TOOL_SUFFIX}'] = {'xyz': list(pose.xyz), 'rpy': list(pose.rpy)}
        written = self.config.save_target(request.name, data, self.robot.name, instance_override=override,
                                          note=None if existing else self._note())
        text = '; '.join(f'{tool} xyz {[round(v, 3) for v in pose.xyz]} rpy {[round(v, 3) for v in pose.rpy]}'
                         for tool, pose in stored.items())
        return text, written

    # ---- object -----------------------------------------------------------------------------------------------
    def _teach_object(self, request, override: bool):
        if override:
            raise Refused('instance_override is for base and hand targets of an object')
        if ts.SEPARATOR in request.name:
            raise Refused('a contact (<instance>/<name>, in the object frame) is not taught: a held object is taught '
                          'as a carry pose, a plain name in the base frame')
        existing = self._existing(request.name, None, None, request.name, False, 'object')
        hold, hand = self._hold(list(request.tools))
        placed = self.config.object_instances().get(hold.get('object'))
        held_type = hold.get('type') or (placed.object if placed is not None else None)
        wanted = existing.get('type')
        if wanted and held_type and wanted != held_type:
            raise Refused(f'{request.name!r} is for {wanted}, the hands hold {hold["object"]} ({held_type})')
        base = self.robot.base['frame']
        position, quaternion = self._tcp_poses([hand], base)[hand]
        grip = hold['grips'][hand]                       # the TCP in the object frame, measured at the close
        x, y, z, w = quaternion_from_matrix(grip)
        unturn = (-x, -y, -z, w)                         # the object's turn seen from the TCP
        back = rotate(unturn, [-grip[row][3] for row in range(3)])   # the object's origin seen from the TCP
        reach = rotate(quaternion, back)
        pose = {'xyz': [round(p + r, 5) for p, r in zip(position, reach, strict=True)],
                'rpy': [round(a, 5) for a in rpy_from_quaternion(quaternion_multiply(quaternion, unturn))]}
        data = {key: value for key, value in existing.items() if key not in ('xyz', 'rpy')}
        data['moves'] = 'object'
        if held_type and not wanted:
            data['type'] = held_type
        if data.get('path'):
            data['path'] = [*data['path'][:-1], pose]   # the way there stays, the end is where the object is now
        else:
            data.update(pose)
        written = self.config.save_target(request.name, data, self.robot.name, note=None if existing else self._note())
        return f'{hold["object"]} on {hand}: xyz {pose["xyz"]} rpy {pose["rpy"]} in {base}', written

    def _hold(self, tools: list[str]) -> tuple[dict, str]:
        """The hold an object teach takes and the hand whose grip places the object: the hold of ``tools[0]``, else
        the only one."""
        holds = [hold for hold in self.holds if hold.get('hands') and hold.get('grips')]
        if not holds:
            raise Refused('nothing is held: grip the object first')
        if tools:
            hold = next((h for h in holds if tools[0] in h['hands']), None)
            if hold is None:
                raise Refused(f'{tools[0]} holds nothing')
            return hold, tools[0] if tools[0] in hold['grips'] else next(iter(hold['grips']))
        if len(holds) > 1:
            listed = '; '.join(f'{h["object"]} ({", ".join(h["hands"])})' for h in holds)
            raise Refused(f'several objects are held: {listed}; name the hand')
        hold = holds[0]
        return hold, next(h for h in hold['hands'] if h in hold['grips'])

    # ---- joints -----------------------------------------------------------------------------------------------
    def _teach_joints(self, request, override: bool):
        if override:
            raise Refused('instance_override is for base and hand targets of an object')
        if ts.SEPARATOR in request.name:
            raise Refused('a joint target is a plain name')
        existing = self._existing(request.name, None, None, request.name, False, 'joints')
        groups = joint_groups(self.config, self.robot.name)
        wanted = list(request.tools) or list(existing.get('joints', {})) or list(groups)
        unknown = [group for group in wanted if group not in groups]
        if unknown:
            raise Refused(f'no joint group {", ".join(unknown)} (the robot has: {", ".join(groups) or "none"})')
        if not self.joints:
            raise Refused('no joint states: is the robot publishing them (profile joints.state)?')
        if reason := self._stale(joints=True):
            raise Refused(reason)
        wanted_joints = [joint for group in wanted for joint in groups[group]]
        missing = [joint for joint in wanted_joints if joint not in self.joints]
        if missing:
            raise Refused(f'not in the joint states: {", ".join(missing)}')
        values = joint_groups_of(self.config, self.robot.name, {joint: self.joints[joint] for joint in wanted_joints})
        data = {**existing, 'moves': 'joints', 'joints': {**existing.get('joints', {}), **values}}
        written = self.config.save_target(request.name, data, self.robot.name, note=None if existing else self._note())
        return ', '.join(f'{group} {value}' for group, value in values.items()), written


def main(args=None):
    rclpy.init(args=args)
    node = Teach()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
