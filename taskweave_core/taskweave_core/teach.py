"""Teach node: saves the robot's current base pose as a named location or as a stance at an object, or its
tools' TCP poses as a point on an object.

Stances are stored relative to the object instance (robots/<robot>/stances.yaml), so one teach covers every
instance of that object type; kind ``stance_instance`` stores an override for that instance only.

The base pose comes from the robot profile's ``base.odom`` topic, which must report the pose in the cell frame
(ground truth in simulation, the localisation output on a real robot).

Object points (kind ``object_point``, name ``<instance>/<point>``) store each tool's canonical TCP (profile
``tool_frames``: flange link + offset) relative to the instance's top item in objects/<object>.yaml, for every
instance of that type; go_to_point drives the tools back to them. The TCP comes from TF in the cell frame
(go_to_point's ``cell_frame``). Jog the tools onto the object with the pad, then teach (pad ``teach`` button or
the CLI).

    ros2 service call /taskweave/teach_point taskweave_interfaces/srv/TeachPoint "{name: bay_2, kind: location}"
    taskweave teach location bay_2
    taskweave teach point bay_2_stack_1/rails_top [--tool left]
"""

from __future__ import annotations

import datetime
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from taskweave_interfaces.srv import TeachPoint

from . import names, tf_util
from .config import ConfigError, ConfigStore, Pose2D
from .config.poses import quaternion_from_rpy, quaternion_multiply, rotate

MAX_POSE_AGE_S = 1.0


def yaw_from_quaternion(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Teach(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'teach'), **kwargs)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('robot', '')
        self.declare_parameter('cell_frame', '')  # default: go_to_point's cell_frame, else 'world'
        self.config = ConfigStore(self.get_parameter('config_dir').value)
        self.robot = self.config.robot(self.get_parameter('robot').value)
        self.cell_frame = self.get_parameter('cell_frame').value or self._skill_cell_frame()
        self.pose: Pose2D | None = None
        self.stamp = 0.0
        self.tf, self.tf_listener = tf_util.buffer(self)  # cleared when sim time jumps back
        self.create_subscription(Odometry, self.robot.base['odom'], self._odom, 10)
        self.create_service(TeachPoint, names.TEACH_POINT, self._teach)
        self.get_logger().info(f'teach: base pose from {self.robot.base["odom"]}, tool TCPs in {self.cell_frame}')

    def _skill_cell_frame(self) -> str:
        try:
            return self.config.skill_config('go_to_point').get('cell_frame', 'world')
        except (ConfigError, KeyError):
            return 'world'

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = Pose2D(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.stamp = self._now()

    def tcp_pose(self, tool: str):
        """The tool's canonical TCP in the cell frame: (position, quaternion xyzw); raises if TF lacks it."""
        frame = self.robot.data.get('tool_frames', {}).get(tool)
        if frame is None:
            raise ValueError(f'no tool_frames.{tool} in the robot profile')
        t = self.tf.lookup_transform(self.cell_frame, frame['link'], rclpy.time.Time()).transform
        flange_q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        offset = rotate(flange_q, frame.get('xyz', [0.0, 0.0, 0.0]))
        position = (t.translation.x + offset[0], t.translation.y + offset[1], t.translation.z + offset[2])
        return position, quaternion_multiply(flange_q, quaternion_from_rpy(frame.get('rpy', [0.0, 0.0, 0.0])))

    def _teach_object_point(self, request, response):
        tools = list(request.tools) or list(self.robot.data.get('tool_frames', {}))
        try:
            poses = {tool: self.tcp_pose(tool) for tool in tools}
            note = f'taught {datetime.date.today().isoformat()}'
            path, stored = self.config.save_object_point(request.name, poses, note)
        except Exception as error:  # noqa: BLE001 - tf2 raises several types; the caller gets the message
            response.success, response.message = False, f'object point {request.name!r}: {error}'
            return response
        response.success, response.file = True, str(path)
        response.message = f'object point {request.name!r}: ' + '; '.join(
            f'{tool} xyz {[round(v, 3) for v in pose.xyz]} rpy {[round(v, 3) for v in pose.rpy]}'
            for tool, pose in stored.items())
        self.get_logger().info(f'taught {response.message} -> {path}')
        return response

    def _teach(self, request, response):
        if request.kind == 'object_point':
            return self._teach_object_point(request, response)
        if self.pose is None or self._now() - self.stamp > MAX_POSE_AGE_S:
            response.success, response.message = False, f'no current pose on {self.robot.base["odom"]}'
            return response
        kind = request.kind or 'location'
        try:
            if kind in ('stance', 'stance_instance'):
                path, offset = self.config.save_stance(request.name, self.pose, self.robot.name,
                                                       per_instance=kind == 'stance_instance')
                stored = f'offset {offset.to_dict()} (base at {self.pose.to_dict()})'
            else:
                path, stored = self.config.save_base_pose(request.name, self.pose, kind), self.pose.to_dict()
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        response.success, response.file = True, str(path)
        response.message = f'{kind} {request.name!r} = {stored}'
        self.get_logger().info(f'taught {response.message} -> {path}')
        return response


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
