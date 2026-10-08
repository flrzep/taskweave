"""Teach node: saves the robot's current base pose as a named location or stance.

The pose comes from the robot profile's ``base.odom`` topic, which must report the pose in the cell frame
(ground truth in simulation, the localisation output on a real robot).

    ros2 service call /taskweave/teach_point taskweave_interfaces/srv/TeachPoint "{name: bay_2, kind: location}"
    taskweave teach location bay_2
"""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from taskweave_interfaces.srv import TeachPoint

from . import names
from .config import ConfigStore, Pose2D

MAX_POSE_AGE_S = 1.0


def yaw_from_quaternion(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Teach(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'teach'), **kwargs)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('robot', '')
        self.config = ConfigStore(self.get_parameter('config_dir').value)
        self.robot = self.config.robot(self.get_parameter('robot').value)
        self.pose: Pose2D | None = None
        self.stamp = 0.0
        self.create_subscription(Odometry, self.robot.base['odom'], self._odom, 10)
        self.create_service(TeachPoint, names.TEACH_POINT, self._teach)
        self.get_logger().info(f'teach: base pose from {self.robot.base["odom"]}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = Pose2D(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.stamp = self._now()

    def _teach(self, request, response):
        if self.pose is None or self._now() - self.stamp > MAX_POSE_AGE_S:
            response.success, response.message = False, f'no current pose on {self.robot.base["odom"]}'
            return response
        try:
            path = self.config.save_base_pose(request.name, self.pose, request.kind or 'location', self.robot.name)
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
            return response
        response.success, response.file = True, str(path)
        response.message = f'{request.kind or "location"} {request.name!r} = {self.pose.to_dict()}'
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
