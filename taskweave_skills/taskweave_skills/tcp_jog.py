"""Jog a tool's TCP from velocity commands, through IK (the pad's tool modes, taskweave_core.operator_joy).

    ros2 run taskweave_skills tcp_jog --ros-args -p config_dir:=... -p robot:=...

Per IK tool of the robot profile it takes geometry_msgs/TwistStamped on /taskweave/jog/<tool>: the flange's velocity
in the base frame (operator_joy turns pad sticks into it). While commands keep coming, it integrates a flange target
at ``rate_hz`` (sim time), solves each step by one local IK descent from the current joints (the profile's ik
section: same solver, joint limits and collision checks as named poses), and sends the arm's joints as position
targets on joints.command. A step without a solution (a joint limit, a collision, out of reach) is not taken: the
arm stops there and the reason is logged. Silent when no commands come, so it never competes with skills.
"""

from __future__ import annotations

import math
import threading
import time

import re

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from taskweave_core.config import ConfigStore

NAMESPACE = '/taskweave/jog'
FRESH_S = 0.3  # wall seconds a command stays valid


def rotation(omega: np.ndarray, dt: float) -> np.ndarray:
    """Rotation matrix for turning at ``omega`` (rad/s, axis * rate) for ``dt`` seconds (Rodrigues)."""
    angle = float(np.linalg.norm(omega)) * dt
    if angle < 1e-12:
        return np.eye(3)
    k = omega / np.linalg.norm(omega)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * kx @ kx


def integrate(pose: np.ndarray, linear, angular, dt: float) -> np.ndarray:
    """Flange pose after moving at (linear, angular) velocity, both in the base frame, for ``dt`` seconds."""
    moved = pose.copy()
    moved[:3, :3] = rotation(np.asarray(angular, float), dt) @ pose[:3, :3]
    moved[:3, 3] = pose[:3, 3] + np.asarray(linear, float) * dt
    return moved


class TcpJog(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'tcp_jog'), **kwargs)
        for name, default in (('config_dir', ''), ('robot', ''), ('rate_hz', 20.0)):
            self.declare_parameter(name, default)
        profile = ConfigStore(self.get_parameter('config_dir').value).robot(self.get_parameter('robot').value)
        self.tools = {}
        if not profile.data.get('ik'):
            self.get_logger().info('no ik section in the robot profile: TCP jogging off')
            return
        from geometry_msgs.msg import TwistStamped
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from sensor_msgs.msg import JointState
        from tf2_ros import Buffer, TransformListener

        from .ik import IkClient

        self.JointState = JointState
        self.ik = IkClient(self, profile)
        self.base = profile.base['frame']
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        joints = profile.role('joints')
        self.command = self.create_publisher(JointState, joints['command'], 10)
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, joints['state'], self._joints, 10)
        self.lock = threading.Lock()
        for tool in self.ik.groups:
            if tool in self.ik.tools:
                self.tools[tool] = {'twist': None, 'received': -math.inf, 'target': None, 'blocked': False,
                                    'joints': re.compile(self.ik.joint_pattern.format(tool=tool))}
                self.create_subscription(TwistStamped, f'{NAMESPACE}/{tool}', lambda m, t=tool: self._twist(t, m), 10)
        self.last_time = None
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._step,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'TCP jog for {", ".join(self.tools)} on {NAMESPACE}/<tool> -> {joints["command"]}')

    def _joints(self, msg) -> None:
        with self.lock:
            self.positions.update(zip(msg.name, msg.position, strict=False))

    def _twist(self, tool: str, msg) -> None:
        t = msg.twist
        with self.lock:
            self.tools[tool]['twist'] = ((t.linear.x, t.linear.y, t.linear.z), (t.angular.x, t.angular.y, t.angular.z))
            self.tools[tool]['received'] = time.monotonic()

    def _flange(self, tool: str) -> np.ndarray | None:
        from .ik import pose_matrix
        try:
            transform = self.tf.lookup_transform(self.base, self.ik.tools[tool]['link'], rclpy.time.Time()).transform
        except Exception as error:  # noqa: BLE001 - several tf2 exception types
            self.get_logger().warning(f'jog {tool}: no transform {self.base} -> flange: {error}', throttle_duration_sec=2)
            return None
        return pose_matrix(transform.translation, transform.rotation)

    def _step(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_time is None else min(max(now - self.last_time, 0.0), 0.2)
        self.last_time = now
        for tool, jog in self.tools.items():
            with self.lock:
                fresh = time.monotonic() - jog['received'] < FRESH_S
                # the arm's joints only: the state topic can carry joints IK's model lacks (a sim's base joints)
                twist = jog['twist']
                current = {n: v for n, v in self.positions.items() if jog['joints'].fullmatch(n)}
            if not fresh or twist is None or not any(abs(v) > 1e-9 for part in twist for v in part):
                jog['target'], jog['blocked'] = None, False  # the next jog starts from where the arm is
                continue
            if jog['target'] is None:
                jog['target'] = self._flange(tool)
                if jog['target'] is None:
                    continue
            target = integrate(jog['target'], twist[0], twist[1], dt)
            joints = self.ik.step(tool, target, current)
            if joints is None:
                if not jog['blocked']:
                    self.get_logger().warning(f'jog {tool}: stopped (joint limit, collision or out of reach)')
                jog['blocked'] = True
                continue
            jog['target'], jog['blocked'] = target, False
            self.command.publish(self.JointState(name=list(joints), position=list(joints.values())))


def main(args=None):
    rclpy.init(args=args)
    node = TcpJog()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
