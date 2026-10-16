"""Jog a tool's TCP from velocity commands, through IK (the pad's tool modes, taskweave_core.operator_joy).

    ros2 run taskweave_skills tcp_jog --ros-args -p config_dir:=... -p robot:=...

Per IK tool of the robot profile it takes geometry_msgs/TwistStamped on /taskweave/jog/<tool>: the flange's velocity
in the base frame (operator_joy turns pad sticks into it). While commands keep coming, it integrates a flange target
at ``rate_hz`` (sim time) and sends the arm's joints as position targets on joints.command. Each step is one damped
least-squares step on the arm's Jacobian from the robot's URDF (``robots/<robot>/moveit/robot.urdf``, chain.py): near
a singularity the hand slows down instead of stopping, and a joint that reaches its limit is held there while the
other joints carry on (the HMI hears which, on the operator notice topic). A step MoveIt finds in collision
(/check_state_validity) is not taken. Without the URDF, each step is one local IK descent through MoveIt instead.
Silent when no commands come, so it never competes with skills. A tool that holds an object (/taskweave/holding) is
not jogged.

Elbow: std_msgs/Float64 on /taskweave/jog/<tool>/elbow (rad/s) swings the elbow while the flange stays: the joints move
along the arm's self-motion (the Jacobian's null space; a 7-joint arm has one), at about that rate, and the step keeps
correcting the flange onto where it was. Without the URDF: the swivel joint (profile ik.swivel_joint, default the
arm's third joint) turns and one IK descent puts the flange back.

ROS parameters: config_dir, robot, rate_hz (20.0, steps per second), validity_timeout_s (0.1: a collision check
slower than this lets the step through, so jogging never blocks on it).
"""

from __future__ import annotations

import math
import re
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from taskweave_core.config import ConfigStore

NAMESPACE = '/taskweave/jog'
FRESH_S = 0.3  # wall seconds a command stays valid
VALIDITY_POLL_S = 0.002  # poll period of a state validity request
MAX_STEP_S = 0.2  # longest time integrated in one step (after a stall or a sim pause the target does not jump)
FOLLOW_M, FOLLOW_RAD = 0.005, 0.02  # a reached flange this close to the target keeps the target, else takes its place
SWIVEL_SIGN = 0.1  # the self-motion's swivel-joint component above which its sign sets the direction


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
    PARAMS = {'config_dir': '', 'robot': '', 'rate_hz': 20.0, 'validity_timeout_s': 0.1}  # ROS parameters (module doc)

    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'tcp_jog'), **kwargs)
        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
        self.validity_timeout = float(self.get_parameter('validity_timeout_s').value)
        store = ConfigStore(self.get_parameter('config_dir').value)
        profile = store.robot(self.get_parameter('robot').value)
        self.tools = {}
        if not profile.data.get('ik'):
            self.get_logger().info('no ik section in the robot profile: TCP jogging off')
            return
        from geometry_msgs.msg import TwistStamped
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64

        from taskweave_core import tf_util

        from .ik import IkClient

        self.JointState = JointState
        self.ik = IkClient(self, profile)
        self.base = profile.base['frame']
        self.tf, self.tf_listener = tf_util.buffer(self)  # cleared when sim time jumps back
        joints = profile.role('joints')
        self.command = self.create_publisher(JointState, joints['command'], 10)
        self.positions: dict[str, float] = {}
        self.create_subscription(JointState, joints['state'], self._joints, 10)
        self.lock = threading.Lock()
        for tool in self.ik.groups:
            if tool in self.ik.tools:
                self.tools[tool] = {'twist': None, 'received': -math.inf, 'target': None, 'blocked': False,
                                    'joints': re.compile(self.ik.joint_pattern.format(tool=tool)),
                                    'elbow': 0.0, 'elbow_received': -math.inf,
                                    'swivel': profile.data['ik'].get('swivel_joint', '').format(tool=tool) or None}
                self.create_subscription(TwistStamped, f'{NAMESPACE}/{tool}', lambda m, t=tool: self._twist(t, m), 10)
                self.create_subscription(Float64, f'{NAMESPACE}/{tool}/elbow', lambda m, t=tool: self._elbow(t, m), 10)
        self._setup_chains(store, profile)
        self.held_tools: set = set()  # tools holding an object (taskweave_skills.hold): jogging one would pull it apart
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String

        from .hold import TOPIC as HOLD_TOPIC
        self.create_subscription(String, HOLD_TOPIC, self._holding,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_time = None
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._step,
                          callback_group=MutuallyExclusiveCallbackGroup())
        self.get_logger().info(f'TCP jog for {", ".join(self.tools)} on {NAMESPACE}/<tool> -> {joints["command"]}')

    def _setup_chains(self, store, profile) -> None:
        """Velocity-level jogging from the URDF (chain.py), plus MoveIt's state validity check for collisions."""
        from moveit_msgs.srv import GetStateValidity
        from std_msgs.msg import String

        from taskweave_core import names
        from taskweave_core.config.limits import joint_limits, urdf_path

        from .chain import Chain

        self.String = String
        self.notice = self.create_publisher(String, names.OPERATOR_NOTICE, 10)
        self.limits = joint_limits(store, profile.name)
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        path = urdf_path(store, profile.name)
        for tool, jog in self.tools.items():
            jog['chain'], jog['names'], jog['held'], jog['null'] = None, [], '', None
            if not path.is_file():
                continue
            try:
                chain = Chain.from_urdf(path, self.base, self.ik.tools[tool]['link'], tuning=self.ik.tuning['chain'])
            except (ValueError, OSError) as error:
                self.get_logger().warning(f'jog {tool}: no URDF chain ({error}); jogging through IK')
                continue
            jog['chain'] = chain
            jog['names'] = [name for name in chain.movable if jog['joints'].fullmatch(name)]
        if any(jog['chain'] for jog in self.tools.values()):
            self.get_logger().info(f'jog: velocity steps on the URDF chain ({path.name})')

    def _valid(self, group: str, joints: dict) -> bool:
        """MoveIt's collision check of the arm at ``joints``; True when the service is not there (nothing to ask)."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity

        if not self.validity.service_is_ready():
            return True
        request = GetStateValidity.Request(group_name=group, robot_state=RobotState(
            joint_state=self.JointState(name=list(joints), position=list(joints.values())), is_diff=True))
        future = self.validity.call_async(request)
        deadline = time.monotonic() + self.validity_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(VALIDITY_POLL_S)
        return not future.done() or future.result().valid  # too slow: do not block jogging on it

    def _held_notice(self, tool: str, jog: dict, held: str) -> None:
        """Tell the operator (HMI header) once when a joint starts holding at its limit."""
        if held and held != jog['held']:
            text = f'jog {tool}: {held} at its limit, the other joints carry on'
            self.get_logger().info(text)
            self.notice.publish(self.String(data=__import__('json').dumps({'text': text, 'ok': False})))
        jog['held'] = held

    def _chain_step(self, tool: str, jog: dict, target: np.ndarray, null: np.ndarray | None = None) -> bool:
        """One velocity step toward ``target`` (flange, base frame) on the URDF chain; False if not taken."""
        from .chain import pose_error

        with self.lock:
            positions = dict(self.positions)
        if any(name not in positions for name in jog['names']):
            return False
        joints, held = jog['chain'].solve_step(positions, target, jog['names'], self.limits, null=null)
        self._held_notice(tool, jog, held)
        if not self._valid(self.ik.groups[tool], joints):
            if not jog['blocked']:
                self.get_logger().warning(f'jog {tool}: stopped (the next step is in collision)')
                self.notice.publish(self.String(data=__import__('json').dumps(
                    {'text': f'jog {tool}: stopped, the next step is in collision', 'ok': False})))
            jog['blocked'] = True
            return False
        reached = jog['chain'].fk({**positions, **joints})
        # where the hand could not follow (a limit, out of reach), the target stays with the hand instead of running off
        error = pose_error(target, reached)
        followed = np.linalg.norm(error[:3]) < FOLLOW_M and np.linalg.norm(error[3:]) < FOLLOW_RAD
        jog['target'] = target if followed else reached
        jog['blocked'] = False
        self.command.publish(self.JointState(name=list(joints), position=list(joints.values())))
        return True

    def _holding(self, msg) -> None:
        import json
        self.held_tools = set(json.loads(msg.data or '{}').get('tools', []))

    def _joints(self, msg) -> None:
        with self.lock:
            self.positions.update(zip(msg.name, msg.position, strict=False))

    def _twist(self, tool: str, msg) -> None:
        t = msg.twist
        with self.lock:
            self.tools[tool]['twist'] = ((t.linear.x, t.linear.y, t.linear.z), (t.angular.x, t.angular.y, t.angular.z))
            self.tools[tool]['received'] = time.monotonic()

    def _elbow(self, tool: str, msg) -> None:
        with self.lock:
            self.tools[tool]['elbow'] = float(msg.data)
            self.tools[tool]['elbow_received'] = time.monotonic()

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
        dt = 0.0 if self.last_time is None else min(max(now - self.last_time, 0.0), MAX_STEP_S)
        self.last_time = now
        for tool, jog in self.tools.items():
            with self.lock:
                fresh = time.monotonic() - jog['received'] < FRESH_S
                # the arm's joints only: the state topic can carry joints IK's model lacks (a sim's base joints)
                twist = jog['twist']
                current = {n: v for n, v in self.positions.items() if jog['joints'].fullmatch(n)}
                elbow = jog['elbow'] if time.monotonic() - jog['elbow_received'] < FRESH_S else 0.0
            if abs(elbow) > 1e-9:
                self._swing(tool, jog, current, elbow * dt)
                continue
            if abs(elbow) <= 1e-9:
                jog['null'] = None
            if not fresh or twist is None or not any(abs(v) > 1e-9 for part in twist for v in part):
                jog['target'], jog['blocked'] = None, False  # the next jog starts from where the arm is
                continue
            if tool in self.held_tools:
                if not jog['blocked']:
                    self.get_logger().warning(f'jog {tool}: holding an object with it, not jogging (open the gripper '
                                              'or move the object with move_object)')
                jog['blocked'] = True
                continue
            if jog['target'] is None:
                jog['target'] = self._flange(tool)
                if jog['target'] is None:
                    continue
            target = integrate(jog['target'], twist[0], twist[1], dt)
            if jog.get('chain') is not None:  # IK fallback without a URDF chain
                self._chain_step(tool, jog, target)
                continue
            joints = self.ik.step(tool, target, current)
            if joints is None:
                if not jog['blocked']:
                    self.get_logger().warning(f'jog {tool}: stopped (joint limit, collision or out of reach)')
                jog['blocked'] = True
                continue
            jog['target'], jog['blocked'] = target, False
            self.command.publish(self.JointState(name=list(joints), position=list(joints.values())))

    def _swing(self, tool: str, jog: dict, current: dict, angle: float) -> None:
        """Turn the swivel joint by ``angle`` and put the flange back where it is (the elbow moves, the TCP not)."""
        if tool in self.held_tools or not current:
            return
        swivel = jog['swivel'] or (sorted(current)[2] if len(current) > 2 else None)
        if swivel not in current:
            self.get_logger().warning(f'jog {tool}: no swivel joint {swivel!r}', throttle_duration_sec=5)
            return
        if jog['target'] is None:
            jog['target'] = self._flange(tool)
            if jog['target'] is None:
                return
        if jog.get('chain') is not None:  # IK fallback without a URDF chain
            with self.lock:
                positions = dict(self.positions)
            if any(name not in positions for name in jog['names']):
                return
            direction = jog['chain'].null_direction(positions, jog['names'])
            # the self-motion has no sign of its own: positive turns the swivel joint up, and it keeps its way
            index = jog['names'].index(swivel) if swivel in jog['names'] else 0
            if abs(direction[index]) > SWIVEL_SIGN:
                direction = direction * np.sign(direction[index])
            elif jog['null'] is not None and direction @ jog['null'] < 0:
                direction = -direction
            jog['null'] = direction
            self._chain_step(tool, jog, jog['target'], null=direction * angle)
            return
        joints = self.ik.step(tool, jog['target'], {**current, swivel: current[swivel] + angle})
        if joints is None:
            if not jog['blocked']:
                self.get_logger().warning(f'jog {tool}: elbow stopped (joint limit, collision or no way to keep the TCP)')
            jog['blocked'] = True
            return
        jog['blocked'] = False
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
