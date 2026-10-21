"""Jog a tool's TCP from velocity commands, through IK (the pad's tool modes, taskweave_core.operator_joy).

    ros2 run taskweave_skills tcp_jog --ros-args -p config_dir:=... -p robot:=...

Per IK tool of the robot profile it takes geometry_msgs/TwistStamped on /taskweave/jog/<tool>: the flange's velocity
in the base frame (operator_joy turns pad sticks into it). While commands keep coming, it integrates a flange target
at ``rate_hz`` (sim time) and sends the arm's joints as position targets on joints.command. Each step is one damped
least-squares step on the arm's Jacobian from the robot's URDF (``robots/<robot>/moveit/robot.urdf``, chain.py): near
a singularity the hand slows down instead of stopping, and a joint that reaches its limit is held there while the
other joints carry on (the HMI hears which, on the operator notice topic). A step MoveIt finds in collision
(/check_state_validity) is not taken. Without the URDF, each step is one local IK descent through MoveIt instead.
Silent when no commands come, so it never competes with skills.

Object jog: a tool that holds an object (/taskweave/holding) jogs the OBJECT instead. The twist the pad sends for that
tool's flange is applied as it is to the object frame (the same axes in the base frame; the object turns about its own
origin, the linear part is the velocity of that origin). Every holding hand's target is the moved object pose times the
grip (``grips`` of the holding message: the hand's TCP in the object frame, measured at the grip), taken from the flange
link by the profile's tool frame. The step of a two-hand hold is atomic: each hand's chain step is solved, one combined
collision check asks MoveIt about all the arms' joints (with the object's ``collision`` shapes attached inside the
request, never in the live scene), and if a hand cannot follow within the follow tolerance (joint limit, out of reach)
or the state collides, no hand moves, so the hands are never pulled apart; else one JointState carries all arms. A
single hand that lags keeps the object target with the hand, as the TCP jog does. The twist topic of either holding
hand moves the object (the freshest wins). The skill server re-anchors the object pose from the live hand when its
skills start (HoldState.reanchor), so the jog needs no bookkeeping there. Which tools jog an object is published
latched on names.JOG_STATE as JSON ``{"<tool>": {"object": "<instance>", "hands": [...]}}`` (``{}``: none); the
switch is automatic: opening the gripper ends the hold and the tool jogs the arm again.

Elbow: std_msgs/Float64 on /taskweave/jog/<tool>/elbow (rad/s) swings the elbow while the flange stays (also while the
tool holds an object: the hand and the object stay): the joints move
along the arm's self-motion (the Jacobian's null space; a 7-joint arm has one), at about that rate, and the step keeps
correcting the flange onto where it was; the swivel joint (profile arms.<tool>.swivel), if given, sets which way is
positive (it turns up). Without the URDF: the swivel joint turns and one IK descent puts the flange back; an arm without
arms.<tool>.swivel then has no elbow swing.

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


def swivel_joint(profile: dict, tool: str) -> str | None:
    """The joint the elbow swing turns: profile arms.<tool>.swivel (deprecated: ik.swivel_joint, {tool} = tool
    name); None if the profile names none (no arm structure is assumed)."""
    arm = (profile.get('arms') or {}).get(tool) or {}
    return arm.get('swivel') or (profile.get('ik') or {}).get('swivel_joint', '').format(tool=tool) or None


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
                                    'swivel': swivel_joint(profile.data, tool)}
                self.create_subscription(TwistStamped, f'{NAMESPACE}/{tool}', lambda m, t=tool: self._twist(t, m), 10)
                self.create_subscription(Float64, f'{NAMESPACE}/{tool}/elbow', lambda m, t=tool: self._elbow(t, m), 10)
        self._setup_chains(store, profile)
        self.held_tools: set = set()  # tools holding an object (taskweave_skills.hold): their jog moves the object
        self.holds: list[dict] = []   # the holds of the holding message: {object, type, hands, grips}
        self.objects: dict[str, dict] = {}  # object -> {target (4x4, base frame) | None, blocked}: the jogged ones
        self.shape_cache: dict = {}   # object type -> its collision shapes
        self.store, self.robot_data = store, profile.data
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String

        from taskweave_core import names

        from .hold import TOPIC as HOLD_TOPIC
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, HOLD_TOPIC, self._holding, latched)
        self.mode_state = self.create_publisher(String, names.JOG_STATE, latched)
        self._publish_state()
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

    def _valid(self, group: str, joints: dict, attached=()) -> bool:
        """MoveIt's collision check of the arm at ``joints`` (group '': the whole robot) with ``attached`` objects
        (AttachedCollisionObject, only inside this request); True when the service is not there (nothing to ask)."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetStateValidity

        if not self.validity.service_is_ready():
            return True
        request = GetStateValidity.Request(group_name=group, robot_state=RobotState(
            joint_state=self.JointState(name=list(joints), position=list(joints.values())), is_diff=True,
            attached_collision_objects=list(attached)))
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
        data = json.loads(msg.data or '{}')   # the legacy keys are the first hold; ``holds`` has them all
        self.held_tools = set(data.get('tools', [])).union(*(h.get('hands', []) for h in data.get('holds', [])))
        self.holds = [{'object': h['object'], 'type': h.get('type'), 'hands': list(h['hands']),
                       'grips': {hand: np.array(grip, float) for hand, grip in (h.get('grips') or {}).items()}}
                      for h in data.get('holds', []) if h.get('grips')]
        self.objects = {name: state for name, state in self.objects.items() if any(h['object'] == name
                                                                                   for h in self.holds)}
        self._publish_state()

    def _publish_state(self) -> None:
        """The tools whose jog moves an object, latched (names.JOG_STATE)."""
        state = {hand: {'object': h['object'], 'hands': list(h['hands'])}
                 for h in self.holds for hand in h['hands'] if hand in self.tools}
        text = __import__('json').dumps(state, sort_keys=True)
        if text != getattr(self, '_state_text', None):
            self._state_text = text
            self.mode_state.publish(self.String(data=text))

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
            self.get_logger().warning(f'jog {tool}: no transform {self.base} -> flange: {error}',
                                      throttle_duration_sec=2)
            return None
        return pose_matrix(transform.translation, transform.rotation)

    def _step(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if self.last_time is None else min(max(now - self.last_time, 0.0), MAX_STEP_S)
        self.last_time = now
        moving: dict[str, list] = {}   # object -> [(received, twist)] of its holding hands' fresh commands
        swinging: set = set()          # hands whose elbow swings: their object stands still
        for tool, jog in self.tools.items():
            with self.lock:
                fresh = time.monotonic() - jog['received'] < FRESH_S
                # the arm's joints only: the state topic can carry joints IK's model lacks (a sim's base joints)
                twist = jog['twist']
                current = {n: v for n, v in self.positions.items() if jog['joints'].fullmatch(n)}
                elbow = jog['elbow'] if time.monotonic() - jog['elbow_received'] < FRESH_S else 0.0
            if abs(elbow) > 1e-9:
                swinging.add(tool)
                self._swing(tool, jog, current, elbow * dt)
                continue
            if abs(elbow) <= 1e-9:
                jog['null'] = None
            if not fresh or twist is None or not any(abs(v) > 1e-9 for part in twist for v in part):
                jog['target'], jog['blocked'] = None, False  # the next jog starts from where the arm is
                continue
            hold = self._hold_of(tool)
            if hold is not None:     # its twist moves the object (below, once for all its hands)
                moving.setdefault(hold['object'], []).append((jog['received'], twist))
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
        for hold in self.holds:
            commands = moving.get(hold['object'])
            if not commands or swinging & set(hold['hands']):
                self.objects.pop(hold['object'], None)   # the next jog starts from where the object is
                continue
            self._object_step(hold, max(commands, key=lambda c: c[0])[1], dt)

    def _hold_of(self, tool: str) -> dict | None:
        return next((h for h in self.holds if tool in h['hands']), None)

    def _attached(self, hold: dict) -> list:
        """The held object's collision shapes as an AttachedCollisionObject on its first hand's flange link, for one
        validity request (not for the live scene); [] if its type has none."""
        from .collision import HoldCollision, shapes_of, touch_links

        kind = hold.get('type')
        if kind not in self.shape_cache:
            try:
                self.shape_cache[kind] = shapes_of(self.store.object(kind).get('collision'))
            except (KeyError, ValueError, TypeError, AttributeError):
                self.shape_cache[kind] = []
        shapes = self.shape_cache[kind]
        hand = next((h for h in hold['hands'] if h in hold['grips']), None)
        if not shapes or hand is None:
            return []
        return [HoldCollision._attached(hold['object'], self.ik.tools[hand]['link'], shapes,
                                        self._tool_matrix(hand) @ np.linalg.inv(hold['grips'][hand]),
                                        touch_links(self.robot_data, hold['hands']))]

    def _tool_matrix(self, tool: str) -> np.ndarray:
        """Flange -> TCP of ``tool`` (profile tool_frames)."""
        from .ik import rpy_matrix
        frame = self.ik.tools[tool]
        return rpy_matrix(frame.get('xyz', [0, 0, 0]), frame.get('rpy', [0, 0, 0]))

    def _object_pose(self, hold: dict) -> np.ndarray | None:
        """The held object's pose (base frame) from the live flange of its first hand."""
        hand = next((h for h in hold['hands'] if h in hold['grips'] and h in self.tools), None)
        flange = self._flange(hand) if hand else None
        return None if flange is None else flange @ self._tool_matrix(hand) @ np.linalg.inv(hold['grips'][hand])

    def _object_step(self, hold: dict, twist, dt: float) -> None:
        """One jog step of the held object: the twist moves it about its own origin, every hand follows. A two-hand
        hold steps atomically: if one hand cannot follow, or the combined state collides, nothing moves."""
        from .chain import pose_error

        name, hands = hold['object'], [h for h in hold['hands'] if h in hold['grips'] and h in self.tools]
        if not hands:
            return
        state = self.objects.setdefault(name, {'target': None, 'blocked': False})
        if state['target'] is None:
            state['target'] = self._object_pose(hold)
            if state['target'] is None:
                return
        target = integrate(state['target'], twist[0], twist[1], dt)
        goals = {hand: target @ hold['grips'][hand] @ np.linalg.inv(self._tool_matrix(hand)) for hand in hands}
        with self.lock:
            positions = dict(self.positions)
        joints, reached, held = {}, {}, {}
        if all(self.tools[hand].get('chain') is not None for hand in hands):
            for hand in hands:
                jog = self.tools[hand]
                if any(n not in positions for n in jog['names']):
                    return
                arm, held[hand] = jog['chain'].solve_step(positions, goals[hand], jog['names'], self.limits)
                joints.update(arm)
                tip = jog['chain'].fk({**positions, **arm})
                error = pose_error(goals[hand], tip)
                reached[hand] = (np.linalg.norm(error[:3]) < FOLLOW_M and np.linalg.norm(error[3:]) < FOLLOW_RAD, tip)
            if len(hands) > 1 and not all(ok for ok, _ in reached.values()):
                lagging = ' and '.join(h for h, (ok, _) in reached.items() if not ok)
                self._object_blocked(state, f'jog {name}: stopped, {lagging} cannot follow (limit or out of reach), '
                                            'the hands stay together')
                return
            group = self.ik.groups[hands[0]] if len(hands) == 1 else ''
            if not self._valid(group, joints, self._attached(hold)):
                self._object_blocked(state, f'jog {name}: stopped, the next step is in collision')
                return
        else:   # IK fallback without URDF chains: every hand's step must succeed
            for hand in hands:
                arm = self.ik.step(hand, goals[hand], {n: v for n, v in positions.items()
                                                       if self.tools[hand]['joints'].fullmatch(n)})
                if arm is None:
                    self._object_blocked(state, f'jog {name}: stopped, {hand} cannot follow (joint limit, collision '
                                                'or out of reach)')
                    return
                joints.update(arm)
        for hand, limit in held.items():
            self._held_notice(hand, self.tools[hand], limit)
        if len(hands) == 1 and hands[0] in reached and not reached[hands[0]][0]:
            # the hand could not follow (a limit, out of reach): the target stays with the hand
            state['target'] = reached[hands[0]][1] @ self._tool_matrix(hands[0]) @ np.linalg.inv(
                hold['grips'][hands[0]])
        else:
            state['target'] = target
        state['blocked'] = False
        self.command.publish(self.JointState(name=list(joints), position=list(joints.values())))

    def _object_blocked(self, state: dict, text: str) -> None:
        """Nothing moves: say so once (log and operator notice)."""
        if not state['blocked']:
            self.get_logger().warning(text)
            self.notice.publish(self.String(data=__import__('json').dumps({'text': text, 'ok': False})))
        state['blocked'] = True

    def _swing(self, tool: str, jog: dict, current: dict, angle: float) -> None:
        """Turn the swivel joint by ``angle`` and put the flange back where it is (the elbow moves, the TCP not)."""
        if not current:      # a held object stays too: the swing only moves the elbow
            return
        swivel = jog['swivel']
        if jog.get('chain') is None and swivel not in current:
            reason = (f'swivel joint {swivel!r} (profile arms.{tool}.swivel) is not one of its joints' if swivel else
                      f'the robot profile has no arms.{tool}.swivel (the joint the swing turns)')
            self.get_logger().warning(f'jog {tool}: no elbow swing without the URDF chain: {reason}',
                                      throttle_duration_sec=5)
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
            # the self-motion has no sign of its own: positive turns the swivel joint up (without one: the way it
            # started), and it keeps its way
            index = jog['names'].index(swivel) if swivel in jog['names'] else None
            if index is not None and abs(direction[index]) > SWIVEL_SIGN:
                direction = direction * np.sign(direction[index])
            elif jog['null'] is not None and direction @ jog['null'] < 0:
                direction = -direction
            jog['null'] = direction
            self._chain_step(tool, jog, jog['target'], null=direction * angle)
            return
        joints = self.ik.step(tool, jog['target'], {**current, swivel: current[swivel] + angle})
        if joints is None:
            if not jog['blocked']:
                self.get_logger().warning(
                    f'jog {tool}: elbow stopped (joint limit, collision or no way to keep the TCP)')
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
