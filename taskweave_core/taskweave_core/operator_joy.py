"""Joystick buttons for the operator, configured in operator.yaml:

    joy:
      topic: /joy
      pause_resume_button: 7          # pause or resume the running mission
      abort_button: 6                 # abort the running mission
      actions:                        # manual skill calls, only while no mission is running
        - {button: 0, skill: gripper, args: {side: both, action: close}}
        - {button: 11, skill: move_to_pose, args: {pose: lift_high}}

Acts on button press (rising edge). Manual actions go straight to the skill's RunSkill action; they are ignored
while a mission runs or is paused, so manual and mission commands never compete. A new press of the same action
while it is still running is ignored. Without a ``joy`` entry the node logs that and stays idle.

Pad modes (optional): drive, or one per tool that jogs that tool's TCP (taskweave_core.jog, sent as the flange's
velocity in the base frame to e.g. taskweave_skills' tcp_jog):

    joy:
      drive_topic: /joy_drive         # the pad goes on here only in drive mode (the base teleop listens here)
      drive_button: 8                 # back to drive
      arms:                           # tool -> mode button and where its jog twists go (taskweave_skills tcp_jog)
        left:  {button: 13, twist: /taskweave/jog/left}
      jog: {enable_button: 4, fast_button: 5, tilt_axis: 5, speed: 0.05, fast_speed: 0.15, turn: 0.3, fast_turn: 0.6}
      actions:
        - {button: 0, skill: gripper, args: {side: both, action: close}, arm_args: {side: '{arm}'}}

In a tool mode the base gets no pad input, actions use ``arm_args`` ('{arm}' = the tool), and the TCP moves only
while the enable button is held. Holding the elbow trigger too (``jog.elbow_axis``, default L2) swings that arm's elbow
with the right stick left/right while the TCP stays (``elbow_turn`` / ``fast_elbow_turn`` rad/s, sent as
std_msgs/Float64 on ``<twist>/elbow``). Modes switch only while no mission runs; a starting mission puts the pad back to
drive. The mode is published on names.OPERATOR_MODE. Tool links and the base frame come from the robot profile.

Teaching object points (optional): jog the tools onto the object, then press the teach button. It stores the TCP
of the tool being jogged (in drive mode: of every tool) as ``point`` (``<instance>/<point>``) through the teach
node. ``point`` is read from operator.yaml at each press, so it can be changed while the system runs:

    joy:
      teach: {button: 10, point: bay_2_stack_1/rails_top}
"""

from __future__ import annotations

import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from taskweave_interfaces.action import RunSkill
from taskweave_interfaces.msg import TaskStatus
from taskweave_interfaces.srv import TeachPoint

from . import jog, names, tf_util
from .config import ConfigStore

DRIVE = 'drive'
JOG_DEFAULTS = {'enable_button': 4, 'fast_button': 5, 'tilt_axis': 5, 'elbow_axis': 4, 'elbow_turn': 0.6,
                'fast_elbow_turn': 1.5, 'speed': 0.05, 'fast_speed': 0.15,
                'turn': 0.3, 'fast_turn': 0.6}

BUSY = (TaskStatus.RUNNING, TaskStatus.PAUSED)


class OperatorJoy(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'operator_joy'), **kwargs)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('robot', '')
        self.store = store = ConfigStore(self.get_parameter('config_dir').value)
        config = store.operator().get('joy')
        self.state = TaskStatus.IDLE
        self.previous: list[int] = []
        self.actions: list[dict] = []
        self.running: dict[int, object] = {}  # action index -> goal handle future while it runs
        self.mode = DRIVE
        self.arms: dict = {}
        self.moving = self.swinging = False  # jog commands sent; swinging: the elbow trigger is held
        self.drive_pub = self.mode_pub = None
        if not config:
            self.get_logger().info('no joy section in operator.yaml; operator buttons disabled')
            return
        self._setup_modes(config, store)
        self.buttons = {'toggle': config.get('pause_resume_button'), 'abort': config.get('abort_button')}
        services = (('pause', names.PAUSE), ('resume', names.RESUME), ('abort', names.ABORT))
        self.triggers = {name: self.create_client(Trigger, service) for name, service in services}
        self.teach_button = config.get('teach', {}).get('button')
        self.teach_client = self.create_client(TeachPoint, names.TEACH_POINT) if self.teach_button is not None else None
        self.actions = list(config.get('actions', []))
        self.skill_clients = {skill: ActionClient(self, RunSkill, names.skill_action(skill))
                              for skill in {a['skill'] for a in self.actions}}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskStatus, names.STATUS, self._status, latched)
        self.create_subscription(Joy, config['topic'], self._joy, 10)
        manual = ', '.join(f'{a["button"]}={a["skill"]}' for a in self.actions) or 'none'
        self.get_logger().info(f'operator buttons on {config["topic"]}: {self.buttons}; manual actions: {manual}')

    def _setup_modes(self, config: dict, store: ConfigStore) -> None:
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, names.OPERATOR_MODE, latched)
        self.drive_pub = self.create_publisher(Joy, config['drive_topic'], 10) if config.get('drive_topic') else None
        self.drive_button = config.get('drive_button')
        self.jog = {**JOG_DEFAULTS, **config.get('jog', {})}
        arms = config.get('arms', {})
        if arms:
            from geometry_msgs.msg import TwistStamped
            from std_msgs.msg import Float64

            self.Float64 = Float64
            robot = self.get_parameter('robot').value
            if not robot:
                raise ValueError('operator.yaml joy.arms needs the robot parameter (tool links, base frame)')
            profile = store.robot(robot)
            self.base_frame = profile.base['frame']
            self.TwistStamped = TwistStamped
            self.tf, self.tf_listener = tf_util.buffer(self)  # cleared when sim time jumps back
            for tool, spec in arms.items():
                frame = profile.data.get('tool_frames', {}).get(tool)
                if frame is None:
                    raise ValueError(f'operator.yaml joy.arms.{tool}: no tool_frames.{tool} in the robot profile')
                self.arms[tool] = {'button': spec['button'], 'frame': frame,
                                   'pub': self.create_publisher(TwistStamped, spec['twist'], 10),
                                   'elbow': self.create_publisher(Float64, f'{spec["twist"]}/elbow', 10)}
        self.mode_pub.publish(String(data=DRIVE))

    def _set_mode(self, mode: str) -> None:
        if mode == self.mode:
            return
        self._stop_jog()
        self.mode = mode
        self.mode_pub.publish(String(data=mode))
        self.get_logger().info(f'pad mode: {mode}')

    def _status(self, msg) -> None:
        self.state = msg.state
        if self.state in BUSY and self.mode != DRIVE:
            self._set_mode(DRIVE)  # a mission owns the robot

    def _modes(self, msg: Joy, buttons: list[int]) -> None:
        if self.state not in BUSY:
            if self.arms and self._pressed(buttons, self.drive_button):
                self._set_mode(DRIVE)
            for tool, arm in self.arms.items():
                if self._pressed(buttons, arm['button']):
                    self._set_mode(tool)
        if self.mode == DRIVE:
            if self.drive_pub is not None:
                self.drive_pub.publish(msg)
        else:
            self._jog(msg)

    def _jog(self, msg: Joy) -> None:
        j, buttons, axes = self.jog, list(msg.buttons), list(msg.axes)

        def held(index):
            return index is not None and index < len(buttons) and bool(buttons[index])

        if not held(j['enable_button']):
            self._stop_jog()
            return
        fast = held(j['fast_button'])
        arm = self.arms[self.mode]
        elbow = jog.elbow_rate(axes, j['elbow_axis'], j['fast_elbow_turn' if fast else 'elbow_turn'])
        if j['elbow_axis'] < len(axes) and axes[j['elbow_axis']] > 0.5:  # elbow trigger: swing it, the TCP stays
            self._send(arm, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            arm['elbow'].publish(self.Float64(data=float(elbow)))
            self.moving = self.swinging = True
            return
        if self.swinging:  # elbow trigger released: stop the swing before the sticks move the TCP again
            arm['elbow'].publish(self.Float64(data=0.0))
            self.swinging = False
        tilt = j['tilt_axis'] < len(axes) and axes[j['tilt_axis']] > 0.5
        linear, angular = jog.stick_velocity(axes, tilt, j['fast_speed' if fast else 'speed'],
                                             j['fast_turn' if fast else 'turn'])
        link = arm['frame']['link']
        try:
            q = self.tf.lookup_transform(self.base_frame, link, rclpy.time.Time()).transform.rotation
        except Exception as error:  # noqa: BLE001 - tf2 raises several types; jogging just pauses
            self.get_logger().warning(f'jog {self.mode}: no transform {self.base_frame} -> {link}: {error}',
                                      throttle_duration_sec=2.0)
            return
        v, w = jog.flange_twist(linear, angular, jog.quaternion_rotation(q.x, q.y, q.z, q.w), arm['frame'])
        self._send(arm, v, w)
        self.moving = any(abs(c) > 1e-9 for c in (*v, *w))

    def _send(self, arm: dict, linear, angular) -> None:
        twist = self.TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = (float(v) for v in linear)
        twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z = (float(v) for v in angular)
        arm['pub'].publish(twist)

    def _stop_jog(self) -> None:
        if self.moving and self.mode in self.arms:
            self._send(self.arms[self.mode], (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            self.arms[self.mode]['elbow'].publish(self.Float64(data=0.0))
        self.moving = self.swinging = False

    def _pressed(self, buttons: list[int], index: int | None) -> bool:
        if index is None or index >= len(buttons):
            return False
        before = self.previous[index] if index < len(self.previous) else 0
        return bool(buttons[index]) and not before

    def _joy(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        if self._pressed(buttons, self.buttons['abort']):
            self._call('abort')
        elif self._pressed(buttons, self.buttons['toggle']):
            self._call('resume' if self.state == TaskStatus.PAUSED else 'pause')
        if self.mode_pub is not None:
            self._modes(msg, buttons)
        if self._pressed(buttons, self.teach_button):
            self._teach()
        for index, action in enumerate(self.actions):
            if self._pressed(buttons, action['button']):
                self._run(index, self._for_mode(action))
        self.previous = buttons

    def _for_mode(self, action: dict) -> dict:
        """In a tool mode an action's arm_args replace its args ('{arm}' = the tool)."""
        if self.mode == DRIVE or 'arm_args' not in action:
            return action
        args = {k: v.replace('{arm}', self.mode) if isinstance(v, str) else v for k, v in action['arm_args'].items()}
        return {**action, 'args': {**action.get('args', {}), **args}}

    def _teach(self) -> None:
        point = self.store.operator().get('joy', {}).get('teach', {}).get('point')
        if self.state in BUSY:
            self.get_logger().warning('teach ignored: a mission is running or paused')
            return
        if not point:
            self.get_logger().warning('teach: no joy.teach.point in operator.yaml')
            return
        if not self.teach_client.service_is_ready():
            self.get_logger().warning('teach: teach node not available')
            return
        tools = [] if self.mode == DRIVE else [self.mode]
        self.get_logger().info(f'teach {point} ({", ".join(tools) or "all tools"})')
        request = TeachPoint.Request(name=point, kind='object_point', tools=tools)
        self.teach_client.call_async(request).add_done_callback(self._taught)

    def _taught(self, future) -> None:
        response = future.result()
        if response.success:
            self.get_logger().info(f'taught {response.message} -> {response.file}')
        else:
            self.get_logger().warning(f'teach failed: {response.message}')

    def _call(self, command: str) -> None:
        client = self.triggers[command]
        if not client.service_is_ready():
            self.get_logger().warning(f'{command}: task handler not available')
            return
        self.get_logger().info(f'operator: {command}')
        client.call_async(Trigger.Request())

    def _run(self, index: int, action: dict) -> None:
        label = f'{action["skill"]} {json.dumps(action.get("args", {}))}'
        if self.state in BUSY:
            self.get_logger().warning(f'manual {label} ignored: a mission is running or paused')
            return
        if index in self.running:
            self.get_logger().info(f'manual {label} still running')
            return
        client = self.skill_clients[action['skill']]
        if not client.server_is_ready():
            self.get_logger().warning(f'manual {label}: skill not available')
            return
        self.get_logger().info(f'manual {label}')
        goal = RunSkill.Goal(step_id=f'manual_{index}', skill=action['skill'], instruction=f'operator: {label}',
                             args_json=json.dumps(action.get('args', {})), timeout_s=0.0)
        future = client.send_goal_async(goal)
        self.running[index] = future
        future.add_done_callback(lambda f: self._accepted(index, label, f))

    def _accepted(self, index: int, label: str, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.running.pop(index, None)
            return
        handle.get_result_async().add_done_callback(lambda f: self._done(index, label, f))

    def _done(self, index: int, label: str, future) -> None:
        self.running.pop(index, None)
        result = future.result().result
        outcome = ['succeeded', 'failed', 'timeout', 'canceled', 'rejected'][result.outcome]
        text = f'manual {label}: {outcome}{": " + result.message if result.message else ""}'
        if outcome == 'succeeded':  # one call site per severity: rclpy refuses to mix them
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorJoy()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
