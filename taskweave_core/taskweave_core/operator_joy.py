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
"""

from __future__ import annotations

import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from taskweave_interfaces.action import RunSkill
from taskweave_interfaces.msg import TaskStatus

from . import names
from .config import ConfigStore

BUSY = (TaskStatus.RUNNING, TaskStatus.PAUSED)


class OperatorJoy(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'operator_joy'), **kwargs)
        self.declare_parameter('config_dir', '')
        config = ConfigStore(self.get_parameter('config_dir').value).operator().get('joy')
        self.state = TaskStatus.IDLE
        self.previous: list[int] = []
        self.actions: list[dict] = []
        self.running: dict[int, object] = {}  # action index -> goal handle future while it runs
        if not config:
            self.get_logger().info('no joy section in operator.yaml; operator buttons disabled')
            return
        self.buttons = {'toggle': config.get('pause_resume_button'), 'abort': config.get('abort_button')}
        services = (('pause', names.PAUSE), ('resume', names.RESUME), ('abort', names.ABORT))
        self.triggers = {name: self.create_client(Trigger, service) for name, service in services}
        self.actions = list(config.get('actions', []))
        self.skill_clients = {skill: ActionClient(self, RunSkill, names.skill_action(skill))
                              for skill in {a['skill'] for a in self.actions}}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskStatus, names.STATUS, lambda msg: setattr(self, 'state', msg.state), latched)
        self.create_subscription(Joy, config['topic'], self._joy, 10)
        manual = ', '.join(f'{a["button"]}={a["skill"]}' for a in self.actions) or 'none'
        self.get_logger().info(f'operator buttons on {config["topic"]}: {self.buttons}; manual actions: {manual}')

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
        for index, action in enumerate(self.actions):
            if self._pressed(buttons, action['button']):
                self._run(index, action)
        self.previous = buttons

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
