"""Joystick buttons for the operator: pause/resume toggle and abort, configured in operator.yaml.

    joy: {topic: /joy, pause_resume_button: 7, abort_button: 6}

Acts on button press (rising edge). Without a ``joy`` entry the node logs that and stays idle.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

from taskweave_interfaces.msg import TaskStatus

from . import names
from .config import ConfigStore


class OperatorJoy(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'operator_joy'), **kwargs)
        self.declare_parameter('config_dir', '')
        config = ConfigStore(self.get_parameter('config_dir').value).operator().get('joy')
        self.state = TaskStatus.IDLE
        self.previous: list[int] = []
        if not config:
            self.get_logger().info('no joy section in operator.yaml; operator buttons disabled')
            return
        self.buttons = {'toggle': config.get('pause_resume_button'), 'abort': config.get('abort_button')}
        self.clients = {name: self.create_client(Trigger, service)
                        for name, service in (('pause', names.PAUSE), ('resume', names.RESUME), ('abort', names.ABORT))}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskStatus, names.STATUS, lambda msg: setattr(self, 'state', msg.state), latched)
        self.create_subscription(Joy, config['topic'], self._joy, 10)
        self.get_logger().info(f'operator buttons on {config["topic"]}: {self.buttons}')

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
        self.previous = buttons

    def _call(self, command: str) -> None:
        client = self.clients[command]
        if not client.service_is_ready():
            self.get_logger().warning(f'{command}: task handler not available')
            return
        self.get_logger().info(f'operator: {command}')
        client.call_async(Trigger.Request())


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
