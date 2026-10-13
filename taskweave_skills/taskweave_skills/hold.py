"""What the robot holds, shared by the joint skills of one skill server (and published for others).

go_to_point reports when its tools reached an object point exactly (no offset). Closing the grippers of those tools
then makes a hold: the object, the point, and each TCP's pose relative to the object, measured at that moment.
While a hold lasts, free-arm motions (move_to_pose, go_to_point) refuse and move_object moves the object with all
holding tools locked to it; opening one of the holding grippers releases it. A simulator reset (sim time jumps back:
Stop/Play, restart) ends the hold too: the world starts over with open hands.

The hold is published latched on /taskweave/holding (std_msgs/String, JSON; ``{}`` when nothing is held):
``{"object": instance, "point": "<instance>/<point>", "tools": [...], "links": [flange links]}``. A simulator can
attach the object to the first link there (a "magic" grasp) and a TCP jog node can refuse to pull the hands apart.
"""

from __future__ import annotations

import json
import threading

import numpy as np

TOPIC = '/taskweave/holding'
RESET_JUMP_S = 0.5  # sim time going back by more than this is a simulator reset


class HoldState:
    _instances: dict = {}

    @classmethod
    def for_node(cls, node, robot) -> HoldState:
        if id(node) not in cls._instances:
            cls._instances[id(node)] = cls(node, robot)
        return cls._instances[id(node)]

    def __init__(self, node, robot):
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String

        from taskweave_core import tf_util

        self.node, self.String = node, String
        self.base = robot.base['frame']
        self.tool_frames = robot.data.get('tool_frames', {})
        self.tf, self.tf_listener = tf_util.buffer(node)  # cleared when sim time jumps back
        self.lock = threading.Lock()
        self.reached: dict | None = None   # {point, instance, tools, object_in_base}
        self.hold: dict | None = None      # {object, point, tools, tcp_in_object, object_in_base}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, TOPIC, latched)
        self._publish()
        from rclpy.clock import JumpThreshold
        from rclpy.duration import Duration
        threshold = JumpThreshold(min_forward=None, min_backward=Duration(seconds=-RESET_JUMP_S), on_clock_change=True)
        self.jump_handle = node.get_clock().create_jump_callback(threshold, post_callback=self._on_reset)

    def _on_reset(self, _jump) -> None:
        with self.lock:
            hold = self.hold
            self.hold, self.reached = None, None
        if hold is not None:
            self.node.get_logger().warning(f'sim time jumped back: hold of {hold["object"]} ended')
            self._publish()

    # ---- poses ------------------------------------------------------------------
    def transform(self, parent: str, child: str) -> np.ndarray:
        """4x4 pose of ``child`` in ``parent`` from TF (latest); raises LookupError."""
        import rclpy

        from .ik import pose_matrix
        try:
            t = self.tf.lookup_transform(parent, child, rclpy.time.Time()).transform
        except Exception as error:  # noqa: BLE001 - several tf2 exception types
            raise LookupError(f'no transform {parent} -> {child}: {error}') from error
        return pose_matrix(t.translation, t.rotation)

    def tcp_poses(self, tools) -> dict[str, np.ndarray]:
        """Current TCP poses (4x4, base frame) of the given tools: flange from TF * the profile's tool frame."""
        from .ik import rpy_matrix
        result = {}
        for tool in tools:
            frame = self.tool_frames[tool]
            result[tool] = self.transform(self.base, frame['link']) @ rpy_matrix(frame.get('xyz', [0, 0, 0]),
                                                                                 frame.get('rpy', [0, 0, 0]))
        return result

    # ---- state ------------------------------------------------------------------
    def reached_point(self, point: str, instance: str, tools, object_in_base: np.ndarray | None) -> None:
        """The tools are at ``point`` now (None: they left any point)."""
        with self.lock:
            self.reached = None if object_in_base is None else {
                'point': point, 'instance': instance, 'tools': list(tools), 'object_in_base': object_in_base}

    def grasp(self, tools) -> str | None:
        """Grippers of ``tools`` closed: hold the object they are at. Returns its name, or None."""
        with self.lock:
            reached = self.reached
        if reached is None or not set(reached['tools']) <= set(tools):
            return None
        tcp = self.tcp_poses(reached['tools'])
        inverse = np.linalg.inv(reached['object_in_base'])
        with self.lock:
            self.hold = {'object': reached['instance'], 'point': reached['point'], 'tools': reached['tools'],
                         'object_in_base': reached['object_in_base'],
                         'tcp_in_object': {tool: inverse @ pose for tool, pose in tcp.items()}}
        self._publish()
        return reached['instance']

    def release(self, tools) -> str | None:
        """Grippers of ``tools`` opened: the hold ends if one of them held it. Returns the released object."""
        with self.lock:
            hold = self.hold
            if hold is None or not set(hold['tools']) & set(tools):
                return None
            self.hold, self.reached = None, None
        self._publish()
        return hold['object']

    def held(self) -> dict | None:
        with self.lock:
            return None if self.hold is None else dict(self.hold)

    def moved(self, object_in_base: np.ndarray) -> None:
        """The held object was moved (move_object): its pose in the base frame now."""
        with self.lock:
            if self.hold is not None:
                self.hold['object_in_base'] = object_in_base

    def _publish(self) -> None:
        with self.lock:
            hold = self.hold
        data = {} if hold is None else {'object': hold['object'], 'point': hold['point'], 'tools': hold['tools'],
                                        'links': [self.tool_frames[t]['link'] for t in hold['tools']]}
        self.publisher.publish(self.String(data=json.dumps(data)))
