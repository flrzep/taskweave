"""The tool path of the arm move that is about to run, published for viewers (a simulator's viewport, an HMI).

Topic ``names.PLANNED_PATH/<hand>`` (nav_msgs/Path, latched): the TCP positions along the move, in the robot's base
frame (``header.frame_id``), published when the move starts; a Path without poses when it has ended. Only hands whose
TCP moves are published. Positions only: the orientations are left at identity.

The path is what the move will command: the planner's trajectory, or the straight joint-space move. Nothing reads it
back; a viewer that is not there costs one FK per sample.
"""

from __future__ import annotations

import math

from taskweave_core import names
from taskweave_core.node_cache import for_node

SAMPLES = 40             # states along a move (a longer planned trajectory is thinned to about this many)
MIN_LENGTH_M = 0.01      # a hand whose TCP travels less than this is not shown


def sample_states(start: dict, goal: dict, path=None, samples: int = SAMPLES, ease=lambda fraction: fraction) -> list:
    """Every joint's position at points along a move: from ``start`` (every joint) straight to ``goal``, or along
    ``path`` (motion_plan.Trajectory) with the goal joints it does not move going straight alongside. ``ease``: the
    time law of the straight part (fraction -> fraction), as the move plays it."""
    def straight(joints: dict, fraction: float) -> dict:
        s = ease(max(0.0, min(1.0, fraction)))
        return {name: start[name] + (target - start[name]) * s for name, target in joints.items() if name in start}

    if path is None:
        return [{**start, **straight(goal, k / samples)} for k in range(samples + 1)]
    rest = {name: value for name, value in goal.items() if name not in path.names}
    count = len(path.times)
    stride = max(1, count // samples)
    picked = sorted({*range(0, count, stride), count - 1})
    duration = path.times[-1]
    return [{**start, **straight(rest, path.times[i] / duration if duration else 1.0),
             **dict(zip(path.names, path.points[i], strict=True))} for i in picked]


def tool_points(states: list, fk, hands: list, min_length: float = MIN_LENGTH_M) -> dict:
    """{hand: [(x, y, z), ...]} of the TCP at each of ``states``, for the hands that travel at least ``min_length``.
    ``fk(state, hands)`` gives {hand: 4x4} or None (no kinematics): then nothing is shown."""
    poses = [fk(state, hands) for state in states]
    if not poses or any(pose is None for pose in poses):
        return {}
    found = {}
    for hand in hands:
        points = [tuple(float(v) for v in pose[hand][:3, 3]) for pose in poses]
        if sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False)) >= min_length:
            found[hand] = points
    return found


class PlannedPath:
    """One latched publisher per hand, shared by the motion skills of a node."""

    _instances: dict = {}

    @classmethod
    def for_node(cls, node, frame: str) -> PlannedPath:
        return for_node(cls._instances, node, lambda n: cls(n, frame))

    def __init__(self, node, frame: str):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path
        from rclpy.qos import DurabilityPolicy, QoSProfile

        self.node, self.frame = node, frame
        self.Path, self.PoseStamped = Path, PoseStamped
        self.qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publishers: dict = {}
        self.shown: set = set()

    def _publish(self, hand: str, points: list) -> None:
        if hand not in self.publishers:
            self.publishers[hand] = self.node.create_publisher(self.Path, f'{names.PLANNED_PATH}/{hand}', self.qos)
        message = self.Path()
        message.header.frame_id = self.frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        for x, y, z in points:
            pose = self.PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.publishers[hand].publish(message)

    def show(self, points: dict) -> None:
        """Publish ``points`` ({hand: [(x, y, z)]}, base frame); a hand shown before and not in it is cleared."""
        for hand in self.shown - set(points):
            self._publish(hand, [])
        for hand, path in points.items():
            self._publish(hand, path)
        self.shown = set(points)

    def clear(self) -> None:
        self.show({})
