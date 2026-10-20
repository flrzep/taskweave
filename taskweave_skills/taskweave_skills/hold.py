"""What the robot holds, per hand, shared by the joint skills of one skill server (and published for others).

The registry (skill-api-spec 3.3):

    reached:    {hand: {point, instance, object_in_base}}   move_gripper put the hand on an object point (no offset);
                                                            cleared per hand when that hand moves elsewhere
    holds:      {object: {object, point, hands, tcp_in_object {hand}, object_in_base}}
    hand_holds: {hand: object}

Closing the grippers of hands that are at an object point makes them hold that object: each hand's TCP pose relative to
the object is measured at that moment. A partial close is a valid hold of the hands that closed (a two-hand point
closed with one hand: that hand holds). Opening a hand takes it out of its hold; a hold with no hands is released.
While a hold lasts, free-arm motions (move_to_pose, go_to_point, move_gripper) refuse and move_object moves the object
with all holding tools locked to it. A simulator reset (sim time jumps back: Stop/Play, restart) ends every hold: the
world starts over with open hands.

The old single-hold calls (reached_point, reached_at, grasp, release, held, moved) work on top of the registry: ``held``
is the first hold, ``grasp`` needs every hand of the reached point to close, ``release`` ends a hold whole.

The holds are published latched on /taskweave/holding (std_msgs/String, JSON; ``{}`` when nothing is held): the legacy
keys describe the first hold, ``{"object": instance, "point": "<instance>/<point>", "tools": [...], "links": [flange
links]}``, plus ``"holds": [{"object", "point", "hands", "links"}, ...]`` for all of them. A simulator can attach the
object to the first link there (a "magic" grasp) and a TCP jog node can refuse to pull the hands apart.
"""

from __future__ import annotations

import json
import threading

import numpy as np

from taskweave_core.node_cache import for_node

TOPIC = '/taskweave/holding'


class HoldState:
    _instances: dict = {}

    @classmethod
    def for_node(cls, node, robot) -> HoldState:
        return for_node(cls._instances, node, lambda n: cls(n, robot))

    def __init__(self, node, robot, reset_jump_s: float = 0.5):
        """``reset_jump_s``: sim time going back by more than this is a simulator reset."""
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String

        from taskweave_core import clock_jump, tf_util

        self.node, self.String = node, String
        self.base = robot.base['frame']
        self.tool_frames = robot.data.get('tool_frames', {})
        self.tf, self.tf_listener = tf_util.buffer(node)  # cleared when sim time jumps back
        self.lock = threading.Lock()
        self.reached: dict[str, dict] = {}   # hand -> {point, instance, object_in_base}
        self.holds: dict[str, dict] = {}     # object -> {object, point, hands, tcp_in_object, object_in_base}
        self._first_link: str | None = None  # links[0] of the last published hold (the sim's magic grasp sits there)
        self.on_reset: list = []             # called after a simulator reset (e.g. clear the object state)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = node.create_publisher(String, TOPIC, latched)
        self._publish()
        clock_jump.watch(node, reset_jump_s).callbacks.append(self._on_reset)

    def _on_reset(self, _jump=None) -> None:
        with self.lock:
            holds = list(self.holds)
            self.holds, self.reached = {}, {}
        for name in holds:
            self.node.get_logger().warning(f'sim time jumped back: hold of {name} ended')
        if holds:
            self._publish()
        for callback in list(self.on_reset):
            try:
                callback()
            except Exception as error:  # noqa: BLE001 - a reset hook must not stop the others
                self.node.get_logger().warning(f'sim time jumped back: {error}')

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

    # ---- reached ----------------------------------------------------------------
    def reach(self, hands, point: str, instance: str, object_in_base: np.ndarray) -> None:
        """The ``hands`` are at ``point`` (an object point, no offset) now."""
        with self.lock:
            for hand in hands:
                self.reached[hand] = {'point': point, 'instance': instance, 'object_in_base': object_in_base}

    def leave(self, hands) -> None:
        """The ``hands`` moved elsewhere: no longer at any point."""
        with self.lock:
            for hand in hands:
                self.reached.pop(hand, None)

    def reached_hands(self, hands=None) -> dict[str, dict]:
        """{hand: {point (<instance>/<point>), instance}} for the hands at an object point now (only ``hands``)."""
        with self.lock:
            return {hand: {k: info[k] for k in ('point', 'instance')} for hand, info in self.reached.items()
                    if hands is None or hand in hands}

    def reached_point(self, point: str | None, instance: str | None, tools, object_in_base: np.ndarray | None) -> None:
        """Old call: the tools are at ``point`` now, and no other hand is at a point (None: none is)."""
        with self.lock:
            self.reached = {}
        if object_in_base is not None:
            self.reach(tools, point, instance, object_in_base)

    def reached_at(self) -> dict | None:
        """Old call: where the tools are now: {point (<instance>/<point>), instance, tools} (the hands at the point of
        the first reached hand), or None."""
        with self.lock:
            if not self.reached:
                return None
            first = next(iter(self.reached.values()))
            return {'point': first['point'], 'instance': first['instance'],
                    'tools': [h for h, info in self.reached.items() if info['point'] == first['point']]}

    # ---- holds ------------------------------------------------------------------
    def close_hands(self, hands, types: dict | None = None) -> dict[str, dict]:
        """Grippers of ``hands`` closed: every one of them that is at an object point (and holds nothing yet) holds that
        object now, joining its hold or starting it (``types``: {object: type}, kept).
        Returns {object: {hands: [new hands], new: bool (a new hold)}}."""
        with self.lock:
            todo = {hand: dict(self.reached[hand]) for hand in hands
                    if hand in self.reached and not any(hand in h['hands'] for h in self.holds.values())}
            existing = {name: dict(hold) for name, hold in self.holds.items()}
        if not todo:
            return {}
        tcp = self.tcp_poses(todo)
        result: dict[str, dict] = {}
        with self.lock:
            for hand, info in todo.items():
                name = info['instance']
                hold = self.holds.get(name) or {'object': name, 'point': info['point'], 'hands': [],
                                                'object_in_base': (existing.get(name) or info)['object_in_base'],
                                                'tcp_in_object': {}, 'type': (types or {}).get(name)}
                new = name not in self.holds
                hold['hands'] = [*hold['hands'], hand]
                hold['tcp_in_object'] = {**hold['tcp_in_object'],
                                         hand: np.linalg.inv(hold['object_in_base']) @ tcp[hand]}
                self.holds[name] = hold
                entry = result.setdefault(name, {'hands': [], 'new': new})
                entry['hands'].append(hand)
        self._publish()
        return result

    def open_hands(self, hands) -> dict[str, dict]:
        """Grippers of ``hands`` opened: they leave their holds (and are no longer at a point); a hold left with no
        hands is released. Returns {object: {hands: [the hands that let go], released: bool}}."""
        result: dict[str, dict] = {}
        with self.lock:
            for name, hold in list(self.holds.items()):
                let_go = [hand for hand in hold['hands'] if hand in hands]
                if not let_go:
                    continue
                hold['hands'] = [hand for hand in hold['hands'] if hand not in let_go]
                for hand in let_go:
                    hold['tcp_in_object'].pop(hand, None)
                    self.reached.pop(hand, None)
                released = not hold['hands']
                if released:
                    del self.holds[name]
                result[name] = {'hands': let_go, 'released': released}
        if result:
            self._publish()
        return result

    def grasp(self, tools) -> str | None:
        """Old call: grippers of ``tools`` closed: hold the object they are at, if all of its point's hands closed.
        Returns its name, or None."""
        at = self.reached_at()
        if at is None or not set(at['tools']) <= set(tools):
            return None
        return next(iter(self.close_hands(at['tools'])), None)

    def release(self, tools) -> str | None:
        """Old call: grippers of ``tools`` opened: the hold ends (whole) if one of them held it. Returns the object."""
        with self.lock:
            hold = next((h for h in self.holds.values() if set(h['hands']) & set(tools)), None)
            if hold is None:
                return None
            del self.holds[hold['object']]
            self.reached = {}
        self._publish()
        return hold['object']

    @property
    def hand_holds(self) -> dict[str, str]:
        """{hand: object it holds}."""
        with self.lock:
            return {hand: name for name, hold in self.holds.items() for hand in hold['hands']}

    def held_all(self) -> list[dict]:
        """Every hold as {object, point, hands, tools (= hands), tcp_in_object, object_in_base}."""
        with self.lock:
            return [self._view(hold) for hold in self.holds.values()]

    def hold_of(self, hand: str) -> dict | None:
        """The hold ``hand`` is in, or None."""
        with self.lock:
            return next((self._view(h) for h in self.holds.values() if hand in h['hands']), None)

    def held(self) -> dict | None:
        """Old call: the first hold (see held_all), or None."""
        with self.lock:
            return next((self._view(hold) for hold in self.holds.values()), None)

    def type_of(self, hold: dict, instances: dict) -> str | None:
        """The object type of ``hold``: recorded at the grip, else its placed instance's (``instances``)."""
        instance = instances.get(hold['object'])
        return hold.get('type') or (instance.object if instance else None)

    @staticmethod
    def _view(hold: dict) -> dict:
        return {**hold, 'hands': list(hold['hands']), 'tools': list(hold['hands']),
                'tcp_in_object': dict(hold['tcp_in_object'])}

    @property
    def hold(self) -> dict | None:
        """Old attribute: the first hold; assigning a legacy hold dict (``tools``) replaces all, None clears them."""
        return self.held()

    @hold.setter
    def hold(self, value: dict | None) -> None:
        with self.lock:
            self.holds = {}
            if value is not None:
                hands = list(value.get('hands', value.get('tools', ())))
                self.holds[value['object']] = {**value, 'hands': hands}

    def moved(self, object_in_base: np.ndarray, name: str | None = None) -> None:
        """The held object was moved (move_object): its pose in the base frame now (``name``: default the first)."""
        with self.lock:
            hold = self.holds.get(name) if name else next(iter(self.holds.values()), None)
            if hold is not None:
                hold['object_in_base'] = object_in_base
                for hand in [h for h, info in self.reached.items()
                             if info['instance'] == hold['object'] and h not in hold['hands']]:
                    del self.reached[hand]    # that hand reached the object where it was: it is no longer there

    def _publish(self) -> None:
        with self.lock:
            holds = [self._view(hold) for hold in self.holds.values()]
        data = {}
        if holds:
            first = holds[0]
            data = {'object': first['object'], 'point': first['point'], 'tools': first['hands'],
                    'links': [self.tool_frames[t]['link'] for t in first['hands']],
                    'holds': [{'object': h['object'], **({'type': h['type']} if h.get('type') else {}),
                               'point': h['point'], 'hands': h['hands'],
                               'links': [self.tool_frames[t]['link'] for t in h['hands']]} for h in holds]}
        first_link = data['links'][0] if data and data['links'] else None
        if self._first_link is not None and first_link is not None and first_link != self._first_link:
            # the sim attaches the object to links[0] and ignores payloads while attached: release it first
            self.publisher.publish(self.String(data=json.dumps({})))
        self._first_link = first_link
        self.publisher.publish(self.String(data=json.dumps(data)))
