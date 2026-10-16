"""Global speed override (skill API spec section 4): the task handler owns it, skills follow it.

- ``OverrideState``: the handler's rules (range 1-100 %, test mode latch: lower at run start, clamp while on,
  restore at run end only if it lowered it). No ROS.
- ``SpeedOverride``: one per node; follows the handler's latched topic ``/taskweave/speed_override``. Skills never
  read the parameter.
- ``WorkClock``: work time = the integral of the effective override over node time. The effective override follows
  the target at a slew of at most ``slew_per_s`` (fraction/s). Phase-clock playback, scaled stall/settle/stuck checks
  and step timeouts all run on it. At a constant override o it is exactly ``(now - start) * o``, so at 100 % it is
  bit-identical to plain elapsed time.
"""

from __future__ import annotations

import threading
from collections.abc import Callable

DEFAULT_SLEW_PER_S = 2.0   # fraction/s: 10 % -> 100 % in about 0.5 s (spec Q3)
MIN_PERCENT, MAX_PERCENT = 1.0, 100.0
TEST_PERCENT, CAP_PERCENT = 10.0, 50.0
PRESETS = (10, 50, 100)


class WorkClock:
    """Work seconds since ``start``, advancing at the effective override (a fraction, 0.01-1)."""

    def __init__(self, start: float, target: Callable[[], float] | None = None,
                 slew_per_s: float = DEFAULT_SLEW_PER_S):
        self.target = target or (lambda: 1.0)
        self.slew_per_s = slew_per_s
        self.start = start
        self.o = self._target()           # effective override, in effect since the anchor
        self._anchor_t, self._anchor_w = start, 0.0
        self._last_t, self._work = start, 0.0
        self.o_min = self.o
        self._lock = threading.Lock()

    def _target(self) -> float:
        return max(MIN_PERCENT / 100, min(1.0, float(self.target())))

    def __call__(self, now: float) -> float:
        """Work time at ``now`` (node time); then the effective override moves toward the target."""
        with self._lock:
            now = max(now, self._last_t)  # a clock that steps back (sim reset) stands still
            work = self._anchor_w + (now - self._anchor_t) * self.o
            target = self._target()
            if target != self.o:
                step = self.slew_per_s * (now - self._last_t) if self.slew_per_s > 0 else abs(target - self.o)
                o = min(target, self.o + step) if target > self.o else max(target, self.o - step)
                if o != self.o:
                    self._anchor_t, self._anchor_w, self.o = now, work, o
                    self.o_min = min(self.o_min, o)
            self._last_t, self._work = now, work
            return work

    def stats(self) -> dict:
        """{override_min, override_avg} in percent over the time so far (avg = work time / elapsed time)."""
        elapsed = self._last_t - self.start
        avg = self._work / elapsed if elapsed > 0 else self.o
        return {'override_min': round(100 * min(self.o_min, avg), 1), 'override_avg': round(100 * avg, 1)}


class SpeedOverride:
    """The handler's override as seen by one node (latched topic). ``fraction()`` is the target, 0.01-1; 1 until the
    first message. ``clock(start)`` makes a WorkClock that follows it."""

    _instances: dict = {}
    _instances_lock = threading.Lock()

    @classmethod
    def for_node(cls, node) -> SpeedOverride:
        with cls._instances_lock:
            key = id(node)
            if key not in cls._instances or cls._instances[key].node is not node:
                cls._instances[key] = cls(node)
            return cls._instances[key]

    def __init__(self, node=None):
        self.node = node
        self.percent, self.test_mode = MAX_PERCENT, False
        if node is not None and hasattr(node, 'create_subscription'):
            from rclpy.qos import DurabilityPolicy, QoSProfile

            from taskweave_interfaces.msg import SpeedOverride as SpeedOverrideMsg

            from . import names
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(SpeedOverrideMsg, names.SPEED_OVERRIDE, self._message, latched)

    def _message(self, msg) -> None:
        self.percent, self.test_mode = max(MIN_PERCENT, min(MAX_PERCENT, float(msg.percent))), bool(msg.test_mode)

    def fraction(self) -> float:
        return self.percent / 100.0

    def clock(self, start: float, slew_per_s: float = DEFAULT_SLEW_PER_S) -> WorkClock:
        return WorkClock(start, self.fraction, slew_per_s)


class OverrideState:
    """The task handler's override and test mode (not persisted). Every change goes through ``request``."""

    def __init__(self, percent: float = MAX_PERCENT, test_percent: float = TEST_PERCENT,
                 cap_percent: float = CAP_PERCENT):
        self.percent, self.test_mode = float(percent), False
        self.test_percent, self.cap_percent = float(test_percent), float(cap_percent)
        self._lowered: tuple[float, float] | None = None  # (before, lowered to) by test mode at run start

    def request(self, percent: float = 0.0, test_mode: bool | None = None,
                test_percent: float = 0.0) -> tuple[bool, str, float]:
        """Set the override (percent <= 0: unchanged), test mode (None: unchanged) and its start value
        (test_percent <= 0: unchanged). Returns (ok, message, percent now)."""
        if percent > 0 and not MIN_PERCENT <= percent <= MAX_PERCENT:
            return False, f'speed override {percent:g} % is outside {MIN_PERCENT:g}-{MAX_PERCENT:g} %', self.percent
        if test_percent > 0 and not MIN_PERCENT <= test_percent <= MAX_PERCENT:
            return False, f'test speed {test_percent:g} % is outside {MIN_PERCENT:g}-{MAX_PERCENT:g} %', self.percent
        notes = []
        if test_mode is not None and bool(test_mode) != self.test_mode:
            self.test_mode = bool(test_mode)
            notes.append(f'test mode {"on" if self.test_mode else "off"}')
        if test_percent > 0:
            self.test_percent = min(float(test_percent), self.cap_percent)
            if test_percent > self.cap_percent:
                notes.append(f'test speed clamped to the cap {self.cap_percent:g} %')
        if percent > 0:
            if self.test_mode and percent > self.cap_percent:
                percent = self.cap_percent
                notes.append(f'clamped to the test mode cap {self.cap_percent:g} %')
            if float(percent) != self.percent:
                self._lowered = None  # an explicit choice: run end does not restore over it
            self.percent = float(percent)
        return True, '; '.join([f'speed {self.percent:g} %', *notes]), self.percent

    def run_started(self) -> bool:
        """Test mode: lower the override to test_percent if it is above. True if it changed."""
        self._lowered = None
        if self.test_mode and self.percent > self.test_percent:
            self._lowered = (self.percent, self.test_percent)
            self.percent = self.test_percent
            return True
        return False

    def run_ended(self) -> bool:
        """Restore the override test mode lowered at run start (unless it was changed since). True if it changed."""
        lowered, self._lowered = self._lowered, None
        if lowered is not None and self.percent == lowered[1]:
            self.percent = lowered[0]
            return True
        return False

    def as_dict(self) -> dict:
        return {'percent': self.percent, 'test_mode': self.test_mode, 'test_percent': self.test_percent,
                'cap_percent': self.cap_percent, 'presets': list(PRESETS)}
