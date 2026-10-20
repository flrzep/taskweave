"""Base class for skills. A skill executes one plan step; it never knows about plans or other steps.

Write a skill by subclassing ``Skill``:

    class Wait(Skill):
        name = 'wait'
        description = 'Wait for a number of seconds.'
        args_schema = {'type': 'object', 'required': ['seconds'],
                       'properties': {'seconds': {'type': 'number', 'minimum': 0}}}
        achieves = ['the given time has passed']

        def execute(self, ctx, args):
            if not ctx.sleep(args['seconds']):
                return ctx.interrupted()
            return SkillResult.succeeded()

Skills get the config store, the robot profile and their own parameters (``skills/<name>.yaml``) in ``setup``.
They address hardware only through robot roles, e.g. ``self.robot.base['cmd_vel']``.
Point arguments are marked in the schema with ``"x-taskweave-point": "base_target" | "object_point"``.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .config import ConfigStore, RobotProfile
from .executive import Outcome
from .speed import DEFAULT_SLEW_PER_S, SpeedOverride, WorkClock


@dataclass
class SkillResult:
    outcome: Outcome
    message: str = ''
    data: dict = field(default_factory=dict)

    @classmethod
    def succeeded(cls, message: str = '', **data: Any) -> SkillResult:
        return cls(Outcome.SUCCEEDED, message, data)

    @classmethod
    def failed(cls, message: str, **data: Any) -> SkillResult:
        return cls(Outcome.FAILED, message, data)

    @classmethod
    def rejected(cls, message: str) -> SkillResult:
        return cls(Outcome.REJECTED, message)


class SkillContext:
    """What a running skill can use: feedback, cancel and timeout checks, and a time source.

    ``speed``: the node's global override (taskweave_core.speed.SpeedOverride); none = always 100 %. The timeout runs
    on work time (``work_now``), so a step at 10 % gets ten times its nominal timeout."""

    def __init__(self, now: Callable[[], float], timeout_s: float, is_canceled: Callable[[], bool],
                 publish_feedback: Callable[[float, str], None], instruction: str = '', logger=None,
                 speed: SpeedOverride | None = None):
        self.now, self.timeout_s, self.instruction, self.logger = now, timeout_s, instruction, logger
        self._is_canceled, self._publish = is_canceled, publish_feedback
        self.speed = speed
        self.started = now()
        self.work = WorkClock(self.started, speed.fraction if speed else None)

    @property
    def canceled(self) -> bool:
        return self._is_canceled()

    @property
    def expired(self) -> bool:
        return self.timeout_s > 0 and self.work_now() > self.timeout_s

    @property
    def should_stop(self) -> bool:
        return self.canceled or self.expired

    def elapsed(self) -> float:
        return self.now() - self.started

    def work_now(self) -> float:
        """Work seconds since the step started: node time integrated at the effective speed override."""
        return self.work(self.now())

    def work_clock(self, slew_per_s: float = DEFAULT_SLEW_PER_S) -> WorkClock:
        """A new work clock from now, following the override at ``slew_per_s`` (phase-clock playback of a move)."""
        return WorkClock(self.now(), self.speed.fraction if self.speed else None, slew_per_s)

    def feedback(self, progress: float = -1.0, status: str = '') -> None:
        self._publish(progress, status)

    def sleep(self, seconds: float, poll_s: float = 0.02) -> bool:
        """Sleep ``seconds`` of node time (sim time if enabled). Returns False if canceled or timed out first."""
        end = self.now() + seconds
        while self.now() < end:
            if self.should_stop:
                return False
            time.sleep(poll_s)
        return not self.should_stop

    def interrupted(self, message: str = '') -> SkillResult:
        """The right result after ``should_stop``: CANCELED or TIMEOUT."""
        if self.canceled:
            return SkillResult(Outcome.CANCELED, message or 'canceled')
        return SkillResult(Outcome.TIMEOUT, message or f'timed out after {self.timeout_s:g} s')


class Skill:
    name: str = ''
    description: str = ''
    args_schema: dict = {'type': 'object'}
    needs: list[str] = []
    achieves: list[str] = []
    implementation: str = 'script'
    default_timeout_s: float = 60.0

    startup_params: tuple = ()   # skill yaml keys bound to a publisher, client or subscription made in setup

    def setup(self, node, config: ConfigStore, robot: RobotProfile, params: dict) -> None:
        """Create publishers/subscribers and read parameters (``configure``). Called once when the skill server
        starts."""
        self.node, self.config, self.robot = node, config, robot
        self.configure(params)

    def configure(self, params: dict) -> None:
        """Everything derived from the yaml parameters and the profile (``self.robot``), nothing that holds a ROS
        resource. Called by ``setup``, and again by the skill server's reload_config between goals with the new
        values: so keep it free of side effects on ROS and re-runnable. A key that a resource of ``setup`` was made
        from belongs in ``startup_params``."""
        self.params = params

    def execute(self, ctx: SkillContext, args: dict) -> SkillResult:
        raise NotImplementedError

    def stop(self) -> None:
        """Bring the hardware this skill commands to a safe stop. Called after cancel, timeout and errors."""
