"""The executive: runs a plan step by step and applies the decision rules. No ROS dependency.

The ROS task handler node wraps this class. It provides a ``Dispatcher`` that sends steps to skills, feeds skill
results back through ``on_result``, calls ``tick`` periodically for timeouts, and forwards operator commands.

Decision rules (per step, from the mission):
  SUCCEEDED                 merge ``data['world_state']`` into the world state, continue
  FAILED / TIMEOUT          retry while attempts <= retries, then apply on_failure
  REJECTED                  apply on_failure directly (invalid arguments do not fix themselves)
  on_failure: skip          log and continue with the next step
  on_failure: abort         the mission ends as FAILED
  on_failure: retry/replan  start the step again (retry) or ask the planner (replan); both count toward max_replans
Operator: pause cancels the running skill and later re-runs the same step; abort cancels and ends as ABORTED.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Any, Protocol

from . import placeholders
from .plan import Plan, Step
from .planner import Planner
from .speed import WorkClock
from .world_state import WorldState

# skill data kept in step_finished (smoke test)
EPISODE_DATA = ('path', 'mode', 'choice', 'plan_s', 'plan_points', 'torso')


class Outcome(Enum):
    SUCCEEDED = 0
    FAILED = 1
    TIMEOUT = 2
    CANCELED = 3
    REJECTED = 4


class State(Enum):
    IDLE = 0
    RUNNING = 1
    PAUSED = 2
    SUCCEEDED = 3
    FAILED = 4
    ABORTED = 5


ACTIVE = (State.RUNNING, State.PAUSED)


class Dispatcher(Protocol):
    def dispatch(self, token: int, step: Step, attempt: int) -> None:
        """Start ``step`` (arguments resolved); report back via ``Executive.on_result(token, ...)``."""

    def cancel(self, token: int) -> None:
        """Cancel the step started with ``token``. The skill must still report a result (normally CANCELED)."""


@dataclass
class Status:
    state: State = State.IDLE
    plan_id: str = ''
    goal: str = ''
    cycle: int = 0
    step_index: int = -1
    step_count: int = 0
    step: Step | None = None
    attempt: int = 0
    step_status: str = ''
    step_progress: float = -1.0
    step_started: float = 0.0
    last_error: str = ''
    log: list[str] = field(default_factory=list)


class Executive:
    def __init__(self, planner: Planner, dispatcher: Dispatcher, world: WorldState,
                 validate: Callable[[Plan], list[str]] = lambda plan: [],
                 clock: Callable[[], float] = time.monotonic,
                 on_status: Callable[[Status], None] = lambda status: None,
                 on_episode: Callable[[dict], None] = lambda event: None,
                 timeout_grace_s: float = 2.0, speed: Callable[[], float] | None = None,
                 annotate: Callable[[Step], tuple[str | None, list[str]]] | None = None):
        self.planner, self.dispatcher, self.world, self.validate = planner, dispatcher, world, validate
        self.clock, self.on_status, self.on_episode = clock, on_status, on_episode
        self.timeout_grace_s = timeout_grace_s
        self.speed = speed             # the global override as a fraction: step timeouts run on work time
        self.annotate = annotate       # (kind, resources) of a step once its placeholders are filled in
        self._work: WorkClock | None = None
        self.status = Status()
        self.plan: Plan | None = None
        self._lock = threading.RLock()
        self._token = 0
        self._running = False          # a skill is executing for the current token
        self._interrupt: str | None = None  # 'pause' | 'abort' | 'timeout' while waiting for a CANCELED result
        self._replans: dict[str, int] = {}

    # ---- operator / node API -------------------------------------------------
    def start(self, start_step: str = '') -> Status:
        """Plan cycle 1 and start it, at step ``start_step`` (an id) if given: the earlier steps are left out, the
        robot and objects are taken as they are. Later cycles start at their first step. Returns the status (FAILED
        with a reason if the plan is invalid or has no such step)."""
        with self._lock:
            if self.status.state in ACTIVE:
                raise RuntimeError('a mission is already active')
            self.status = Status(state=State.RUNNING)
            self._begin_cycle(1, start_step)
            return self.status

    def pause(self) -> bool:
        with self._lock:
            if self.status.state != State.RUNNING:
                return False
            if self._running:
                self._interrupt = 'pause'
                self.dispatcher.cancel(self._token)
            else:
                self._set_state(State.PAUSED)
            return True

    def resume(self) -> bool:
        with self._lock:
            if self.status.state != State.PAUSED or self._interrupt:
                return False
            self.status.state = State.RUNNING
            self._log('resumed')
            self._run_current(fresh=False)
            return True

    def abort(self, reason: str = 'aborted by operator') -> bool:
        with self._lock:
            if self.status.state not in ACTIVE:
                return False
            self.status.last_error = reason
            if self._running:
                self._interrupt = 'abort'
                self.dispatcher.cancel(self._token)
            else:
                self._finish(State.ABORTED, reason)
            return True

    def on_feedback(self, token: int, progress: float, status: str) -> None:
        with self._lock:
            if token == self._token and self._running:
                self.status.step_progress, self.status.step_status = progress, status
                self.on_status(self.status)

    def tick(self) -> None:
        """Call periodically: enforces step timeouts (skill timeout plus a grace period), on work time if ``speed``."""
        with self._lock:
            step = self.status.step
            if not (self._running and step and self._interrupt is None):
                return
            elapsed = self._work(self.clock()) if self._work else self.clock() - self.status.step_started
            if elapsed > step.timeout_s + self.timeout_grace_s:
                self._interrupt = 'timeout'
                self.dispatcher.cancel(self._token)

    def on_result(self, token: int, outcome: Outcome, message: str = '', data: dict | None = None) -> None:
        with self._lock:
            if token != self._token or not self._running:
                return  # stale result from an earlier attempt
            self._running = False
            interrupt, self._interrupt = self._interrupt, None
            if interrupt == 'timeout' and outcome == Outcome.CANCELED:
                outcome, message = Outcome.TIMEOUT, f'no result within {self.status.step.timeout_s:g} s'
            speed = (data or {}).get('speed') or (self._work.stats() if self._work else None)
            summary = {k: data[k] for k in EPISODE_DATA if k in data} if data else {}
            self._episode('step_finished', outcome=outcome.name, message=message,
                          **({'speed': speed} if speed else {}), **({'data': summary} if summary else {}))
            if interrupt == 'abort':
                self._finish(State.ABORTED, self.status.last_error or 'aborted')
            elif interrupt == 'pause':
                self._set_state(State.PAUSED, 'paused; the step will run again on resume')
            elif outcome == Outcome.SUCCEEDED:
                self.world.update((data or {}).get('world_state', {}))
                self._advance()
            else:
                self._handle_failure(outcome, message)

    # ---- internals --------------------------------------------------------------
    def _begin_cycle(self, cycle: int, start_step: str = '') -> None:
        plan = self.planner.make_plan(self.world.snapshot(), cycle)
        if plan is None:  # the status keeps showing the last cycle that ran
            self._finish(State.SUCCEEDED, 'mission complete' if cycle > 1 else 'nothing to do')
            return
        self.status.cycle, self._replans = cycle, {}
        problems = self.validate(plan)
        if problems:
            self._finish(State.FAILED, 'plan rejected: ' + '; '.join(problems))
            return
        self.plan = plan
        self.status.plan_id, self.status.goal, self.status.step_count = plan.plan_id, plan.goal, len(plan.steps)
        try:
            first = plan.index_of(start_step) if start_step else 0
        except KeyError as error:
            self._finish(State.FAILED, str(error.args[0]))
            return
        self._log(f'cycle {cycle}: plan {plan.plan_id} with {len(plan.steps)} steps'
                  + (f', starting at {start_step} (step {first + 1})' if first else ''))
        self.status.step_index = first
        self._run_current(fresh=True)

    def _run_current(self, fresh: bool) -> None:
        step = self.plan.steps[self.status.step_index]
        if fresh:
            self.status.attempt = 1
        try:
            resolved = replace(step, args=placeholders.resolve(step.args, self.world.snapshot()))
        except placeholders.PlaceholderError as error:
            self.status.step = step
            self._handle_failure(Outcome.REJECTED, str(error.args[0]))
            return
        if self.annotate is not None and resolved.args != step.args:   # a placeholder hid the kind until now
            resolved.kind, resolved.resources = self.annotate(resolved)
        self._token += 1
        self._running = True
        self.status.step, self.status.step_started = resolved, self.clock()
        self._work = WorkClock(self.status.step_started, self.speed) if self.speed else None
        self.status.step_status, self.status.step_progress = 'dispatched', -1.0
        self._episode('step_started')
        self.on_status(self.status)
        self.dispatcher.dispatch(self._token, resolved, self.status.attempt)

    def _advance(self) -> None:
        if self.status.step_index + 1 < len(self.plan.steps):
            self.status.step_index += 1
            self._run_current(fresh=True)
        else:
            self._begin_cycle(self.status.cycle + 1)

    def _handle_failure(self, outcome: Outcome, message: str) -> None:
        step = self.plan.steps[self.status.step_index]
        reason = f'{step.id} ({step.skill}) {outcome.name.lower()}: {message}'
        self.status.last_error = reason
        self._log(reason)
        if outcome != Outcome.REJECTED and self.status.attempt <= step.retries:
            self.status.attempt += 1
            self._log(f'{step.id}: retry {self.status.attempt - 1} of {step.retries}')
            self._run_current(fresh=False)
            return
        policy = step.on_failure
        if policy == 'skip':
            self._log(f'{step.id}: skipped')
            self._advance()
        elif policy in ('retry', 'replan'):
            used = self._replans.get(step.id, 0)
            if used >= step.max_replans:
                self._finish(State.FAILED, f'{reason}; max_replans ({step.max_replans}) reached')
                return
            self._replans[step.id] = used + 1
            if policy == 'retry':
                self._log(f'{step.id}: starting again ({used + 1}/{step.max_replans})')
                self._run_current(fresh=True)
                return
            result = self.planner.replan(self.plan, self.status.step_index, reason, self.world.snapshot())
            if result is None:
                self._finish(State.FAILED, f'{reason}; planner could not replan')
                return
            plan, index = result
            problems = self.validate(plan)
            if problems:
                self._finish(State.FAILED, 'replanned plan rejected: ' + '; '.join(problems))
                return
            self.plan, self.status.step_index, self.status.step_count = plan, index, len(plan.steps)
            self._log(f'replanned: continue at {plan.steps[index].id} ({used + 1}/{step.max_replans})')
            self._run_current(fresh=True)
        else:
            self._finish(State.FAILED, reason)

    def _finish(self, state: State, message: str) -> None:
        if state in (State.FAILED, State.ABORTED):
            self.status.last_error = message
        self.status.step_index, self.status.step = -1, None
        self._set_state(state, message)

    def _set_state(self, state: State, message: str = '') -> None:
        self.status.state = state
        self._log(message or state.name.lower())
        self.on_status(self.status)

    def _log(self, message: str) -> None:
        self.status.log = (self.status.log + [message])[-50:]

    def _episode(self, kind: str, **extra: Any) -> None:
        step = self.status.step
        self.on_episode({'event': kind, 'time': self.clock(), 'plan_id': self.status.plan_id,
                         'cycle': self.status.cycle, 'step_id': step.id, 'skill': step.skill,
                         'instruction': step.instruction, 'args': step.args, 'attempt': self.status.attempt, **extra})
