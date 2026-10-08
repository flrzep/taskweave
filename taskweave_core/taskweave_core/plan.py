"""Plan model: what the planner hands to the task handler. No ROS dependency."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field

DEFAULT_TIMEOUT_S = 60.0
DEFAULT_RETRIES = 0
DEFAULT_ON_FAILURE = 'abort'
DEFAULT_MAX_REPLANS = 1


@dataclass
class Step:
    """One fully specified step: mission defaults are already applied."""

    id: str
    instruction: str
    skill: str
    args: dict = field(default_factory=dict)
    timeout_s: float = DEFAULT_TIMEOUT_S
    retries: int = DEFAULT_RETRIES
    on_failure: str = DEFAULT_ON_FAILURE
    replan_from: str | None = None
    max_replans: int = DEFAULT_MAX_REPLANS


@dataclass
class Plan:
    plan_id: str
    goal: str
    robot: str
    steps: list[Step]
    loop: str = 'once'                 # once | forever
    max_cycles: int | None = None      # with loop: forever, stop after this many cycles
    stop_when: str | None = None       # with loop: forever, stop when this world-state key is truthy

    def index_of(self, step_id: str) -> int:
        for index, step in enumerate(self.steps):
            if step.id == step_id:
                return index
        raise KeyError(f'plan {self.plan_id} has no step {step_id!r}')

    @classmethod
    def from_mission(cls, mission: dict, plan_id: str, goal: str | None = None) -> Plan:
        """Build a plan from a validated mission dict, applying the mission's defaults to every step."""
        defaults = mission.get('defaults', {})
        steps = []
        for number, raw in enumerate(mission['steps'], start=1):
            steps.append(Step(
                id=raw.get('id', f's{number}'),
                instruction=raw['instruction'],
                skill=raw['skill'],
                args=copy.deepcopy(raw.get('args', {})),
                timeout_s=float(raw.get('timeout_s', defaults.get('timeout_s', DEFAULT_TIMEOUT_S))),
                retries=int(raw.get('retries', defaults.get('retries', DEFAULT_RETRIES))),
                on_failure=raw.get('on_failure', defaults.get('on_failure', DEFAULT_ON_FAILURE)),
                replan_from=raw.get('replan_from'),
                max_replans=int(raw.get('max_replans', defaults.get('max_replans', DEFAULT_MAX_REPLANS))),
            ))
        return cls(plan_id=plan_id, goal=goal or mission['goal'], robot=mission['robot'], steps=steps,
                   loop=mission.get('loop', 'once'), max_cycles=mission.get('max_cycles'),
                   stop_when=mission.get('stop_when'))
