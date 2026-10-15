"""Planners turn a goal into a plan. The task handler only depends on the ``Planner`` interface.

``MissionFilePlanner`` reads ``missions/<name>.yaml``. A VLM planner will implement the same interface,
using the goal, the world state and the skill catalog, and must return a ``Plan`` that passes validation.

A mission's ``facts`` are worked out from the placed objects (with their run-time object state) when each cycle is
planned and filled into its steps, so they stay the same for the whole cycle while the cycle changes the objects:

    facts:
      next_stack: {fullest: frame_type2, among: "bay_*_stack_2_front"}   # most items left; ties: first by name
    steps:
    - {instruction: Pick a frame, include: pick_frame, with: {frame: "{next_stack}"}}

When a fact finds nothing (every matching stack is empty), the mission is complete.
"""

from __future__ import annotations

import fnmatch
import itertools
from abc import ABC, abstractmethod
from collections.abc import Mapping
from typing import Any

from .config import ConfigError, ConfigStore
from .plan import Plan, _fill


class Planner(ABC):
    @abstractmethod
    def make_plan(self, world: Mapping[str, Any], cycle: int) -> Plan | None:
        """Plan for the given cycle (1-based), or None when the goal is complete."""

    def replan(self, plan: Plan, failed_index: int, reason: str, world: Mapping[str, Any]) -> tuple[Plan, int] | None:
        """After a failure: a plan and the step index to continue from, or None if replanning is not possible."""
        return None


class MissionFilePlanner(Planner):
    """Plans from a mission file. Loop and stop rules come from the mission itself."""

    def __init__(self, store: ConfigStore, mission: str, goal: str | None = None, robot: str = ''):
        self.store, self.mission_name, self.goal, self.robot = store, mission, goal, robot
        self.mission = store.expanded_mission(mission)  # validates now, so errors surface before anything starts
        if self.mission.get('params'):
            raise ConfigError(store.root / f'missions/{mission}.yaml', 'takes parameters '
                              f'({", ".join(self.mission["params"])}): a block for other missions to include, not run')
        self._ids = itertools.count(1)

    def make_plan(self, world: Mapping[str, Any], cycle: int) -> Plan | None:
        loop = self.mission.get('loop', 'once')
        if cycle > 1 and loop == 'once':
            return None
        if self.mission.get('max_cycles') and cycle > self.mission['max_cycles']:
            return None
        stop_when = self.mission.get('stop_when')
        if stop_when and world.get(stop_when):
            return None
        facts = self.facts()
        if any(value is None for value in facts.values()):
            return None  # e.g. nothing left to pick
        mission = {**self.mission, 'steps': [_fill(step, facts) for step in self.mission['steps']]}
        return Plan.from_mission(mission, f'{self.mission_name}-{next(self._ids):04d}', self.goal, self.robot)

    def facts(self) -> dict[str, Any]:
        """The mission's facts now: {name: value, or None if nothing fits}."""
        return {name: fact_value(self.store, spec) for name, spec in self.mission.get('facts', {}).items()}


def fact_value(store: ConfigStore, spec: Mapping[str, Any]) -> Any:
    """One fact from the placed objects: ``fullest`` = the instance of that type with the most items left (among
    names matching ``among``), or None."""
    candidates = sorted((name, instance.stack_count) for name, instance in store.object_instances().items()
                        if instance.object == spec['fullest'] and fnmatch.fnmatchcase(name, spec.get('among', '*'))
                        and instance.stack_count > 0)
    return max(candidates, key=lambda c: c[1])[0] if candidates else None

    def replan(self, plan: Plan, failed_index: int, reason: str, world: Mapping[str, Any]) -> tuple[Plan, int] | None:
        """Continue from the failed step's ``replan_from`` step (or the failed step itself) with fresh retries."""
        target = plan.steps[failed_index].replan_from
        return plan, plan.index_of(target) if target else failed_index
