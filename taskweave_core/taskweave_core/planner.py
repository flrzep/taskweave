"""Planners turn a goal into a plan. The task handler only depends on the ``Planner`` interface.

``MissionFilePlanner`` reads ``missions/<name>.yaml``. A VLM planner will implement the same interface,
using the goal, the world state and the skill catalog, and must return a ``Plan`` that passes validation.
"""

from __future__ import annotations

import itertools
from abc import ABC, abstractmethod
from collections.abc import Mapping
from typing import Any

from .config import ConfigStore
from .plan import Plan


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
        return Plan.from_mission(self.mission, f'{self.mission_name}-{next(self._ids):04d}', self.goal, self.robot)

    def replan(self, plan: Plan, failed_index: int, reason: str, world: Mapping[str, Any]) -> tuple[Plan, int] | None:
        """Continue from the failed step's ``replan_from`` step (or the failed step itself) with fresh retries."""
        target = plan.steps[failed_index].replan_from
        return plan, plan.index_of(target) if target else failed_index
