"""Torso look-ahead over a held sequence: one torso height per step of a hold window, planned before the first moves.

``torso: auto`` alone chooses each held step's height when the step runs (ik.solve_with_torso); nothing looks at the
steps after it, yet one step's height decides what later ones can do, through the arm branch the steps leave and the
ride (a torso change with the held object riding the column, the arms following). ``TorsoPlan.plan`` takes the window
([WindowStep]: what must hold at each step, predicted from the mission) and the checks (``Checks``: reach bound, a step
at a height, a ride) and returns a height per step, chosen by, in this order:

1. feasible: every step passes at its height from the state the steps before it leave (chained), every ride into a new
   height passes, and ``keep`` / pinned heights are kept;
2. fewest torso changes (a change at the first step counts: the hold exists already);
3. the changes as late as possible: the height is kept while it works, a change happens at the step that needs it;
4. least torso travel;
5. least sum of the steps' own costs (``Check.cost``: arm cost and wrist bend, as solve_with_torso);
6. the lower heights. The same input gives the same plan.

Search: best-first over (step, height) in that order, with lazy checks: a step is checked only when its partial plan is
the best open one (``Check.cost`` must not be negative). So a window the current height serves costs one step check
per step and no ride (the keep check); when the current height fails at a step, changes at that step are tried first,
nearest height first, then one step earlier. A failure (bound, or a ``lasting`` step failure) rules its height out
for that step whatever the state. Heights: the profile grid (ik.torso.candidates) and the current height; grid heights
within half a grid step of the current one are the current one (as solve_with_torso).

No sequence works: the plan fails naming the blocking step (the furthest the search got), the heights it works at and
the constraint that kept the torso off them, with a table per step (heights that worked, reasons for the others). The
blocking step is uncertain (``WindowStep.certain`` False): the plan covers the steps before it instead (``cut``).

``memory`` (kept across plans by the owner, e.g. until a restart) remembers each step and ride check by the step's
``key``, the heights and the start state rounded: a later cycle with the same steps makes no calls for them.

``SkillChecks`` gives the real checks from a carry skill: the reach bound (``IkClient._unreachable``), a step as
solve_with_torso checks one height (``IkClient._torso_height``: IK of every target, collisions, lines walked with
``_walk_lines``, the line after an offset), a ride with ``_state_at`` (held hands re-solved at sampled heights, each
state collision checked).
"""

from __future__ import annotations

import heapq
import itertools
import json
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol

AUTO, KEEP = 'auto', 'keep'


@dataclass
class WindowStep:
    """What must hold at one step of the window. ``torso``: auto (the plan chooses), keep (no change at this step) or
    a height (pinned: ``torso: <number>``, a held pose that sets the torso); ``why`` names a constraint in a failure
    (default ``torso: <value>``). ``certain``: False for a step the prediction is unsure of (after a step it could not
    predict, a window cut short). ``key``: identifies the step's checks in the memory (rounded object pose, grasp, base
    pose; None: never remembered). For ``SkillChecks``: the holding ``hands``, the IK ``targets`` ([{tool: tcp target}],
    the main one first), the straight ``lines`` walked one after the other ([{tool: 4x4 TCP goal}], mode linear), the
    ``ahead`` tools (the line after an offset approach), the ``elbow`` side, the predicted ``base`` pose."""
    id: str
    label: str = ''
    torso: str | float = AUTO
    why: str = ''
    certain: bool = True
    key: str | None = None
    hands: tuple = ()
    targets: list = field(default_factory=list)
    lines: list = field(default_factory=list)
    ahead: dict | None = None
    elbow: str | None = None
    base: Any = None

    def rule(self) -> str:
        if self.why:
            return self.why
        return f'torso: {self.torso:g}' if isinstance(self.torso, int | float) else f'torso: {self.torso}'


@dataclass
class Check:
    """One check's answer: ``ok``, the predicted joints after it (``state``), the step's own ``cost`` (>= 0), why it
    failed; ``lasting``: the failure holds at this height whatever the start state (no IK, out of reach)."""
    ok: bool
    state: dict | None = None
    cost: float = 0.0
    reason: str = ''
    lasting: bool = False


class Checks(Protocol):
    """The feasibility checks the plan calls. ``state``: the predicted joints (torso included) before the check."""

    def bound(self, step: WindowStep, height: float) -> str | None:
        """Why no arm can reach the step's targets at ``height`` (free: no IK call), or None."""

    def step(self, step: WindowStep, height: float, state: dict) -> Check:
        """The step at ``height`` (the torso already there) from ``state``: its targets, lines, next line."""

    def ride(self, step: WindowStep, state: dict, height: float) -> Check:
        """The torso from ``state``'s height to ``height`` at the start of ``step``, the held object riding."""


@dataclass
class Plan:
    """``TorsoPlan.plan``'s answer. ``heights``: step id -> height (the steps planned, in order); ``changes``:
    [{at, from, to}]; ``states``: step id -> predicted joints after the step (seeds for its move); ``checks``: calls
    made (rides and steps; memory hits in ``remembered``); ``cut``: why the plan ends before the window does;
    ``message`` and ``table`` (per step: heights that worked, reasons for the others) on failure."""
    ok: bool
    window: list = field(default_factory=list)
    heights: dict = field(default_factory=dict)
    changes: list = field(default_factory=list)
    states: dict = field(default_factory=dict)
    message: str = ''
    table: list = field(default_factory=list)
    cut: str = ''
    checks: int = 0
    remembered: int = 0
    planned_s: float = 0.0

    def report(self) -> dict:
        """``data.torso_plan`` of the step that planned."""
        out = {'window': self.window, 'heights': {k: round(v, 4) for k, v in self.heights.items()},
               'changes': self.changes, 'source': 'memory' if self.checks == 0 and self.remembered else 'planned',
               'checks': self.checks, 'planned_s': round(self.planned_s, 2)}
        out |= {'cut': self.cut} if self.cut else {}
        out |= {'message': self.message, 'table': self.table} if not self.ok else {}
        return out


@dataclass(eq=False)
class _Node:
    k: int                 # steps done
    h: float               # the torso height they leave
    state: dict
    key: tuple
    parent: _Node | None = None


class TorsoPlan:
    """Plans the torso heights of a hold window (module doc). ``grid``: the profile's ik.torso.candidates;
    ``tolerance``: heights closer than this are the same (a pin at the current height is no change); ``memory``: the
    cross-cycle memory (kept by the owner; the newest ``kept`` entries); ``digits``: the state rounding of its keys;
    ``max_checks``: give up after this many calls (None: never)."""

    def __init__(self, grid, tolerance: float = 1e-3, memory: dict | None = None, kept: int = 512, digits: int = 3,
                 max_checks: int | None = None):
        self.grid = sorted({round(float(h), 6) for h in grid})
        self.spacing = min((b - a for a, b in zip(self.grid, self.grid[1:], strict=False)), default=0.0)
        self.tolerance, self.kept, self.digits, self.max_checks = tolerance, kept, digits, max_checks
        self.memory = {} if memory is None else memory

    def heights(self, start: float) -> list[float]:
        """The heights the plan chooses from: the grid, a grid height within half a grid step of ``start`` replaced
        by it."""
        return sorted({h for h in self.grid if abs(h - start) >= self.spacing / 2} | {round(float(start), 6)})

    def plan(self, steps: list[WindowStep], start: float, state: dict, checks: Checks, name: str = '',
             should_stop: Callable[[], bool] = lambda: False) -> Plan:
        """The heights for ``steps`` from the torso at ``start`` and the joints ``state`` (module doc)."""
        return _Search(self, steps, round(float(start), 6), checks, name, should_stop).run(state)

    def remember(self, key: str, check: Check) -> None:
        self.memory.pop(key, None)                 # newest last: the oldest go beyond ``kept``
        self.memory[key] = check
        for old in list(self.memory)[:-self.kept] if len(self.memory) > self.kept else ():
            self.memory.pop(old, None)


class _Search:
    def __init__(self, owner: TorsoPlan, steps, start, checks, name, should_stop):
        self.o, self.steps, self.start, self.checks = owner, list(steps), start, checks
        self.name, self.should_stop = name, should_stop
        self.all = owner.heights(start)
        self.calls = self.remembered = 0
        self.works = [set() for _ in self.steps]           # per step: heights that passed (from some state)
        self.out = [{} for _ in self.steps]                # per step: height -> first reason it failed
        self.dead = {}                                     # (step, height) -> reason, whatever the state
        self.bounds = {}                                   # (step, height) -> bound reason or None
        self.began = time.monotonic()

    # ---- the search

    def run(self, state: dict) -> Plan:
        if not self.steps:
            return self._done(Plan(True))
        heap, order, closed, first = [], itertools.count(), {}, {}
        n = len(self.steps)
        root = _Node(0, self.start, state, (0, (), 0.0, 0.0, ()))
        heapq.heappush(heap, (root.key, next(order), root, None))
        while heap:
            if self.should_stop():
                return self._done(Plan(False, message='torso plan stopped'))
            key, _, node, height = heapq.heappop(heap)
            if height is None:                             # a checked node: close it, open its next step
                tag = (node.k, node.h)
                if tag in closed:
                    continue
                closed[tag] = node
                first.setdefault(node.k, node)
                if node.k == n:
                    return self._done(self._success(node))
                for h in self._allowed(node.k, node.h):
                    if (node.k + 1, h) not in closed and (node.k, h) not in self.dead:
                        heapq.heappush(heap, (self._edge_key(node, h), next(order), node, h))
                continue
            if (node.k + 1, height) in closed:             # an unchecked edge whose end is settled already
                continue
            if self.o.max_checks is not None and self.calls >= self.o.max_checks:
                return self._done(self._gave_up(first))
            check = self._try(node, height)
            if check is not None:
                done = _Node(node.k + 1, height, check.state, (*key[:3], round(key[3] + check.cost, 9), key[4]), node)
                heapq.heappush(heap, (done.key, next(order), done, None))
        return self._done(self._failure(first))

    def _allowed(self, k: int, h: float) -> list[float]:
        rule = self.steps[k].torso
        if rule == KEEP:
            return [h]
        if isinstance(rule, int | float):
            return [h] if abs(float(rule) - h) <= self.o.tolerance else [round(float(rule), 6)]
        return sorted(set(self.all) | {h})

    @staticmethod
    def _edge_key(node: _Node, h: float) -> tuple:
        """(changes, change steps latest first (negated), travel, cost, heights): smaller is better (module doc)."""
        changes, late, travel, cost, path = node.key
        if h != node.h:
            changes, late, travel = changes + 1, (-node.k, *late), round(travel + abs(h - node.h), 6)
        return changes, late, travel, cost, (*path, h)

    def _try(self, node: _Node, h: float) -> Check | None:
        """The edge: ride to ``h`` if it changes, then step ``node.k`` at ``h``. The passing check or None."""
        k, step = node.k, self.steps[node.k]
        reason = self._bound(k, h)
        if reason:
            return None
        state = node.state
        if h != node.h:
            ride = self._call('ride', step, node.h, h, state, lambda: self.checks.ride(step, state, h))
            if not ride.ok:
                self.out[k].setdefault(h, f'ride {node.h:g}->{h:g}: {ride.reason}')
                return None
            state = ride.state
        check = self._call('step', step, h, h, state, lambda: self.checks.step(step, h, state))
        if not check.ok:
            self.out[k].setdefault(h, check.reason)
            if check.lasting:
                self.dead[(k, h)] = check.reason
            return None
        self.works[k].add(h)
        return check

    def _bound(self, k: int, h: float) -> str | None:
        if (k, h) not in self.bounds:
            bound = getattr(self.checks, 'bound', None)
            reason = bound(self.steps[k], h) if bound is not None else None
            self.bounds[(k, h)] = reason
            if reason:
                self.out[k].setdefault(h, reason)
                self.dead[(k, h)] = reason
        return self.bounds[(k, h)]

    def _call(self, kind: str, step: WindowStep, h_from: float, h: float, state: dict, call) -> Check:
        key = None if step.key is None else json.dumps(
            [kind, step.key, round(h_from, 4), round(h, 4), self._state_key(state)], sort_keys=True, default=str)
        if key is not None and key in self.o.memory:
            self.remembered += 1
            return self.o.memory[key]
        self.calls += 1
        check = call()
        if key is not None:
            self.o.remember(key, check)
        return check

    def _state_key(self, state: dict):
        return {n: round(float(v), self.o.digits) if isinstance(v, int | float) else v
                for n, v in sorted((state or {}).items())}

    # ---- the answers

    def _path(self, node: _Node) -> list[_Node]:
        nodes = []
        while node.parent is not None:
            nodes.append(node)
            node = node.parent
        return nodes[::-1]

    def _success(self, node: _Node, cut: str = '') -> Plan:
        plan = Plan(True, cut=cut)
        for done in self._path(node):
            step = self.steps[done.k - 1]
            plan.heights[step.id], plan.states[step.id] = done.h, done.state
            if done.h != done.parent.h:
                plan.changes.append({'at': step.id, 'from': round(done.parent.h, 4), 'to': round(done.h, 4)})
        return plan

    def _done(self, plan: Plan) -> Plan:
        plan.checks, plan.remembered, plan.planned_s = self.calls, self.remembered, time.monotonic() - self.began
        plan.window = [s.id for s in self.steps]
        return plan

    def _head(self) -> str:
        return (f'torso plan over {self.name or "the hold"} ({self.steps[0].id}..{self.steps[-1].id}, '
                f'{len(self.steps)} moves): ')

    def _gave_up(self, first: dict) -> Plan:
        d = max(first)
        where = self.steps[min(d, len(self.steps) - 1)].id
        return Plan(False, message=f'{self._head()}gave up after {self.calls} checks (max_checks), got to {where}',
                    table=self._table())

    def _failure(self, first: dict) -> Plan:
        """The furthest step the search reached blocks: every height from the best plan up to it, constraints left
        out, says what it would take; the message names the constraint or the reasons."""
        d = max(first)                                   # steps 0..d-1 passed somehow, step d nowhere
        node, step = first[d], self.steps[d]
        if not step.certain:
            plan = self._success(node, cut=f'{step.id} {step.label}'.strip() + ' (uncertain) works at no height from '
                                 'the plan: planned up to the step before it')
            return plan
        for h in self.all:                               # what would work there (rides from the best plan's height)
            if h not in self.works[d] and (d, h) not in self.dead:
                self._try(node, h)
        works = sorted(self.works[d])
        what = f'{step.id} {step.label}'.strip()
        if works and isinstance(step.torso, int | float):
            text = f'{step.id} {step.rule()} pins {float(step.torso):g}, but {what} works at {self._bands(works)} only'
        elif works and step.torso == KEEP:
            j = d
            while j >= 0 and self.steps[j].torso == KEEP:
                j -= 1
            run = self.steps[j + 1:d + 1]
            source = f'set by {self.steps[j].id} {self.steps[j].rule()}' if j >= 0 and \
                self.steps[j].torso != AUTO else f'from {self.steps[j].id}' if j >= 0 else 'the height at the start'
            text = (f'{", ".join(s.id for s in run)} {" / ".join(sorted({s.rule() for s in run}))} '
                    f'hold{"s" if len(run) == 1 else ""} {node.h:g} ({source}) to {step.id}, which works at '
                    f'{self._bands(works)} only')
        elif works:
            text = f'{what} works at {self._bands(works)} only'
        else:
            text = f'{what} works at no height ({self._reasons(d)})'
        return Plan(False, message=self._head() + 'no heights work: ' + text, table=self._table())

    def _table(self) -> list[dict]:
        return [{'id': s.id, 'label': s.label, 'torso': s.torso, 'works': [round(h, 4) for h in sorted(self.works[k])],
                 'out': [{'height': round(h, 4), 'reason': r} for h, r in sorted(self.out[k].items())
                         if h not in self.works[k]]}
                for k, s in enumerate(self.steps)]

    def _runs(self, heights: list[float]) -> list[list[float]]:
        """``heights`` in runs of neighbours on this plan's height list."""
        index = {h: i for i, h in enumerate(self.all)}
        runs = []
        for h in sorted(heights, key=lambda h: (index.get(h, math.inf), h)):
            if runs and h in index and index.get(runs[-1][-1]) == index[h] - 1:
                runs[-1].append(h)
            else:
                runs.append([h])
        return runs

    def _bands(self, heights: list[float]) -> str:
        return ', '.join(f'{r[0]:g}' if len(r) == 1 else f'{r[0]:g}-{r[-1]:g}' for r in self._runs(heights))

    def _reasons(self, k: int) -> str:
        """Step ``k``'s failures, neighbouring heights with the same reason as one band."""
        out = sorted((h, r) for h, r in self.out[k].items() if h not in self.works[k])
        parts = []
        for h, reason in out:
            if parts and parts[-1][1] == reason and self._runs([parts[-1][0][-1], h]) == [[parts[-1][0][-1], h]]:
                parts[-1][0].append(h)
            else:
                parts.append(([h], reason))
        return '; '.join(f'{self._bands(hs)}: {reason}' for hs, reason in parts) or 'not tried'


class SkillChecks:
    """The real ``Checks`` from a carry skill (gripper_motion / object_motion): ``skill.ik`` (an IkClient with
    ik.torso and the URDF chains), ``skill._state_at``, ``skill._walk_lines``, ``skill._ahead_check``. A step's joints
    after it: the end of its lines (mode linear), else the IK solution of its main target."""

    def __init__(self, skill, should_stop: Callable[[], bool] = lambda: False):
        self.skill, self.ik, self.should_stop = skill, skill.ik, should_stop
        self.joint = self.ik.torso['joint']

    def bound(self, step: WindowStep, height: float) -> str | None:
        return self.ik._unreachable(step.targets, height, allow_flip=False) if step.targets else None

    def step(self, step: WindowStep, height: float, state: dict) -> Check:
        from .ik import Samples, TorsoError

        if not step.targets and not step.lines:            # nothing to reach (a base move): the arms stay
            return Check(True, {**state, self.joint: height})
        samples, end = Samples(step.targets or [step.lines[-1]]), {}
        if step.lines:
            def verify(h):
                problem, end['state'] = self.skill._walk_lines(step.lines, state, h)
                return problem
            samples.verify = verify
        if step.ahead:
            samples.ahead = self.skill._ahead_check(step.ahead, state)
        try:      # one height of solve_with_torso: the torso is there already (no torso cost, no second check)
            choice = self.ik._torso_height(samples, height, {}, state, self.joint, height, 0.0, self.should_stop,
                                           step.elbow, False)
        except TorsoError as error:
            reason = str(error)
            return Check(False, reason=reason, lasting=not reason.startswith(('line:', 'next move:')))
        after = end.get('state') or {**state, self.joint: height, **choice.solution}
        return Check(True, after, choice.cost)

    def ride(self, step: WindowStep, state: dict, height: float) -> Check:
        """The column moves in samples ik.torso.line_sample_m apart; at each the held hands follow (``_state_at``
        from the hands where the last sample left them) and the state is collision checked."""
        import re

        from .ik import LINE_SAMPLE_M

        hands = list(step.hands) or list(self.ik.groups)
        sample = float(self.ik.torso.get('line_sample_m', LINE_SAMPLE_M))
        begin = float(state[self.joint])
        parts = max(1, math.ceil(abs(height - begin) / sample - 1e-9))
        patterns = [re.compile(self.ik.joint_pattern.format(tool=t)) for t in hands]
        for i in range(1, parts + 1):
            h = begin + (height - begin) * i / parts
            moved, problem = self.skill._state_at(hands, state, h, poses=self.ik.tcps_at(hands, state))
            if moved is None:
                return Check(False, reason=problem)
            arms = {n: v for n, v in moved.items() if any(p.fullmatch(n) for p in patterns)}
            if not self.ik._state_valid({self.joint: h, **self.ik.idle_joints(hands, moved), **arms}, hands[0]):
                return Check(False, reason=f'collision at {h:.3f}')
            state = moved
        return Check(True, {**state, self.joint: height})
