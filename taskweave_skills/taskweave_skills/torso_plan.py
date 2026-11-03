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

Search: best-first over (step, height, state) in that order, with lazy checks: a step is checked only when its partial
plan is the best open one (``Check.cost`` must not be negative). So a window the current height serves costs one step
check per step and no ride (the keep check); when the current height fails at a step, changes at that step are tried
first, nearest height first, then one step earlier. A node (step, height) is closed once per distinct state the later
checks depend on (``Checks.relevant``; states within ``match`` of each other are the same), up to ``branches``
states: a later check that fails from one arm branch is tried from the others. A failure (bound, or a ``lasting``
step failure) rules its height out for that step whatever the state. Heights: the profile grid (ik.torso.candidates)
and the current height; grid heights within half a grid step of the current one are the current one (as
solve_with_torso).

Checks are local (repeatable). A certain step is rescued once, as soon as every height its rule allows has failed
from the first (best) plan that reached it, before any earlier change is explored (as solve_with_torso falls back to
the global search at the same step): each of those heights is checked again with ``rescue=True`` (``SkillChecks``:
the global IK search), the local checks already made not repeated; a height that passes only so is planned and listed
in ``Plan.rescued``. The rescue keeps ``max_changes``.

``Plan.outcome``: planned, cut, infeasible (the search finished and found nothing), capped (``max_checks``) or stopped
(``should_stop``). ``Plan.certain``: infeasible because a step has every height ruled out by lasting failures (the
reach bound: geometric, whatever the arm state); only then is the hold known to be impossible. A search that ran out
of local options is infeasible but not certain (local IK depends on the seeds, the rescue runs from one plan per
step, ``branches`` / ``max_changes`` cut it); a capped or stopped plan says nothing about the hold.

No sequence works: the plan fails naming the blocking step (the furthest the search got), the heights it works at and
the constraint that kept the torso off them, with a table per step (heights that worked, reasons for the others). The
blocking step is uncertain (``WindowStep.certain`` False): the plan covers the steps before it instead (``cut``).

``memory`` (kept across plans by the owner, e.g. until a restart) remembers each step and ride check by the step's
``key`` and the heights, for the relevant states it was made from (``SkillChecks``: the arms, idle one included, and
the torso; not the wheels); a check from a state within ``match`` of a remembered one is answered from it: a later
cycle with the same steps makes no calls for them.

``SkillChecks`` gives the real checks from a carry skill: the reach bound (``IkClient._unreachable``), a step as
solve_with_torso checks one height (``IkClient.check_height``: IK of every target, collisions, lines walked with
``_walk_lines``, the line after an offset), a ride with ``_state_at`` (held hands re-solved at sampled heights, each
state collision checked).

Left to the integration (T4): refining the height between grid heights at a change step (F17 ``refine``) and seeding
from F17's band memory: both fit in ``Checks.step(rescue=True)`` or as extra ``grid`` heights.
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
    (default ``torso: <value>``). ``drive``: a base move (the torso is kept, as navigate does while holding).
    ``certain``: False for a step the prediction is unsure of (after a step it could not predict, a window cut short).
    ``key``: identifies the step's checks in the memory (rounded object pose, grasp, base pose; None: never
    remembered). For ``SkillChecks``: the holding ``hands``, the IK ``targets`` ([{tool: tcp target}], the main one
    first), the straight ``lines`` walked one after the other ([{tool: 4x4 TCP goal}], mode linear), the ``ahead``
    tools (the line after an offset approach), the ``elbow`` side, other ``fixed`` joints of the step, the predicted
    ``base`` pose."""
    id: str
    label: str = ''
    torso: str | float = AUTO
    why: str = ''
    drive: bool = False
    certain: bool = True
    key: str | None = None
    hands: tuple = ()
    targets: list = field(default_factory=list)
    lines: list = field(default_factory=list)
    ahead: dict | None = None
    elbow: str | None = None
    fixed: dict = field(default_factory=dict)
    base: Any = None

    def rule(self) -> str:
        if self.why:
            return self.why
        return f'torso: {self.torso:g}' if isinstance(self.torso, int | float) else f'torso: {self.torso}'


@dataclass
class Check:
    """One check's answer: ``ok``, the predicted joints after it (``state``), the step's own ``cost`` (>= 0), why it
    failed; ``lasting``: the failure holds at this height whatever the start state (out of reach)."""
    ok: bool
    state: dict | None = None
    cost: float = 0.0
    reason: str = ''
    lasting: bool = False


class Checks(Protocol):
    """The feasibility checks the plan calls. ``state``: the predicted joints (torso included) before the check.
    ``rescue``: the one-time second try before a step is given up (e.g. the global IK search). Optional:
    ``relevant(step, state)``, the part of a state later checks depend on (default: all of it)."""

    def bound(self, step: WindowStep, height: float) -> str | None:
        """Why no arm can reach the step's targets at ``height`` (free: no IK call), or None."""

    def step(self, step: WindowStep, height: float, state: dict, rescue: bool = False) -> Check:
        """The step at ``height`` (the torso already there) from ``state``: its targets, lines, next line."""

    def ride(self, step: WindowStep, state: dict, height: float, rescue: bool = False) -> Check:
        """The torso from ``state``'s height to ``height`` at the start of ``step``, the held object riding."""


@dataclass
class Plan:
    """``TorsoPlan.plan``'s answer. ``heights``: step id -> height (the steps planned, in order); ``changes``:
    [{at, from, to}]; ``states``: step id -> predicted joints after the step (seeds for its move); ``checks``: calls
    made (rides and steps, rescues included; memory hits in ``remembered``); ``rescued``: [{at, height}] that passed
    only with the rescue; ``cut``: why the plan ends before the window does; ``message`` and ``table`` (per step:
    heights that worked, reasons for the others) on failure; ``outcome`` and ``certain``: see the module doc."""
    ok: bool
    outcome: str = 'planned'
    exhaustive: bool = False
    window: list = field(default_factory=list)
    heights: dict = field(default_factory=dict)
    changes: list = field(default_factory=list)
    states: dict = field(default_factory=dict)
    rescued: list = field(default_factory=list)
    message: str = ''
    table: list = field(default_factory=list)
    cut: str = ''
    checks: int = 0
    remembered: int = 0
    planned_s: float = 0.0

    @property
    def certain(self) -> bool:
        """The hold is impossible (not just unplanned): a step no height can serve whatever the state."""
        return self.outcome == 'infeasible' and self.exhaustive

    def report(self) -> dict:
        """``data.torso_plan`` of the step that planned."""
        out = {'outcome': self.outcome, 'window': self.window,
               'heights': {k: round(v, 4) for k, v in self.heights.items()}, 'changes': self.changes,
               'source': 'memory' if self.checks == 0 and self.remembered else 'planned',
               'checks': self.checks, 'planned_s': round(self.planned_s, 2)}
        out |= {'rescued': self.rescued} if self.rescued else {}
        out |= {'cut': self.cut} if self.cut else {}
        out |= {'message': self.message, 'table': self.table, 'certain': self.certain} if not self.ok else {}
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
    cross-cycle memory (kept by the owner; the newest ``kept`` checks, ``variants`` states each); ``match``: states
    whose joints all differ by at most this (rad, m) are the same, in the memory and the closed set; ``branches``:
    distinct states kept per (step, height); ``max_changes``: torso changes a
    plan may have (None: any; each more multiplies the search of an infeasible window); ``max_checks``: give up after
    this many calls (None: never); ``rescue``: the one-time rescue of a blocking step (module doc)."""

    def __init__(self, grid, tolerance: float = 1e-3, memory: dict | None = None, kept: int = 512, variants: int = 8,
                 match: float = 0.01, branches: int = 3, max_changes: int | None = 2, max_checks: int | None = 150,
                 rescue: bool = True):
        self.grid = sorted({round(float(h), 6) for h in grid})
        self.spacing = min((b - a for a, b in zip(self.grid, self.grid[1:], strict=False)), default=0.0)
        self.tolerance, self.kept, self.variants, self.match = tolerance, kept, variants, match
        self.max_checks = max_checks
        self.branches, self.max_changes, self.rescue = max(1, int(branches)), max_changes, rescue
        self.memory = {} if memory is None else memory

    def heights(self, start: float) -> list[float]:
        """The heights the plan chooses from: the grid, a grid height within half a grid step of ``start`` replaced
        by it."""
        return sorted({h for h in self.grid if abs(h - start) >= self.spacing / 2} | {round(float(start), 6)})

    def plan(self, steps: list[WindowStep], start: float, state: dict, checks: Checks, name: str = '',
             should_stop: Callable[[], bool] = lambda: False) -> Plan:
        """The heights for ``steps`` from the torso at ``start`` and the joints ``state`` (module doc)."""
        return _Search(self, steps, round(float(start), 6), checks, name, should_stop).run(state)

    def recall(self, key: str, state: dict) -> Check | None:
        """The remembered check of ``key`` made from a state within ``match`` of ``state``, or None."""
        return next((check for seen, check in self.memory.get(key, ()) if near(seen, state, self.match)), None)

    def remember(self, key: str, state: dict, check: Check) -> None:
        entries = [*self.memory.pop(key, []), (dict(state), check)][-self.variants:]
        self.memory[key] = entries                 # newest last: the oldest go beyond ``kept``
        for old in list(self.memory)[:-self.kept] if len(self.memory) > self.kept else ():
            self.memory.pop(old, None)


def near(a: dict, b: dict, match: float) -> bool:
    """The same joints, every number within ``match``, everything else equal."""
    if a.keys() != b.keys():
        return False
    for name, value in a.items():
        other = b[name]
        numbers = isinstance(value, int | float) and isinstance(other, int | float)
        if (abs(float(value) - float(other)) > match) if numbers else value != other:
            return False
    return True


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
        self.rescued, self.rescues = set(), []             # steps rescued; [{at, height}] passed only so
        self.saved = set()                                 # (step, height) the rescue passed: rescued again there
        self.hopeless = None                               # a step every height of which is ruled out
        self.pruned = []                                   # what branches / max_changes left out (not exhaustive)
        self.tried = {}                                    # (node, height, kind) -> Check made from that node
        self.watch = set()                                 # first nodes of a step whose edges are not settled
        self.pending = []                                  # watched nodes whose best edge failed: sweep due
        self.began = time.monotonic()

    # ---- the search

    def run(self, state: dict) -> Plan:
        if not self.steps:
            return self._done(Plan(True))
        heap, order, closed, first = [], itertools.count(), {}, {}
        n = len(self.steps)
        root = _Node(0, self.start, state, (0, (), 0.0, 0.0, ()))
        heapq.heappush(heap, (root.key, next(order), root, None))
        while True:
            if self.hopeless is not None:                  # a step no height can serve whatever the state
                return self._done(self._failure(first, self.hopeless))
            if not heap and not self.pending and not self._rescue(first[max(first)], heap, order):
                if self.should_stop():                     # cut short in the last rescue: nothing is proven
                    return self._done(Plan(False, 'stopped', message='torso plan stopped'))
                if self._spent():
                    return self._done(self._failure(first, gave_up=True))
                return self._done(self._failure(first))
            if self.should_stop():
                return self._done(Plan(False, 'stopped', message='torso plan stopped'))
            due = [p for p in self.pending if self._due(p, heap)]
            if due:                                        # before more changes or an earlier one: sweep, rescue
                self.pending = [p for p in self.pending if p not in due]
                for pending in due:
                    self._sweep(pending, closed, heap, order)
                continue
            key, _, node, height = heapq.heappop(heap)
            if height is None:                             # a checked node: close it, open its next step
                states = closed.setdefault((node.k, node.h), [])
                mark = self._relevant(self._step_of(node), node.state)
                if any(near(mark, seen, self.o.match) for seen in states):
                    continue
                if len(states) >= self.o.branches:
                    self.pruned.append('branches')
                    continue
                states.append(mark)
                if node.k == n:
                    return self._done(self._success(node))
                pushed = 0
                for h in self._allowed(node, record=True):
                    if self._full(closed, node.k + 1, h):
                        self.pruned.append('branches')
                    elif (node.k, h) not in self.dead:
                        heapq.heappush(heap, (self._edge_key(node, h), next(order), node, h))
                        pushed += 1
                if first.setdefault(node.k, node) is node:     # the first plan that reaches the step
                    if pushed:
                        self.watch.add(node)
                    else:                                  # nothing to try from it: rescue at once
                        self._rescue(node, heap, order)
                continue
            if self._full(closed, node.k + 1, height):     # an unchecked edge whose end is settled already
                self.pruned.append('branches')
                self.watch.discard(node)
                continue
            if self._spent():
                return self._done(self._failure(first, gave_up=True))
            check = self._try(node, height)
            if check is not None:
                self._push(heap, order, node, height, key, check)
                self.watch.discard(node)
            elif node in self.watch:
                self.watch.discard(node)
                self.pending.append(node)

    @staticmethod
    def _due(pending: _Node, heap: list) -> bool:
        """The next item would add a change or change at an earlier step than ``pending``'s."""
        if not heap:
            return True
        key, _, node, height = heap[0]
        return key[0] > pending.key[0] or (height is not None and height != node.h and node.k < pending.k)

    def _sweep(self, node: _Node, closed: dict, heap, order) -> None:
        """The best edge from the first node of a step failed and the search would move on to more or earlier
        changes: the step's other heights from it now (local, best first, until one passes), and if none passes, the
        step's rescue from it (as the per-step search falls back to the global search at the same step)."""
        self.watch.discard(node)
        for h in sorted(self._allowed(node), key=lambda h: self._edge_key(node, h)):
            if self._spent() or self.should_stop():
                return
            if self._full(closed, node.k + 1, h):
                self.pruned.append('branches')
                continue
            if (node.k, h) in self.dead:
                continue
            check = self._try(node, h)
            if check is not None:
                self._push(heap, order, node, h, self._edge_key(node, h), check)
                return
        self._rescue(node, heap, order)

    def _step_of(self, node: _Node) -> WindowStep:
        return self.steps[node.k - 1] if node.k else self.steps[0]

    def _push(self, heap, order, node, height, key, check):
        done = _Node(node.k + 1, height, check.state, (*key[:3], round(key[3] + check.cost, 9), key[4]), node)
        heapq.heappush(heap, (done.key, next(order), done, None))

    def _rescue(self, node: _Node, heap: list, order) -> bool:
        """Step ``node.k`` failed locally at every height from ``node`` (the first plan that reached it): if certain
        and not rescued yet, every height its rule allows (within max_changes) is tried again with rescue=True.
        True if anything passed."""
        d = node.k
        if not self.o.rescue or d >= len(self.steps) or d in self.rescued or not self.steps[d].certain:
            return False
        passed = False
        for h in self._allowed(node):
            if (d, h) in self.dead:
                continue
            if self._spent() or self.should_stop():
                return passed                              # cut short: not rescued (the caller sees why)
            check = self._try(node, h, rescue=True)
            if check is not None:
                passed = True
                self._push(heap, order, node, h, self._edge_key(node, h), check)
        self.rescued.add(d)
        return passed

    def _spent(self) -> bool:
        return self.o.max_checks is not None and self.calls >= self.o.max_checks

    def _full(self, closed: dict, k: int, h: float) -> bool:
        return len(closed.get((k, h), ())) >= self.o.branches

    def _allowed(self, node: _Node, record: bool = False) -> list[float]:
        """The heights step ``node.k`` may take after ``node``: its rule, within max_changes (``record``: note what
        that left out)."""
        step, h = self.steps[node.k], node.h
        if step.torso == KEEP or step.drive:
            return [h]
        if isinstance(step.torso, int | float):
            heights = [h] if abs(float(step.torso) - h) <= self.o.tolerance else [round(float(step.torso), 6)]
        else:
            heights = sorted(set(self.all) | {h})
        room = self.o.max_changes is None or node.key[0] < self.o.max_changes
        if not room and record and any(x != h for x in heights):
            self.pruned.append('max_changes')
        return [x for x in heights if x == h or room]

    @staticmethod
    def _edge_key(node: _Node, h: float) -> tuple:
        """(changes, change steps latest first (negated), travel, cost, heights): smaller is better (module doc)."""
        changes, late, travel, cost, path = node.key
        if h != node.h:
            changes, late, travel = changes + 1, (-node.k, *late), round(travel + abs(h - node.h), 6)
        return changes, late, travel, cost, (*path, h)

    def _once(self, node: _Node, h: float, kind: str, call) -> Check:
        """A check from ``node`` made at most once per search (the rescue does not repeat the local ones)."""
        if (node, h, kind) not in self.tried:
            self.tried[(node, h, kind)] = call()
        return self.tried[(node, h, kind)]

    def _try(self, node: _Node, h: float, rescue: bool = False) -> Check | None:
        """The edge: ride to ``h`` if it changes, then step ``node.k`` at ``h`` (``rescue``: each local failure tried
        once more with rescue=True). The passing check or None."""
        k, step = node.k, self.steps[node.k]
        if self._bound(k, h):
            return None
        state, saved = node.state, False
        if h != node.h:
            ride = self._once(node, h, 'ride', lambda: self._call(
                'ride', step, node.h, h, node.state, lambda: self.checks.ride(step, node.state, h)))
            if not ride.ok and rescue:
                ride = self._once(node, h, 'ride+', lambda: self._call(
                    'ride+', step, node.h, h, node.state, lambda: self.checks.ride(step, node.state, h, rescue=True)))
                saved = ride.ok
            if not ride.ok:
                self.out[k].setdefault(h, f'ride {node.h:g}->{h:g}: {ride.reason}')
                return None
            state = ride.state
        kind = 'step' if not saved else 'step after ride+'
        check = self._once(node, h, kind, lambda: self._call(
            'step', step, h, h, state, lambda: self.checks.step(step, h, state)))
        if not check.ok and (rescue or (k, h) in self.saved) and not check.lasting and self.o.rescue:
            check = self._once(node, h, kind + '+', lambda: self._call(
                'step+', step, h, h, state, lambda: self.checks.step(step, h, state, rescue=True)))
            saved = saved or check.ok
        if not check.ok:
            self.out[k].setdefault(h, check.reason)
            if check.lasting:
                self._rule_out(k, h, check.reason)
            return None
        self.works[k].add(h)
        if saved:
            self.saved.add((k, h))
            self.rescues.append({'at': step.id, 'height': round(h, 4)})
        return check

    def _bound(self, k: int, h: float) -> str | None:
        if (k, h) not in self.bounds:
            bound = getattr(self.checks, 'bound', None)
            reason = bound(self.steps[k], h) if bound is not None else None
            self.bounds[(k, h)] = reason
            if reason:
                self.out[k].setdefault(h, reason)
                self._rule_out(k, h, reason)
        return self.bounds[(k, h)]

    def _call(self, kind: str, step: WindowStep, h_from: float, h: float, state: dict, call) -> Check:
        """A check, from the memory when the step has a key and a remembered relevant state is within ``match`` of
        this one: the remembered relevant joints are laid over this state (the wheels stay as they are now)."""
        key = None if step.key is None else json.dumps([kind, step.key, round(h_from, 4), round(h, 4)])
        relevant = self._relevant(step, state)
        hit = self.o.recall(key, relevant) if key is not None else None
        if hit is not None:
            self.remembered += 1
            after = None if hit.state is None else {**state, **self._relevant(step, hit.state)}
            return Check(hit.ok, after, hit.cost, hit.reason, hit.lasting)
        self.calls += 1
        check = call()
        if key is not None:
            self.o.remember(key, relevant, check)
        return check

    def _relevant(self, step: WindowStep, state: dict) -> dict:
        relevant = getattr(self.checks, 'relevant', None)
        return dict(state or {}) if relevant is None else relevant(step, state or {})

    # ---- the answers

    def _path(self, node: _Node) -> list[_Node]:
        nodes = []
        while node.parent is not None:
            nodes.append(node)
            node = node.parent
        return nodes[::-1]

    def _success(self, node: _Node, cut: str = '') -> Plan:
        plan = Plan(True, 'cut' if cut else 'planned', cut=cut)
        for done in self._path(node):
            step = self.steps[done.k - 1]
            plan.heights[step.id], plan.states[step.id] = done.h, done.state
            if done.h != done.parent.h:
                plan.changes.append({'at': step.id, 'from': round(done.parent.h, 4), 'to': round(done.h, 4)})
        planned = {(r['at'], r['height']) for r in self.rescues}
        plan.rescued = [{'at': i, 'height': round(h, 4)} for i, h in plan.heights.items()
                        if (i, round(h, 4)) in planned]
        return plan

    def _done(self, plan: Plan) -> Plan:
        plan.checks, plan.remembered, plan.planned_s = self.calls, self.remembered, time.monotonic() - self.began
        plan.window = [s.id for s in self.steps]
        return plan

    def _head(self) -> str:
        return (f'torso plan over {self.name or "the hold"} ({self.steps[0].id}..{self.steps[-1].id}, '
                f'{len(self.steps)} moves): ')

    def _rule_out(self, k: int, h: float, reason: str) -> None:
        self.dead[(k, h)] = reason
        step = self.steps[k]
        pinned = isinstance(step.torso, int | float) and (k, round(float(step.torso), 6)) in self.dead
        # an earlier pin off the grid may be kept to this step: its height counts as one of the step's
        held = {round(float(s.torso), 6) for s in self.steps[:k] if isinstance(s.torso, int | float)}
        if self.hopeless is None and (pinned or all((k, x) in self.dead for x in set(self.all) | held)):
            self.hopeless = k

    def _failure(self, first: dict, blocking: int | None = None, gave_up: bool = False) -> Plan:
        """The furthest step the search reached blocks (or ``blocking``, a step no height serves): every height from
        the best plan up to it, constraints left out, says what it would take (within max_checks); the message names
        the constraint or the reasons."""
        d = max(first) if blocking is None else blocking     # steps 0..d-1 passed somehow, step d nowhere
        node, step = first.get(d) or first[max(first)], self.steps[d]
        if not step.certain and not gave_up:
            plan = self._success(node, cut=f'{step.id} {step.label}'.strip() + ' (uncertain) works at no height from '
                                 'the plan: planned up to the step before it')
            return plan
        for h in self.all:                               # what would work there (rides from the best plan's height)
            if self._spent() or self.should_stop():
                break
            if h not in self.works[d] and (d, h) not in self.dead:
                self._try(node, h)
        works = sorted(self.works[d])
        what = f'{step.id} {step.label}'.strip()
        if works and isinstance(step.torso, int | float):
            text = f'{step.id} {step.rule()} pins {float(step.torso):g}, but {what} works at {self._bands(works)} only'
        elif works and (step.torso == KEEP or step.drive):
            j = d
            while j >= 0 and (self.steps[j].torso == KEEP or self.steps[j].drive):
                j -= 1
            run = self.steps[j + 1:d + 1]
            source = f'set by {self.steps[j].id} {self.steps[j].rule()}' if j >= 0 and \
                self.steps[j].torso != AUTO else f'from {self.steps[j].id}' if j >= 0 else 'the height at the start'
            rules = sorted({'a base move' if s.drive else s.rule() for s in run})
            text = (f'{", ".join(s.id for s in run)} {" / ".join(rules)} hold{"s" if len(run) == 1 else ""} '
                    f'{node.h:g} ({source}) to {step.id}, which works at {self._bands(works)} only')
        elif works:
            text = f'{what} works at {self._bands(works)} only'
        else:
            text = f'{what} works at no height ({self._reasons(d)})'
        why = f'gave up after {self.calls} checks (max_checks), got to ' if gave_up else 'no heights work: '
        if 'max_changes' in self.pruned and not gave_up:
            text += f' (plans with more than {self.o.max_changes} torso changes were not searched: max_changes)'
        # proof only from the reach bound (geometric, whatever the state): local failures are not proof (the
        # rescue ran from one plan per step), neither is a search branches / max_changes cut
        exhaustive = blocking is not None
        return Plan(False, 'capped' if gave_up else 'infeasible', exhaustive and not gave_up,
                    message=self._head() + why + text, table=self._table())

    def _table(self) -> list[dict]:
        rescued = {(r['at'], r['height']) for r in self.rescues}
        rows = []
        for k, s in enumerate(self.steps):
            row = {'id': s.id, 'label': s.label, 'torso': s.torso,
                   'works': [round(h, 4) for h in sorted(self.works[k])],
                   'out': [{'height': round(h, 4), 'reason': r} for h, r in sorted(self.out[k].items())
                           if h not in self.works[k]]}
            row |= {'rescued': sorted(h for i, h in rescued if i == s.id)} if any(i == s.id for i, _ in rescued) else {}
            rows.append(row)
        return rows

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
    after it: the end of its lines (mode linear), else the IK solution of its main target. Local IK only, except with
    ``rescue`` (the global search, as solve_with_torso falls back to). ``ride_sample_m``: the ride's sample spacing
    (default: ik.torso.line_sample_m, at most half the grid's usual step, so a collision between grid heights is
    seen)."""

    def __init__(self, skill, should_stop: Callable[[], bool] = lambda: False, ride_sample_m: float | None = None):
        from .ik import LINE_SAMPLE_M

        self.skill, self.ik, self.should_stop = skill, skill.ik, should_stop
        self.joint = self.ik.torso['joint']
        grid = sorted({round(float(v), 6) for v in self.ik.torso.get('candidates', ())})
        gaps = [round(b - a, 6) for a, b in zip(grid, grid[1:], strict=False)]
        half = max(gaps, key=lambda g: (gaps.count(g), -g)) / 2 if gaps else math.inf    # the grid's usual step
        self.ride_sample = ride_sample_m or min(float(self.ik.torso.get('line_sample_m', LINE_SAMPLE_M)), half)

    def _hands(self, step: WindowStep) -> list[str]:
        return list(step.hands) or list(self.ik.groups)

    def relevant(self, step: WindowStep, state: dict) -> dict:
        """The torso and every arm's joints: what the later checks start from and collide with (the idle arm too:
        IK and the rides check it), not the wheels or the base."""
        import re

        patterns = [re.compile(self.ik.joint_pattern.format(tool=t)) for t in self.ik.groups]
        return {n: v for n, v in state.items() if n == self.joint or any(p.fullmatch(n) for p in patterns)}

    def bound(self, step: WindowStep, height: float) -> str | None:
        targets = self._targets(step)
        return self.ik._unreachable(targets, height, allow_flip=False) if targets else None

    @staticmethod
    def _targets(step: WindowStep) -> list:
        """The IK targets: the step's, else the end of its last line ({tool: 4x4} -> tool targets)."""
        from .points import target_of

        if step.targets:
            return step.targets
        return [{tool: target_of(goal) for tool, goal in step.lines[-1].items()}] if step.lines else []

    def step(self, step: WindowStep, height: float, state: dict, rescue: bool = False) -> Check:
        from .ik import OUT_OF_REACH, Samples, TorsoError

        state = {**state, self.joint: height}     # the torso is there: _walk_lines reads no live pose, no re-solve
        targets = self._targets(step)
        if not targets:                          # nothing to reach (a base move): the arms stay
            return Check(True, state)
        samples, end = Samples(targets), {}
        if step.lines:
            def verify(h):
                problem, end['state'] = self.skill._walk_lines(step.lines, state, h)
                return problem
            samples.verify = verify
        if step.ahead:
            samples.ahead = self.skill._ahead_check(step.ahead, state)
        try:
            choice = self.ik.check_height(samples, height, state, fixed=step.fixed, elbow=step.elbow,
                                          global_search=rescue, should_stop=self.should_stop)
        except TorsoError as error:      # local no-IK depends on the seeds (the state): only the reach bound lasts
            return Check(False, reason=str(error), lasting=OUT_OF_REACH in str(error))
        after = end.get('state') or {**state, **choice.solution}
        return Check(True, after, choice.cost)

    def ride(self, step: WindowStep, state: dict, height: float, rescue: bool = False) -> Check:
        """The column moves in samples ``ride_sample`` apart; at each the held hands follow (``_state_at``, local IK
        unless ``rescue``, from the hands where the last sample left them) and the state is collision checked."""
        import re

        hands = self._hands(step)
        begin = float(state[self.joint])
        parts = max(1, math.ceil(abs(height - begin) / self.ride_sample - 1e-9))
        patterns = [re.compile(self.ik.joint_pattern.format(tool=t)) for t in hands]
        for i in range(1, parts + 1):
            h = begin + (height - begin) * i / parts
            poses = self.ik.tcps_at(hands, state)
            if len(poses) < len(hands):
                return Check(False, reason=f'no URDF chain for {sorted(set(hands) - set(poses))}')
            moved, problem = self.skill._state_at(hands, state, h, poses=poses, global_search=rescue)
            if moved is None:
                return Check(False, reason=problem)
            arms = {n: v for n, v in moved.items() if any(p.fullmatch(n) for p in patterns)}
            if not self.ik._state_valid({self.joint: h, **self.ik.idle_joints(hands, moved), **arms}, hands[0]):
                return Check(False, reason=f'collision at {h:.3f}')
            state = moved
        return Check(True, {**state, self.joint: height})
