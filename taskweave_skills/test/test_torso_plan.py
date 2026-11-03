"""Torso look-ahead over a hold window (torso_plan.TorsoPlan) with fake checks (no ROS). The bands are the measured
ones of the frame cell (design section 2), on the profile grid 0.0..0.7 by 0.1."""

from types import SimpleNamespace

import numpy as np
import pytest

from taskweave_skills.torso_plan import KEEP, Check, SkillChecks, TorsoPlan, WindowStep

GRID = [round(0.1 * i, 1) for i in range(8)]
LIVE = 0.4925                     # the torso when the hold begins (today's keep height)


def band(*ranges):
    """A predicate of the height: inside one of the (low, high) ranges."""
    return lambda h: any(low - 1e-6 <= h <= high + 1e-6 for low, high in ranges)


class Fake:
    """``works``: step id -> predicate of the height (or of (height, state)); ``rides``: (step id, from, to) -> reason
    or None; ``rescue``: step id -> predicate that passes with rescue=True. Counts the calls; the state carries the
    trail of (step, height) so chaining is visible."""

    def __init__(self, works, rides=None, bound=None, lasting=True, rescue=None):
        self.works, self.rides, self.out, self.lasting = works, rides or (lambda *a: None), bound, lasting
        self.rescue = rescue or {}
        self.steps, self.ridden, self.seen, self.rescues = [], [], [], []

    def bound(self, step, height):
        return self.out(step.id, height) if self.out else None

    def step(self, step, height, state, rescue=False):
        if rescue:
            self.rescues.append((step.id, height))
            ok = self.rescue.get(step.id, band())
        else:
            self.steps.append((step.id, height))
            self.seen.append((step.id, state['trail']))
            ok = self.works[step.id]
        if not (ok(height, state) if ok.__code__.co_argcount == 2 else ok(height)):
            return Check(False, reason='no IK solution', lasting=self.lasting)
        return Check(True, {**state, 'lift': height, 'trail': (*state['trail'], (step.id, height))}, cost=0.1)

    def ride(self, step, state, height, rescue=False):
        self.ridden.append((step.id, state['lift'], height))
        reason = self.rides(step.id, round(state['lift'], 4), round(height, 4))
        if reason:
            return Check(False, reason=reason)
        return Check(True, {**state, 'lift': height})


def start(h=LIVE):
    return {'lift': h, 'trail': ()}


TOP = {     # top of a full stack: section 2, lines included (grasp->lift 0.40-0.50, lift->tilt 0.0 and 0.40-0.70)
    's1_s7': band((0.4, 0.5)), 's1_s9': band((0.0, 0.0), (0.4, 0.7)), 's1_s11': band((0.0, 0.0), (0.2, 0.7)),
    's2_s2': band((0.0, 0.6)), 's2_s3': band((0.0, 0.7)), 's2_s5': band((0.0, 0.45)), 's2_s6': band((0.0, 0.35))}
LABELS = {'s1_s7': 'lift', 's1_s9': 'tilt', 's1_s11': 'carry', 's2_s2': 'stand_up 1', 's2_s3': 'hang-approach',
          's2_s5': 'hook 10 cm above', 's2_s6': 'onto the hook'}


def window(rules=None, works=TOP):
    return [WindowStep(i, LABELS.get(i, ''), torso=(rules or {}).get(i, 'auto')) for i in works]


def test_keep_wins_when_it_works_with_one_check_per_step_and_no_ride():
    low = {i: band((0.15, 0.25)) for i in TOP}          # the lowest frame: 0.15-0.25 serves the whole hold
    fake = Fake(low)
    plan = TorsoPlan(GRID).plan(window(works=low), 0.2, start(0.2), fake)
    assert plan.ok and plan.changes == [] and set(plan.heights.values()) == {0.2}
    assert fake.steps == [(i, 0.2) for i in TOP] and fake.ridden == [] and plan.checks == len(TOP)


def test_top_stack_gives_exactly_one_change_at_the_hook():
    fake = Fake(TOP)
    plan = TorsoPlan(GRID).plan(window(), LIVE, start(), fake, name='the hold of frame_1')
    assert plan.ok, plan.message
    assert plan.changes == [{'at': 's2_s5', 'from': LIVE, 'to': 0.3}]
    assert [plan.heights[i] for i in TOP] == [LIVE] * 5 + [0.3, 0.3]
    assert plan.states['s2_s6']['trail'] == tuple((i, plan.heights[i]) for i in TOP)     # chained step by step
    assert ('s2_s5', LIVE, 0.4) in fake.ridden and ('s2_s5', LIVE, 0.3) in fake.ridden  # nearest first, at the hook
    assert {step for step, _, _ in fake.ridden} <= {'s2_s5', 's2_s6'}                  # no earlier change tried
    assert plan.checks <= 18 and plan.report()['changes'] == plan.changes and plan.outcome == 'planned'


def test_a_change_moves_one_step_earlier_only_when_no_ride_at_the_late_step_works():
    fake = Fake(TOP, rides=lambda step, a, b: 'collides with the hanger' if step == 's2_s5' else None)
    plan = TorsoPlan(GRID).plan(window(), LIVE, start(), fake)
    assert plan.changes == [{'at': 's2_s3', 'from': LIVE, 'to': 0.3}]


SET_DOWN = {'s1_s11': band((0.0, 0.75)), 'set_down': band((0.2, 0.3))}   # 0.35 out of reach, 0.40 joint 4 limit


def set_down_rides(step, a, b):
    """Measured: from a carry at 0.6 / 0.7 the ride collides down to 0.20 and 0.25; from 0.5 only to 0.20."""
    low = 0.25 if a >= 0.6 else 0.2
    return f'ride collides at {b:g}' if step == 'set_down' and b <= low + 1e-6 else None


def test_the_set_down_pair_constraint_is_honoured():
    grid = [round(0.05 * i, 2) for i in range(16)]
    fake = Fake(SET_DOWN, rides=set_down_rides)
    plan = TorsoPlan(grid).plan(window(works=SET_DOWN), 0.7, start(0.7), fake)
    assert plan.heights == {'s1_s11': 0.7, 'set_down': 0.3}
    pinned = window({'set_down': 0.25}, works=SET_DOWN)               # 0.25 from the carry at 0.7: the ride collides,
    plan = TorsoPlan(grid).plan(pinned, 0.7, start(0.7), Fake(SET_DOWN, rides=set_down_rides))
    assert plan.heights == {'s1_s11': 0.25, 'set_down': 0.25}        # so the change goes one step earlier
    plan = TorsoPlan(grid).plan(pinned, 0.5, start(0.5), Fake(SET_DOWN, rides=set_down_rides))
    assert plan.heights == {'s1_s11': 0.5, 'set_down': 0.25}         # from 0.5 the ride works: change at set_down


def test_keep_and_pinned_heights_are_respected():
    plan = TorsoPlan(GRID).plan(window({'s2_s5': KEEP}), LIVE, start(), Fake(TOP))
    assert plan.changes == [{'at': 's2_s3', 'from': LIVE, 'to': 0.3}]   # no change at the keep step: one before it
    plan = TorsoPlan(GRID).plan(window({'s2_s2': 0.2}), LIVE, start(), Fake(TOP))
    assert plan.heights['s2_s2'] == 0.2 and plan.changes[0] == {'at': 's2_s2', 'from': LIVE, 'to': 0.2}
    assert len(plan.changes) == 1                                        # 0.2 serves the rest of the hold


def test_a_keep_that_blocks_is_named_in_the_failure():
    keeps = {i: KEEP for i in TOP if i != 's1_s7'}
    plan = TorsoPlan(GRID).plan(window(keeps), LIVE, start(), Fake(TOP), name='the hold of frame_1')
    assert not plan.ok
    assert plan.message == ('torso plan over the hold of frame_1 (s1_s7..s2_s6, 7 moves): no heights work: '
                            's1_s9, s1_s11, s2_s2, s2_s3, s2_s5, s2_s6 torso: keep hold 0.4 (from s1_s7) to s2_s6, '
                            'which works at 0-0.3 only')
    rows = {r['id']: r for r in plan.table}
    assert rows['s1_s7']['works'] == [0.4, LIVE] and rows['s2_s6']['works'] == [0.0, 0.1, 0.2, 0.3]
    assert plan.report()['table'] == plan.table


def test_a_pin_that_blocks_is_named_in_the_failure():
    steps = window({'s2_s6': 0.4})
    steps[-1].why = 'held pose hook sets the torso'
    plan = TorsoPlan(GRID).plan(steps, LIVE, start(), Fake(TOP))
    assert not plan.ok and plan.message.endswith(
        'no heights work: s2_s6 held pose hook sets the torso pins 0.4, but s2_s6 onto the hook works at 0-0.3 only')


def test_a_step_no_height_serves_is_named_with_its_reasons():
    works = {**TOP, 's2_s6': band()}
    plan = TorsoPlan(GRID).plan(window(works=works), LIVE, start(), Fake(works))
    assert not plan.ok and plan.message.endswith('s2_s6 onto the hook works at no height (0-0.7: no IK solution)')


def test_an_uncertain_step_cuts_the_plan_instead_of_failing():
    works = {**TOP, 's2_s6': band()}
    steps = window(works=works)
    steps[-1].certain = False
    plan = TorsoPlan(GRID).plan(steps, LIVE, start(), Fake(works))
    assert plan.ok and 's2_s6' not in plan.heights and 'uncertain' in plan.cut and plan.heights['s2_s5'] <= 0.45


def test_the_same_input_gives_the_same_plan_and_memory_answers_the_next_cycle():
    first = TorsoPlan(GRID).plan(window(), LIVE, start(), Fake(TOP))
    second = TorsoPlan(GRID).plan(window(), LIVE, start(), Fake(TOP))
    assert (first.heights, first.changes, first.checks) == (second.heights, second.changes, second.checks)
    planner, steps = TorsoPlan(GRID), window()
    for step in steps:
        step.key = f'{step.id}@pose'
    cold = planner.plan(steps, LIVE, start(), Fake(TOP))
    fake = Fake(TOP)
    warm = planner.plan(steps, LIVE, start(), fake)
    assert warm.heights == cold.heights and warm.checks == 0 and fake.steps == [] and warm.remembered == cold.checks
    assert warm.report()['source'] == 'memory' and cold.report()['source'] == 'planned'


def test_the_reach_bound_and_lasting_failures_save_calls():
    fake = Fake(TOP, bound=lambda step, h: 'out of reach' if step == 's2_s6' and h >= 0.45 else None)
    plan = TorsoPlan(GRID).plan(window(), LIVE, start(), fake)
    assert plan.ok and not any(step == 's2_s6' and h >= 0.45 for step, h in fake.steps)
    failed = [(s, h) for s, h in fake.steps if not TOP[s](h)]
    assert len(failed) == len(set(failed))       # a lasting failure is not checked again from another state


def test_the_state_a_step_starts_from_is_what_the_steps_before_left():
    def tilt(h, state):                  # the tilt works only from the lift's arm branch at 0.4
        return ('s1_s7', 0.4) in state['trail']
    works = {**TOP, 's1_s9': tilt}
    fake = Fake(works, lasting=False)
    plan = TorsoPlan(GRID).plan(window(works=works), LIVE, start(), fake)
    assert plan.ok and plan.heights['s1_s7'] == 0.4 and plan.changes[0] == {'at': 's1_s7', 'from': LIVE, 'to': 0.4}


def test_max_checks_gives_up_with_a_message():
    plan = TorsoPlan(GRID, max_checks=3).plan(window(), LIVE, start(), Fake(TOP))
    assert not plan.ok and 'gave up after 3 checks' in plan.message


def test_a_later_check_that_fails_from_one_arm_branch_is_tried_from_another():
    """Review I1: s2_s6 works at 0.3 only from the branch s2_s3 leaves at 0.3 (a line, not lasting); the change at
    s2_s5 reaches (s2_s6, 0.3) first with another branch."""
    works = {**TOP, 's2_s6': lambda h, st: abs(h - 0.3) < 1e-6 and ('s2_s3', 0.3) in st['trail']}
    plan = TorsoPlan(GRID).plan(window(works=works), LIVE, start(), Fake(works, lasting=False))
    assert plan.ok, plan.message
    assert plan.changes == [{'at': 's2_s3', 'from': LIVE, 'to': 0.3}]
    again = TorsoPlan(GRID).plan(window(works=works), LIVE, start(), Fake(works, lasting=False))
    assert (again.heights, again.checks) == (plan.heights, plan.checks)
    capped = TorsoPlan(GRID, max_checks=plan.checks - 1).plan(window(works=works), LIVE, start(),
                                                             Fake(works, lasting=False))
    assert not capped.ok and 'gave up' in capped.message


def test_a_blocking_step_is_rescued_once_before_the_plan_fails():
    """Review I2: on the profile grid the set-down from a carry at 0.7 has only 0.3 (0.2 rides into the column,
    0.4 is beyond the joint limit); local IK finds nothing there, the rescue (global search) does."""
    grid = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.75]
    works = {'s1_s11': band((0.6, 0.75)), 'set_down': band((0.2, 0.25))}     # the carry stays high
    fake = Fake(works, rides=set_down_rides, lasting=False, rescue={'set_down': band((0.3, 0.3))})
    plan = TorsoPlan(grid).plan(window(works=works), 0.7, start(0.7), fake)
    assert plan.ok, plan.message
    assert plan.heights == {'s1_s11': 0.7, 'set_down': 0.3} and plan.rescued == [{'at': 'set_down', 'height': 0.3}]
    assert plan.report()['rescued'] == plan.rescued
    assert len(fake.rescues) <= len(grid)                              # once per height at the blocking step
    fake = Fake(works, rides=set_down_rides, lasting=False)
    plan = TorsoPlan(grid).plan(window(works=works), 0.7, start(0.7), fake)
    assert not plan.ok and len(fake.rescues) <= len(grid)              # nothing rescues: it fails, rescued once
    plan = TorsoPlan(grid, rescue=False).plan(window(works=works), 0.7, start(0.7), Fake(
        works, rides=set_down_rides, lasting=False, rescue={'set_down': band((0.3, 0.3))}))
    assert not plan.ok


class Held(Fake):
    """Later checks depend on the arms (the idle one too: IK and rides check it) and the torso, not the wheels (as
    SkillChecks.relevant)."""

    def relevant(self, step, state):
        return {n: v for n, v in state.items() if n in ('lift', 'left_j1', 'right_j1')}


def test_memory_hits_with_the_wheels_changed_and_noise_but_not_with_the_idle_arm_moved():
    """Review I4, N5, N6: matched within ``match`` (a noisy joint across a rounding boundary still hits), the wheels
    left out, the idle arm in (a remembered pass must not hide a collision with it)."""
    planner, steps = TorsoPlan(GRID), window()
    for step in steps:
        step.key = f'{step.id}@pose'
    first = {'lift': LIVE, 'left_j1': 0.3049, 'right_j1': 1.0, 'wheel_l': 12.0, 'trail': ()}
    cold = planner.plan(steps, LIVE, first, Held(TOP))
    fake = Held(TOP)
    later = {**first, 'left_j1': 0.3051, 'right_j1': 1.002, 'wheel_l': 873.5, 'trail': ('cycle 2',)}
    warm = planner.plan(steps, LIVE, later, fake)
    assert warm.ok and warm.heights == cold.heights and warm.checks == 0 and fake.steps == []
    assert warm.states['s2_s6']['wheel_l'] == 873.5 and warm.states['s2_s6']['trail'] == ('cycle 2',)
    idle = planner.plan(steps, LIVE, {**later, 'right_j1': -0.4}, Held(TOP))      # the idle arm moved: checked again
    assert idle.checks > 0
    held = planner.plan(steps, LIVE, {**later, 'left_j1': 0.9}, Held(TOP))        # the held arm differs: checked again
    assert held.checks > 0


def test_states_within_match_are_one_branch():
    from taskweave_skills.torso_plan import near
    assert near({'a': 0.3049, 'b': 'x'}, {'a': 0.3051, 'b': 'x'}, 0.01)
    assert not near({'a': 0.30}, {'a': 0.32}, 0.01) and not near({'a': 0.3}, {'a': 0.3, 'b': 1.0}, 0.01)

    class Noisy(Held):
        """Three plans reach (b, 0.3): two leave the held arm at 0.305 +- noise (across a rounding boundary), the
        third (a at 0.4) another branch (0.9), the only one c works from."""

        def step(self, step, height, state, rescue=False):
            check = super().step(step, height, state, rescue)
            if check.ok and step.id == 'b':
                trail = check.state['trail']
                check.state['left_j1'] = 0.9 if ('a', 0.4) in trail else 0.3049 if ('a', 0.5) in trail else 0.3051
            return check
    works = {'a': band((0.3, 0.5)), 'b': band((0.3, 0.3)),
             'c': lambda h, st: abs(h - 0.3) < 1e-6 and st['left_j1'] == 0.9}
    state = {**start(0.5), 'left_j1': 0.3, 'right_j1': 1.0}
    plan = TorsoPlan(GRID, branches=2).plan(window(works=works), 0.5, state, Noisy(works, lasting=False))
    assert plan.ok and plan.heights == {'a': 0.4, 'b': 0.3, 'c': 0.3}       # the noisy pair took one slot only
    exact = TorsoPlan(GRID, branches=2, match=0.0).plan(window(works=works), 0.5, state, Noisy(works, lasting=False))
    assert not exact.ok                                                      # matched exactly, they take both


# ---- fix round 2: the rescue within the default budget, branches 3, max_changes, outcomes

GRID_S1 = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.75]


def high_rides(step, a, b):
    """The carry is high: any ride from >= 0.6 down to <= 0.25 collides, at every step."""
    return f'ride collides at {b:g}' if a >= 0.6 and b <= 0.25 + 1e-6 else None


@pytest.mark.parametrize('before', [3, 4, 5])
def test_a_long_set_down_window_is_rescued_well_under_the_default_cap(before):
    """Review N1: carry steps that work at 0.6-0.75, then a set-down that works locally at 0.2-0.25 only (rides
    collide there) and at 0.3 only with the rescue: 4, 5 and 6 steps, at the default max_checks."""
    works = {**{f'c{i}': band((0.6, 0.75)) for i in range(before)}, 'set_down': band((0.2, 0.25))}
    fake = Fake(works, rides=high_rides, lasting=False, rescue={'set_down': band((0.3, 0.3))})
    plan = TorsoPlan(GRID_S1).plan(window(works=works), 0.7, start(0.7), fake)
    assert plan.ok and plan.heights['set_down'] == 0.3 and plan.rescued == [{'at': 'set_down', 'height': 0.3}]
    assert plan.changes == [{'at': 'set_down', 'from': 0.7, 'to': 0.3}]
    assert plan.checks <= 23 + before and len(fake.rescues) <= len(GRID_S1)       # n checks + one sweep


def test_the_top_stack_with_the_hook_only_by_the_rescue_is_planned_under_the_default_cap():
    works = {**TOP, 's2_s6': band()}
    fake = Fake(works, lasting=False, rescue={'s2_s6': band((0.3, 0.3))})
    plan = TorsoPlan(GRID).plan(window(works=works), LIVE, start(), fake)
    assert plan.ok and plan.changes == [{'at': 's2_s5', 'from': LIVE, 'to': 0.3}]
    assert {'at': 's2_s6', 'height': 0.3} in plan.rescued and plan.checks <= 40


def test_a_third_arm_branch_is_tried_with_the_default_branches():
    """s2_s6 works at 0.3 only from the branch a change at s2_s2 leaves (after the ones at s2_s5 and s2_s3)."""
    works = {**TOP, 's2_s6': lambda h, st: abs(h - 0.3) < 1e-6 and ('s2_s2', 0.3) in st['trail']}
    plan = TorsoPlan(GRID).plan(window(works=works), LIVE, start(), Fake(works, lasting=False))
    assert plan.ok and plan.changes == [{'at': 's2_s2', 'from': LIVE, 'to': 0.3}] and plan.checks <= 90
    assert not TorsoPlan(GRID, branches=2).plan(window(works=works), LIVE, start(), Fake(works, lasting=False)).ok


def test_max_changes_is_kept_by_the_rescue_and_named():
    """Review N2: a hold that needs 4 changes is refused with max_changes 2, even when a rescue would pass."""
    works = {'a': band((0.0, 0.1)), 'b': band((0.6, 0.7)), 'c': band((0.0, 0.1)), 'd': band((0.6, 0.7))}
    fake = Fake(works, lasting=False, rescue={'d': band((0.0, 0.7))})
    plan = TorsoPlan(GRID).plan(window(works=works), 0.3, start(0.3), fake)
    assert not plan.ok and 'more than 2 torso changes were not searched: max_changes' in plan.message
    assert plan.outcome == 'infeasible' and not plan.certain           # left out by max_changes: not known impossible
    plan = TorsoPlan(GRID, max_changes=None).plan(window(works=works), 0.3, start(0.3), Fake(works, lasting=False))
    assert plan.ok and len(plan.changes) == 4 and plan.rescued == []


def test_the_rescue_does_not_repeat_local_checks():
    """Review N4: from one state, each local check is made once, the rescue included."""
    works = {**{f'c{i}': band((0.6, 0.75)) for i in range(3)}, 'set_down': band((0.2, 0.25))}
    fake = Fake(works, rides=high_rides, lasting=False, rescue={'set_down': band((0.3, 0.3))})
    TorsoPlan(GRID_S1).plan(window(works=works), 0.7, start(0.7), fake)
    local = [(step, h, trail) for (step, h), (_, trail) in zip(fake.steps, fake.seen, strict=True)]
    assert len(local) == len(set(local)) and len(fake.rescues) == len(set(fake.rescues))


def test_the_outcome_says_whether_the_hold_is_impossible_or_only_unplanned():
    """T4's ruling: only a finished search that found nothing (certain) fails the first held step; capped or
    stopped falls back to the per-step choice."""
    assert TorsoPlan(GRID).plan(window(), LIVE, start(), Fake(TOP)).outcome == 'planned'
    nowhere = {**TOP, 's2_s6': band()}
    plan = TorsoPlan(GRID).plan(window(works=nowhere), LIVE, start(), Fake(nowhere))
    assert plan.outcome == 'infeasible' and plan.certain and plan.report()['certain'] is True
    plan = TorsoPlan(GRID).plan(window(works=nowhere), LIVE, start(), Fake(nowhere, lasting=False))
    assert plan.outcome == 'capped' and not plan.certain and 'gave up after 150 checks' in plan.message
    calls = iter(range(1000))
    plan = TorsoPlan(GRID).plan(window(works=nowhere), LIVE, start(), Fake(nowhere, lasting=False),
                                should_stop=lambda: next(calls) > 5)
    assert plan.outcome == 'stopped' and not plan.certain and plan.report()['outcome'] == 'stopped'
    steps = window(works=nowhere)
    steps[-1].certain = False
    assert TorsoPlan(GRID).plan(steps, LIVE, start(), Fake(nowhere)).outcome == 'cut'


def test_a_base_move_keeps_the_torso():
    works = {'s1_s11': band((0.0, 0.7)), 'drive': band((0.0, 0.7)), 'hook': band((0.0, 0.35))}
    steps = window(works=works)
    steps[1].drive = True
    plan = TorsoPlan(GRID).plan(steps, LIVE, start(), Fake(works))
    assert plan.changes == [{'at': 'hook', 'from': LIVE, 'to': 0.3}]


# ---- the real checks from a skill, with the fake IK of test_ik_torso

def _fake_ik():
    pytest.importorskip('test_ik_torso')
    import test_ik_torso
    return test_ik_torso


def test_skill_checks_solve_one_height_walk_the_lines_and_end_where_the_lines_end():
    t = _fake_ik()
    searches = []

    class Ik(t.FakeIk):
        def solve(self, tools, fixed, should_stop=lambda: False, current=None, allow_flip=True, elbow=None,
                  global_search=True, global_only=False):
            searches.append(global_search)
            return super().solve(tools, fixed, should_stop, current, allow_flip, elbow, global_search, global_only)
    ik = Ik(reach=lambda tool, target, h: 0.0 if h <= 0.35 else None)
    walked = []

    def walk(legs, current, height):
        walked.append((len(legs), current['lift'], height))
        return ('leg 1: collision' if height < 0.1 else None), {**current, 'lift': height, 'left_j1': 0.7}
    skill = SimpleNamespace(ik=ik, _walk_lines=walk, _ahead_check=None, _state_at=None)
    checks = SkillChecks(skill)
    step = WindowStep('s2_s6', targets=[t.TOOLS], lines=[{'left': np.eye(4)}], hands=('left', 'right'))
    ok = checks.step(step, 0.3, t.now(0.3))
    assert ok.ok and ok.state['left_j1'] == 0.7 and walked[-1] == (1, 0.3, 0.3) and searches == []
    assert checks.step(step, 0.4, t.now(0.4)).ok and searches == []          # a line walks: no IK of its goal
    solved = WindowStep('s2_s6', targets=[t.TOOLS], hands=('left', 'right'))     # no line: IK of the goal
    out = checks.step(solved, 0.4, t.now(0.4))
    assert not out.ok and not out.lasting and 'no IK solution' in out.reason     # local no-IK: the seeds decide
    assert not checks.step(solved, 0.4, t.now(0.4), rescue=True).ok and searches[-1] is True
    count = len(searches)
    assert checks.step(step, 0.4, t.now(0.4), rescue=True).ok and len(searches) == count   # a line: walked again
    line = checks.step(step, 0.0, t.now(0.0))
    assert not line.ok and not line.lasting and line.reason == 'line: leg 1: collision'
    assert checks.step(WindowStep('drive'), 0.3, t.now(0.3)).ok           # nothing to reach: the arms stay
    assert checks.bound(step, 0.3) is None                               # no chains: no bound
    ik._unreachable = lambda goals, h, allow_flip=True: 'out of reach: left target 1.20 m from its shoulder'
    assert checks.step(step, 0.3, t.now(0.3)).lasting                     # the reach bound lasts


def test_skill_checks_take_the_targets_of_a_step_with_lines_only():
    """Review M1: the end of the last line ({tool: 4x4}) becomes the IK target (the bound, a step without lines)."""
    t = _fake_ik()
    ik = t.FakeIk()
    seen = []
    ik.check_height = lambda samples, h, state, **kw: seen.append(list(samples)) or SimpleNamespace(
        solution={'left_j1': 0.1}, cost=0.0)
    goal = np.eye(4)
    goal[:3, 3] = [0.6, 0.2, 0.3]
    skill = SimpleNamespace(ik=ik, _walk_lines=lambda legs, current, h: (None, None), _ahead_check=None)
    assert SkillChecks._targets(WindowStep('set_down', lines=[{'left': goal}]))[0]['left']['tcp'] == [0.6, 0.2, 0.3]
    assert SkillChecks(skill).step(WindowStep('set_down', lines=[{'left': goal}]), 0.3, t.now(0.3)).ok and not seen
    assert SkillChecks(skill).bound(WindowStep('set_down', lines=[{'left': goal}]), 0.3) is None


def test_skill_checks_ride_in_samples_with_the_held_hands_following_on_local_ik():
    t = _fake_ik()
    ik = t.FakeIk(collides=lambda joints: abs(joints['lift'] - 0.5) < 1e-6)
    ik.torso['line_sample_m'] = 0.3
    ik.tcps_at = lambda tools, state: {tool: np.eye(4) * state['lift'] for tool in tools}
    calls = []

    def state_at(hands, current, height, poses=None, global_search=True):
        calls.append((round(current['lift'], 4), round(height, 4), round(float(poses['left'][0, 0]), 4),
                      global_search))
        return {**current, 'lift': height, 'left_j1': height}, None
    assert SkillChecks(SimpleNamespace(ik=ik, _state_at=state_at)).ride_sample == pytest.approx(0.02)   # lookahead
    ik.torso['lookahead'] = {'ride_sample_m': 0.5}
    assert SkillChecks(SimpleNamespace(ik=ik, _state_at=state_at)).ride_sample == pytest.approx(0.05)   # half a step
    del ik.torso['lookahead']
    checks = SkillChecks(SimpleNamespace(ik=ik, _state_at=state_at), ride_sample_m=0.2)
    ride = checks.ride(WindowStep('set_down', hands=('left',)), t.now(0.7), 0.3)
    assert not ride.ok and ride.reason == 'collision at 0.500' and calls == [(0.7, 0.5, 0.7, False)]
    calls.clear()
    ride = checks.ride(WindowStep('set_down', hands=('left',)), t.now(0.6), 0.2, rescue=True)
    assert ride.ok and ride.state['lift'] == 0.2 and calls == [(0.6, 0.4, 0.6, True), (0.4, 0.2, 0.4, True)]
    ik.tcps_at = lambda tools, state: {}
    assert 'no URDF chain' in checks.ride(WindowStep('set_down', hands=('left',)), t.now(0.6), 0.2).reason


def test_state_at_takes_predicted_poses_and_local_ik():
    pytest.importorskip('sensor_msgs')
    from taskweave_skills.gripper_motion import MoveGripper
    solved = []
    ik = SimpleNamespace(torso={'joint': 'lift'}, joint_motion=lambda joint, state, h: np.eye(4),
                         solve=lambda tools, fixed, **kw: solved.append((tools, kw)) or {'left_j1': 1.0},
                         tcps_at=lambda tools, state: {})          # no chain: no proof the column carries the hand

    def live(hands):
        raise AssertionError('the live poses were read')
    skill = SimpleNamespace(ik=ik, hold=SimpleNamespace(hand_holds={'left': 'box'}, tcp_poses=live),
                            p=SimpleNamespace(position_tolerance=1e-3))
    pose = np.eye(4)
    pose[:3, 3] = [0.5, 0.1, 0.2]
    state, problem = MoveGripper._state_at(skill, ['left'], {'lift': 0.7}, 0.3, poses={'left': pose},
                                           global_search=False)
    assert problem is None and state == {'lift': 0.3, 'left_j1': 1.0}
    assert solved[0][0]['left']['tcp'] == [0.5, 0.1, 0.2] and solved[0][1]['global_search'] is False


def test_state_at_keeps_the_arms_when_the_column_carries_the_held_hands():
    """Fix round 1 I3: a torso ride moves everything above the column; when the chains say the hands' FK at the new
    height is the column's motion of their FK now, the arms keep their joints (no IK); else IK as before."""
    pytest.importorskip('sensor_msgs')
    from taskweave_skills.gripper_motion import MoveGripper

    def lifted(h):
        m = np.eye(4)
        m[2, 3] = h
        return m
    solved = []
    ik = SimpleNamespace(torso={'joint': 'lift'}, joint_motion=lambda joint, state, h: lifted(h - state['lift']),
                         solve=lambda tools, fixed, **kw: solved.append(tools) or {'left_j1': 1.0},
                         tcps_at=lambda tools, state: {t: lifted(state['lift'] + 0.2) for t in tools})
    skill = SimpleNamespace(ik=ik, hold=SimpleNamespace(hand_holds={'left': 'box'}),
                            p=SimpleNamespace(position_tolerance=1e-3))
    state, problem = MoveGripper._state_at(skill, ['left'], {'lift': 0.7, 'left_j1': 0.4}, 0.3, poses={})
    assert problem is None and state == {'lift': 0.3, 'left_j1': 0.4} and solved == []
    ik.tcps_at = lambda tools, state: {t: lifted(0.2) for t in tools}       # the hand does not ride: IK
    skill.hold.tcp_poses = lambda hands: {h: lifted(0.2) for h in hands}
    state, _ = MoveGripper._state_at(skill, ['left'], {'lift': 0.7, 'left_j1': 0.4}, 0.3)
    assert state['left_j1'] == 1.0 and len(solved) == 1


def test_plan_rejects_nothing_on_an_empty_window():
    plan = TorsoPlan(GRID).plan([], LIVE, start(), Fake({}))
    assert plan.ok and plan.heights == {} and plan.checks == 0


@pytest.mark.parametrize('rule', [KEEP, 0.4925])
def test_a_keep_or_a_pin_at_the_current_height_is_no_change(rule):
    plan = TorsoPlan(GRID).plan(window({'s1_s7': rule}), LIVE, start(), Fake(TOP))
    assert plan.heights['s1_s7'] == LIVE and plan.changes == [{'at': 's2_s5', 'from': LIVE, 'to': 0.3}]


# ---- fix round 3: certain only when proven (T4 stops a mission on it)

def _keep_keep():
    return [WindowStep('a', torso=KEEP), WindowStep('b', torso=KEEP)]


def test_a_cap_inside_the_last_rescue_is_capped_not_certain():
    """Review C1 (probe A): b passes only with the rescue; the cap falls in it."""
    works = {'a': band((0.0, 0.7)), 'b': band()}
    plan = TorsoPlan(GRID, max_checks=2).plan(_keep_keep(), LIVE, start(), Fake(works, lasting=False,
                                                                              rescue={'b': band((0.0, 0.7))}))
    assert plan.outcome == 'capped' and not plan.certain
    plan = TorsoPlan(GRID, max_checks=3).plan(_keep_keep(), LIVE, start(), Fake(works, lasting=False,
                                                                              rescue={'b': band((0.0, 0.7))}))
    assert plan.ok and plan.rescued == [{'at': 'b', 'height': LIVE}]


@pytest.mark.parametrize('polls', range(6))
def test_a_stop_inside_the_last_rescue_is_stopped_not_certain(polls):
    """Review C1 (probe B)."""
    works = {'a': band((0.0, 0.7)), 'b': band()}
    count = iter(range(1000))
    plan = TorsoPlan(GRID).plan(_keep_keep(), LIVE, start(), Fake(works, lasting=False, rescue={'b': band((0.0, 0.7))}),
                                should_stop=lambda: next(count) > polls)
    assert plan.outcome in ('stopped', 'planned') and not plan.certain


def test_an_exhausted_local_search_is_not_certain():
    """Review C2 (probe C, C2): the rescue passes only from a plan other than the first that reaches the step, so the
    search finds nothing although a plan exists: infeasible, but not certain."""
    works = {'a': band((0.3, 0.3), (LIVE, LIVE)), 'b': band()}
    fake = Fake(works, lasting=False, rescue={'b': lambda h, st: ('a', 0.3) in st['trail']})
    plan = TorsoPlan(GRID).plan(window(works=works), LIVE, start(), fake)
    assert not plan.ok and plan.outcome == 'infeasible' and not plan.certain
    works = {'c': band((0.6, 0.75), (0.4, 0.4)), 'set_down': band()}
    fake = Fake(works, rides=lambda step, a, b: 'ride collides' if a >= 0.6 and b <= 0.35 else None, lasting=False,
                rescue={'set_down': band((0.3, 0.3))})
    plan = TorsoPlan(GRID_S1).plan(window(works=works), 0.7, start(0.7), fake)
    assert not plan.certain
    plan = TorsoPlan(GRID, rescue=False).plan(window(works={**TOP, 's2_s6': band()}), LIVE, start(),
                                              Fake({**TOP, 's2_s6': band()}, lasting=False))
    assert not plan.certain


def test_branches_cut_at_push_time_are_not_certain():
    """Review C3 (probe 4): the I1 case with one branch."""
    works = {**TOP, 's2_s6': lambda h, st: abs(h - 0.3) < 1e-6 and ('s2_s3', 0.3) in st['trail']}
    plan = TorsoPlan(GRID, branches=1, max_checks=None).plan(window(works=works), LIVE, start(),
                                                             Fake(works, lasting=False))
    assert not plan.ok and not plan.certain


def test_a_step_after_an_off_grid_pin_is_not_hopeless_while_the_pin_height_may_work():
    """Probe D: every grid height of x is out of reach, the pinned 0.33 is not."""
    works = {'pin': band((0.0, 0.75)), 'x': band((0.33, 0.33))}
    steps = [WindowStep('pin', torso=0.33), WindowStep('x')]
    fake = Fake(works, bound=lambda i, h: 'out of reach' if i == 'x' and abs(h - 0.33) > 1e-6 else None)
    plan = TorsoPlan(GRID).plan(steps, LIVE, start(), fake)
    assert plan.ok and plan.heights == {'pin': 0.33, 'x': 0.33}
    works = {'pin': band((0.0, 0.75)), 'x': band()}
    out = Fake(works, bound=lambda i, h: 'out of reach' if i == 'x' else None)
    plan = TorsoPlan(GRID).plan(steps, LIVE, start(), out)
    assert not plan.ok and plan.certain                  # every height, the pin's too, out of reach: proven


def test_a_pin_out_of_reach_exactly_is_not_hopeless_while_a_height_within_tolerance_of_it_works():
    """Re-review 3 P1 (probe 6 E): x is pinned within tolerance of the start height; the pin itself is out of reach,
    the start height is not, and a plan reaches x at it (a at 0.3, b back at the start height)."""
    pin = round(LIVE + 0.0008, 6)
    works = {'a': band((LIVE, LIVE), (0.3, 0.3)),
             'b': lambda h, st: abs(h - 0.3) < 1e-6 or (abs(h - LIVE) < 1e-6 and ('a', 0.3) in st['trail']),
             'x': band((LIVE, LIVE))}
    bound = lambda i, h: 'out of reach: x' if i == 'x' and abs(h - pin) < 1e-6 else None   # noqa: E731
    steps = [WindowStep('a'), WindowStep('b'), WindowStep('x', torso=pin)]
    plan = TorsoPlan(GRID).plan(steps, LIVE, start(), Fake(works, lasting=False, bound=bound))
    assert not plan.certain
    assert plan.ok and plan.heights['x'] == LIVE, plan.message


def test_skill_checks_put_a_step_where_its_base_stands_and_report_progress_and_seed_the_rescue_from_a_band():
    """T2/T4: a step's predicted base reaches IkClient.at_base (world), progress is called per step check, and the
    rescue starts IK from the arms of F17's remembered band for the same search."""
    t = _fake_ik()
    worlds, seeds = [], []

    class Ik(t.FakeIk):
        def check_height(self, samples, height, current, fixed=None, elbow=None, allow_flip=False,
                         global_search=False, should_stop=lambda: False, hint=None):
            worlds.append(self.world)
            seeds.append(hint)
            return SimpleNamespace(solution={'left_j1': 0.2}, cost=0.0)
    ik = Ik(reach=lambda tool, target, h: 0.0)
    skill = SimpleNamespace(ik=ik, _walk_lines=None, _ahead_check=None, _state_at=None)
    seen = []
    checks = SkillChecks(skill, world=lambda moved: ('world_joint', moved), progress=lambda step: seen.append(step.id))
    moved = np.eye(4)
    there = WindowStep('hook', targets=[t.TOOLS], base=moved)
    assert checks.step(there, 0.3, t.now(0.3)).ok and worlds[-1] == ('world_joint', moved) and ik.world is None
    assert checks.step(WindowStep('lift', targets=[t.TOOLS]), 0.3, t.now(0.3)).ok and worlds[-1] is None
    assert seen == ['hook', 'lift']
    key = ik._band_key(_band_samples(there), {}, None, False)
    ik.bands[key] = (0.33, {'left_j1': 0.9})
    assert checks.band(there) == (0.33, {'left_j1': 0.9})
    checks.step(there, 0.3, t.now(0.3), rescue=True)
    assert seeds[-1] == {'left_j1': 0.9} and seeds[-2] is None


def _band_samples(step):
    from taskweave_skills.ik import Samples
    return Samples(step.targets)


def test_refine_halves_toward_the_height_before_the_change_and_keeps_off_the_very_edge():
    """F17's refine at a change step: c works up to 0.37 (0.3 planned, 0.4 out): 0.35 passes, 0.375 does not, so
    one finest step of margin from it: 0.325."""
    works = {'c': band((0.0, 0.37)), 'd': band((0.0, 0.7))}
    steps = [WindowStep('c'), WindowStep('d')]
    fake = Fake(works, lasting=False)
    found = TorsoPlan(GRID).refine(steps, 0.3, start(LIVE), fake, LIVE, 2)
    assert found[0] == 0.325 and set(found[1]) == {'c', 'd'} and ('c', LIVE, 0.35) in fake.ridden
    stops = iter([False, False, True])
    assert TorsoPlan(GRID).refine(steps, 0.3, start(LIVE), Fake(works, lasting=False), LIVE, 2,
                                  lambda: next(stops, True)) is None       # cut short: the planned height
    assert TorsoPlan(GRID).refine(steps, 0.3, start(LIVE), Fake({'c': band((0.0, 0.3)), 'd': works['d']}), LIVE,
                                  2) is None                              # nothing between passes: keep 0.3
    found = TorsoPlan(GRID).refine(steps, 0.3, start(LIVE), Fake({'c': band((0.0, 0.39)), 'd': works['d']}), LIVE, 3)
    assert found[0] == 0.375                     # 0.3875 passes on the very edge (0.4 out): its inner 0.375
    assert TorsoPlan(GRID).refine(steps, 0.3, start(LIVE), fake, LIVE, 0) is None


def test_a_check_cut_short_by_a_stop_is_not_remembered():
    """T4: the time budget runs out during a check (a ride, an IK call): the next plan must make it again."""
    memory, stop = {}, {'now': False}

    class Late(Fake):
        def step(self, step, height, state, rescue=False):
            stop['now'] = stop['now'] or step.id == 'b'       # the budget ends while b is checked
            return super().step(step, height, state, rescue)
    steps = [WindowStep('a', key='a'), WindowStep('b', key='b')]
    works = {'a': band((0.0, 0.75)), 'b': band((0.0, 0.75))}
    plan = TorsoPlan(GRID, memory=memory).plan(steps, LIVE, start(), Late(works), should_stop=lambda: stop['now'])
    assert plan.outcome == 'stopped' and any('"a"' in key for key in memory)
    assert not any('"b"' in key for key in memory)
