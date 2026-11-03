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
    or None. Counts the calls; the state carries the trail of (step, height) so chaining is visible."""

    def __init__(self, works, rides=None, bound=None, lasting=True):
        self.works, self.rides, self.out, self.lasting = works, rides or (lambda *a: None), bound, lasting
        self.steps, self.ridden, self.seen = [], [], []

    def bound(self, step, height):
        return self.out(step.id, height) if self.out else None

    def step(self, step, height, state):
        self.steps.append((step.id, height))
        self.seen.append((step.id, state['trail']))
        ok = self.works[step.id]
        if not (ok(height, state) if ok.__code__.co_argcount == 2 else ok(height)):
            return Check(False, reason='no IK solution', lasting=self.lasting)
        return Check(True, {'lift': height, 'trail': (*state['trail'], (step.id, height))}, cost=0.1)

    def ride(self, step, state, height):
        self.ridden.append((step.id, state['lift'], height))
        reason = self.rides(step.id, round(state['lift'], 4), round(height, 4))
        if reason:
            return Check(False, reason=reason)
        return Check(True, {'lift': height, 'trail': state['trail']})


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
    assert all(step == 's2_s5' for step, _, _ in fake.ridden)                          # no earlier change tried
    assert plan.checks <= 15 and plan.report()['changes'] == plan.changes


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
    assert len(fake.steps) == len(set(fake.steps))     # a lasting failure is not checked again from another state


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


# ---- the real checks from a skill, with the fake IK of test_ik_torso

def test_skill_checks_solve_one_height_walk_the_lines_and_end_where_the_lines_end():
    from test_ik_torso import TOOLS, FakeIk, now
    ik = FakeIk(reach=lambda tool, target, h: 0.0 if h <= 0.35 else None)
    walked = []

    def walk(legs, current, height):
        walked.append((len(legs), current['lift'], height))
        return ('leg 1: collision' if height < 0.1 else None), {**current, 'lift': height, 'left_j1': 0.7}
    skill = SimpleNamespace(ik=ik, _walk_lines=walk, _ahead_check=None, _state_at=None)
    checks = SkillChecks(skill)
    step = WindowStep('s2_s6', targets=[TOOLS], lines=[{'left': np.eye(4)}], hands=('left', 'right'))
    ok = checks.step(step, 0.3, now(0.3))
    assert ok.ok and ok.state['left_j1'] == 0.7 and walked[-1] == (1, 0.3, 0.3)
    out = checks.step(step, 0.4, now(0.4))
    assert not out.ok and out.lasting and 'no IK solution' in out.reason
    line = checks.step(step, 0.0, now(0.0))
    assert not line.ok and not line.lasting and line.reason == 'line: leg 1: collision'
    assert checks.step(WindowStep('drive'), 0.3, now(0.3)).ok           # nothing to reach: the arms stay
    assert checks.bound(step, 0.3) is None                               # no chains: no bound


def test_skill_checks_ride_in_samples_with_the_held_hands_following():
    from test_ik_torso import FakeIk, now
    ik = FakeIk(collides=lambda joints: abs(joints['lift'] - 0.5) < 1e-6)
    ik.torso['line_sample_m'] = 0.2
    ik.tcps_at = lambda tools, state: {t: np.eye(4) * state['lift'] for t in tools}
    calls = []

    def state_at(hands, current, height, poses=None):
        calls.append((round(current['lift'], 4), round(height, 4), round(float(poses['left'][0, 0]), 4)))
        return {**current, 'lift': height, 'left_j1': height}, None
    checks = SkillChecks(SimpleNamespace(ik=ik, _state_at=state_at))
    ride = checks.ride(WindowStep('set_down', hands=('left',)), now(0.7), 0.3)
    assert not ride.ok and ride.reason == 'collision at 0.500' and calls == [(0.7, 0.5, 0.7)]
    calls.clear()
    ride = checks.ride(WindowStep('set_down', hands=('left',)), now(0.6), 0.2)
    assert ride.ok and ride.state['lift'] == 0.2 and calls == [(0.6, 0.4, 0.6), (0.4, 0.2, 0.4)]


def test_state_at_takes_predicted_poses_instead_of_the_live_ones():
    pytest.importorskip('sensor_msgs')
    from taskweave_skills.gripper_motion import MoveGripper
    solved = []
    ik = SimpleNamespace(torso={'joint': 'lift'}, joint_motion=lambda joint, state, h: np.eye(4),
                         solve=lambda tools, fixed, **kw: solved.append(tools) or {'left_j1': 1.0})

    def live(hands):
        raise AssertionError('the live poses were read')
    skill = SimpleNamespace(ik=ik, hold=SimpleNamespace(hand_holds={'left': 'box'}, tcp_poses=live),
                            p=SimpleNamespace(position_tolerance=1e-3))
    pose = np.eye(4)
    pose[:3, 3] = [0.5, 0.1, 0.2]
    state, problem = MoveGripper._state_at(skill, ['left'], {'lift': 0.7}, 0.3, poses={'left': pose})
    assert problem is None and state == {'lift': 0.3, 'left_j1': 1.0} and solved[0]['left']['tcp'] == [0.5, 0.1, 0.2]


def test_plan_rejects_nothing_on_an_empty_window():
    plan = TorsoPlan(GRID).plan([], LIVE, start(), Fake({}))
    assert plan.ok and plan.heights == {} and plan.checks == 0


@pytest.mark.parametrize('rule', [KEEP, 0.4925])
def test_a_keep_or_a_pin_at_the_current_height_is_no_change(rule):
    plan = TorsoPlan(GRID).plan(window({'s1_s7': rule}), LIVE, start(), Fake(TOP))
    assert plan.heights['s1_s7'] == LIVE and plan.changes == [{'at': 's2_s5', 'from': LIVE, 'to': 0.3}]
