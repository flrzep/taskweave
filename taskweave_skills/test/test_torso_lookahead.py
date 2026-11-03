"""The torso look-ahead in the carry part (MoveObject._lookahead, _planned_choice) driven like the executive does,
step by step with its context, with fake checks (test_torso_plan.Fake) instead of IK: the plan made at the first held
step and kept for the later ones, a retry, a start mid-plan, a jog between steps, a hold no height serves, a capped or
stopped plan, a planned height that fails when its step runs, no context."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('sensor_msgs')     # the carry part imports the joint skills (ROS messages)

from lookahead_cell import Registry, build, cell, ctx_of
from test_torso_plan import GRID, LIVE, Fake, band

from taskweave_core import motion
from taskweave_core.executive import Outcome
from taskweave_core.skill import SkillResult
from taskweave_skills.points import moved_object

UP = {'hand': 'holding', 'by': [0, 0, 0.1]}
STEPS = [('a', 'move', UP), ('b', 'move', UP), ('c', 'move', UP), ('g', 'grip', {'hands': 'all', 'action': 'open'}),
         ('h', 'move', {'to': 'home'})]
WORKS = {'a': band((0.4, 0.5)), 'b': band((0.0, 0.7)), 'c': band((0.0, 0.35))}   # like lift .. hook: one change at c


class Rig:
    """The executive's side: runs the steps of STEPS one after the other through the carry part's look-ahead and
    'moves' (the torso to the height taken, the object by the step), as a fake dispatcher."""

    def __init__(self, store, works=WORKS, bound=None, lookahead=None, refine=0):
        self.registry = Registry()
        self.move, self.carry = build(store, self.registry)
        self.state = {'lift': LIVE, 'trail': ()}
        self.fake = Fake(works, bound=bound, lasting=bound is not None)
        more = {'lookahead': lookahead} if lookahead else {}
        self.carry.ik.torso = {'joint': 'lift', 'candidates': GRID, 'refine': refine, **more}
        self.carry._start = lambda: dict(self.state)
        self.carry._torso_checks = lambda ctx, world=None, progress=None, stop=None: self.fake
        self.carry._world = lambda base: None
        self.store = store

    def look(self, k, steps=STEPS, carry=None, **kw):
        """Step ``k``'s look-ahead: (refused or None, carry._look, ctx)."""
        carry = carry or self.carry
        ctx = ctx_of(steps, k, **kw)
        carry.ahead = lambda: self.move._ahead(ctx)
        call = self.move._plan(steps[k][2]).calls[0]
        m = motion.values(motion.resolve(self.store, 'duo_bot', 'move.carry', call))
        refused = carry._lookahead(ctx, self.registry.held_all()[0], call, m)
        return refused, carry._look, ctx

    def done(self, height, by=0.1):
        """The step ran: the torso at ``height``, the object moved ``by`` up (the arms as the plan said)."""
        self.state = {**self.state, 'lift': height}
        self.registry.hold['object_in_base'] = moved_object(self.registry.hold['object_in_base'], [0, 0, by])


@pytest.fixture
def store(tmp_path):
    return cell(tmp_path)


def test_the_plan_is_made_at_the_first_held_step_and_kept_for_the_later_ones(store):
    rig = Rig(store)
    refused, look, ctx = rig.look(0)
    assert refused is None and look['report']['source'] == 'planned'
    assert look['report']['heights'] == {'a': LIVE, 'b': LIVE, 'c': 0.3} and look['height'] == LIVE
    assert look['report']['changes'] == [{'at': 'c', 'from': LIVE, 'to': 0.3}]
    assert look['report']['end'] == 'g: a grip open'
    assert 'planning the torso over 3 held moves' in ctx.fed
    calls = len(rig.fake.steps), len(rig.fake.ridden)
    rig.done(LIVE)
    _, look, _ = rig.look(1)
    assert look['report'] == {'source': 'kept', 'from': 'a', 'height': LIVE} and look['height'] == LIVE
    rig.done(LIVE)
    _, look, _ = rig.look(2)
    assert look['height'] == 0.3 and look['report']['source'] == 'kept'
    assert (len(rig.fake.steps), len(rig.fake.ridden)) == calls          # the kept steps made no check


def test_a_change_step_refines_its_height_when_it_runs(store):
    rig = Rig(store, refine=2)
    rig.look(0)
    rig.done(LIVE)
    rig.look(1)
    rig.done(LIVE)
    import taskweave_skills.torso_window as tw
    predicted, real = [], tw.predict

    def spy(skill, hold, first, ahead, base_now):
        predicted.append((first['id'], hold['object_in_base'][2, 3]))
        return real(skill, hold, first, ahead, base_now)
    tw.predict = spy
    try:
        _, look, _ = rig.look(2)
    finally:
        tw.predict = real
    # 0.3 planned, 0.4 out: 0.35 passes, 0.375 does not; one finest step of margin from it: 0.325 (checked)
    assert look['height'] == 0.325 and look['report']['refined'] == {'from': 0.3, 'to': 0.325}
    assert rig.carry.kept[('p1', 1, 'box_1')].plan.changes == [{'at': 'c', 'from': LIVE, 'to': 0.325}]
    assert predicted == [('c', rig.registry.hold['object_in_base'][2, 3])]    # re-predicted from the real hold


def test_a_refine_cut_short_by_the_budget_keeps_the_planned_height(store):
    rig = Rig(store, refine=2)
    rig.look(0)
    rig.done(LIVE)
    rig.look(1)
    rig.done(LIVE)
    rig.carry.ik.torso['lookahead'] = {'budget_s': 1e-9}
    _, look, _ = rig.look(2)
    assert look['height'] == 0.3 and 'refined' not in look['report']


def test_a_retry_after_a_refusal_is_refused_again_and_a_later_proof_fails_the_step(store):
    out = lambda sid, h: 'out of reach: left target 1.40 m from its shoulder' if sid == 'c' else None   # noqa: E731
    rig = Rig(store, bound=out)
    refused, _, _ = rig.look(0)
    calls = len(rig.fake.steps)
    again, look, _ = rig.look(0)                           # on_failure retry / replan: the same step again
    assert again.outcome == Outcome.REJECTED and again.message == refused.message and len(rig.fake.steps) == calls
    assert look['report']['source'] == 'kept' and look['report']['certain']
    late = Rig(store, bound=out)
    late.fake.out = lambda sid, h: None                    # the plan at a passes; the hook goes out of reach later
    late.look(0)
    late.done(LIVE)
    late.fake.out = out
    late.state['lift'] = 0.45                              # b moved the torso before it failed: re-planned there
    failed, look, _ = late.look(1)
    assert failed.outcome == Outcome.FAILED and look['report']['source'] == 'replanned'


def test_a_retry_keeps_the_plan_while_the_state_is_the_planned_one(store):
    rig = Rig(store)
    rig.look(0)
    rig.done(LIVE)
    rig.look(1)                            # b failed before it moved: the executive runs it again
    _, look, _ = rig.look(1)
    assert look['report']['source'] == 'kept'
    rig.state['lift'] = 0.45               # b failed after the torso moved: plan again from here
    _, look, _ = rig.look(1)
    assert look['report']['source'] == 'replanned' and 'torso is 0.042 m off' in look['report']['why']
    assert look['report']['window'] == ['b', 'c']


def test_a_start_mid_plan_plans_from_there(store):
    rig = Rig(store)
    _, look, _ = rig.look(1)
    assert look['report']['source'] == 'planned' and look['report']['window'] == ['b', 'c']
    assert look['report']['changes'] == [{'at': 'c', 'from': LIVE, 'to': 0.3}]


def test_a_jog_between_steps_plans_again(store):
    rig = Rig(store)
    rig.look(0)
    rig.done(LIVE)
    rig.registry.hold['object_in_base'] = moved_object(rig.registry.hold['object_in_base'], [0.05, 0, 0])   # jogged
    _, look, _ = rig.look(1)
    assert look['report']['source'] == 'replanned' and 'held object is 0.050 m off' in look['report']['why']
    rig.done(LIVE)
    _, look, _ = rig.look(2)                # the new plan holds for the rest
    assert look['report'] == {'source': 'kept', 'from': 'b', 'height': 0.3}


def test_another_plan_or_cycle_plans_again(store):
    rig = Rig(store)
    rig.look(0)
    rig.done(LIVE)
    for kw in ({'cycle': 2}, {'plan_id': 'p2'}):
        _, look, _ = rig.look(1, **kw)
        assert look['report']['source'] == 'planned'


def test_a_hold_no_height_serves_is_refused_before_anything_moves(store):
    out = lambda sid, h: 'out of reach: left target 1.40 m from its shoulder' if sid == 'c' else None   # noqa: E731
    rig = Rig(store, bound=out)
    refused, look, _ = rig.look(0)
    assert isinstance(refused, SkillResult) and refused.outcome == Outcome.REJECTED
    assert 'torso plan over the hold of box_1 (a..c, 3 moves): no heights work: c' in refused.message
    assert refused.data['torso_plan']['certain'] and refused.data['torso_plan']['table'][2]['id'] == 'c'
    assert rig.fake.ridden == []            # nothing checked beyond what proves it, nothing moved


@pytest.mark.parametrize('lookahead, outcome', [({'max_checks': 1}, 'capped'), ({'budget_s': 1e-9}, 'stopped')])
def test_a_capped_or_stopped_plan_falls_back_to_the_per_step_choice(store, lookahead, outcome):
    rig = Rig(store, lookahead=lookahead)
    refused, look, _ = rig.look(0)
    assert refused is None and look['height'] is None and look['report']['outcome'] == outcome
    asked = []
    chosen, planned = rig.carry._planned_choice(None, lambda h: asked.append(h) or SimpleNamespace(skipped=''))
    assert asked == [None] and not planned
    rig.done(0.45)
    _, look, _ = rig.look(1)               # the later steps choose as without a plan
    assert look['height'] is None and look['report']['source'] == 'per step'


def test_a_planned_height_is_checked_when_the_step_runs_and_planned_again_once_if_it_fails(store):
    rig = Rig(store)
    _, look, ctx = rig.look(0)
    asked = []
    chosen, planned = rig.carry._planned_choice(ctx, lambda h: asked.append(h) or SimpleNamespace(skipped=''))
    assert asked == [LIVE] and planned and chosen.skipped == 'planned by the torso look-ahead'
    asked.clear()

    def fails_planned(h):
        asked.append(h)
        return SkillResult.failed('no IK solution') if h is not None else SimpleNamespace(skipped='')
    chosen, planned = rig.carry._planned_choice(ctx, fails_planned)
    assert asked == [LIVE, LIVE, None] and not planned           # re-planned once (same answer), then per step
    assert rig.carry._look['report']['source'] == 'replanned' and 'failed here' in rig.carry._look['report']['why']


def test_without_context_or_with_lookahead_off_the_step_chooses_as_today(store):
    rig = Rig(store)
    ctx = SimpleNamespace(context={}, step_id='', should_stop=False, feedback=lambda *a: None)
    call = rig.move._plan(UP).calls[0]
    assert rig.carry._lookahead(ctx, rig.registry.held_all()[0], call, {'torso': 'auto'}) is None
    assert rig.carry._look is None and rig.fake.steps == []
    asked = []
    rig.carry._planned_choice(ctx, lambda h: asked.append(h) or SimpleNamespace(skipped=''))
    assert asked == [None]
    off = Rig(store, lookahead={'enabled': False})
    refused, look, _ = off.look(0)
    assert refused is None and look is None and off.fake.steps == []
    keep = Rig(store)                       # a step with torso keep plans nothing
    ctx = ctx_of(STEPS, 0)
    assert keep.carry._lookahead(ctx, keep.registry.held_all()[0], call, {'torso': 'keep'}) is None
    assert keep.fake.steps == []


def test_the_result_says_what_the_look_ahead_did(store):
    rig = Rig(store)
    rig.look(0)
    result = rig.carry._with_plan(SkillResult.succeeded('moved', torso=[{'height': LIVE, 'planned': True}]))
    assert result.data['torso_plan']['source'] == 'planned' and np.isclose(result.data['torso'][0]['height'], LIVE)


def test_a_posture_move_ends_the_plan_and_the_first_carry_after_it_plans_again(store):
    steps = [STEPS[0], ('p', 'move', {'elbow': 'down'}), *STEPS[1:]]
    rig = Rig(store, works={**WORKS, 'p': band((0.0, 0.75))})
    _, look, _ = rig.look(0, steps)
    assert look['report']['window'] == ['a', 'p'] and look['report']['until'].startswith('p: a posture move')
    assert rig.fake.steps == [('a', LIVE), ('p', LIVE)]       # the steps after it: no check (the arms are unknown)
    rig.done(LIVE)
    _, look, _ = rig.look(2, steps)                           # b: planned from the real arms, to the end
    assert look['report']['source'] == 'planned' and look['report']['window'] == ['b', 'c']


def test_a_step_out_of_reach_after_a_posture_move_still_refuses_the_first_step(store):
    steps = [STEPS[0], ('p', 'move', {'elbow': 'down'}), *STEPS[1:]]
    out = lambda sid, h: 'out of reach: left target 1.40 m from its shoulder' if sid == 'c' else None   # noqa: E731
    rig = Rig(store, works={**WORKS, 'p': band((0.0, 0.75))}, bound=out)
    refused, _, _ = rig.look(0, steps)
    assert refused is not None and refused.outcome == Outcome.REJECTED and refused.data['torso_plan']['certain']
    assert 'no heights work: c' in refused.message and rig.fake.steps == [('a', LIVE), ('p', LIVE)]


def test_a_planned_ride_keeps_the_arms_and_an_unplanned_one_solves_them_as_before():
    """Fix round 1 I3: the executed torso move of a planned height is the ride the plan checked (arms unchanged)."""
    from taskweave_skills.object_motion import MoveObject

    def lifted(h):
        m = np.eye(4)
        m[2, 3] = h
        return m
    solved = []
    hold = {'object': 'box_1', 'hands': ['left'], 'tcp_in_object': {'left': np.eye(4)}, 'object_in_base': np.eye(4)}
    skill = SimpleNamespace(
        ik=SimpleNamespace(torso={'joint': 'lift'}, joint_motion=lambda j, state, h: lifted(h - state['lift']),
                           tcps_at=lambda tools, state: {t: lifted(state['lift']) for t in tools},
                           solve=lambda tools, fixed, *a, **kw: solved.append(tools) or {'left_j1': 0.9}),
        hold=SimpleNamespace(reanchor=lambda: True, held_all=lambda: [hold], hand_holds={'left': 'box_1'}),
        _hands_moved=lambda joints: set(), _start=lambda: {'lift': 0.5, 'left_j1': 0.1},
        p=SimpleNamespace(position_tolerance=1e-3), _arm_joints=lambda goal: [], keep_arms=True)
    ctx = SimpleNamespace(should_stop=False)
    goal, failure = MoveObject._widen(skill, ctx, {'lift': 0.3}, 'ride')
    assert failure is None and goal == {'lift': 0.3} and solved == []
    skill.keep_arms = False
    goal, _ = MoveObject._widen(skill, ctx, {'lift': 0.3}, 'ride')
    assert goal == {'lift': 0.3, 'left_j1': 0.9} and len(solved) == 1


def test_a_refusal_is_planned_again_once_the_state_changed(store):
    """Re-review N3: the operator fixed something (here: the frame moved): the retry plans again, not repeats."""
    out = lambda sid, h: 'out of reach: left target 1.40 m from its shoulder' if sid == 'c' else None   # noqa: E731
    rig = Rig(store, bound=out)
    rig.look(0)
    rig.fake.out = lambda sid, h: None
    rig.registry.hold['object_in_base'] = moved_object(rig.registry.hold['object_in_base'], [0.05, 0, 0])
    refused, look, _ = rig.look(0)
    assert refused is None and look['report']['source'] == 'planned' and look['height'] is not None


def test_a_blocked_planned_ride_is_tried_again_with_the_arms_re_solved():
    """Re-review N2: the planned ride keeps the arms; blocked, once more as without a plan (F16's re-solve)."""
    from taskweave_skills.object_motion import MoveObject
    rides = []
    skill = SimpleNamespace(keep_arms=False, node=None)
    skill._set_torso = lambda ctx, chosen, label, planner: rides.append(skill.keep_arms) or (
        SkillResult.failed('blocked') if skill.keep_arms else None)
    ctx = SimpleNamespace(should_stop=False)
    assert MoveObject._ride(skill, ctx, None, 'leg', 'auto', True) is None and rides == [True, False]
    rides.clear()
    assert MoveObject._ride(skill, ctx, None, 'leg', 'auto', False) is None and rides == [False]
