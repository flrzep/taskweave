"""hands any_one: the selection rule (hand_choice.py), without ROS."""

from taskweave_core.skill import Outcome, SkillResult
from taskweave_skills.hand_choice import Probe, choose, one_hand

HANDS = ['left', 'right']


def build(hand):
    return f'target_{hand}'


def test_infeasible_hands_lose_then_least_motion_then_profile_order():
    probes = {'target_left': Probe(True, 2.0), 'target_right': Probe(True, 0.5)}
    target, report = choose(HANDS, {}, build, probes.get)
    assert target == 'target_right' and report['chosen'] == 'right'
    probes['target_right'] = Probe(False, 0.0, 'no IK solution for right')
    target, report = choose(HANDS, {}, build, probes.get)
    assert target == 'target_left' and report['candidates']['right'] == {
        'feasible': False, 'motion_rad': None, 'repeatable': None, 'reason': 'no IK solution for right'}
    probes = {'target_left': Probe(True, 1.0), 'target_right': Probe(True, 1.0)}
    assert [choose(HANDS, {}, build, probes.get)[1]['chosen'] for _ in range(3)] == ['left'] * 3
    assert choose(['right', 'left'], {}, build, probes.get)[1]['chosen'] == 'right'      # the profile's order


def test_a_busy_hand_or_one_without_a_target_is_never_chosen_and_the_reasons_are_listed():
    chosen, _ = choose(HANDS, {'left': 'frame'}, build, lambda t: Probe(True, 9.0))
    assert chosen == 'target_right'
    result = choose(HANDS, {'left': 'frame'},
                    lambda hand: SkillResult.rejected(f'no pose for {hand}'), lambda t: Probe())
    assert result.outcome is Outcome.REJECTED
    assert result.message == 'hands any_one: no hand can take it (left: holds frame; right: no pose for right)'


def test_grip_takes_the_single_hand_at_a_point_else_rejects():
    assert one_hand('any_one', {'right': {}}, HANDS, 'is at a point') == ['right']
    none = one_hand('any_one', {}, HANDS, 'is at a point')
    assert none.outcome is Outcome.REJECTED and 'no hand is at a point' in none.message
    both = one_hand('any_one', {'left': {}, 'right': {}}, HANDS, 'is at a point')
    assert both.outcome is Outcome.REJECTED and 'ambiguous: left and right' in both.message


def test_motions_within_the_tolerance_are_equal_and_the_profile_order_decides():
    near = {'target_left': Probe(True, 1.0), 'target_right': Probe(True, 1.02)}
    _, report = choose(HANDS, {}, build, near.get, tolerance_rad=0.05)
    assert report['chosen'] == 'left' and report['tolerance_rad'] == 0.05
    swapped = {'target_left': Probe(True, 1.02), 'target_right': Probe(True, 1.0)}
    assert choose(HANDS, {}, build, swapped.get, tolerance_rad=0.05)[1]['chosen'] == 'left'    # noise: profile order
    assert choose(['right', 'left'], {}, build, swapped.get, tolerance_rad=0.05)[1]['chosen'] == 'right'
    far = {'target_left': Probe(True, 1.3), 'target_right': Probe(True, 1.0)}
    assert choose(HANDS, {}, build, far.get, tolerance_rad=0.05)[1]['chosen'] == 'right'        # least motion


def test_a_failed_target_is_passed_on_not_turned_into_a_refusal():
    result = choose(HANDS, {}, lambda hand: SkillResult.failed('where is the robot? no TF'), lambda t: Probe())
    assert result.outcome is Outcome.FAILED and result.message == 'where is the robot? no TF'


def test_a_hand_that_needed_the_global_search_comes_after_repeatable_ones():
    probes = {'target_left': Probe(True, 2.0), 'target_right': Probe(True, 0.5, repeatable=False)}
    _, report = choose(HANDS, {}, build, probes.get)
    assert report['chosen'] == 'left' and report['candidates']['right']['repeatable'] is False
    probes['target_left'] = Probe(True, 2.0, repeatable=False)           # none repeatable: the least motion again
    assert choose(HANDS, {}, build, probes.get)[1]['chosen'] == 'right'


def test_every_candidate_has_the_same_keys():
    probes = {'target_left': Probe(True, 2.0), 'target_right': Probe(False, 0.0, 'no IK')}
    _, report = choose(HANDS, {}, build, probes.get)
    keys = {tuple(sorted(candidate)) for candidate in report['candidates'].values()}
    assert keys == {('feasible', 'motion_rad', 'reason', 'repeatable')}
