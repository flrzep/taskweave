"""Torso height search (IkClient.solve_with_torso) with a fake IK and a fake validity check (no ROS needed)."""

from types import SimpleNamespace

import jsonschema
import numpy as np
import pytest

from taskweave_core.config import schemas
from taskweave_skills.ik import IkClient, IkError, TorsoError

GRID = [round(0.1 * i, 1) for i in range(0, 8)]   # 0.0 .. 0.7
TOOLS = {'left': {'tcp': [0.6, 0.2, 0.3], 'approach': [1, 0, 0], 'closing': [0, 1, 0]},
         'right': {'tcp': [0.6, -0.2, 0.3], 'approach': [1, 0, 0], 'closing': [0, 1, 0]}}


class FakeIk(IkClient):
    """solve(): ``reach(tool, target, height)`` -> an arm joint value or None (out of reach); valid(): ``collides``
    (joints) -> True when the state is in collision. No ROS."""

    def __init__(self, reach=lambda tool, target, h: 0.0, collides=lambda joints: False, weight=1.0, grid=GRID,
                 accept=None):
        self.torso = {'joint': 'lift', 'candidates': grid, 'weights': {'torso': weight}, 'accept_cost': accept}
        self.choose, self.last, self.validity = None, {}, object()
        self.groups, self.joint_pattern = {'left': 'left_arm', 'right': 'right_arm'}, '{tool}_j[12]'
        self.reach, self.collides, self.solved, self.checked = reach, collides, [], []

    def solve(self, tools, fixed, should_stop=lambda: False, current=None, allow_flip=True, elbow=None,
              global_search=True):
        result, self.last = {}, {}
        for tool, target in tools.items():
            value = self.reach(tool, target, fixed['lift'])
            self.solved.append((tool, fixed['lift']))
            if value is None:
                raise IkError(f'no IK solution for {tool}: out of reach')
            result.update({f'{tool}_j1': value, f'{tool}_j2': 0.0})
            self.last[tool] = {'flipped': False, 'candidates': 1}
        return result

    def valid(self, joints, group):
        self.checked.append((dict(joints), group))
        return not self.collides(joints)


def now(h):
    return {'lift': h, 'left_j1': 0.0, 'left_j2': 0.0, 'right_j1': 0.0, 'right_j2': 0.0}


def test_the_least_torso_move_among_the_feasible_heights_wins():
    ik = FakeIk(reach=lambda tool, target, h: 0.0 if h >= 0.3 else None)
    assert ik.solve_with_torso([TOOLS], {}, now(0.1)).height == pytest.approx(0.3)   # nearest feasible above
    assert ik.solve_with_torso([TOOLS], {}, now(0.55)).height == pytest.approx(0.55)   # the current one is a candidate
    assert ik.solve_with_torso([TOOLS], {}, now(0.7)).height == pytest.approx(0.7)


def test_both_hands_are_solved_at_the_one_height_and_the_hand_that_fails_rejects_it():
    ik = FakeIk(reach=lambda tool, target, h: None if tool == 'right' and h < 0.4 else 0.0)
    choice = ik.solve_with_torso([TOOLS], {}, now(0.1))
    assert choice.height == pytest.approx(0.4) and choice.fixed['lift'] == pytest.approx(0.4)
    assert set(choice.solution) == {'left_j1', 'left_j2', 'right_j1', 'right_j2'}
    reasons = {r['height']: r['reason'] for r in choice.rejected}
    assert 'right' in reasons[0.1] and 'right' in reasons[0.3]


def test_the_same_input_twice_gives_the_same_height_and_report():
    def arm_cost(tool, target, h):                     # a wiggly landscape with exact ties
        return abs(h - 0.35) % 0.1
    first = FakeIk(reach=arm_cost).solve_with_torso([TOOLS], {}, now(0.3))
    second = FakeIk(reach=arm_cost).solve_with_torso([TOOLS], {}, now(0.3))
    assert first.height == second.height and first.cost == second.cost and first.rejected == second.rejected


def test_a_tie_goes_to_the_smaller_move_then_the_lower_height():
    ik = FakeIk(weight=0.0)                            # no torso cost: 0.3 and 0.5 tie from 0.4
    assert ik.solve_with_torso([TOOLS], {}, now(0.4)).height == pytest.approx(0.4)
    ik = FakeIk(weight=0.0, reach=lambda tool, target, h: 0.0 if h in (0.3, 0.5) else None)
    assert ik.solve_with_torso([TOOLS], {}, now(0.4)).height == pytest.approx(0.3)


def test_the_torso_weight_trades_torso_travel_against_arm_motion():
    class Arm(FakeIk):                                 # the arm cost falls with the height: 0.2 per m of lift
        def _arm_cost(self, arm, current):
            return 0.2 * (0.7 - arm['left_j1'])
    reach = lambda tool, target, h: h                  # noqa: E731
    assert Arm(reach=reach, weight=0.1, accept=0).solve_with_torso([TOOLS], {}, now(0.1)).height == pytest.approx(0.7)
    assert Arm(reach=reach, weight=1.0, accept=0).solve_with_torso([TOOLS], {}, now(0.1)).height == pytest.approx(0.1)


def test_a_height_where_the_idle_arm_collides_is_rejected():
    # the state checked holds the torso, the solved arms and the idle arm as it is now (current)
    ik = FakeIk(collides=lambda joints: joints['lift'] < 0.35)
    choice = ik.solve_with_torso([{'left': TOOLS['left']}], {}, now(0.1))
    assert choice.height == pytest.approx(0.4)
    reasons = {r['height']: r['reason'] for r in choice.rejected}
    assert reasons[0.1] == reasons[0.3] == 'collision (idle arm included)'
    assert all(set(j) == set(now(0.1)) and group == 'left_arm' for j, group in ik.checked)
    assert all(j['right_j1'] == 0.0 and j['right_j2'] == 0.0 for j, _ in ik.checked)       # the idle arm, as it is


def test_no_height_works_names_every_height_and_its_reason():
    ik = FakeIk(reach=lambda tool, target, h: None)
    with pytest.raises(TorsoError, match='no torso height works') as raised:
        ik.solve_with_torso([TOOLS], {}, now(0.3))
    assert sorted(r['height'] for r in raised.value.rejected) == [*GRID]
    assert all('out of reach' not in r['reason'] and 'no IK solution' in r['reason'] for r in raised.value.rejected)
    assert isinstance(raised.value, IkError)


def test_a_line_is_sampled_and_a_height_that_fails_mid_line_is_rejected():
    def line(height):                                  # the goal first, then points along the line (x 0.0 .. 0.6)
        return [{'left': {**TOOLS['left'], 'tcp': [x, 0.2, 0.3]}} for x in (0.6, 0.0, 0.2, 0.4)]

    def reach(tool, target, h):                        # at 0.1 and below the middle of the line is out of reach
        return None if h <= 0.1 and target['tcp'][0] == 0.2 else 0.0
    ik = FakeIk(reach=reach)
    choice = ik.solve_with_torso(line, {}, now(0.1))
    assert choice.height == pytest.approx(0.2)
    first = {r['height']: r['reason'] for r in choice.rejected}[0.1]
    assert first.startswith('sample 2')
    assert [h for tool, h in ik.solved if h == 0.1] == [0.1, 0.1, 0.1]    # goal, start, then the failing sample


def test_the_search_stops_when_no_farther_height_can_win_and_says_so():
    ik = FakeIk(weight=1.0, accept=-1)                 # never accepted at once: the search runs
    choice = ik.solve_with_torso([TOOLS], {}, now(0.5))
    assert choice.height == pytest.approx(0.5) and choice.cost == pytest.approx(0.0)
    assert {r['height'] for r in choice.rejected} == {h for h in GRID if h != 0.5}
    assert all(r['reason'].startswith('not tried') for r in choice.rejected)
    assert {h for _, h in ik.solved} == {0.5}


def test_the_other_fixed_joints_stay_and_a_missing_torso_state_is_an_error():
    seen = []

    class Spy(FakeIk):
        def solve(self, tools, fixed, *args, **kwargs):
            seen.append(dict(fixed))
            return super().solve(tools, fixed, *args, **kwargs)
    choice = Spy().solve_with_torso([TOOLS], {'head': 0.2}, now(0.3))
    assert all(f['head'] == 0.2 for f in seen) and choice.fixed == {'head': 0.2, 'lift': 0.3}
    with pytest.raises(IkError, match='no state'):
        Spy().solve_with_torso([TOOLS], {}, {'left_j1': 0.0})


def test_without_the_check_service_the_state_counts_as_valid():
    ik = FakeIk(collides=lambda joints: True)
    ik.validity = None
    assert ik.solve_with_torso([TOOLS], {}, now(0.3)).height == pytest.approx(0.3)


def test_tcps_at_moves_the_hand_with_the_torso_through_the_chain():
    class Chain:
        def fk(self, state):
            pose = np.eye(4)
            pose[2, 3] = 0.5 + state['lift']
            return pose
    ik = FakeIk()
    ik.chains, ik.tools = {'left': (Chain(), [])}, {'left': {'link': 'l7', 'xyz': [0.1, 0, 0], 'rpy': [0, 0, 0]},
                                                    'right': {'link': 'r7'}}
    poses = ik.tcps_at(['left', 'right'], {'lift': 0.2})
    assert set(poses) == {'left'} and poses['left'][:3, 3] == pytest.approx([0.1, 0.0, 0.7])


# ---- the old behaviour without ik.torso ------------------------------------------------------------------------

def test_a_robot_without_ik_torso_keeps_the_torso_fallback_list():
    from taskweave_skills.gripper_motion import MoveGripper, Target
    leg = Target('x', ['left'], TOOLS)
    skill = SimpleNamespace(ik=SimpleNamespace(torso=None))
    assert not MoveGripper._searches(skill, leg)
    skill = SimpleNamespace(ik=None)
    assert not MoveGripper._searches(skill, leg)
    skill = SimpleNamespace(ik=SimpleNamespace(torso={'joint': 'lift'}))
    assert MoveGripper._searches(skill, leg)
    assert not MoveGripper._searches(skill, Target('x', ['left'], TOOLS, joints={'lift': 0.1}))   # a pose sets it
    assert not MoveGripper._searches(skill, Target('x', ['left'], {}))                            # no tool target


def test_the_profile_schema_takes_ik_torso_as_an_optional_key():
    base = {'name': 'r', 'base': {'type': 'differential', 'frame': 'b'}, 'ik': {'groups': {}, 'joints': 'j'}}
    validator = jsonschema.Draft202012Validator(schemas.ROBOT_PROFILE)
    errors = lambda doc: [e.message for e in validator.iter_errors(doc)]   # noqa: E731
    without = errors(base)
    assert errors({**base, 'ik': {**base['ik'], 'torso': {'joint': 'j', 'candidates': [0.0, 0.1],
                                                          'weights': {'torso': 2.0}, 'line_sample_m': 0.2}}}) == without
    assert errors({**base, 'ik': {**base['ik'], 'torso': {'joint': 'j'}}})                  # candidates are needed
    assert errors({**base, 'ik': {**base['ik'], 'torso': {'joint': 'j', 'candidates': [0], 'x': 1}}})
    assert schemas.default(schemas.ROBOT_PROFILE, 'ik', 'torso', 'weights', 'torso') == 1.0


# ---- the common case: the current height is fine ---------------------------------------------------------------

def test_a_feasible_current_height_is_accepted_after_one_solve_and_check_per_sample():
    ik = FakeIk()                                     # accept_cost None: any feasible current height wins at once
    choice = ik.solve_with_torso([TOOLS, TOOLS], {}, now(0.45))
    assert choice.height == pytest.approx(0.45) and choice.skipped and choice.rejected == []
    assert len(ik.solved) == 4 and len(ik.checked) == 2        # 2 samples x 2 hands solved, 2 states checked
    assert choice.report()['skipped']


def test_a_current_height_whose_arm_cost_is_too_high_starts_the_search():
    class Arm(FakeIk):
        def _arm_cost(self, arm, current):             # the arm is cheaper the higher the torso
            return 2.0 - arm['left_j1']
    ik = Arm(reach=lambda tool, target, h: h, weight=0.1, accept=1.5)
    assert ik.solve_with_torso([TOOLS], {}, now(0.1)).height == pytest.approx(0.7)
    ik = Arm(reach=lambda tool, target, h: h, weight=0.1, accept=1.5)
    choice = ik.solve_with_torso([TOOLS], {}, now(0.6))        # cost 1.4 at 0.6: accepted
    assert choice.height == pytest.approx(0.6) and len(ik.solved) == 2


def test_the_answers_of_the_search_are_reused_by_the_move_that_follows(monkeypatch):
    ik = FakeIk()
    ik.memo = {}
    ik.solve_with_torso([TOOLS], {}, now(0.45))
    assert len(ik.memo) == 1
    key = ik._memo_key(TOOLS, {'lift': 0.45}, now(0.45), True, None)
    assert key in ik.memo


def test_staying_at_the_current_height_skips_the_state_check_when_ik_avoids_collisions():
    ik = FakeIk(collides=lambda joints: True)
    ik.avoid_collisions = True
    choice = ik.solve_with_torso([TOOLS, TOOLS], {}, now(0.45))
    assert choice.height == pytest.approx(0.45) and ik.checked == [] and len(ik.solved) == 4
