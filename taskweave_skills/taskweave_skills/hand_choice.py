"""``hands: any_one``: pick the one free hand that can take a target (skill-api-spec 3.1).

Pure selection logic, no ROS: the skill supplies what is robot-specific (building a one-hand target, probing IK).

Rule, in this order:
    1. only free hands (a hand in a hold is never a candidate);
    2. only hands the target can apply to (``build`` returns a target; else its reason is reported);
    3. IK-feasible hands first (``probe``: a solve with the profile's ik.choose settings, no motion);
    4. then the least joint motion from the current state, motions within a tolerance of the least counting as equal
       (a near-tie is noise, not a reason);
    5. then the profile's hand order (the same answer every time).
If no hand is left, the result says why for every hand.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from taskweave_core.skill import Outcome, SkillResult


@dataclass
class Probe:
    """What trying a target on one hand showed: IK feasible, joint motion (rad, sum over the arm), why not."""
    feasible: bool = True
    motion_rad: float = 0.0
    reason: str = ''


def choose(hands: list[str], busy: dict[str, str], build, probe,
           tolerance_rad: float = 0.0) -> tuple[Any, dict] | SkillResult:
    """(the chosen hand's target, the report) or a REJECTED naming a reason per hand.

    ``hands``: the robot's hands in profile order. ``busy``: {hand: object} for hands in a hold. ``build(hand)``: the
    target for that hand alone, or a SkillResult (REJECTED) saying why it has none. ``probe(target)``: a ``Probe``.
    ``tolerance_rad``: motions this close to the least one are equal (the profile order then decides).
    The report is {'chosen': hand, 'tolerance_rad', 'candidates': {hand: {'feasible', 'motion_rad', 'reason'}}}."""
    reasons: dict[str, str] = {}
    found: list[tuple[str, Any, Probe]] = []
    for hand in hands:
        if hand in busy:
            reasons[hand] = f'holds {busy[hand]}'
            continue
        target = build(hand)
        if isinstance(target, SkillResult):
            if target.outcome is Outcome.FAILED:   # e.g. no TF: it is the same for every hand, not a refusal
                return target
            reasons[hand] = target.message
            continue
        result = probe(target)
        if result.feasible:
            found.append((hand, target, result))
        else:
            reasons[hand] = result.reason or 'not reachable'
    report = {'candidates': {hand: {'feasible': False, 'reason': why} for hand, why in reasons.items()}}
    if not found:
        text = '; '.join(f'{hand}: {why}' for hand, why in reasons.items())
        return SkillResult.rejected(f'hands any_one: no hand can take it ({text})')
    least = min(item[2].motion_rad for item in found)
    near = [item for item in found if item[2].motion_rad <= least + tolerance_rad]   # profile order kept
    for hand, _, result in found:
        report['candidates'][hand] = {'feasible': True, 'motion_rad': float(result.motion_rad)}
    report['chosen'], report['tolerance_rad'] = near[0][0], float(tolerance_rad)
    return near[0][1], report


def one_hand(rule: str, picked: dict, hands: list[str], what: str) -> list[str] | SkillResult:
    """``grip`` with ``any_one``: the one hand in ``picked`` (e.g. the hands at an object point), else REJECTED."""
    names = [hand for hand in hands if hand in picked]
    if len(names) == 1:
        return names
    if not names:
        return SkillResult.rejected(f'hands {rule}: no hand {what}')
    return SkillResult.rejected(f'hands {rule} is ambiguous: {" and ".join(names)} {what}: name the hand')
