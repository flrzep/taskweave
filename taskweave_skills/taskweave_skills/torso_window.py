"""The hold window of a held step (P6-10b): what the steps after it ask of the torso, predicted from the real hold and
from what ``Move`` resolved of each of them (``Move._ahead``), as torso_plan WindowSteps; and the plan kept for the
later steps of the window (``Kept``).

The window is this step and the steps after it up to the one that ends the hold or cannot be predicted (Move._ahead's
``end``). Each step is predicted from the held object's pose the steps before it leave (now: the hold's
``object_in_base``) and the base pose (where it stands now, until a base move):

    carry       ``by`` / ``at`` / turn / tilt: points.moved_object; a holding hand's move or a hand target:
                MoveObject._hand_goals (a leg per waypoint); a held pose: its path (base frame), its joints (a torso
                value pins the step); onto a contact: MoveObject._goal at the predicted base pose, the contact itself
                after an offset as ``ahead``. ``lines`` (mode linear): the hands' goals per leg; ``targets``: the last
                leg first (as the per-step search samples them).
    navigate,   a base move: the torso stays while holding, the object stays in the base frame; the later steps stand
    straight    at the target's pose, or forward / sideways of the pose before.
    posture     the torso stays, the arms swing: the arm state after it is not predicted, so the steps after it are
                uncertain (a failure there cuts the plan instead of failing the hold) and ``loose`` (no arm check).

A step's ``base`` is the base motion since now (4x4 in the base frame now) or None; ``World`` turns it into the value of
MoveIt's virtual joint for the IK and validity requests (IkClient.at_base).
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field

import numpy as np

from taskweave_core import motion
from taskweave_core.skill import SkillResult

from .ik import rpy_matrix
from .parts import RESOLVED
from .points import target_of
from .torso_plan import AUTO, KEEP, WindowStep


@dataclass
class Window:
    steps: list = field(default_factory=list)       # [WindowStep], this step first
    objects: dict = field(default_factory=dict)     # step id -> the held object (4x4, base frame) before the step
    loose: set = field(default_factory=set)         # step ids whose arm state is not predicted (after a posture move)
    end: str = ''                                   # why the window ends ('': with the cycle)

    @property
    def ids(self) -> list[str]:
        return [step.id for step in self.steps]


def planar(x: float, y: float, yaw: float) -> np.ndarray:
    m = np.eye(4)
    m[:2, :2] = [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    m[:2, 3] = x, y
    return m


def flat(matrix: np.ndarray) -> np.ndarray:
    """The planar part (x, y, yaw) of a 4x4 pose."""
    return planar(float(matrix[0, 3]), float(matrix[1, 3]), math.atan2(matrix[1, 0], matrix[0, 0]))


@dataclass
class World:
    """MoveIt's virtual joint now (``joint``, its transform ``virtual`` in the model frame) and the base in the model
    frame now (``model_base``): a base motion since now -> the joint's value then (IkClient.at_base)."""
    joint: str
    virtual: np.ndarray
    model_base: np.ndarray

    def __call__(self, moved):
        if moved is None:
            return None
        return self.joint, self.model_base @ moved @ np.linalg.inv(self.model_base) @ self.virtual


def _rounded(value, digits: int = 2):
    if isinstance(value, dict):
        return {k: _rounded(v, digits) for k, v in sorted(value.items())}
    if isinstance(value, list | tuple | np.ndarray):
        return [_rounded(v, digits) for v in value]
    return round(float(value), digits) if isinstance(value, float | int | np.floating) else value


def predict(skill, hold: dict, first: dict, ahead: list, base_now: np.ndarray | None) -> Window:
    """The window from this step (``first``: {id, label, call}, the carry call it runs) and ``ahead`` (Move._ahead).
    ``skill``: the carry part (MoveObject); ``base_now``: the base in the cell now (4x4), None if not known (a base
    move then ends the window)."""
    window, obj, moved, loose = Window(), hold['object_in_base'], None, False
    items = [{'id': first['id'], 'label': first.get('label', ''), 'part': 'carry', 'calls': [first['call']],
              'carrying': hold['object']}]
    items += [entry if 'end' in entry else {'id': entry['id'], 'label': entry['plan'].desc, 'part': entry['plan'].part,
                                            'calls': entry['plan'].calls, 'carrying': entry['plan'].carrying}
              for entry in ahead]
    for item in items:
        sid = item['id']
        if 'end' in item:
            window.end = f'{sid}: {item["end"]}'
            break
        window.objects[sid] = obj
        if item['part'] in ('navigate', 'straight'):
            if base_now is None:
                window.end = f'{sid}: where the base stands is not known'
                break
            for call in item['calls']:
                moved = _driven(moved, call, base_now)
            step = WindowStep(sid, item['label'], drive=True, certain=not loose, base=moved)
        elif item['part'] == 'posture':
            step = WindowStep(sid, item['label'], torso=KEEP, why='a posture move', certain=not loose, base=moved)
            loose = True
        elif item['part'] == 'carry' and item['carrying'] == hold['object'] and len(item['calls']) == 1:
            found = _carry(skill, {**hold, 'object_in_base': obj}, sid, item['label'], item['calls'][0], base_now,
                           moved, not loose)
            if isinstance(found, str):
                window.end = f'{sid}: {found}'
                break
            step, obj = found
        else:
            window.end = f'{sid}: not a move of {hold["object"]}'
            break
        if not step.certain:
            window.loose.add(sid)
        window.steps.append(step)
    return window


def _driven(moved, call: dict, base_now: np.ndarray) -> np.ndarray:
    """The base motion since now after a base move (``call``: a navigate or straight call as Move made it)."""
    pose = (call.get(RESOLVED) or {}).get('pose')
    if pose is not None:          # a base target: where it ends, in the cell
        return np.linalg.inv(flat(base_now)) @ planar(pose.x, pose.y, pose.yaw)
    return (np.eye(4) if moved is None else moved) @ planar(float(call.get('distance_m', 0.0)),
                                                            float(call.get('sideways_m', 0.0)), 0.0)


def _carry(skill, hold: dict, sid: str, label: str, call: dict, base_now, moved, certain: bool):
    """(WindowStep, the object after it) of a carry call, or why it cannot be predicted."""
    from .object_motion import OBJECT_MODE

    to, resolved = call.get('to') or {}, call.get(RESOLVED) or {}
    m = motion.values(motion.resolve(skill.config, skill.robot.name, skill.name, call))
    joint, grips, hands = skill.ik.torso['joint'], hold['tcp_in_object'], list(hold['hands'])
    torso, why, fixed, ahead = AUTO if m.get('torso', 'keep') == 'auto' else KEEP, '', {}, None
    skill.predicted_base = None if moved is None or base_now is None else base_now @ moved
    try:
        if resolved.get('hand') or resolved.get('hand_target'):
            legs = skill._hand_goals(hold, to, resolved)
            if isinstance(legs, SkillResult):
                return legs.message
            poses = [pose for pose, _ in legs]
        elif 'pose' in to:
            held = resolved.get('held')
            if held is None:
                kind = skill.hold.type_of(hold, skill.config.object_instances())
                held = skill.config.held_pose(skill.robot.name, kind, to['pose'])
            fixed = dict(held['joints'])
            if joint in fixed:     # the pose sets the torso: the step is pinned there (the object rides)
                torso, why = round(float(fixed.pop(joint)), 6), f'held pose {to["pose"]} sets the torso'
            poses = [rpy_matrix(w.xyz, w.rpy) for w in held['path']]
            if not poses:
                return f'held pose {to["pose"]} has no path'
        else:
            goal = skill._goal(hold, to, resolved.get('onto'))
            if isinstance(goal, SkillResult):
                return goal.message
            poses = [goal[0]]
            if goal[2] is not None:      # onto with an offset: the contact itself comes next, in a straight line
                ahead = {hand: target_of(goal[2] @ grips[hand]) for hand in hands}
    except (KeyError, LookupError) as error:
        return str(error.args[0]) if error.args else str(error)
    finally:
        skill.predicted_base = None
    goals = [{hand: pose @ grips[hand] for hand in hands} for pose in poses]
    tools = [{hand: target_of(goal) for hand, goal in leg.items()} for leg in goals]
    lines = goals if m.get('mode', OBJECT_MODE) == 'linear' else []
    key = json.dumps(_rounded([tools, torso, fixed, ahead, m.get('elbow'), bool(lines),
                               None if moved is None else moved[:3, :].tolist()]), sort_keys=True, default=str)
    step = WindowStep(sid, label, torso=torso, why=why, certain=certain, key=key, hands=tuple(hands),
                      targets=[tools[-1], *tools[:-1]], lines=lines, ahead=ahead, elbow=m.get('elbow'), fixed=fixed,
                      base=moved)
    return step, poses[-1]


@dataclass
class Kept:
    """A plan kept for the later steps of its window: the plan (torso_plan.Plan), the window it was made for, the
    joints it started from and when."""
    plan: object
    window: Window
    start: dict
    first: str              # the step that planned
    source: str = 'planned'
    world: object = None    # World of the plan's checks (the base motions of its steps are from where it planned)

    def covers(self, sid: str, ahead_ids: list[str]) -> bool:
        """``sid`` is a step of this window and the steps after it are still the window's (the same plan)."""
        ids = self.window.ids
        if sid not in ids:
            return False
        rest = ids[ids.index(sid) + 1:]
        return ahead_ids[:len(rest)] == rest

    def before(self, sid: str) -> dict | None:
        """The joints the plan expects before step ``sid``."""
        ids = self.window.ids
        k = ids.index(sid)
        return self.start if k == 0 else self.plan.states.get(ids[k - 1])

    def mismatch(self, sid: str, current: dict, hold: dict, joint: str, pattern: str, tolerances: dict) -> str | None:
        """Why the state now is not the one the plan expects before step ``sid`` (torso within ``torso_m``, every
        holding hand's TCP within ``object_m``, the held arms' joints within ``branch_rad`` unless the step is
        ``loose``), or None."""
        expected = self.before(sid)
        if expected is None:
            return 'the plan has no state before it'
        off = abs(float(current.get(joint, math.inf)) - float(expected.get(joint, math.inf)))
        if off > tolerances['torso_m']:
            return f'the torso is {off:.3f} m off the plan'
        then = self.window.objects.get(sid)
        if then is not None:
            grips = hold['tcp_in_object']
            off = max(float(np.linalg.norm((hold['object_in_base'] @ g)[:3, 3] - (then @ g)[:3, 3]))
                      for g in grips.values())
            if off > tolerances['object_m']:
                return f'the held object is {off:.3f} m off the plan'
        if sid not in self.window.loose:
            arms = [re.compile(pattern.format(tool=hand)) for hand in hold['hands']]
            names = [n for n in expected if any(a.fullmatch(n) for a in arms)]
            off = max((abs(float(current.get(n, math.inf)) - float(expected[n])) for n in names), default=0.0)
            if off > tolerances['branch_rad']:
                return f'the arms are {off:.2f} rad off the plan'
        return None
