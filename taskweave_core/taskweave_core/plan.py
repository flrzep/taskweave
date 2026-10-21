"""Plan model: what the planner hands to the task handler. No ROS dependency."""

from __future__ import annotations

import copy
import re
from collections.abc import Callable
from dataclasses import dataclass, field

DEFAULT_TIMEOUT_S = 60.0
DEFAULT_RETRIES = 0
DEFAULT_ON_FAILURE = 'abort'
DEFAULT_MAX_REPLANS = 1
MAX_INCLUDE_DEPTH = 5
STEP_SETTINGS = ('timeout_s', 'retries', 'on_failure', 'max_replans')


class IncludeError(ValueError):
    """A mission include cannot be expanded (unknown mission, missing or unknown parameter, a cycle)."""


class _OriginStep(dict):
    """A step dict that remembers where it came from (``origin``: {'step': index in the top mission's own steps,
    'include': names of the include chain, outermost first (empty for the mission's own steps), 'inner_step': index
    in the innermost mission}). An attribute, not a key: the dict itself, its equality and its serialisation are
    those of a plain step."""

    origin: dict | None = None


def _with_origin(step: dict, origin: dict) -> dict:
    tagged = _OriginStep(step)
    tagged.origin = origin
    return tagged


def _fill(value, values: dict):
    """``value`` with ``{param}`` replaced by ``values[param]``; other placeholders (world state) stay. A string that
    is just one placeholder takes the value's type."""
    if isinstance(value, str):
        whole = re.fullmatch(r'\{(\w+)\}', value)
        if whole and whole.group(1) in values:
            return copy.deepcopy(values[whole.group(1)])
        return re.sub(r'\{(\w+)\}', lambda m: str(values[m.group(1)]) if m.group(1) in values else m.group(0), value)
    if isinstance(value, dict):
        return {k: _fill(v, values) for k, v in value.items()}
    if isinstance(value, list):
        return [_fill(v, values) for v in value]
    return value


def expand_includes(mission: dict, load: Callable[[str], dict], _depth: int = 0, _stack: tuple = ()) -> dict:
    """The mission with every include step replaced by the included mission's steps (recursively): their ``{param}``
    placeholders filled from the step's ``with``, the include step's timeout_s / retries / on_failure / max_replans
    (if set) and else the included mission's defaults applied to them, ids prefixed with
    the include step's id (``s3_s2``). Every step gets an explicit id. ``load(name)`` returns a mission dict."""
    steps = []
    for number, raw in enumerate(mission['steps'], start=1):
        step_id = raw.get('id', f's{number}')
        if 'include' not in raw:
            steps.append(_with_origin({**copy.deepcopy(raw), 'id': step_id},
                                      {'step': number - 1, 'include': [], 'inner_step': number - 1}))
            continue
        name = raw['include']
        if name in _stack or _depth >= MAX_INCLUDE_DEPTH:
            raise IncludeError(f'step {step_id}: including {name!r} again ({" > ".join((*_stack, name))})')
        try:
            inner = load(name)
        except Exception as error:  # noqa: BLE001 - the loader's error, with where it came from
            raise IncludeError(f'step {step_id}: cannot include mission {name!r}: {error}') from error
        params, values = inner.get('params', {}), dict(raw.get('with', {}))
        unknown, missing = sorted(set(values) - set(params)), sorted(set(params) - set(values))
        if unknown or missing:
            raise IncludeError(f'step {step_id}: mission {name!r} ' + '; '.join(
                ([f'has no parameter {", ".join(unknown)} (it takes: {", ".join(params) or "none"})']
                 if unknown else [])
                + ([f'needs {", ".join(missing)}'] if missing else [])))
        expanded = expand_includes(inner, load, _depth + 1, (*_stack, name))
        inner_defaults = inner.get('defaults', {})
        overrides = {k: raw[k] for k in STEP_SETTINGS if k in raw}  # the include step's own settings win
        for sub in expanded['steps']:
            inner_origin = getattr(sub, 'origin', None) or {'include': [], 'inner_step': 0}
            sub_origin = {'step': number - 1, 'include': [name, *inner_origin['include']],
                          'inner_step': inner_origin['inner_step']}
            sub = {**_fill(sub, values), **overrides}
            sub.update({k: inner_defaults[k] for k in STEP_SETTINGS if k in inner_defaults and k not in sub})
            sub['id'] = f'{step_id}_{sub["id"]}'
            if sub.get('replan_from'):
                sub['replan_from'] = f'{step_id}_{sub["replan_from"]}'
            steps.append(_with_origin(sub, sub_origin))
    return {**copy.deepcopy({k: v for k, v in mission.items() if k != 'steps'}), 'steps': steps}



@dataclass
class Step:
    """One fully specified step: mission defaults are already applied."""

    id: str
    instruction: str
    skill: str
    args: dict = field(default_factory=dict)
    timeout_s: float = DEFAULT_TIMEOUT_S
    retries: int = DEFAULT_RETRIES
    on_failure: str = DEFAULT_ON_FAILURE
    replan_from: str | None = None
    max_replans: int = DEFAULT_MAX_REPLANS
    origin: dict | None = field(default=None, compare=False, repr=False)  # see _OriginStep; None: not from a mission
    # filled when the plan is validated (step_info.py): the kind of motion of a move step (base, straight, hand,
    # object, posture, joints; None for other skills or while a placeholder hides it) and the resources it uses
    kind: str | None = field(default=None, compare=False)
    resources: list[str] = field(default_factory=list, compare=False)


@dataclass
class Plan:
    plan_id: str
    goal: str
    robot: str
    steps: list[Step]
    loop: str = 'once'                 # once | forever
    max_cycles: int | None = None      # with loop: forever, stop after this many cycles
    stop_when: str | None = None       # with loop: forever, stop when this world-state key is truthy

    def index_of(self, step_id: str) -> int:
        for index, step in enumerate(self.steps):
            if step.id == step_id:
                return index
        raise KeyError(f'plan {self.plan_id} has no step {step_id!r}')

    @classmethod
    def from_mission(cls, mission: dict, plan_id: str, goal: str | None = None, robot: str = '') -> Plan:
        """Build a plan from a validated mission dict, applying the mission's defaults to every step.

        Missions may name a robot; without one they run on ``robot`` (the robot the system drives). Include steps
        must be expanded first (expand_includes, ConfigStore.expanded_mission).
        """
        defaults = mission.get('defaults', {})
        steps = []
        for number, raw in enumerate(mission['steps'], start=1):
            if 'include' in raw:
                raise IncludeError(f'step {raw.get("id", f"s{number}")} includes {raw["include"]!r}: expand includes '
                                   'first')
            steps.append(Step(
                id=raw.get('id', f's{number}'),
                instruction=raw['instruction'],
                skill=raw['skill'],
                args=copy.deepcopy(raw.get('args', {})),
                timeout_s=float(raw.get('timeout_s', defaults.get('timeout_s', DEFAULT_TIMEOUT_S))),
                retries=int(raw.get('retries', defaults.get('retries', DEFAULT_RETRIES))),
                on_failure=raw.get('on_failure', defaults.get('on_failure', DEFAULT_ON_FAILURE)),
                replan_from=raw.get('replan_from'),
                max_replans=int(raw.get('max_replans', defaults.get('max_replans', DEFAULT_MAX_REPLANS))),
                origin=getattr(raw, 'origin', None) or {'step': number - 1, 'include': [], 'inner_step': number - 1},
            ))
        return cls(plan_id=plan_id, goal=goal or mission['goal'], robot=mission.get('robot') or robot, steps=steps,
                   loop=mission.get('loop', 'once'), max_cycles=mission.get('max_cycles'),
                   stop_when=mission.get('stop_when'))
