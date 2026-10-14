"""Plan validation: everything that can be checked before the robot moves.

Checks: the robot profile and embodiment exist, every skill has an implementation that fits the robot and is in
the catalog, every argument object matches the skill's JSON Schema, every point a skill argument refers to exists
in config (object points also fit the robot's tools), and ``replan_from`` targets exist.

Skills mark point arguments in their schema with ``"x-taskweave-point": "base_target" | "joint_pose" | "object_point"
| "instance_point" | "instance_contact"``.
Arguments that contain placeholders are only checked once their values are known (at step start).
"""

from __future__ import annotations

import copy
from collections.abc import Mapping

from jsonschema import Draft202012Validator

from . import placeholders
from .config import ConfigError, ConfigStore
from .embodiment import check_grasp
from .plan import Plan

POINT_KEYWORD = 'x-taskweave-point'


def validate_plan(plan: Plan, catalog: Mapping[str, dict], store: ConfigStore) -> list[str]:
    """Return a list of human-readable problems; empty means the plan may start.

    ``catalog`` maps skill name -> JSON Schema of its arguments.
    """
    problems: list[str] = []
    for load in (store.robot, store.embodiment):
        try:
            load(plan.robot)
        except ConfigError as error:
            problems.append(f'robot {plan.robot!r}: {error}')
    ids = [step.id for step in plan.steps]
    for duplicate in sorted({i for i in ids if ids.count(i) > 1}):
        problems.append(f'step id {duplicate!r} is used more than once')
    for step in plan.steps:
        where = f'step {step.id!r} ({step.skill})'
        if step.replan_from and step.replan_from not in ids:
            problems.append(f'{where}: replan_from {step.replan_from!r} is not a step of this plan')
        configured = bool(_implementations(store, step.skill))
        if configured:
            implementation, reasons = _select(store, step.skill, plan.robot)
            if implementation is None:
                problems.append(f'{where}: no implementation fits robot {plan.robot!r} ({"; ".join(reasons)})')
                continue
        if step.skill not in catalog:
            problems.append(f'{where}: no skill server offers it; available: {", ".join(sorted(catalog)) or "none"}')
            continue
        problems.extend(f'{where}: {p}' for p in validate_args(step.args, catalog[step.skill], store, plan.robot))
    return problems


def validate_args(args: dict, schema: dict, store: ConfigStore, robot: str) -> list[str]:
    """Validate one argument object; keys whose values still contain placeholders are skipped."""
    pending = {key for key, value in args.items() if placeholders.names(value)}
    checked_schema = copy.deepcopy(schema)
    if pending and isinstance(checked_schema.get('required'), list):
        checked_schema['required'] = [key for key in checked_schema['required'] if key not in pending]
    concrete = {key: value for key, value in args.items() if key not in pending}
    problems = []
    for error in sorted(Draft202012Validator(checked_schema).iter_errors(concrete), key=lambda e: list(e.path)):
        key = '.'.join(str(part) for part in error.path)
        problems.append(f'args{"." + key if key else ""}: {error.message}')
    for key, spec in checked_schema.get('properties', {}).items():
        kind = spec.get(POINT_KEYWORD) if isinstance(spec, dict) else None
        if kind and key in concrete:
            problem = check_point(kind, concrete[key], store, robot)
            if problem:
                problems.append(f'args.{key}: {problem}')
    return problems


def check_point(kind: str, value, store: ConfigStore, robot: str) -> str | None:
    """Return a problem description if a referenced point does not exist, else None."""
    try:
        if kind == 'base_target':
            store.base_target(str(value), robot)
        elif kind == 'joint_pose':
            from .config.limits import pose_problems

            problems = pose_problems(store, robot, str(value))   # raises KeyError for an unknown pose
            if problems:
                return f'pose {value} does not fit robot {robot!r}: ' + '; '.join(problems)
        elif kind in ('object_point', 'instance_point'):
            # object_point: <instance>/<point> (a placed object) or <object>.<point> (a type's point);
            # instance_point: only <instance>/<point>, for skills that go to one placed object
            if kind == 'instance_point' and '/' not in str(value):
                example = next(iter(store.point_names(robot)['placed object points']), '<instance>/<point>')
                return f'{value} names no placed object: use <instance>/<point>, e.g. {example}'
            if '/' in str(value):
                instance, _ = store.instance_point(str(value))
                obj, point = instance.object, str(value).partition('/')[2]
            else:
                obj, _, point = str(value).partition('.')
                store.object_point(obj, point)
            problems = check_grasp(store.object(obj)['points'][point], store.embodiment(robot))
            if problems:
                return f'{value} does not fit robot {robot!r}: ' + '; '.join(problems)
        elif kind == 'instance_contact':  # <instance>/<contact>: where a held object is put
            store.instance_contact(str(value))
        else:
            return f'unknown point kind {kind!r} in skill schema'
    except KeyError as error:
        return str(error.args[0]) if error.args else repr(error)
    except ConfigError as error:
        return str(error)
    return None


def _implementations(store: ConfigStore, skill: str) -> list:
    try:
        return store.skill_config(skill).get('implementations', [])
    except ConfigError:
        return []


def _select(store: ConfigStore, skill: str, robot: str):
    try:
        return store.select_implementation(skill, robot)
    except ConfigError as error:
        return None, [str(error)]
