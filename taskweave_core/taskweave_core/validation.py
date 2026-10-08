"""Plan validation: everything that can be checked before the robot moves.

Checks: the robot profile exists, every skill is in the catalog, every argument object matches the skill's
JSON Schema, every point a skill argument refers to exists in config, and ``replan_from`` targets exist.

Skills mark point arguments in their schema with ``"x-taskweave-point": "base_target" | "object_point"``.
Arguments that contain placeholders are only checked once their values are known (at step start).
"""

from __future__ import annotations

import copy
from collections.abc import Mapping

from jsonschema import Draft202012Validator

from . import placeholders
from .config import ConfigError, ConfigStore
from .plan import Plan

POINT_KEYWORD = 'x-taskweave-point'


def validate_plan(plan: Plan, catalog: Mapping[str, dict], store: ConfigStore) -> list[str]:
    """Return a list of human-readable problems; empty means the plan may start.

    ``catalog`` maps skill name -> JSON Schema of its arguments.
    """
    problems: list[str] = []
    try:
        store.robot(plan.robot)
    except ConfigError as error:
        problems.append(f'robot {plan.robot!r}: {error}')
    ids = [step.id for step in plan.steps]
    for duplicate in sorted({i for i in ids if ids.count(i) > 1}):
        problems.append(f'step id {duplicate!r} is used more than once')
    for step in plan.steps:
        where = f'step {step.id!r} ({step.skill})'
        if step.replan_from and step.replan_from not in ids:
            problems.append(f'{where}: replan_from {step.replan_from!r} is not a step of this plan')
        if step.skill not in catalog:
            problems.append(f'{where}: unknown skill; available: {", ".join(sorted(catalog)) or "none"}')
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
        elif kind == 'object_point':
            obj, _, point = str(value).partition('.')
            store.object_point(obj, point)
        else:
            return f'unknown point kind {kind!r} in skill schema'
    except KeyError as error:
        return str(error.args[0]) if error.args else repr(error)
    except ConfigError as error:
        return str(error)
    return None
