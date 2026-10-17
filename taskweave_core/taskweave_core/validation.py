"""Plan validation: everything that can be checked before the robot moves.

Checks: the robot profile and embodiment exist, every skill has an implementation that fits the robot and is in
the catalog, every argument object matches the skill's JSON Schema, every point a skill argument refers to exists
in config (object points also fit the robot's tools), and ``replan_from`` targets exist.

Skills mark point arguments in their schema with ``"x-taskweave-point": "base_target" | "joint_pose" | "object_point"
| "instance_point" | "instance_contact" | "via_target"``, at any depth (nested objects, oneOf variants, list items).
Values that contain placeholders are only checked once their values are known (at step start): the skip is by path,
so a placeholder may stand for a scalar, one vec3 component or a whole field. So are points on an object an earlier
step of the plan puts down (an argument ``place_as: <name>``, e.g. gripper): it does not exist yet.
"""

from __future__ import annotations

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
    return [text for _, _, text in _plan_problems(plan, catalog, store)]


def validate_plan_structured(plan: Plan, catalog: Mapping[str, dict], store: ConfigStore) -> list[dict]:
    """The problems of ``validate_plan`` as ``{'path': ['steps', 3, 'args', 'to', 'point'], 'message': ...}``.

    ``path`` is the place in the plan's ``steps`` list (with includes that is the expanded plan, so the index is the
    expanded step's). Plan-level problems have path ``[]``, step-level ones ``['steps', i, ...]``. The message has
    no step prefix (the string form of ``validate_plan`` has)."""
    return [{'path': path, 'message': message} for path, message, _ in _plan_problems(plan, catalog, store)]


def _plan_problems(plan: Plan, catalog: Mapping[str, dict], store: ConfigStore) -> list[tuple[list, str, str]]:
    """(path, message, text) per problem; text is the line validate_plan returns."""
    problems: list[tuple[list, str, str]] = []
    for load in (store.robot, store.embodiment):
        try:
            load(plan.robot)
        except ConfigError as error:
            problems.append(([], str(error), f'robot {plan.robot!r}: {error}'))
    ids = [step.id for step in plan.steps]
    put_down: set[str] = set()  # objects earlier steps put down (place_as): their points exist only then
    for duplicate in sorted({i for i in ids if ids.count(i) > 1}):
        message = f'step id {duplicate!r} is used more than once'
        problems.append((['steps', ids.index(duplicate), 'id'], message, message))
    for index, step in enumerate(plan.steps):
        where = f'step {step.id!r} ({step.skill})'
        here = ['steps', index]

        def add(message: str, path: list, where: str = where, label: str = '') -> None:
            problems.append((path, message, f'{where}: {label}{message}'))

        if step.replan_from and step.replan_from not in ids:
            add(f'replan_from {step.replan_from!r} is not a step of this plan', here + ['replan_from'])
        configured = bool(_implementations(store, step.skill))
        if configured:
            implementation, reasons = _select(store, step.skill, plan.robot)
            if implementation is None:
                add(f'no implementation fits robot {plan.robot!r} ({"; ".join(reasons)})', here + ['skill'])
                continue
        if step.skill not in catalog:
            add(f'no skill server offers it; available: {", ".join(sorted(catalog)) or "none"}', here + ['skill'])
            continue
        for path, message in validate_args_structured(step.args, catalog[step.skill], store, plan.robot,
                                                      later=put_down):
            add(message, here + path, label=f'{".".join(str(part) for part in path)}: ')
        if isinstance(step.args.get('place_as'), str):
            put_down.add(step.args['place_as'])
    return problems


def _object_of(value) -> str:
    """The placed object a point reference names: stance:<instance>[/v], <instance>/<point>, <object>.<point>."""
    text = str(value).removeprefix('stance:')
    return text.split('/', 1)[0] if '/' in text else text


# ---- path-based argument checks ----------------------------------------------------------------------------------
# A path is a list of keys and list indexes into the args object. Placeholders ({param}, world state) are found by
# path, so they work at any depth: in a scalar, in one component of a vec3, or for a whole field (by: "{offset}").

SUBSCHEMAS = ('oneOf', 'anyOf', 'allOf', 'if', 'then', 'else')


def placeholder_paths(value, path: tuple = ()) -> list[tuple]:
    """The paths of the scalars (strings) under ``value`` that contain a placeholder, ``value`` itself included."""
    if isinstance(value, str):
        return [path] if placeholders.names(value) else []
    if isinstance(value, Mapping):
        return [p for key, item in value.items() for p in placeholder_paths(item, path + (key,))]
    if isinstance(value, (list, tuple)):
        return [p for index, item in enumerate(value) for p in placeholder_paths(item, path + (index,))]
    return []


def _relax(schema, path: tuple):
    """Copy of ``schema`` that accepts anything at ``path`` (in every oneOf/anyOf/... branch that reaches it)."""
    if not path:
        return True
    if not isinstance(schema, dict):
        return schema
    schema = dict(schema)
    head, rest = path[0], path[1:]
    if isinstance(head, str):
        properties = dict(schema.get('properties', {}))
        if head in properties:
            properties[head] = _relax(properties[head], rest)
            schema['properties'] = properties
        elif isinstance(schema.get('additionalProperties'), dict):   # other unknown keys keep their schema
            properties[head] = _relax(schema['additionalProperties'], rest)
            schema['properties'] = properties
    else:   # one list component: prefixItems up to it, the rest stays under items
        items, prefix = schema.get('items'), list(schema.get('prefixItems', []))
        if head < len(prefix) or isinstance(items, dict):
            prefix += [items] * (head + 1 - len(prefix))
            prefix[head] = _relax(prefix[head], rest)
            schema['prefixItems'] = prefix
    for keyword in SUBSCHEMAS:
        sub = schema.get(keyword)
        if isinstance(sub, list):
            schema[keyword] = [_relax(item, path) for item in sub]
        elif isinstance(sub, dict):
            schema[keyword] = _relax(sub, path)
    return schema


def unique_required(variants: list) -> list[list[str]]:
    """Per oneOf variant, the required keys no other variant requires."""
    required = [set(v.get('required', [])) if isinstance(v, Mapping) else set() for v in variants]
    return [sorted(own - set().union(*(r for j, r in enumerate(required) if j != i)))
            for i, own in enumerate(required)]


def pick_variant(schema: Mapping, value) -> int | None:
    """Index of the ``oneOf`` variant of ``schema`` that ``value`` is: the one whose unique required key is present
    (never whole schemas: placeholders would break that). An empty or missing value is the first variant (a new
    step). None if the schema has no oneOf, the value is not an object, or none of the variants' keys is in it."""
    variants = schema.get('oneOf') if isinstance(schema, Mapping) else None
    if not variants:
        return None
    if value is None or (isinstance(value, Mapping) and not value):
        return 0
    if not isinstance(value, Mapping):
        return None
    for index, keys in enumerate(unique_required(variants)):
        if any(key in value for key in keys):
            return index
    return None


def oneof_problems(schema, path: str = 'schema', allow_several: bool = False) -> list[str]:
    """Check every ``oneOf`` in a skill schema, at any depth: each variant needs exactly one required key that no
    other variant requires, so the editor can tell the variants apart (``allow_several``: at least one). Returns
    the problems, [] when the schema is fine. A test helper for skill schemas."""
    problems: list[str] = []
    if isinstance(schema, Mapping):
        variants = schema.get('oneOf')
        if isinstance(variants, list):
            for index, keys in enumerate(unique_required(variants)):
                title = variants[index].get('title', index) if isinstance(variants[index], Mapping) else index
                if not keys or (len(keys) > 1 and not allow_several):
                    problems.append(f'{path}.oneOf[{index}] ({title}): needs exactly one unique required key, has '
                                    f'{keys or "none"}')
        for key, value in schema.items():
            problems.extend(oneof_problems(value, f'{path}.{key}', allow_several))
    elif isinstance(schema, list):
        for index, value in enumerate(schema):
            problems.extend(oneof_problems(value, f'{path}[{index}]', allow_several))
    return problems


def _branches(schema) -> list[dict]:
    """``schema`` plus the subschemas that apply with it or as alternatives to it (oneOf, anyOf, allOf, then, else)."""
    if not isinstance(schema, dict):
        return []
    found = [schema]
    for keyword in ('oneOf', 'anyOf', 'allOf'):
        for sub in schema.get(keyword, []):
            found.extend(_branches(sub))
    for keyword in ('then', 'else'):
        found.extend(_branches(schema.get(keyword)))
    return found


def _point_checks(value, schema, path: tuple = ()):
    """(path, kind, value) for every value whose schema has ``x-taskweave-point``, at any depth: object properties,
    oneOf variants (the variant the value is, if it can be told), list items. Strings with placeholders are left out."""
    if isinstance(value, str) and placeholders.names(value):
        return
    branches = _branches(schema)
    if path and not isinstance(value, (Mapping, list)):
        kind = next((node[POINT_KEYWORD] for node in branches if node.get(POINT_KEYWORD)), None)
        if kind:
            yield path, kind, value
    if isinstance(value, Mapping):
        skipped = set()   # variants the value is not
        for node in branches:
            index = pick_variant(node, value)
            if index is not None:
                skipped |= {id(sub) for i, sub in enumerate(node['oneOf']) if i != index}
        for node in branches:
            if id(node) not in skipped:
                for key, spec in node.get('properties', {}).items():
                    if key in value:
                        yield from _point_checks(value[key], spec, path + (key,))
    elif isinstance(value, list):
        for node in branches:
            if isinstance(node.get('items'), dict):
                for index, item in enumerate(value):
                    yield from _point_checks(item, node['items'], path + (index,))


def _sorted_errors(validator, instance):
    return sorted(validator.iter_errors(instance), key=lambda e: [str(p) for p in e.path])


def validate_args_structured(args: dict, schema: dict, store: ConfigStore, robot: str,
                             later: set[str] = frozenset()) -> list[tuple[list, str]]:
    """Validate one argument object; returns ``(path, message)`` per problem, ``path`` starting with ``'args'``
    (``['args', 'to', 'point']``). Placeholder scalars and whole fields are skipped by path, at any depth, and so are
    point checks of objects in ``later`` (put down by an earlier step, not there yet).

    A failing nested ``oneOf`` is reported through the variant the value is (see ``pick_variant``), at the paths
    inside it; if it is none, as "give one of: <keys>"."""
    checked = schema
    for path in placeholder_paths(args):
        checked = _relax(checked, path)
    problems: list[tuple[list, str]] = []
    for error in _sorted_errors(Draft202012Validator(checked), args):
        base = ['args', *error.path]
        if error.validator == 'oneOf' and error.path and isinstance(error.instance, Mapping):
            index = pick_variant(error.schema, error.instance)
            if index is None:
                keys = [key for keys in unique_required(error.schema['oneOf']) for key in keys]
                problems.append((base, f'give one of: {", ".join(keys)}'))
            else:   # the variant it is: its own problems (none: only placeholders made the alternatives fail)
                problems.extend((base + list(e.path), e.message) for e in
                                _sorted_errors(Draft202012Validator(error.schema['oneOf'][index]), error.instance))
            continue
        problems.append((base, error.message))
    seen = set()
    for path, kind, value in _point_checks(args, checked):
        if (path, kind) in seen or _object_of(value) in later:
            continue
        seen.add((path, kind))
        problem = check_point(kind, value, store, robot)
        if problem:
            problems.append((['args', *path], problem))
    return problems


def validate_args(args: dict, schema: dict, store: ConfigStore, robot: str, later: set[str] = frozenset()) -> list[str]:
    """Validate one argument object; values with placeholders are skipped (at any depth), and point checks of
    objects in ``later`` (put down by an earlier step, not there yet). Strings ``args.<path>: <message>``;
    ``validate_args_structured`` has the same problems with the path as a list."""
    return [f'{".".join(str(part) for part in path)}: {message}' for path, message in
            validate_args_structured(args, schema, store, robot, later)]


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
        elif kind == 'via_target':  # a waypoint: pose:<joint pose> or <instance>/<point>
            text = str(value)
            if text.startswith('pose:'):
                return check_point('joint_pose', text.removeprefix('pose:'), store, robot)
            return check_point('instance_point', text, store, robot)
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
