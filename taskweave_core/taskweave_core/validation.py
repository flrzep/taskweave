"""Plan validation: everything that can be checked before the robot moves.

Checks: the robot profile and embodiment exist, every skill has an implementation that fits the robot and is in
the catalog, every argument object matches the skill's JSON Schema, every point a skill argument refers to exists
in config (object points also fit the robot's tools), and ``replan_from`` targets exist.

Skills mark point arguments in their schema with ``"x-taskweave-point": "target" | "joint_group" | "held_object"``,
at any depth (nested objects, oneOf variants, list items). A ``target`` is a name the store resolves (cell schema 4);
``x-taskweave-target-kinds`` next to it lists the kinds of target the field takes (base, hand, object, joints). The
other kinds (``base_target``, ``joint_pose``, ``tool_pose``, ``object_point``, ``instance_point``,
``instance_contact``, ``via_target``, ``joint_pose_only``, ``held_pose``) are the point kinds of the skills of a cell
before schema 4 (LEGACY_KINDS); on a cell of schema 4 a skill that still marks one is told to use ``target``.
A ``move`` step is also checked per kind of motion (step_info.move_problems): an option the kind does not take.
Values that contain placeholders are only checked once their values are known (at step start): the skip is by path,
so a placeholder may stand for a scalar, one vec3 component or a whole field. So are points on an object an earlier
step of the plan puts down (an argument ``place_as: <name>``, e.g. grip): it does not exist yet.
"""

from __future__ import annotations

from collections.abc import Mapping

from jsonschema import Draft202012Validator

from . import placeholders
from .config import ConfigError, ConfigStore
from .config.migrate_move import replaced_by_move_message
from .config.migrate_skills import removed_skill_message
from .embodiment import check_grasp
from .plan import Plan
from .step_info import MOVE_SKILL, move_problems, step_info

POINT_KEYWORD = 'x-taskweave-point'
KINDS_KEYWORD = 'x-taskweave-target-kinds'
LEGACY_KINDS = ('base_target', 'joint_pose', 'tool_pose', 'object_point', 'instance_point', 'instance_contact',
                'via_target', 'joint_pose_only', 'held_pose')    # the point kinds before cell schema 4


def validate_plan(plan: Plan, catalog: Mapping[str, dict], store: ConfigStore) -> list[str]:
    """Return a list of human-readable problems; empty means the plan may start.

    ``catalog`` maps skill name -> JSON Schema of its arguments.
    """
    return [text for _, _, text in _plan_problems(plan, catalog, store)]


def validate_plan_structured(plan: Plan, catalog: Mapping[str, dict], store: ConfigStore) -> list[dict]:
    """The problems of ``validate_plan`` as ``{'path': ['steps', 3, 'args', 'to', 'point'], 'message': ...}``.

    ``path`` is the place in the plan's ``steps`` list (with includes that is the expanded plan, so the index is the
    expanded step's). Plan-level problems have path ``[]``, step-level ones ``['steps', i, ...]``. The message has
    no step prefix (the string form of ``validate_plan`` has).

    Every problem also has ``origin``, the place in the mission file(s): ``None`` for plan-level problems, else
    ``{'step': index in the mission's own steps (for a step from an include: the include step's index),
    'include': None for the mission's own steps, the included mission's name for a step from an include, a list of
    names (outermost first) when includes are nested, 'inner_path': the path inside the (innermost) mission, e.g.
    ['steps', 2, 'args', 'to', 'point']}``. With a placeholder filled from an include's ``with`` the path is that
    of the placeholder in the included mission."""
    return [{'path': path, 'message': message, 'origin': _origin(plan, path)}
            for path, message, _ in _plan_problems(plan, catalog, store)]


def _origin(plan: Plan, path: list) -> dict | None:
    if len(path) < 2 or path[0] != 'steps' or not 0 <= path[1] < len(plan.steps):
        return None
    origin = plan.steps[path[1]].origin
    if origin is None:
        return None
    chain = origin['include']
    return {'step': origin['step'], 'include': None if not chain else chain[0] if len(chain) == 1 else list(chain),
            'inner_path': ['steps', origin['inner_step'], *path[2:]]}


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

        step.kind, step.resources = step_kind_resources(step, catalog, store, plan.robot)
        removed = (replaced_by_move_message(step.skill, str(store.root)) if store.targets_format() else None) \
            or removed_skill_message(step.skill, step.args, str(store.root))
        if removed:   # an old skill name: say how to migrate, whatever the catalog offers
            add(removed, here + ['skill'])
            continue
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
        if step.skill == MOVE_SKILL and store.targets_format():
            for path, message in move_problems(step.args, store, plan.robot):
                add(message, here + path, label=f'{".".join(str(part) for part in path)}: ')
        if isinstance(step.args.get('place_as'), str):
            put_down.add(step.args['place_as'])
    return problems


def step_kind_resources(step, catalog: Mapping[str, dict], store: ConfigStore,
                        robot: str) -> tuple[str | None, list[str]]:
    """The kind and resources of a step (step_info.py); a step whose references do not resolve keeps the schema's."""
    try:
        return step_info(step.skill, step.args, catalog.get(step.skill), store, robot)
    except (KeyError, ConfigError, ValueError):
        return None, list((catalog.get(step.skill) or {}).get('x-taskweave-resources') or [])


def plan_step_info(plan: Plan, catalog: Mapping[str, dict], store: ConfigStore) -> list[dict]:
    """``{'id', 'kind', 'resources'}`` per step, for the plan graph (what ``validate_plan`` also puts on the steps)."""
    return [{'id': step.id, 'kind': kind, 'resources': resources} for step in plan.steps
            for kind, resources in [step_kind_resources(step, catalog, store, plan.robot)]]


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
    """(path, kind, value, target kinds) for every value whose schema has ``x-taskweave-point``, at any depth: object
    properties, oneOf variants (the variant the value is, if it can be told), list items. ``target kinds`` is the
    schema's ``x-taskweave-target-kinds`` (a tuple, empty: any). Strings with placeholders are left out."""
    if isinstance(value, str) and placeholders.names(value):
        return
    branches = _branches(schema)
    if path and not isinstance(value, (Mapping, list)):
        found = next((node for node in branches if node.get(POINT_KEYWORD)), None)
        if found:
            kinds = next((node[KINDS_KEYWORD] for node in branches if node.get(KINDS_KEYWORD)), ())
            yield path, found[POINT_KEYWORD], value, tuple(kinds)
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
    for path, kind, value, kinds in _point_checks(args, checked):
        if (path, kind) in seen or _object_of(value) in later:
            continue
        seen.add((path, kind))
        problem = check_point(kind, value, store, robot, kinds)
        if problem:
            problems.append((['args', *path], problem))
    return problems


def validate_args(args: dict, schema: dict, store: ConfigStore, robot: str, later: set[str] = frozenset()) -> list[str]:
    """Validate one argument object; values with placeholders are skipped (at any depth), and point checks of
    objects in ``later`` (put down by an earlier step, not there yet). Strings ``args.<path>: <message>``;
    ``validate_args_structured`` has the same problems with the path as a list."""
    return [f'{".".join(str(part) for part in path)}: {message}' for path, message in
            validate_args_structured(args, schema, store, robot, later)]


def check_point(kind: str, value, store: ConfigStore, robot: str, kinds=()) -> str | None:
    """Return a problem description if a referenced point does not exist, else None. ``kinds``: for a ``target``, the
    kinds of target the field takes (base, hand, object, joints; empty: any): a target that moves something else is a
    problem. A hand target is also checked against the robot's tools."""
    try:
        if kind == 'target':
            found = store.target(str(value), robot, kinds=tuple(kinds) or None)
            if found.kind == 'hand':
                problems = check_grasp({**found.data, **found.data.get('alone', {})}, store.embodiment(robot))
                if problems:
                    return f'{value} does not fit robot {robot!r}: ' + '; '.join(problems)
        elif kind in LEGACY_KINDS and store.targets_format():
            return (f'point kind {kind!r} is for cells before schema 4: this cell has targets, the skill schema '
                    'must say "x-taskweave-point": "target"')
        elif kind == 'base_target':
            store.base_target(str(value), robot)
        elif kind == 'joint_pose':
            from .config.limits import pose_problems

            problems = pose_problems(store, robot, str(value))   # raises KeyError for an unknown pose
            if problems:
                return f'pose {value} does not fit robot {robot!r}: ' + '; '.join(problems)
        elif kind == 'tool_pose':   # a named pose that places tools (move_gripper to.pose)
            from .config.limits import pose_problems

            if not store.named_pose(robot, str(value))['tools']:    # raises KeyError for an unknown pose
                return f'pose {value} places no tools (joint values only): use move_joints'
            problems = pose_problems(store, robot, str(value))
            if problems:
                return f'pose {value} does not fit robot {robot!r}: ' + '; '.join(problems)
        elif kind == 'joint_pose_only':   # a joint pose without tool targets; a leading pose: is allowed (via items)
            name = str(value).removeprefix('pose:')
            problem = check_point('joint_pose', name, store, robot)
            if problem is None and store.named_pose(robot, name)['tools']:
                return f'pose {name} places tools: use move_gripper to.pose'
            return problem
        elif kind == 'joint_group':
            from .config.limits import joint_groups

            groups = joint_groups(store, robot)
            if str(value) not in groups:
                return f'robot {robot!r} has no joint group {value!r} (has: {", ".join(sorted(groups)) or "none"})'
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
        elif kind == 'held_pose':  # a held pose of the robot for any object type (the held type is known at run time)
            names = sorted({name for poses in store.held_poses(robot).values() for name in poses})
            if str(value) not in names:
                return (f'robot {robot!r} has no held pose {value!r} (robots/{robot}/held_poses.yaml; it has: '
                        f'{", ".join(names) or "none"})')
        elif kind == 'held_object':  # the placed object a skill acts on while it is held
            if str(value) not in store.object_instances():
                return str(store.missing_instance(
                    str(value), f'unknown placed object {value!r}: held objects are placed objects').args[0])
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
