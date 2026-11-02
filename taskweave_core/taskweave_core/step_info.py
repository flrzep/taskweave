"""What a plan step is, from its arguments: the kind of motion of a ``move`` step, the resources it uses, and the
per-kind option check (the rules of config/targets.py ``MOVE_OPTIONS``).

A ``move`` step says what moves in one of its forms (``to`` a target, ``hand`` relative to now, ``base`` straight,
``elbow``, ``joints``); a ``to`` step takes its kind from the target. The kind is what the result's ``data.kind`` says
afterwards, one of the operator's five commands (navigate, straight, hand, posture, joints). The option table has one
more row, ``carry``: a hand move of a held object (an object target, ``hand: holding``), kind hand. Every other skill
has no kind and the resources its argument schema lists in ``x-taskweave-resources``.

The old form ``{object: <held | instance>, by / at}`` (cell schema 4) is ``{hand: holding, ...}`` since cell schema 5:
a schema 4 cell still runs it (``taskweave check`` notes it), a schema 5 cell gets a problem naming the migration.

``validate_plan`` uses ``move_problems`` and fills ``Step.kind`` / ``Step.resources`` from ``step_info`` (the live plan
graph lights the resources). Values with placeholders are unknown until the step starts: such a step keeps the
schema's resources and no kind.
"""

from __future__ import annotations

from collections.abc import Mapping

from . import placeholders
from .config import ConfigError, ConfigStore, packs
from .config.targets import (
    FORM_OF_TARGET,
    HOLDING,
    MOVES_WORD,
    RELATIVE_OPTIONS,
    TORSO,
    kind_of,
    move_label,
    move_option_problems,
)

MOVE_SKILL = 'move'
RESOURCES = 'x-taskweave-resources'
ORDER = ('base', 'hands', 'torso', 'joints')    # the vocabulary of resources, in the order a step lists them
OPTIONS = ('hands', 'offset_m', 'reference', 'only', 'torso', 'tolerance_deg', 'motion')
RELATIVE_FORMS = ('hand', 'carry')
FORM_KEYS = (('base', 'straight'), ('elbow', 'posture'), ('joints', 'joints'))
OBJECT_FORM_SCHEMA = 5      # the cell schema from which ``{object: ..., by / at}`` is ``{hand: holding, ...}``
MIGRATE = 'taskweave config migrate'


def old_object_form(args: Mapping) -> bool:
    """The relative move of a held object as cell schema 4 wrote it: ``object`` with by / at, no ``to``, no ``hand``
    (what ``taskweave config migrate`` rewrites)."""
    return 'object' in args and 'to' not in args and 'hand' not in args and ('by' in args or 'at' in args)


def old_object_text(args: Mapping, cell: str = '<cell dir>') -> str:
    """What the old object form is now, with the command that rewrites it."""
    named = args.get('object')
    which = f', object: {named}' if isinstance(named, str) and named != 'held' else ''
    return (f'{{object: {named}, by / at}} is {{hand: holding, by / at{which}}} since cell schema 5 '
            f'(`{MIGRATE} {cell}` rewrites it)')


def _unknown(value) -> bool:
    """A value that has a placeholder somewhere: it is only known when the step starts."""
    if isinstance(value, str):
        return bool(placeholders.names(value))
    if isinstance(value, Mapping):
        return any(_unknown(v) for v in value.values())
    if isinstance(value, list):
        return any(_unknown(v) for v in value)
    return False


def resolve_form(args: Mapping, store: ConfigStore, robot: str):
    """``(form, target, problem)`` of a move step: ``form`` one of MOVE_FORMS (navigate, straight, hand, carry, posture,
    joints; None if it cannot be told yet: a placeholder, no form key), ``target`` the resolved target of a ``to`` form
    (else None), ``problem`` why the target cannot be resolved (a one-line text, else None)."""
    if 'to' in args:
        reference = args['to']
        if not isinstance(reference, str) or _unknown(reference):
            return None, None, None
        try:
            target = store.target(reference, robot)
        except (KeyError, ConfigError) as error:
            return None, None, str(error.args[0]) if isinstance(error, KeyError) and error.args else str(error)
        if target.kind != 'base':
            return FORM_OF_TARGET[target.kind], target, None
        mode = (args.get('motion') or {}).get('mode') if isinstance(args.get('motion'), Mapping) else None
        return ('straight' if mode == 'straight' else 'navigate'), target, None
    if 'hand' in args:
        return ('carry' if args['hand'] == HOLDING else 'hand'), None, None
    if 'object' in args:
        return 'carry', None, None
    for key, form in FORM_KEYS:
        if key in args:
            return form, None, None
    return None, None, None


def move_notes(args: Mapping, store: ConfigStore) -> list[tuple[list, str]]:
    """Notes on a move step that do not stop it, as ``(path in the args, message)``: the old object form on a cell of
    schema 4 (it runs; ``taskweave config migrate`` rewrites it). On schema 5 the form is a problem
    (``move_problems``)."""
    if old_object_form(args) and packs.cell_schema(store.root) < OBJECT_FORM_SCHEMA:
        return [(['args', 'object'], old_object_text(args, str(store.root)))]
    return []


def _speed_abs_problem(form: str, options: Mapping) -> str | None:
    motion = options.get('motion')
    if isinstance(motion, Mapping) and motion.get('speed_abs') is not None and form in ('navigate', 'straight'):
        return 'motion.speed_abs does not apply (the base takes motion.speed)'
    return None


def move_problems(args: Mapping, store: ConfigStore, robot: str) -> list[tuple[list, str]]:
    """The problems of a move step that the arguments alone show, as ``(path in the args, message)`` with the path
    starting at ``'args'``: an option that does not apply to the kind of motion (the target's kind for a ``to`` step), a
    ``motion`` key or value it does not take, a hand target in the cell frame (not supported yet). A step whose kind
    is not known yet (placeholder) is checked as far as its keys go."""
    form, target, _ = resolve_form(args, store, robot)
    if form is None:
        return []
    ref = args['to'] if target is not None else None
    problems: list[tuple[list, str]] = []

    def add(path: list, text: str) -> None:
        text = text.removeprefix(f'move {form}: ')
        if ref is not None:
            message = f"{ref!r} moves {MOVES_WORD[target.kind]}; {text}"
        else:
            message = f'{move_label(kind_of(form))}: {text}'
        problems.append((['args', *path], message))

    if target is not None and target.kind in ('hand', 'object') and target.frame == 'cell':
        add([], f'cell-frame {target.kind} targets are not supported yet')
    if old_object_form(args) and packs.cell_schema(store.root) >= OBJECT_FORM_SCHEMA:
        add(['object'], old_object_text(args, str(store.root)))
    options = {key: args[key] for key in OPTIONS if key in args}
    relative = form in RELATIVE_FORMS and target is None
    for option, value in options.items():
        if _unknown(value) and option != 'motion':
            continue
        if option == 'motion' and isinstance(value, Mapping):
            for key, each in value.items():
                if _unknown(each):
                    continue
                for text in move_option_problems(form, {'motion': {key: each}}):
                    add(['motion', key], text)
            speed = _speed_abs_problem(form, options)
            if speed:
                add(['motion', 'speed_abs'], speed)
            continue
        for text in move_option_problems(form, {option: value}, relative, target is not None):
            if option == 'torso' and form in ('navigate', 'straight') and value == 'auto':
                text = 'torso auto needs a hand or object target'
            add([option], text)
    extra = [key for key in RELATIVE_OPTIONS if key in args]
    if form in RELATIVE_FORMS and target is None:
        if 'hand' in args and 'hands' in args and args['hand'] != HOLDING:   # holding: hands picks the hold
            add(['hands'], 'hand and hands are two spellings: give one (hand names the hands of a relative move, '
                           'hands filters the hands of a target)')
        torso = options.get('torso')
        if isinstance(torso, int | float | list) and not isinstance(torso, bool):
            add(['torso'], 'a relative move takes torso auto or keep (a value needs a target or joints)')
    else:
        for key in extra:
            add([key], f'option {key} does not apply')
        if form == 'joints' and target is None and 'only' in options:
            add(['only'], 'option only applies to a joint target (to:)')
    return problems


# ---- kind and resources -----------------------------------------------------------------------------------------
def _moving(value) -> bool:
    """A joints value that moves its group: a number or a list (not keep, auto)."""
    return not isinstance(value, str) and value is not None


def _joint_groups_used(joints: Mapping, only, arm_groups: set[str]) -> set[str]:
    used = set()
    for group, value in joints.items():
        if only is not None and group not in only:
            continue
        if not _moving(value) and value != 'auto':
            continue
        used.add('torso' if group == TORSO else 'hands' if group in arm_groups else 'joints')
    return used


def move_resources(form: str, args: Mapping, target, arm_groups: set[str]) -> list[str]:
    """The resources a move step uses: a base move the base (the torso too when it names a height), a hand move or a
    carry the hands (the torso when it is auto or a value: their default is auto), a posture move the hands, a joints
    move what its groups are (the torso, the arms as hands, the rest as joints)."""
    used: set[str] = set()
    joints = dict(target.joints) if target is not None else {}
    step_torso = args.get('torso')
    if form in ('navigate', 'straight'):
        used.add('base')
        word = step_torso if step_torso is not None else joints.get(TORSO)
        if _moving(word):
            used.add('torso')
    elif form in ('hand', 'carry'):
        used.add('hands')
        word = step_torso if step_torso is not None else joints.get(TORSO)
        if word is None or word == 'auto' or _moving(word):
            used.add('torso')
    elif form == 'posture':
        used.add('hands')
    elif form == 'joints':
        values = joints if target is not None else dict(args.get('joints') or {})
        only = args.get('only') if target is not None else None
        used |= _joint_groups_used(values, only if isinstance(only, list) else None, arm_groups)
        return [r for r in ORDER if r in used] or ['joints']
    if form in ('navigate', 'straight', 'hand', 'carry'):   # a joints part next to the pose: the groups but the torso
        used |= _joint_groups_used({g: v for g, v in joints.items() if g != TORSO}, None, arm_groups)
    return [r for r in ORDER if r in used]


def step_info(skill: str, args: Mapping, schema: Mapping | None, store: ConfigStore, robot: str) -> tuple[str | None,
                                                                                                        list[str]]:
    """``(kind, resources)`` of a step. Only ``move`` has a kind, and resources that depend on the arguments; for other
    skills the resources are the schema's ``x-taskweave-resources`` (empty if none)."""
    declared = list((schema or {}).get(RESOURCES) or [])
    if skill != MOVE_SKILL:
        return None, declared
    try:
        form, target, _ = resolve_form(args, store, robot)
        if form is None:
            return None, declared
        from .config.limits import arm_group_names

        return kind_of(form), move_resources(form, args, target, arm_group_names(store, robot))
    except (KeyError, ConfigError):
        return None, declared
