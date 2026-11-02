"""Cell pack migration 4 -> 5: the move commands from the operator's view (five kinds: navigate, straight, hand,
posture, joints; the held-object moves are hand moves).

    missions/*.yaml, operator.yaml   a move step {object: <held | instance>, by | at, ...} -> {hand: holding, by | at,
                                     ..., object: <instance>}: ``object`` stays only when it names an instance (which
                                     held object), ``object: held`` goes; the key order and the comments stay
    skills/move/base.yaml            -> skills/move/navigate.yaml (the same bytes)
    skills/move/object.yaml          -> skills/move/carry.yaml (the same bytes)
    robots/<robot>/skill_overrides.yaml  keys move.base -> move.navigate, move.object -> move.carry
    tests/*.reference.yaml           smoke references: limits.tolerance_by_skill move.base -> move.navigate,
                                     move.object -> move.hand; each step's data.kind base -> navigate, object -> hand
                                     and data.module move.base -> move.navigate, move.object -> move.carry

Nothing is written until every file is built; a file that cannot be written stops the run before anything changes
(RuntimeError). A new part file that exists already is kept and flagged (merge by hand). A second run finds nothing to
do. Schema 4 stays readable: the old step form runs (``taskweave check`` notes it) and the old part files are read while
the new ones are missing (ConfigStore.skill_config).
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

from .migrate_skills import Plan, _dump, _flag, _read, _show
from .migrate_targets import _rename_key, apply_plan, writable_problems
from .store import PARTS_BEFORE, skill_file
from .targets import HOLDING, OLD_KINDS

MOVE = 'move'
HELD = 'held'
RELATIVE = ('by', 'at')
PART_RENAMES = {before: part for part, before in PARTS_BEFORE.items()}      # move.base -> move.navigate
TOLERANCE_RENAMES = {f'{MOVE}.{old}': f'{MOVE}.{new}' for old, new in OLD_KINDS.items()}   # smoke limits: by kind


def object_form_step(args) -> bool:
    """A move step's args in the held-object form of schema 4: ``object`` with by / at, no ``to``, no ``hand``."""
    return isinstance(args, Mapping) and 'object' in args and 'to' not in args and 'hand' not in args \
        and any(key in args for key in RELATIVE)


def convert_args(args) -> None:
    """``{object: x, by...}`` -> ``{hand: holding, by..., object: x}`` in place (``object: held`` dropped), comments
    and the flow style kept."""
    from ruamel.yaml.scalarstring import PlainScalarString

    named = args['object']
    _rename_key(args, 'object', 'hand')
    args['hand'] = PlainScalarString(HOLDING)     # not the quotes of the value it replaces
    if named != HELD:
        args['object'] = named


def _steps(items, where, say) -> bool:
    changed = False
    for number, step in enumerate(items, start=1):
        if not isinstance(step, Mapping) or step.get('skill') != MOVE or not object_form_step(step.get('args')):
            continue
        before = _show(step['args'])
        convert_args(step['args'])
        say(f'step    {where(step, number)}: move {before} -> move {_show(step["args"])}')
        changed = True
    return changed


def migrate_missions(plan: Plan, say) -> None:
    for path in sorted((plan.cell / 'missions').glob('*.yaml')):
        mission, handler = _read(path)
        if isinstance(mission, Mapping) and isinstance(mission.get('steps'), list) and _steps(
                mission['steps'], lambda s, n, p=path: f'{p.stem} {s.get("id", f"s{n}")}', say):
            plan.writes[path] = _dump(mission, handler)


def migrate_operator(plan: Plan, say) -> None:
    path = plan.cell / 'operator.yaml'
    if not path.is_file():
        return
    document, handler = _read(path)
    actions = ((document or {}).get('joy') or {}).get('actions') if isinstance(document, Mapping) else None
    if isinstance(actions, list) and _steps(actions, lambda a, n: f'operator button {a.get("button", n)}', say):
        plan.writes[path] = _dump(document, handler)


def migrate_part_files(plan: Plan, say) -> None:
    for before, part in PART_RENAMES.items():
        old, new = plan.cell / skill_file(before), plan.cell / skill_file(part)
        if not old.is_file():
            continue
        if new.is_file():
            _flag(plan, say, f'{plan.rel(old)} kept: {plan.rel(new)} exists already; merge them by hand, then delete '
                             f'{old.name}')
            continue
        plan.writes[new] = old.read_bytes().decode('utf-8')    # the same bytes
        plan.removes.append(old)
        say(f'move    {plan.rel(old)} -> {plan.rel(new)}')


def migrate_skill_overrides(plan: Plan, say) -> None:
    for path in sorted((plan.cell / 'robots').glob('*/skill_overrides.yaml')):
        document, handler = _read(path)
        if not isinstance(document, Mapping):
            continue
        keys = [key for key in document if key in PART_RENAMES]
        for key in keys:
            if PART_RENAMES[key] in document:
                _flag(plan, say, f'{plan.rel(path)}: {key} kept, {PART_RENAMES[key]} exists already; merge them')
                continue
            _rename_key(document, key, PART_RENAMES[key])
            say(f'rename  {plan.rel(path)} {key} -> {PART_RENAMES[key]}')
            plan.writes[path] = _dump(document, handler)


def migrate_references(plan: Plan, say) -> None:
    """Smoke references (tests/*.reference.yaml): the kind ids and limits of schema 5."""
    modules = {f'{MOVE}.{old}': f'{MOVE}.{new}' for old, new in
               ((b.partition('.')[2], p.partition('.')[2]) for b, p in PART_RENAMES.items())}
    for path in sorted((plan.cell / 'tests').glob('*.reference.yaml')):
        document, handler = _read(path)
        if not isinstance(document, Mapping):
            continue
        changes = 0
        limits = (document.get('limits') or {}).get('tolerance_by_skill') if isinstance(
            document.get('limits'), Mapping) else None
        if isinstance(limits, Mapping):
            for key in [k for k in limits if k in TOLERANCE_RENAMES]:
                if TOLERANCE_RENAMES[key] in limits:
                    _flag(plan, say, f'{plan.rel(path)}: limits {key} kept, {TOLERANCE_RENAMES[key]} exists already')
                    continue
                _rename_key(limits, key, TOLERANCE_RENAMES[key])
                changes += 1
        for step in document.get('steps') or []:
            data = step.get('data') if isinstance(step, Mapping) else None
            if not isinstance(data, Mapping):
                continue
            if data.get('kind') in OLD_KINDS:
                data['kind'] = OLD_KINDS[data['kind']]
                changes += 1
            if data.get('module') in modules:
                data['module'] = modules[data['module']]
                changes += 1
        if changes:
            say(f'rewrite {plan.rel(path)}  ({changes} kind id(s) of schema 4)')
            plan.writes[path] = _dump(document, handler)


def migrate_kinds(cell: Path, robot_packs, dry_run: bool, say) -> list[str]:
    """Migration 4 -> 5 (MIGRATIONS signature). Raises RuntimeError if a file cannot be written (nothing changed)."""
    plan = Plan(cell)
    migrate_missions(plan, say)
    migrate_operator(plan, say)
    migrate_part_files(plan, say)
    migrate_skill_overrides(plan, say)
    migrate_references(plan, say)
    problems = writable_problems(plan)
    if problems and not dry_run:
        raise RuntimeError('; '.join(problems) + ': nothing was changed')
    for problem in problems:
        say(f'note    {problem} (a real run stops here)')
    if not dry_run:
        apply_plan(plan)
    do = 'would ' if dry_run else ''
    say(f'{"would write" if dry_run else "wrote"} {len(plan.writes)} file(s), {do}remove {len(plan.removes)}; '
        f'{len(plan.flags)} flag(s)')
    for flag in plan.flags:
        say(f'FLAG    {flag}')
    return []


__all__ = ['migrate_kinds', 'convert_args', 'object_form_step']
