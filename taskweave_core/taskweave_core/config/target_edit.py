"""Writing targets: ``ConfigStore.save_target``, ``delete_target`` and ``rename_target``. The files are edited with
ruamel so comments, key order and the one-line style of hand-written entries stay; every result is checked against the
file's schema (ConfigStore._edit_yaml) and the target against targets.target_problems before anything is written.

Where a target goes: a plain name to ``cell/targets.yaml`` (it exists there, or ``frame: cell``, or a new base target)
or to the robot's ``targets.yaml`` (joints, hand and object targets in the base frame); ``<instance>/<name>`` to the
object type: a stance (moves the base) to ``robots/<robot>/objects/<type>.yaml``, hand and object targets to
``objects/<type>.yaml`` with the robot's own settings (elbow, grip_m) in the robot's file. An existing hand or object
target is written back to the files that hold it: a key stays in the layer it came from, a new key goes to the
robot's file when it is a robot setting and to the type's file otherwise.

``type=<object type>`` (keyword of save_target, delete_target and rename_target; ``reference`` is then the bare name)
addresses the type's own target with no placed object, e.g. to teach the first target of a type nothing is placed
for. It is the type layers only (no instance override). The read is ``ConfigStore.type_target(type, name, robot)``.

Refusals of a write are ``TargetWriteError`` (a ValueError); a target that is not there is a ``TargetError`` (a
KeyError), as in the lookups.
"""

from __future__ import annotations

import datetime
from pathlib import Path
from typing import Any

from . import target_store as ts
from . import targets as model
from .targets import TargetError, TargetWriteError

ROUND = 4


def _clean(value: Any) -> Any:
    """Plain data as ruamel nodes: numbers rounded, lists and small mappings on one line, ``path`` one waypoint per
    line."""
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    if isinstance(value, bool) or value is None or isinstance(value, str | int):
        return value
    if isinstance(value, float):
        return round(value, ROUND) + 0.0
    if isinstance(value, list | tuple):
        seq = CommentedSeq(_clean(v) for v in value)
        if all(not isinstance(v, dict | list) for v in value):
            seq.fa.set_flow_style()
        return seq
    node = CommentedMap((key, _clean(each)) for key, each in value.items())
    node.fa.set_flow_style()
    return node


def _entry(data: dict) -> Any:
    """One target (or settings) entry: one line when it is short and flat, else one key per line."""
    node = _clean(data)
    flat = all(not isinstance(v, dict | list) or k == 'joints' for k, v in data.items())
    if flat and 'path' not in data and len(data) <= 6:
        node.fa.set_flow_style()
    else:
        node.fa.set_block_style()
    return node


def _update(entry, data: dict) -> None:
    """Set ``entry`` (an existing mapping) to ``data`` key by key, so unchanged keys keep their comments."""
    from ruamel.yaml.comments import CommentedMap

    for key in [k for k in entry if k not in data]:
        entry.pop(key)
        entry.ca.items.pop(key, None)
    for key, value in data.items():
        old = entry.get(key)
        new = _clean(value)
        if old is not None and _plain(old) == _plain(new):
            continue
        if isinstance(new, CommentedMap) and isinstance(old, CommentedMap) and key in model.MERGE_PER_KEY:
            _update(old, value)
            continue
        entry[key] = new
        if key in entry.ca.items and not isinstance(new, CommentedMap | list):
            entry.ca.items.pop(key, None)  # a comment about the old value no longer describes the new one


def _plain(node):
    import json

    return json.loads(json.dumps(node))


def _walk(document, keys: list[str], create: bool):
    """The mapping at ``keys`` below ``document`` (created when ``create``), else None."""
    from ruamel.yaml.comments import CommentedMap

    node = document
    for key in keys:
        child = node.get(key)
        if not isinstance(child, dict):
            if not create:
                return None
            child = node[key] = CommentedMap()
            child.fa.set_block_style()
        node = child
    return node


def _write(store, relative: str, keys: list[str], data: dict | None, note: str | None = None, auto: bool = True) -> Path:
    """Set the entry at ``keys`` (the last one is the name) of a target file to ``data``, or remove it (``None``).
    ``note`` is the entry's end-of-line comment (it replaces one); with ``auto`` a new entry gets ``saved <date>``."""

    def edit(document):
        parent = _walk(document, keys[:-1], create=data is not None)
        name = keys[-1]
        if data is None:
            if parent is None or name not in parent:
                raise TargetWriteError(f'no {"/".join(keys)!r} in {relative}')
            del parent[name]
            parent.ca.items.pop(name, None)
            for depth in range(len(keys) - 1, 0, -1):  # a container left empty goes too (not the top level)
                node = _walk(document, keys[: depth - 1], create=False) if depth > 1 else document
                if node is not None and keys[depth - 1] in node and not node[keys[depth - 1]]:
                    del node[keys[depth - 1]]
            return
        entry = parent.get(name)
        new = not isinstance(entry, dict)
        if new:
            parent[name] = entry = _entry(data)
        else:
            _update(entry, data)
        text = note or (f'saved {datetime.date.today().isoformat()}' if new and auto else None)
        if text and entry.fa.flow_style() is True:  # a block entry has no end-of-line comment
            parent.ca.items.pop(name, None)
            parent.yaml_add_eol_comment(text, name, column=2)

    return store._edit_yaml(relative, edit)


def _split(reference: str) -> tuple[str | None, str]:
    instance, separator, name = reference.rpartition(ts.SEPARATOR)
    return (instance or None) if separator else None, name


def _check_name(store, name: str) -> None:
    try:
        store._check_name(name)
    except TargetWriteError:
        raise
    except ValueError as error:
        raise TargetWriteError(str(error)) from error


def _check_data(data: dict, frame: str) -> dict:
    """``data`` without ``frame`` (the file says it), checked as a complete target."""
    if not isinstance(data, dict):
        raise TargetWriteError('a target is a mapping')
    problems = model.target_problems(data, frame)
    if problems:
        raise TargetWriteError(problems[0])
    return {k: v for k, v in data.items() if k != 'frame'}


# ---- addressing: <name>, <instance>/<name> or type= ---------------------------------------------------------------
def _check_type(store, robot: str | None, type: str) -> None:
    known = ts.type_names(store, robot)
    if type not in known:
        raise TargetError(f'no object type {type!r} (types: {", ".join(known) or "none"})')


def _on_type(store, reference: str, robot: str, type: str | None, verb: str, override: bool):
    """``(type, instance, name)`` of an address with a type: ``type=`` (reference is the bare name) or the type of the
    placed object in ``<instance>/<name>``; ``(None, None, name)`` for a plain name."""
    instance, name = _split(reference)
    if type is not None:
        if instance is not None:
            raise TargetWriteError(f'{reference!r} with type={type!r}: give the bare target name, not <instance>/<name>')
        if override:
            raise TargetWriteError('instance_override is for <instance>/<name> references')
        _check_type(store, robot, type)
        return type, None, name
    if instance is None:
        if override:
            raise TargetWriteError('instance_override is for <instance>/<name> references')
        return None, None, name
    placed = store.object_instances()
    if instance not in placed:
        raise store.missing_instance(instance, f'cannot {verb} {instance}/{name}: no object {instance!r}')
    return placed[instance].object, instance, name


def _layers_of(type: str, instance: str | None, override: bool, name: str, robot: str):
    own, theirs = f'objects/{type}.yaml', f'robots/{robot}/objects/{type}.yaml'
    keys = ['instances', instance, name] if override else ['targets', name]
    return own, theirs, keys


# ---- save ----------------------------------------------------------------------------------------------------------
def save_target(
    store,
    reference: str,
    data: dict,
    robot: str,
    *,
    instance_override: bool = False,
    rename: str | None = None,
    type: str | None = None,
    note: str | None = None,
) -> list[Path]:
    """Create or update a target (see the module text). ``type``: save the type's own target ``reference`` (no placed
    object needed). ``note``: the end-of-line comment of the entry (replaces one there); without it a new entry gets
    ``saved <date>`` and an existing one keeps its comment."""
    on_type, instance, name = _on_type(store, reference, robot, type, 'save', instance_override)
    if on_type is None:
        written = _save_plain(store, name, data, robot, note)
    else:
        written = _save_on_type(store, on_type, instance, name, data, robot, instance_override, note)
    if rename and rename != name:
        written += rename_target(store, reference, rename, robot, type=type)
    return list(dict.fromkeys(written))


def _save_plain(store, name: str, data: dict, robot: str, note: str | None) -> list[Path]:
    _check_name(store, name)
    cell, mine = ts.cell_targets(store), ts.robot_targets(store, robot)
    if name in cell and name in mine:
        raise TargetWriteError(f'target {name!r} is in {ts.CELL_FILE} and {ts.robot_file_label(robot)}: rename one')
    moves = data.get('moves')
    if name in cell:
        in_cell = True
    elif name in mine:
        in_cell = False
    elif data.get('frame') in ('cell', 'base'):
        in_cell = data['frame'] == 'cell'
    else:
        in_cell = moves == 'base'
    if moves == 'joints' and in_cell:
        raise TargetWriteError("a joints target has no frame: it goes to the robot's targets.yaml")
    entry = _check_data(data, 'cell' if in_cell else 'base')
    relative = ts.CELL_FILE if in_cell else ts.robot_file_label(robot)
    return [_write(store, relative, [name], entry, note)]


def _save_on_type(
    store, type: str, instance: str | None, name: str, data: dict, robot: str, override: bool, note: str | None
) -> list[Path]:
    _check_name(store, name)
    own, theirs, keys = _layers_of(type, instance, override, name, robot)
    kinds = ts.type_target_kinds(store, robot, type, instance)
    moves = data.get('moves')
    if moves is None:  # settings only: the robot's elbow, grip_m ... on a target that exists
        if name not in kinds:
            raise TargetWriteError(
                f'settings for an unknown target {name!r} of {type} objects: give moves, or one of '
                f'{", ".join(sorted(kinds)) or "none"}'
            )
        return [_write(store, theirs, keys, {k: v for k, v in data.items() if k != 'frame'} or None, note)]
    if name in kinds and kinds[name] != moves:
        raise TargetWriteError(
            f'target {name!r} of {type} objects moves {kinds[name]}: it cannot be saved as moving '
            f'{moves} (rename one of them)'
        )
    entry = _check_data(data, 'object')
    if moves == 'joints':
        raise TargetWriteError("a joints target has no object frame: it goes to the robot's targets.yaml")
    if moves == 'base':
        if override and name in kinds:
            entry = {k: v for k, v in entry.items() if k != 'moves'}
        return [_write(store, theirs, keys, entry, note)]
    old_own, old_theirs = _layer(store, own, keys), _layer(store, theirs, keys)
    common, mine = split_layers(entry, old_own or {}, old_theirs or {})
    written = []
    for relative, part, old in ((own, common, old_own), (theirs, mine, old_theirs)):
        if part:
            first = relative == own
            written.append(_write(store, relative, keys, part, note if first else None, auto=first))
        elif old is not None:  # nothing of the target is left in this layer
            written.append(_write(store, relative, keys, None))
    return written


def split_layers(entry: dict, own: dict, theirs: dict) -> tuple[dict, dict]:
    """Which layer each key of a complete hand or object target goes to: a key that is in the object type's entry
    (``own``) or the robot's (``theirs``) stays there (when both have it the robot's value is the one set, the type's
    stays as it is), a new key goes to the robot's
    when it is a robot setting (elbow, grip_m) and to the type's otherwise. Keys that merge per part (grip_m, alone,
    joints) are split by part the same way. Returns ``(own part, robot part)``."""
    common: dict = {}
    mine: dict = {}

    def default(key: str, in_own: bool, in_theirs: bool) -> str:
        if in_theirs and not in_own:
            return 'theirs'
        return 'own' if in_own else ('theirs' if key in model.ROBOT_SETTINGS else 'own')

    def part(layer: dict, key: str) -> dict:
        return layer.get(key) if isinstance(layer.get(key), dict) else {}

    for key, value in entry.items():
        in_own, in_theirs = key in own, key in theirs
        if key in model.MERGE_PER_KEY and isinstance(value, dict):
            parts: dict[str, dict] = {'own': {}, 'theirs': {}}
            for sub, each in value.items():
                has_own, has_theirs = sub in part(own, key), sub in part(theirs, key)
                if has_own:  # the robot's value wins when both have it: the type's stays as it is
                    parts['own'][sub] = part(own, key)[sub] if has_theirs else each
                if has_theirs:
                    parts['theirs'][sub] = each
                if not has_own and not has_theirs:
                    parts[default(key, in_own, in_theirs)][sub] = each
            if parts['own']:
                common[key] = parts['own']
            if parts['theirs']:
                mine[key] = parts['theirs']
            continue
        if in_own and in_theirs:
            common[key], mine[key] = own[key], value
        else:
            layer = 'own' if in_own else 'theirs' if in_theirs else default(key, in_own, in_theirs)
            (common if layer == 'own' else mine)[key] = value
    return common, mine


def _layer(store, relative: str, keys: list[str]) -> dict | None:
    return _walk_plain(_read_plain(store, relative), keys)


def _read_plain(store, relative: str) -> dict:
    path = store.root / relative
    return (store._read_yaml(path) or {}) if path.is_file() else {}


def _walk_plain(data: dict, keys: list[str]) -> dict | None:
    node: Any = data
    for key in keys:
        node = node.get(key) if isinstance(node, dict) else None
    return node if isinstance(node, dict) else None


# ---- delete --------------------------------------------------------------------------------------------------------
def delete_target(
    store,
    reference: str,
    robot: str,
    *,
    instance_override: bool = False,
    type: str | None = None,
    cascade: bool = False,
) -> list[Path]:
    """Remove a target from the files that define it. Deleting a target of an object type that instance overrides
    or other robots' settings still refer to is refused (``TargetWriteError`` listing them) unless ``cascade``: they
    go too."""
    on_type, instance, name = _on_type(store, reference, robot, type, 'delete', instance_override)
    if on_type is None:
        cell, mine = ts.cell_targets(store), ts.robot_targets(store, robot)
        if name in cell and name in mine:
            raise TargetWriteError(f'target {name!r} is in {ts.CELL_FILE} and {ts.robot_file_label(robot)}: rename one')
        if name not in cell and name not in mine:
            raise ts._unknown_plain(store, robot, name, cell, mine)
        return [_write(store, ts.CELL_FILE if name in cell else ts.robot_file_label(robot), [name], None)]
    own, theirs, keys = _layers_of(on_type, instance, instance_override, name, robot)
    found = [relative for relative in (own, theirs) if _layer(store, relative, keys) is not None]
    if not found:
        raise ts._missing_on_type(store, robot, on_type, name)
    orphans = [] if instance_override else _orphans(store, on_type, name, robot, found)
    if orphans and not cascade:
        listing = ', '.join(f'{relative} {".".join(path)}' for relative, path in orphans)
        raise TargetWriteError(
            f'target {name!r} of {on_type} objects is still used by settings that would be left without a target: '
            f'{listing}; delete those first, or delete with cascade=True'
        )
    written = [_write(store, relative, keys, None) for relative in found]
    written += [_write(store, relative, path, None) for relative, path in orphans]
    return list(dict.fromkeys(written))


def _orphans(store, type: str, name: str, robot: str, found: list[str]) -> list[tuple[str, list[str]]]:
    """The entries named ``name`` that would have no ``moves`` left once the type target is removed from ``found``:
    instance overrides with only settings (in the type's file and every robot's), and other robots' settings-only
    entries on the type. ``[(file label, keys)]``."""
    own, mine = f'objects/{type}.yaml', f'robots/{robot}/objects/{type}.yaml'
    files = [own, *(f'robots/{r}/objects/{type}.yaml' for r in store.robot_names())]
    own_instances = _read_plain(store, own).get('instances') or {}
    my_instances = _read_plain(store, mine).get('instances') or {}

    def defines(instance: str) -> bool:
        return any('moves' in ((table.get(instance) or {}).get(name) or {}) for table in (own_instances, my_instances))

    result = []
    for relative in dict.fromkeys(files):
        data = _read_plain(store, relative)
        entry = (data.get('targets') or {}).get(name)
        if relative not in found and relative != mine and isinstance(entry, dict) and 'moves' not in entry:
            result.append((relative, ['targets', name]))  # another robot's settings for the deleted target
        for instance, table in (data.get('instances') or {}).items():
            if isinstance((table or {}).get(name), dict) and not defines(instance):
                result.append((relative, ['instances', instance, name]))
    return result


# ---- rename --------------------------------------------------------------------------------------------------------
def rename_target(store, reference: str, new_name: str, robot: str, *, type: str | None = None) -> list[Path]:
    _check_name(store, new_name)
    on_type, instance, name = _on_type(store, reference, robot, type, 'rename', False)
    if on_type is None:
        cell, mine = ts.cell_targets(store), ts.robot_targets(store, robot)
        if name not in cell and name not in mine:
            raise ts._unknown_plain(store, robot, name, cell, mine)
        if new_name in cell or new_name in mine:
            raise TargetWriteError(
                f'target {new_name!r} exists already ({ts.CELL_FILE} or {ts.robot_file_label(robot)})'
            )
        relative = ts.CELL_FILE if name in cell else ts.robot_file_label(robot)
        return [_rename_in(store, relative, [], name, new_name)]
    if name not in ts.type_target_kinds(store, robot, on_type, instance):
        raise ts._missing_on_type(store, robot, on_type, name)
    if new_name in ts.type_target_kinds(store, robot, on_type, instance):
        raise TargetWriteError(f'object {on_type!r} has a target {new_name!r} already')
    return _rename_on_type(store, on_type, name, new_name, robot, instance)


def _rename_on_type(store, type: str, name: str, new_name: str, robot: str, instance: str | None) -> list[Path]:
    """Rename in the files of the type. A type-level name is renamed in every table that holds it (the type's, the
    instance overrides, other robots' settings); a name that exists only in instance overrides means a different
    target in each instance, so only the addressed ``instance``'s tables are renamed."""
    written = []
    own = f'objects/{type}.yaml'
    files = ((own, None), *((f'robots/{r}/objects/{type}.yaml', r) for r in store.robot_names()))
    type_level = any(name in (_read_plain(store, relative).get('targets') or {}) for relative, _ in files)
    if not type_level and instance is None:
        raise ts._missing_on_type(store, robot, type, name)
    shared = name in ts.object_file(store, type).get('targets', {})
    for relative, owner in files:
        data = _read_plain(store, relative)
        if not data:
            continue
        tables = [['targets']] if type_level else []
        tables += [['instances', i] for i in (data.get('instances') or {}) if type_level or i == instance]
        for keys in tables:
            table = _walk_plain(data, keys)
            entry = table.get(name) if table else None
            mine = owner in (None, robot) or (shared and 'moves' not in (entry or {}))
            if entry is None or not mine:
                continue
            if new_name in table:
                raise TargetWriteError(
                    f'{".".join(keys)} in {relative} has a {new_name!r} already: rename or delete it first'
                )
            written.append(_rename_in(store, relative, keys, name, new_name))
        if owner == robot and data.get('handled_from') == name:
            written.append(_set_handled_from(store, relative, new_name))
    return written


def _rename_in(store, relative: str, keys: list[str], name: str, new_name: str) -> Path:
    def edit(document):
        node = _walk(document, keys, create=False)
        if node is None or name not in node:
            raise TargetWriteError(f'no {name!r} in {relative}')
        store._rename_key(node, name, new_name, new_name, relative)

    return store._edit_yaml(relative, edit)


def _set_handled_from(store, relative: str, name: str) -> Path:
    def edit(document):
        document['handled_from'] = name

    return store._edit_yaml(relative, edit)
