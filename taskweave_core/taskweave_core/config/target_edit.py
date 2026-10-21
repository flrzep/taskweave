"""Writing targets: ``ConfigStore.save_target``, ``delete_target`` and ``rename_target``. The files are edited with
ruamel so comments, key order and the one-line style of hand-written entries stay; every result is checked against the
file's schema (ConfigStore._edit_yaml) and the target against targets.target_problems before anything is written.

Where a target goes: a plain name to ``cell/targets.yaml`` (it exists there, or ``frame: cell``, or a new base target)
or to the robot's ``targets.yaml`` (joints, hand and object targets in the base frame); ``<instance>/<name>`` to the
object type: a stance (moves the base) to ``robots/<robot>/objects/<type>.yaml``, hand and object targets to
``objects/<type>.yaml`` with the robot's own settings (elbow, grip_m) in the robot's file.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from . import target_store as ts
from . import targets as model
from .targets import TargetError

ROUND = 5


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


def _write(store, relative: str, keys: list[str], data: dict | None) -> Path:
    """Set the entry at ``keys`` (the last one is the name) of a target file to ``data``, or remove it (``None``)."""

    def edit(document):
        parent = _walk(document, keys[:-1], create=data is not None)
        name = keys[-1]
        if data is None:
            if parent is None or name not in parent:
                raise ValueError(f'no {"/".join(keys)!r} in {relative}')
            del parent[name]
            parent.ca.items.pop(name, None)
            for depth in range(len(keys) - 1, 0, -1):  # a container left empty goes too (not the top level)
                node = _walk(document, keys[: depth - 1], create=False) if depth > 1 else document
                if node is not None and keys[depth - 1] in node and not node[keys[depth - 1]]:
                    del node[keys[depth - 1]]
            return
        entry = parent.get(name)
        if isinstance(entry, dict):
            _update(entry, data)
        else:
            parent[name] = _entry(data)

    return store._edit_yaml(relative, edit)


def _split(reference: str) -> tuple[str | None, str]:
    instance, separator, name = reference.rpartition(ts.SEPARATOR)
    return (instance or None) if separator else None, name


def _check_name(store, name: str) -> None:
    store._check_name(name)


def _check_data(data: dict, frame: str) -> dict:
    """``data`` without ``frame`` (the file says it), checked as a complete target."""
    if not isinstance(data, dict):
        raise ValueError('a target is a mapping')
    problems = model.target_problems(data, frame)
    if problems:
        raise ValueError(problems[0])
    return {k: v for k, v in data.items() if k != 'frame'}


# ---- save ----------------------------------------------------------------------------------------------------------
def save_target(
    store, reference: str, data: dict, robot: str, *, instance_override: bool = False, rename: str | None = None
) -> list[Path]:
    instance, name = _split(reference)
    if instance is None:
        if instance_override:
            raise ValueError('instance_override is for <instance>/<name> references')
        written = _save_plain(store, name, data, robot)
    else:
        written = _save_on_type(store, instance, name, data, robot, instance_override)
    if rename and rename != name:
        written += rename_target(store, reference, rename, robot)
    return list(dict.fromkeys(written))


def _save_plain(store, name: str, data: dict, robot: str) -> list[Path]:
    _check_name(store, name)
    cell, mine = ts.cell_targets(store), ts.robot_targets(store, robot)
    if name in cell and name in mine:
        raise TargetError(f'target {name!r} is in {ts.CELL_FILE} and {ts.robot_file_label(robot)}: rename one')
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
        raise ValueError("a joints target has no frame: it goes to the robot's targets.yaml")
    entry = _check_data(data, 'cell' if in_cell else 'base')
    relative = ts.CELL_FILE if in_cell else ts.robot_file_label(robot)
    return [_write(store, relative, [name], entry)]


def _save_on_type(store, instance: str, name: str, data: dict, robot: str, override: bool) -> list[Path]:
    _check_name(store, name)
    placed = store.object_instances()
    if instance not in placed:
        raise store.missing_instance(instance, f'cannot save {instance}/{name}: no object {instance!r}')
    type = placed[instance].object
    own, theirs = f'objects/{type}.yaml', f'robots/{robot}/objects/{type}.yaml'
    keys = (lambda: ['instances', instance, name]) if override else (lambda: ['targets', name])
    kinds = ts.type_target_kinds(store, robot, type)
    moves = data.get('moves')
    if moves is None:  # settings only: the robot's elbow, grip_m ... on a target that exists
        if name not in kinds:
            raise TargetError(
                f'settings for an unknown target {name!r} of {type} objects: give moves, or one of '
                f'{", ".join(sorted(kinds)) or "none"}'
            )
        return [_write(store, theirs, keys(), {k: v for k, v in data.items() if k != 'frame'} or None)]
    if name in kinds and kinds[name] != moves:
        raise TargetError(
            f'target {name!r} of {type} objects moves {kinds[name]}: it cannot be saved as moving '
            f'{moves} (rename one of them)'
        )
    entry = _check_data(data, 'object')
    if moves == 'joints':
        raise ValueError("a joints target has no object frame: it goes to the robot's targets.yaml")
    if moves == 'base':
        if override and name in kinds:
            entry = {k: v for k, v in entry.items() if k != 'moves'}
        return [_write(store, theirs, keys(), entry)]
    mine = {k: v for k, v in entry.items() if k in model.ROBOT_SETTINGS}
    common = {k: v for k, v in entry.items() if k not in model.ROBOT_SETTINGS}
    written = [_write(store, own, keys(), common)]
    old = _layer(store, theirs, keys())
    if mine or old:
        written.append(_write(store, theirs, keys(), mine or None) if mine else _remove_settings(store, theirs, keys()))
    return written


def _layer(store, relative: str, keys: list[str]) -> dict | None:
    path = store.root / relative
    if not path.is_file():
        return None
    node: Any = store._read_yaml(path) or {}
    for key in keys:
        node = node.get(key) if isinstance(node, dict) else None
    return node if isinstance(node, dict) else None


def _remove_settings(store, relative: str, keys: list[str]) -> Path:
    """Drop the robot's settings entry when the save no longer has any (an entry that holds more is kept as it is)."""
    old = _layer(store, relative, keys) or {}
    rest = {k: v for k, v in old.items() if k not in model.ROBOT_SETTINGS}
    return _write(store, relative, keys, rest or None)


# ---- delete --------------------------------------------------------------------------------------------------------
def delete_target(store, reference: str, robot: str, *, instance_override: bool = False) -> list[Path]:
    instance, name = _split(reference)
    if instance is None:
        cell, mine = ts.cell_targets(store), ts.robot_targets(store, robot)
        if name in cell and name in mine:
            raise TargetError(f'target {name!r} is in {ts.CELL_FILE} and {ts.robot_file_label(robot)}: rename one')
        if name not in cell and name not in mine:
            raise ts._unknown_plain(store, robot, name, cell, mine)
        return [_write(store, ts.CELL_FILE if name in cell else ts.robot_file_label(robot), [name], None)]
    placed = store.object_instances()
    if instance not in placed:
        raise store.missing_instance(instance, f'cannot delete {instance}/{name}: no object {instance!r}')
    type = placed[instance].object
    own, theirs = f'objects/{type}.yaml', f'robots/{robot}/objects/{type}.yaml'
    keys = ['instances', instance, name] if instance_override else ['targets', name]
    found = [relative for relative in (own, theirs) if _layer(store, relative, keys) is not None]
    if not found:
        raise ts._missing_on_type(store, robot, type, name)
    return [_write(store, relative, keys, None) for relative in found]


# ---- rename --------------------------------------------------------------------------------------------------------
def rename_target(store, reference: str, new_name: str, robot: str) -> list[Path]:
    _check_name(store, new_name)
    instance, name = _split(reference)
    if instance is None:
        cell, mine = ts.cell_targets(store), ts.robot_targets(store, robot)
        if name not in cell and name not in mine:
            raise ts._unknown_plain(store, robot, name, cell, mine)
        if new_name in cell or new_name in mine:
            raise TargetError(f'target {new_name!r} exists already ({ts.CELL_FILE} or {ts.robot_file_label(robot)})')
        relative = ts.CELL_FILE if name in cell else ts.robot_file_label(robot)
        return [_rename_in(store, relative, [], name, new_name)]
    placed = store.object_instances()
    if instance not in placed:
        raise store.missing_instance(instance, f'cannot rename {instance}/{name}: no object {instance!r}')
    type = placed[instance].object
    if name not in ts.type_target_kinds(store, robot, type):
        raise ts._missing_on_type(store, robot, type, name)
    if new_name in ts.type_target_kinds(store, robot, type):
        raise TargetError(f'object {type!r} has a target {new_name!r} already')
    return _rename_on_type(store, type, name, new_name, robot)


def _rename_on_type(store, type: str, name: str, new_name: str, robot: str) -> list[Path]:
    written = []
    own = f'objects/{type}.yaml'
    shared = name in ts.object_file(store, type).get('targets', {})
    for relative, owner in ((own, None), *((f'robots/{r}/objects/{type}.yaml', r) for r in store.robot_names())):
        path = store.root / relative
        if not path.is_file():
            continue
        data = store._read_yaml(path) or {}
        for keys in (['targets'], *(['instances', i] for i in data.get('instances', {}))):
            table = _walk_plain(data, keys)
            entry = table.get(name) if table else None
            mine = owner in (None, robot) or (shared and 'moves' not in (entry or {}))
            if entry is not None and mine:
                written.append(_rename_in(store, relative, keys, name, new_name))
        if owner == robot and data.get('handled_from') == name:
            written.append(_set_handled_from(store, relative, new_name))
    return written


def _walk_plain(data: dict, keys: list[str]) -> dict | None:
    node: Any = data
    for key in keys:
        node = node.get(key) if isinstance(node, dict) else None
    return node if isinstance(node, dict) else None


def _rename_in(store, relative: str, keys: list[str], name: str, new_name: str) -> Path:
    def edit(document):
        node = _walk(document, keys, create=False)
        if node is None or name not in node:
            raise ValueError(f'no {name!r} in {relative}')
        store._rename_key(node, name, new_name, new_name, relative)

    return store._edit_yaml(relative, edit)


def _set_handled_from(store, relative: str, name: str) -> Path:
    def edit(document):
        document['handled_from'] = name

    return store._edit_yaml(relative, edit)
