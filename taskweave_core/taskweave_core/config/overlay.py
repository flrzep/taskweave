"""Per-key cell overrides of robot pack files: the rules on plain data (stdlib only, so tools without taskweave, e.g.
frame_sim's cellconfig, can use or copy them).

A cell pack file robots/<robot>/<file> of a robot pack file (MERGED_FILES) is one of two kinds:

    whole file   no MARKER: the cell file replaces the pack's file (a copy made before per-key overrides)
    keyed        top-level ``~override: keys``: only what the cell changes, laid over the pack's file:
                 mappings merge key by key, lists and scalars replace, ``<key>: ~delete`` removes the pack's key.
                 poses.yaml and targets.yaml merge one level deep: a pose or target is replaced whole (DEPTH).

Saving (ConfigStore._edit_yaml) and ``taskweave config slim`` write keyed overrides: delta(pack, wanted). Only an
explicit edit writes ~delete: slim reads a pack key a whole-file copy lacks as never had (the copy predates it).
"""

from __future__ import annotations

import copy

MARKER = '~override'    # top-level key of a keyed override
KEYS = 'keys'           # its value
DELETE = '~delete'      # value in a keyed override: remove this key of the pack's file
DEPTH = {'poses.yaml': 1, 'targets.yaml': 1}   # levels merged key by key (default: all); below, entries replaced whole
MERGED_FILES = ('profile.yaml', 'embodiment.yaml', 'navigation.yaml', 'sim.yaml', 'poses.yaml', 'targets.yaml',
                'pose_targets.yaml')


def is_keyed(data) -> bool:
    return isinstance(data, dict) and data.get(MARKER) == KEYS


def same(a, b) -> bool:
    """Equal values of equal types (1 is not 1.0, True is not 1), recursively."""
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b, strict=True))
    return type(a) is type(b) and a == b


def strip(value):
    """A value without DELETE entries (they mean nothing where the pack has no such key)."""
    if isinstance(value, dict):
        return {k: strip(v) for k, v in value.items() if v != DELETE}
    return copy.deepcopy(value)


def merge(base, over, depth: int | None = None):
    """``over`` laid on ``base``: mappings key by key for ``depth`` levels (None: all), anything else replaces."""
    if depth == 0 or not isinstance(base, dict) or not isinstance(over, dict):
        return strip(over)
    result = copy.deepcopy(base)
    for key, value in over.items():
        if value == DELETE:
            result.pop(key, None)
        elif key in result:
            result[key] = merge(result[key], value, None if depth is None else depth - 1)
        else:
            result[key] = strip(value)
    return result


def overlay(base: dict | None, cell, file: str):
    """What the cell's copy of a robot pack file means: keyed, laid over ``base`` (the pack's data); else whole."""
    if not is_keyed(cell):
        return cell
    return merge(base or {}, {k: v for k, v in cell.items() if k != MARKER}, DEPTH.get(file))


def delta(base: dict, data: dict, depth: int | None = None, deleted: bool = True) -> dict:
    """The keyed override (without MARKER) that turns ``base`` into ``data``: merge(base, delta) == data.
    ``deleted`` False: base keys ``data`` lacks are left out (not overridden) instead of becoming DELETE. That is the
    reading of a whole-file copy made before the pack had those keys (slim): missing means never had it. DELETE
    is written only by explicit edits."""
    result = {}
    for key, value in data.items():
        if key not in base:
            result[key] = value
        elif depth != 1 and isinstance(value, dict) and isinstance(base[key], dict):
            inner = delta(base[key], value, None if depth is None else depth - 1, deleted)
            if inner:
                result[key] = inner
        elif not same(value, base[key]):
            result[key] = value
    if deleted:
        result.update({key: DELETE for key in base if key not in data})
    return result


def key_paths(over: dict, depth: int | None = None, deleted: bool = False, prefix: str = '') -> list[str]:
    """Dotted paths of the leaves of a keyed override (a replaced list or, below ``depth``, a replaced mapping is one
    leaf); ``deleted``: only the DELETE ones."""
    paths = []
    for key, value in over.items():
        if depth != 1 and isinstance(value, dict) and value:
            paths += key_paths(value, None if depth is None else depth - 1, deleted, f'{prefix}{key}.')
        elif not deleted or value == DELETE:
            paths.append(f'{prefix}{key}')
    return paths


def same_paths(base: dict, data: dict, depth: int | None = None, prefix: str = '') -> list[str]:
    """Dotted paths of the largest parts of ``data`` equal to ``base`` (what a keyed override leaves out)."""
    paths = []
    for key, value in data.items():
        if key not in base:
            continue
        if same(value, base[key]):
            paths.append(f'{prefix}{key}')
        elif depth != 1 and isinstance(value, dict) and isinstance(base[key], dict):
            paths += same_paths(base[key], value, None if depth is None else depth - 1, f'{prefix}{key}.')
    return paths
