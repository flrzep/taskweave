"""``{name}`` placeholders in step arguments, filled from the world state when a step starts.

A string that is exactly one placeholder takes the world-state value with its type (``"{lane}"`` -> ``2``);
placeholders inside longer strings are formatted in (``"stance:{stack}"`` -> ``"stance:lane_2_stack_a"``).
"""

from __future__ import annotations

import re
from collections.abc import Mapping
from typing import Any

PATTERN = re.compile(r'\{([A-Za-z_][A-Za-z0-9_.]*)\}')


class PlaceholderError(KeyError):
    """A placeholder has no value in the world state."""


def names(value: Any) -> set[str]:
    """All placeholder names used anywhere in a (nested) value."""
    if isinstance(value, str):
        return set(PATTERN.findall(value))
    if isinstance(value, Mapping):
        return set().union(*(names(v) for v in value.values())) if value else set()
    if isinstance(value, (list, tuple)):
        return set().union(*(names(v) for v in value)) if value else set()
    return set()


def resolve(value: Any, world: Mapping[str, Any]) -> Any:
    """Return a copy of ``value`` with every placeholder replaced from ``world``."""
    if isinstance(value, str):
        whole = PATTERN.fullmatch(value)
        if whole:
            return _lookup(whole.group(1), world)
        return PATTERN.sub(lambda match: str(_lookup(match.group(1), world)), value)
    if isinstance(value, Mapping):
        return {key: resolve(item, world) for key, item in value.items()}
    if isinstance(value, list):
        return [resolve(item, world) for item in value]
    return value


def _lookup(name: str, world: Mapping[str, Any]) -> Any:
    if name not in world:
        raise PlaceholderError(f'world state has no value for placeholder {{{name}}}')
    return world[name]
