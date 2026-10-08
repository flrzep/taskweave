"""World state: named facts that plans refer to through placeholders (e.g. ``{fullest_stack}``).

taskweave stores and serves the values; applications decide what the facts are and who updates them
(skills return them in their result data, or an application node sets them).
"""

from __future__ import annotations

import threading
from collections.abc import Mapping
from typing import Any


class WorldState(Mapping):
    """Thread-safe key/value store. Iteration and lookups see a consistent snapshot per call."""

    def __init__(self, initial: Mapping[str, Any] | None = None):
        self._data: dict[str, Any] = dict(initial or {})
        self._lock = threading.Lock()

    def __getitem__(self, key: str) -> Any:
        with self._lock:
            return self._data[key]

    def __iter__(self):
        return iter(self.snapshot())

    def __len__(self) -> int:
        with self._lock:
            return len(self._data)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._data)

    def update(self, values: Mapping[str, Any]) -> None:
        with self._lock:
            self._data.update(values)

    def set(self, key: str, value: Any) -> None:
        self.update({key: value})
