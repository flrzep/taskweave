"""Launch argument helpers shared by the launch files."""

from __future__ import annotations

TRUE = ('1', 'true', 'yes', 'on')


def flag(value: str) -> bool:
    return value.strip().lower() in TRUE
