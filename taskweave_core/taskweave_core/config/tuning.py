"""The robot profile's motion_tuning (how the arm motion code steps) with the schema's defaults filled in."""

from __future__ import annotations

from .schemas import MOTION_TUNING_DEFAULTS


def motion_tuning(profile=None) -> dict:
    """{group: {key: value}} for a robot profile (RobotProfile, its data dict, or None for the defaults): the profile's
    motion_tuning over schemas.MOTION_TUNING_DEFAULTS. A fresh dict each call."""
    data = getattr(profile, 'data', profile) or {}
    given = data.get('motion_tuning') or {}
    return {group: {**values, **(given.get(group) or {})} for group, values in MOTION_TUNING_DEFAULTS.items()}
