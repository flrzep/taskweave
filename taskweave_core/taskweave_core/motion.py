"""Motion options of the arm skills: where each value comes from, and the effective speed (skill-api-spec 2.1).

Layers, later wins:

1. skill   skills/<skill>.yaml ``motion:`` (and ``speed:``, ``accel:``: the standard speeds and accelerations)
2. robot   robots/<robot>/skill_overrides.yaml ``<skill>: {motion, speed, accel}``
3. target  data of the target: an object point's ``approach_m`` and ``elbow`` (objects/*.yaml, robot
           object_points.yaml),
           a held pose's ``motion``
4. step    ``args.motion``

``resolve`` returns ``{key: {'value': ..., 'source': 'skill' | 'robot' | 'target' | 'step'}}``. Keys are the motion
options (``mode``, ``planner``, ``approach_m``, ``elbow``, ``speed`` as fraction, ``speed_abs``, ``accel`` as fraction,
...) plus ``standard_speed`` and ``standard_accel`` (the skill yaml's ``speed:`` and ``accel:`` numbers, merged per
number: the source is the layer that set the last one). Skills and the HMI (defaults next to each field) share it.
The global speed override is not applied here: speed.py applies it at run time.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from .config import ConfigError, ConfigStore
from .config.schemas import MOTION_PROPERTIES, MOTION_SPEED_KEYS

SOURCES = ('skill', 'robot', 'target', 'step')
TARGET_KEYS = ('approach_m', 'elbow')   # motion options an object point can carry


def resolve(store: ConfigStore, robot: str, skill: str, args: Mapping | None = None,
            target_data: Mapping | None = None) -> dict[str, dict]:
    """The motion options of a step with the layer each value comes from (see the module doc).

    ``args``: the step's arguments (only ``args['motion']`` is read). ``target_data``: the target's own values, flat
    (``approach_m``, ``elbow``) and/or under ``motion`` (a held pose's); see ``target_data_for``.
    """
    result: dict[str, dict] = {}

    def apply(layer: str, motion: Any = None, speed: Any = None, accel: Any = None) -> None:
        if isinstance(motion, Mapping):
            for key, value in motion.items():
                if value is not None:
                    result[key] = {'value': value, 'source': layer}
        for key, data in (('standard_speed', speed), ('standard_accel', accel)):
            if isinstance(data, Mapping) and data:   # a plain number (drive_straight speed) is not a speed table
                merged = dict(result.get(key, {}).get('value', {}))
                merged.update(data)
                result[key] = {'value': merged, 'source': layer}

    config = store.skill_config(skill)
    apply('skill', config.get('motion'), config.get('speed'), config.get('accel'))
    override = store.skill_overrides(robot).get(skill, {})
    apply('robot', override.get('motion'), override.get('speed'), override.get('accel'))
    if target_data:
        apply('target', {**{key: target_data.get(key) for key in TARGET_KEYS}, **(target_data.get('motion') or {})})
    step_motion = (args or {}).get('motion')
    apply('step', step_motion if isinstance(step_motion, Mapping) else None)
    return result


def values(resolved: Mapping[str, Mapping]) -> dict[str, Any]:
    """Just the values of a ``resolve`` result."""
    return {key: entry['value'] for key, entry in resolved.items()}


def target_data_for(store: ConfigStore, robot: str, args: Mapping) -> dict:
    """The target's motion data for a step's args: ``approach_m`` and ``elbow`` of the object point ``args.to.point``
    (or ``args.point``), robot settings on top. {} for anything else, a placeholder, or a point that does not exist."""
    to = args.get('to')
    reference = to.get('point') if isinstance(to, Mapping) else args.get('point')
    if not isinstance(reference, str) or '{' in reference or '/' not in reference:
        return {}
    try:
        _, point = store.instance_point(reference, robot)
    except (KeyError, ConfigError):
        return {}
    return {key: point[key] for key in TARGET_KEYS if point.get(key) is not None}


def effective_speed(standard: float, limit: float | None = None, speed: float | None = None,
                    speed_abs: float | None = None, override: float = 1.0) -> float:
    """``min(limit, standard x speed | speed_abs) x override`` (spec 2.1) for one speed number.

    ``speed``: the step's fraction of the standard (default 1); ``speed_abs``: an absolute speed instead (it wins over
    ``speed``); ``limit``: the robot limit (None: unlimited), which caps the absolute value first. ``override``:
    the global override as a fraction; the run-time value comes from speed.py, the default 1 leaves it out. The
    override only slows a move: it is clamped to 1.
    """
    wanted = speed_abs if speed_abs is not None else standard * (1.0 if speed is None else speed)
    if limit is not None:
        wanted = min(wanted, limit)
    return wanted * min(max(override, 0.0), 1.0)


def effective_speeds(resolved: Mapping[str, Mapping], limits: Mapping[str, float] | None = None,
                     override: float = 1.0) -> dict[str, float]:
    """``effective_speed`` for every speed of the skill's standard table (``resolved['standard_speed']``), with the
    step's ``speed`` / ``speed_abs[key]`` and the robot ``limits`` ({key: max}); keys without a standard value but
    with an absolute one are included."""
    standard = values(resolved).get('standard_speed', {})
    motion = values(resolved)
    fraction, absolute = motion.get('speed'), motion.get('speed_abs') or {}
    limits = limits or {}
    return {key: effective_speed(standard.get(key, 0.0), limits.get(key), fraction, absolute.get(key), override)
            for key in dict.fromkeys([*standard, *(k for k in absolute if k in MOTION_SPEED_KEYS)])}


__all__ = ['MOTION_PROPERTIES', 'resolve', 'values', 'target_data_for', 'effective_speed', 'effective_speeds']
