"""Mission files for the HMI: list, load, validate and save. No ROS dependency.

Saving validates first (schema, then the plan against the skill catalog and the config) and writes nothing on
failure. Existing files are updated in place with ruamel.yaml, so top-level comments and key order survive;
the steps list is rewritten as a whole.
"""

from __future__ import annotations

import io
import re
from collections.abc import Mapping
from pathlib import Path

from jsonschema import Draft202012Validator

from taskweave_core.config import ConfigError, ConfigStore, schemas
from taskweave_core.plan import Plan
from taskweave_core.validation import validate_plan

NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_\-]*$')
TOP_LEVEL_ORDER = ('goal', 'robot', 'loop', 'max_cycles', 'stop_when', 'defaults', 'steps')
STEP_ORDER = ('id', 'instruction', 'skill', 'args', 'timeout_s', 'retries', 'on_failure', 'replan_from', 'max_replans')


class MissionError(ValueError):
    """A mission cannot be saved; ``problems`` lists every reason."""

    def __init__(self, problems: list[str]):
        self.problems = problems
        super().__init__('; '.join(problems))


def check_name(name: str) -> str:
    if not NAME.match(name or ''):
        raise MissionError([f'invalid mission name {name!r}: letters, digits, "_" and "-", starting with a letter'])
    return name


def list_missions(store: ConfigStore) -> list[dict]:
    result = []
    for name in store.mission_names():
        try:
            mission = store.mission(name)
            result.append({'name': name, 'goal': mission['goal'], 'robot': mission['robot'],
                           'steps': len(mission['steps']), 'valid': True})
        except ConfigError as error:
            result.append({'name': name, 'goal': '', 'robot': '', 'steps': 0, 'valid': False, 'error': str(error)})
    return result


def validate(mission: Mapping, store: ConfigStore, catalog: Mapping[str, dict], robot: str) -> list[str]:
    """Every problem with a mission dict; empty means it may be saved. ``catalog``: skill -> args schema."""
    errors = sorted(Draft202012Validator(schemas.MISSION).iter_errors(mission), key=lambda e: list(e.absolute_path))
    if errors:
        return [f'{".".join(str(p) for p in e.absolute_path) or "mission"}: {e.message}' for e in errors]
    problems = []
    if mission['robot'] != robot:
        problems.append(f'mission is for robot {mission["robot"]!r}, this system drives {robot!r}')
    plan = Plan.from_mission(dict(mission), 'hmi-check')
    return problems + validate_plan(plan, catalog, store)


def save(name: str, mission: Mapping, store: ConfigStore, catalog: Mapping[str, dict], robot: str) -> Path:
    """Validate and write missions/<name>.yaml. Raises MissionError with all problems; nothing is written then."""
    check_name(name)
    problems = validate(mission, store, catalog, robot)
    if problems:
        raise MissionError(problems)
    from ruamel.yaml import YAML
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    handler = YAML()
    handler.preserve_quotes = True
    handler.width = 120
    path = store.root / 'missions' / f'{name}.yaml'
    document = handler.load(path.read_text(encoding='utf-8')) if path.is_file() else None
    if not isinstance(document, CommentedMap):
        document = CommentedMap()
    for key in list(document):
        if key not in mission:
            del document[key]
    for key in TOP_LEVEL_ORDER:
        if key not in mission:
            continue
        value = mission[key]
        if key == 'steps':
            steps = CommentedSeq()
            for step in value:
                entry = CommentedMap((k, step[k]) for k in STEP_ORDER if k in step and step[k] not in (None, ''))
                if isinstance(entry.get('args'), Mapping):
                    args = CommentedMap(entry['args'])
                    args.fa.set_flow_style()
                    entry['args'] = args
                steps.append(entry)
            value = steps
        elif key == 'defaults':
            value = CommentedMap(value)
            value.fa.set_flow_style()
        document[key] = value
    path.parent.mkdir(parents=True, exist_ok=True)
    buffer = io.StringIO()
    handler.dump(document, buffer)
    path.write_text(buffer.getvalue(), encoding='utf-8')
    return path


def template(robot: str) -> dict:
    return {'goal': 'Describe what this mission achieves', 'robot': robot, 'loop': 'once',
            'defaults': {'timeout_s': 120, 'retries': 1, 'on_failure': 'abort'},
            'steps': [{'instruction': 'Wait one second', 'skill': 'wait', 'args': {'seconds': 1.0}}]}
