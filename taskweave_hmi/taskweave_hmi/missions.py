"""Mission files for the HMI: list, load, validate, save, rename and delete. No ROS dependency.

Saving validates first (schema, then the plan against the skill catalog and the config) and writes nothing on
failure. Existing files are updated in place with ruamel.yaml, so top-level comments and key order survive;
the steps list is rewritten as a whole. A rename is a save under the new name that carries the old file's comments
over and then removes the old file; no other config refers to a mission by name. A running mission keeps its plan
when its file is renamed or deleted.
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
            result.append({'name': name, 'goal': mission['goal'], 'robot': mission.get('robot', ''),
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
    if mission.get('robot') and mission['robot'] != robot:
        problems.append(f'mission is for robot {mission["robot"]!r}, this system drives {robot!r}')
    plan = Plan.from_mission(dict(mission), 'hmi-check', robot=robot)
    return problems + validate_plan(plan, catalog, store)


def mission_path(store: ConfigStore, name: str) -> Path:
    return store.root / 'missions' / f'{check_name(name)}.yaml'


def save(name: str, mission: Mapping, store: ConfigStore, catalog: Mapping[str, dict], robot: str,
         rename_from: str | None = None) -> Path:
    """Validate and write missions/<name>.yaml; with ``rename_from``, the mission of that name becomes ``name``.
    Raises MissionError with all problems; nothing is written then."""
    path = mission_path(store, name)
    source = mission_path(store, rename_from) if rename_from and rename_from != name else path
    problems = []
    if source != path:
        if not source.is_file():
            problems.append(f'mission {rename_from!r} does not exist')
        if path.exists():
            problems.append(f'a mission named {name!r} already exists')
    problems += validate(mission, store, catalog, robot)
    if problems:
        raise MissionError(problems)
    from ruamel.yaml import YAML
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    handler = YAML()
    handler.preserve_quotes = True
    handler.width = 120
    document = handler.load(source.read_text(encoding='utf-8')) if source.is_file() else None
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
    if source != path:
        source.unlink()
    store.reload()
    return path


def delete(name: str, store: ConfigStore) -> Path:
    path = mission_path(store, name)
    if not path.is_file():
        raise MissionError([f'mission {name!r} does not exist'])
    path.unlink()
    store.reload()
    return path


def template(robot: str) -> dict:
    return {'goal': 'Describe what this mission achieves', 'loop': 'once',
            'defaults': {'timeout_s': 120, 'retries': 1, 'on_failure': 'abort'},
            'steps': [{'instruction': 'Wait one second', 'skill': 'wait', 'args': {'seconds': 1.0}}]}
