"""Mission files for the HMI: list, load, validate, save, rename and delete. No ROS dependency.

Saving validates first (schema, then the plan against the skill catalog and the config) and writes nothing on
failure. Existing files are updated in place with ruamel.yaml, so top-level comments and key order survive;
the steps list is rewritten as a whole. A rename is a save under the new name that carries the old file's comments
over and then removes the old file; no other config refers to a mission by name. A running mission keeps its plan
when its file is renamed or deleted.

A refused check names the places of its problems too (``places``: the step of the mission's own steps and the path in
it, from ``validate_plan_structured``), so the editor marks the step and the field. ``preview_plan`` is the plan a
start would run now, with each step's kind of motion and resources (``plan_step_info``) for the plan graph.
"""

from __future__ import annotations

import io
import re
from collections.abc import Mapping
from pathlib import Path

from jsonschema import Draft202012Validator

from taskweave_core.config import ConfigError, ConfigStore, schemas
from taskweave_core.plan import IncludeError, Plan, expand_includes
from taskweave_core.planner import MissionFilePlanner
from taskweave_core.validation import plan_step_info, validate_plan, validate_plan_structured

NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_\-]*$')
TOP_LEVEL_ORDER = ('goal', 'robot', 'loop', 'max_cycles', 'stop_when', 'defaults', 'steps')
# a step's keys in the file; an include step has include and with instead of skill and args
STEP_ORDER = ('id', 'instruction', 'skill', 'args', 'include', 'with', 'timeout_s', 'retries', 'on_failure',
              'replan_from', 'max_replans')


class MissionError(ValueError):
    """A mission cannot be saved; ``problems`` lists every reason, ``places`` where they are (``places_of``)."""

    def __init__(self, problems: list[str], places: list[dict] | None = None):
        self.problems = problems
        self.places = places or []
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
            steps = len(store.expanded_mission(name)['steps'])
            result.append({'name': name, 'goal': mission['goal'], 'robot': mission.get('robot', ''),
                           'steps': steps, 'valid': True, 'params': mission.get('params', {})})
        except ConfigError as error:
            result.append({'name': name, 'goal': '', 'robot': '', 'steps': 0, 'valid': False, 'error': str(error)})
    return result


def validate(mission: Mapping, store: ConfigStore, catalog: Mapping[str, dict], robot: str) -> list[str]:
    """Every problem with a mission dict; empty means it may be saved. ``catalog``: skill -> args schema."""
    return check(mission, store, catalog, robot)[0]


def check(mission: Mapping, store: ConfigStore, catalog: Mapping[str, dict],
          robot: str) -> tuple[list[str], list[dict]]:
    """``(problems, places)`` of a mission dict: the problems as ``validate`` gives them and, for each problem of a
    step, where it is: ``{'problem': index into problems, 'step': index into the mission's own steps (an include
    step for a step it brings in), 'path': the path in that step (['args', 'to'], ['skill'], ...; [] for a step from
    an include), 'include': None or the included mission, 'message': the problem without its step prefix (a
    schema error: as in problems)}``."""
    errors = sorted(Draft202012Validator(schemas.MISSION).iter_errors(mission), key=lambda e: list(e.absolute_path))
    if errors:
        problems = [f'{".".join(str(p) for p in e.absolute_path) or "mission"}: {e.message}' for e in errors]
        paths = [list(e.absolute_path) for e in errors]
        places = [{'problem': i, 'step': path[1], 'path': [str(p) for p in path[2:]], 'include': None,
                   'message': problems[i]}
                  for i, path in enumerate(paths) if len(path) >= 2 and path[0] == 'steps' and isinstance(path[1], int)]
        return problems, places
    problems = []
    if mission.get('robot') and mission['robot'] != robot:
        problems.append(f'mission is for robot {mission["robot"]!r}, this system drives {robot!r}')
    try:
        expanded = expand_includes(dict(mission), store.mission)
    except IncludeError as error:
        return problems + [str(error)], []
    plan = Plan.from_mission(expanded, 'hmi-check', robot=robot)
    texts = validate_plan(plan, catalog, store)
    structured = validate_plan_structured(plan, catalog, store)
    places = places_of(structured, len(problems)) if len(structured) == len(texts) else []
    return problems + texts, places


def places_of(structured: list[dict], offset: int = 0) -> list[dict]:
    """The places of ``validate_plan_structured`` problems in the mission file (see ``check``); plan-level ones
    have none."""
    places = []
    for index, problem in enumerate(structured):
        origin = problem.get('origin')
        if origin is not None:
            inner = origin['include'] is not None
            places.append({'problem': offset + index, 'step': origin['step'],
                           'path': [] if inner else list(origin['inner_path'][2:]), 'include': origin['include'],
                           'message': problem['message']})
        elif len(problem['path']) >= 2 and problem['path'][0] == 'steps':
            places.append({'problem': offset + index, 'step': problem['path'][1], 'path': list(problem['path'][2:]),
                           'include': None, 'message': problem['message']})
    return places


def preview_plan(store: ConfigStore, mission: str, robot: str, catalog: Mapping[str, dict] | None = None) -> dict:
    """The plan a start of ``mission`` would run now, shaped like the live plan (``plan_id`` ``<mission>-preview``);
    no steps when there is nothing to do (e.g. every stack is empty). Each step has ``kind`` (the kind of motion of a
    move step, '' when there is none or a placeholder hides it) and ``resources``; ``catalog`` (skill -> args schema,
    may be empty) gives the resources of the other skills."""
    plan = MissionFilePlanner(store, mission, None, robot).make_plan({}, 1)
    if plan is None:
        return {'plan_id': f'{mission}-preview', 'goal': '', 'robot': robot, 'loop': '', 'steps': [],
                'message': 'nothing to do now'}
    info = plan_step_info(plan, catalog or {}, store)
    return {'plan_id': f'{mission}-preview', 'goal': plan.goal, 'robot': plan.robot, 'loop': plan.loop,
            'steps': [{'id': step.id, 'instruction': step.instruction, 'skill': step.skill, 'args': step.args,
                       'timeout_s': step.timeout_s, 'retries': step.retries, 'on_failure': step.on_failure,
                       'kind': each['kind'] or '', 'resources': list(each['resources'])}
                      for step, each in zip(plan.steps, info, strict=True)]}


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
    found, places = check(mission, store, catalog, robot)
    if problems or found:
        raise MissionError(problems + found, [{**place, 'problem': place['problem'] + len(problems)}
                                              for place in places])
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
                for flow in ('args', 'with'):
                    if isinstance(entry.get(flow), Mapping):
                        values = CommentedMap(entry[flow])
                        values.fa.set_flow_style()
                        entry[flow] = values
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
