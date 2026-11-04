"""Mission files for the HMI: list, load, validate, save, rename and delete. No ROS dependency.

Saving validates first (schema, then the plan against the skill catalog and the config) and writes nothing on
failure. Existing files are updated in place with ruamel.yaml, so top-level comments and key order survive;
the steps list is rewritten as a whole. A rename is a save under the new name that carries the old file's comments
over and then removes the old file; no other config refers to a mission by name. A running mission keeps its plan
when its file is renamed or deleted.

A refused check names the places of its problems too (``places``: the step of the mission's own steps and the path in
it, from ``validate_plan_structured``), so the editor marks the step and the field. ``preview_plan`` is the plan a
start would run now, with each step's kind of motion and resources (``plan_step_info``) and, for the plan graph's
phases and replan lines, its ``origin`` (``step_origin``), ``replan_from`` and ``max_replans``, as the live plan
has them.
``edit`` is the graph editor's save: operator-style ops (insert, remove, move, wrap, unwrap, set; paths like
``steps/0/steps/1/parallel/0/steps/2``) applied on the file's ruamel tree in place, so comments inside the steps
survive too, then validated and written as a whole; the ops of a mission that does not exist yet make the file.
Both carry the notes of the steps too (what validation notes without stopping it, e.g. the move form of cell schema
4): ``notes`` for a check, ``notes`` on each step of a preview.
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
from taskweave_core.step_info import MOVE_SKILL, move_notes
from taskweave_core.validation import plan_step_info, validate_plan, validate_plan_structured

NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_\-]*$')
TOP_LEVEL_ORDER = ('goal', 'robot', 'params', 'facts', 'loop', 'max_cycles', 'stop_when', 'defaults', 'steps')
# a step's keys in the file; an include step has include and with instead of skill and args; a block (schema v2) has
# group, repeat or parallel and its steps or lanes
STEP_ORDER = ('id', 'instruction', 'skill', 'args', 'include', 'with', 'group', 'repeat', 'parallel', 'when', 'join',
              'max_cycles', 'timeout_s', 'retries', 'on_failure', 'replan_from', 'max_replans', 'steps')
BLOCK_KEYS = ('group', 'repeat', 'parallel')
LIST_KEYS = ('steps', 'parallel')   # the lists an op's path runs through; set edits the other keys
FLOW_KEYS = ('args', 'with', 'defaults', 'repeat')   # mappings written on one line


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


def step_notes(skill: str, args: Mapping, store: ConfigStore) -> list[str]:
    """The notes of a step as ``validate_plan`` puts them on ``Step.notes`` ('args.object: ...')."""
    if skill != MOVE_SKILL:
        return []
    return [f'{".".join(str(part) for part in path)}: {message}' for path, message in move_notes(args, store)]


def notes(mission: Mapping, store: ConfigStore, robot: str) -> list[dict]:
    """The notes of a mission dict's steps, placed as ``check`` places a problem: ``{'step': index into the
    mission's own steps (an include step for a step it brings in), 'path': the path in that step ([] for a step from
    an include), 'include': None or the included mission, 'message'}``; none for a mission that does not expand
    (``check`` says why)."""
    if not Draft202012Validator(schemas.MISSION).is_valid(mission):
        return []
    try:
        plan = Plan.from_mission(expand_includes(dict(mission), store.mission), 'hmi-check', robot=robot)
    except (IncludeError, ConfigError, ValueError):
        return []
    result = []
    for step in plan.steps:
        if step.skill != MOVE_SKILL:
            continue
        origin = step.origin or {}
        include = (origin.get('include') or [None])[0]
        for path, message in move_notes(step.args, store):
            where = '.'.join(str(part) for part in path)
            result.append({'step': origin.get('step'), 'path': [] if include else list(path), 'include': include,
                           'message': f'{where}: {message}' if include else message})
    return result


def step_origin(origin: Mapping | None) -> dict | None:
    """Where a plan step came from, as the plan JSON has it (PlanStep.origin_json): ``{'include': names of the
    includes, outermost first ([] for the mission's own steps), 'step': index in the innermost mission's own steps,
    'top': index of the step, or of the include step it came from, in the top mission's own steps}``; None when the
    step is not from a mission. ``origin`` is core's ``Step.origin`` (``step`` is the top index there)."""
    if not origin:
        return None
    return {'include': list(origin.get('include') or []), 'step': origin.get('inner_step', origin.get('step')),
            'top': origin.get('step')}


def preview_plan(store: ConfigStore, mission: str, robot: str, catalog: Mapping[str, dict] | None = None) -> dict:
    """The plan a start of ``mission`` would run now, shaped like the live plan (``plan_id`` ``<mission>-preview``);
    no steps when there is nothing to do (e.g. every stack is empty). Each step has ``kind`` (the kind of motion of a
    move step, '' when there is none or a placeholder hides it), ``resources``, ``origin`` (``step_origin``),
    ``replan_from`` ('' for the step itself), ``max_replans`` and ``notes`` (``step_notes``);
    ``catalog`` (skill -> args schema, may be empty) gives the resources of the other skills."""
    plan = MissionFilePlanner(store, mission, None, robot).make_plan({}, 1)
    if plan is None:
        return {'plan_id': f'{mission}-preview', 'goal': '', 'robot': robot, 'loop': '', 'steps': [],
                'message': 'nothing to do now'}
    info = plan_step_info(plan, catalog or {}, store)
    return {'plan_id': f'{mission}-preview', 'goal': plan.goal, 'robot': plan.robot, 'loop': plan.loop,
            'steps': [{'id': step.id, 'instruction': step.instruction, 'skill': step.skill, 'args': step.args,
                       'timeout_s': step.timeout_s, 'retries': step.retries, 'on_failure': step.on_failure,
                       'kind': each['kind'] or '', 'resources': list(each['resources']),
                       'origin': step_origin(step.origin), 'replan_from': step.replan_from or '',
                       'max_replans': step.max_replans, 'notes': step_notes(step.skill, step.args, store)}
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



# ---- edits from the graph editor ----
def _parts(path: str) -> list[str]:
    return [p for p in str(path or '').split('/') if p != '']


def _walk(doc, parts: list[str], what: str):
    """The node at ``parts`` of the tree, or MissionError naming ``what``."""
    node = doc
    for p in parts:
        try:
            node = node[int(p)] if isinstance(node, list) else node[p]
        except (KeyError, IndexError, ValueError, TypeError):
            raise MissionError([f'{what}: no {"/".join(parts)} in the mission']) from None
    return node


def _slot(doc, path: str, what: str, make: bool = False) -> tuple:
    """``(list, index)`` for a path into a list (``steps/2``); ``make``: an absent ``steps`` or ``parallel`` of a
    mapping is created (an insert into an empty mission or block) and the index may be the list's length."""
    from ruamel.yaml.comments import CommentedSeq

    parts = _parts(path)
    if len(parts) < 2 or parts[-2] not in LIST_KEYS:
        raise MissionError([f'{what}: {path!r} is not a place in a list of steps'])
    try:
        index = int(parts[-1])
    except ValueError:
        raise MissionError([f'{what}: {path!r} does not end in an index']) from None
    parent = _walk(doc, parts[:-2], what)
    if not isinstance(parent, Mapping):
        raise MissionError([f'{what}: no list at {"/".join(parts[:-1])}'])
    if parts[-2] not in parent:
        if not make:
            raise MissionError([f'{what}: no list at {"/".join(parts[:-1])}'])
        parent[parts[-2]] = CommentedSeq()
    seq = parent[parts[-2]]
    if not isinstance(seq, list):
        raise MissionError([f'{what}: {"/".join(parts[:-1])} is not a list'])
    if index < 0 or index > len(seq) - (0 if make else 1):
        raise MissionError([f'{what}: index {index} is outside {"/".join(parts[:-1])} ({len(seq)} entries)'])
    return seq, index


def _yaml(value, key: str | None = None):
    """A JSON value as ruamel nodes: steps with their keys in file order, flow style for the one-line mappings."""
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    if isinstance(value, Mapping):
        keys = list(value)
        if key in (None, 'steps', 'parallel') and ('instruction' in value or 'steps' in value or 'skill' in value):
            keys = [k for k in STEP_ORDER if k in value] + [k for k in value if k not in STEP_ORDER]
        out = CommentedMap((k, _yaml(value[k], k)) for k in keys if value[k] is not None)
        if key in FLOW_KEYS:
            out.fa.set_flow_style()
        return out
    if isinstance(value, list):
        return CommentedSeq(_yaml(v, key) for v in value)
    return value


def plain(node):
    """A ruamel tree as plain dicts and lists."""
    if isinstance(node, Mapping):
        return {k: plain(v) for k, v in node.items()}
    if isinstance(node, list):
        return [plain(v) for v in node]
    return node


def _block_steps(block) -> list:
    """The steps a block holds, in order (a parallel: its lanes' steps one after the other)."""
    if 'parallel' in block:
        return [step for lane in block['parallel'] if isinstance(lane, Mapping) for step in lane.get('steps') or []]
    return list(block.get('steps') or [])


def apply_ops(doc, ops: list) -> None:
    """Apply the editor's ops on a ruamel tree in place (the file's comments stay). An op that cannot be applied
    raises MissionError naming it; the ops before it have been applied then (``edit`` discards the tree).
    ``insert {at, step}``: the step (a dict) at the path, ``at`` an index up to the list's length; ``remove {at}``;
    ``move {from, to}``: ``to`` names the slot as it is before the move; ``wrap {from, to, as, block}``: the steps
    from ``from`` to ``to`` (the same list) become the steps of a new block (as: group | repeat | parallel, the block's
    own keys in ``block``); ``unwrap {at}``: a block is replaced by its steps; ``set {at, key, value}``: one key of a
    step, block or the mission ('' at), None removes it."""
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    for n, op in enumerate(ops):
        kind = op.get('op') if isinstance(op, Mapping) else None
        what = f'op {n + 1} ({kind})'
        if kind == 'insert':
            step = op.get('step')
            if not isinstance(step, Mapping):
                raise MissionError([f'{what}: the step must be an object'])
            seq, i = _slot(doc, op.get('at'), what, make=True)
            seq.insert(i, _yaml(step))
        elif kind == 'remove':
            seq, i = _slot(doc, op.get('at'), what)
            del seq[i]
        elif kind == 'move':
            src, to = str(op.get('from') or ''), str(op.get('to') or '')
            if to.startswith(src.rstrip('/') + '/'):
                raise MissionError([f'{what}: cannot move a block into itself'])
            seq_from, i = _slot(doc, src, what)
            seq_to, j = _slot(doc, to, what, make=True)
            item = seq_from.pop(i)
            if seq_to is seq_from and j > i:
                j -= 1
            seq_to.insert(j, item)
        elif kind == 'wrap':
            block = op.get('block')
            as_ = op.get('as')
            if as_ not in BLOCK_KEYS or not isinstance(block, Mapping):
                raise MissionError([f'{what}: as must be group, repeat or parallel, with the block as an object'])
            seq, i = _slot(doc, op.get('from'), what)
            seq_to, j = _slot(doc, op.get('to'), what)
            if seq_to is not seq or j < i:
                raise MissionError([f'{what}: from and to must be in the same list, from first'])
            block = {k: v for k, v in block.items() if k not in LIST_KEYS}
            if as_ == 'group':
                block['group'] = True
            elif as_ == 'repeat':
                block['repeat'] = block.get('repeat') if isinstance(block.get('repeat'), Mapping) else {}
            node = _yaml(block)
            inner = CommentedSeq(seq[i:j + 1])
            if as_ == 'parallel':
                node['parallel'] = CommentedSeq([CommentedMap(steps=inner)])
            else:
                node['steps'] = inner
            del seq[i:j + 1]
            seq.insert(i, node)
        elif kind == 'unwrap':
            seq, i = _slot(doc, op.get('at'), what)
            block = seq[i]
            if not isinstance(block, Mapping) or not any(k in block for k in BLOCK_KEYS):
                raise MissionError([f'{what}: {op.get("at")} is not a block'])
            seq[i:i + 1] = _block_steps(block)
        elif kind == 'set':
            key = op.get('key')
            if not isinstance(key, str) or not key or key in LIST_KEYS:
                raise MissionError([f'{what}: key must name a field (not {key!r})'])
            node = _walk(doc, _parts(op.get('at')), what)
            if not isinstance(node, Mapping):
                raise MissionError([f'{what}: {op.get("at")!r} is not a step, block or the mission'])
            if op.get('value') is None:
                node.pop(key, None)
            else:
                node[key] = _yaml(op['value'], key)
        else:
            raise MissionError([f'{what}: unknown op'])


def edit(name: str, ops: list, store: ConfigStore, catalog: Mapping[str, dict], robot: str,
         rename_from: str | None = None) -> dict:
    """Apply ``ops`` (``apply_ops``) on missions/<name>.yaml (an empty document when there is no such file), validate
    the result and write it: ``{'mission': the mission as saved, 'file': its path in the config}``. MissionError with
    every problem, nothing written then. ``rename_from``: the ops apply to that mission's file, saved as ``name``."""
    from ruamel.yaml import YAML
    from ruamel.yaml.comments import CommentedMap

    path = mission_path(store, name)
    source = mission_path(store, rename_from) if rename_from and rename_from != name else path
    problems = []
    if source != path:
        if not source.is_file():
            problems.append(f'mission {rename_from!r} does not exist')
        if path.exists():
            problems.append(f'a mission named {name!r} already exists')
    if problems:
        raise MissionError(problems)
    handler = YAML()
    handler.preserve_quotes = True
    handler.width = 120
    document = handler.load(source.read_text(encoding='utf-8')) if source.is_file() else None
    if not isinstance(document, CommentedMap):
        document = CommentedMap()
    apply_ops(document, list(ops or []))
    mission = plain(document)
    found, places = check(mission, store, catalog, robot)
    if found:
        raise MissionError(found, places)
    path.parent.mkdir(parents=True, exist_ok=True)
    buffer = io.StringIO()
    handler.dump(document, buffer)
    path.write_text(buffer.getvalue(), encoding='utf-8')
    if source != path:
        source.unlink()
    store.reload()
    return {'mission': mission, 'file': path.relative_to(store.root).as_posix()}


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
