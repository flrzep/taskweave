"""Mission files for the HMI: list, load, validate, save, rename and delete. No ROS dependency.

Saving validates first (schema, then the plan against the skill catalog and the config) and writes nothing on
failure. Existing files are updated in place with ruamel.yaml, so top-level comments and key order survive;
the steps list is rewritten as a whole. A rename is a save under the new name that carries the old file's comments
over and then removes the old file; missions include each other by name, so a rename or delete of a mission other
missions include is refused (``included_by``). The routes refuse every mission write while a mission runs: the
planner rereads the file for each cycle of a looping mission and on a replan.

A refused check names the places of its problems too (``places``: the step of the mission's own steps and the path in
it, from ``validate_plan_structured``), so the editor marks the step and the field. ``preview_plan`` is the plan a
start would run now, with each step's kind of motion and resources (``plan_step_info``) and, for the plan graph's
phases and replan lines, its ``origin`` (``step_origin``), ``replan_from`` and ``max_replans``, as the live plan
has them.
``edit`` is the graph editor's save: operator-style ops (insert, remove, move, wrap, unwrap, set; paths like
``steps/0/steps/1/parallel/0/steps/2``) applied on the file's ruamel tree in place, so comments inside the steps
survive too, then validated and written as a whole through ``ConfigStore.save`` (the pack lock the other writers
take, an atomic write, a commit when the pack is a git work tree); the ops of a new mission make the file. The ops
of schema v2 (blocks: group, repeat, parallel, when) are refused on a core whose schema lacks them
(``blocks_supported``): the file would not load there.
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
INDEX = re.compile(r'^(0|[1-9][0-9]*)$')   # a list index in an op path, as written: no sign, no leading zero


class MissionError(ValueError):
    """A mission cannot be saved; ``problems`` lists every reason, ``places`` where they are (``places_of``)."""

    def __init__(self, problems: list[str], places: list[dict] | None = None):
        self.problems = problems
        self.places = places or []
        super().__init__('; '.join(problems))


class MissionConflict(MissionError):
    """The file changed since the editor opened it: the ops would land on other steps (``edit``'s ``base``)."""


def blocks_supported() -> bool:
    """Whether this core's mission schema knows blocks (group, repeat, parallel, when): schema v2."""
    return all(key in schemas.STEP['properties'] for key in BLOCK_KEYS)


def features() -> list[str]:
    """What this core's mission schema supports, for the editor: ``mission_blocks`` with schema v2."""
    return ['mission_blocks'] if blocks_supported() else []


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
        problems += included_by(store, rename_from, 'renamed')
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



def included_by(store: ConfigStore, name: str, what: str) -> list[str]:
    """The problem when other missions include ``name`` (they would not load once it is ``what``: renamed,
    deleted); none otherwise. A mission that does not load itself is skipped."""
    store.reload()   # a mission written by hand since the last reload counts too
    users = sorted(other for other in store.mission_names() if other != name and _includes(store, other, name))
    if not users:
        return []
    return [f'mission {name!r} is included by {", ".join(users)}: it cannot be {what} while they refer to it']


def _includes(store: ConfigStore, mission: str, name: str) -> bool:
    try:
        steps = store.mission(mission).get('steps') or []
    except ConfigError:
        return False

    def walk(items) -> bool:
        for step in items:
            if not isinstance(step, Mapping):
                continue
            if step.get('include') == name or walk(step.get('steps') or []) or walk(_block_steps(step)):
                return True
        return False
    return walk(steps)


# ---- edits from the graph editor ----
def _parts(path, what: str) -> list[str]:
    """The components of an op path (``steps/0/steps/1``, '' for the mission); an empty component (a leading or a
    doubled '/') is refused: a path names one place, one way (the move guard compares paths)."""
    text = str(path or '')
    parts = text.split('/') if text else []
    if '' in parts:
        raise MissionError([f'{what}: {text!r} is not a path in the mission (an empty part)'])
    return parts


def _walk(doc, parts: list[str], what: str):
    """The node at ``parts`` of the tree, or MissionError naming ``what``."""
    node = doc
    for p in parts:
        try:
            if isinstance(node, list):
                if not INDEX.match(p):   # no negatives, signs or leading zeros: one spelling per place
                    raise ValueError(p)
                node = node[int(p)]
            else:
                node = node[p]
        except (KeyError, IndexError, ValueError, TypeError):
            raise MissionError([f'{what}: no {"/".join(parts)} in the mission']) from None
    return node


def _slot(doc, path: str, what: str, make: bool = False) -> tuple:
    """``(list, index)`` for a path into a list (``steps/2``); ``make``: an absent ``steps`` or ``parallel`` of a
    mapping is created (an insert into an empty mission or block) and the index may be the list's length."""
    from ruamel.yaml.comments import CommentedSeq

    parts = _parts(path, what)
    if len(parts) < 2 or parts[-2] not in LIST_KEYS:
        raise MissionError([f'{what}: {path!r} is not a place in a list of steps'])
    if not INDEX.match(parts[-1]):
        raise MissionError([f'{what}: {path!r} does not end in an index (0, 1, 2 ...)'])
    index = int(parts[-1])
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
    """A JSON value as ruamel nodes: steps with their keys in file order and without their null keys (a null
    from a form field is an absent key), flow style for the one-line mappings. Nulls inside a value stay: a
    null in a joint-group value means that joint is left alone."""
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    if isinstance(value, Mapping):
        keys = list(value)
        step = key in (None, 'steps', 'parallel') and ('instruction' in value or 'steps' in value or 'skill' in value)
        if step:
            keys = [k for k in STEP_ORDER if k in value] + [k for k in value if k not in STEP_ORDER]
        out = CommentedMap((k, _yaml(value[k], k)) for k in keys if value[k] is not None or not step)
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
    if isinstance(block.get('parallel'), list):
        return [step for lane in block['parallel'] if isinstance(lane, Mapping) for step in lane.get('steps') or []]
    return list(block.get('steps') or [])


def _holds(node, target) -> bool:
    """Whether ``target`` (a list of the tree) is ``node`` or inside it."""
    if node is target:
        return True
    inner = node.values() if isinstance(node, Mapping) else node if isinstance(node, list) else ()
    return any(_holds(v, target) for v in inner)


def unsupported(ops) -> list[str]:
    """The problem when ``ops`` write what this core's schema lacks (``blocks_supported``): wrap and unwrap, an
    inserted step or a set key of schema v2; the file would not load on this core. None with schema v2."""
    missing = [k for k in STEP_ORDER if k not in schemas.STEP['properties']]
    missing_top = [k for k in TOP_LEVEL_ORDER if k not in schemas.MISSION['properties']]
    if not missing and not missing_top:
        return []
    bad = []
    for n, op in enumerate(ops or []):
        kind = op.get('op') if isinstance(op, Mapping) else None
        keys = []
        if kind in ('wrap', 'unwrap'):
            keys = ['a block']
        elif kind == 'insert' and isinstance(op.get('step'), Mapping):
            keys = [k for k in missing if k in op['step']]
        elif kind == 'set':
            at_step = _parts(op.get('at'), f'op {n + 1} ({kind})')   # '' is the mission, else a step or block
            keys = [op['key']] if op.get('key') in (missing if at_step else missing_top) else []
        if keys:
            bad.append(f'op {n + 1} ({kind}): {", ".join(keys)}')
    if not bad:
        return []
    return [f'this core has no mission blocks (group, repeat, parallel, when): {"; ".join(bad)}']


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
            seq_from, i = _slot(doc, op.get('from'), what)
            seq_to, j = _slot(doc, op.get('to'), what, make=True)
            item = seq_from[i]
            if seq_to is not seq_from and _holds(item, seq_to):   # by identity: the paths may spell it any way
                raise MissionError([f'{what}: cannot move a block into itself'])
            seq_from.pop(i)
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
            node = _walk(doc, _parts(op.get('at'), what), what)
            if not isinstance(node, Mapping):
                raise MissionError([f'{what}: {op.get("at")!r} is not a step, block or the mission'])
            if op.get('value') is None:
                node.pop(key, None)
            else:
                node[key] = _yaml(op['value'], key)
        else:
            raise MissionError([f'{what}: unknown op'])


def edit(name: str, ops: list, store: ConfigStore, catalog: Mapping[str, dict], robot: str,
         rename_from: str | None = None, base: str | None = None, create: bool = False) -> dict:
    """Apply ``ops`` (``apply_ops``) on missions/<name>.yaml, validate the result and write it: ``{'mission': the
    mission as saved, 'file': its path in the config, 'hash': the file's hash now (the next edit's base)}``.
    MissionError with every problem, nothing written then. ``create``: the mission is new, made from the ops on an
    empty document (refused when a file of that name exists; without it a missing file is refused: the ops name
    places in a file). ``base``: the file's hash when the editor opened it (``ConfigStore.file_hash``, the ETag of
    its GET), required unless ``create``: the ops name places in that version; a file that changed since is a
    MissionConflict, nothing written. The write is ``ConfigStore.save`` (the pack lock, an atomic write, a commit
    in a git work tree). ``rename_from``: the ops apply to that mission's file, saved as ``name`` and the old file
    deleted; refused while other missions include the old name. Ops of schema v2 are refused on a core without it
    (``unsupported``)."""
    from ruamel.yaml import YAML
    from ruamel.yaml.comments import CommentedMap

    from taskweave_core.config.save import SaveConflict, SaveError

    path = mission_path(store, name)
    source = mission_path(store, rename_from) if rename_from and rename_from != name else path
    problems = []
    if source != path and not source.is_file():
        problems.append(f'mission {rename_from!r} does not exist')
    if (source != path or create) and path.exists():
        problems.append(f'a mission named {name!r} already exists')
    if not create and source == path and not path.is_file():
        problems.append(f'mission {name!r} does not exist (deleted since it was opened?)')
    if source != path and source.is_file():
        problems += included_by(store, rename_from, 'renamed')
    if base is None and not create:
        problems.append('base is missing: the hash the mission was read with (the ETag of its GET), '
                        'the ops name places in that version of the file')
    problems += unsupported(ops)
    if problems:
        raise MissionError(problems)
    relative = source.relative_to(store.root).as_posix()
    if not create and store.file_hash(relative) != base:
        raise MissionConflict([f'{relative} changed since it was opened (another editor, the pad or a hand edit): '
                               'open the mission again and redo your changes'])
    handler = YAML()
    handler.preserve_quotes = True
    handler.width = 120
    try:
        document = handler.load(source.read_text(encoding='utf-8')) if source.is_file() else None
    except Exception as error:  # noqa: BLE001 - ruamel's errors have no common base worth naming
        raise MissionError([f'{relative} is not valid YAML: {str(error).splitlines()[0]}']) from error
    if not isinstance(document, CommentedMap):
        document = CommentedMap()
    apply_ops(document, list(ops or []))
    mission = plain(document)
    found, places = check(mission, store, catalog, robot)
    if found:
        raise MissionError(found, places)
    buffer = io.StringIO()
    handler.dump(document, buffer)
    file = path.relative_to(store.root).as_posix()
    try:
        saved = store.save(file, buffer.getvalue(), base=None if create or source != path else base,
                           message=f'HMI: edit mission {name}')
    except SaveConflict as error:
        raise MissionConflict([f'{relative} changed since it was opened (another editor, the pad or a hand edit): '
                               'open the mission again and redo your changes']) from error
    except SaveError as error:
        errors = [f'{p["path"]}{p["where"]}: {p["message"]}' for p in error.problems if p['severity'] == 'error']
        raise MissionError(errors or [str(error)]) from error
    if source != path:   # the old file goes under its own lock: a failure here leaves both, and says so
        try:
            store.delete(relative, base=base, message=f'HMI: rename mission {rename_from} to {name}')
        except (SaveConflict, SaveError) as error:
            raise MissionError([f'mission {name!r} was saved, but {rename_from!r} could not be removed ({error}): '
                                f'both exist now. Undo this save, or open {name!r} to go on and delete '
                                f'{rename_from!r} yourself']) from error
    return {'mission': mission, 'file': file, 'hash': saved['hash']}


def delete(name: str, store: ConfigStore) -> Path:
    path = mission_path(store, name)
    if not path.is_file():
        raise MissionError([f'mission {name!r} does not exist'])
    problems = included_by(store, name, 'deleted')
    if problems:
        raise MissionError(problems)
    path.unlink()
    store.reload()
    return path


def template(robot: str) -> dict:
    return {'goal': 'Describe what this mission achieves', 'loop': 'once',
            'defaults': {'timeout_s': 120, 'retries': 1, 'on_failure': 'abort'},
            'steps': [{'instruction': 'Wait one second', 'skill': 'wait', 'args': {'seconds': 1.0}}]}
