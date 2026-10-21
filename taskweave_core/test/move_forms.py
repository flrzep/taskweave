"""Helpers of the targets migration tests: what a migrated ``move`` step asks for, checked against the cell (the move
skill itself is another task; this is the data side): the form of the step, the kind of the target it names, the
options that form takes (targets.move_option_problems), and the step ids of a mission before and after."""

import re
from pathlib import Path

import yaml

from taskweave_core.config import ConfigStore
from taskweave_core.config import targets as model

SELECTORS = ('to', 'hand', 'object', 'base', 'elbow', 'joints')
PLACEHOLDER = re.compile(r'\{\w+\}')


def run_time_objects(store: ConfigStore) -> set[str]:
    """The names ``grip place_as`` creates in the missions: objects that exist only while a mission runs."""
    found = set()
    for name in store.mission_names():
        for step in store.mission(name).get('steps', []):
            if (step.get('args') or {}).get('place_as'):
                found.add(step['args']['place_as'])
    return found


def form_of(
    args: dict, store: ConfigStore | None = None, robot: str | None = None, runtime: frozenset | set = frozenset()
) -> tuple[str | None, list[str]]:
    """``(form, problems)`` of one move step's args: base, straight, hand, object, elbow or joints."""
    problems: list[str] = []
    if 'to' in args:
        target = args['to']
        if PLACEHOLDER.search(str(target)) or str(target).partition('/')[0] in runtime:
            return None, problems  # known when the step starts
        if store is None:
            return None, problems
        try:
            found = store.target(str(target), robot)
        except KeyError as error:
            return None, [f'to {target}: {error}']
        mode = (args.get('motion') or {}).get('mode')
        return ('straight' if found.kind == 'base' and mode == 'straight' else found.kind), problems
    if 'hand' in args:
        return 'hand', problems
    if 'object' in args:
        return 'object', problems
    if 'base' in args:
        return 'straight', problems
    if 'elbow' in args:
        return 'elbow', problems
    if 'joints' in args:
        return 'joints', problems
    return None, ['no form selector (to, hand, object, base, elbow, joints)']


def step_problems(
    args: dict, store: ConfigStore | None = None, robot: str | None = None, runtime: frozenset | set = frozenset()
) -> list[str]:
    form, problems = form_of(args, store, robot, runtime)
    if form is not None:
        options = {k: v for k, v in args.items() if k not in SELECTORS}
        problems += model.move_option_problems(form, options)
    return problems


def mission_problems(store: ConfigStore, name: str) -> list[str]:
    """Every move step of mission ``name``: its form and options hold, its target resolves."""
    mission = store.mission(name)
    robots = store.robot_names()
    robot = mission.get('robot') or store.system().get('robot') or (robots[0] if len(robots) == 1 else None)
    problems, runtime = [], run_time_objects(store)
    for number, step in enumerate(mission['steps'], start=1):
        if step.get('skill') != 'move':
            continue
        label = f'{name} {step.get("id", f"s{number}")}'
        problems += [f'{label}: {p}' for p in step_problems(step.get('args') or {}, store, robot, runtime)]
    return problems


def step_shape(path: Path) -> list[tuple]:
    """What must not change in a migrated mission: per step its id, instruction, include, with and settings."""
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    keep = ('id', 'instruction', 'include', 'with', 'timeout_s', 'retries', 'on_failure', 'replan_from', 'max_replans')
    return [tuple((k, step.get(k)) for k in keep) for step in data.get('steps', []) if isinstance(step, dict)]
