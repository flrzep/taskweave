"""taskweave command line.

    taskweave check CONFIG_DIR            validate every config file offline (no ROS needed)
    taskweave skills                      list the skill catalog
    taskweave run MISSION [--goal TEXT]   start a mission
    taskweave status [--follow]           show the task status
    taskweave pause | resume | abort
    taskweave teach location NAME         save the current base pose as a cell location
    taskweave teach stance OBJ[/VARIANT] [--instance]
                                          save it as the robot's stance at an object (for the object type,
                                          or for this instance only)
    taskweave teach point INSTANCE/POINT [--tool TOOL ...]
                                          save the tools' current TCP poses as a point on the object type
    taskweave describe [CONFIG_DIR ROBOT]  robot card + skills (running system, or offline from config)
"""

from __future__ import annotations

import argparse
import json
import sys

STATE_NAMES = ['idle', 'running', 'paused', 'succeeded', 'failed', 'aborted']


def describe_offline(config_dir: str, robot: str) -> int:
    """Robot card from config alone: skills = those with an implementation that fits the robot."""
    from .config import ConfigStore
    from .embodiment import robot_card
    from .skill_server import load_skill_class

    store = ConfigStore(config_dir)
    skills = []
    for path in sorted((store.root / 'skills').glob('*.yaml')):
        implementation, reasons = store.select_implementation(path.stem, robot)
        if implementation is None:
            print(f'# {path.stem}: not available ({"; ".join(reasons)})', file=sys.stderr)
            continue
        try:
            cls = load_skill_class(implementation['class'])
        except (ImportError, AttributeError, TypeError, ValueError) as error:
            print(f'# {path.stem}: implementation {implementation["name"]} cannot be loaded ({error})', file=sys.stderr)
            continue
        skills.append({'name': cls.name, 'description': cls.description, 'args_schema': cls.args_schema,
                       'needs': cls.needs, 'achieves': cls.achieves,
                       'implementation': f'{implementation["type"]}:{implementation["name"]}'})
    print(robot_card(store.embodiment(robot), skills, store.point_names(robot)))
    return 0


def describe(node) -> int:
    from taskweave_interfaces.srv import DescribeRobot

    from . import names

    print(_call(node, DescribeRobot, names.DESCRIBE_ROBOT, DescribeRobot.Request()).card)
    return 0


def check(config_dir: str) -> int:
    from .config import ConfigError, ConfigStore
    from .plan import Plan

    store = ConfigStore(config_dir)
    problems = 0

    def attempt(label, function):
        nonlocal problems
        try:
            function()
            print(f'  ok    {label}')
        except (ConfigError, KeyError, ValueError) as error:
            problems += 1
            print(f'  FAIL  {label}: {error}')

    robots = sorted(p.name for p in (store.root / 'robots').glob('*') if p.is_dir())
    for robot in robots:
        attempt(f'robots/{robot}', lambda r=robot: (store.robot(r), store.stances(r)))
        attempt(f'robots/{robot}/embodiment.yaml', lambda r=robot: _consistent(store, r))
    attempt('cell/locations.yaml', store.locations)
    attempt('cell/objects.yaml', store.object_instances)
    attempt('operator.yaml', store.operator)
    for path in sorted((store.root / 'objects').glob('*.yaml')):
        attempt(f'objects/{path.name}', lambda n=path.stem: store.object(n))
    for path in sorted((store.root / 'skills').glob('*.yaml')):
        attempt(f'skills/{path.name}', lambda n=path.stem: _importable(store.skill_config(n)))
    for name in store.mission_names():
        attempt(f'missions/{name}.yaml', lambda n=name: Plan.from_mission(store.expanded_mission(n), 'check'))
    print(f'{problems} problem(s)' if problems else 'config ok (skill arguments are checked against a running catalog)')
    return 1 if problems else 0


def _consistent(store, robot: str) -> None:
    from .embodiment import check_profile

    problems = check_profile(store.embodiment(robot), store.robot(robot).data)
    if problems:
        raise ValueError('; '.join(problems))


def _importable(skill_config: dict) -> None:
    from .skill_server import load_skill_class

    broken = []
    for implementation in skill_config.get('implementations', []):
        try:
            load_skill_class(implementation['class'])
        except (ImportError, AttributeError, TypeError, ValueError) as error:
            broken.append(f'{implementation["name"]}: {error}')
    if broken:
        raise ValueError('cannot load ' + '; '.join(broken))


def _ros(function):
    import rclpy

    rclpy.init()
    node = rclpy.create_node('taskweave_cli')
    try:
        return function(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _call(node, srv_type, name, request, timeout=10.0):
    import rclpy

    client = node.create_client(srv_type, name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise SystemExit(f'{name} not available - is the task handler running?')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        raise SystemExit(f'{name}: no response')
    return future.result()


def skills(node) -> int:
    from taskweave_interfaces.srv import ListSkills

    from . import names

    for info in sorted(_call(node, ListSkills, names.LIST_SKILLS, ListSkills.Request()).skills, key=lambda i: i.name):
        schema = json.loads(info.args_schema_json or '{}')
        args = ', '.join(f'{k}{"*" if k in schema.get("required", []) else ""}' for k in schema.get('properties', {}))
        print(f'{info.name:<18} {info.implementation:<7} ({args})  {info.description}')
    return 0


def run(node, mission: str, goal: str) -> int:
    from taskweave_interfaces.srv import StartMission

    from . import names

    response = _call(node, StartMission, names.START_MISSION, StartMission.Request(mission=mission, goal=goal))
    print(('started ' + response.plan_id) if response.accepted else 'rejected: ' + response.message)
    return 0 if response.accepted else 1


def trigger(node, command: str) -> int:
    from std_srvs.srv import Trigger

    from . import names

    response = _call(node, Trigger, getattr(names, command.upper()), Trigger.Request())
    print(f'{command}: {"ok" if response.success else "not possible"} (state: {response.message})')
    return 0 if response.success else 1


def teach(node, kind: str, name: str, tools: list[str] | None = None) -> int:
    from taskweave_interfaces.srv import TeachPoint

    from . import names

    request = TeachPoint.Request(name=name, kind=kind, tools=tools or [])
    response = _call(node, TeachPoint, names.TEACH_POINT, request)
    print(('saved ' + response.message + ' -> ' + response.file) if response.success else 'failed: ' + response.message)
    return 0 if response.success else 1


def status(node, follow: bool) -> int:
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile

    from taskweave_interfaces.msg import TaskStatus

    from . import names

    seen = []

    def show(msg):
        step = f'{msg.step.step_id} {msg.step.skill} "{msg.step.instruction}"' if msg.step_index >= 0 else '-'
        progress = f'{msg.step_progress:.0%}' if msg.step_progress >= 0 else ''
        line = (f'[{STATE_NAMES[msg.state]}] {msg.plan_id or "-"} cycle {msg.cycle} '
                f'step {msg.step_index + 1}/{msg.step_count}: {step} attempt {msg.attempt} '
                f'{msg.step_status} {progress} {msg.step_elapsed_s:.1f}s')
        if msg.last_error:
            line += f'\n    last error: {msg.last_error}'
        print(line, flush=True)
        seen.append(msg)

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TaskStatus, names.STATUS, show, qos)
    while rclpy.ok() and (follow or not seen):
        rclpy.spin_once(node, timeout_sec=0.5)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog='taskweave', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('check').add_argument('config_dir')
    sub.add_parser('skills')
    run_parser = sub.add_parser('run')
    run_parser.add_argument('mission')
    run_parser.add_argument('--goal', default='')
    sub.add_parser('status').add_argument('--follow', '-f', action='store_true')
    for command in ('pause', 'resume', 'abort'):
        sub.add_parser(command)
    teach_parser = sub.add_parser('teach')
    teach_parser.add_argument('kind', choices=['location', 'stance', 'point'])
    teach_parser.add_argument('name', help='location name, OBJECT_INSTANCE[/VARIANT] for a stance, '
                                           'OBJECT_INSTANCE/POINT for an object point')
    teach_parser.add_argument('--instance', action='store_true',
                              help='stance: store for this object instance only, not for its object type')
    teach_parser.add_argument('--tool', action='append', dest='tools',
                              help='point: store only this tool (repeatable; default: every tool)')
    describe_parser = sub.add_parser('describe')
    describe_parser.add_argument('config_dir', nargs='?')
    describe_parser.add_argument('robot', nargs='?')
    options = parser.parse_args(argv)
    if options.command == 'check':
        return check(options.config_dir)
    if options.command == 'skills':
        return _ros(skills)
    if options.command == 'run':
        return _ros(lambda node: run(node, options.mission, options.goal))
    if options.command == 'describe':
        if options.config_dir and options.robot:
            return describe_offline(options.config_dir, options.robot)
        return _ros(describe)
    if options.command == 'teach':
        kind = {'point': 'object_point', 'stance': 'stance_instance' if options.instance else 'stance'}.get(
            options.kind, options.kind)
        return _ros(lambda node: teach(node, kind, options.name, options.tools))
    if options.command == 'status':
        return _ros(lambda node: status(node, options.follow))
    return _ros(lambda node: trigger(node, options.command))


def entry() -> int:
    """Console entry point: config problems are reported as one line, not a traceback."""
    from .config import ConfigError

    try:
        return main()
    except ConfigError as error:
        print(f'config error: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(entry())
