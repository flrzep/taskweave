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
    taskweave smoke run MISSION [--reference FILE] [--save-reference FILE] [--hmi URL] [--cycles N]
                                          run a mission in the sim from a clean start (HMI recording API), check
                                          each step against a reference run (smoke.py)
    taskweave smoke compare REFERENCE RUN  the same check for a saved run, offline
    taskweave config migrate CELL_DIR [--robot-packs DIR] [--name NAME] [--dry-run]
                                          bring a cell pack (config directory) up to this taskweave's schema, e.g.
                                          split robot-intrinsic files out into robot packs (config/migrate.py)
    taskweave config migrate-skills CELL_DIR [--dry-run]
                                          only the last migration: move_to_pose, go_to_point, gripper and the old
                                          move_object become move_joints, move_gripper, grip, move_object, set_posture
                                          in missions, operator.yaml and skills/ (config/migrate_skills.py)
    taskweave config slim CELL_DIR [--dry-run]
                                          rewrite whole-file overrides of robot pack files as per-key overrides
                                          (only the keys that differ; config/slim.py)
    taskweave config init CELL_DIR --robot PACK [--name NAME]
                                          a new, empty, valid cell pack (config/packfile.py)
    taskweave config export CELL_DIR FILE.tar.gz
                                          the pack's files plus manifest.yaml (sha256 per file, git rev) in one file
    taskweave config import FILE.tar.gz CELL_DIR [--force]
                                          check the archive (paths, links, hashes, schema, taskweave check), then
                                          replace the folder (old one kept as .bak-<time>) or commit into a git repo
    taskweave config files CELL_DIR [ROBOT]
                                          where each robot file comes from (cell pack, override, robot pack)
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
    from .embodiment import profile_warnings
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

    for robot in store.robot_names():
        attempt(f'robots/{robot}', lambda r=robot: (store.robot(r), store.stances(r)))
        attempt(f'robots/{robot}/embodiment.yaml', lambda r=robot: _consistent(store, r))
        try:
            for warning in profile_warnings(store.robot(robot).data):
                print(f'  warn  robots/{robot}/profile.yaml: {warning}')
        except (ConfigError, KeyError, ValueError):
            pass    # the attempt above reported it
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


def packfile_command(options) -> int:
    from .config import packfile

    try:
        if options.action == 'init':
            print(f'created {packfile.init_cell(options.cell_dir, options.robot, options.name)}')
            return check(options.cell_dir)
        if options.action == 'export':
            manifest = packfile.export_cell(options.cell_dir, options.file)
            print(f'wrote {options.file}: {manifest["name"]}, {len(manifest["files"])} files, '
                  f'git rev {manifest["git_rev"] or "none"}')
            return 0
        report = packfile.import_cell(options.file, options.cell_dir, options.force)
    except packfile.PackfileError as error:
        print(f'error: {error}', file=sys.stderr)
        return 1
    where = {'swap': f'old pack kept as {report["backup"]}', 'git': f'commit {report["commit"] or "(no change)"}',
             'new': 'new folder'}[report['mode']]
    print(f'imported {report["name"]} ({report["files"]} files) into {options.cell_dir}: {where}')
    for warning in report['warnings']:
        print(f'warning: {warning}')
    return 0


def robot_files(config_dir: str, robot: str | None) -> int:
    """Each robot file: where it is read from and where an edit goes."""
    from .config import ConfigStore
    from .config.packs import CELL_ROBOT_FILES, ROBOT_PACK_FILES
    from .config.store import POINT_FILE_SCHEMAS

    store = ConfigStore(config_dir)
    for name in [robot] if robot else store.robot_names():
        print(f'{name}: ' + ('; '.join(p.label for p in store.robot_packs(name)) or 'no robot pack'))
        cell = store.root / 'robots' / name
        extra = sorted(p.name for p in cell.glob('*') if p.name not in ROBOT_PACK_FILES + CELL_ROBOT_FILES)
        for file in ROBOT_PACK_FILES + CELL_ROBOT_FILES + tuple(extra):
            source = store.robot_file_source(name, file)
            if source['source'] != 'missing':
                keyed = f'  [{source["override_count"]} key(s) differ]' if source['override'] == 'keys' else ''
                edited = file in POINT_FILE_SCHEMAS and source['save_path'] != source['path']  # HMI, teaching
                saves = f'  (edits -> {store.display_path(source["save_path"])})' if edited else ''
                print(f'  {file:<20} {source["source"]:<11} {store.display_path(source["path"])}{keyed}{saves}')
    return 0


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
    from . import smoke

    smoke.add_arguments(sub.add_parser('smoke', help='mission smoke test against a reference run'))
    config_sub = sub.add_parser('config', help='cell pack maintenance').add_subparsers(dest='action', required=True)
    migrate_parser = config_sub.add_parser('migrate', help='bring a cell pack up to the schema of this taskweave')
    migrate_parser.add_argument('cell_dir')
    migrate_parser.add_argument('--robot-packs', help='folder for robot packs of robot files no pack has yet')
    migrate_parser.add_argument('--name', help='cell pack name (default: the folder name)')
    migrate_parser.add_argument('--dry-run', action='store_true', help='only print what would be done')
    skills_parser = config_sub.add_parser('migrate-skills', help='cell pack schema 2 -> 3: the new skill model')
    skills_parser.add_argument('cell_dir')
    skills_parser.add_argument('--dry-run', action='store_true', help='only print what would be done')
    slim_parser = config_sub.add_parser('slim', help='whole-file overrides of robot pack files to per-key overrides')
    slim_parser.add_argument('cell_dir')
    slim_parser.add_argument('--dry-run', action='store_true', help='only print what would be done')
    init_parser = config_sub.add_parser('init', help='create an empty, valid cell pack')
    init_parser.add_argument('cell_dir')
    init_parser.add_argument('--robot', required=True, help='robot pack the cell uses')
    init_parser.add_argument('--name', help='cell pack name (default: the folder name)')
    export_parser = config_sub.add_parser('export', help='write a cell pack to one .tar.gz file')
    export_parser.add_argument('cell_dir')
    export_parser.add_argument('file')
    import_parser = config_sub.add_parser('import', help='take a cell pack .tar.gz into a folder (checked first)')
    import_parser.add_argument('file')
    import_parser.add_argument('cell_dir')
    import_parser.add_argument('--force', action='store_true', help='replace a non-empty folder (old pack kept)')
    files_parser = config_sub.add_parser('files', help='where each robot file comes from')
    files_parser.add_argument('cell_dir')
    files_parser.add_argument('robot', nargs='?')
    options = parser.parse_args(argv)
    if options.command == 'smoke':
        return smoke.main(options)
    if options.command == 'config':
        if options.action in ('init', 'export', 'import'):
            return packfile_command(options)
        if options.action == 'files':
            return robot_files(options.cell_dir, options.robot)
        if options.action == 'slim':
            from .config.slim import slim

            return slim(options.cell_dir, options.dry_run)
        if options.action == 'migrate-skills':
            from .config.migrate import migrate_skills_only

            return migrate_skills_only(options.cell_dir, options.dry_run)
        from .config.migrate import migrate

        return migrate(options.cell_dir, options.robot_packs, options.name, options.dry_run)
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
