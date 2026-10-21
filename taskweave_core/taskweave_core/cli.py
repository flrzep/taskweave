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
    taskweave state show [CELL_DIR]       the run-time object state: what is taken, stack counts, placed objects
    taskweave state clear [CELL_DIR]      forget it (objects back as in cell/objects.yaml); running processes see it
                                          on their next read (the store re-reads by file stamp), no reload needed
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
    taskweave config files CELL_DIR [ROBOT] [--json]
                                          where each robot file comes from (cell pack, override, robot pack); --json:
                                          the catalogue of every file (kind, schema, reload, hash; config/catalogue.py)
    taskweave config schema KIND_OR_PATH [--pack robot:NAME]
                                          the JSON Schema of a file kind (object_type, mission, profile ...)
    taskweave config validate PATH [--cell CELL_DIR] [--pack P] [--json]
                                          check one file as it is on disk (schema and cross-file checks)
    taskweave config save PATH --from FILE [--base HASH] [--message M] [--cell CELL_DIR] [--pack P]
                                          validate and write a file (conflict: exit 3), commit it in a git repo
    taskweave config history PATH [--limit N] [--cell CELL_DIR] [--pack P]
                                          the commits of a file
    taskweave config restore PATH REV [--base HASH] [--cell CELL_DIR] [--pack P]
                                          save a file as it was at a commit
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
    """Check every file of the cell pack and its robot packs (config/check.py); the exit code is 1 on an error."""
    from .config import ConfigStore
    from .config.check import checked_files

    errors = 0
    for entry, problems in checked_files(ConfigStore(config_dir)):
        label = entry['path'] if entry['pack'] == 'cell' else f'{entry["pack"]}: {entry["path"]}'
        failed = [p for p in problems if p['severity'] == 'error']
        errors += bool(failed)
        print(f'  {"FAIL" if failed else "ok  "}  {label}' + (f': {_say(failed[0])}' if failed else ''))
        for problem in failed[1:] + [p for p in problems if p['severity'] != 'error']:
            print(f'  {"FAIL" if problem["severity"] == "error" else "warn"}  {label}: {_say(problem)}')
    print(f'{errors} file(s) with problems' if errors else 'config ok (skill arguments are checked against a running '
                                                          'catalog)')
    return 1 if errors else 0


def _say(problem: dict) -> str:
    place = f'{problem["where"]}: ' if problem['where'] else ''
    return place + problem['message']


def config_files_json(config_dir: str) -> int:
    from .config import ConfigStore

    print(json.dumps(ConfigStore(config_dir).files(), indent=2))
    return 0


def config_schema(target: str, pack: str) -> int:
    from .config.catalogue import json_schema

    try:
        print(json.dumps(json_schema(target, pack), indent=2))
    except KeyError as error:
        print(error.args[0], file=sys.stderr)
        return 2
    return 0


def _config_target(path: str, cell: str | None, pack: str):
    """(cell pack folder, path relative to its pack, pack) of a file or of a path relative to ``pack``; None and a
    message on stderr if it cannot be told."""
    from pathlib import Path

    from .config.packs import CELL_SCHEMA, CELL_SCHEMA_OLDEST, cell_schema

    target = Path(path).expanduser().resolve()
    if target.is_file():
        folder = next((p for p in target.parents if (p / 'pack.yaml').is_file()), None)
        if folder is None:
            print(f'{path} is in no pack (no pack.yaml above it)', file=sys.stderr)
            return None
        if CELL_SCHEMA_OLDEST <= cell_schema(folder) <= CELL_SCHEMA:
            return str(folder), target.relative_to(folder).as_posix(), 'cell'
        if cell:
            return cell, target.relative_to(folder).as_posix(), f'robot:{folder.name}'
        print(f'{path} is in a robot pack: give the cell pack with --cell', file=sys.stderr)
        return None
    if cell:
        return cell, path, pack
    print(f'{path} is not a file; for a path inside a pack give --cell', file=sys.stderr)
    return None


def config_validate(path: str, cell: str | None, pack: str, as_json: bool) -> int:
    """Validate one file as it is on disk. ``path`` is a file (inside a cell pack, or with ``cell`` inside a robot
    pack) or, with ``cell``, a path relative to ``pack``."""
    from .config import ConfigStore

    found = _config_target(path, cell, pack)
    if found is None:
        return 2
    cell, relative, pack = found
    store = ConfigStore(cell)
    problems = store.validate_file(relative, None, pack)
    if as_json:
        print(json.dumps(problems, indent=2))
    else:
        for problem in problems:
            line = f':{problem["line"]}' if problem['line'] else ''
            print(f'{problem["severity"]:<7} {relative}{line} {_say(problem)}')
        print(f'{relative}: ' + ('ok' if not problems else f'{len(problems)} problem(s)'))
    return 1 if any(p['severity'] == 'error' for p in problems) else 0


def config_write(options) -> int:
    """config save | history | restore (config/save.py): through core, validated, committed when in a repo."""
    from .config import ConfigStore, SaveConflict, SaveError

    found = _config_target(options.path, options.cell, options.pack)
    if found is None:
        return 2
    cell, relative, pack = found
    store = ConfigStore(cell)
    try:
        if options.action == 'history':
            for row in store.history(relative, pack, options.limit):
                print(f'{row["rev"][:8]}  {row["time"]}  {row["author"]}  {row["message"].splitlines()[0]}')
            return 0
        if options.action == 'save':
            text = sys.stdin.read() if options.source == '-' else open(options.source, encoding='utf-8').read()
            result = store.save(relative, text, pack, options.base, options.message)
        else:
            result = store.restore(relative, options.rev, pack, options.base)
    except SaveConflict as error:
        print(f'conflict: {error} (the file is now {error.current_hash})', file=sys.stderr)
        return 3
    except SaveError as error:
        for problem in error.problems:
            line = f':{problem["line"]}' if problem['line'] else ''
            print(f'{problem["severity"]:<7} {problem["path"]}{line} {_say(problem)}', file=sys.stderr)
        print('nothing written', file=sys.stderr)
        return 1
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    for warning in result['warnings']:
        print(f'warning {warning if isinstance(warning, str) else warning["message"]}')
    print(f'{relative}: saved, hash {result["hash"]}, rev {result["rev"] or "-"}, '
          f'reload: {", ".join(result["reload"])}')
    return 0


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


RELOAD_TYPE = 'taskweave_interfaces/srv/ReloadConfig'


def reload_services(node, wait_s: float = 3.0) -> list[str]:
    """The reload_config services in the graph (the task handler, every skill server); waits for discovery."""
    import time

    deadline = time.monotonic() + wait_s
    while True:
        found = sorted(name for name, types in node.get_service_names_and_types() if RELOAD_TYPE in types)
        if found or time.monotonic() >= deadline:
            return found
        time.sleep(0.1)


def reload_all(node, paths=(), timeout: float = 10.0) -> list[dict]:
    """Call every reload_config service: ``[{node, service, status (reloaded | busy | failed | unreachable), message,
    reloaded, needs_restart: [{path, what}]}]``."""
    from taskweave_interfaces.srv import ReloadConfig

    results = []
    for service in reload_services(node):
        try:
            answer = _call(node, ReloadConfig, service, ReloadConfig.Request(paths=list(paths)), timeout)
        except SystemExit as error:
            results.append({'node': service.rsplit('/', 1)[0], 'service': service, 'status': 'unreachable',
                            'message': str(error), 'reloaded': [], 'needs_restart': []})
            continue
        results.append({'node': answer.node, 'service': service, 'status': answer.status, 'message': answer.message,
                        'reloaded': list(answer.reloaded),
                        'needs_restart': [{'path': r.path, 'what': r.what} for r in answer.needs_restart]})
    return results


def print_reload(results: list[dict]) -> int:
    """What each process reloaded and what needs a restart; 0 if every process answered reloaded, 1 if one was busy,
    failed or unreachable (or none was found)."""
    for each in results:
        print(f'{each["node"]}: {each["status"]}' + (f' ({each["message"]})' if each['message'] else ''))
        for path in each['reloaded']:
            print(f'  reloaded      {path}')
        for item in each['needs_restart']:
            print(f'  needs restart {item["path"]}: {item["what"]}')
    if not results:
        print('no reload_config service found - is the task stack running?')
    return 0 if results and all(each['status'] == 'reloaded' for each in results) else 1


def config_reload(hmi: str | None, paths) -> int:
    """``taskweave config reload``: ask every process over ROS, or the HMI (POST <url>/api/config/reload with
    ``{"paths": [...]}``, answering ``{"nodes": [...]}`` as ``reload_all`` returns them)."""
    if hmi is None:
        return print_reload(_ros(lambda node: reload_all(node, paths)))
    import urllib.request

    body = json.dumps({'paths': list(paths)}).encode()
    request = urllib.request.Request(hmi.rstrip('/') + '/api/config/reload', body,
                                     {'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=60) as reply:
        return print_reload(json.load(reply)['nodes'])


def state_show(cell_dir: str) -> int:
    """The object state over cell/objects.yaml, one line per changed object."""
    from .config import ConfigStore

    store = ConfigStore(cell_dir)
    state = store.object_state()
    print(f'object state: {store.state_path()}' if state else
          f'no object state ({store.state_path()}): objects as in cell/objects.yaml')
    for name, change in sorted(state.items()):
        if change.get('gone'):
            print(f'  {name}: taken')
        elif 'object' in change:
            print(f'  {name}: placed ({change["object"]} at x={change["x"]}, y={change["y"]}, yaw={change["yaw"]})')
        else:
            print(f'  {name}: {change["count"]} left')
    return 0


def state_clear(cell_dir: str) -> int:
    from .config import ConfigStore

    store = ConfigStore(cell_dir)
    existed = store.state_path().exists()
    store.clear_object_state()
    print(f'object state cleared ({store.state_path()})' if existed else 'no object state to clear')
    return 0


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
    state_parser = sub.add_parser('state', help='run-time object state').add_subparsers(dest='action', required=True)
    for action, text in (('show', 'what is taken, stack counts, placed objects'), ('clear', 'put every object back')):
        state_parser.add_parser(action, help=text).add_argument('cell_dir', nargs='?', default='.')
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
    reload_parser = config_sub.add_parser('reload', help='make a running task stack re-read its config (no restart)')
    reload_parser.add_argument('paths', nargs='*', help='only these files (relative to their pack)')
    where = reload_parser.add_mutually_exclusive_group()
    where.add_argument('--ros', action='store_true', help='call the reload_config service of every process (default)')
    where.add_argument('--hmi', metavar='URL', help='ask the HMI to do it (http://host:8090)')
    files_parser = config_sub.add_parser('files', help='where each robot file comes from')
    files_parser.add_argument('cell_dir')
    files_parser.add_argument('robot', nargs='?')
    files_parser.add_argument('--json', action='store_true',
                              help='the catalogue of every file of the cell pack and its robot packs instead')
    schema_parser = config_sub.add_parser('schema', help='JSON Schema of a file kind or path')
    schema_parser.add_argument('target', help='a kind (object_type, mission, profile ...) or a path in a pack')
    schema_parser.add_argument('--pack', default='cell', help="for a path: 'cell' (default) or robot:<name>")
    validate_parser = config_sub.add_parser('validate', help='check one file as it is on disk')
    validate_parser.add_argument('path')
    validate_parser.add_argument('--cell', help='the cell pack (for a robot pack file, or a path relative to it)')
    validate_parser.add_argument('--pack', default='cell', help="for a relative path: 'cell' or robot:<name>")
    validate_parser.add_argument('--json', action='store_true')
    save_parser = config_sub.add_parser('save', help='validate and write a file, commit it when in a git repo')
    save_parser.add_argument('path')
    save_parser.add_argument('--from', dest='source', required=True, metavar='FILE', help="the new text ('-': stdin)")
    save_parser.add_argument('--base', help='the hash the text is based on (config files); needed to replace a file')
    save_parser.add_argument('--message', '-m')
    history_parser = config_sub.add_parser('history', help='the commits that changed a file')
    history_parser.add_argument('path')
    history_parser.add_argument('--limit', type=int, default=50)
    restore_parser = config_sub.add_parser('restore', help='save a file as it was at a commit')
    restore_parser.add_argument('path')
    restore_parser.add_argument('rev')
    restore_parser.add_argument('--base', help='the hash of the file now')
    for each in (save_parser, history_parser, restore_parser):
        each.add_argument('--cell', help='the cell pack (for a robot pack file, or a path relative to it)')
        each.add_argument('--pack', default='cell', help="for a relative path: 'cell' or robot:<name>")
    options = parser.parse_args(argv)
    if options.command == 'smoke':
        return smoke.main(options)
    if options.command == 'config':
        if options.action in ('init', 'export', 'import'):
            return packfile_command(options)
        if options.action == 'reload':
            return config_reload(options.hmi, options.paths)
        if options.action == 'schema':
            return config_schema(options.target, options.pack)
        if options.action in ('save', 'history', 'restore'):
            return config_write(options)
        if options.action == 'validate':
            return config_validate(options.path, options.cell, options.pack, options.json)
        if options.action == 'files':
            if options.json:
                return config_files_json(options.cell_dir)
            return robot_files(options.cell_dir, options.robot)
        if options.action == 'slim':
            from .config.slim import slim

            return slim(options.cell_dir, options.dry_run)
        if options.action == 'migrate-skills':
            from .config.migrate import migrate_skills_only

            return migrate_skills_only(options.cell_dir, options.dry_run)
        from .config.migrate import migrate

        return migrate(options.cell_dir, options.robot_packs, options.name, options.dry_run)
    if options.command == 'state':
        return (state_show if options.action == 'show' else state_clear)(options.cell_dir)
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
