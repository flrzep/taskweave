#!/usr/bin/env python3
"""Dump every resolvable target of a cell pack as a JSON table, and compare two tables (the equivalence check of the
targets migration, cell schema 3 -> 4).

    python3 tools/dump_targets.py CELL_DIR [--robot R ...] [--out FILE]      write the table (stdout without --out)
    python3 tools/dump_targets.py --compare BEFORE.json AFTER.json [--log MIGRATE_OUTPUT] [--tolerance 1e-6]

A table has the same shape for both layouts, so the dump of an unmigrated cell (old point files) can be compared with
the dump of the migrated copy. Sections: locations, stances (the cell-frame pose of each stance, per placed object
and variant), handled (where a turned object is handled from, for several robot positions), points (a hand point on a
placed object as the skills read it: tool poses and settings), contacts, held (held poses), poses (joint and tool
poses). A migrated cell is dumped twice: through the old readers (``legacy``, the shapes the old store returned) and
through ``ConfigStore.target`` (``native``); both must equal the dump of the old cell.

The compare takes the names the migration changed from its printed output (``--log``: lines ``rename ...`` and
``drop ...``): a renamed stance, held pose, pose or contact is compared under its new name, a dropped pose twin is not
missing. It exits 1 on any difference and prints the counts per section.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'taskweave_core'))

from taskweave_core.config import ConfigStore, Pose2D  # noqa: E402
from taskweave_core.config import targets as model  # noqa: E402

NEARS = [
    None,
    (7.0, 1.0, 0.0),
    (9.0, 1.0, 3.1416),
    (5.0, 3.0, 0.0),
    (0.0, 0.0, 0.0),
    (12.0, 1.5, 1.5708),
    (10.0, 0.0, -1.5708),
    (2.0, 0.0, 0.0),
    (8.0, 2.5, 3.1416),
    (6.8, 1.0, -1.5708),
]


def _pose2(pose) -> list[float]:
    return [float(pose.x), float(pose.y), float(pose.yaw)]


def _pose3(pose) -> dict:
    return {'xyz': [float(v) for v in pose.xyz], 'rpy': [float(v) for v in pose.rpy]}


def _axes(entry: dict) -> dict:
    """tcp, approach, closing with the axes made unit and square (what the pose is, however it was spelled)."""
    approach, closing = model.axes_from_rpy(model.rpy_from_axes(entry['approach'], entry['closing']))
    return {'tcp': [float(v) for v in entry['tcp']], 'approach': list(approach), 'closing': list(closing)}


def _near(near):
    return None if near is None else Pose2D(*near, 'cell')


def _point(point: dict) -> dict:
    """An object point as the skills read it: tool poses, one-hand poses, attributes, settings."""
    out: dict = {}
    for key, value in point.items():
        if key == 'alone':
            out[key] = None if not value else {k: _pose3(p) for k, p in value.items()}
        elif key.endswith(model.TOOL_SUFFIX):
            out[key] = _pose3(value)
        elif key == 'approach':
            out[key] = value
        elif value is not None:
            out[key] = value
    return out


def _robots(store: ConfigStore, wanted) -> list[str]:
    return list(wanted) if wanted else store.robot_names()


def _placed_pairs(store, robot: str, kind: str) -> list[str]:
    return store.point_names(robot)[kind]


def collect_legacy(store: ConfigStore, robots: list[str]) -> dict:
    """The table through the readers the old layout had (a migrated cell answers them from its targets)."""
    instances = store.object_instances(state=False)
    out: dict = {
        'locations': {n: _pose2(p) for n, p in store.locations().items()},
        'stances': {},
        'handled': {},
        'points': {},
        'contacts': {},
        'held': {},
        'poses': {},
    }
    near = [_near(n) for n in NEARS] + [Pose2D(p.x, p.y, p.yaw, 'cell') for p in store.locations().values()]
    for robot in robots:
        for inst in instances.values():
            for variant, offset in store.stance_variants(robot, inst).items():
                out['stances'][f'{robot}/{inst.name}/{variant}'] = _pose2(inst.pose.compose(offset))
            if store.stance_variants(robot, inst):
                out['handled'][f'{robot}/{inst.name}'] = [
                    _pose2(store.handling_instance(inst, robot, n).pose) for n in near
                ]
        for ref in _placed_pairs(store, robot, 'placed object points'):
            out['points'][f'{robot}/{ref}'] = _point(store.instance_point(ref, robot)[1])
        for ref in _placed_pairs(store, robot, 'placed object contacts'):
            if ref not in out['contacts']:
                out['contacts'][ref] = _pose3(store.instance_contact(ref)[1])
        for type_name, table in store.held_poses(robot).items():
            for name in table:
                entry = store.held_pose(robot, type_name, name)
                out['held'][f'{robot}/{type_name}/{name}'] = {
                    'joints': {j: float(v) for j, v in entry['joints'].items()},
                    'path': [_pose3(p) for p in entry['path']],
                }
        for name in store.joint_poses(robot):
            pose = store.named_pose(robot, name)
            out['poses'][f'{robot}/{name}'] = {
                'joints': {j: float(v) for j, v in pose['joints'].items()},
                'tools': {tool: _axes(entry) for tool, entry in pose['tools'].items()},
                'elbow': pose['elbow'],
            }
    return out


def collect_native(store: ConfigStore, robots: list[str]) -> dict:
    """The same table through ``ConfigStore.target`` (cell schema 4 only)."""
    from taskweave_core.config.limits import group_values

    out: dict = {'locations': {}, 'stances': {}, 'handled': {}, 'points': {}, 'contacts': {}, 'held': {}, 'poses': {}}
    instances = store.object_instances(state=False)
    for robot in robots:
        for ref, target in store.targets(robot).items():
            if '/' not in ref:
                if target.frame == 'cell':
                    out['locations'][ref] = _pose2(target.pose)
                elif target.kind == 'object':
                    joints = group_values(store, robot, target.joints) if target.joints else {}
                    out['held'][f'{robot}/{target.held_type}/{ref}'] = {
                        'joints': joints,
                        'path': [_pose3(p) for p in target.poses],
                    }
                elif target.kind in ('joints', 'hand'):
                    joints = group_values(store, robot, target.joints) if target.joints else {}
                    tools = {}
                    for tool, pose in target.tool_poses.items():
                        approach, closing = model.axes_from_rpy(pose.rpy)
                        tools[tool] = {'tcp': list(pose.xyz), 'approach': list(approach), 'closing': list(closing)}
                    out['poses'][f'{robot}/{ref}'] = {
                        'joints': joints,
                        'tools': tools,
                        'elbow': target.data.get('elbow'),
                    }
                continue
            inst, _, name = ref.partition('/')
            if target.kind == 'base':
                out['stances'][f'{robot}/{inst}/{name}'] = _pose2(target.pose)
            elif target.kind == 'hand':
                out['points'][f'{robot}/{ref}'] = _point(target.to_point())
            elif target.kind == 'object':
                out['contacts'].setdefault(ref, _pose3(target.pose))
        for inst in instances.values():
            if any(k.startswith(f'{robot}/{inst.name}/') for k in out['stances']):
                out['handled'][f'{robot}/{inst.name}'] = [
                    _pose2(store.handling_instance(inst, robot, _near(n)).pose) for n in NEARS
                ] + [
                    _pose2(store.handling_instance(inst, robot, Pose2D(p.x, p.y, p.yaw, 'cell')).pose)
                    for p in store.locations().values()
                ]
    return out


def dump(cell: str | Path, robots=None) -> dict:
    store = ConfigStore(cell, check_schema=False)
    names = _robots(store, robots)
    table = {
        'cell': str(cell),
        'targets_format': store.targets_format(),
        'robots': names,
        'instances': {n: i.object for n, i in store.object_instances(state=False).items()},
        'legacy': collect_legacy(store, names),
    }
    if store.targets_format():
        table['native'] = collect_native(store, names)
    return table


# ---- compare ------------------------------------------------------------------------------------------------------
def read_log(path: str | Path) -> dict:
    """The renames and drops the migration printed (``rename ...`` and ``drop ...`` lines)."""
    renames = {'stance': {}, 'held pose': {}, 'pose': {}, 'contact': {}}
    drops: list[str] = []
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        found = re.match(r'rename\s+stance robots/([\w\-]+)/stances\.yaml ([\w\-]+) ([\w\-]+) -> ([\w\-]+)', line)
        if found:
            robot, type_name, old, new = found.groups()
            renames['stance'][f'{robot}/{type_name}/{old}'] = new
            continue
        found = re.match(r'rename\s+held pose ([\w\-]+) ([\w\-]+) -> ([\w\-]+)', line)
        if found:
            robot, old, new = found.groups()
            renames['held pose'][f'{robot}/{old}'] = new
            continue
        found = re.match(r'rename\s+pose ([\w\-]+) ([\w\-]+) .* -> ([\w\-]+)$', line)
        if found:
            robot, old, new = found.groups()
            renames['pose'][f'{robot}/{old}'] = new
            continue
        found = re.match(r'rename\s+contact objects/([\w\-]+)\.yaml ([\w\-]+) -> ([\w\-]+)', line)
        if found:
            type_name, old, new = found.groups()
            renames['contact'][f'{type_name}/{old}'] = new
            continue
        found = re.match(r'drop\s+robots/([\w\-]+)/poses\.yaml: ([\w\-]+)', line)
        if found:
            drops.append(f'{found.group(1)}/{found.group(2)}')
    return {'renames': renames, 'drops': drops}


def _close(a, b, tolerance: float) -> bool:
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_close(a[k], b[k], tolerance) for k in a)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_close(x, y, tolerance) for x, y in zip(a, b, strict=True))
    if isinstance(a, bool) or isinstance(b, bool) or not isinstance(a, int | float) or not isinstance(b, int | float):
        return a == b
    if tolerance and abs(a - b) <= tolerance:
        return True
    return a == b or math.isclose(a, b, abs_tol=tolerance)


def _renamed(section: str, key: str, log: dict) -> str:
    """The key of an old entry after the migration's renames."""
    renames = log['renames']
    if section == 'stances':  # robot/inst/variant: renames are per robot and type, the type is not in the key
        robot, inst, variant = key.split('/')
        return (
            key
            if (robot, inst, variant) not in log.get('stance_keys', {})
            else f'{robot}/{inst}/{log["stance_keys"][(robot, inst, variant)]}'
        )
    if section == 'held':  # robot/type/name
        robot, type_name, name = key.split('/')
        new = renames['held pose'].get(f'{robot}/{name}')
        return f'{robot}/{type_name}/{new}' if new else key
    if section == 'poses':
        robot, name = key.split('/')
        new = renames['pose'].get(key)
        return f'{robot}/{new}' if new else key
    return key


def _stance_keys(log: dict, instances: dict, before: dict) -> dict:
    """``{(robot, inst, variant): new}`` from the per-type stance renames and the placed objects' types."""
    out = {}
    for key, new in log['renames']['stance'].items():
        robot, type_name, old = key.split('/')
        for inst, kind in instances.items():
            if kind == type_name:
                out[(robot, inst, old)] = new
    return out


def compare(before: dict, after: dict, log: dict | None, tolerance: float, say=print) -> bool:
    """True if ``after`` (a migrated cell, legacy and native views) equals ``before`` modulo the logged renames."""
    log = log or {'renames': {'stance': {}, 'held pose': {}, 'pose': {}, 'contact': {}}, 'drops': []}
    types = before['instances']
    log['stance_keys'] = _stance_keys(log, types, before)
    ok = True
    views = [('legacy', after['legacy'])] + ([('native', after['native'])] if 'native' in after else [])
    for view, table in views:
        say(f'== {view} view of the migrated cell against the old cell')
        for section in ('locations', 'stances', 'handled', 'points', 'contacts', 'held', 'poses'):
            old = {}
            for key, value in before['legacy'][section].items():
                new_key = _renamed(section, key, log)
                if section == 'contacts':
                    type_name = types.get(key.split('/')[0])
                    new_name = log['renames']['contact'].get(f'{type_name}/{key.split("/")[1]}')
                    new_key = f'{key.split("/")[0]}/{new_name}' if new_name else key
                if section == 'handled':  # keyed by robot/inst: values are poses of handled turns
                    new_key = key
                old[new_key] = value
            new = dict(table[section])
            missing = sorted(set(old) - set(new))
            extra = sorted(set(new) - set(old))
            dropped = [k for k in missing if section == 'poses' and k in log['drops']]
            missing = [k for k in missing if k not in dropped]
            differ = sorted(k for k in set(old) & set(new) if not _close(old[k], new[k], tolerance))
            equal = len(set(old) & set(new)) - len(differ)
            say(
                f'{section:10} equal {equal:4}  differ {len(differ)}  only before {len(missing)}  only after '
                f'{len(extra)}' + (f'  dropped by the migration {len(dropped)}' if dropped else '')
            )
            for label, keys in (('differ', differ), ('only before', missing), ('only after', extra)):
                for key in keys[:20]:
                    say(f'    {label}: {key}')
                    if label == 'differ':
                        say(f'        before {json.dumps(old[key])[:300]}')
                        say(f'        after  {json.dumps(new[key])[:300]}')
            ok = ok and not (differ or missing or extra)
    return ok


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('cell', nargs='?')
    parser.add_argument('--robot', action='append', help='only these robots (default: every robot of the cell)')
    parser.add_argument('--out')
    parser.add_argument('--compare', nargs=2, metavar=('BEFORE', 'AFTER'))
    parser.add_argument('--log', help='the migration output (renames and drops are read from it)')
    parser.add_argument('--tolerance', type=float, default=1e-6)
    options = parser.parse_args(argv)
    if options.compare:
        before, after = (json.loads(Path(p).read_text(encoding='utf-8')) for p in options.compare)
        log = read_log(options.log) if options.log else None
        same = compare(before, after, log, options.tolerance)
        print('EQUAL' if same else 'DIFFERENT')
        return 0 if same else 1
    if not options.cell:
        parser.error('a cell directory (or --compare)')
    text = json.dumps(dump(options.cell, options.robot), indent=1, sort_keys=False)
    if options.out:
        Path(options.out).write_text(text, encoding='utf-8')
    else:
        print(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
