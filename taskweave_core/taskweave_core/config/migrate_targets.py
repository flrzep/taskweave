"""Cell pack migration 3 -> 4: targets (``taskweave config migrate CELL_DIR`` chains it after 1 -> 2 and 2 -> 3;
design: plans/p6-targets-move.md section 6). Every place the robot goes to is a target now, in four files; the six
motion skills are one skill, ``move``. Rewritten with ruamel (comments and key order kept, entries changed in place):

  cell/locations.yaml            -> cell/targets.yaml          each entry ``moves: base``, ``frame`` dropped
  objects/<type>.yaml            ``points`` and ``contacts`` -> ``targets`` (``moves: hand`` / ``moves: object``); a
                                 contact a mission steps onto gets the ``reference`` the steps named
  robots/<r>/stances.yaml        -> robots/<r>/objects/<type>.yaml  ``targets`` (``moves: base``), ``instances``,
                                 ``handled_from`` = the old first variant; with robots/<r>/object_points.yaml the
                                 robot's settings (elbow, grip_m) on the type's hand targets
  robots/<r>/poses.yaml, held_poses.yaml
                                 -> robots/<r>/targets.yaml    joint poses ``moves: joints`` with joint group values,
                                 tool poses ``moves: hand`` (``<tool>_tool: {xyz, rpy}``), held poses ``moves: object``
                                 with their ``type``; a keyed override (``~override: keys``) stays keyed
  skills/<six skills>.yaml       -> skills/move/{base,straight,hand,object,posture,joints}.yaml (the same bytes) and
                                 a new skills/move.yaml
  robots/<r>/skill_overrides.yaml  keys renamed to the module names (move.hand ...)
  missions/*.yaml, operator.yaml every motion step is ``move`` (migrate_move.py); step ids never change
  robot packs                    a writable robot pack at schema 1 (system.yaml robot_packs, TASKWEAVE_ROBOT_PACKS, the
                                 --robot-packs folder): poses.yaml -> targets.yaml, schema_version 2; an installed
                                 (read-only) one is only reported. ``taskweave config migrate PACK_DIR`` migrates a
                                 robot pack (or a folder of them) on its own.

Names that clash once the kind prefix is gone are renamed and printed (``rename``): a stance named like an object
target of its type gets ``_stance`` (knob -> knob_stance), a held pose named like a plain target gets ``_held``
(set_down -> set_down_held), a pose named like a location ``_pose``, a contact named like a point ``_contact``. Steps
follow. Nothing is written until every file is built; each is written aside and renamed. What cannot be mapped
cleanly is flagged (printed again at the end); what cannot be mapped at all stops the migration with the file or step
named (RuntimeError, nothing changed).
"""

from __future__ import annotations

import fnmatch
import os
import re
from collections.abc import Mapping
from pathlib import Path

import yaml

from . import overlay, packs
from . import targets as model
from .migrate_move import StepConverter, Unresolved
from .migrate_skills import (
    Plan,
    _dump,
    _flag,
    _flow_of,
    _map,
    _placeholder,
    _read,
    _seq,
    _show,
    apply,
)

COMMAND = 'taskweave config migrate'
SKILL_FILES = {
    'navigate_to': 'base',
    'drive_straight': 'straight',
    'move_gripper': 'hand',
    'move_object': 'object',
    'set_posture': 'posture',
    'move_joints': 'joints',
}
MODULE_NAMES = {old: f'move.{new}' for old, new in SKILL_FILES.items()}
MOVE_YAML = (
    '# the move skill: one skill for every motion; its modules (base, straight, hand, object, posture, joints)\n'
    '# keep their parameters in skills/move/<module>.yaml (taskweave config migrate, schema 4)\n'
    'implementations:\n'
    '  - {name: dispatch, type: script, class: "taskweave_skills.move:Move"}\n'
)
OLD_WORDS = re.compile(
    r'#.*(\blocations\.yaml|\bstances\.yaml|\bheld_poses\.yaml|\bobject_points\.yaml|\bposes\.yaml'
    r'|\bstance:|\bnavigate_to\b|\bdrive_straight\b|\bmove_gripper\b|\bmove_object\b|\bset_posture\b'
    r'|\bmove_joints\b|teach (location|stance|point)\b|\bpose:)'
)
AXIS_TOLERANCE = 5e-7  # the rebuilt tool axes may differ from the old ones by this much (design: within 1e-6)
MIN_DECIMALS, MAX_DECIMALS = 4, 10


# ---- ruamel helpers ---------------------------------------------------------------------------------------------
def _carry(old, old_key, new, new_key=None) -> None:
    """The comment entry of ``old[old_key]`` onto ``new[new_key]``."""
    item = getattr(old, 'ca', None) and old.ca.items.get(old_key)
    if item:
        new.ca.items[new_key if new_key is not None else old_key] = list(item)


def _comment_lines(token) -> list[str]:
    """The text of a comment token (or list of tokens) as plain lines without the leading ``#``."""
    if token is None:
        return []
    tokens = token if isinstance(token, list) else [token]
    lines = []
    for each in tokens:
        if each is None:
            continue
        for line in str(each.value).splitlines():
            line = line.strip()
            if line.startswith('#'):
                lines.append(line.lstrip('#').strip())
    return lines


def _key_comment(parent, key, child=None) -> list[str]:
    """The comment after ``key:`` of a block mapping (on the key's line, or the lines that follow it)."""
    item = parent.ca.items.get(key)
    lines = _comment_lines(item[2]) if item and len(item) > 2 else []
    if not lines and child is not None and getattr(child, 'ca', None) and child.ca.comment:
        lines = _comment_lines(child.ca.comment[0])
    return lines


def _rename_key(mapping, old: str, new: str) -> None:
    """``old`` becomes ``new`` at the same place with its value and comments."""
    position = list(mapping).index(old)
    value = mapping[old]
    item = mapping.ca.items.get(old)
    del mapping[old]
    mapping.insert(position, new, value)
    if item:
        mapping.ca.items[new] = item
        mapping.ca.items.pop(old, None)


def _round_float(value: float, decimals: int) -> float:
    rounded = round(value, decimals)
    return 0.0 if rounded == 0 else rounded


def _plain(value):
    from .migrate_skills import _plain as plain

    return plain(value)


def _is_marker(name) -> bool:
    return str(name) == overlay.MARKER


def _set_start_comment(document, lines: list[str]) -> None:
    if lines:
        document.yaml_set_start_comment('\n'.join(lines))


def _comment_before(mapping, key, lines: list[str], indent: int = 0) -> None:
    if lines:
        mapping.yaml_set_comment_before_after_key(key, before='\n'.join(lines), indent=indent)


# ---- tool axes -> roll pitch yaw -------------------------------------------------------------------------------
def tool_rpy(approach, closing) -> list[float]:
    """The roll-pitch-yaw of a tool pose given as approach and closing axes, with as few decimals as keep the rebuilt
    axes within AXIS_TOLERANCE of the old ones."""
    approach, closing = [float(v) for v in approach], [float(v) for v in closing]
    exact = model.rpy_from_axes(approach, closing)
    want = model.axes_from_rpy(exact)
    for decimals in range(MIN_DECIMALS, MAX_DECIMALS + 1):
        rpy = [_round_float(v, decimals) for v in exact]
        have = model.axes_from_rpy(rpy)
        if (
            max(abs(a - b) for pair in zip(have, want, strict=True) for a, b in zip(*pair, strict=True))
            <= AXIS_TOLERANCE
        ):
            return rpy
    return [_round_float(v, MAX_DECIMALS) for v in exact]


# ---- joint groups -----------------------------------------------------------------------------------------------
def group_values(joints: Mapping, groups: dict[str, list[str]], label: str) -> list[tuple[str, object]]:
    """``{joint: value}`` -> ``[(group, value)]``: a group of one joint takes the number, a group of several a list in
    the group's order (``None`` where the pose leaves the joint). A joint in no group stops the migration."""
    left = dict(joints)
    result: list[tuple[str, object]] = []
    for group, members in groups.items():
        if not any(joint in left for joint in members):
            continue
        values = [left.pop(joint, None) for joint in members]
        result.append((group, values[0] if len(members) == 1 else _seq(values)))
    if left:
        raise RuntimeError(
            f'{label}: joint {", ".join(left)} is in no joint group of the robot (profile.yaml '
            'joints.groups): add the group, then run taskweave config migrate again'
        )
    return result


def joints_node(joints: Mapping, groups: dict, label: str, flow: bool):
    return _map(dict(group_values(joints, groups, label)), flow)


# ---- poses and held poses ---------------------------------------------------------------------------------------
def is_tool_pose(entry) -> bool:
    return isinstance(entry, Mapping) and 'tools' in entry


def convert_pose(entry, groups: dict, label: str, flags: list[str]) -> None:
    """One old pose entry (a ruamel mapping) becomes a target, in place."""
    flow = _flow_of(entry)
    old = dict(entry)
    entry.clear()
    entry['moves'] = 'joints'
    if not is_tool_pose(old):
        entry['joints'] = joints_node(old, groups, label, flow)
        return
    entry['moves'] = 'hand'
    for tool, pose in old['tools'].items():
        entry[f'{tool}{model.TOOL_SUFFIX}'] = _map(
            {'xyz': pose['tcp'], 'rpy': _seq(tool_rpy(pose['approach'], pose['closing']))}, True
        )
    if old.get('elbow'):
        entry['elbow'] = old['elbow']
    if old.get('joints'):
        entry['joints'] = joints_node(old['joints'], groups, label, flow)
    extra = [key for key in old if key not in ('tools', 'joints', 'elbow')]
    for key in extra:
        entry[key] = old[key]
        flags.append(f'{label}: key {key} kept as it is')


def twin_names(doc) -> dict[str, str]:
    """``{twin: parent}`` for each ``<pose>_joints`` the poses migration of skills made: the joint twin of a pose that
    has joints and tools, same joints as its parent's."""
    twins = {}
    for name, entry in doc.items():
        if _is_marker(name) or not str(name).endswith('_joints') or isinstance(entry, str):
            continue
        parent = str(name)[: -len('_joints')]
        other = doc.get(parent)
        if (
            is_tool_pose(other)
            and other.get('joints')
            and not is_tool_pose(entry)
            and _plain(dict(entry)) == _plain(dict(other['joints']))
        ):
            twins[str(name)] = parent
    return twins


def convert_poses_doc(
    doc, groups: dict, label: str, flags: list[str], say, used: set[str] = frozenset(), renames: Mapping | None = None
) -> dict[str, str]:
    """A poses.yaml document becomes targets (in place): every pose converted, twins dropped when unused, poses renamed
    per ``renames``. Returns the dropped twins ``{twin: parent}``."""
    twins = {name: parent for name, parent in twin_names(doc).items() if name not in used}
    for name in twins:
        say(f'drop    {label}: {name} (the joints of {twins[name]}, nothing uses it)')
        del doc[name]
    for name in list(doc):
        if _is_marker(name) or isinstance(doc[name], str) or not isinstance(doc[name], Mapping):
            continue
        convert_pose(doc[name], groups, f'{label} pose {name}', flags)
    for name, new in (renames or {}).items():
        if name in doc:
            _rename_key(doc, name, new)
    return twins


def convert_held(entry, type_name: str | None, groups: dict, label: str) -> None:
    """One held pose entry (``{joints, path}``) becomes an object target, in place."""
    flow = _flow_of(entry)
    old = dict(entry)
    entry.clear()
    entry['moves'] = 'object'
    if type_name:
        entry['type'] = type_name
    if old.get('joints'):
        entry['joints'] = joints_node(old['joints'], groups, label, flow)
    entry['path'] = old.get('path', _seq([]))
    for key in old:
        if key not in ('joints', 'path'):
            entry[key] = old[key]


# ---- the cell: what the migration needs to know -------------------------------------------------------------------
def _load(path: Path):
    return yaml.safe_load(path.read_text(encoding='utf-8')) if path.is_file() else None


def _old_view(data: dict) -> dict:
    """An object type file as the old layout would have it: a file that is migrated already (a run that stopped before
    its last step) gives its hand targets as ``points`` and its object targets as ``contacts`` again."""
    targets = data.get('targets')
    if 'points' in data or 'contacts' in data or not isinstance(targets, dict):
        return data
    return {
        **data,
        'points': {n: e for n, e in targets.items() if (e or {}).get('moves') == 'hand'},
        'contacts': {n: e for n, e in targets.items() if (e or {}).get('moves') == 'object'},
    }


class Book:
    """The cell as the old layout says it: placed objects, object types, stances, poses, held poses, contacts, and the
    names every old thing gets in the new layout. Built once; the file writers and the step converters read it."""

    def __init__(self, cell: Path, say, flag):
        from .store import ConfigStore

        self.cell, self.say, self.flag = cell, say, flag
        self.store = ConfigStore(cell, check_schema=False)
        self.system_robot = (self.store.system() or {}).get('robot')
        self.missions = {p.stem: _load(p) or {} for p in sorted((cell / 'missions').glob('*.yaml'))}
        self.robots = self._robots()
        self.default_robot = self.system_robot or (self.robots[0] if len(self.robots) == 1 else None)
        objects = _load(cell / 'cell' / 'objects.yaml') or {}
        self.instances = {str(n): str(e['object']) for n, e in objects.items() if isinstance(e, Mapping)}
        self.types = {p.stem: _old_view(_load(p) or {}) for p in sorted((cell / 'objects').glob('*.yaml'))}
        self.locations = set(_load(cell / 'cell' / 'locations.yaml') or {})
        self.stances = {r: (_load(cell / 'robots' / r / 'stances.yaml') or {}) for r in self.robots}
        self.settings = {r: (_load(cell / 'robots' / r / 'object_points.yaml') or {}) for r in self.robots}
        self.held = {r: (_load(cell / 'robots' / r / 'held_poses.yaml') or {}) for r in self.robots}
        self.poses = {r: (_load(cell / 'robots' / r / 'poses.yaml') or {}) for r in self.robots}
        self.groups = {r: self._groups(r) for r in self.robots}
        self.used_poses = self._used_poses()
        self._runtime: dict[str, str | None] = {}
        self.contact_new: dict[tuple[str, str], str] = {}
        self.stance_new: dict[tuple[str, str, str], str] = {}
        self.pose_new: dict[tuple[str, str], str] = {}
        self.held_new: dict[tuple[str, str], str] = {}
        self.contact_ref: dict[tuple[str, str], str] = {}
        self.twins: dict[str, dict[str, str]] = {}
        self._names()

    # -- discovery ------------------------------------------------------------------------------------------------
    def _robots(self) -> list[str]:
        found = {p.name for p in (self.cell / 'robots').glob('*') if p.is_dir()}
        if self.system_robot:
            found.add(self.system_robot)
        found |= {str(m['robot']) for m in self.missions.values() if m.get('robot')}
        return sorted(found)

    def _groups(self, robot: str) -> dict[str, list[str]]:
        from .limits import joint_groups
        from .store import ConfigError

        try:
            return joint_groups(self.store, robot)
        except (ConfigError, KeyError, OSError, ValueError):
            return {}

    def pack_names(self, robot: str) -> dict[str, dict]:
        """The poses (old layout) or targets (new) the robot packs give: ``{name: entry}`` of the first pack."""
        from .store import ConfigError

        try:
            for pack in self.store.robot_packs(robot):
                for file in ('targets.yaml', 'poses.yaml'):
                    if (pack.path / file).is_file():
                        data = _load(pack.path / file) or {}
                        return {k: v for k, v in data.items() if not _is_marker(k)}
        except ConfigError:
            pass
        return {}

    @staticmethod
    def _steps(mission: Mapping):
        return [s for s in mission.get('steps', []) if isinstance(s, Mapping)]

    def _used_poses(self) -> set[str]:
        used: set[str] = set()
        actions = ((_load(self.cell / 'operator.yaml') or {}).get('joy') or {}).get('actions') or []
        stepish = [(a.get('skill'), a.get('args')) for a in actions if isinstance(a, Mapping)]
        for mission in self.missions.values():
            stepish += [(s.get('skill'), s.get('args')) for s in self._steps(mission)]
        for _, args in stepish:
            if not isinstance(args, Mapping):
                continue
            if isinstance(args.get('pose'), str):
                used.add(args['pose'])
            to = args.get('to')
            if isinstance(to, Mapping) and isinstance(to.get('pose'), str):
                used.add(to['pose'])
            via = (args.get('motion') or {}).get('via') if isinstance(args.get('motion'), Mapping) else None
            for item in via if isinstance(via, list) else []:
                if isinstance(item, str):
                    used.add(item.removeprefix('pose:'))
        return used

    # -- names ----------------------------------------------------------------------------------------------------
    def type_targets(self, type_name: str) -> dict[str, str]:
        """The robot-independent targets of a type in the old layout: points (hand) and contacts (object), by their
        new names."""
        data = self.types.get(type_name, {})
        found = {name: 'hand' for name in data.get('points', {})}
        found.update({self.contact_new.get((type_name, name), name): 'object' for name in data.get('contacts', {})})
        return found

    def _names(self) -> None:
        for type_name, data in self.types.items():  # a contact named like a point of its type
            for name in data.get('contacts') or {}:
                if name in (data.get('points') or {}):
                    new = f'{name}_contact'
                    self.contact_new[(type_name, name)] = new
                    self._rename('contact', f'objects/{type_name}.yaml {name}', new)
        for robot in self.robots:
            self._stance_names(robot)
            self._pose_names(robot)
        self._contact_references()

    def _rename(self, kind: str, what: str, new: str) -> None:
        self.say(f'rename  {kind} {what} -> {new}')

    def variants(self, robot: str, type_name: str) -> list[str]:
        return list((self.stances.get(robot, {}).get('objects') or {}).get(type_name) or {})

    def _stance_names(self, robot: str) -> None:
        stances = self.stances.get(robot, {})
        by_type: dict[str, set[str]] = {}
        for type_name, table in (stances.get('objects') or {}).items():
            by_type.setdefault(type_name, set()).update(table or {})
        for instance, table in (stances.get('instances') or {}).items():
            type_name = self._stance_instance_type(instance, robot)
            by_type.setdefault(type_name, set()).update(table or {})
        for type_name, names in by_type.items():
            taken = set(self.type_targets(type_name)) | set(self.settings.get(robot, {}).get(type_name) or {})
            for name in list(self.variants(robot, type_name)) + sorted(names - set(self.variants(robot, type_name))):
                new = f'{name}_stance' if name in taken else name
                if new in taken:
                    raise RuntimeError(
                        f'robots/{robot}/stances.yaml {type_name} {name}: both {name} and {new} are '
                        f'targets of {type_name}: rename the stance by hand, then run {COMMAND} again'
                    )
                self.stance_new[(robot, type_name, name)] = new
                if new != name:
                    self._rename('stance', f'robots/{robot}/stances.yaml {type_name} {name}', new)

    def _stance_instance_type(self, instance: str, robot: str) -> str:
        found = self.instance_type(instance)
        if found is None:
            raise RuntimeError(
                f'robots/{robot}/stances.yaml instances: {instance} is no object of cell/objects.yaml '
                'and the type of the run-time object cannot be told: add it to cell/objects.yaml or '
                f'remove the entry, then run {COMMAND} again'
            )
        return found

    def _pose_names(self, robot: str) -> None:
        doc = self.poses.get(robot) or {}
        self.twins[robot] = {n: p for n, p in twin_names(doc).items() if n not in self.used_poses}
        pose_names = {n for n in self.pack_names(robot)} | {str(n) for n in doc if not _is_marker(n)}
        pose_names -= set(self.twins[robot])
        for name in sorted(pose_names & self.locations):
            self.pose_new[(robot, name)] = f'{name}_pose'
            self._rename('pose', f'{robot} {name} (clashes with the location {name})', f'{name}_pose')
        plain = self.locations | {self.pose_new.get((robot, n), n) for n in pose_names}
        seen: dict[str, tuple[str, object]] = {}
        for type_name, table in (self.held.get(robot) or {}).items():
            for name, entry in (table or {}).items():
                key = _plain(entry)
                if name in seen and seen[name][1] != key:
                    raise RuntimeError(
                        f'robots/{robot}/held_poses.yaml: {name} differs between {seen[name][0]} and '
                        f'{type_name}: a held pose name is one target now, rename one of them by hand, '
                        f'then run {COMMAND} again'
                    )
                seen.setdefault(name, (type_name, key))
        for name in seen:
            if name in plain:
                self.held_new[(robot, name)] = f'{name}_held'
                self._rename('held pose', f'{robot} {name}', f'{name}_held')

    # -- instances and types ----------------------------------------------------------------------------------
    def instance_type(self, name: str) -> str | None:
        """The object type of a placed object; a run-time object (not in cell/objects.yaml, e.g. put down by ``grip
        place_as``) is inferred from the targets the missions use on it, else from its name; None if that fails."""
        if name in self.instances:
            return self.instances[name]
        if name not in self._runtime:
            self._runtime[name] = self._infer_type(name)
        return self._runtime[name]

    def _infer_type(self, name: str) -> str | None:
        used = self._targets_used_on(name)
        if used:
            types = [t for t in self.types if all(u in self.type_targets(t) for u in used)]
            if len(types) == 1:
                return types[0]
        tokens = set(name.split('_'))
        scored = sorted(((len(tokens & set(t.split('_'))), t) for t in self.types), reverse=True)
        if scored and scored[0][0] > 0 and (len(scored) == 1 or scored[0][0] > scored[1][0]):
            return scored[0][1]
        return None

    def _targets_used_on(self, name: str) -> set[str]:
        """The target names the missions use on the instance ``name``: ``name/<target>`` literally or through the
        ``with`` of an include."""
        found: set[str] = set()
        for mission in self.missions.values():
            for step in self._steps(mission):
                if 'include' in step and isinstance(step.get('with'), Mapping):
                    inner = self.missions.get(str(step['include']), {})
                    for param, value in step['with'].items():
                        if value == name:
                            found |= _slash_names(inner, '{' + str(param) + '}')
            found |= _slash_names(mission, name)
        return found

    def _contact_references(self) -> None:
        """``onto: X/C, contact: K`` -> the reference K on the target C of X's type when every step on it names the
        same K (steps with a placeholder instance cannot be counted: they keep their own)."""
        uses: dict[tuple[str, str], set[str]] = {}
        for mission in self.missions.values():
            for step in self._steps(mission):
                to = (step.get('args') or {}).get('to') if isinstance(step.get('args'), Mapping) else None
                if step.get('skill') == 'move_object' and isinstance(to, Mapping) and 'onto' in to:
                    instance, _, contact = str(to['onto']).partition('/')
                    type_name = None if _placeholder(instance) else self.instance_type(instance)
                    if type_name and isinstance(to.get('contact'), str):
                        uses.setdefault((type_name, contact), set()).add(to['contact'])
        for key, names in uses.items():
            if len(names) == 1:
                self.contact_ref[key] = self.reference_name(next(iter(names)))

    def reference_name(self, contact: str) -> str:
        """The new name of the held object's contact ``contact`` (it may have been renamed on its type)."""
        owners = [t for t, data in self.types.items() if contact in (data.get('contacts') or {})]
        if not owners:
            self.flag(f'contact {contact} is a contact of no object type: the reference was written as it is')
            return contact
        return self.contact_new.get((owners[0], contact), contact)

    # -- references for the step converter -----------------------------------------------------------------------
    def location_name(self, name: str) -> str:
        return name

    def pose_name(self, robot: str, name: str) -> str:
        return self.pose_new.get((robot, name), name)

    def held_name(self, robot: str, name: str) -> str:
        return self.held_new.get((robot, name), name)

    def first_variant(self, robot: str, type_name: str) -> str:
        variants = self.variants(robot, type_name)
        if robot is None:
            raise Unresolved('the step has no robot to read the stance of: name the mission robot or system.yaml robot')
        if not variants:
            raise Unresolved(f'robot {robot} has no stance for {type_name} objects (robots/{robot}/stances.yaml)')
        return self.stance_new[(robot, type_name, variants[0])]

    def stance_reference(self, robot: str, text: str, mission: str | None, flag) -> str:
        """``<inst>[/<variant>]`` of an old ``stance:`` reference -> ``<inst>/<name>``."""
        instance, _, variant = text.partition('/')
        candidates = self._candidates(instance, mission, robot)
        names: dict[str, list[str]] = {}
        for value, type_name in candidates:
            if variant:
                name = self.stance_new.get((robot, type_name, variant), variant)
            else:
                name = self.first_variant(robot, type_name)
            names.setdefault(name, []).append(value)
            if value not in self.instances and not _placeholder(instance):
                flag(f'assumed {name} for {value} (a run-time object, taken for a {type_name})')
        if len(names) != 1:
            raise Unresolved(
                f'stance:{text}: the instance stands for objects whose stance names differ '
                f'({", ".join(f"{n}: {v}" for n, v in names.items())}): name the variant by hand'
            )
        name = next(iter(names))
        runtime = [f'{v} (a {t})' for v, t in candidates if v not in self.instances and not v.startswith('{')]
        if runtime and _placeholder(instance):
            flag(f'assumed {name} for {", ".join(runtime)}: a run-time object has no stance to read')
        return f'{instance}/{name}'

    def _candidates(self, instance: str, mission: str | None, robot: str) -> list[tuple[str, str]]:
        """``[(instance name, type)]`` the stance reference can stand for."""
        if not _placeholder(instance):
            type_name = self.instance_type(instance)
            if type_name is None:
                raise Unresolved(f'{instance} is no object of cell/objects.yaml and its type cannot be told')
            return [(instance, type_name)]
        match = re.fullmatch(r'\{(\w+)\}', instance)
        facts = (self.missions.get(mission or '', {}).get('facts') or {}) if match else {}
        if match and match.group(1) in facts:  # a fact of the mission itself
            values = self._fact_values(facts[match.group(1)])
        else:
            values = self._param_values(mission, match.group(1)) if match and mission else []
        out = []
        for value in values:
            type_name = self.instance_type(value)
            if type_name and self.variants(robot, type_name):
                out.append((value, type_name))
        if not out:
            types = [t for t in self.types if self.variants(robot, t)]
            out = [(f'{{{t}}}', t) for t in types]
            if not out:
                raise Unresolved(f'{instance}: no object for it has a stance')
        return out

    def _param_values(self, mission: str | None, param: str, depth: int = 0) -> list[str]:
        """What the includers give the parameter ``param`` of ``mission`` (facts expanded)."""
        values: list[str] = []
        if mission is None or depth > 3:
            return values
        for name, other in self.missions.items():
            for step in self._steps(other):
                if str(step.get('include')) != mission or not isinstance(step.get('with'), Mapping):
                    continue
                if param not in step['with']:
                    continue
                given = step['with'][param]
                match = re.fullmatch(r'\{(\w+)\}', str(given))
                if match and match.group(1) in (other.get('facts') or {}):
                    values += self._fact_values(other['facts'][match.group(1)])
                elif match and match.group(1) in (other.get('params') or {}):
                    values += self._param_values(name, match.group(1), depth + 1)
                else:
                    values.append(str(given))
        return list(dict.fromkeys(values))

    def _fact_values(self, fact: Mapping) -> list[str]:
        pattern = fact.get('among', '*')
        return [n for n, t in self.instances.items() if t == fact.get('fullest') and fnmatch.fnmatch(n, pattern)]

    def contact_target(self, onto: str, contact, flag) -> tuple[str, str | None]:
        """``onto: X/C`` + ``contact: K`` -> (the new target, the reference the step must carry or None)."""
        instance, _, name = str(onto).partition('/')
        type_name = None if _placeholder(instance) else self.instance_type(instance)
        new_name = self.contact_new.get((type_name, name), name) if type_name else name
        if not isinstance(contact, str):
            return f'{instance}/{new_name}', None
        reference = self.reference_name(contact)
        if type_name and self.contact_ref.get((type_name, name)) == reference:
            return f'{instance}/{new_name}', None
        return f'{instance}/{new_name}', reference

    def joint_groups_value(self, robot: str, joints, flag):
        groups = self.groups.get(robot) or {}
        if not groups:
            raise Unresolved(f'robot {robot} has no joint groups (profile.yaml joints.groups): cannot name joints')
        return joints_node(joints, groups, f'joints of a step for {robot}', _flow_of(joints))


def _slash_names(mission: Mapping, prefix: str) -> set[str]:
    """The ``<name>`` of every string ``<prefix>/<name>`` in the args of a mission's steps."""
    found: set[str] = set()

    def walk(value):
        if isinstance(value, str) and value.startswith(prefix + '/') and '/' not in value[len(prefix) + 1 :]:
            found.add(value[len(prefix) + 1 :])
        elif isinstance(value, Mapping):
            for each in value.values():
                walk(each)
        elif isinstance(value, list):
            for each in value:
                walk(each)

    for step in mission.get('steps', []) if isinstance(mission, Mapping) else []:
        if isinstance(step, Mapping):
            walk(step.get('args'))
    return found


# ---- the files ---------------------------------------------------------------------------------------------------
def migrate_locations(plan: Plan, book: Book, say) -> None:
    path = plan.cell / 'cell' / 'locations.yaml'
    if not path.is_file():
        return
    document, handler = _read(path)
    if document is None:
        from ruamel.yaml.comments import CommentedMap

        document = CommentedMap()
    for name, entry in document.items():
        if not isinstance(entry, Mapping):
            continue
        frame = entry.pop('frame', 'cell')
        if frame != 'cell':
            _flag(plan, say, f'cell/locations.yaml {name}: frame {frame} dropped: cell/targets.yaml is the cell frame')
        entry.insert(0, 'moves', 'base')
    plan.writes[plan.cell / 'cell' / 'targets.yaml'] = _dump(document, handler)
    plan.removes.append(path)
    say(f'move    cell/locations.yaml -> cell/targets.yaml  ({len(document)} base target(s), moves: base)')


def migrate_object_types(plan: Plan, book: Book, say) -> None:
    """objects/<type>.yaml: ``points`` and ``contacts`` become ``targets``, at the place of the first of the two."""
    for path in sorted((plan.cell / 'objects').glob('*.yaml')):
        document, handler = _read(path)
        if not isinstance(document, Mapping) or not ({'points', 'contacts'} & set(document)):
            continue
        type_name = path.stem
        keys = [key for key in document if key in ('points', 'contacts')]
        for key in keys:
            for name, entry in (document[key] or {}).items():
                entry.insert(0, 'moves', 'hand' if key == 'points' else 'object')
                if key == 'contacts' and (type_name, name) in book.contact_ref:
                    entry['reference'] = book.contact_ref[(type_name, name)]
        _rename_key(document, keys[0], 'targets')
        targets = document['targets']
        for name in list(targets):
            if keys[0] == 'contacts' and (type_name, name) in book.contact_new:
                _rename_key(targets, name, book.contact_new[(type_name, name)])
        for key in keys[1:]:
            other = document[key]
            lines = _key_comment(document, key, other)
            added = []
            for name in list(other):
                new_name = book.contact_new.get((type_name, name), name) if key == 'contacts' else name
                targets[new_name] = other[name]
                _carry(other, name, targets, new_name)
                added.append(new_name)
            if added:
                _comment_before(targets, added[0], lines, indent=2)
            del document[key]
        plan.writes[path] = _dump(document, handler)
        say(f'rewrite objects/{type_name}.yaml  (points, contacts -> targets: {", ".join(targets)})')


def _first_header(document) -> list[str]:
    return _comment_lines(document.ca.comment[1]) if getattr(document, 'ca', None) and document.ca.comment else []


def migrate_robot_objects(plan: Plan, book: Book, robot: str, say) -> None:
    """robots/<robot>/stances.yaml + object_points.yaml -> robots/<robot>/objects/<type>.yaml."""
    from ruamel.yaml.comments import CommentedMap

    stances_path = plan.cell / 'robots' / robot / 'stances.yaml'
    points_path = plan.cell / 'robots' / robot / 'object_points.yaml'
    stances, handler = _read(stances_path) if stances_path.is_file() else (None, None)
    settings, points_handler = _read(points_path) if points_path.is_file() else (None, None)
    handler = handler or points_handler
    stances = stances if isinstance(stances, Mapping) else CommentedMap()
    settings = settings if isinstance(settings, Mapping) else CommentedMap()
    if handler is None:
        return
    objects = stances.get('objects') or CommentedMap()
    instances = stances.get('instances') or CommentedMap()
    type_names = list(dict.fromkeys([*objects, *settings, *(book._stance_instance_type(i, robot) for i in instances)]))
    header = _first_header(stances) + _first_header(settings)
    plan.removes += [p for p in (stances_path, points_path) if p.is_file()]
    for number, type_name in enumerate(type_names):
        out = CommentedMap()
        variants = objects.get(type_name) or CommentedMap()
        lines = _key_comment(objects, type_name, variants) if type_name in objects else []
        lines += _key_comment(settings, type_name, settings.get(type_name)) if type_name in settings else []
        names = book.variants(robot, type_name)
        if names:
            out['handled_from'] = book.stance_new[(robot, type_name, names[0])]
        targets = CommentedMap()
        for variant, entry in variants.items():
            new = book.stance_new[(robot, type_name, variant)]
            entry.pop('frame', None)
            entry.insert(0, 'moves', 'base')
            targets[new] = entry
            _carry(variants, variant, targets, new)
        table = settings.get(type_name) or CommentedMap()
        point_names = set((book.types.get(type_name) or {}).get('points') or {})
        for point, entry in table.items():
            if point not in point_names:
                _flag(
                    plan,
                    say,
                    f'robots/{robot}/object_points.yaml {type_name} {point}: dropped, the settings are '
                    f'for no point ({type_name} '
                    + ('has no such point' if type_name in book.types else 'is no object type')
                    + '; check warned)',
                )
                continue
            targets[point] = entry
            _carry(table, point, targets, point)
        if targets:
            out['targets'] = targets
        mine = CommentedMap()
        for instance, table in instances.items():
            if book._stance_instance_type(instance, robot) != type_name:
                continue
            entry = CommentedMap()
            for variant, pose in (table or {}).items():
                new = book.stance_new[(robot, type_name, variant)]
                pose.pop('frame', None)
                if new not in targets or targets[new].get('moves') != 'base':
                    pose.insert(0, 'moves', 'base')
                entry[new] = pose
                _carry(table, variant, entry, new)
            mine[instance] = entry
            _carry(instances, instance, mine, instance)
        if mine:
            out['instances'] = mine
        top = header if number == 0 else []
        if not out:
            continue
        _set_start_comment(out, top + ([''] if top and lines else []) + lines)
        target = plan.cell / 'robots' / robot / 'objects' / f'{type_name}.yaml'
        plan.writes[target] = _dump(out, handler)
        say(
            f'write   robots/{robot}/objects/{type_name}.yaml  (stances {", ".join(targets) or "none"}'
            + (f'; instances {", ".join(mine)}' if mine else '')
            + ')'
        )
    if not type_names:
        say(f'remove  robots/{robot}/stances.yaml  (empty)')
    for path in (stances_path, points_path):
        if path.is_file():
            say(f'remove  {path.relative_to(plan.cell).as_posix()}')


def migrate_robot_targets(plan: Plan, book: Book, robot: str, say) -> None:
    """robots/<robot>/poses.yaml + held_poses.yaml -> robots/<robot>/targets.yaml."""
    from ruamel.yaml.comments import CommentedMap

    poses_path = plan.cell / 'robots' / robot / 'poses.yaml'
    held_path = plan.cell / 'robots' / robot / 'held_poses.yaml'
    if not poses_path.is_file() and not held_path.is_file():
        return
    groups = book.groups.get(robot) or {}
    flags: list[str] = []
    handler = None
    document = None
    if poses_path.is_file():
        document, handler = _read(poses_path)
        document = document if isinstance(document, Mapping) else CommentedMap()
        if not groups and any(not _is_marker(n) for n in document):
            raise RuntimeError(
                f'robots/{robot}/poses.yaml: robot {robot} has no joint groups (profile.yaml '
                'joints.groups): the poses cannot be converted'
            )
        renames = {n: new for (r, n), new in book.pose_new.items() if r == robot}
        convert_poses_doc(document, groups, f'robots/{robot}/poses.yaml', flags, say, book.used_poses, renames)
        plan.removes.append(poses_path)
    keyed = overlay.is_keyed(document) if document is not None else False
    if held_path.is_file():
        held, held_handler = _read(held_path)
        handler = handler or held_handler
        if document is None:
            document = CommentedMap()
            if book.pack_names(robot):
                document[overlay.MARKER] = overlay.KEYS
                keyed = True
                from .store import OVERRIDE_HEADER

                document.yaml_set_start_comment(OVERRIDE_HEADER.format(file='targets.yaml'))
        header = _first_header(held)
        merged: dict[str, tuple] = {}
        for type_name, table in (held if isinstance(held, Mapping) else {}).items():
            for name, entry in (table or {}).items():
                new = book.held_new.get((robot, name), name)
                if new in merged:
                    merged[new][0].append(type_name)
                    continue
                merged[new] = ([type_name], entry, _key_comment(table, name, entry), table, name)
        first = True
        for new, (types, entry, _lines, table, name) in merged.items():
            convert_held(
                entry,
                types[0] if len(types) == 1 else None,
                groups,
                f'robots/{robot}/held_poses.yaml {types[0]} {name}',
            )
            document[new] = entry
            _carry(table, name, document, new)
            if first and header:
                _comment_before(document, new, header)
            first = False
            say(
                f'held    {robot} {name} -> {new} (moves: object'
                + (f', type {types[0]}' if len(types) == 1 else f', no type: the same for {", ".join(types)}')
                + ')'
            )
        plan.removes.append(held_path)
    for flag in flags:
        _flag(plan, say, flag)
    target = plan.cell / 'robots' / robot / 'targets.yaml'
    plan.writes[target] = _dump(document, handler).replace(
        "cell override of the robot pack's poses.yaml", "cell override of the robot pack's targets.yaml", 1
    )
    say(f'write   robots/{robot}/targets.yaml' + ('  (keyed override, ~override: keys)' if keyed else ''))
    for path in (poses_path, held_path):
        if path.is_file():
            say(f'remove  {path.relative_to(plan.cell).as_posix()}')


def migrate_skill_files(plan: Plan, book: Book, steps_changed: bool, say) -> None:
    """skills/<old skill>.yaml -> skills/move/<module>.yaml (the same bytes); skills/move.yaml new."""
    skills = plan.cell / 'skills'
    moved = False
    for old, module in SKILL_FILES.items():
        path = skills / f'{old}.yaml'
        if not path.is_file():
            continue
        target = skills / 'move' / f'{module}.yaml'
        if target.is_file():
            _flag(
                plan,
                say,
                f'skills/{old}.yaml kept: skills/move/{module}.yaml exists already; merge them by hand, '
                f'then delete {old}.yaml',
            )
            continue
        plan.writes[target] = path.read_bytes().decode('utf-8')  # the same bytes
        plan.removes.append(path)
        moved = True
        say(f'move    skills/{old}.yaml -> skills/move/{module}.yaml')
    if (moved or steps_changed) and not (skills / 'move.yaml').is_file():
        plan.writes[skills / 'move.yaml'] = MOVE_YAML
        say('write   skills/move.yaml')


def migrate_skill_overrides(plan: Plan, robot: str, say) -> None:
    path = plan.cell / 'robots' / robot / 'skill_overrides.yaml'
    if not path.is_file():
        return
    document, handler = _read(path)
    if not isinstance(document, Mapping) or not any(key in MODULE_NAMES for key in document):
        return
    for key in [k for k in document if k in MODULE_NAMES]:
        _rename_key(document, key, MODULE_NAMES[key])
        say(f'rename  robots/{robot}/skill_overrides.yaml {key} -> {MODULE_NAMES[key]}')
    plan.writes[path] = _dump(document, handler)


# ---- steps -----------------------------------------------------------------------------------------------------------
def migrate_missions(plan: Plan, book: Book, say) -> bool:
    changed = False
    for path in sorted((plan.cell / 'missions').glob('*.yaml')):
        mission, handler = _read(path)
        if not isinstance(mission, Mapping) or not isinstance(mission.get('steps'), list):
            continue
        converter = StepConverter(book, mission.get('robot'), path.stem)
        done = False
        for number, step in enumerate(mission['steps'], start=1):
            if not isinstance(step, Mapping) or not isinstance(step.get('skill'), str):
                continue
            where = f'{path.stem} {step.get("id", f"s{number}")}'
            try:
                result = converter.convert(step['skill'], step.get('args'))
            except Unresolved as error:
                raise RuntimeError(f'{where}: {error}') from error
            if result is None:
                continue
            say(f'step    {where}: {step["skill"]} {_show(step.get("args") or {})} -> move {_show(result.args)}')
            step['skill'] = 'move'
            step['args'] = result.args
            done = True
            for flag in result.flags:
                _flag(plan, say, f'{where}: {flag}')
        _flag_params(plan, mission, path.stem, say)
        if done:
            changed = True
            plan.writes[path] = _dump(mission, handler)
    return changed


def _flag_params(plan: Plan, mission: Mapping, name: str, say) -> None:
    for param, info in (mission.get('params') or {}).items():
        kind = (info or {}).get('point')
        if kind and kind != 'instance':
            _flag(
                plan,
                say,
                f'{name}: params {param} has the point kind {kind}, which the new validation replaces by '
                'target: left as it is',
            )


def migrate_operator(plan: Plan, book: Book, say) -> bool:
    path = plan.cell / 'operator.yaml'
    if not path.is_file():
        return False
    document, handler = _read(path)
    actions = ((document or {}).get('joy') or {}).get('actions') if isinstance(document, Mapping) else None
    if not isinstance(actions, list):
        return False
    converter, changed = StepConverter(book, None), False
    for action in actions:
        if not isinstance(action, Mapping) or not isinstance(action.get('skill'), str):
            continue
        where = f'operator button {action.get("button")}'
        try:
            result = converter.convert(action['skill'], action.get('args'))
        except Unresolved as error:
            raise RuntimeError(f'{where}: {error}') from error
        if result is None:
            continue
        say(f'action  {where}: {action["skill"]} {_show(action.get("args") or {})} -> move {_show(result.args)}')
        action['skill'], action['args'] = 'move', result.args
        changed = True
        if 'arm_args' in action:
            _flag(plan, say, f'{where}: arm_args of {action["skill"]} are not rewritten; check them by hand')
        for flag in result.flags:
            _flag(plan, say, f'{where}: {flag}')
    if changed:
        plan.writes[path] = _dump(document, handler)
    return changed


# ---- comments that still name an old thing ---------------------------------------------------------------------------
def list_comments(plan: Plan, say) -> None:
    gone = {*plan.removes}
    seen: set[Path] = set()
    texts = {path: text for path, text in plan.writes.items()}
    for path in sorted(plan.cell.rglob('*.yaml')):
        if path in gone or 'moveit' in path.parts or '.git' in path.parts:
            continue
        texts.setdefault(path, None)
    for path in sorted(texts):
        if path in seen:
            continue
        seen.add(path)
        text = texts[path]
        if text is None:
            try:
                text = path.read_text(encoding='utf-8')
            except (OSError, UnicodeDecodeError):
                continue
        for number, line in enumerate(text.splitlines(), start=1):
            if OLD_WORDS.search(line):
                note = f'{_rel(plan, path)}:{number}: {line.strip()}'
                plan.listed.append(note)
                say(f'comment {note}')


# ---- robot packs -----------------------------------------------------------------------------------------------------
def _pack_groups(folder: Path) -> dict[str, list[str]]:
    import tempfile

    from .limits import joint_groups
    from .store import ConfigStore

    with tempfile.TemporaryDirectory() as scratch:
        store = ConfigStore(scratch, check_schema=False, first_packs=(folder,))
        return joint_groups(store, folder.name)


def is_robot_pack(folder: Path) -> bool:
    return (folder / packs.PACK_FILE).is_file() and (folder / 'profile.yaml').is_file()


def robot_pack_folders(path: Path) -> list[Path]:
    """The robot pack folders at ``path``: the folder itself, or the packs it holds."""
    if is_robot_pack(path):
        return [path]
    return sorted(p for p in path.glob('*') if p.is_dir() and is_robot_pack(p))


def plan_robot_pack(plan: Plan, folder: Path, say, label: str | None = None) -> None:
    """poses.yaml -> targets.yaml of one robot pack, schema_version 2 (plan only)."""
    pack = packs.read_pack(folder) or {}
    label = label or folder.name
    if int(pack.get('schema_version', 1)) >= 2:
        say(f'pack    {label}: schema {pack.get("schema_version")}, nothing to do')
        return
    poses_path = folder / 'poses.yaml'
    if poses_path.is_file():
        document, handler = _read(poses_path)
        document = document if isinstance(document, Mapping) else None
        if document is not None:
            groups = _pack_groups(folder)
            if not groups and any(not _is_marker(n) for n in document):
                raise RuntimeError(f'{label}: the profile has no joint groups: cannot convert poses.yaml')
            flags: list[str] = []
            convert_poses_doc(document, groups, f'{label} poses.yaml', flags, say)
            for flag in flags:
                _flag(plan, say, flag)
            plan.writes[folder / 'targets.yaml'] = _dump(document, handler)
            plan.removes.append(poses_path)
            for number, line in enumerate(plan.writes[folder / 'targets.yaml'].splitlines(), start=1):
                if OLD_WORDS.search(line):
                    plan.listed.append(f'{label} targets.yaml:{number}: {line.strip()}')
                    say(f'comment {plan.listed[-1]}')
            say(f'move    {label}: poses.yaml -> targets.yaml')
    text = (folder / packs.PACK_FILE).read_text(encoding='utf-8')
    plan.writes[folder / packs.PACK_FILE] = re.sub(r'(?m)^schema_version:\s*\d+', 'schema_version: 2', text)
    say(f'write   {label}: pack.yaml (schema_version 2)')


def migrate_robot_packs_only(path: Path, dry_run: bool, say) -> int:
    """``taskweave config migrate PACK_DIR``: a robot pack (or a folder of them) on its own."""
    folders = robot_pack_folders(path)
    plan = Plan(path if not is_robot_pack(path) else path.parent)
    try:
        for folder in folders:
            plan_robot_pack(plan, folder, say)
        problems = writable_problems(plan)
    except RuntimeError as error:
        say(f'{path}: {error}')
        return 2
    if problems and not dry_run:
        say(f'{path}: ' + '; '.join(problems) + ': nothing was changed')
        return 1
    if not dry_run:
        apply_plan(plan)
    say(
        f'{"would write" if dry_run else "wrote"} {len(plan.writes)} file(s), {"would " if dry_run else ""}remove '
        f'{len(plan.removes)}; {len(plan.flags)} flag(s)'
    )
    for flag in plan.flags:
        say(f'FLAG    {flag}')
    return 0


def plan_cell_robot_packs(plan: Plan, book: Book, robot_packs: Path | None, say) -> None:
    """The robot packs the cell uses (every robot of the cell, plus the --robot-packs folder): migrated when writable,
    an installed one is only reported."""
    from .store import ConfigError

    done: set[Path] = set()
    candidates: list[tuple[Path, bool, str]] = []
    for robot in book.robots:
        try:
            for pack in book.store.robot_packs(robot):
                candidates.append((pack.path.resolve(), pack.location.writable, pack.label))
        except ConfigError as error:
            say(f'note    robot {robot}: {error}')
    if robot_packs is not None:
        candidates += [
            (f.resolve(), True, f'robot pack {f.name} ({robot_packs})') for f in robot_pack_folders(robot_packs)
        ]
    for folder, writable, label in candidates:
        if folder in done:
            continue
        done.add(folder)
        pack = packs.read_pack(folder) or {}
        if int(pack.get('schema_version', 1)) >= 2:
            continue
        if not writable:
            _flag(
                plan,
                say,
                f'{label} is at schema {pack.get("schema_version", 1)} and read-only: migrate it where it '
                f'comes from ({COMMAND} <robot pack dir>)',
            )
            continue
        plan_robot_pack(plan, folder, say, label)


# ---- the migration -------------------------------------------------------------------------------------------------
def migrate_targets(cell: Path, robot_packs: Path | None, dry_run: bool, say) -> list[str]:
    """Migration 3 -> 4 (MIGRATIONS signature). Raises RuntimeError if a file cannot be written or a name cannot be
    resolved (nothing changed)."""
    do = 'would ' if dry_run else ''
    plan = Plan(cell)
    book = Book(cell, say, lambda text: _flag(plan, say, text))
    _check_first_variants(plan, book, say)
    migrate_locations(plan, book, say)
    migrate_object_types(plan, book, say)
    for robot in book.robots:
        folder = cell / 'robots' / robot
        if folder.exists() and (folder.is_symlink() or not folder.resolve().is_relative_to(cell)):
            _flag(
                plan,
                say,
                f'robots/{robot} is a link out of the cell pack: its files were not touched; migrate them '
                'where they live',
            )
            continue
        migrate_robot_objects(plan, book, robot, say)
        migrate_robot_targets(plan, book, robot, say)
        migrate_skill_overrides(plan, robot, say)
    changed = migrate_missions(plan, book, say)
    changed = migrate_operator(plan, book, say) or changed
    migrate_skill_files(plan, book, changed, say)
    plan_cell_robot_packs(plan, book, robot_packs, say)
    list_comments(plan, say)
    problems = writable_problems(plan)
    if problems and not dry_run:
        raise RuntimeError('; '.join(problems) + ': nothing was changed')
    for problem in problems:
        say(f'note    {problem} (a real run stops here)')
    if not dry_run:
        apply_plan(plan)
    say(
        f'{"would write" if dry_run else "wrote"} {len(plan.writes)} file(s), {do}remove '
        f'{len(plan.removes)}; {len(plan.flags)} flag(s); {len(plan.listed)} comment(s) name an old file or skill'
    )
    for flag in plan.flags:
        say(f'FLAG    {flag}')
    for text in plan.listed:
        say(f'comment {text}')
    return []


def _check_first_variants(plan: Plan, book: Book, say) -> None:
    """The first variant of a type is its default stance in the old layout: robots of the cell that disagree are
    flagged (a ``stance:<inst>`` step is rewritten with the mission's robot)."""
    types = {t for r in book.robots for t in ((book.stances.get(r) or {}).get('objects') or {})}
    for type_name in sorted(types):
        firsts = {r: book.first_variant(r, type_name) for r in book.robots if book.variants(r, type_name)}
        if len(set(firsts.values())) > 1:
            _flag(
                plan,
                say,
                f'{type_name}: the robots of the cell start their stance lists differently ('
                + ', '.join(f'{r}: {v}' for r, v in firsts.items())
                + '): `stance:<inst>` steps were '
                "rewritten with the mission's robot",
            )


def writable_problems(plan: Plan) -> list[str]:
    """What would stop the writing part-way (checked before anything is touched): a file to write or remove that is a
    symbolic link or not writable, a folder (the nearest one that exists) not writable, pack.yaml of the cell."""
    problems = []
    for path in [*plan.writes, *plan.removes, plan.cell / packs.PACK_FILE]:
        name = _rel(plan, path)
        if path.is_symlink():
            problems.append(f'{name} is a symbolic link: not touched')
        elif path.exists() and not os.access(path, os.W_OK):
            problems.append(f'{name} is not writable')
        else:
            parent = path.parent
            while not parent.exists():
                parent = parent.parent
            if not os.access(parent, os.W_OK):
                problems.append(f'{_rel(plan, parent)}/ is not writable')
    return list(dict.fromkeys(problems))


def _rel(plan: Plan, path: Path) -> str:
    try:
        return plan.rel(path)
    except ValueError:
        return str(path)


def apply_plan(plan: Plan) -> None:
    """The files of the plan, written aside and renamed; new folders made first."""
    for path in plan.writes:
        path.parent.mkdir(parents=True, exist_ok=True)
    apply(plan)


__all__ = ['migrate_targets', 'migrate_robot_packs_only', 'Book', 'is_robot_pack', 'robot_pack_folders']
