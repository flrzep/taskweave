"""Cell pack migration 2 -> 3: the skill model of skill-api-spec.md (``taskweave config migrate-skills CELL_DIR
[--dry-run]``, also the last step of ``taskweave config migrate``).

The old skills move_to_pose, go_to_point, gripper and the old move_object (flat args by / at / pose / to + contact /
elbow) are replaced by move_joints, move_gripper, grip, move_object (args ``to``) and set_posture. Rewritten, comments
and key order kept (ruamel.yaml):

  missions/*.yaml    every step's skill and args (spec 5.2); includes, ``{placeholders}``, ``defaults``, step ids and
                     ``replan_from`` stay (a step that has to be split gets ``<id>_joints`` / ``<id>_tools`` and the
                     references to it follow)
  operator.yaml      the actions' skill, args and arm_args
  skills/*.yaml      move_gripper, grip, set_posture, move_joints written from the new layout with the cell's own
                     values (speeds, tolerances, approach_m, torso heights) taken from go_to_point / gripper /
                     move_object / move_to_pose; move_object.yaml switched to the new class; the old files removed
  robots/<r>/poses.yaml
                     a pose of this cell with joints and tools gets a joint-only twin ``<pose>_joints`` (the
                     non-arm joints) that move_joints can take; the pose itself stays as it is

Nothing is written until every file is built and checked; then each file is written aside and renamed. A step the
migration cannot map cleanly is still rewritten mechanically and flagged: every flag is printed again at the end.
Data the new skills read as the old ones did (held poses, object_points ``elbow`` / ``grip_m``, the points' own
``approach_m``) is not touched. Comments that still name an old skill are listed, not edited.
"""

from __future__ import annotations

import os
import re
import tempfile
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from . import skill_templates

COMMAND = 'taskweave config migrate-skills'
OLD_SKILLS = ('move_to_pose', 'go_to_point', 'gripper')    # gone; move_object keeps its name, its args changed
OLD_OBJECT_KEYS = ('by', 'at', 'pose', 'contact', 'offset_m', 'turn_deg', 'tilt_deg', 'elbow')
NEW_OBJECT_KEYS = ('hands', 'motion', 'object')
OLD_SKILL_YAMLS = {'go_to_point': 'move_gripper', 'gripper': 'grip', 'move_to_pose': 'move_joints'}
COMMENT_WORDS = re.compile(r'#.*\b(move_to_pose|go_to_point)\b')
PLACEHOLDER = re.compile(r'\{\w+\}')
COMMENT_COLUMN = 32     # where the templates align their line comments
STEP_SETTINGS = ('timeout_s', 'retries', 'on_failure', 'max_replans')


REPLACED_BY = {'move_to_pose': 'move_joints (joint poses) or move_gripper (tool poses)', 'go_to_point': 'move_gripper',
               'gripper': 'grip'}


def removed_skill_message(skill: str, args: Mapping | None = None, cell: str = '<cell dir>') -> str | None:
    """The validation message for a step that names a removed skill or gives move_object its old args; None if it
    does not. It names the migration command."""
    command = f'run `{COMMAND} {cell}` (it rewrites missions, operator.yaml and skills/)'
    if skill in OLD_SKILLS:
        return f'the skill {skill} was removed, use {REPLACED_BY[skill]}: {command}'
    if skill == 'move_object' and is_old_move_object(args or {}):
        return (f'move_object takes its target as `to` now (by / at / pose / onto + contact), and the elbow argument '
                f'is the skill set_posture: {command}')
    return None


REMOVED_CLASSES = {'taskweave_skills.joints': ('MoveToPose', 'GoToPoint', 'MoveObject', 'Gripper')}


def removed_class_message(module: str, name: str) -> str | None:
    """The message for a skills/<skill>.yaml implementation class that was removed (the old skills of joints.py)."""
    if name in REMOVED_CLASSES.get(module, ()):
        return (f'{module}:{name} was removed with the old skills (move_to_pose, go_to_point, gripper, old '
                f'move_object): run `{COMMAND} <cell dir>`')
    return None


def is_old_move_object(args: Mapping) -> bool:
    """Old move_object args: no ``to``, or ``to`` as a string (a contact); new ones have ``to`` as an object."""
    if isinstance(args.get('to'), Mapping) or any(key in args for key in NEW_OBJECT_KEYS):
        return False
    to = args.get('to')
    if isinstance(to, str) and PLACEHOLDER.fullmatch(to) and 'contact' not in args:
        return False    # `to: "{target}"` (spec 8.2) is the new form: the parameter holds the whole target
    return 'to' in args or any(key in args for key in OLD_OBJECT_KEYS)


# ---- ruamel helpers --------------------------------------------------------------------------------------------
def _handler():
    from .store import _yaml_handler

    return _yaml_handler()


def _yaml_for(text: str):
    """The round-trip handler set to the indentation of ``text``: how far its first block list is indented under its
    key (``steps:`` / ``- item`` at the same column, or ``actions:`` / ``  - item`` two further in)."""
    handler = _handler()
    lines = text.splitlines()
    for before, line in zip(lines, lines[1:], strict=False):
        dash = re.match(r'(\s*)- ', line)
        key = re.match(r'''(\s*)[\w"'~-][^#]*:\s*(#.*)?$''', before)
        if dash and key:
            offset = max(len(dash.group(1)) - len(key.group(1)), 0)
            handler.indent(mapping=2, sequence=offset + 2, offset=offset)
            break
    return handler


def _map(items, flow: bool = False):
    from ruamel.yaml.comments import CommentedMap

    node = CommentedMap(items)
    if flow:
        node.fa.set_flow_style()
    return node


def _seq(items, flow: bool = True):
    from ruamel.yaml.comments import CommentedSeq

    node = CommentedSeq(items)
    if flow:
        node.fa.set_flow_style()
    return node


def _flow_of(node) -> bool:
    return bool(getattr(node, 'fa', None) and node.fa.flow_style())


def _show(value) -> str:
    """A step's args on one line, as the output prints them."""
    text = yaml.safe_dump(_plain(value), default_flow_style=True, width=10_000, sort_keys=False).strip()
    return text.removesuffix('\n...').strip()


def _plain(value):
    if isinstance(value, Mapping):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, str):
        return str(value)
    if isinstance(value, bool) or value is None:
        return value
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value)
    return value


def _placeholder(value) -> bool:
    return isinstance(value, str) and bool(PLACEHOLDER.search(value))


# ---- the poses the missions name -------------------------------------------------------------------------------
class PoseBook:
    """What the robots of the cell say about a named pose: joints only, tools only, or both."""

    def __init__(self, cell: Path, say):
        from .store import ConfigError, ConfigStore

        self.cell, self.say = cell, say
        self.store = ConfigStore(cell, check_schema=False)
        self.robots: list[str] = []
        system = cell / 'system.yaml'
        if system.is_file():
            robot = (yaml.safe_load(system.read_text(encoding='utf-8')) or {}).get('robot')
            if robot:
                self.robots.append(str(robot))
        self.robots += sorted(p.name for p in (cell / 'robots').glob('*') if p.is_dir() and p.name not in self.robots)
        self.poses: dict[str, dict] = {}
        for robot in self.robots:
            try:
                self.poses[robot] = dict(self.store.joint_poses(robot))
            except ConfigError as error:
                say(f'note    robots/{robot}/poses.yaml not readable ({error}): its poses are not classified')
                self.poses[robot] = {}
        self._arm_joints: dict[str, set | None] = {}

    @staticmethod
    def kind_of(entry) -> str:
        if isinstance(entry, Mapping) and 'tools' in entry:
            return 'both' if entry.get('joints') else 'tools'
        return 'joints'

    def classify(self, name: str, robot: str | None = None) -> tuple[str, dict | None, list[str]]:
        """(kind, the pose entry, notes): kind is joints, tools, both or unknown; the entry is the one of the first
        robot that has the pose (the mission's robot, the system's, the rest); notes name disagreeing robots."""
        order = ([robot] if robot in self.poses else []) + [r for r in self.robots if r != robot]
        found = [(r, self.poses[r][name]) for r in order if name in self.poses.get(r, {})]
        if not found:
            return 'unknown', None, []
        kinds = {self.kind_of(entry) for _, entry in found}
        notes = [] if len(kinds) == 1 else [f'robots disagree on {name}: ' + ', '.join(
            f'{r} {self.kind_of(entry)}' for r, entry in found) + f'; used {found[0][0]}']
        return self.kind_of(found[0][1]), found[0][1], notes

    def arm_joints(self, robot: str) -> set | None:
        """The joints that belong to an arm (moving them moves a hand); None if the robot's profile is unreadable."""
        if robot not in self._arm_joints:
            from .limits import arm_group_names, joint_groups
            from .store import ConfigError

            try:
                groups = joint_groups(self.store, robot)
                arms = {j for g in arm_group_names(self.store, robot) for j in groups.get(g, [])}
                self._arm_joints[robot] = arms or None    # no arm group known: the arm joints are unknown
            except (ConfigError, KeyError, OSError, ValueError):
                self._arm_joints[robot] = None
        return self._arm_joints[robot]

    def non_arm_joints(self, name: str, robot: str | None) -> tuple[dict, bool]:
        """The joints of pose ``name`` that are not an arm's, and whether the arm joints were known."""
        _, entry, _ = self.classify(name, robot)
        owner = next((r for r in ([robot] if robot in self.poses else []) + self.robots
                      if name in self.poses.get(r, {})), None)
        joints = dict((entry or {}).get('joints') or {})
        arms = self.arm_joints(owner) if owner else None
        if arms is None:
            return joints, False
        return {j: v for j, v in joints.items() if j not in arms}, True


# ---- step rewriting --------------------------------------------------------------------------------------------
@dataclass
class Rewrite:
    """What one old step becomes: ``steps`` are (skill, args) in order, flags are for the report."""

    steps: list[tuple[str, object]]
    flags: list[str] = field(default_factory=list)
    split: bool = False         # two steps with the ids <id>_joints, <id>_tools


class Converter:
    def __init__(self, poses: PoseBook):
        self.poses = poses

    # -- helpers
    @staticmethod
    def _take(args: Mapping, keys: tuple) -> dict:
        return {k: args[k] for k in keys if k in args}

    @staticmethod
    def _rest(args: Mapping, used: tuple) -> dict:
        return {k: v for k, v in args.items() if k not in used}

    def convert(self, skill: str, args, robot: str | None, kinds: Mapping | None = None) -> Rewrite | None:
        """The rewrite of one step, or None if the step is not an old one. ``kinds``: mission params' point kinds."""
        if not isinstance(args, Mapping):
            args = {}
        if skill == 'move_to_pose':
            return self._pose(args, robot, kinds or {})
        if skill == 'go_to_point':
            return self._point(args)
        if skill == 'gripper':
            return self._grip(args)
        if skill == 'move_object' and is_old_move_object(args):
            return self._object(args)
        return None

    def _extras(self, rewrite: Rewrite, args: Mapping, used: tuple) -> Rewrite:
        """Keys the old skill did not know stay in the top-level args of the new step; flagged."""
        extra = self._rest(args, used)
        if extra:
            last = rewrite.steps[-1][1]
            for key, value in extra.items():
                last[key] = value
            rewrite.flags.append(f'unknown old args kept as they are: {", ".join(extra)}')
        return rewrite

    # -- move_to_pose
    def _pose(self, args: Mapping, robot, kinds: Mapping) -> Rewrite:
        flow = _flow_of(args)
        pose = args.get('pose')
        if pose is None:
            return self._extras(Rewrite([('move_joints', _map({}, flow))]), args, ())
        if _placeholder(pose):
            name = re.fullmatch(r'\{(\w+)\}', str(pose))
            kind = kinds.get(name.group(1)) if name else None
            flag = (f'pose {pose} is a placeholder: assumed {"a tool pose" if kind == "tool_pose" else "a joint pose"}'
                    f'{"" if kind in ("tool_pose", "joint_pose") else " (the parameter has no point kind)"}')
            if kind == 'tool_pose':
                return Rewrite([('move_gripper', _map({'to': _map({'pose': pose}, flow)}, flow))], [flag])
            return self._extras(Rewrite([('move_joints', _map({'pose': pose}, flow))], [flag]), args, ('pose',))
        kind, entry, notes = self.poses.classify(str(pose), robot)
        if kind in ('joints', 'unknown'):
            flags = notes + ([f'pose {pose} is not in the poses of any robot of the cell: assumed joints only']
                             if kind == 'unknown' else [])
            return self._extras(Rewrite([('move_joints', _map({'pose': pose}, flow))], flags), args, ('pose',))
        tools = list(entry['tools'])
        elbow = entry.get('elbow')

        def gripper_args(hands: bool = True) -> object:
            to = _map({'to': _map({'pose': pose}, flow)}, flow)
            if hands:
                to['hands'] = _seq(tools)
            if elbow:
                to['motion'] = _map({'elbow': elbow}, flow)
            return to

        if kind == 'tools':
            return self._extras(Rewrite([('move_gripper', gripper_args())], notes), args, ('pose',))
        joints, known = self.poses.non_arm_joints(str(pose), robot)
        flags = notes + [f'pose {pose} has joints and tools: split into move_joints {pose}_joints and move_gripper '
                         f'{pose}' + ('' if known else ' (arm joints unknown: the joint twin keeps every joint)')]
        if not joints:
            return self._extras(Rewrite([('move_gripper', gripper_args())], flags[:-1] + [
                f'pose {pose} has joints and tools, no joints besides the arms: move_gripper only']), args, ('pose',))
        steps = [('move_joints', _map({'pose': f'{pose}_joints'}, flow)), ('move_gripper', gripper_args())]
        return self._extras(Rewrite(steps, flags, split=True), args, ('pose',))

    # -- go_to_point
    def _point(self, args: Mapping) -> Rewrite:
        flow = _flow_of(args)
        flags = []
        to = _map({'point': args.get('point')}, flow)
        if 'offset_m' in args:
            to['offset_m'] = args['offset_m']
        new = _map({'to': to}, flow)
        if 'tools' in args:
            new['hands'] = args['tools']
        motion = _map({}, flow)
        linear = args.get('linear')
        if _placeholder(linear):
            flags.append(f'linear is the placeholder {linear}: set motion.mode by hand')
        elif linear:
            motion['mode'] = 'linear'
        if 'approach_m' in args:
            if linear and not _placeholder(linear):
                flags.append(f'approach_m {args["approach_m"]} dropped: a linear move ignored it')
            else:
                motion['approach_m'] = args['approach_m']
        if motion:
            new['motion'] = motion
        return self._extras(Rewrite([('move_gripper', new)], flags), args, ('point', 'offset_m', 'tools', 'linear',
                                                                            'approach_m'))

    # -- gripper
    def _grip(self, args: Mapping) -> Rewrite:
        flow = _flow_of(args)
        new = _map({}, flow)
        if 'side' in args:
            new['hands'] = self.hands(args['side'])
        if 'action' in args:
            new['action'] = args['action']
        if 'place_as' in args:
            new['place_as'] = args['place_as']
        return self._extras(Rewrite([('grip', new)]), args, ('side', 'action', 'place_as'))

    @staticmethod
    def hands(side):
        """Old ``side`` -> ``hands``: both -> all, a hand -> [hand], a placeholder stays."""
        if _placeholder(side):
            return side
        return 'all' if side == 'both' else _seq([side])

    # -- move_object
    def _object(self, args: Mapping) -> Rewrite:
        flow = _flow_of(args)
        if 'pose' in args:
            return self._extras(Rewrite([('move_object', _map({'to': _map({'pose': args['pose']}, flow)}, flow))]),
                                args, ('pose',))
        if 'elbow' in args:
            return self._extras(Rewrite([('set_posture', _map({'elbow': args['elbow']}, flow))]), args, ('elbow',))
        if 'to' in args:    # the old ``to`` is a contact of a placed object
            onto = _map({'onto': args['to']}, flow)
            for key in ('contact', 'offset_m'):
                if key in args:
                    onto[key] = args[key]
            return self._extras(Rewrite([('move_object', _map({'to': onto}, flow))]), args,
                                ('to', 'contact', 'offset_m'))
        key = 'at' if 'at' in args else 'by'
        to = _map({key: args.get(key)}, flow)
        for extra in ('turn_deg', 'tilt_deg'):
            if extra in args:
                to[extra] = args[extra]
        return self._extras(Rewrite([('move_object', _map({'to': to}, flow))]), args,
                            (key, 'turn_deg', 'tilt_deg'))


# ---- the files -------------------------------------------------------------------------------------------------
@dataclass
class Plan:
    """Files to write (path -> text) and to remove, built before anything is touched."""

    cell: Path
    writes: dict[Path, str] = field(default_factory=dict)
    removes: list[Path] = field(default_factory=list)
    flags: list[str] = field(default_factory=list)
    listed: list[str] = field(default_factory=list)

    def rel(self, path: Path) -> str:
        return path.relative_to(self.cell).as_posix()


def _dump(document, handler) -> str:
    import io

    stream = io.StringIO()
    handler.dump(document, stream)
    return stream.getvalue()


def _read(path: Path):
    """(document, handler) of a YAML file, the handler set to the file's list indentation."""
    text = path.read_text(encoding='utf-8')
    handler = _yaml_for(text)
    return handler.load(text), handler


def _kinds(mission) -> dict:
    return {name: (info or {}).get('point') for name, info in (mission.get('params') or {}).items()}


def migrate_missions(plan: Plan, poses: PoseBook, say) -> None:
    """missions/*.yaml: every step rewritten; ``replan_from`` follows split steps."""
    converter = Converter(poses)
    documents: dict[str, object] = {}
    handlers: dict[str, object] = {}
    renames: dict[str, dict[str, str]] = {}     # mission -> {old step id: first new id} for split steps
    changed: set[str] = set()
    for path in sorted((plan.cell / 'missions').glob('*.yaml')):
        mission, handler = _read(path)
        if not isinstance(mission, Mapping) or not isinstance(mission.get('steps'), list):
            continue
        documents[path.stem], handlers[path.stem] = mission, handler
        steps = mission['steps']
        kinds, robot, index = _kinds(mission), mission.get('robot'), 0
        grasped = included = reached = False   # what the mission itself did so far (the nothing-held flag)
        while index < len(steps):
            step = steps[index]
            if not isinstance(step, Mapping):
                index += 1
                continue
            step_id = str(step.get('id', f's{index + 1}'))
            if 'include' in step:
                included = True
            skill = step.get('skill')
            holds_nothing = reached and not grasped and not included and skill == 'move_object'
            result = converter.convert(skill, step.get('args'), robot, kinds) if isinstance(skill, str) else None
            if skill == 'go_to_point':
                reached = True
            if skill == 'gripper' and (step.get('args') or {}).get('action') == 'close':
                grasped = True
            if result is None:
                index += 1
                continue
            changed.add(path.stem)
            where = f'{path.stem} {step_id}'
            flags = list(result.flags)
            if holds_nothing and result.steps[0][0] == 'move_object':
                flags.append('nothing is held at this step (the mission closes no gripper before it): it fails with '
                             '"nothing is held" as before; fix it by hand')
            old_args = step.get('args') or {}
            if result.split:
                _split_step(steps, index, step, step_id, result)
                renames.setdefault(path.stem, {})[step_id] = f'{step_id}_joints'
                say(f'step    {where}: {skill} {_show(old_args)} ->')
                for new_step in steps[index:index + 2]:
                    say(f'            {new_step["id"]}: {new_step["skill"]} {_show(new_step["args"])}')
                index += 2
            else:
                new_skill, new_args = result.steps[0]
                say(f'step    {where}: {skill} {_show(old_args)} -> {new_skill} {_show(new_args)}')
                step['skill'] = new_skill
                step['args'] = new_args
                index += 1
            for flag in flags:
                _flag(plan, say, f'{where}: {flag}')
    for name, mission in documents.items():     # replan_from references to split steps
        for step in mission['steps']:
            if isinstance(step, Mapping) and step.get('replan_from'):
                new = _renamed(str(step['replan_from']), name, documents, renames)
                if new != step['replan_from']:
                    say(f'step    {name} {step.get("id", "?")}: replan_from {step["replan_from"]} -> {new}')
                    step['replan_from'] = new
                    changed.add(name)
    for name in sorted(changed):
        plan.writes[plan.cell / 'missions' / f'{name}.yaml'] = _dump(documents[name], handlers[name])


def _split_step(steps, index, step, step_id, result) -> None:
    """Replace ``steps[index]`` by the joints step and the tools step; later steps without an id keep the id they had
    (s<n> of the old numbering)."""
    from ruamel.yaml.comments import CommentedMap

    (joint_skill, joint_args), (tool_skill, tool_args) = result.steps
    joints = CommentedMap()
    joints['id'] = f'{step_id}_joints'
    if 'instruction' in step:
        joints['instruction'] = f'{step["instruction"]} (joints)'
    joints['skill'], joints['args'] = joint_skill, joint_args
    for key in STEP_SETTINGS:
        if key in step:
            joints[key] = step[key]
    if 'id' in step:
        step['id'] = f'{step_id}_tools'
    else:
        step.insert(0, 'id', f'{step_id}_tools')
    step['skill'], step['args'] = tool_skill, tool_args
    steps.insert(index, joints)
    for later in range(index + 2, len(steps)):  # ids the executive would number by position keep their old number
        if isinstance(steps[later], Mapping) and 'id' not in steps[later]:
            steps[later].insert(0, 'id', f's{later}')


def _renamed(reference: str, mission: str, documents: dict, renames: dict) -> str:
    """``reference`` (a step id of ``mission``, or ``<include id>_<id inside>``) after the splits."""
    own = renames.get(mission, {})
    if reference in own:
        return own[reference]
    for number, step in enumerate(documents[mission]['steps'], start=1):
        if isinstance(step, Mapping) and 'include' in step:
            prefix = f'{step.get("id", f"s{number}")}_'
            inner = str(step['include'])
            if reference.startswith(prefix) and inner in documents:
                new = _renamed(reference[len(prefix):], inner, documents, renames)
                if new != reference[len(prefix):]:
                    return prefix + new
    return reference


def _flag(plan: Plan, say, text: str) -> None:
    plan.flags.append(text)
    say(f'FLAG    {text}')


# ---- operator.yaml -----------------------------------------------------------------------------------------------
def migrate_operator(plan: Plan, poses: PoseBook, say) -> None:
    path = plan.cell / 'operator.yaml'
    if not path.is_file():
        return
    document, handler = _read(path)
    actions = ((document or {}).get('joy') or {}).get('actions')
    if not isinstance(actions, list):
        return
    converter, changed = Converter(poses), False
    for action in actions:
        if not isinstance(action, Mapping):
            continue
        skill = action.get('skill')
        result = converter.convert(skill, action.get('args'), None) if isinstance(skill, str) else None
        if result is None:
            continue
        changed = True
        new_skill, new_args = result.steps[0]
        where = f'operator button {action.get("button")}'
        say(f'action  {where}: {skill} {_show(action.get("args") or {})} -> {new_skill} {_show(new_args)}')
        if result.split:     # a joint-only action step cannot do both: the pad keeps the tools part
            _flag(plan, say, f'{where}: pose {action["args"].get("pose")} has joints and tools: the button now runs '
                             f'the move_joints part only; add a second button for move_gripper by hand')
        for flag in result.flags:
            if not result.split or 'split' not in flag:
                _flag(plan, say, f'{where}: {flag}')
        action['skill'], action['args'] = new_skill, new_args
        if 'arm_args' in action:
            arm = action['arm_args']
            if skill == 'gripper':
                new = _map({}, _flow_of(arm))
                for key, value in arm.items():
                    new['hands' if key == 'side' else key] = value
                say(f'action  {where}: arm_args {_show(arm)} -> {_show(new)}')
                action['arm_args'] = new
            else:
                _flag(plan, say, f'{where}: arm_args of {skill} are not rewritten; check them by hand')
    if changed:
        plan.writes[path] = _dump(document, handler)


# ---- skills/*.yaml -----------------------------------------------------------------------------------------------
SPEED_KEYS = {'max_joint_speed': 'joint_rad_s', 'max_linear_speed': 'joint_linear_m_s', 'tcp_speed': 'tcp_m_s',
              'tcp_turn': 'tcp_rad_s'}
TOLERANCE_KEYS = {'position_tolerance': 'position', 'settle_s': 'settle_s', 'stall_timeout_s': 'stall_timeout_s',
                  'state_timeout_s': 'state_timeout_s'}
HANDLED = {*SPEED_KEYS, *TOLERANCE_KEYS, 'implementations', 'cell_frame', 'lift_joint', 'lift_values', 'approach_m',
           'elbow_tolerance_deg', 'grip_drift_m'}


def _carry(old, old_key, new, new_key) -> None:
    """The line comment of ``old[old_key]`` onto ``new[new_key]``."""
    items = getattr(old, 'ca', None) and old.ca.items.get(old_key)
    if items and items[2] is not None:
        new.yaml_add_eol_comment(items[2].value.strip().lstrip('#').strip(), new_key, column=COMMENT_COLUMN)


def _lay(new, section, old, mapping: Mapping | None = None, keys: tuple | None = None) -> None:
    """Old flat values onto the template: ``mapping`` old key -> key in ``new[section]`` (section None: top level)."""
    target = new if section is None else new[section]
    for old_key, new_key in (mapping or {k: k for k in keys or ()}).items():
        if old_key in old:
            target[new_key] = old[old_key]
            _carry(old, old_key, target, new_key)


def build_skill_yaml(skill: str, old: Mapping | None, old_name: str | None) -> tuple[object, list[str]]:
    """The new skills/<skill>.yaml: the template with the values of the old file laid over it; the flags."""
    document = _yaml_for(skill_templates.TEMPLATES[skill]).load(skill_templates.TEMPLATES[skill])
    old, flags = old or {}, []
    _lay(document, None, old, keys=('cell_frame',))
    if 'speed' in document:
        _lay(document, 'speed', old, {k: v for k, v in SPEED_KEYS.items() if v in document['speed']})
    _lay(document, 'tolerances', old, TOLERANCE_KEYS)
    if skill == 'move_gripper':
        if 'approach_m' in old:
            document['motion']['approach_m'] = old['approach_m']
            _carry(old, 'approach_m', document['motion'], 'approach_m')
        if 'lift_joint' in old or 'lift_values' in old:
            _lay(document, 'torso_fallback', old, {'lift_joint': 'joint', 'lift_values': 'values'})
    if skill == 'set_posture':     # the old move_object swung the elbows at its joint speed and tolerance
        _lay(document, None, old, keys=('elbow_tolerance_deg',))
    if skill == 'move_object':
        _lay(document, None, old, keys=('grip_drift_m',))
    extra = [k for k in old if k not in HANDLED and k not in document]
    for key in extra:
        document[key] = old[key]
    if extra:
        flags.append(f'{old_name}.yaml: keys the new layout does not know kept as they are: {", ".join(extra)}')
    extras = [i for i in old.get('implementations', []) if i.get('params') or set(i) - {'name', 'type', 'class',
                                                                                         'requires', 'params'}]
    for implementation in extras:   # implementation params of the old file apply to every implementation
        if implementation.get('params'):
            for new_impl in document['implementations']:
                new_impl['params'] = implementation['params']
            flags.append(f'{old_name}.yaml: implementation params {_show(implementation["params"])} copied to '
                         f'every implementation of {skill}')
    return document, flags


def migrate_skill_yamls(plan: Plan, say) -> None:
    skills = plan.cell / 'skills'
    if not skills.is_dir():
        return

    def load(name):
        path = skills / f'{name}.yaml'
        return _read(path)[0] if path.is_file() else None

    def put(skill, old, old_name):
        target = skills / f'{skill}.yaml'
        if target.is_file():
            say(f'keep    skills/{skill}.yaml  exists already: not replaced')
            return
        document, flags = build_skill_yaml(skill, old, old_name)
        plan.writes[target] = _dump(document, _yaml_for(skill_templates.TEMPLATES[skill]))
        say(f'write   skills/{skill}.yaml  (from {old_name}.yaml: ' + ', '.join(
            f'{k}={_show(v)}' for k, v in _carried(old, skill)) + ')')
        for flag in flags:
            _flag(plan, say, flag)

    for old_name, skill in OLD_SKILL_YAMLS.items():
        old = load(old_name)
        if old is None:
            continue
        put(skill, old, old_name)
        plan.removes.append(skills / f'{old_name}.yaml')
        say(f'remove  skills/{old_name}.yaml')
    current = load('move_object')
    classes = [i.get('class', '') for i in (current or {}).get('implementations', [])]
    if current is not None and any(c.endswith('joints:MoveObject') for c in classes):
        put('set_posture', current, 'move_object')
        document, flags = build_skill_yaml('move_object', current, 'move_object')
        plan.writes[skills / 'move_object.yaml'] = _dump(document, _yaml_for(skill_templates.MOVE_OBJECT))
        say('rewrite skills/move_object.yaml  (class taskweave_skills.object_motion:MoveObject, layout of '
            'spec 2.2: ' + ', '.join(f'{k}={_show(v)}' for k, v in _carried(current, 'move_object')) + ')')
        for flag in flags:
            _flag(plan, say, flag)


CARRIED = {     # the values of an old skills/<skill>.yaml each new file takes over
    'move_gripper': (*SPEED_KEYS, *TOLERANCE_KEYS, 'cell_frame', 'approach_m', 'lift_joint', 'lift_values'),
    'grip': ('max_joint_speed', *TOLERANCE_KEYS, 'cell_frame'),
    'move_object': (*SPEED_KEYS, *TOLERANCE_KEYS, 'cell_frame', 'grip_drift_m'),
    'set_posture': ('max_joint_speed', *TOLERANCE_KEYS, 'cell_frame', 'elbow_tolerance_deg'),
    'move_joints': ('max_joint_speed', 'max_linear_speed', *TOLERANCE_KEYS),
}


def _carried(old, skill: str) -> list:
    return [(k, old[k]) for k in old if k in CARRIED[skill]]


# ---- poses.yaml: the joint twin of a pose with joints and tools --------------------------------------------------
def migrate_poses(plan: Plan, poses: PoseBook, say) -> None:
    for robot in poses.robots:
        path = plan.cell / 'robots' / robot / 'poses.yaml'
        own: set[str] = set()
        if path.is_file():
            document, handler = _read(path)
            if isinstance(document, Mapping):
                own = {str(k) for k in document}
                changed = False
                for name in list(document):
                    entry = document[name]
                    if not (poses.kind_of(entry) == 'both' and not str(name).startswith('~')):
                        continue
                    twin = f'{name}_joints'
                    if twin in document:
                        continue
                    joints, known = poses.non_arm_joints(str(name), robot)
                    if not joints:
                        continue
                    node = _map(joints, True)
                    position = list(document).index(name) + 1
                    document.insert(position, twin, node)
                    changed = True
                    say(f'pose    robots/{robot}/poses.yaml: {name} has joints and tools: added {twin} '
                        f'{_show(joints)} (move_joints); {name} itself stays'
                        + ('' if known else ' (arm joints unknown: every joint)'))
                    if not known:
                        _flag(plan, say, f'robots/{robot}/poses.yaml {twin}: the profile gave no arm joints, the twin '
                                         'keeps every joint of the pose: remove the arm joints by hand')
                if changed:
                    plan.writes[path] = _dump(document, handler)
        for name, entry in poses.poses.get(robot, {}).items():     # combined poses of the robot pack: not ours to edit
            if poses.kind_of(entry) == 'both' and name not in own and f'{name}_joints' not in poses.poses[robot]:
                _flag(plan, say, f'robot {robot} pose {name} (robot pack) has joints and tools: add {name}_joints to '
                                 f'the pack by hand before a mission can use move_joints on it')


# ---- comments that still name an old skill -------------------------------------------------------------------------
def list_comments(plan: Plan, say) -> None:
    gone = {*plan.removes}
    for path in sorted(plan.cell.rglob('*.yaml')):
        if path in gone or 'moveit' in path.parts:
            continue
        try:
            lines = path.read_text(encoding='utf-8').splitlines()
        except (OSError, UnicodeDecodeError):
            continue
        for number, line in enumerate(lines, start=1):
            if COMMENT_WORDS.search(line):
                text = f'{plan.rel(path)}:{number}: {line.strip()}'
                plan.listed.append(text)
                say(f'comment {text}')


# ---- apply -------------------------------------------------------------------------------------------------------
def check_writable(plan: Plan) -> list[str]:
    """Problems that would stop the writing part-way (checked before anything is touched)."""
    problems = []
    for path in [*plan.writes, *plan.removes]:
        if path.is_symlink():
            problems.append(f'{plan.rel(path)} is a symbolic link: not touched')
        elif path.exists() and not os.access(path, os.W_OK):
            problems.append(f'{plan.rel(path)} is not writable')
        elif not os.access(path.parent, os.W_OK):
            problems.append(f'{plan.rel(path.parent)}/ is not writable')
    return problems


def apply(plan: Plan) -> None:
    staged = []
    try:
        for path, text in plan.writes.items():
            handle, name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
            with os.fdopen(handle, 'w', encoding='utf-8', newline='') as stream:
                stream.write(text)
            if path.exists():
                os.chmod(name, path.stat().st_mode & 0o7777)
            staged.append((name, path))
    except OSError:
        for name, _ in staged:
            Path(name).unlink(missing_ok=True)
        raise
    for name, path in staged:
        os.replace(name, path)
    for path in plan.removes:
        path.unlink(missing_ok=True)


def migrate_skills(cell: Path, robot_packs, dry_run: bool, say) -> None:
    """Migration 2 -> 3 (MIGRATIONS signature). Raises RuntimeError if a file cannot be written (nothing changed)."""
    do = 'would ' if dry_run else ''
    plan = Plan(cell)
    poses = PoseBook(cell, say)
    migrate_missions(plan, poses, say)
    migrate_operator(plan, poses, say)
    migrate_skill_yamls(plan, say)
    migrate_poses(plan, poses, say)
    list_comments(plan, say)
    problems = check_writable(plan)
    if problems and not dry_run:
        raise RuntimeError('; '.join(problems) + ': nothing was changed')
    for problem in problems:
        say(f'note    {problem} (a real run stops here)')
    if not dry_run:
        apply(plan)
    say(f'{"would write" if dry_run else "wrote"} {len(plan.writes)} file(s), {do}remove {len(plan.removes)}; '
        f'{len(plan.flags)} flagged step(s); {len(plan.listed)} comment(s) name an old skill')
    for flag in plan.flags:
        say(f'FLAG    {flag}')
    for text in plan.listed:
        say(f'comment {text}')
