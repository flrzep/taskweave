"""Cell pack migration 3 -> 4, the step part: the six motion skills (navigate_to, drive_straight, move_gripper,
move_object, set_posture, move_joints) become the one skill ``move`` (design: plans/p6-targets-move.md 4.1, 4.3, 6.2).

``StepConverter`` rewrites the args of one step (mission or operator action); the skill name, ids, instructions,
includes, ``with``, ``defaults`` stay. The references follow the new target syntax: a stance
``stance:<inst>[/<variant>]``
is ``<inst>/<variant>`` (the first variant of the robot where none was named), a location is its bare name, a pose its
name, ``<inst>/<point>`` stays. Targets that were renamed (a stance that clashed with an object target, a held pose
that clashed with a plain target) are followed. ``torso`` moved out of ``motion`` to the step; an object step that
did not name ``motion.torso: auto`` gets ``torso: keep`` so nothing changes (the move skill's default for an object is
auto). The ``book`` (migrate_targets.Book) knows the cell: which stances, poses and contacts exist, their new names.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

from .migrate_skills import _flow_of, _map, _seq

OLD_SKILLS = ('navigate_to', 'drive_straight', 'move_gripper', 'move_object', 'set_posture', 'move_joints')
STANCE = 'stance:'
LOCATION = 'location:'
POSE = 'pose:'
RELATIVE_HAND = ('by', 'frame', 'turn_deg', 'tilt_deg', 'at', 'rpy_deg')
SPLIT_HANDS = ('all', 'auto')


MIGRATE_COMMAND = 'taskweave config migrate'
REPLACED_BY_MOVE = {   # an old motion skill -> how the step reads with move
    'navigate_to': 'move {to: <base target>}',
    'drive_straight': 'move {to: <base target>, motion: {mode: straight}} or move {base: {by: [distance, 0]}}',
    'move_gripper': 'move {to: <hand target>} or move {hand: <hands>, by / at: ...}',
    'move_object': 'move {to: <object target>} or move {object: held, by / at: ...}',
    'set_posture': 'move {elbow: <side>}',
    'move_joints': 'move {to: <joint target>} or move {joints: {<group>: <value>}}',
    'move_to_pose': 'move {to: <joint or hand target>}',
    'go_to_point': 'move {to: <hand target>}',
}


def replaced_by_move_message(skill: str, cell: str = '<cell dir>') -> str | None:
    """The validation message for a step of a schema 4 cell that names one of the motion skills ``move`` replaced
    (they are not served any more); None for any other skill. It names the migration command."""
    if skill not in REPLACED_BY_MOVE:
        return None
    return (f'the skill {skill} was replaced by move on a cell of schema 4 (targets): write the step as '
            f'{REPLACED_BY_MOVE[skill]} (`{MIGRATE_COMMAND} {cell}` rewrites the steps of a cell of an older schema)')


class Unresolved(RuntimeError):
    """A step the migration cannot map: it stops (nothing is changed) and names the step."""


@dataclass
class Rewrite:
    """The new args of one step and the flags for the report."""

    args: object
    flags: list[str] = field(default_factory=list)


class StepConverter:
    def __init__(self, book, robot: str | None, mission: str | None = None):
        self.book, self.mission = book, mission
        self.robot = robot or book.default_robot
        self.flags: list[str] = []

    # -- entry --------------------------------------------------------------------------------------------------
    def convert(self, skill: str, args) -> Rewrite | None:
        """The rewrite of one step, or None if the step does not use one of the six skills."""
        if skill not in OLD_SKILLS:
            return None
        self.flags = []
        args = args if isinstance(args, Mapping) else _map({})
        new = getattr(self, f'_{skill}')(args)
        return Rewrite(new, list(self.flags))

    def _flag(self, text: str) -> None:
        self.flags.append(text)

    def _rest(self, args: Mapping, used: tuple, new) -> None:
        """Keys the old skill did not know stay in the new args, flagged."""
        extra = [key for key in args if key not in used]
        for key in extra:
            new[key] = args[key]
        if extra:
            self._flag(f'unknown old args kept as they are: {", ".join(extra)}')

    # -- references -----------------------------------------------------------------------------------------------
    def base_target(self, text) -> str:
        """navigate_to target / drive_straight to: ``stance:<inst>[/<variant>]``, ``location:<name>`` or a location."""
        if not isinstance(text, str):
            return text
        if text.startswith(STANCE):
            return self.book.stance_reference(self.robot, text[len(STANCE) :], self.mission, self._flag)
        if text.startswith(LOCATION):
            return text[len(LOCATION) :]
        return self.book.location_name(text)

    def pose_name(self, name):
        return self.book.pose_name(self.robot, name) if isinstance(name, str) else name

    def point_name(self, text):
        """``<instance>/<point>`` (hand target of a placed object)."""
        return text

    def _motion(self, args: Mapping, keep_torso: bool = False) -> tuple[object | None, object | None]:
        """(motion node without ``torso`` or None if nothing is left, the torso value or None)."""
        motion = args.get('motion')
        if not isinstance(motion, Mapping):
            return None, None
        torso = motion.get('torso')
        rest = _map({k: v for k, v in motion.items() if k != 'torso'}, _flow_of(motion))
        return (rest or None), torso

    def _via(self, motion):
        """motion.via items: ``pose:<name>`` -> the target name (a joint map item stays, flagged)."""
        via = motion.get('via') if isinstance(motion, Mapping) else None
        if not isinstance(via, list):
            return
        for index, item in enumerate(via):
            if isinstance(item, str):
                name = item[len(POSE) :] if item.startswith(POSE) else item
                via[index] = self.pose_name(name) if item.startswith(POSE) else name
            else:
                self._flag('motion.via has a joint map item: it was left as it is, name a joint target instead')

    # -- navigate_to ----------------------------------------------------------------------------------------------
    def _navigate_to(self, args: Mapping):
        new = _map({}, _flow_of(args))
        for key, value in args.items():
            if key == 'target':
                new['to'] = self.base_target(value)
            else:
                new[key] = value
                self._flag(f'unknown old args kept as they are: {key}')
        return new

    # -- drive_straight -------------------------------------------------------------------------------------------
    def _drive_straight(self, args: Mapping):
        flow = _flow_of(args)
        new = _map({}, flow)
        if 'to' in args:
            new['to'] = self.base_target(args['to'])
            new['motion'] = _map({'mode': 'straight'}, True if flow else False)
            self._rest(args, ('to', 'distance_m', 'sideways_m'), new)
            if 'sideways_m' in args:
                self._flag('sideways_m is not used with to: dropped')
            return new
        by = _seq([args.get('distance_m', 0), args.get('sideways_m', 0)])
        new['base'] = _map({'by': by}, True)
        self._rest(args, ('distance_m', 'sideways_m', 'to'), new)
        return new

    # -- move_joints ----------------------------------------------------------------------------------------------
    def _move_joints(self, args: Mapping):
        flow = _flow_of(args)
        new = _map({}, flow)
        for key, value in args.items():
            if key == 'pose':
                new['to'] = self.pose_name(value)
            elif key == 'joints':
                new['joints'] = self.book.joint_groups_value(self.robot, value, self._flag)
            elif key == 'motion':
                self._via(value)
                new['motion'] = value
            else:
                new[key] = value
        return new

    # -- move_gripper ---------------------------------------------------------------------------------------------
    def _move_gripper(self, args: Mapping):
        flow = _flow_of(args)
        to = args.get('to') if isinstance(args.get('to'), Mapping) else {}
        motion, torso = self._motion(args)
        self._via(motion)
        new = _map({}, flow)
        relative = any(key in to for key in RELATIVE_HAND)
        done_to = False
        for key, value in args.items():
            if key == 'to':
                if relative:
                    new['hand'] = self._hand(args.get('hands'))
                    for each in RELATIVE_HAND:
                        if each in to:
                            new[each] = to[each]
                    self._extras_of(to, RELATIVE_HAND, new)
                elif 'pose' in to:
                    new['to'] = self.pose_name(to['pose'])
                    self._extras_of(to, ('pose',), new)
                else:
                    new['to'] = self.point_name(to.get('point'))
                    if 'offset_m' in to:
                        new['offset_m'] = to['offset_m']
                    self._extras_of(to, ('point', 'offset_m'), new)
                done_to = True
            elif key == 'hands':
                if not relative:
                    new['hands'] = value
            elif key == 'motion':
                if motion is not None:
                    new['motion'] = motion
                if torso is not None:
                    new['torso'] = torso
            else:
                new[key] = value
                self._flag(f'unknown old args kept as they are: {key}')
        if not done_to:
            self._flag('move_gripper without to')
        return new

    def _extras_of(self, to: Mapping, used: tuple, new) -> None:
        extra = [key for key in to if key not in used]
        for key in extra:
            new[key] = to[key]
        if extra:
            self._flag(f'unknown old to keys kept as they are: {", ".join(extra)}')

    @staticmethod
    def _hand(hands):
        """The hands of a relative move: ``hand`` is a name, a list of names, or all."""
        if hands is None:
            return 'all'
        if isinstance(hands, str):
            return 'all' if hands in SPLIT_HANDS else hands
        names = list(hands)
        return names[0] if len(names) == 1 else _seq(names)

    # -- move_object ----------------------------------------------------------------------------------------------
    def _move_object(self, args: Mapping):
        flow = _flow_of(args)
        to = args.get('to') if isinstance(args.get('to'), Mapping) else {}
        motion, torso = self._motion(args)
        if torso is None:
            torso = 'keep'
        new = _map({}, flow)
        for key, value in args.items():
            if key == 'to':
                if 'pose' in to:
                    new['to'] = self.book.held_name(self.robot, to['pose'])
                    self._extras_of(to, ('pose',), new)
                elif 'onto' in to:
                    self._onto(to, new)
                else:
                    if 'object' not in args:
                        new['object'] = 'held'
                    for each in ('by', 'at', 'turn_deg', 'tilt_deg'):
                        if each in to:
                            new[each] = to[each]
                    self._extras_of(to, ('by', 'at', 'turn_deg', 'tilt_deg'), new)
            elif key == 'motion':
                if motion is not None:
                    new['motion'] = motion
                new['torso'] = torso
            elif key == 'object':
                new['object'] = value
            else:
                new[key] = value
        if 'motion' not in args:
            new['torso'] = torso
        return new

    def _onto(self, to: Mapping, new) -> None:
        onto = to['onto']
        contact = to.get('contact')
        target, reference = self.book.contact_target(onto, contact, self._flag)
        new['to'] = target
        if 'offset_m' in to:
            new['offset_m'] = to['offset_m']
        if reference is not None:
            new['reference'] = reference
        self._extras_of(to, ('onto', 'contact', 'offset_m'), new)

    # -- set_posture ----------------------------------------------------------------------------------------------
    def _set_posture(self, args: Mapping):
        new = _map({}, _flow_of(args))
        for key, value in args.items():
            new[key] = value
        return new


__all__ = ['OLD_SKILLS', 'Rewrite', 'StepConverter', 'Unresolved']
