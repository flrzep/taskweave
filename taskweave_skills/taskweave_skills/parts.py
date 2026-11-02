"""The parts of the ``move`` skill: the modules that do one kind of motion (navigate, straight, hand, carry, posture,
joints). They are the classes of the old motion skills (navigate_to, drive_straight, move_gripper, move_object,
set_posture, move_joints); ``Move`` builds them as ``move.<part>``, with the configuration of
``skills/move/<part>.yaml``, and calls them with what it resolved. ``carry`` moves what the hands hold: it does the
hand moves of a held object (kind hand). Before cell schema 5 navigate and carry were ``base`` and ``object``: a cell
of schema 4 keeps skills/move/base.yaml and object.yaml until it is migrated (ConfigStore reads the old name).

``RESOLVED`` is the one extra key of the arguments of such a call: a dict of Python objects (resolved targets, joint
values) that the module uses instead of looking a name up. It never comes from a mission step (it is in no args
schema), only from ``Move``.
"""

from __future__ import annotations

RESOLVED = '_resolved'
PARTS = {   # part -> the skill of the old layout that was this kind of motion
    'navigate': 'navigate_to',
    'straight': 'drive_straight',
    'hand': 'move_gripper',
    'carry': 'move_object',
    'posture': 'set_posture',
    'joints': 'move_joints',
}
PART_OF_OLD = {old: part for part, old in PARTS.items()}


def part_name(part: str) -> str:
    """The name of a part as a skill config key: ``move.hand``."""
    return f'move.{part}'


def part_key(old_name: str) -> str:
    """The skill config key (``select_implementation``, ``skill_params``) of a motion skill of the old layout:
    ``move.<part>``."""
    return part_name(PART_OF_OLD[old_name])


def in_move(module) -> bool:
    """Is this module built by ``Move`` (the targets layout)? Its messages then point at move's forms and options
    instead of at the old skills."""
    return str(getattr(module, 'name', '')).startswith('move.')
