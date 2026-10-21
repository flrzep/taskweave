"""The parts of the ``move`` skill: the modules that do one kind of motion (base, straight, hand, object, posture,
joints). They are the classes of the old motion skills (navigate_to, drive_straight, move_gripper, move_object,
set_posture, move_joints); in a cell in the targets layout (cell schema 4) ``Move`` builds them as ``move.<part>``,
with the configuration of ``skills/move/<part>.yaml``, and calls them with what it resolved. In an older cell the
same classes are the served skills under their old names and read their old arguments.

``RESOLVED`` is the one extra key of the arguments of such a call: a dict of Python objects (resolved targets, joint
values) that the module uses instead of looking a name up. It never comes from a mission step (it is in no args
schema), only from ``Move``.
"""

from __future__ import annotations

RESOLVED = '_resolved'
PARTS = {   # part -> the skill of the old layout that was this kind of motion
    'base': 'navigate_to',
    'straight': 'drive_straight',
    'hand': 'move_gripper',
    'object': 'move_object',
    'posture': 'set_posture',
    'joints': 'move_joints',
}
PART_OF_OLD = {old: part for part, old in PARTS.items()}


def part_name(part: str) -> str:
    """The name of a part as a skill config key: ``move.hand``."""
    return f'move.{part}'


def part_key(config, old_name: str) -> str:
    """The skill config key (``select_implementation``, ``skill_params``) of a motion skill of the old layout: its
    own name in an older cell, ``move.<part>`` in a cell in the targets layout."""
    return part_name(PART_OF_OLD[old_name]) if config.targets_format() else old_name


def in_move(module) -> bool:
    """Is this module built by ``Move`` (the targets layout)? Its messages then point at move's forms and options
    instead of at the old skills."""
    return str(getattr(module, 'name', '')).startswith('move.')
