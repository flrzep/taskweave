"""Which skills a skill server serves for a cell pack (targets layout or older), where a skill part's file is, and that
the catalogue and the reload map know the parts of a skill (skills/<skill>/<part>.yaml)."""

import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore, catalogue
from taskweave_core.config.store import skill_file

FIXTURES = Path(__file__).parent / 'fixtures'


def test_a_part_of_a_skill_has_its_file_in_the_skills_folder():
    assert skill_file('wait') == 'skills/wait.yaml'
    assert skill_file('move') == 'skills/move.yaml'
    assert skill_file('move.hand') == 'skills/move/hand.yaml'
    assert skill_file('move/hand') == 'skills/move/hand.yaml'


def test_the_store_reads_a_skill_part_by_either_spelling(tmp_path):
    root = tmp_path / 'cell'
    shutil.copytree(FIXTURES / 'targets_new', root)
    (root / 'skills' / 'move').mkdir()
    (root / 'skills' / 'move' / 'hand.yaml').write_text(
        'implementations: [{name: a, type: script, class: "x:Y"}]\nmotion: {approach_m: 0.12}\n')
    store = ConfigStore(root)
    assert store.skill_config('move.hand')['motion'] == {'approach_m': 0.12}
    assert store.skill_config('move/hand') == store.skill_config('move.hand')
    assert store.skill_config('move/base') == {}


def test_the_catalogue_lists_a_part_file_as_a_skill_file_that_reloads_between_goals(tmp_path):
    kind = catalogue.kind_of('skills/move/hand.yaml')
    assert kind.kind == 'skill_part' and kind.reload == ('reload_service',) and kind.schema is not None
    assert catalogue.kind_of('skills/move.yaml').kind == 'skill'


def test_the_names_a_skill_server_serves():
    pytest.importorskip('rclpy')
    from taskweave_core.skill_server import served_names, skill_of

    old = ['wait', 'navigate_to', 'drive_straight', 'move_gripper', 'move_object', 'set_posture', 'move_joints', 'grip']
    served, notes = served_names(old, targets=True)
    assert served == ['wait', 'move', 'grip'] and len(notes) == 6          # six folded into move, once
    assert served_names(['wait', 'move', 'grip'], targets=True) == (['wait', 'move', 'grip'], [])
    served, notes = served_names(['move', 'wait', 'navigate_to'], targets=False)
    assert served == ['wait', 'navigate_to'] and 'needs a cell pack in the targets layout' in notes[0]
    assert served_names(['custom'], targets=True)[0] == ['custom']          # other skills are served as asked
    assert skill_of('/cell/skills/move/hand.yaml') == 'move' and skill_of('skills/wait.yaml') == 'wait'
