"""docs/skills.md is generated (tools/gen_skills_doc.py) from the skill classes: it must match a regeneration, and it
must cover every skill with an example."""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

REPO = Path(__file__).resolve().parents[2]
GENERATOR = REPO / 'tools' / 'gen_skills_doc.py'

pytestmark = pytest.mark.skipif(not GENERATOR.exists(), reason='tools/ and docs/ are not shipped with the package')


@pytest.fixture(scope='module')
def generator():
    spec = importlib.util.spec_from_file_location('gen_skills_doc', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_skills_md_is_up_to_date(generator):
    current = generator.OUTPUT.read_text(encoding='utf-8')
    assert current == generator.generate(), 'docs/skills.md is out of date: run python3 tools/gen_skills_doc.py'


def test_every_skill_is_documented_with_an_example(generator):
    skills = generator.load_skills()
    data = generator.examples()
    assert {skill.name for skill in skills} == set(data), 'docs/examples/skill_steps.yaml needs one entry per skill'
    text = generator.OUTPUT.read_text(encoding='utf-8')
    for skill in skills:
        assert f'## `{skill.name}`' in text


def test_the_skills_of_the_docs_are_wait_move_grip(generator):
    """The catalogue of a cell in the targets layout: three skills. The classes of the six motion skills ``move``
    replaced are its parts, not documented as skills."""
    assert {skill.name for skill in generator.load_skills()} == {'wait', 'move', 'grip'}
    text = generator.OUTPUT.read_text(encoding='utf-8')
    for part in ('base', 'straight', 'hand', 'object', 'posture', 'joints'):
        assert f'`move.{part}`' in text and f'skills/move/{part}.yaml' in text


def test_every_shipped_skill_has_a_skill_yaml_in_the_example_cell(generator):
    """AGENTS.md "add a skill": docs/examples/skills/<name>.yaml is what a cell pack's skill file starts from, and
    what the reference shows as `requires`. ``move`` also has its parts in docs/examples/skills/move/."""
    names = {skill.name for skill in generator.load_skills()}
    for name in names:
        assert (REPO / 'docs' / 'examples' / 'skills' / f'{name}.yaml').is_file(), f'docs/examples/skills/{name}.yaml'
    assert set(generator.parts('move')) == {'base', 'straight', 'hand', 'object', 'posture', 'joints'}


def test_the_launch_example_names_every_shipped_skill(generator):
    """The README's launch command lists the skills to serve: a new skill must be in it."""
    import re

    text = (REPO / 'README.md').read_text(encoding='utf-8')
    listed = set(re.search(r'skills:=([\w,]+)', text).group(1).split(','))
    assert listed == {skill.name for skill in generator.load_skills()}


def test_the_generation_needs_no_ros_graph(generator):
    # classes are read, never constructed: every skill class has the attributes the reference prints
    for skill in generator.load_skills():
        assert skill.name and skill.description and isinstance(skill.args_schema, dict)
