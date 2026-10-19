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


def test_every_shipped_skill_has_a_template_and_an_example_yaml(generator):
    """AGENTS.md "add a skill": config/skill_templates.py (what taskweave config migrate-skills starts from) and
    docs/examples/skills/<name>.yaml, kept equal by test_migrate_skills."""
    from taskweave_core.config.skill_templates import TEMPLATES

    names = {skill.name for skill in generator.load_skills()}
    assert names == set(TEMPLATES), 'skill_templates.TEMPLATES needs one entry per skill'
    for name in names:
        assert (REPO / 'docs' / 'examples' / 'skills' / f'{name}.yaml').is_file(), f'docs/examples/skills/{name}.yaml'


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
