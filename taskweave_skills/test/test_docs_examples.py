"""The example missions of docs/ (README.md, docs/missions.md) validate against the real skill schemas and a small
example cell (docs/examples/cell on top of the skills test fixture): skills exist, arguments fit their schemas, every
point, stance, contact and held pose exists, includes expand. The per-skill example steps of docs/skills.md are
checked the same way (test_skills_doc)."""

import copy
import shutil
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('moveit_msgs')

from taskweave_core.config import ConfigStore, packs  # noqa: E402
from taskweave_core.config.schemas import HAND_RULES, hands_schema  # noqa: E402
from taskweave_core.plan import Plan  # noqa: E402
from taskweave_core.validation import validate_plan  # noqa: E402
from taskweave_skills import shipped_skills  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
EXAMPLES = REPO / 'docs' / 'examples'
FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
ROBOT = 'duo_bot'
SKILLS = shipped_skills()    # every skill of the package: a new skill is in here without being registered

pytestmark = pytest.mark.skipif(not EXAMPLES.is_dir(), reason='docs/examples is not shipped with the package')


def catalog(store):
    """name -> args schema, with the robot's hands filled in like the skill servers do."""
    hands = list(store.robot(ROBOT).data.get('grippers', {}))
    out = {}
    for skill in SKILLS:
        schema = copy.deepcopy(skill.args_schema)
        if 'hands' in schema.get('properties', {}):
            rules = ('auto', 'all') if skill.name == 'grip' else getattr(skill, 'hand_rules', HAND_RULES)
            schema['properties']['hands'] = hands_schema(hands, rules)
        out[skill.name] = schema
    return out


@pytest.fixture
def cell(tmp_path, monkeypatch):
    monkeypatch.setattr(packs, 'installed_location', lambda: None)
    root = tmp_path / 'cell'
    shutil.copytree(FIXTURE, root)
    shutil.copytree(EXAMPLES / 'cell', root, dirs_exist_ok=True)
    shutil.copytree(EXAMPLES / 'missions', root / 'missions')
    return root


MISSIONS = sorted(path.stem for path in (EXAMPLES / 'missions').glob('*.yaml'))


def test_the_example_missions_exist():
    assert {'pick_and_place', 'one_hand', 'pick_part', 'restock', 'bin_to_fixture'} <= set(MISSIONS)


@pytest.mark.parametrize('name', MISSIONS)
def test_example_mission_validates(cell, name):
    store = ConfigStore(cell)
    plan = Plan.from_mission(store.expanded_mission(name), name, robot=ROBOT)
    assert plan.steps
    assert validate_plan(plan, catalog(store), store) == []


def test_the_example_cell_is_a_cell_pack_of_its_own(tmp_path, monkeypatch, capsys):
    """`taskweave check docs/examples/cell` works as it stands (pack.yaml, its own robot), and so with the missions."""
    from taskweave_core import cli

    monkeypatch.setattr(packs, 'installed_location', lambda: None)
    assert cli.check(str(EXAMPLES / 'cell')) == 0, capsys.readouterr().out
    root = tmp_path / 'cell'
    shutil.copytree(EXAMPLES / 'cell', root)
    shutil.copytree(EXAMPLES / 'missions', root / 'missions')
    assert cli.check(str(root)) == 0, capsys.readouterr().out


def test_the_included_block_is_expanded_into_the_mission(cell):
    store = ConfigStore(cell)
    plan = Plan.from_mission(store.expanded_mission('restock'), 'restock', robot=ROBOT)
    assert [step.id for step in plan.steps][:2] == ['pick_s1', 'pick_s2']
    assert plan.steps[0].skill == 'navigate_to' and plan.steps[-1].skill == 'grip'


def skill_steps():
    import yaml

    data = yaml.safe_load((EXAMPLES / 'skill_steps.yaml').read_text(encoding='utf-8'))
    return [(name, step) for name, steps in data.items() for step in steps]


@pytest.mark.parametrize('name,step', skill_steps(), ids=lambda value: value if isinstance(value, str) else
                         value['instruction'][:30])
def test_every_example_step_of_the_skill_reference_validates(cell, name, step):
    store = ConfigStore(cell)
    assert step['skill'] == name
    plan = Plan.from_mission({'goal': 'one step', 'steps': [step]}, 'one_step', robot=ROBOT)
    assert validate_plan(plan, catalog(store), store) == []


def test_the_missions_in_the_docs_are_the_validated_files():
    """README.md and docs/missions.md show the example files verbatim, so what they print is what was validated."""
    shown = {'README.md': ['readme_example'], 'docs/missions.md': ['pick_and_place', 'one_hand', 'pick_part',
                                                                    'restock', 'bin_to_fixture']}
    for doc, names in shown.items():
        text = (REPO / doc).read_text(encoding='utf-8')
        for name in names:
            source = (EXAMPLES / 'missions' / f'{name}.yaml').read_text(encoding='utf-8').rstrip('\n')
            assert source in text, f'{doc} does not show docs/examples/missions/{name}.yaml as it is'


def test_relative_links_and_documented_paths_exist():
    import re

    for doc in ['README.md', 'AGENTS.md', 'docs/missions.md', 'docs/skills.md', 'docs/configuration.md',
                'docs/architecture.md']:
        path = REPO / doc
        text = path.read_text(encoding='utf-8')
        for target in re.findall(r'\]\(([^)#\s]+)(?:#[^)]*)?\)', text):
            if '://' in target:
                continue
            assert (path.parent / target).exists(), f'{doc}: broken link {target}'
    agents = (REPO / 'AGENTS.md').read_text(encoding='utf-8')
    for target in re.findall(r'`((?:docs|tools|taskweave_\w+)/[\w./-]*\w)`', agents):
        assert '<' not in target and (REPO / target).exists(), f'AGENTS.md: {target} does not exist'
