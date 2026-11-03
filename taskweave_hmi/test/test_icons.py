"""The step icons (web/icons/*.svg, web/icons.js): every icon icons.js names has a file, the files are clean 24 px
icons in currentColor, icons.js holds them inline unchanged, and every skill and move kind has an icon (static: reads
the sources, no ROS). ``python3 test_icons.py --regenerate`` rewrites the inline block of icons.js from the files."""

import ast
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
ICONS = WEB / 'icons'
ICONS_JS = WEB / 'icons.js'
ROOT = Path(__file__).resolve().parents[2]
SKILLS = ROOT / 'taskweave_skills' / 'taskweave_skills'
TARGETS = ROOT / 'taskweave_core' / 'taskweave_core' / 'config' / 'targets.py'
EXPECTED = {'navigate', 'straight', 'hand', 'carry', 'posture', 'joints', 'grip-close', 'grip-open', 'wait', 'other'}
BY_KIND = {'move', 'grip'}   # skills icons.js picks by kind (move) or by args.action (grip), not from SKILL_ICONS
OTHER = set()                # skills that show the other icon on purpose
COLOURS = {'currentColor', 'none'}
MAX_BYTES = 2048
SVG_NS = '{http://www.w3.org/2000/svg}'
BEGIN, END = '// ICON_SVG begin', '// ICON_SVG end'


def js() -> str:
    return ICONS_JS.read_text(encoding='utf-8')


def js_map(name: str) -> dict[str, str]:
    """A flat ``const NAME = { key: 'value', ... };`` of icons.js."""
    body = re.search(rf'const {name} = \{{(.*?)\}};', js(), re.S).group(1)
    return dict(re.findall(r"'?([\w-]+)'?:\s*'([^']*)'", body))


def referenced() -> set[str]:
    """Every icon id icons.js can answer: the values of its maps and the ids it returns literally."""
    ids = {v for m in ('KIND_ICONS', 'SKILL_ICONS', 'GRIP_ICONS') for v in js_map(m).values()}
    code = js().split(BEGIN)[0]
    ids |= set(re.findall(r"(?:return|\?|\|\|)\s*'([\w-]+)'\s*[;:]", code))
    return ids


def inline_block() -> str:
    lines = [f"  '{p.stem}': '{' '.join(p.read_text(encoding='utf-8').split())}',"
             for p in sorted(ICONS.glob('*.svg'))]
    return '\n'.join([BEGIN + ' (generated from web/icons/*.svg by test_icons.py --regenerate)',
                      'const ICON_SVG = {', *lines, '};', ''])


def regenerate() -> None:
    text = js()
    start, end = text.index(BEGIN), text.index(END)
    ICONS_JS.write_text(text[:start] + inline_block() + text[end:], encoding='utf-8', newline='\n')


def skill_names() -> set[str]:
    """The skills of taskweave_skills: classes with a string ``name`` and an ``args_schema``."""
    names = set()
    for file in SKILLS.glob('*.py'):
        for node in ast.walk(ast.parse(file.read_text(encoding='utf-8'))):
            if not isinstance(node, ast.ClassDef):
                continue
            attrs = {t.id: s.value for s in node.body if isinstance(s, ast.Assign)
                     for t in s.targets if isinstance(t, ast.Name)}
            if isinstance(attrs.get('name'), ast.Constant) and 'args_schema' in attrs:
                names.add(attrs['name'].value)
    return names


def module_constant(file: Path, name: str):
    for node in ast.parse(file.read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(name)


def test_the_files_are_the_icons():
    assert {p.stem for p in ICONS.glob('*.svg')} == EXPECTED


def test_every_icon_icons_js_names_has_a_file():
    ids = referenced()
    assert 'other' in ids and 'carry' in ids
    assert ids <= {p.stem for p in ICONS.glob('*.svg')}


@pytest.mark.parametrize('name', sorted(EXPECTED))
def test_svg_is_clean(name):
    file = ICONS / f'{name}.svg'
    assert file.stat().st_size <= MAX_BYTES
    root = ET.parse(file).getroot()
    assert root.tag == f'{SVG_NS}svg'
    assert root.get('viewBox') == '0 0 24 24'
    assert root.get('width') is None and root.get('height') is None
    for node in root.iter():
        assert node.get('id') is None and node.get('style') is None and node.get('class') is None, name
        for attr in ('fill', 'stroke', 'color', 'stop-color'):
            assert node.get(attr) in (None, *COLOURS), f'{name}: {attr}={node.get(attr)}'
        assert node.tag in (f'{SVG_NS}svg', f'{SVG_NS}path', f'{SVG_NS}circle', f'{SVG_NS}rect', f'{SVG_NS}g'), name
    assert root.get('stroke') == 'currentColor' and root.get('stroke-width') == '2'
    assert root.get('stroke-linecap') == 'round'


def test_icons_js_holds_the_files_inline():
    text = js()
    assert text[text.index(BEGIN):text.index(END)] == inline_block(), \
        'icons.js is stale: python3 taskweave_hmi/test/test_icons.py --regenerate'


def test_every_skill_has_an_icon():
    skills = skill_names()
    assert {'move', 'grip', 'wait'} <= skills
    mapped = js_map('SKILL_ICONS')
    assert skills - set(mapped) - BY_KIND - OTHER == set()
    assert set(mapped) <= skills, 'SKILL_ICONS names a skill that does not exist'


def test_every_move_kind_and_grip_action_has_an_icon():
    kinds = js_map('KIND_ICONS')
    assert set(module_constant(TARGETS, 'MOVE_FORMS')) <= set(kinds)
    assert set(module_constant(TARGETS, 'OLD_KINDS')) <= set(kinds)
    assert module_constant(TARGETS, 'HOLDING') == re.search(r"const HOLDING = '(\w+)'", js()).group(1)
    schema = re.search(r"'action': \{'enum': (\[[^\]]*\])", (SKILLS / 'gripper_motion.py').read_text(encoding='utf-8'))
    assert set(ast.literal_eval(schema.group(1))) == set(js_map('GRIP_ICONS'))


if __name__ == '__main__' and '--regenerate' in sys.argv:
    regenerate()
