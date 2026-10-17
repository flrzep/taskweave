"""Agnostic-code checks: how far the code is from robot-, cell- and object-agnostic (all of that belongs in YAML).

Each check counts findings in the non-test code of taskweave_core, taskweave_skills and taskweave_hmi:
names (robot/cell/object words, code and comments counted apart), unnamed numbers in the skill and motion
modules, module tuning constants (to move into config) and arm structure assumptions (fixed joint indices).

Mode, by env TASKWEAVE_STRICT_AGNOSTIC:
  unset   every check passes and warns with its count and first findings (see them with ``pytest -rw``)
  1       a check fails when its count is above BASELINE (counts may only go down; lower BASELINE as they do)
  zero    a check fails on any finding
"""

import ast
import functools
import io
import os
import re
import tokenize
import warnings
from pathlib import Path

import pytest

# robot, cell and object words that must not appear in code (case-insensitive; '_' counts as a word break)
FORBIDDEN = ['galbot', 's1', 'g1', 'a2d', 'agibot', 'adient', 'frame_type', 'rails', 'karakuri', 'hanger', 'knob',
             'bay_', 'stack_2']

# counts on 16 Oct 2026; strict mode (=1) fails above these
BASELINE = {'names_code': 0, 'names_comments': 0, 'numbers': 0, 'constants': 0, 'arm_structure': 0}

ALLOWED_NUMBERS = {0, 1, 2, 3, 4, 0.5, 100, 1000, 1e6, 1e9, 180, 360}  # by abs value; plus epsilons <= 1e-6
PROTOCOL_ARGS = {'create_publisher': 2, 'create_subscription': 3, 'round': 1}  # from this arg on: QoS depth, digits
PROTOCOL_KEYWORDS = {'throttle_duration_sec'}  # keyword arguments that are not behaviour: log throttling
SCHEMA_KEYS = {'minimum', 'maximum', 'exclusiveMinimum', 'exclusiveMaximum', 'minItems', 'maxItems', 'multipleOf'}

# module UPPER_CASE numbers that are protocol, not tuning
PROTOCOL = {
    'SUCCESS': 'MoveIt error code',
    'FRAME_TRANSFORM_FAILURE': 'MoveIt error code',
    'RETRY': 'MoveIt error codes worth a retry',
    'CHECK_BATCH': 'requests in flight to the state validity service',
    'FRESH_S': 'jog command lifetime (wall clock), part of the jog topic protocol',
    'ELBOW_WORDS': 'word -> unit direction vocabulary of ik.preferred_elbow',
}
PROTOCOL_SUFFIXES = ('POLL_S',)  # service/action poll periods
# algorithm constants kept in code on purpose (P1 review): numerical guards and fixed conventions of an algorithm,
# nothing a robot or cell would tune
ALGORITHM = {
    'MIN_DT_S': 'smallest time step a controller divides by',
    'REVERSE_HYSTERESIS': 'drive direction switch hysteresis of the base controller',
    'SWING_STRAIGHT_STEPS': 'interpolation steps of the straight leg after an elbow swing',
    'MIN_SPAN': 'smallest joint span a plan check divides by',
    'MAX_STEP_S': 'largest jog integration step',
    'FOLLOW_M': 'jog target follow tolerance (m)',
    'FOLLOW_RAD': 'jog target follow tolerance (rad)',
    'SWIVEL_SIGN': 'direction convention of the jog swivel axis',
}

ROOT = Path(__file__).resolve().parents[2]
PACKAGES = ['taskweave_core', 'taskweave_skills', 'taskweave_hmi']
MOTION = sorted((ROOT / 'taskweave_skills' / 'taskweave_skills').glob('*.py')) + [
    ROOT / 'taskweave_hmi' / 'taskweave_hmi' / 'arm.py']
SHOW = int(os.environ.get('TASKWEAVE_AGNOSTIC_SHOW', 10))


def rel(path) -> str:
    return Path(path).relative_to(ROOT).as_posix()


def sources(suffixes=('.py',)) -> list[Path]:
    out = []
    for package in PACKAGES:
        for path in sorted((ROOT / package).rglob('*')):
            parts = path.relative_to(ROOT / package).parts
            if path.suffix in suffixes and 'test' not in parts and '__pycache__' not in parts and path.is_file():
                out.append(path)
    return out


def report(key: str, findings: list[str]):
    mode = os.environ.get('TASKWEAVE_STRICT_AGNOSTIC', '')
    text = f'agnostic {key}: {len(findings)} (baseline {BASELINE[key]})' + ''.join(f'\n  {f}' for f in findings[:SHOW])
    if len(findings) > SHOW:
        text += f'\n  ... {len(findings) - SHOW} more'
    if mode == 'zero':
        assert not findings, text
    elif mode:
        assert len(findings) <= BASELINE[key], text
    if findings:
        warnings.warn(text, stacklevel=2)


# ---- 1. names

def word_pattern(words) -> re.Pattern:
    # 'bay_' runs on; words ending in a letter may take a plural or a number (frame_type2); 's1' stands alone
    parts = [w if w.endswith('_') else rf'{w}(?![a-z0-9])' if w[-1].isdigit() else rf'{w}s?\d*(?![a-z0-9])'
             for w in words]
    return re.compile(rf'(?<![a-z0-9])(?:{"|".join(parts)})', re.IGNORECASE)


WORDS = word_pattern(FORBIDDEN)


def docstring_lines(tree) -> set[int]:
    lines = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Module | ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef) and node.body:
            first = node.body[0]
            if isinstance(first, ast.Expr) and isinstance(getattr(first.value, 'value', None), str):
                lines.add(first.lineno)
    return lines


def python_names(path: Path):
    """(line, word, is_comment) for each forbidden word: comments and docstrings count as comments."""
    text = path.read_text(encoding='utf-8')
    docs = docstring_lines(ast.parse(text))
    for tok in tokenize.generate_tokens(io.StringIO(text).readline):
        for m in WORDS.finditer(tok.string):
            comment = tok.type == tokenize.COMMENT or (tok.type == tokenize.STRING and tok.start[0] in docs)
            yield tok.start[0] + tok.string.count('\n', 0, m.start()), m.group(), comment


def web_names(path: Path):
    """Same for JS and HTML; a crude comment test (//, /* */, <!-- -->) is good enough for counting."""
    block = False
    for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        starts_block = '/*' in line or '<!--' in line
        for m in WORDS.finditer(line):
            before = line[:m.start()]
            comment = block or '//' in before.replace('://', '') or '/*' in before or '<!--' in before \
                or line.lstrip().startswith('*')
            yield number, m.group(), comment
        if starts_block and not re.search(r'\*/|-->', line.split('/*')[-1].split('<!--')[-1]):
            block = True
        elif block and re.search(r'\*/|-->', line):
            block = False


@functools.cache
def name_findings():
    code, comments = [], []
    for path in sources(('.py',)) + sources(('.js', '.html')):
        for line, word, comment in (python_names(path) if path.suffix == '.py' else web_names(path)):
            (comments if comment else code).append(f'{rel(path)}:{line} {word}')
    return code, comments


def test_names_in_code():
    report('names_code', name_findings()[0])


def test_names_in_comments():
    report('names_comments', name_findings()[1])


# ---- 2. unnamed numbers

def is_upper(target) -> bool:
    if isinstance(target, ast.Tuple):
        return all(is_upper(e) for e in target.elts)
    return isinstance(target, ast.Name) and target.id.isupper()


def is_schema(node: ast.Dict) -> bool:
    return any(isinstance(k, ast.Constant) and k.value in SCHEMA_KEYS for k in node.keys)


def named_nodes(tree) -> set[int]:
    """ids of nodes whose numbers are named: UPPER constants, class fields, parameter defaults, schemas, indices."""
    roots = []
    for node in tree.body:
        if isinstance(node, ast.Assign) and all(is_upper(t) for t in node.targets):
            roots.append(node.value)
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            roots += [n for n in node.body if isinstance(n, ast.Assign | ast.AnnAssign)]
        elif isinstance(node, ast.arguments):
            roots += node.defaults + [d for d in node.kw_defaults if d is not None]
        elif isinstance(node, ast.Assign) and any('schema' in ast.unparse(t).lower() for t in node.targets):
            roots.append(node.value)
        elif isinstance(node, ast.Dict) and is_schema(node):
            roots.append(node)
        elif isinstance(node, ast.Subscript):
            roots.append(node.slice)
        elif isinstance(node, ast.Call):
            func = node.func.attr if isinstance(node.func, ast.Attribute) else getattr(node.func, 'id', '')
            roots += node.args[PROTOCOL_ARGS[func]:] if func in PROTOCOL_ARGS else []
            roots += [k.value for k in node.keywords if k.arg in PROTOCOL_KEYWORDS]
    return {id(n) for root in roots for n in ast.walk(root)}


def number_of(node):
    if isinstance(node, ast.Constant) and type(node.value) in (int, float):
        return node.value
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub) and isinstance(node.operand, ast.Constant) \
            and type(node.operand.value) in (int, float):
        return -node.operand.value
    return None


def by_line(finding: str):
    path, rest = finding.split(':', 1)
    return path, int(rest.split()[0])


def allowed(value) -> bool:
    return abs(value) in ALLOWED_NUMBERS or abs(value) <= 1e-6


def test_unnamed_numbers():
    findings = []
    for path in MOTION:
        tree = ast.parse(path.read_text(encoding='utf-8'))
        named = named_nodes(tree)
        negated = {id(n.operand) for n in ast.walk(tree) if number_of(n) is not None and isinstance(n, ast.UnaryOp)}
        for node in ast.walk(tree):
            value = number_of(node)
            if value is not None and id(node) not in named and id(node) not in negated and not allowed(value):
                findings.append(f'{rel(path)}:{node.lineno} {value}')
    report('numbers', sorted(findings, key=by_line))


# ---- 3. module tuning constants

def numeric(node) -> bool:
    if number_of(node) is not None:
        return True
    if isinstance(node, ast.UnaryOp | ast.BinOp):
        return all(numeric(n) for n in ([node.operand] if isinstance(node, ast.UnaryOp) else [node.left, node.right]))
    if isinstance(node, ast.Call):  # math.radians(150)
        return bool(node.args) and all(numeric(a) for a in node.args)
    if isinstance(node, ast.Tuple | ast.List | ast.Set):
        return bool(node.elts) and all(numeric(e) for e in node.elts)
    if isinstance(node, ast.Dict):
        return bool(node.values) and all(numeric(v) for v in node.values)
    return isinstance(node, ast.Attribute) and ast.unparse(node) in ('math.pi', 'math.inf')


def test_module_constants():
    findings = []
    for path in MOTION:
        for node in ast.parse(path.read_text(encoding='utf-8')).body:
            if not (isinstance(node, ast.Assign) and all(is_upper(t) for t in node.targets)):
                continue
            target = node.targets[0]
            pairs = zip(target.elts, node.value.elts, strict=False) \
                if isinstance(target, ast.Tuple) and isinstance(node.value, ast.Tuple) else [(target, node.value)]
            for name, value in pairs:
                if isinstance(name, ast.Name) and numeric(value) and name.id not in {**PROTOCOL, **ALGORITHM} \
                        and not name.id.endswith(PROTOCOL_SUFFIXES):
                    findings.append(f'{rel(path)}:{node.lineno} {name.id} = {ast.unparse(value)}')
    report('constants', findings)


# ---- 4. arm structure assumptions

def joints(node) -> bool:
    return bool(re.search(r'name|joint', ast.unparse(node), re.I))


def test_arm_structure():
    """Fixed indices into joint-name lists (names[1], names[3], names[5]) and joint counts (len(names) < 7) assume one
    arm layout."""
    findings = []
    for path in MOTION:
        tree, owner = ast.parse(path.read_text(encoding='utf-8')), {}
        for func in ast.walk(tree):  # outer functions first, so the innermost one wins
            if isinstance(func, ast.FunctionDef | ast.AsyncFunctionDef):
                owner.update({id(n): func.name for n in ast.walk(func)})
        for node in ast.walk(tree):
            pick = isinstance(node, ast.Subscript) and isinstance(node.value, ast.Name | ast.Attribute) \
                and joints(node.value) and number_of(node.slice) is not None
            count = isinstance(node, ast.Compare) and isinstance(node.left, ast.Call) \
                and ast.unparse(node.left.func) == 'len' and joints(node.left) \
                and any(number_of(c) not in (None, 0, 1) for c in node.comparators)
            if pick or count:
                findings.append(f'{rel(path)}:{node.lineno} {path.stem}.{owner.get(id(node), "")} {ast.unparse(node)}')
    report('arm_structure', sorted(findings, key=by_line))


def test_sources_found():
    if not (ROOT / 'taskweave_skills').is_dir():
        pytest.skip('taskweave_skills not next to taskweave_core')
    assert sources() and MOTION
