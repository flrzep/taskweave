"""Mission edits from the graph editor: operator-style ops applied on the ruamel tree (comments kept), then
validated and saved as a whole (missions.edit)."""

import shutil
from pathlib import Path

import pytest
from ruamel.yaml import YAML

from taskweave_core.config import ConfigStore
from taskweave_hmi.missions import MissionError, apply_ops, edit, save

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'config'
CATALOG = {
    'move': {'type': 'object', 'required': ['to'],
             'properties': {'to': {'type': 'string', 'x-taskweave-point': 'target',
                                   'x-taskweave-target-kinds': ['base']}}},
    'wait': {'type': 'object', 'required': ['seconds'], 'properties': {'seconds': {'type': 'number', 'minimum': 0}}},
}
WAIT = {'instruction': 'Wait a little', 'skill': 'wait', 'args': {'seconds': 2}}


@pytest.fixture
def store(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


def doc(text):
    return YAML().load(text)


def plain(node):
    """A ruamel tree as plain dicts and lists (what the browser sees)."""
    if isinstance(node, dict):
        return {k: plain(v) for k, v in node.items()}
    if isinstance(node, list):
        return [plain(v) for v in node]
    return node


# ---- the ops on a tree (no validation) ----
THREE = 'goal: g\nsteps:\n  - {id: a, instruction: A, skill: wait, args: {seconds: 1}}\n' \
        '  - {id: b, instruction: B, skill: wait, args: {seconds: 1}}\n' \
        '  - {id: c, instruction: C, skill: wait, args: {seconds: 1}}\n'
ids = lambda d: [s.get('id') for s in d['steps']]  # noqa: E731


def test_insert_remove_and_set():
    d = doc(THREE)
    apply_ops(d, [{'op': 'insert', 'at': 'steps/1', 'step': {'id': 'x', **WAIT}}])
    assert ids(d) == ['a', 'x', 'b', 'c']
    apply_ops(d, [{'op': 'insert', 'at': 'steps/4', 'step': {'id': 'y', **WAIT}}])  # at the end
    assert ids(d) == ['a', 'x', 'b', 'c', 'y']
    apply_ops(d, [{'op': 'remove', 'at': 'steps/1'}, {'op': 'remove', 'at': 'steps/3'}])
    assert ids(d) == ['a', 'b', 'c']
    apply_ops(d, [{'op': 'set', 'at': 'steps/0', 'key': 'on_failure', 'value': 'skip'},
                  {'op': 'set', 'at': 'steps/0', 'key': 'id', 'value': None},
                  {'op': 'set', 'at': '', 'key': 'goal', 'value': 'new goal'},
                  {'op': 'set', 'at': 'steps/1', 'key': 'args', 'value': {'seconds': 5}}])
    assert plain(d['steps'][0]) == {'instruction': 'A', 'skill': 'wait', 'args': {'seconds': 1}, 'on_failure': 'skip'}
    assert d['goal'] == 'new goal' and plain(d['steps'][1]['args']) == {'seconds': 5}


def test_move_names_the_slot_before_the_move():
    d = doc(THREE)
    apply_ops(d, [{'op': 'move', 'from': 'steps/0', 'to': 'steps/3'}])  # to the end
    assert ids(d) == ['b', 'c', 'a']
    apply_ops(d, [{'op': 'move', 'from': 'steps/2', 'to': 'steps/0'}])  # to the front
    assert ids(d) == ['a', 'b', 'c']
    apply_ops(d, [{'op': 'move', 'from': 'steps/0', 'to': 'steps/1'}])  # before b: where it is
    assert ids(d) == ['a', 'b', 'c']
    apply_ops(d, [{'op': 'move', 'from': 'steps/0', 'to': 'steps/2'}])  # before c
    assert ids(d) == ['b', 'a', 'c']


def test_wrap_and_unwrap_blocks():
    d = doc(THREE)
    apply_ops(d, [{'op': 'wrap', 'from': 'steps/0', 'to': 'steps/1', 'as': 'group',
                   'block': {'id': 'both', 'instruction': 'A and B'}}])
    assert ids(d) == ['both', 'c']
    assert plain(d['steps'][0]) == {'id': 'both', 'instruction': 'A and B', 'group': True, 'steps': [
        {'id': 'a', 'instruction': 'A', 'skill': 'wait', 'args': {'seconds': 1}},
        {'id': 'b', 'instruction': 'B', 'skill': 'wait', 'args': {'seconds': 1}}]}
    # a step inside the block, by its path
    apply_ops(d, [{'op': 'set', 'at': 'steps/0/steps/1', 'key': 'retries', 'value': 2}])
    assert d['steps'][0]['steps'][1]['retries'] == 2
    apply_ops(d, [{'op': 'move', 'from': 'steps/1', 'to': 'steps/0/steps/0'}])  # c into the block, first
    assert [s['id'] for s in d['steps'][0]['steps']] == ['c', 'a', 'b'] and ids(d) == ['both']
    apply_ops(d, [{'op': 'unwrap', 'at': 'steps/0'}])
    assert ids(d) == ['c', 'a', 'b']
    apply_ops(d, [{'op': 'wrap', 'from': 'steps/1', 'to': 'steps/2', 'as': 'repeat',
                   'block': {'instruction': 'Twice', 'repeat': {'times': 2}}}])
    assert plain(d['steps'][1])['repeat'] == {'times': 2} and len(d['steps'][1]['steps']) == 2
    apply_ops(d, [{'op': 'wrap', 'from': 'steps/0', 'to': 'steps/0', 'as': 'parallel',
                   'block': {'instruction': 'Lanes'}}])
    block = plain(d['steps'][0])
    assert block['parallel'] == [{'steps': [{'id': 'c', 'instruction': 'C', 'skill': 'wait', 'args': {'seconds': 1}}]}]
    apply_ops(d, [{'op': 'insert', 'at': 'steps/0/parallel/1', 'step': {'instruction': 'Lane 2', 'steps': []}}])
    assert len(d['steps'][0]['parallel']) == 2
    apply_ops(d, [{'op': 'unwrap', 'at': 'steps/0'}])  # a parallel unwraps to its lanes' steps in order
    assert ids(d) == ['c', None]  # lane 2 was empty; the repeat block (no id) follows
    assert d['steps'][1]['repeat']


def test_bad_ops_are_refused_whole():
    d = doc(THREE)
    for op in ({'op': 'remove', 'at': 'steps/9'}, {'op': 'remove', 'at': 'goal'}, {'op': 'fly', 'at': 'steps/0'},
               {'op': 'move', 'from': 'steps/0', 'to': 'steps/0/steps/0'},
               {'op': 'set', 'at': 'steps/0', 'key': 'steps', 'value': 1},
               {'op': 'insert', 'at': 'steps/1', 'step': 'text'}, {'op': 'unwrap', 'at': 'steps/1'}):
        with pytest.raises(MissionError):
            apply_ops(d, [op])
    assert ids(d) == ['a', 'b', 'c']


# ---- edit: ops on the file, validated, saved ----
def test_edit_keeps_comments_and_validates(store):
    path = store.root / 'missions' / 'patrol.yaml'
    path.write_text('# my patrol\n' + path.read_text().replace(
        '  - instruction: Wait for the operator', '  # the pause\n  - instruction: Wait for the operator'))
    store.reload()
    result = edit('patrol', [{'op': 'insert', 'at': 'steps/1', 'step': WAIT},
                             {'op': 'set', 'at': 'steps/0', 'key': 'on_failure', 'value': 'skip'}],
                  store, CATALOG, 'demo_bot')
    text = path.read_text()
    assert text.startswith('# my patrol') and '# the pause' in text
    first_two = [s['instruction'] for s in result['mission']['steps']][:2]
    assert first_two == ['Drive to the stance in front of shelf A', 'Wait a little']
    assert result['mission']['steps'][0]['on_failure'] == 'skip' and result['file'] == 'missions/patrol.yaml'
    assert ConfigStore(store.root).mission('patrol')['steps'][1] == WAIT
    # a refused edit writes nothing and names the step
    with pytest.raises(MissionError) as error:
        edit('patrol', [{'op': 'set', 'at': 'steps/0', 'key': 'args', 'value': {'to': 'nowhere'}}],
             store, CATALOG, 'demo_bot')
    assert any("unknown target 'nowhere'" in p for p in error.value.problems) and error.value.places
    assert path.read_text() == text


def test_edit_makes_a_new_mission_from_ops(store):
    go = {'instruction': 'Go {where}', 'skill': 'move', 'args': {'to': '{where}'}}
    result = edit('fresh', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'A new one'},
                            {'op': 'set', 'at': '', 'key': 'params', 'value': {'where': {'point': 'target'}}},
                            {'op': 'insert', 'at': 'steps/0', 'step': go}],
                  store, CATALOG, 'demo_bot')
    assert result['mission']['goal'] == 'A new one' and 'fresh' in store.mission_names()
    # params are written too (they were dropped from new files before)
    assert ConfigStore(store.root).mission('fresh')['params'] == {'where': {'point': 'target'}}


def test_edit_renames(store):
    edit('night', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'At night'}], store, CATALOG, 'demo_bot',
         rename_from='patrol')
    assert 'patrol' not in store.mission_names() and store.mission('night')['goal'] == 'At night'


def test_save_writes_params_and_facts(store):
    mission = {'goal': 'Block', 'params': {'where': {'point': 'target', 'description': 'a stance'}},
               'steps': [{'instruction': 'Go {where}', 'skill': 'move', 'args': {'to': '{where}'}}]}
    save('block', mission, store, CATALOG, 'demo_bot')
    assert ConfigStore(store.root).mission('block')['params'] == mission['params']
