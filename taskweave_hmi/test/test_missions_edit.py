"""Mission edits from the graph editor: operator-style ops applied on the ruamel tree (comments kept), then
validated and saved as a whole (missions.edit)."""

import shutil
from pathlib import Path

import pytest
from ruamel.yaml import YAML

from taskweave_core.config import ConfigStore
from taskweave_hmi import missions
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


def opened(store, name='patrol'):
    """The base of an edit: the file's hash as the editor read it."""
    return store.file_hash(f'missions/{name}.yaml')


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
                  store, CATALOG, 'demo_bot', base=opened(store))
    text = path.read_text()
    assert text.startswith('# my patrol') and '# the pause' in text
    first_two = [s['instruction'] for s in result['mission']['steps']][:2]
    assert first_two == ['Drive to the stance in front of shelf A', 'Wait a little']
    assert result['mission']['steps'][0]['on_failure'] == 'skip' and result['file'] == 'missions/patrol.yaml'
    assert ConfigStore(store.root).mission('patrol')['steps'][1] == WAIT
    # a refused edit writes nothing and names the step
    with pytest.raises(MissionError) as error:
        edit('patrol', [{'op': 'set', 'at': 'steps/0', 'key': 'args', 'value': {'to': 'nowhere'}}],
             store, CATALOG, 'demo_bot', base=opened(store))
    assert any("unknown target 'nowhere'" in p for p in error.value.problems) and error.value.places
    assert path.read_text() == text


def test_edit_makes_a_new_mission_from_ops(store):
    go = {'instruction': 'Go {where}', 'skill': 'move', 'args': {'to': '{where}'}}
    result = edit('fresh', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'A new one'},
                            {'op': 'set', 'at': '', 'key': 'params', 'value': {'where': {'point': 'target'}}},
                            {'op': 'insert', 'at': 'steps/0', 'step': go}],
                  store, CATALOG, 'demo_bot', create=True)
    assert result['mission']['goal'] == 'A new one' and 'fresh' in store.mission_names()
    # params are written too (they were dropped from new files before)
    assert ConfigStore(store.root).mission('fresh')['params'] == {'where': {'point': 'target'}}


def test_edit_renames(store):
    edit('night', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'At night'}], store, CATALOG, 'demo_bot',
         rename_from='patrol', base=opened(store))
    assert 'patrol' not in store.mission_names() and store.mission('night')['goal'] == 'At night'


def test_save_writes_params_and_facts(store):
    mission = {'goal': 'Block', 'params': {'where': {'point': 'target', 'description': 'a stance'}},
               'steps': [{'instruction': 'Go {where}', 'skill': 'move', 'args': {'to': '{where}'}}]}
    save('block', mission, store, CATALOG, 'demo_bot')
    assert ConfigStore(store.root).mission('block')['params'] == mission['params']


def test_edit_refuses_a_new_mission_under_a_taken_name_and_a_missing_one(store):
    before = (store.root / 'missions' / 'patrol.yaml').read_text()
    with pytest.raises(MissionError, match="'patrol' already exists"):
        edit('patrol', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'Taken'}], store, CATALOG, 'demo_bot',
             create=True)
    assert (store.root / 'missions' / 'patrol.yaml').read_text() == before
    with pytest.raises(MissionError, match="'ghost' does not exist"):
        edit('ghost', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'Gone'}], store, CATALOG, 'demo_bot',
             base='any')
    assert not (store.root / 'missions' / 'ghost.yaml').exists()


def test_edit_refuses_a_file_that_changed_since_the_open(store):
    from taskweave_hmi.missions import MissionConflict

    opened = store.file_hash('missions/patrol.yaml')
    goal = lambda text: [{'op': 'set', 'at': '', 'key': 'goal', 'value': text}]  # noqa: E731
    result = edit('patrol', goal('One'), store, CATALOG, 'demo_bot', base=opened)
    assert result['hash'] == store.file_hash('missions/patrol.yaml') != opened
    with pytest.raises(MissionConflict, match='changed since'):
        edit('patrol', goal('Two'), store, CATALOG, 'demo_bot', base=opened)
    assert store.mission('patrol')['goal'] == 'One'
    edit('patrol', goal('Two'), store, CATALOG, 'demo_bot', base=result['hash'])
    assert store.mission('patrol')['goal'] == 'Two'


def test_edit_refuses_a_file_that_is_not_yaml(store):
    (store.root / 'missions' / 'patrol.yaml').write_text('goal: [unclosed\nsteps: ')
    with pytest.raises(MissionError, match='not valid YAML'):
        edit('patrol', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'x'}], store, CATALOG, 'demo_bot',
             base=opened(store))


def test_edit_needs_the_base_it_was_read_with_unless_it_creates(store):
    before = (store.root / 'missions' / 'patrol.yaml').read_text()
    with pytest.raises(MissionError, match='base is missing'):
        edit('patrol', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'x'}], store, CATALOG, 'demo_bot')
    assert (store.root / 'missions' / 'patrol.yaml').read_text() == before
    result = edit('fresh', [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'New'},
                            {'op': 'insert', 'at': 'steps/0', 'step': WAIT}], store, CATALOG, 'demo_bot', create=True)
    assert result['hash'] == store.file_hash('missions/fresh.yaml')


def test_paths_have_one_spelling_so_a_block_cannot_be_moved_into_itself(store):
    d = doc(THREE)
    apply_ops(d, [{'op': 'wrap', 'from': 'steps/0', 'to': 'steps/1', 'as': 'group', 'block': {'id': 'g'}}])
    text = str(plain(d))
    for to in ('steps/00/steps/0', '/steps/0/steps/0', 'steps/+0/steps/0', 'steps//0/steps/0', 'steps/0/steps/0'):
        with pytest.raises(MissionError):
            apply_ops(d, [{'op': 'move', 'from': 'steps/0', 'to': to}])
        assert str(plain(d)) == text, to
    for at in ('steps/-1', 'steps/01', 'steps/-1/steps/0'):
        with pytest.raises(MissionError):
            apply_ops(d, [{'op': 'set', 'at': at, 'key': 'retries', 'value': 1}])
        with pytest.raises(MissionError):
            apply_ops(d, [{'op': 'remove', 'at': at}])
    assert str(plain(d)) == text
    apply_ops(d, [{'op': 'move', 'from': 'steps/1', 'to': 'steps/0/steps/0'}])   # another step into the block
    assert [s['id'] for s in d['steps'][0]['steps']] == ['c', 'a', 'b']


@pytest.mark.skipif(missions.blocks_supported(), reason='this core has mission blocks')
def test_block_ops_are_refused_on_a_core_without_blocks(store):
    path = store.root / 'missions' / 'patrol.yaml'
    before = path.read_text()
    for ops in ([{'op': 'wrap', 'from': 'steps/0', 'to': 'steps/1', 'as': 'group', 'block': {'id': 'g'}}],
                [{'op': 'insert', 'at': 'steps/0',
                  'step': {'instruction': 'Twice', 'repeat': {'times': 2}, 'steps': []}}],
                [{'op': 'set', 'at': 'steps/0', 'key': 'when', 'value': 'loose_part_seen'}]):
        with pytest.raises(MissionError, match='no mission blocks'):
            edit('patrol', ops, store, CATALOG, 'demo_bot', base=opened(store))
        assert path.read_text() == before
    assert missions.features() == []
    assert missions.unsupported([{'op': 'set', 'at': '', 'key': 'max_cycles', 'value': 3}]) == []   # a v1 key


def test_rename_and_delete_are_refused_while_another_mission_includes_it(store):
    (store.root / 'missions' / 'rounds.yaml').write_text(
        'goal: Patrol twice\nsteps:\n  - instruction: A round\n    include: patrol\n')
    store.reload()
    goal = [{'op': 'set', 'at': '', 'key': 'goal', 'value': 'At night'}]
    with pytest.raises(MissionError, match="included by rounds: it cannot be renamed"):
        edit('night', goal, store, CATALOG, 'demo_bot', rename_from='patrol', base=opened(store))
    with pytest.raises(MissionError, match='included by rounds'):
        save('night', store.mission('patrol'), store, CATALOG, 'demo_bot', rename_from='patrol')
    with pytest.raises(MissionError, match='cannot be deleted'):
        missions.delete('patrol', store)
    assert {'patrol', 'rounds'} <= set(store.mission_names())
    missions.delete('rounds', store)   # nothing includes rounds
    assert 'rounds' not in store.mission_names()


def test_nulls_inside_a_value_stay_and_a_null_step_key_goes():
    d = doc(THREE)
    apply_ops(d, [{'op': 'set', 'at': 'steps/0', 'key': 'args', 'value': {'seconds': 2, 'joint': None}},
                  {'op': 'insert', 'at': 'steps/3', 'step': {**WAIT, 'id': None, 'retries': None}}])
    assert plain(d['steps'][0]['args']) == {'seconds': 2, 'joint': None}
    assert plain(d['steps'][3]) == WAIT


def test_an_edit_that_fails_half_way_writes_nothing(store):
    path = store.root / 'missions' / 'patrol.yaml'
    before = path.read_text()
    with pytest.raises(MissionError, match='op 2'):
        edit('patrol', [{'op': 'insert', 'at': 'steps/0', 'step': WAIT}, {'op': 'remove', 'at': 'steps/99'}],
             store, CATALOG, 'demo_bot', base=opened(store))
    assert path.read_text() == before and len(store.mission('patrol')['steps']) == 3
