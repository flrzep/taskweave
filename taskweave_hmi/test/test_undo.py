"""Undo of a Teach save (undo.py) and the pieces of a save from the robot that need no ROS: the teach node's reference
for a location or stance (points.teach_reference) and what a capture would change (arm.capture_changes)."""

import math
import shutil
from pathlib import Path

import pytest

from taskweave_core.config import ConfigStore
from taskweave_hmi import arm, points, undo

FIXTURE = Path(__file__).resolve().parents[2] / 'taskweave_core' / 'test' / 'fixtures' / 'targets_new'
ROBOT = 'bot_two_arm'


@pytest.fixture
def config(tmp_path):
    shutil.copytree(FIXTURE, tmp_path / 'config')
    return ConfigStore(tmp_path / 'config')


def save_location(config, history, name, x):
    return history.run(config, undo.teach_files(ROBOT), lambda: points.save(
        config, ROBOT, 'location', [name], {'x': x, 'y': 0.0, 'yaw': 0.0}), f'location {name}')


def test_a_save_is_undone_once_and_only_while_its_files_are_as_it_left_them(config):
    history = undo.Undo()
    before = (config.root / 'cell/targets.yaml').read_text()
    file, saved = save_location(config, history, 'waiting', 2.5)
    assert file == 'cell/targets.yaml' and saved['files'] == ['cell/targets.yaml']
    assert saved['label'] == 'location waiting'
    assert config.locations()['waiting'].x == pytest.approx(2.5)
    assert history.undo(config, saved['id']) == {'label': 'location waiting', 'files': ['cell/targets.yaml']}
    assert (config.root / 'cell/targets.yaml').read_text() == before and config.locations()['waiting'].x == 2.0
    with pytest.raises(undo.UndoError, match='nothing to undo'):
        history.undo(config, saved['id'])
    _, first = save_location(config, history, 'waiting', 2.5)
    save_location(config, history, 'spawn', 1.0)              # a later save of the same file: the first is not undone
    with pytest.raises(undo.UndoError, match='cell/targets.yaml changed since that save'):
        history.undo(config, first['id'])
    assert config.locations()['spawn'].x == pytest.approx(1.0)


def test_a_save_that_changes_nothing_has_no_undo_and_old_saves_are_dropped(config):
    history = undo.Undo(keep=1)
    assert save_location(config, history, 'waiting', 2.0)[1] is None   # the same pose: no file changed
    _, first = save_location(config, history, 'waiting', 2.5)
    save_location(config, history, 'spawn', 1.0)
    with pytest.raises(undo.UndoError, match='too old'):
        history.undo(config, first['id'])


def test_undo_of_a_save_that_made_a_file_removes_it(config):
    history = undo.Undo()
    made = config.root / 'robots/bot_lift/objects/frame_type2.yaml'
    assert not made.exists()
    _, saved = history.run(config, undo.teach_files('bot_lift', 'frame_type2'), lambda: points.save(
        config, 'bot_lift', 'stance', ['objects', 'frame_type2', 'pick'], {'x': -1.0, 'y': 0.0, 'yaw': 0.0}), 'stance')
    assert made.is_file() and saved['files'] == ['robots/bot_lift/objects/frame_type2.yaml']
    history.undo(config, saved['id'])
    assert not made.exists()


def test_a_save_from_the_robot_asks_the_teach_node_for_the_name_or_the_object_it_is_measured_at(config):
    assert points.teach_reference(config, ROBOT, 'location', ['spawn'], None) == ('spawn', False, None)
    assert points.teach_reference(config, ROBOT, 'stance', ['objects', 'frame_type2', 'pick'], 'bay_2_stack_2_front') \
        == ('bay_2_stack_2_front/pick', False, 'bay_2_stack_2_front')
    assert points.teach_reference(config, ROBOT, 'stance', ['instances', 'bay_1_stack_1_back', 'pick'], None) \
        == ('bay_1_stack_1_back/pick', True, 'bay_1_stack_1_back')
    with pytest.raises(points.PointError, match='pick the placed frame_type2'):
        points.teach_reference(config, ROBOT, 'stance', ['objects', 'frame_type2', 'pick'], None)
    with pytest.raises(points.PointError, match='part_box_1 is not a placed frame_type2'):
        points.teach_reference(config, ROBOT, 'stance', ['objects', 'frame_type2', 'pick'], 'part_box_1')
    with pytest.raises(points.PointError, match='spawn already exists'):
        points.teach_reference(config, ROBOT, 'location', ['spawn'], None, create=True)
    with pytest.raises(points.PointError, match='moves joints'):     # a joint target's name is not a location
        points.teach_reference(config, ROBOT, 'location', ['home'], None)
    assert points.stored_pose(config, ROBOT, 'location', ['spawn']) == {'x': 2.0, 'y': -0.45, 'yaw': 0.0}


def test_a_capture_shows_how_far_each_hand_moves_and_turns_and_each_joint_changes():
    old = {'left': {'xyz': [0.5, 0.2, 0.4], 'rpy': [0.0, 0.0, 0.0]},
           'right': {'xyz': [0.5, -0.2, 0.4], 'rpy': [0, 0, 0]}}
    new = {'left': {'xyz': [0.53, 0.2, 0.44], 'rpy': [0.0, 0.0, math.radians(10)]},
           'right': {'xyz': [0.5, -0.2, 0.4], 'rpy': [0, 0, 0]}}
    changes = arm.capture_changes(old, new, {'torso': 0.5, 'j1': 0.1}, {'torso': 0.55, 'j1': 0.1 + math.radians(3)},
                                  linear=['torso'])
    left, right = changes['hands']
    assert (left['tool'], left['moved_mm'], left['turned_deg']) == ('left', 50.0, 10.0)
    assert (right['moved_mm'], right['turned_deg']) == (0.0, 0.0)
    assert [(j['joint'], j['unit'], j['change']) for j in changes['joints']] == [
        ('torso', 'm', 0.05), ('j1', 'deg', 3.0)]
    added = arm.capture_changes({}, {'left': old['left']}, {}, {})['hands'][0]
    assert added['old'] is None and added['moved_mm'] is None


def test_saves_run_one_at_a_time_so_an_undo_never_drops_a_save_it_does_not_know(config):
    import threading

    history, order, results = undo.Undo(), [], {}

    def first():
        started.set()
        second_thread.start()
        second_thread.join(0.3)          # the second save waits for this one to end
        order.append('A')
        return points.save(config, ROBOT, 'location', ['waiting'], {'x': 2.5, 'y': 0.0, 'yaw': 0.0})

    def second():
        results['B'] = history.run(config, undo.teach_files(ROBOT), lambda: order.append('B') or points.save(
            config, ROBOT, 'location', ['spawn'], {'x': 1.0, 'y': 0.0, 'yaw': 0.0}), 'location spawn')
    started, second_thread = threading.Event(), threading.Thread(target=second)
    history.run(config, undo.teach_files(ROBOT), first, 'location waiting')
    second_thread.join()
    assert order == ['A', 'B']
    history.undo(config, results['B'][1]['id'])          # undoes spawn only: waiting keeps the first save
    config.reload()
    assert config.locations()['waiting'].x == pytest.approx(2.5) and config.locations()['spawn'].x == 2.0


def test_a_write_that_fails_after_changing_files_is_held_for_undo(config):
    history = undo.Undo()

    def write():
        points.save(config, ROBOT, 'location', ['waiting'], {'x': 2.5, 'y': 0.0, 'yaw': 0.0})
        raise points.PointError('the second step was refused')
    with pytest.raises(points.PointError) as refused:
        history.run(config, undo.teach_files(ROBOT), write, 'location waiting')
    assert refused.value.undo['files'] == ['cell/targets.yaml']
    history.undo(config, refused.value.undo['id'])
    assert config.locations()['waiting'].x == 2.0


def test_an_undo_that_stops_half_way_can_be_tried_again_for_the_rest(config, monkeypatch):
    from taskweave_core.config import SaveError

    history = undo.Undo()
    files = undo.teach_files(ROBOT, 'frame_type2')

    def write():
        points.save(config, ROBOT, 'location', ['waiting'], {'x': 2.5, 'y': 0.0, 'yaw': 0.0})
        points.save(config, ROBOT, 'stance', ['objects', 'frame_type2', 'pick'], {'x': -1.2, 'y': 0.0, 'yaw': 0.0})
    _, saved = history.run(config, files, write, 'two files')
    real, calls = config.save, []

    def flaky(path, *args, **kwargs):
        calls.append(path)
        if path == 'cell/targets.yaml' and calls.count(path) == 1:
            raise SaveError([{'path': path, 'pack': 'cell', 'where': '', 'line': None, 'col': None,
                              'message': 'refused once', 'severity': 'error'}])
        return real(path, *args, **kwargs)
    monkeypatch.setattr(config, 'save', flaky)
    with pytest.raises(undo.UndoError, match='refused once.*undone already; Undo again for the rest'):
        history.undo(config, saved['id'])                  # the robot's file first: done, then the cell's fails
    assert history.undo(config, saved['id'])['label'] == 'two files'
    config.reload()
    assert config.locations()['waiting'].x == 2.0


def test_a_name_with_a_slash_is_no_location_and_an_instance_entry_is_its_own(config):
    from taskweave_hmi import targets

    with pytest.raises(points.PointError, match='a name has no "/"'):
        points.teach_reference(config, ROBOT, 'location', ['bay_2_stack_2_front/pick'], None)
    with pytest.raises(points.PointError, match='unknown point address'):
        points.teach_reference(config, ROBOT, 'location', ['spawn', 'x'], None)
    assert targets.instance_entry(config, ROBOT, 'frame_type2', 'bay_1_stack_1_back', 'pick')['x'] == -1.528
    assert targets.instance_entry(config, ROBOT, 'frame_type2', 'bay_1_stack_1_front', 'pick') == {}
