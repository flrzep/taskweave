"""The hold window of a held step (torso_window, Move._ahead) on a test cell with the frame cell's numbers (no ROS):
where the window ends, the predicted object and base poses (design section 2), the step rules, the virtual joint of a
predicted base pose."""

import math

import numpy as np
import pytest

pytest.importorskip('sensor_msgs')     # the carry part imports the joint skills (ROS messages)

from lookahead_cell import CARRY, CYCLE, GRIPS, PICK, TOP, base_matrix, build, cell, ctx_of

from taskweave_core.config.poses import Pose2D
from taskweave_skills.ik import rpy_matrix
from taskweave_skills.points import moved_object
from taskweave_skills.torso_plan import AUTO, KEEP
from taskweave_skills.torso_window import World, flat, planar, predict


def window_of(store, steps, k=0, registry=None, **kw):
    move, carry = build(store, registry)
    ctx = ctx_of(steps, k, **kw)
    first = move._plan(steps[k][2])
    ahead = move._ahead(ctx)
    hold = carry.hold.held_all()[0]
    return predict(carry, hold, {'id': steps[k][0], 'label': first.desc, 'call': first.calls[0]}, ahead,
                   carry.hold.transform('world', 'base_link')), ahead


def close(a, b, tol=1e-3):
    return np.allclose(np.asarray(a, float), np.asarray(b, float), atol=tol)


@pytest.fixture
def store(tmp_path):
    return cell(tmp_path)


def test_the_frame_cycle_window_ends_at_the_grip_that_opens(store):
    window, ahead = window_of(store, CYCLE)
    assert window.ids == ['s1_s7', 's1_s8', 's1_s9', 's1_s10', 's1_s11', 's2_s1', 's2_s2', 's2_s3', 's2_s4', 's2_s5',
                          's2_s6']
    assert window.end == 's2_s7: a grip open' and ahead[-1] == {'id': 's2_s7', 'end': 'a grip open'}


def test_the_frame_cycle_window_predicts_the_poses_of_section_2(store):
    window, _ = window_of(store, CYCLE)
    steps = {step.id: step for step in window.steps}
    origin = lambda sid: window.objects[sid][:3, 3]          # noqa: E731  (the object before the step)
    lift = moved_object(TOP, [0, 0, 0.3])
    assert close(origin('s1_s8'), [1.0, 0.0, 0.097])         # after the lift, section 2: (1.0, 0, 0.097)
    tilted = moved_object(lift, [0, 0, 0], None, 0.0, 30.0)
    assert close(window.objects['s1_s10'], tilted)           # the tilt keeps the origin, turns 30 deg about y
    assert close(window.objects['s2_s1'], rpy_matrix([1.0, 0, 0.4], [0, -0.8727, 0]))   # carry: base frame
    stand = steps['s2_s2']
    assert [len(stand.lines), list(stand.targets[0])] == [2, ['left', 'right']]
    assert close(stand.lines[0]['left'], rpy_matrix([1.3, 0, 0.5], [0, -0.8727, 0]) @ GRIPS['left'])
    assert close(stand.targets[0]['left']['tcp'], (rpy_matrix([1.1, 0, 0.7], [0, -1.5708, 0]) @ GRIPS['left'])[:3, 3])
    # the hook from the hang stance (x 1.1, facing the hanger at x -0.115): plate 10 cm above it, then on it. Section 2
    # assumed the hanger 1.1 m ahead (x 1.14); the cell puts it 1.215 m ahead: x 1.255, z as measured (0.472 / 0.372)
    hook = window.objects['s2_s6']
    assert close(hook[:3, 3], [1.255, 0.0, 0.472])
    on = (steps['s2_s6'].lines[-1]['left'] @ np.linalg.inv(GRIPS['left']))[:3, 3]
    assert close(on, [1.255, 0.0, 0.372])
    assert steps['s2_s5'].ahead is not None and close(steps['s2_s5'].ahead['left']['tcp'],
                                                      (steps['s2_s6'].lines[-1]['left'])[:3, 3])


def test_base_moves_keep_the_torso_and_move_the_later_steps(store):
    window, _ = window_of(store, CYCLE)
    steps = {step.id: step for step in window.steps}
    assert steps['s1_s10'].drive and close(steps['s1_s10'].base, planar(-0.5, 0, 0))
    hang = flat(base_matrix(PICK)) @ steps['s2_s5'].base                 # the base in the cell at the hook
    assert close(hang, planar(1.1, 0.0, 3.1416)) and steps['s2_s4'].drive
    assert steps['s1_s7'].base is None and steps['s1_s9'].base is None        # before any base move: as it stands
    assert steps['s2_s1'].key is None and steps['s2_s5'].key != steps['s2_s6'].key


def test_a_posture_move_keeps_the_torso_and_makes_the_later_steps_uncertain(store):
    window, _ = window_of(store, CYCLE)
    steps = {step.id: step for step in window.steps}
    assert steps['s1_s8'].torso == KEEP and steps['s1_s8'].certain and steps['s1_s7'].certain
    assert not steps['s1_s9'].certain and 's1_s9' in window.loose and 's1_s7' not in window.loose
    assert steps['s1_s7'].torso == AUTO and steps['s2_s2'].torso == AUTO


def test_the_frame_carry_window_and_a_held_pose_that_sets_the_torso(store):
    window, _ = window_of(store, CARRY)
    assert window.ids == ['s1_s7', 's1_s9', 's1_s11', 's2', 's3'] and window.end == 's4: a grip open'
    down = window.steps[-1]
    assert close((down.lines[0]['left'] @ np.linalg.inv(GRIPS['left']))[:3, 3], [0.9, 0.0, -0.2])
    assert close(down.targets[0]['left']['tcp'], (rpy_matrix([0.85, 0, -0.563], [0, 0, 0]) @ GRIPS['left'])[:3, 3])
    pinned, _ = window_of(store, [('a', 'move', {'hand': 'holding', 'by': [0, 0, 0.1]}),
                                  ('b', 'move', {'to': 'carry_low'})])
    assert pinned.steps[1].torso == 0.3 and 'sets the torso' in pinned.steps[1].why and pinned.end == ''


@pytest.mark.parametrize('ahead, end', [
    (('w', 'wait', {'seconds': 1.0}), 'w: a wait step'),
    (('free', 'move', {'hand': 'right', 'by': [0, 0, 0.1]}), 'free: a move to target move of free hands'),
    (('j', 'move', {'to': 'home'}), 'j: a move to joint positions move'),
    (('bad', 'move', {'to': 'no_such_target'}), 'bad: it does not resolve now'),
])
def test_the_window_ends_at_a_step_it_cannot_predict(store, ahead, end):
    steps = [CYCLE[0], CYCLE[2], ahead, CYCLE[4]]
    registry = None
    if ahead[0] == 'free':            # only the left hand holds: the right one is free
        from lookahead_cell import Registry
        registry = Registry()
        registry.hold['hands'] = ['left']
        registry.hold['tcp_in_object'] = {'left': GRIPS['left']}
    window, _ = window_of(store, steps, registry=registry)
    assert window.ids == ['s1_s7', 's1_s9'] and window.end.startswith(end)


def test_the_window_ends_at_a_placeholder_with_no_value(store):
    window, _ = window_of(store, CYCLE, resolved={'s1_s11': False})
    assert window.ids[-1] == 's1_s10' and window.end == 's1_s11: a placeholder has no value yet'


def test_a_navigate_ahead_resolves_the_later_steps_at_its_pose_and_asks_no_scene(store):
    move, _ = build(store)
    ahead = move._ahead(ctx_of(CYCLE, 0))
    assert not move._predicting and move._near_at is None and ahead[4]['plan'].part == 'navigate'


def test_world_moves_the_virtual_joint_with_the_base():
    """The virtual joint's value at a predicted base motion D (in the base frame now): M D M^-1 W, M the base in the
    model frame now, W the joint now; with the base the joint's child, the base in the model frame then is M D."""
    model_base = planar(2.0, 0.5, 0.3)
    model_base[2, 3] = 0.588
    virtual = model_base.copy()                      # the joint's child is the base: W = M
    world = World('world_joint', virtual, model_base)
    moved = planar(-0.5, 0.2, math.pi / 2)
    joint, value = world(moved)
    assert joint == 'world_joint' and close(value, model_base @ moved, 1e-9) and world(None) is None
    offset = np.eye(4)
    offset[2, 3] = -0.1                              # a root 10 cm below the base: W = M offset^-1
    world = World('j', model_base @ np.linalg.inv(offset), model_base)
    assert close(world(moved)[1] @ offset, model_base @ moved, 1e-9)
    assert close(flat(base_matrix(Pose2D(1.0, 2.0, 0.5))), planar(1.0, 2.0, 0.5), 1e-9)
