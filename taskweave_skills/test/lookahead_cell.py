"""A test cell for the torso look-ahead (test_torso_window, test_torso_lookahead): the skills fixture plus a hold
cycle like the frame cell's (held poses, a hanger with a hook, its stances; the numbers of the frame cell), Move with
stub parts and a MoveObject that predicts without ROS."""

import shutil
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

from taskweave_core.config import ConfigStore
from taskweave_core.config.poses import Pose2D
from taskweave_skills.move import Move
from taskweave_skills.object_motion import MoveObject
from taskweave_skills.parts import PARTS
from taskweave_skills.torso_window import planar

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
BASE_Z = 0.583                      # base_link above the cell floor
PICK = Pose2D(-1.0, 0.0, 0.0)       # the stance the hold starts at
GRIPS = {'left': np.array([[1, 0, 0, 0], [0, -1, 0, 0.3], [0, 0, -1, 0.05], [0, 0, 0, 1]], float),
         'right': np.array([[1, 0, 0, 0], [0, -1, 0, -0.3], [0, 0, -1, 0.05], [0, 0, 0, 1]], float)}
TOP = np.eye(4)
TOP[:3, 3] = [1.0, 0.0, 0.38 - BASE_Z]   # the top frame of a full stack in base_link (stance 1 m before it)

TARGETS = {
    'carry_flat': {'moves': 'object', 'type': 'box', 'path': [{'xyz': [1.0, 0.0, 0.4], 'rpy': [0.0, -0.8727, 0.0]}]},
    'stand_up': {'moves': 'object', 'type': 'box', 'path': [{'xyz': [1.3, 0.0, 0.5], 'rpy': [0.0, -0.8727, 0.0]},
                                                           {'xyz': [1.1, 0.0, 0.7], 'rpy': [0.0, -1.5708, 0.0]}]},
    'approach_up': {'moves': 'object', 'type': 'box', 'path': [{'xyz': [1.1, 0.0, 0.7], 'rpy': [0.0, -1.5708, 0.0]}]},
    'set_down_held': {'moves': 'object', 'type': 'box', 'path': [{'xyz': [0.9, 0.0, -0.2], 'rpy': [0.0, 0.0, 0.0]},
                                                                {'xyz': [0.85, 0.0, -0.563], 'rpy': [0.0, 0.0, 0.0]}]},
    'carry_low': {'moves': 'object', 'type': 'box', 'joints': {'torso': 0.3},
                  'path': [{'xyz': [1.0, 0.0, 0.2], 'rpy': [0.0, 0.0, 0.0]}]},
}
CELL_TARGETS = {'staging': {'moves': 'base', 'x': 1.765, 'y': 0.0, 'yaw': 3.1416},
                'hang': {'moves': 'base', 'x': 1.1, 'y': 0.0, 'yaw': 3.1416},
                'set_down': {'moves': 'base', 'x': 1.5, 'y': 0.0, 'yaw': 3.1416}}


def cell(tmp_path) -> ConfigStore:
    root = tmp_path / 'config'
    shutil.copytree(FIXTURE, root)

    def add(path, data):
        file = root / path
        old = yaml.safe_load(file.read_text()) if file.exists() else {}
        file.write_text(yaml.safe_dump({**(old or {}), **data}, sort_keys=False))
    box = yaml.safe_load((root / 'objects/box.yaml').read_text())
    box['targets']['plate'] = {'moves': 'object', 'xyz': [0.20, 0.0, 0.04], 'rpy': [0.0, -1.5708, 0.0]}
    (root / 'objects/box.yaml').write_text(yaml.safe_dump(box, sort_keys=False))
    add('objects/hanger.yaml', {'name': 'hanger', 'targets': {'hook': {'moves': 'object', 'xyz': [0.0, 0.0, 0.0],
                                                                        'rpy': [3.1416, 0.0, 0.0],
                                                                        'reference': 'plate'}}})
    add('cell/objects.yaml', {'hanger_1': {'object': 'hanger', 'frame': 'cell', 'x': -0.115, 'y': 0.0, 'z': 1.155,
                                           'yaw': 0.0}})
    add('cell/targets.yaml', CELL_TARGETS)
    add('robots/duo_bot/targets.yaml', TARGETS)
    return ConfigStore(root)


def base_matrix(pose: Pose2D) -> np.ndarray:
    m = planar(pose.x, pose.y, pose.yaw)
    m[2, 3] = BASE_Z
    return m


class Registry:
    """The hold registry of the parts: one hold of box_1 by both hands."""

    def __init__(self, obj=TOP, base=PICK):
        self.hold = {'object': 'box_1', 'point': 'box_1/rim', 'hands': ['left', 'right'], 'tools': ['left', 'right'],
                     'tcp_in_object': dict(GRIPS), 'object_in_base': obj.copy(), 'type': 'box'}
        self.base = base_matrix(base)

    @property
    def hand_holds(self):
        return {hand: self.hold['object'] for hand in self.hold['hands']}

    def held_all(self):
        return [dict(self.hold)]

    def hold_of(self, hand):
        return dict(self.hold)

    def reanchor(self, name=None):
        return True

    def type_of(self, hold, instances):
        return hold.get('type')

    def transform(self, parent, child):
        return self.base


class Part:
    """A stub part for Move (navigate, straight, posture, ...)."""

    def __init__(self, registry, pose=PICK):
        self.hold, self.pose, self.default_timeout_s = registry, pose, 60.0

    def _current(self):
        return self.pose


def build(store, registry=None):
    """(Move, its carry part): Move with stub parts and a MoveObject that resolves and predicts (no ROS)."""
    registry = registry or Registry()
    carry = MoveObject.__new__(MoveObject)
    carry.config, carry.robot, carry.name = store, store.robot('duo_bot'), 'move.carry'
    carry.hold, carry.cell_frame, carry.hands = registry, 'world', ['left', 'right']
    carry.p = SimpleNamespace(grip_drift_m=0.02, position_tolerance=1e-3)
    carry.ik = SimpleNamespace(torso={'joint': 'lift', 'candidates': [round(0.1 * i, 1) for i in range(8)]},
                               joint_pattern='{tool}_joint[1-2]')
    move = Move.__new__(Move)
    move.config, move.robot, move._trial, move._used = store, store.robot('duo_bot'), False, []
    move.hands, move.name, move.unavailable = ['left', 'right'], 'move', {}
    move.modules = {part: carry if part == 'carry' else Part(registry) for part in PARTS}
    move._scene_problem = lambda hold: 'not to be asked while predicting'
    return move, carry


# the frame_cycle hold of the top frame, as the mission's steps (pick_frame from the lift, hang_frame), without the
# held torso: keep (P6-10b T5)
CYCLE = [('s1_s7', 'move', {'hand': 'holding', 'by': [0, 0, 0.3]}),
         ('s1_s8', 'move', {'elbow': 'down'}),
         ('s1_s9', 'move', {'hand': 'holding', 'by': [0, 0, 0], 'tilt_deg': 30}),
         ('s1_s10', 'move', {'base': {'by': [-0.5, 0]}}),
         ('s1_s11', 'move', {'to': 'carry_flat'}),
         ('s2_s1', 'move', {'to': 'staging'}),
         ('s2_s2', 'move', {'to': 'stand_up'}),
         ('s2_s3', 'move', {'to': 'approach_up'}),
         ('s2_s4', 'move', {'to': 'hang', 'motion': {'mode': 'straight'}}),
         ('s2_s5', 'move', {'to': 'hanger_1/hook', 'offset_m': 0.1}),
         ('s2_s6', 'move', {'to': 'hanger_1/hook'}),
         ('s2_s7', 'grip', {'hands': 'all', 'action': 'open'}),
         ('s2_s8', 'move', {'to': 'home'})]
CARRY = [('s1_s7', 'move', {'hand': 'holding', 'by': [0, 0, 0.3]}),
         ('s1_s9', 'move', {'hand': 'holding', 'by': [0, 0, 0], 'tilt_deg': 30}),
         ('s1_s11', 'move', {'to': 'carry_flat'}),
         ('s2', 'move', {'to': 'set_down'}),
         ('s3', 'move', {'to': 'set_down_held', 'torso': 'auto'}),
         ('s4', 'grip', {'hands': 'all', 'action': 'open'})]


def context(steps, k, plan_id='p1', cycle=1, resolved=None):
    """The executive's context of step ``k`` of ``steps`` [(id, skill, args)]."""
    return {'plan_id': plan_id, 'cycle': cycle, 'step_index': k,
            'ahead': [{'id': i, 'skill': skill, 'kind': None, 'args': args,
                       'resolved': True if resolved is None else resolved.get(i, True)}
                      for i, skill, args in steps[k + 1:]]}


def ctx_of(steps, k, **kw):
    fed = []
    ctx = SimpleNamespace(context=context(steps, k, **kw), step_id=steps[k][0], should_stop=False, fed=fed,
                          feedback=lambda progress=-1.0, status='': fed.append(status))
    return ctx
