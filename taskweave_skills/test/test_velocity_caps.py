"""Velocity caps (T2.6): no commanded speed above the robot's limits, and no change for today's S1 config.

``plan_duration`` stretches a move so no joint exceeds its limit; a planned path is stretched in time the same way
(``limit_path``); the base caps are navigate_to's (``base.limits``). With the S1 pack's limits and the speeds the S1
runs at today, the commanded trajectories are identical to those without limits."""

import shutil
from dataclasses import replace
from pathlib import Path

import pytest

pytest.importorskip('sensor_msgs')

from test_speed_playback import Clock, Joints, Speed  # noqa: E402

from taskweave_core.config import ConfigStore  # noqa: E402
from taskweave_core.config.limits import velocity_limits  # noqa: E402
from taskweave_core.skill import Outcome, SkillContext, SkillResult  # noqa: E402
from taskweave_skills import joints as joints_module  # noqa: E402
from taskweave_skills.joints import Params, _JointSkill, limit_path, plan_duration  # noqa: E402
from taskweave_skills.motion_plan import Trajectory  # noqa: E402

FIXTURE = Path(__file__).parent / 'fixtures' / 'config'
PACKS = Path(__file__).resolve().parents[2] / 'taskweave_robots'
ARMS = [f'{side}_joint{n}' for side in ('left', 'right') for n in range(1, 8)]
S1_STEP = {**{joint: 0.7 for joint in ARMS}, 'body_joint': 0.4, 'head_joint1': 0.5, 'head_joint2': -0.3}
S1_START = dict.fromkeys(S1_STEP, 0.0)


@pytest.fixture
def s1_store(monkeypatch, tmp_path):
    monkeypatch.setenv('TASKWEAVE_ROBOT_PACKS', str(PACKS))
    config = tmp_path / 'config'
    shutil.copytree(FIXTURE, config)
    return ConfigStore(config)


def s1_params(limits):
    """Today's S1 direct-move speeds: skills/move_*.yaml joint_rad_s 1.2, joint_linear_m_s 0.25 (docs/examples)."""
    return Params(max_joint_speed=1.2, max_linear_speed=0.25, linear_joints=('body_joint',), velocity_limits=limits)


def play(monkeypatch, params, goal, start, path=None):
    clock = Clock()
    monkeypatch.setattr(joints_module.time, 'sleep', clock.sleep)
    skill = _JointSkill.__new__(_JointSkill)
    skill.p, skill._held = params, {}
    skill.joints = Joints(clock, start)
    ctx = SkillContext(clock.now, 0.0, lambda: False, lambda p, s: None, speed=Speed(100.0))
    result, _ = skill._move(ctx, goal, 'test', path=path)
    assert result is None
    return skill.joints.commands, skill


def test_the_s1_pack_has_velocity_limits(s1_store):
    limits = velocity_limits(s1_store, 'galbot_s1')
    assert limits['left_joint1'] == 1.2 and limits['body_joint'] == 0.25 and limits['head_joint1'] == 1.2


def test_s1_commanded_trajectory_at_100_percent_is_unchanged_by_the_caps(monkeypatch, s1_store):
    limits = velocity_limits(s1_store, 'galbot_s1')
    before, _ = play(monkeypatch, s1_params({}), S1_STEP, S1_START)
    after, skill_after = play(monkeypatch, s1_params(limits), S1_STEP, S1_START)
    assert len(before) > 50 and before == after            # exact equality: every command at every time
    assert not skill_after.__dict__.get('_capped')          # nothing was capped
    slow = Params(max_joint_speed=0.6, max_linear_speed=0.1, linear_joints=('body_joint',))
    assert plan_duration(S1_START, S1_STEP, slow) == plan_duration(S1_START, S1_STEP,
                                                                   replace(slow, velocity_limits=limits))
    # a planned path at the S1's own speeds is played as it is
    path = Trajectory(['left_joint1', 'body_joint'], [0.0, 0.8, 1.6], [[0.0, 0.0], [0.5, 0.1], [0.9, 0.2]])
    assert limit_path(path, s1_params(limits)) == (path, {})
    goal, start = {'left_joint1': 0.9, 'body_joint': 0.2}, {'left_joint1': 0.0, 'body_joint': 0.0}
    planned_before, _ = play(monkeypatch, s1_params({}), goal, start, path=path)
    planned_after, _ = play(monkeypatch, s1_params(limits), goal, start, path=path)
    assert planned_before == planned_after


def test_a_speed_above_the_limit_stretches_the_move(monkeypatch):
    fast = Params(max_joint_speed=3.0, velocity_limits={'j1': 1.0})
    assert plan_duration({'j1': 0.0, 'j2': 0.0}, {'j1': 2.0, 'j2': 2.0}, fast) == pytest.approx(2.0)   # j1: limit
    free = Params(max_joint_speed=3.0)
    assert plan_duration({'j1': 0.0, 'j2': 0.0}, {'j1': 2.0, 'j2': 2.0}, free) == pytest.approx(2 / 3)
    commands, skill = play(monkeypatch, fast, {'j1': 2.0}, {'j1': 0.0})
    assert skill._capped == {'j1': 1.0}
    assert commands[-1][0] - commands[0][0] >= 2.0 - 0.05


def test_a_planned_path_over_a_limit_is_stretched_not_cut():
    path = Trajectory(['j1'], [0.0, 0.5, 1.0], [[0.0], [1.0], [2.0]])        # 2 rad/s
    stretched, capped = limit_path(path, Params(velocity_limits={'j1': 1.0}))
    assert capped == {'j1': 1.0} and stretched.points == path.points
    assert stretched.times == pytest.approx([0.0, 1.0, 2.0])
    assert limit_path(path, Params(velocity_limits={'j1': 2.0}))[1] == {}


def test_an_execute_reports_caps_in_motion_capped(monkeypatch):
    class Skill(_JointSkill):
        def execute(self, ctx, args):
            result, _ = self._move(ctx, {'j1': 1.0}, 'test')
            return result or SkillResult.succeeded('ok', motion={'speed': 1})

    clock = Clock()
    monkeypatch.setattr(joints_module.time, 'sleep', clock.sleep)
    skill = Skill.__new__(Skill)
    skill.p, skill._held = Params(max_joint_speed=3.0, velocity_limits={'j1': 1.0}), {}
    skill.joints = Joints(clock, {'j1': 0.0})
    ctx = SkillContext(clock.now, 0.0, lambda: False, lambda p, s: None, speed=Speed(100.0))
    result = skill.execute(ctx, {})
    assert result.outcome == Outcome.SUCCEEDED
    assert result.data['motion'] == {'speed': 1, 'capped': {'j1': 1.0}}
    skill.joints.positions['j1'] = 0.0
    skill.p = replace(skill.p, velocity_limits={})
    assert 'capped' not in skill.execute(ctx, {}).data['motion']   # nothing capped now: nothing reported
