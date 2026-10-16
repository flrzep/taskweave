"""Phase-clock playback of joint moves and the base's speed scaling under the global speed override (spec 4.3).

Simulated time: time.sleep in the skill modules advances a fake clock, joints follow their commands exactly."""

import math

import pytest

pytest.importorskip('sensor_msgs')

from taskweave_core.config import Pose2D  # noqa: E402
from taskweave_core.skill import SkillContext  # noqa: E402
from taskweave_skills import joints as joints_module  # noqa: E402
from taskweave_skills.joints import Params, _JointSkill, interpolate, plan_duration  # noqa: E402
from taskweave_skills.motion_plan import Trajectory  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def now(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class Speed:
    def __init__(self, percent=100.0):
        self.percent = percent

    def fraction(self):
        return self.percent / 100.0


class Joints:
    """Joint states = the last command (perfect tracking); records every command with its time."""

    def __init__(self, clock, positions):
        self.clock, self.positions, self.reached, self.state_topic = clock, dict(positions), {}, '/s'
        self.commands = []

    def current(self, names, max_age):
        return {n: self.positions[n] for n in names}

    def command(self, targets):
        self.commands.append((self.clock.t, dict(targets)))
        self.positions.update(targets)


def run_move(monkeypatch, percent=100.0, goal=None, path=None, change=None, timeout_s=0.0):
    """_move from j1=0 to ``goal``; ``change``: (time after start, new percent). Returns (result, joints, clock)."""
    clock, speed = Clock(), Speed(percent)
    monkeypatch.setattr(joints_module.time, 'sleep', clock.sleep)
    if change:
        at, value = change
        original = clock.sleep

        def sleep(seconds):
            original(seconds)
            if clock.t - 100.0 >= at:
                speed.percent = value
        monkeypatch.setattr(joints_module.time, 'sleep', sleep)
    skill = _JointSkill.__new__(_JointSkill)
    skill.p = Params(max_joint_speed=1.0)
    skill.joints = Joints(clock, {'j1': 0.0, 'j2': 0.0})
    skill._held = {}
    ctx = SkillContext(clock.now, timeout_s, lambda: False, lambda p, s: None, speed=speed)
    result, _ = skill._move(ctx, goal or {'j1': 1.0}, 'test', path=path)
    return result, skill.joints, clock


def end_of_motion(joints, goal):
    """Time (from the start) of the first command at the goal."""
    return next(t for t, c in joints.commands if all(abs(c[n] - v) < 1e-12 for n, v in goal.items())) - 100.0


def test_at_100_percent_the_commanded_trajectory_is_identical_to_before(monkeypatch):
    """Regression: the commands equal the old playback, reference(now - t0), bit for bit."""
    goal = {'j1': 1.0, 'j2': -0.4}
    result, joints, _ = run_move(monkeypatch, 100.0, goal)
    assert result is None
    p = Params(max_joint_speed=1.0)
    start = {'j1': 0.0, 'j2': 0.0}
    duration = plan_duration(start, goal, p)
    t0 = 100.0
    assert len(joints.commands) > 50
    for t, command in joints.commands:
        assert command == interpolate(start, goal, (t - t0) / duration)   # exact equality, no tolerance


def test_at_100_percent_a_planned_path_plays_back_identically(monkeypatch):
    path = Trajectory(['j1'], [0.0, 0.7, 1.5], [[0.0], [0.6], [1.0]])
    result, joints, _ = run_move(monkeypatch, 100.0, {'j1': 1.0, 'j2': 0.3}, path=path)
    assert result is None
    duration = max(path.duration, Params().min_duration_s)
    for t, command in joints.commands:
        assert command == {**interpolate({'j1': 0.0, 'j2': 0.0}, {'j2': 0.3}, (t - 100.0) / duration),
                           **path.at(t - 100.0)}


@pytest.mark.parametrize('percent, factor', [(10.0, 10.0), (50.0, 2.0)])
def test_low_override_stretches_the_move(monkeypatch, percent, factor):
    _, fast, _ = run_move(monkeypatch, 100.0)
    _, slow, _ = run_move(monkeypatch, percent)
    nominal = end_of_motion(fast, {'j1': 1.0})
    assert end_of_motion(slow, {'j1': 1.0}) == pytest.approx(factor * nominal, rel=0.03)
    # same path, only slower: the position at the same phase is the same
    halfway = [c['j1'] for t, c in slow.commands if t - 100.0 <= factor * nominal / 2][-1]
    assert halfway == pytest.approx(0.5, abs=0.04)


def test_settle_scales_with_the_override(monkeypatch):
    """After the trajectory, settle_s is work time: at 10 % the step settles 10x as long."""
    _, joints, clock = run_move(monkeypatch, 10.0)
    end = end_of_motion(joints, {'j1': 1.0})
    assert clock.t - 100.0 - end == pytest.approx(10 * Params().settle_s, abs=0.05)


def test_a_slow_step_does_not_time_out(monkeypatch):
    """A 1 s move with a 2 s timeout: at 10 % it takes about 13 s and still succeeds (timeout on work time)."""
    result, _, clock = run_move(monkeypatch, 10.0, timeout_s=2.0)
    assert result is None and clock.t - 100.0 > 10.0


def test_a_change_during_the_move_takes_effect_within_a_cycle_and_is_smooth(monkeypatch):
    _, joints, _ = run_move(monkeypatch, 10.0, change=(2.0, 100.0))
    times = [t - 100.0 for t, _ in joints.commands]
    j1 = [c['j1'] for _, c in joints.commands]
    period = 1.0 / Params().rate_hz
    i = next(k for k, t in enumerate(times) if t >= 2.0)
    before = j1[i - 1] - j1[i - 2]
    assert j1[i + 1] - j1[i] > before                     # faster within one control cycle
    phase_steps = [(b - a) for a, b in zip(times, times[1:])]
    assert max(phase_steps) == pytest.approx(period)
    # tau never jumps: its increment per cycle rises by at most the slew (2.0/s) times the cycle
    taus = [interpolate_inverse(v) for v in j1[i - 10:] if 0.001 < v < 0.9]
    increments = [b - a for a, b in zip(taus, taus[1:])]
    rises = [b - a for a, b in zip(increments, increments[1:])]
    assert max(rises) <= 2.0 * period * period + 1e-6


def interpolate_inverse(value):
    """Phase (s) of a 1 s smoothstep move at j1 = value."""
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if mid * mid * (3 - 2 * mid) < value:
            lo = mid
        else:
            hi = mid
    return lo


def test_base_velocity_limits_follow_the_override():
    from taskweave_skills.navigate import Params as NavParams
    from taskweave_skills.navigate import command, scaled

    p = NavParams()
    assert scaled(p, 1.0) is p                            # 100 %: the same parameters, the same commands
    pose, target = Pose2D(0.0, 0.0, 0.0), Pose2D(5.0, 0.0, 0.0)
    previous = (0.0, 0.0, 0.0)
    for _ in range(200):
        previous = command(pose, target, 'differential', scaled(p, 0.1), previous, 0.05)
    assert previous[0] == pytest.approx(0.1 * p.max_linear)
    turn = Pose2D(0.0, 0.0, math.pi / 2)
    previous = (0.0, 0.0, 0.0)
    for _ in range(200):
        previous = command(Pose2D(0.0, 0.0, 0.0), turn, 'differential', scaled(p, 0.1), previous, 0.05)
    assert abs(previous[2]) <= 0.1 * p.max_angular + 1e-12
