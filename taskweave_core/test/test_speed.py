"""Speed override (skill API spec section 4): work clock, slew, test mode rules, timeouts on work time. No ROS."""

import pytest

from taskweave_core.executive import Executive, Outcome, State
from taskweave_core.plan import Plan, Step
from taskweave_core.planner import Planner
from taskweave_core.skill import SkillContext
from taskweave_core.speed import OverrideState, WorkClock
from taskweave_core.world_state import WorldState


class Target:
    def __init__(self, value=1.0):
        self.value = value

    def __call__(self):
        return self.value


def test_work_clock_at_a_constant_override_is_exact():
    clock = WorkClock(10.0, Target(1.0))
    for now in (10.0, 10.02, 10.37, 123.456):
        assert clock(now) == now - 10.0          # bit-identical to elapsed time at 100 %
    clock = WorkClock(10.0, Target(0.1))
    assert clock(30.0) == pytest.approx(2.0)
    clock = WorkClock(0.0, Target(0.5))
    assert clock(4.0) == pytest.approx(2.0)


def test_override_is_clamped_to_1_to_100_percent():
    assert WorkClock(0.0, Target(5.0)).o == 1.0
    assert WorkClock(0.0, Target(0.0)).o == 0.01


def test_slew_limits_how_fast_the_effective_override_follows():
    target = Target(0.1)
    clock = WorkClock(0.0, target, slew_per_s=2.0)
    t = 0.0
    for _ in range(50):
        t += 0.02
        clock(t)
    target.value = 1.0
    values = []
    for _ in range(40):                          # 0.8 s at 50 Hz
        t += 0.02
        clock(t)
        values.append(clock.o)
    steps = [b - a for a, b in zip([0.1, *values], values, strict=False)]
    assert values[0] > 0.1                        # takes effect within one control cycle
    assert max(steps) <= 2.0 * 0.02 + 1e-12       # smooth: at most the slew per cycle
    assert values[-1] == 1.0                      # 10 -> 100 % in about 0.45 s
    assert next(i for i, v in enumerate(values) if v == 1.0) <= 23


def test_stats_report_minimum_and_average():
    target = Target(1.0)
    clock = WorkClock(0.0, target, slew_per_s=0)  # no slew: steps
    clock(5.0)
    target.value = 0.5
    clock(5.0)
    clock(10.0)
    assert clock.stats() == {'override_min': 50.0, 'override_avg': 75.0}


def test_request_validates_the_range():
    state = OverrideState()
    assert state.request(150.0) == (False, 'speed override 150 % is outside 1-100 %', 100.0)
    assert state.request(0.5)[0] is False
    ok, message, percent = state.request(50.0)
    assert ok and percent == 50.0 and message == 'speed 50 %'
    assert state.request(0.0)[2] == 50.0           # 0: unchanged


def test_test_mode_lowers_at_run_start_and_restores_at_run_end():
    state = OverrideState()
    state.request(test_mode=True)
    assert state.percent == 100.0                  # switching it on alone changes nothing
    assert state.run_started() and state.percent == 10.0
    assert state.run_ended() and state.percent == 100.0
    state.request(5.0)                             # already below test_percent: not raised, not restored
    assert not state.run_started() and state.percent == 5.0
    assert not state.run_ended() and state.percent == 5.0


def test_test_mode_clamps_requests_to_the_cap():
    state = OverrideState()
    state.request(test_mode=True)
    ok, message, percent = state.request(80.0)
    assert ok and percent == 50.0 and 'clamped to the test mode cap 50 %' in message
    state.request(test_percent=70.0)
    assert state.test_percent == 50.0
    state.request(test_mode=False)
    assert state.request(80.0)[2] == 80.0


def test_one_request_can_leave_test_mode_and_raise_the_speed():
    state = OverrideState()
    state.request(10.0, test_mode=True)
    ok, message, percent = state.request(100.0, test_mode=False)
    assert ok and percent == 100.0 and not state.test_mode and 'clamped' not in message


def test_one_request_that_keeps_test_mode_on_is_clamped():
    state = OverrideState()
    state.request(10.0, test_mode=True)
    assert state.request(100.0, test_mode=True)[2] == 50.0 and state.test_mode


def test_one_request_can_enter_test_mode_and_is_clamped_at_once():
    state = OverrideState()
    ok, message, percent = state.request(100.0, test_mode=True)
    assert percent == 50.0 and state.test_mode


def test_no_restore_over_a_change_during_the_run():
    state = OverrideState()
    state.request(test_mode=True, test_percent=20.0)
    state.run_started()
    assert state.percent == 20.0
    state.request(40.0)                            # the operator speeds it up during the run
    assert not state.run_ended() and state.percent == 40.0


def test_skill_timeout_runs_on_work_time():
    """A step that takes 10x longer at 10 % does not time out: its nominal timeout is work time."""
    t = {'now': 0.0}
    speed = type('Speed', (), {'fraction': lambda self: 0.1})()
    ctx = SkillContext(lambda: t['now'], 5.0, lambda: False, lambda p, s: None, speed=speed)
    for now in (1.0, 20.0, 49.0):
        t['now'] = now
        assert not ctx.expired
    assert ctx.work_now() == pytest.approx(4.9)
    t['now'] = 50.5
    assert ctx.expired
    plain = SkillContext(lambda: t['now'], 5.0, lambda: False, lambda p, s: None)
    t['now'] = 55.4
    assert not plain.expired and plain.work_now() == plain.elapsed()
    t['now'] = 55.6
    assert plain.expired


class OneStep(Planner):
    def make_plan(self, world, cycle):
        return Plan('p', 'g', 'bot', [Step(id='a', instruction='a', skill='wait', timeout_s=5.0)]) if cycle == 1 \
            else None

    def replan(self, plan, failed_index, reason, world):
        return None


class Dispatcher:
    def __init__(self):
        self.started, self.canceled = [], []

    def dispatch(self, token, step, attempt):
        self.started.append(token)

    def cancel(self, token):
        self.canceled.append(token)


def make(fraction):
    t, episodes, d = {'now': 0.0}, [], Dispatcher()
    ex = Executive(OneStep(), d, WorldState(), clock=lambda: t['now'], on_episode=episodes.append,
                   timeout_grace_s=1.0, speed=(lambda: fraction) if fraction else None)
    ex.start()
    return ex, d, t, episodes


def test_executive_grace_tick_runs_on_the_work_clock():
    ex, d, t, _ = make(0.1)
    t['now'] = 59.0                                # 5.9 work s < 5 + 1
    ex.tick()
    assert d.canceled == []
    t['now'] = 61.0
    ex.tick()
    assert d.canceled == [d.started[-1]]
    ex, d, t, _ = make(None)                       # without an override: wall clock as before
    t['now'] = 6.1
    ex.tick()
    assert d.canceled == [d.started[-1]]


def test_step_finished_carries_the_speed_data():
    ex, d, t, episodes = make(0.5)
    t['now'] = 2.0
    ex.on_result(d.started[-1], Outcome.SUCCEEDED, 'done', {'speed': {'override_min': 50.0, 'override_avg': 50.0}})
    finished = [e for e in episodes if e['event'] == 'step_finished'][0]
    assert finished['speed'] == {'override_min': 50.0, 'override_avg': 50.0}
    assert finished['outcome'] == 'SUCCEEDED' and finished['message'] == 'done'   # existing keys unchanged
    assert ex.status.state == State.SUCCEEDED
    ex, d, t, episodes = make(0.5)                 # no speed in the result (e.g. dispatcher rejected): the handler's
    t['now'] = 2.0
    ex.tick()
    ex.on_result(d.started[-1], Outcome.REJECTED, 'no server')
    finished = [e for e in episodes if e['event'] == 'step_finished'][0]
    assert finished['speed'] == {'override_min': 50.0, 'override_avg': 50.0}


# ---- P1 review fix 5: a lower override applies at once, only increases slew ----
def test_a_decrease_takes_effect_in_the_first_cycle():
    """Test mode lowers 100 -> 10 % just after a clock started: the first move must not run at up to 100 %."""
    target = Target(1.0)
    clock = WorkClock(0.0, target, slew_per_s=2.0)
    target.value = 0.1
    assert clock(0.02) == pytest.approx(0.002)       # the drop may have come at any time in the cycle: the low value
    assert clock.o == pytest.approx(0.1)             # not 1.0 - 2 * 0.02
    assert clock(0.12) == pytest.approx(0.002 + 0.01)
    assert clock.o_min == pytest.approx(0.1)


def test_a_decrease_during_a_ramp_up_is_immediate_too():
    target = Target(0.1)
    clock = WorkClock(0.0, target, slew_per_s=2.0)
    target.value = 1.0
    clock(0.1)
    assert 0.1 < clock.o < 1.0
    target.value = 0.2
    clock(0.12)
    assert clock.o == pytest.approx(0.2)
