"""Every branch of the decision flow, driven by a fake dispatcher (no ROS, no robot)."""

import pytest

from taskweave_core.executive import Executive, Outcome, State
from taskweave_core.plan import Plan, Step
from taskweave_core.planner import Planner
from taskweave_core.world_state import WorldState


class FakeDispatcher:
    def __init__(self):
        self.started, self.canceled, self.contexts = [], [], []

    def dispatch(self, token, step, attempt, context=None):
        self.started.append((token, step, attempt))
        self.contexts.append(context)

    def cancel(self, token):
        self.canceled.append(token)

    @property
    def last(self):
        return self.started[-1]


class ListPlanner(Planner):
    """Returns the same steps for `cycles` cycles."""

    def __init__(self, steps, cycles=1, replan_to=None):
        self.steps, self.cycles, self.replan_to, self.replans = steps, cycles, replan_to, 0

    def make_plan(self, world, cycle):
        if cycle > self.cycles:
            return None
        return Plan(f'p{cycle}', 'goal', 'bot', [Step(**vars(s)) for s in self.steps])

    def replan(self, plan, failed_index, reason, world):
        self.replans += 1
        return None if self.replan_to is None else (plan, self.replan_to)


def step(step_id, **kw):
    return Step(id=step_id, instruction=f'do {step_id}', skill=kw.pop('skill', 'wait'), **kw)


def make(steps, cycles=1, replan_to=None, world=None, validate=lambda plan: []):
    clock = {'t': 0.0}
    dispatcher, statuses, episodes = FakeDispatcher(), [], []
    executive = Executive(ListPlanner(steps, cycles, replan_to), dispatcher, WorldState(world), validate=validate,
                          clock=lambda: clock['t'], on_status=lambda s: statuses.append(s.state),
                          on_episode=episodes.append, timeout_grace_s=1.0)
    return executive, dispatcher, clock, episodes


def finish(executive, dispatcher, outcome=Outcome.SUCCEEDED, message='', data=None):
    executive.on_result(dispatcher.last[0], outcome, message, data)


def test_runs_all_steps_in_order():
    ex, d, _, episodes = make([step('a'), step('b')])
    ex.start()
    finish(ex, d)
    finish(ex, d)
    assert [s.id for _, s, _ in d.started] == ['a', 'b']
    assert ex.status.state == State.SUCCEEDED
    assert [e['event'] for e in episodes] == ['step_started', 'step_finished'] * 2


def test_step_finished_carries_a_small_summary_of_the_skill_data():
    ex, d, _, episodes = make([step('a'), step('b')])
    ex.start()
    finish(ex, d, data={'path': 'planned', 'plan_s': 1.25, 'hands': ['left'], 'motion': {'speed': 1}, 'max_error': 1})
    finish(ex, d, data={'hands': ['left']})
    done = [e for e in episodes if e['event'] == 'step_finished']
    assert done[0]['data'] == {'path': 'planned', 'plan_s': 1.25} and 'data' not in done[1]


def test_invalid_plan_fails_before_dispatch():
    ex, d, _, _ = make([step('a')], validate=lambda plan: ['unknown skill'])
    assert ex.start().state == State.FAILED
    assert 'unknown skill' in ex.status.last_error and d.started == []


def test_placeholders_and_world_updates():
    ex, d, _, _ = make([step('a', args={'target': '{next}'}), step('b', args={'n': '{count}'})], world={'next': 'x'})
    ex.start()
    assert d.last[1].args == {'target': 'x'}
    finish(ex, d, data={'world_state': {'count': 3}})
    assert d.last[1].args == {'n': 3}


def test_a_step_whose_placeholder_hid_its_kind_is_annotated_when_it_starts():
    ex, d, _, _ = make([step('a', args={'target': '{next}'}), step('b', args={'n': 1})], world={'next': 'x'})
    ex.annotate = lambda s: ('base', ['base']) if s.args.get('target') == 'x' else ('wrong', [])
    ex.start()
    assert (d.last[1].kind, d.last[1].resources) == ('base', ['base'])
    assert (ex.plan.steps[0].kind, ex.plan.steps[0].resources) == (None, [])    # the plan's step stays as validated
    finish(ex, d)
    assert d.last[1].kind is None      # no placeholder: what validation put on the step stays


def test_missing_placeholder_applies_on_failure_without_retries():
    ex, d, _, _ = make([step('a', args={'t': '{nope}'}, retries=3, on_failure='skip'), step('b')])
    ex.start()
    assert [s.id for _, s, _ in d.started] == ['b']


def test_retries_then_abort():
    ex, d, _, _ = make([step('a', retries=2)])
    ex.start()
    finish(ex, d, Outcome.FAILED, 'missed')
    finish(ex, d, Outcome.FAILED, 'missed')
    assert d.last[2] == 3
    finish(ex, d, Outcome.FAILED, 'missed')
    assert ex.status.state == State.FAILED
    assert 'a (wait) failed: missed' in ex.status.last_error


def test_rejected_skips_retries():
    ex, d, _, _ = make([step('a', retries=5, on_failure='abort')])
    ex.start()
    finish(ex, d, Outcome.REJECTED, 'bad args')
    assert len(d.started) == 1 and ex.status.state == State.FAILED


def test_skip_continues_with_next_step():
    ex, d, _, _ = make([step('a', on_failure='skip'), step('b')])
    ex.start()
    finish(ex, d, Outcome.FAILED)
    assert d.last[1].id == 'b'


def test_replan_continues_at_planner_index_and_is_bounded():
    ex, d, _, _ = make([step('drive'), step('pick', on_failure='replan', max_replans=1)], replan_to=0)
    ex.start()
    finish(ex, d)
    finish(ex, d, Outcome.FAILED, 'slipped')
    assert d.last[1].id == 'drive' and d.last[2] == 1
    finish(ex, d)
    finish(ex, d, Outcome.FAILED, 'slipped again')
    assert ex.status.state == State.FAILED
    assert 'max_replans (1) reached' in ex.status.last_error


def test_replan_impossible_fails():
    ex, d, _, _ = make([step('a', on_failure='replan')], replan_to=None)
    ex.start()
    finish(ex, d, Outcome.FAILED)
    assert ex.status.state == State.FAILED and 'could not replan' in ex.status.last_error


def test_on_failure_retry_restarts_with_fresh_attempts():
    ex, d, _, _ = make([step('a', retries=1, on_failure='retry', max_replans=1)])
    ex.start()
    finish(ex, d, Outcome.FAILED)
    finish(ex, d, Outcome.FAILED)
    assert d.last[2] == 1  # fresh attempt counter after on_failure: retry
    finish(ex, d, Outcome.SUCCEEDED)
    assert ex.status.state == State.SUCCEEDED


def test_timeout_cancels_and_counts_as_timeout():
    ex, d, clock, episodes = make([step('a', timeout_s=5.0)])
    ex.start()
    clock['t'] = 5.5
    ex.tick()
    assert d.canceled == []
    clock['t'] = 6.5
    ex.tick()
    assert d.canceled == [d.last[0]]
    finish(ex, d, Outcome.CANCELED)
    assert ex.status.state == State.FAILED
    assert episodes[-1]['outcome'] == 'TIMEOUT'


def test_pause_and_resume_rerun_the_same_step():
    ex, d, _, _ = make([step('a', retries=0), step('b')])
    ex.start()
    assert ex.pause()
    assert d.canceled
    finish(ex, d, Outcome.CANCELED)
    assert ex.status.state == State.PAUSED
    assert ex.resume()
    assert d.last[1].id == 'a' and len(d.started) == 2 and d.last[2] == 1
    finish(ex, d)
    finish(ex, d)
    assert ex.status.state == State.SUCCEEDED


def test_abort_while_running_and_while_paused():
    ex, d, _, _ = make([step('a')])
    ex.start()
    ex.abort()
    finish(ex, d, Outcome.CANCELED)
    assert ex.status.state == State.ABORTED

    ex, d, _, _ = make([step('a')])
    ex.start()
    ex.pause()
    finish(ex, d, Outcome.CANCELED)
    assert ex.abort('stop now')
    assert ex.status.state == State.ABORTED and ex.status.last_error == 'stop now'


def test_stale_results_are_ignored():
    ex, d, _, _ = make([step('a', retries=1), step('b')])
    ex.start()
    old = d.last[0]
    finish(ex, d, Outcome.FAILED)
    ex.on_result(old, Outcome.SUCCEEDED)  # late result of the first attempt
    assert d.last[1].id == 'a' and d.last[2] == 2


def test_loops_for_planner_cycles_then_succeeds():
    ex, d, _, _ = make([step('a')], cycles=3)
    ex.start()
    for _ in range(3):
        finish(ex, d)
    assert ex.status.cycle == 3 and ex.status.state == State.SUCCEEDED
    assert len(d.started) == 3


def test_cannot_start_twice():
    ex, d, _, _ = make([step('a')])
    ex.start()
    with pytest.raises(RuntimeError):
        ex.start()


def test_start_step_skips_the_earlier_steps_in_the_first_cycle_only():
    ex, d, _, _ = make([step('a'), step('b'), step('c')], cycles=2)
    ex.start('b')
    assert ex.status.step_index == 1
    for _ in range(5):
        finish(ex, d)
    assert [s.id for _, s, _ in d.started] == ['b', 'c', 'a', 'b', 'c']
    assert ex.status.state == State.SUCCEEDED


def test_unknown_start_step_fails_before_dispatch():
    ex, d, _, _ = make([step('a')])
    ex.start('zz')
    assert ex.status.state == State.FAILED and 'zz' in ex.status.last_error
    assert d.started == []


class PreviewPlanner(ListPlanner):
    def preview(self, cycle):
        return 'p1', len(self.steps), 'the goal'


def test_the_status_shows_the_plan_while_it_is_prepared():
    """The plan is validated before the first step: meanwhile the status is running with the plan id, the step count
    and 'planning' as step status (no step), so a slow validation does not look like a mission without a plan."""
    seen = []
    ex, d, _, _ = make([step('a'), step('b')])
    ex.planner = PreviewPlanner([step('a'), step('b')])
    ex.on_status = lambda s: seen.append((s.state, s.plan_id, s.step_count, s.step_index, s.step_status, s.goal))
    ex.validate = lambda plan: seen.append('validating') or []
    ex.start()
    assert seen[0] == (State.RUNNING, 'p1', 2, -1, 'planning', 'the goal')
    assert seen[1] == 'validating'
    assert seen[2][:4] == (State.RUNNING, 'p1', 2, 0) and seen[2][4] == 'dispatched'


def test_a_rejected_plan_leaves_no_preview_behind():
    ex, d, _, _ = make([step('a')])
    ex.planner = PreviewPlanner([step('a')])
    ex.validate = lambda plan: ['no good']
    status = ex.start()
    assert status.state == State.FAILED and 'no good' in status.last_error
    assert (status.plan_id, status.step_count, status.step_status, status.goal) == ('', 0, '', '')


def test_an_abort_asked_for_while_the_start_was_prepared_ends_it_before_a_step_runs():
    ex, d, _, _ = make([step('a')])
    status = ex.start(early=lambda: 'abort')
    assert status.state == State.ABORTED and status.last_error == 'aborted by operator'
    assert d.started == [] and ex.plan is None


def test_a_pause_asked_for_while_the_start_was_prepared_pauses_the_first_step():
    ex, d, _, _ = make([step('a'), step('b')])
    ex.start(early=lambda: 'pause')
    assert d.canceled == [d.last[0]]    # the dispatched first step is canceled
    finish(ex, d, Outcome.CANCELED)
    assert ex.status.state == State.PAUSED


def test_an_exception_while_the_start_is_prepared_ends_in_failed_with_the_error():
    """Not left running without a plan: every later start would be refused as 'already active'."""
    def boom(plan):
        raise ValueError('no catalog')

    ex, d, _, _ = make([step('a')], validate=boom)
    ex.planner = PreviewPlanner([step('a')])
    status = ex.start()
    assert status.state == State.FAILED and 'ValueError: no catalog' in status.last_error
    assert (status.plan_id, status.step_count, status.step_status) == ('', 0, '')
    assert d.started == []
    ex.start()   # a new start is not refused


def ahead(context):
    return [(a['id'], a['resolved']) for a in context['ahead']]


def test_context_on_first_run_lists_the_rest_of_the_cycle():
    ex, d, _, _ = make([step('a'), step('b', args={'n': 1}, skill='move'), step('c')])
    ex.start()
    assert d.contexts[0] == {'plan_id': 'p1', 'cycle': 1, 'step_index': 0, 'ahead': [
        {'id': 'b', 'skill': 'move', 'kind': None, 'args': {'n': 1}, 'resolved': True},
        {'id': 'c', 'skill': 'wait', 'kind': None, 'args': {}, 'resolved': True}]}
    finish(ex, d)
    finish(ex, d)
    assert [ahead(c) for c in d.contexts] == [[('b', True), ('c', True)], [('c', True)], []]
    assert [c['step_index'] for c in d.contexts] == [0, 1, 2]


def test_context_is_the_same_on_retry():
    ex, d, _, _ = make([step('a', retries=1), step('b')])
    ex.start()
    finish(ex, d, Outcome.FAILED)
    assert [s.id for _, s, _ in d.started] == ['a', 'a'] and d.contexts[0] == d.contexts[1]


def test_context_after_replan_is_the_new_plans_rest():
    class Swap(ListPlanner):
        def replan(self, plan, failed_index, reason, world):
            return Plan('r1', 'goal', 'bot', [step('x'), step('y'), step('z')]), 1

    d = FakeDispatcher()
    ex = Executive(Swap([step('a', on_failure='replan'), step('b')]), d, WorldState(), clock=lambda: 0.0)
    ex.start()
    finish(ex, d, Outcome.FAILED)
    assert d.last[1].id == 'y'
    assert d.contexts[-1]['plan_id'] == 'r1' and ahead(d.contexts[-1]) == [('z', True)]


def test_context_on_start_mid_plan_is_the_rest_from_that_step():
    ex, d, _, _ = make([step('a'), step('b'), step('c'), step('d')])
    ex.start('b')
    assert d.contexts[0]['step_index'] == 1 and ahead(d.contexts[0]) == [('c', True), ('d', True)]


def test_context_resolves_placeholders_and_flags_open_ones():
    ex, d, _, _ = make([step('a'), step('b', args={'lane': '{lane}'}), step('c', args={'to': '{later}'})],
                       world={'lane': 2})
    ex.start()
    by_id = {a['id']: a for a in d.contexts[0]['ahead']}
    assert by_id['b']['args'] == {'lane': 2} and by_id['b']['resolved'] is True
    assert by_id['c']['args'] == {'to': '{later}'} and by_id['c']['resolved'] is False


def test_context_kind_is_annotated_once_a_placeholder_is_filled():
    ex, d, _, _ = make([step('a'), step('b', args={'k': '{k}'})], world={'k': 'hand'})
    ex.annotate = lambda s: (s.args['k'], [])
    ex.start()
    assert d.contexts[0]['ahead'][0]['kind'] == 'hand'


def test_a_failing_context_never_stops_the_step():
    def boom(s):
        raise TypeError('malformed step')

    ex, d, _, _ = make([step('a'), step('b', args={'k': '{k}'})], world={'k': 1})
    ex.annotate = boom
    ex.start()
    assert [s.id for _, s, _ in d.started] == ['a'] and d.contexts == [{}]
    assert ex.status.state == State.RUNNING and any('no look-ahead' in line for line in ex.status.log)
    finish(ex, d)
    assert d.contexts[-1] == {'plan_id': 'p1', 'cycle': 1, 'step_index': 1, 'ahead': []}
