"""The clock guard's decisions (pure functions, no ROS) and the bring-up argument that adds the node."""

import pytest

from taskweave_bringup import clock_guard as cg

T = cg.Timing(jump_s=0.5, settle_s=2.0, cooldown_s=20.0, inactive_s=15.0, steady_s=1.0)


def test_a_jump_back_beyond_the_threshold_is_a_reset():
    assert cg.is_jump(100.0, 3.0, T.jump_s)
    assert cg.is_jump(100.0, 99.4, T.jump_s)
    assert not cg.is_jump(100.0, 99.6, T.jump_s)   # small backward jitter
    assert not cg.is_jump(100.0, 130.0, T.jump_s)  # forward is not a reset
    assert not cg.is_jump(None, 3.0, T.jump_s)     # first message


def test_a_restart_waits_for_the_settle_time_and_a_steady_clock():
    pending, restarted = 100.0, -T.cooldown_s
    assert not cg.nav2_reset_due(pending, 101.0, 100.9, restarted, T)   # settle_s not over
    assert not cg.nav2_reset_due(pending, 103.0, 101.0, restarted, T)   # clock stopped 2 s ago: Stop pressed
    assert cg.nav2_reset_due(pending, 103.0, 102.8, restarted, T)
    assert not cg.nav2_reset_due(None, 103.0, 102.8, restarted, T)      # nothing pending
    assert not cg.nav2_reset_due(pending, 103.0, None, restarted, T)    # no clock seen at all


def test_the_cooldown_holds_back_a_second_restart():
    assert not cg.nav2_reset_due(100.0, 103.0, 102.9, 90.0, T)    # restarted 13 s ago
    assert cg.nav2_reset_due(100.0, 103.0, 102.9, 80.0, T)        # 23 s ago


def test_move_group_restarts_once_the_clock_is_steady_with_no_cooldown():
    assert not cg.move_group_restart_due(None, 103.0, 102.9, T)
    assert not cg.move_group_restart_due(100.0, 101.0, 100.9, T)
    assert not cg.move_group_restart_due(100.0, 103.0, 101.0, T)
    assert cg.move_group_restart_due(100.0, 103.0, 102.9, T)


def test_an_inactive_navigator_restarts_nav2_only_after_the_inactive_time_while_the_clock_runs():
    since, restart = cg.inactive_update(None, active=False, clock_runs=True, now=100.0, timing=T)
    assert (since, restart) == (100.0, False)
    assert cg.inactive_update(since, False, True, 110.0, T) == (100.0, False)
    assert cg.inactive_update(since, False, True, 116.0, T) == (None, True)
    assert cg.inactive_update(since, True, True, 116.0, T) == (None, False)    # became active
    assert cg.inactive_update(since, False, False, 116.0, T) == (None, False)  # the sim is stopped: not a failure


def test_stale_tf_log_matches_only_the_configured_node_and_text():
    assert cg.stale_tf_log('controller_server', 'Failed to transform end pose to global frame (x)',
                           'controller_server', 'Failed to transform')
    assert not cg.stale_tf_log('planner_server', 'Failed to transform end pose', 'controller_server', 'Failed')
    assert not cg.stale_tf_log('controller_server', 'all fine', 'controller_server', 'Failed')
    assert not cg.stale_tf_log('controller_server', 'Failed', 'controller_server', '')  # trigger switched off


# ---- launch argument --------------------------------------------------------------------------------------------
launch = pytest.importorskip('launch_ros')
from test_launch import bringup, described  # noqa: E402


def test_the_guard_node_is_off_by_default(monkeypatch, cell):
    nodes, context = bringup(monkeypatch, cell, hmi='false')
    assert 'clock_guard' not in described(nodes, context)


def test_the_guard_node_comes_with_clock_guard_true(monkeypatch, cell):
    nodes, context = bringup(monkeypatch, cell, hmi='false', moveit='true', clock_guard='true')
    guard = described(nodes, context)['clock_guard']
    assert (guard['package'], guard['executable']) == ('taskweave_bringup', 'clock_guard')
    parameters = guard['parameters'][0]
    assert parameters['nav2'] is False and parameters['move_group'] is True and parameters['use_sim_time'] is False


def test_the_guard_params_file_comes_after_the_defaults(monkeypatch, cell):
    nodes, context = bringup(monkeypatch, cell, hmi='false', clock_guard='true', clock_guard_params='/etc/guard.yaml')
    assert described(nodes, context)['clock_guard']['parameters'][1] == {'file': '/etc/guard.yaml'}
