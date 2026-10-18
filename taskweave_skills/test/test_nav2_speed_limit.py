"""The Nav2 skill's SpeedLimit: by default only 'no limit' goes out (the cmd_vel speed gate scales Nav2), with
speed_limit: true the override as a percentage (nav2.py _limit_speed). No ROS needed."""

from types import SimpleNamespace

from taskweave_skills.nav2 import NAV2_DEFAULTS, Nav2NavigateTo


def skill(**nav):
    sent = []
    return SimpleNamespace(nav={**NAV2_DEFAULTS, **nav}, _speed_sent=None, speed_limit=SimpleNamespace(
        publish=sent.append), SpeedLimit=lambda **kw: kw, sent=sent)


def test_by_default_a_slowed_override_still_sends_no_limit():
    s = skill()
    Nav2NavigateTo._limit_speed(s, 0.5)
    Nav2NavigateTo._limit_speed(s, 0.1)
    assert s.sent == [{'percentage': True, 'speed_limit': 0.0}]   # once, 'no limit': the gate does the slowing


def test_with_speed_limit_on_the_override_goes_to_nav2_as_a_percentage():
    s = skill(speed_limit=True)
    Nav2NavigateTo._limit_speed(s, 0.5)
    Nav2NavigateTo._limit_speed(s, 1.0)
    assert s.sent == [{'percentage': True, 'speed_limit': 50.0}, {'percentage': True, 'speed_limit': 0.0}]
