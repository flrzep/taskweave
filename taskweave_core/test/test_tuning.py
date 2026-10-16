"""Robot profile motion_tuning and arms: the schema's documented defaults, the accessor, the S1 pack's values."""

from pathlib import Path

import jsonschema
import pytest
import yaml

from taskweave_core.config import schemas
from taskweave_core.config.tuning import motion_tuning

PACK_S1 = Path(__file__).resolve().parents[2] / 'taskweave_robots' / 'galbot_s1' / 'profile.yaml'


def test_without_a_section_every_value_is_the_schemas_default():
    tuning = motion_tuning()
    assert tuning == schemas.MOTION_TUNING_DEFAULTS and tuning is not schemas.MOTION_TUNING_DEFAULTS
    for group, values in tuning.items():
        for key, value in values.items():
            assert schemas.default(schemas.ROBOT_PROFILE, 'motion_tuning', group, key) == value
    assert motion_tuning({'name': 'x'}) == motion_tuning(None) == tuning
    tuning['elbow']['rate_rad'] = 1.0                                     # a copy: the defaults stay
    assert motion_tuning()['elbow']['rate_rad'] == 0.03


def test_the_profile_overrides_single_keys():
    profile = type('Profile', (), {'data': {'motion_tuning': {'elbow': {'rate_rad': 0.015}, 'plan': {'tries': 5}}}})()
    tuning = motion_tuning(profile)
    assert tuning['elbow']['rate_rad'] == 0.015 and tuning['plan']['tries'] == 5
    assert tuning['elbow']['settled_rad'] == 0.1 and tuning['chain'] == schemas.MOTION_TUNING_DEFAULTS['chain']


def test_the_schema_checks_tuning_and_elbow_descriptions():
    base = {'name': 'bot', 'base': {'type': 'fixed', 'cmd_vel': '/c', 'odom': '/o', 'frame': 'base'}}
    elbow = {'shoulder': 'l2', 'elbow': 'l4', 'wrist': 'l6'}
    jsonschema.validate({**base, 'arms': {'left': {'elbow': elbow}, 'both': {'group': 'both_arms'}},
                         'motion_tuning': {'line': {'step_iterations': 4}}}, schemas.ROBOT_PROFILE)
    for bad in ({'motion_tuning': {'line': {'step_iteration': 4}}},      # misspelt
                {'motion_tuning': {'line': {'step_iterations': 0.5}}},   # an integer
                {'motion_tuning': {'chain': {'damping': -1}}},
                {'arms': {'left': {'elbow': {'shoulder': 'l2', 'elbow': 'l4'}}}}):   # no wrist
        with pytest.raises(jsonschema.ValidationError):
            jsonschema.validate({**base, **bad}, schemas.ROBOT_PROFILE)


def test_other_documented_defaults():
    assert schemas.default(schemas.ROBOT_PROFILE, 'ik', 'choose', 'limit_margin_deg') == 25.0
    assert schemas.default(schemas.ROBOT_PROFILE, 'ik', 'choose', 'weights', 'motion') == 0.2
    assert schemas.default(schemas.ROBOT_PROFILE, 'grippers', '*', 'closed_from') == 0.1
    assert schemas.default(schemas.NAVIGATION, 'driving', 'any_heading_weight') == 2.0


def test_the_s1_pack_has_todays_values_and_an_elbow_per_arm():
    if not PACK_S1.is_file():
        pytest.skip('taskweave_robots not next to taskweave_core')
    profile = yaml.safe_load(PACK_S1.read_text())
    jsonschema.validate(profile, schemas.ROBOT_PROFILE)
    assert profile['motion_tuning'] == schemas.MOTION_TUNING_DEFAULTS   # every key written out, today's values
    assert profile['arms']['left']['elbow'] == {'shoulder': 'left_link2', 'elbow': 'left_link4', 'wrist': 'left_link6'}
    assert set(profile['arms']) == set(profile['ik']['groups'])
