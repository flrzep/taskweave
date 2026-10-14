"""navigation.yaml -> Nav2 parameters (no ROS graph needed; the module imports ROS message types)."""

import pytest

pytest.importorskip('rclpy')

from taskweave_skills.nav2 import nav2_parameters  # noqa: E402

BASE = {'type': 'holonomic', 'footprint': [0.6, 0.4]}
SETTINGS = {'envelope': {'front': 0.75, 'back': 0.3, 'left': 0.45, 'right': 0.4}, 'padding': 0.03,
            'inflation_radius': 0.65,
            'driving': {'max_forward': 0.6, 'max_reverse': 0.2, 'max_turn': 1.0, 'sideways': True, 'max_sideways': 0.3,
                        'prefer_forward': 5.0}}


def test_envelope_padding_and_inflation_go_to_the_costmaps():
    costmap = nav2_parameters(SETTINGS, BASE)['costmap']
    assert costmap['footprint'] == '[[0.75, 0.45], [0.75, -0.4], [-0.3, -0.4], [-0.3, 0.45]]'
    assert costmap['footprint_padding'] == 0.03 and costmap['inflation_layer.inflation_radius'] == 0.65
    assert nav2_parameters({}, BASE)['costmap']['footprint'] == '[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]'


def test_driving_goes_to_the_controller_with_a_forward_preference():
    values = nav2_parameters(SETTINGS, BASE)
    controller = values['controller']
    assert controller['FollowPath.vx_max'] == 0.6 and controller['FollowPath.vx_min'] == -0.2
    assert controller['FollowPath.vy_max'] == 0.3 and controller['FollowPath.wz_max'] == 1.0
    assert controller['FollowPath.PathAngleCritic.mode'] == 0
    assert controller['FollowPath.PreferForwardCritic.cost_weight'] == 5.0
    assert values['motion_model'] == 'Omni'
    neutral = nav2_parameters({**SETTINGS, 'driving': {'prefer_forward': 0.0}}, BASE)['controller']
    assert neutral['FollowPath.PathAngleCritic.mode'] == 2


def test_a_base_that_is_not_holonomic_never_drives_sideways():
    values = nav2_parameters(SETTINGS, {**BASE, 'type': 'differential'})
    assert values['controller']['FollowPath.vy_max'] == 0.0 and values['motion_model'] == 'DiffDrive'
    assert nav2_parameters({}, {**BASE, 'type': 'differential'})['controller']['FollowPath.vy_max'] == 0.0


def test_carrying_uses_the_carry_envelope():
    settings = {**SETTINGS, 'carry_envelope': {'front': 1.3, 'back': 0.3, 'left': 0.45, 'right': 0.4}}
    assert nav2_parameters(settings, BASE, carrying=True)['costmap']['footprint'].startswith('[[1.3, 0.45]')
    assert nav2_parameters(settings, BASE)['costmap']['footprint'].startswith('[[0.75, 0.45]')
