"""Motion option resolution (skill, robot, target, step), skill_overrides.yaml and effective speeds."""

import pytest
from jsonschema import Draft202012Validator

from taskweave_core import motion
from taskweave_core.config import ConfigError, ConfigStore, schemas

SKILL = """
motion: {mode: free, planner: auto, approach_m: 0.10, torso: auto}
speed: {joint_rad_s: 1.2, tcp_m_s: 0.05}
accel: {joint_rad_s2: 3.0}
"""


@pytest.fixture(autouse=True)
def reach_skill(config_dir):
    (config_dir / 'skills' / 'reach.yaml').write_text(SKILL)


@pytest.fixture
def store(config_dir):
    return ConfigStore(config_dir)


def write_override(config_dir, text):
    (config_dir / 'robots' / 'demo_bot' / 'skill_overrides.yaml').write_text(text)
    return ConfigStore(config_dir)


def test_skill_layer_only(store):
    got = motion.resolve(store, 'demo_bot', 'reach', {})
    assert got['mode'] == {'value': 'free', 'source': 'skill'}
    assert got['approach_m'] == {'value': 0.10, 'source': 'skill'}
    assert got['standard_speed'] == {'value': {'joint_rad_s': 1.2, 'tcp_m_s': 0.05}, 'source': 'skill'}
    assert got['standard_accel']['value'] == {'joint_rad_s2': 3.0}
    assert 'speed' not in got and 'elbow' not in got        # nothing sets them


def test_unknown_skill_or_no_motion_gives_nothing(store):
    assert motion.resolve(store, 'demo_bot', 'wait', None) == {}
    assert motion.resolve(store, 'demo_bot', 'nonexistent', {}) == {}


def test_a_plain_number_speed_is_not_a_table(config_dir):
    (config_dir / 'skills' / 'drive.yaml').write_text('speed: 0.2\n')
    assert motion.resolve(ConfigStore(config_dir), 'demo_bot', 'drive', {}) == {}


def test_precedence_skill_robot_target_step(config_dir):
    store = write_override(config_dir, """
reach:
  motion: {approach_m: 0.05, elbow: up, planner: planned}
  speed: {joint_rad_s: 0.8}
""")
    target = {'approach_m': 0.2, 'elbow': 'down'}
    step = {'motion': {'mode': 'linear', 'planner': 'direct'}}
    got = motion.resolve(store, 'demo_bot', 'reach', step, target)
    assert got['mode'] == {'value': 'linear', 'source': 'step'}
    assert got['planner'] == {'value': 'direct', 'source': 'step'}      # step beats robot (planned) and skill (auto)
    assert got['approach_m'] == {'value': 0.2, 'source': 'target'}       # target beats robot (0.05) and skill (0.10)
    assert got['elbow'] == {'value': 'down', 'source': 'target'}         # target beats robot (up)
    assert got['torso'] == {'value': 'auto', 'source': 'skill'}
    # no target or step: the robot beats the skill
    got = motion.resolve(store, 'demo_bot', 'reach', {})
    assert got['approach_m'] == {'value': 0.05, 'source': 'robot'} and got['planner']['source'] == 'robot'
    assert got['elbow'] == {'value': 'up', 'source': 'robot'}
    # speeds merge per number
    assert got['standard_speed'] == {'value': {'joint_rad_s': 0.8, 'tcp_m_s': 0.05}, 'source': 'robot'}


def test_step_motion_wins_over_everything_and_target_motion_block(store):
    got = motion.resolve(store, 'demo_bot', 'reach', {'motion': {'approach_m': 0.0}},
                         {'approach_m': 0.3, 'motion': {'torso': 'keep'}})
    assert got['approach_m'] == {'value': 0.0, 'source': 'step'}         # 0 is a value, not "unset"
    assert got['torso'] == {'value': 'keep', 'source': 'target'}
    assert motion.values(got)['mode'] == 'free'


def test_other_robots_do_not_see_the_override(config_dir):
    store = write_override(config_dir, 'reach: {motion: {approach_m: 0.05}}\n')
    assert motion.resolve(store, 'other_bot', 'reach', {})['approach_m']['source'] == 'skill'


def test_robot_overrides_found_in_a_robot_pack(config_dir, tmp_path):
    pack = tmp_path / 'packs' / 'demo_bot'
    pack.mkdir(parents=True)
    (pack / 'pack.yaml').write_text('name: demo_bot\nversion: 1\nschema_version: 1\n')
    (pack / 'skill_overrides.yaml').write_text('reach: {motion: {approach_m: 0.07}}\n')
    (config_dir / 'system.yaml').write_text(f'robot_packs: ["{(tmp_path / "packs").as_posix()}"]\n')
    store = ConfigStore(config_dir)
    assert motion.resolve(store, 'demo_bot', 'reach', {})['approach_m'] == {'value': 0.07, 'source': 'robot'}


def test_skill_overrides_file_is_validated(config_dir):
    assert ConfigStore(config_dir).skill_overrides('demo_bot') == {}      # optional
    for bad in ('reach: {motion: {mode: sideways}}', 'reach: {motion: {nonsense: 1}}', 'reach: {speed: {tcp_m_s: -1}}',
                'reach: {other: 1}', 'bad-name!: {}'):
        with pytest.raises(ConfigError):
            write_override(config_dir, bad + '\n').skill_overrides('demo_bot')


def test_target_data_for_reads_the_point(config_dir):
    (config_dir / 'cell' / 'objects.yaml').write_text('box_1: {object: box, frame: cell, x: 1.0, y: 2.0, yaw: 0.0}\n')
    path = config_dir / 'objects' / 'box.yaml'
    path.write_text(path.read_text().replace('top_grasp:\n', 'top_grasp:\n    approach_m: 0.15\n    elbow: up\n', 1))
    store = ConfigStore(config_dir)
    assert motion.target_data_for(store, 'demo_bot', {'to': {'point': 'box_1/top_grasp'}}) == \
        {'approach_m': 0.15, 'elbow': 'up'}
    assert motion.target_data_for(store, 'demo_bot', {'to': {'point': '{p}'}}) == {}
    assert motion.target_data_for(store, 'demo_bot', {'to': {'point': 'box_1/nope'}}) == {}
    assert motion.target_data_for(store, 'demo_bot', {'to': {'by': [0, 0, 0]}}) == {}


def test_effective_speed():
    assert motion.effective_speed(1.0) == 1.0
    assert motion.effective_speed(1.0, speed=0.5) == 0.5
    assert motion.effective_speed(1.0, limit=0.6) == 0.6                          # capped by the robot limit
    assert motion.effective_speed(1.0, limit=0.6, speed=0.5) == 0.5
    assert motion.effective_speed(1.0, limit=0.6, speed_abs=2.0) == 0.6           # absolute is capped first
    assert motion.effective_speed(1.0, speed=0.5, speed_abs=0.3) == 0.3           # absolute wins over the fraction
    assert motion.effective_speed(1.0, speed=0.5, override=0.5) == 0.25
    assert motion.effective_speed(1.0, override=3.0) == 1.0                        # the override only slows
    assert motion.effective_speed(0.2, limit=None, speed_abs=0.1, override=0.1) == pytest.approx(0.01)


def test_effective_speeds_from_a_resolved_step(store):
    resolved = motion.resolve(store, 'demo_bot', 'reach', {'motion': {'speed': 0.5, 'speed_abs': {'tcp_m_s': 0.04}}})
    got = motion.effective_speeds(resolved, limits={'joint_rad_s': 0.5, 'tcp_m_s': 0.03})
    assert got == {'joint_rad_s': 0.5, 'tcp_m_s': 0.03}
    got = motion.effective_speeds(resolved, limits={}, override=0.5)
    assert got == {'joint_rad_s': pytest.approx(0.3), 'tcp_m_s': pytest.approx(0.02)}


def test_motion_schema_fragment():
    validator = Draft202012Validator(schemas.MOTION)
    assert validator.is_valid({'mode': 'linear', 'speed': 0.5, 'planner': 'direct', 'approach_m': 0.1, 'torso': 'auto',
                               'elbow': 'up', 'accel': 1, 'speed_abs': {'tcp_m_s': 0.1}, 'via': ['pose:home'],
                               'tolerance': 0.01})
    for bad in ({'mode': 'x'}, {'speed': 0}, {'speed': 1.5}, {'speed_abs': {}}, {'planner': 'x'}, {'bogus': 1},
                {'torso': 'x'}, {'accel': 2}):
        assert not validator.is_valid(bad), bad
    props = schemas.MOTION['properties']
    assert schemas.MOTION['x-taskweave-group'] == 'Motion (advanced)'
    assert props['speed']['x-taskweave-unit'] == 'fraction' and props['approach_m']['x-taskweave-unit'] == 'm'
    assert props['approach_m']['x-taskweave-show-if'] == {'mode': ['free']}
    assert props['mode']['default'] == 'free' and props['planner']['default'] == 'auto'
    assert props['via']['items']['x-taskweave-point'] == 'via_target'


def test_motion_schema_subset_is_a_copy():
    subset = schemas.motion_schema(['speed', 'accel'])
    assert list(subset['properties']) == ['speed', 'accel'] and subset['additionalProperties'] is False
    subset['properties']['speed']['default'] = 0.5
    assert schemas.MOTION['properties']['speed']['default'] == 1
    with pytest.raises(KeyError, match='unknown motion option'):
        schemas.motion_schema(['nope'])
