"""The launch descriptions are built without running them (no node starts): every argument combination yields the
expected nodes, remappings and parameters. Needs launch_ros; no ROS graph."""

import itertools

import pytest
import yaml

pytest.importorskip('launch_ros')
pytest.importorskip('taskweave_skills')

from conftest import CORE_LAUNCH, SHIPPED_PACKS, load_launch, write  # noqa: E402
from describe import describe  # noqa: E402
from launch import LaunchContext  # noqa: E402
from launch.actions import ExecuteProcess  # noqa: E402
from launch_ros.actions import Node  # noqa: E402

TASKWEAVE = {'task_handler', 'generic_skills', 'operator_joy', 'teach', 'tcp_jog'}
HMI = {'hmi_server'}
MOVEIT = {'move_group'}
NAV2 = {'map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator',
        'lifecycle_manager_navigation'}


def context_for(module, **given) -> LaunchContext:
    """A context with the module's declared arguments at their defaults, then ``given`` over them."""
    context = LaunchContext()
    for argument in module.arguments():
        default = argument.default_value
        if default is not None:
            context.launch_configurations[argument.name] = ''.join(s.perform(context) for s in default)
    context.launch_configurations.update({k: str(v) for k, v in given.items()})
    return context


def expand(entities, context) -> list:
    """The nodes and processes of a description, includes and opaque functions resolved (nothing is executed)."""
    found = []
    for entity in entities:
        if isinstance(entity, Node | ExecuteProcess):
            found.append(entity)
        else:
            found += expand(entity.visit(context) or [], context)
    return found


def bringup(monkeypatch, cell, **given):
    module = load_launch('taskweave_bringup.launch.py')
    monkeypatch.setattr(module, 'taskweave_launch_file', lambda: str(CORE_LAUNCH))  # this checkout's, not an install
    context = context_for(module, config_dir=cell, robot='test_bot', **given)
    return expand(module.nodes(context), context), context


def described(nodes, context) -> dict:
    return {d['name']: d for d in (describe(n, context) for n in nodes)}


@pytest.fixture(autouse=True)
def no_env_packs(monkeypatch):
    monkeypatch.delenv('TASKWEAVE_ROBOT_PACKS', raising=False)


@pytest.mark.parametrize('hmi,moveit,nav2,sim', itertools.product([True, False], repeat=4))
def test_every_combination(monkeypatch, cell, tmp_path, hmi, moveit, nav2, sim):
    nodes, context = bringup(monkeypatch, cell, hmi=str(hmi).lower(), moveit=str(moveit).lower(),
                             nav2=str(nav2).lower(), use_sim_time=str(sim).lower(), map='/maps/cell.yaml',
                             nav2_output_dir=tmp_path / 'nav', hmi_host='10.0.0.2', hmi_port='8123')
    expected = TASKWEAVE | (HMI if hmi else set()) | (MOVEIT if moveit else set()) | (NAV2 if nav2 else set())
    assert sorted(d['name'] for d in (describe(n, context) for n in nodes)) == sorted(expected)
    for node in nodes:
        merged = {}
        for parameter in describe(node, context)['parameters']:
            if isinstance(parameter, dict) and 'file' not in parameter:
                merged.update(parameter)
        assert merged.get('use_sim_time', sim) is sim


def test_taskweave_arguments_pass_through(monkeypatch, cell):
    nodes, context = bringup(monkeypatch, cell, skills='wait,grip', hmi='false')
    parameters = described(nodes, context)['generic_skills']['parameters'][0]
    assert parameters['skills'] == ['wait', 'grip']
    assert parameters['robot'] == 'test_bot'
    assert parameters['config_dir'] == str(cell)


def test_hmi_parameters(monkeypatch, cell):
    nodes, context = bringup(monkeypatch, cell, hmi_host='10.0.0.2', hmi_port='8123')
    hmi = described(nodes, context)['hmi_server']
    assert hmi['parameters'][0] == {'config_dir': str(cell), 'robot': 'test_bot', 'use_sim_time': True,
                                    'host': '10.0.0.2', 'port': 8123}
    assert hmi['respawn'] is True
    nodes, context = bringup(monkeypatch, cell, hmi_recordings_dir='/rec', hmi_recording_post='post {dir}')
    parameters = described(nodes, context)['hmi_server']['parameters'][0]
    assert parameters['recordings_dir'] == '/rec'
    assert parameters['recording_post'] == 'post {dir}'


def test_hmi_default_is_loopback(monkeypatch, cell):
    nodes, context = bringup(monkeypatch, cell)
    assert described(nodes, context)['hmi_server']['parameters'][0]['host'] == '127.0.0.1'


def test_nav2_remappings_and_params(monkeypatch, cell, tmp_path):
    nodes, context = bringup(monkeypatch, cell, hmi='false', nav2='true', map='/maps/cell.yaml', map_topic='/cell_map',
                             nav2_output_dir=tmp_path / 'nav')
    by_name = described(nodes, context)
    assert by_name['map_server']['remappings'] == [('map', '/cell_map')]
    assert by_name['controller_server']['remappings'] == [('cmd_vel', '/test/cmd_vel')]
    assert by_name['behavior_server']['remappings'] == [('cmd_vel', '/test/cmd_vel')]
    assert by_name['planner_server']['remappings'] == []
    assert by_name['lifecycle_manager_navigation']['parameters'][1]['node_names'] == [
        'map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    generated = tmp_path / 'nav' / 'nav2_params.yaml'
    params = yaml.safe_load(generated.read_text(encoding='utf-8'))
    assert params['map_server']['ros__parameters']['yaml_filename'] == '/maps/cell.yaml'
    assert by_name['controller_server']['parameters'][0] == {'file': str(generated)}


def test_nav2_needs_a_map(cell, tmp_path):
    module = load_launch('taskweave_nav2.launch.py')
    context = context_for(module, config_dir=cell, robot='test_bot', nav2_output_dir=tmp_path / 'nav')
    with pytest.raises(ValueError, match='map'):
        module.nodes(context)


def test_moveit_launch(cell):
    module = load_launch('taskweave_moveit.launch.py')
    context = context_for(module, config_dir=cell, robot='test_bot')
    [node] = expand(module.nodes(context), context)
    move_group = describe(node, context)
    assert (move_group['package'], move_group['executable']) == ('moveit_ros_move_group', 'move_group')
    assert move_group['respawn'] is True and move_group['respawn_delay'] == 2.0
    description, planning, extra = move_group['parameters']
    folder = cell.parent / 'packs' / 'test_bot' / 'moveit'
    assert f'file://{folder}/meshes/hull.stl' in description['robot_description']
    assert description['robot_description_semantic'].startswith('<robot name="test_bot"')
    solver = description['robot_description_kinematics.arm.kinematics_solver']
    assert solver == 'kdl_kinematics_plugin/KDLKinematicsPlugin'
    assert description['robot_description_planning.joint_limits.test_joint.max_velocity'] == 1.5
    assert description['use_sim_time'] is True
    assert planning['default_planning_pipeline'] == 'ompl'
    assert extra['allow_trajectory_execution'] is False


def test_moveit_off_without_a_moveit_folder(no_moveit_cell):
    module = load_launch('taskweave_moveit.launch.py')
    context = context_for(module, config_dir=no_moveit_cell, robot='test_bot')
    assert module.nodes(context) == []


def test_standalone_descriptions_build():
    for name in ('taskweave_bringup.launch.py', 'taskweave_nav2.launch.py', 'taskweave_moveit.launch.py'):
        assert load_launch(name).generate_launch_description().entities


def test_shipped_pack_moveit(cell):
    """A shipped robot pack with a moveit/ folder gives a move_group from the pack's files."""
    if not (SHIPPED_PACKS / 'galbot_s1' / 'moveit').is_dir():
        pytest.skip('no shipped robot packs')
    write(cell / 'system.yaml', f'robot: galbot_s1\nrobot_packs: [{SHIPPED_PACKS}]\n')
    module = load_launch('taskweave_moveit.launch.py')
    context = context_for(module, config_dir=cell, robot='galbot_s1')
    [node] = expand(module.nodes(context), context)
    assert any(k.startswith('robot_description_planning.') for k in describe(node, context)['parameters'][0])
