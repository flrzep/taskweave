"""MoveIt move_group for a taskweave robot, from the robot's ``moveit/`` folder (cell pack copy, else robot pack):

robot.urdf (collision hulls in meshes/, made absolute here), robot.srdf (planning groups, disabled collision pairs),
kinematics.yaml (IK solvers) and joint_limits.yaml (speeds and accelerations planned trajectories are timed with).
move_group serves /compute_ik and planning; the robot state comes from /joint_states. Arm motions are executed by
taskweave's joint skills (targets on the profile's joint command topic), so no MoveIt controllers are configured.
A robot without moveit/robot.urdf gets no move_group.
"""

from __future__ import annotations

import yaml

from . import robot_files

OMPL = {  # move_group needs a pipeline even when only IK is used; OMPL with MoveIt's default adapters
    'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
    'ompl': {
        'planning_plugins': ['ompl_interface/OMPLPlanner'],
        'request_adapters': ['default_planning_request_adapters/ResolveConstraintFrames',
                             'default_planning_request_adapters/ValidateWorkspaceBounds',
                             'default_planning_request_adapters/CheckStartStateBounds',
                             'default_planning_request_adapters/CheckStartStateCollision'],
        'response_adapters': ['default_planning_response_adapters/AddTimeOptimalParameterization',
                              'default_planning_response_adapters/ValidateSolution',
                              'default_planning_response_adapters/DisplayMotionPath'],
    },
}


def moveit_parameters(config_dir: str, robot: str, use_sim_time: bool = True) -> list[dict] | None:
    """The parameter dictionaries of move_group, or None when the robot has no moveit/ with a robot.urdf."""
    store = robot_files.open_store(config_dir)
    folder = robot_files.moveit_folder(store, robot)
    if folder is None:
        return None

    def read(name: str) -> str:
        return (folder / name).read_text(encoding='utf-8')

    description = {
        'robot_description': read('robot.urdf').replace('filename="meshes/', f'filename="file://{folder}/meshes/'),
        'robot_description_semantic': read('robot.srdf'),
        'robot_description_kinematics': yaml.safe_load(read('kinematics.yaml')),
        'use_sim_time': use_sim_time}
    if (folder / 'joint_limits.yaml').is_file():  # speeds/accelerations for trajectory timing
        description['robot_description_planning'] = yaml.safe_load(read('joint_limits.yaml'))
    return [description, OMPL, {'publish_robot_description_semantic': True, 'allow_trajectory_execution': False,
                                'planning_scene_monitor_options.joint_state_topic': '/joint_states'}]


def moveit_nodes(config_dir: str, robot: str, use_sim_time: bool = True) -> list:
    """move_group, respawned on exit (an application may end it on purpose, e.g. after a simulator reset)."""
    from launch_ros.actions import Node

    parameters = moveit_parameters(config_dir, robot, use_sim_time)
    if parameters is None:
        return []
    return [Node(package='moveit_ros_move_group', executable='move_group', name='move_group', output='screen',
                 respawn=True, respawn_delay=2.0, parameters=parameters)]

