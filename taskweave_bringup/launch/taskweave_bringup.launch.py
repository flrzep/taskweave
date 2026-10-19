"""taskweave for a cell and robot: task handler, skill server, pad nodes, the web HMI, optionally Nav2 and MoveIt.

ros2 launch taskweave_bringup taskweave_bringup.launch.py config_dir:=/path/to/cell robot:=<robot> \
    skills:="wait,navigate_to,move_joints" nav2:=true map:=/path/map.yaml moveit:=true

What is specific to a simulator or to hardware (clock handling, sensor bridges, the map's source and the transform that
attaches it, recording tools) is not here: the application launches it next to this file. Nav2 needs ``map`` (a Nav2
map yaml) and gets the map frame from the application too. See taskweave_nav2.launch.py for the Nav2 tuning files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from taskweave_bringup.args import flag
from taskweave_bringup.hmi import hmi_nodes
from taskweave_bringup.moveit import moveit_nodes
from taskweave_bringup.nav2 import nav2_nodes


def taskweave_launch_file() -> str:
    return os.path.join(get_package_share_directory('taskweave_core'), 'launch', 'taskweave.launch.py')


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    config_dir, robot, sim_time = value('config_dir'), value('robot'), flag(value('use_sim_time'))
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(taskweave_launch_file()),
        launch_arguments={'config_dir': config_dir, 'robot': robot, 'skills': value('skills'),
                          'use_sim_time': str(sim_time).lower()}.items())]
    if flag(value('hmi')):
        actions += hmi_nodes(config_dir, robot, sim_time, value('hmi_host'), int(value('hmi_port')),
                             value('hmi_recordings_dir'), value('hmi_recording_post'))
    if flag(value('moveit')):
        actions += moveit_nodes(config_dir, robot, sim_time)
    if flag(value('nav2')):
        actions += nav2_nodes(config_dir, robot, value('map'), value('nav2_output_dir'), sim_time,
                              value('nav2_params'), value('map_topic'))
    return actions


def arguments():
    return [
        DeclareLaunchArgument('config_dir', description='taskweave config directory (the cell pack)'),
        DeclareLaunchArgument('robot', description='robot name (its pack and profile)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('skills', default_value='wait,navigate_to',
                              description='comma-separated skill names (see config/skills/)'),
        DeclareLaunchArgument('hmi', default_value='true', description='start the web HMI'),
        DeclareLaunchArgument('hmi_host', default_value='127.0.0.1',
                              description='HMI bind address (no login: a trusted network address only)'),
        DeclareLaunchArgument('hmi_port', default_value='8090'),
        DeclareLaunchArgument('hmi_recordings_dir', default_value='',
                              description='folder for HMI recordings (empty: recording off)'),
        DeclareLaunchArgument('hmi_recording_post', default_value='',
                              description='command run after each recording, {dir} = its folder'),
        DeclareLaunchArgument('nav2', default_value='false', description='start Nav2 (needs map:=)'),
        DeclareLaunchArgument('map', default_value='', description='path of the Nav2 map yaml (nav2:=true)'),
        DeclareLaunchArgument('map_topic', default_value='/map', description='topic the map server publishes'),
        DeclareLaunchArgument('nav2_params', default_value='',
                              description='Nav2 tuning file replacing the packaged default'),
        DeclareLaunchArgument('nav2_output_dir', default_value='',
                              description='where the generated nav2_params.yaml is written (default: a temp folder)'),
        DeclareLaunchArgument('moveit', default_value='false', description='start MoveIt move_group'),
    ]


def generate_launch_description():
    return LaunchDescription([*arguments(), OpaqueFunction(function=nodes)])
