"""Start taskweave for one robot: task handler, a skill server with the given skills, operator buttons.

ros2 launch taskweave_core taskweave.launch.py config_dir:=/path/to/config robot:=a2d \
    skills:="taskweave_skills.basic:Wait,taskweave_skills.navigate:NavigateTo"

Applications with their own skills either list them here or start additional skill servers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_SKILLS = 'taskweave_skills.basic:Wait,taskweave_skills.navigate:NavigateTo'


def nodes(context):
    config_dir = LaunchConfiguration('config_dir').perform(context)
    robot = LaunchConfiguration('robot').perform(context)
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('1', 'true', 'yes')
    skills = [s.strip() for s in LaunchConfiguration('skills').perform(context).split(',') if s.strip()]
    common = {'config_dir': config_dir, 'use_sim_time': sim_time}
    return [
        Node(package='taskweave_core', executable='task_handler', name='task_handler', output='screen',
             parameters=[{**common, 'robot': robot}]),
        Node(package='taskweave_core', executable='skill_server', name='generic_skills', output='screen',
             parameters=[{**common, 'robot': robot, 'skills': skills}]),
        Node(package='taskweave_core', executable='operator_joy', name='operator_joy', output='screen',
             parameters=[common]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', description='taskweave config directory'),
        DeclareLaunchArgument('robot', description='robot profile name'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('skills', default_value=DEFAULT_SKILLS,
                              description='comma-separated "package.module:Class" skill paths'),
        OpaqueFunction(function=nodes),
    ])
