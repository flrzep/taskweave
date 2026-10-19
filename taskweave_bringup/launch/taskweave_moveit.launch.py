"""MoveIt move_group for the robot of a cell, from the robot's moveit/ folder (cell pack copy, else robot pack).

ros2 launch taskweave_bringup taskweave_moveit.launch.py config_dir:=/path/to/cell robot:=<robot>

A robot without moveit/robot.urdf gets no move_group (the launch is then empty).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from taskweave_bringup.args import flag
from taskweave_bringup.moveit import moveit_nodes


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    return moveit_nodes(value('config_dir'), value('robot'), flag(value('use_sim_time')))


def arguments():
    return [
        DeclareLaunchArgument('config_dir', description='taskweave config directory (the cell pack)'),
        DeclareLaunchArgument('robot', description='robot name (its pack and moveit/ folder)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]


def generate_launch_description():
    return LaunchDescription([*arguments(), OpaqueFunction(function=nodes)])
