"""Nav2 for the robot of a cell: map server, planner, controller (MPPI), behaviors, BT navigator, lifecycle manager.

ros2 launch taskweave_bringup taskweave_nav2.launch.py config_dir:=/path/to/cell robot:=<robot> map:=/path/map.yaml

The robot's footprint, driving limits, base frame and topics come from its pack (navigation.yaml, profile base); the
tuning is the packaged nav2_params.yaml unless robots/<robot>/nav2_params.yaml (cell pack or robot pack) or
``nav2_params:=<file>`` replaces it. The parameters file written for the nodes is
``<nav2_output_dir>/nav2_params.yaml``.
The map and the transform that attaches the map frame (a simulator's world, or localisation such as AMCL) come from
the application.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from taskweave_bringup.args import flag
from taskweave_bringup.nav2 import nav2_nodes


def nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    return nav2_nodes(value('config_dir'), value('robot'), value('map'), value('nav2_output_dir'),
                      flag(value('use_sim_time')), value('nav2_params'), value('map_topic'))


def arguments():
    return [
        DeclareLaunchArgument('config_dir', description='taskweave config directory (the cell pack)'),
        DeclareLaunchArgument('robot', description='robot name (its pack and profile)'),
        DeclareLaunchArgument('map', default_value='', description='path of the Nav2 map yaml (required)'),
        DeclareLaunchArgument('map_topic', default_value='/map', description='topic the map server publishes'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('nav2_params', default_value='',
                              description='Nav2 tuning file replacing the packaged default (else '
                                          'robots/<robot>/nav2_params.yaml if present)'),
        DeclareLaunchArgument('nav2_output_dir', default_value='',
                              description='where the generated nav2_params.yaml is written (default: a temp folder)'),
    ]


def generate_launch_description():
    return LaunchDescription([*arguments(), OpaqueFunction(function=nodes)])
