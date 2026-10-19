"""The web HMI node of a bring-up (no login: bind it to a trusted network, see README)."""

from __future__ import annotations


def hmi_parameters(config_dir: str, robot: str, use_sim_time: bool, host: str, port: int,
                   recordings_dir: str = '', recording_post: str = '') -> dict:
    """hmi_server parameters; the recorder's are set only when given (recording is off without a folder)."""
    parameters = {'config_dir': config_dir, 'robot': robot, 'use_sim_time': use_sim_time, 'host': host, 'port': port}
    if recordings_dir:
        parameters['recordings_dir'] = recordings_dir
    if recording_post:
        parameters['recording_post'] = recording_post
    return parameters


def hmi_nodes(config_dir: str, robot: str, use_sim_time: bool, host: str, port: int, recordings_dir: str = '',
              recording_post: str = '') -> list:
    from launch_ros.actions import Node

    return [Node(package='taskweave_hmi', executable='hmi_server', name='hmi_server', output='screen', respawn=True,
                 parameters=[hmi_parameters(config_dir, robot, use_sim_time, host, port, recordings_dir,
                                            recording_post)])]
