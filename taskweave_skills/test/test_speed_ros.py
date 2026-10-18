"""The global speed override end to end: the handler's latched topic slows move_joints on a skill server."""

import json
import time

import pytest
from conftest import wait_until

rclpy = pytest.importorskip('rclpy')

from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from test_joints_ros import run, system  # noqa: E402, F401

from taskweave_core import names  # noqa: E402
from taskweave_interfaces.msg import SpeedOverride  # noqa: E402


def timed(client, pose, timeout_s=20.0):
    start = time.monotonic()
    result = run(client, 'move_joints', {'pose': pose}, timeout_s=timeout_s)
    return result, time.monotonic() - start


def test_override_slows_moves_and_their_timeouts(system):  # noqa: F811
    _, client = system
    publisher = client.create_publisher(SpeedOverride, names.SPEED_OVERRIDE,
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    run(client, 'move_joints', {'pose': 'home'})       # warm up: action discovery is not motion time
    result, full = timed(client, 'ready')
    assert result.outcome == 0, result.message
    run(client, 'move_joints', {'pose': 'home'})
    publisher.publish(SpeedOverride(percent=50.0, cap_percent=50.0, test_percent=10.0))
    wait_until(lambda: client.taskweave_server.speed.fraction() == 0.5, what='the server to take the 50 % override')
    result, half = timed(client, 'ready')
    assert result.outcome == 0, result.message
    assert 1.5 < half / full < 3.0  # wall time of two runs: loose bounds for a loaded machine
    speed = json.loads(result.data_json)['speed']     # move_joints adds the effective time scale of the move
    assert (speed['override_min'], speed['override_avg']) == (50.0, 50.0) and speed['scale'] == pytest.approx(0.5)
    run(client, 'move_joints', {'pose': 'home'})
    publisher.publish(SpeedOverride(percent=25.0, cap_percent=50.0, test_percent=10.0))
    wait_until(lambda: client.taskweave_server.speed.fraction() == 0.25, what='the server to take the 25 % override')
    result, slow = timed(client, 'ready', timeout_s=1.5)   # about 3.2 s of node time: 0.8 work s, no timeout
    assert result.outcome == 0, result.message
    assert slow > 2.5
