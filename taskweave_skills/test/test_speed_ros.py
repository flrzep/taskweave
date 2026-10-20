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


def commanded_speed(client, pose, joint='j1', timeout_s=20.0):
    """Run a move and return (result, rate): how fast the skill's own command stream advanced ``joint`` through the
    middle of its path, in path fractions per second. The stream is stamped where the skill publishes it and fitted
    over many samples, so neither a late action goal, discovery, the settle phase nor a delayed message on a loaded
    runner enters (the wall time of the whole run did, and made the ratio of two runs meaningless under load)."""
    import numpy as np

    from taskweave_skills.joints import JointInterface

    joints = JointInterface._instances[id(client.taskweave_server)]
    samples, publish = [], joints.command

    def record(targets):
        if joint in targets:
            samples.append((time.monotonic(), targets[joint]))
        publish(targets)

    joints.command = record
    try:
        result = run(client, 'move_joints', {'pose': pose}, timeout_s=timeout_s)
    finally:
        del joints.command
    values = [v for _, v in samples]
    low, high = min(values), max(values)
    # the commands that are strictly inside the path: before the start and after the end they sit on the ends. The
    # slope needs only a few of them (a slow runner publishes few); more samples just make the fit steadier.
    middle = [(t, (v - low) / (high - low)) for t, v in samples if 0.02 < (v - low) / (high - low) < 0.98]
    assert len(middle) >= 3, f'only {len(middle)} commands inside the move'
    times, fractions = zip(*middle, strict=True)
    return result, float(np.polyfit(times, fractions, 1)[0])


def test_override_slows_moves_and_their_timeouts(system):  # noqa: F811
    _, client = system
    publisher = client.create_publisher(SpeedOverride, names.SPEED_OVERRIDE,
                                        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    run(client, 'move_joints', {'pose': 'home'})       # warm up: action discovery is not motion time
    result, full = commanded_speed(client, 'far')
    assert result.outcome == 0, result.message
    run(client, 'move_joints', {'pose': 'home'})
    publisher.publish(SpeedOverride(percent=50.0, cap_percent=50.0, test_percent=10.0))
    wait_until(lambda: client.taskweave_server.speed.fraction() == 0.5, what='the server to take the 50 % override')
    result, half = commanded_speed(client, 'far')
    assert result.outcome == 0, result.message
    assert 1.7 < full / half < 2.4, (full, half)  # the commanded move runs about twice as long at 50 %
    speed = json.loads(result.data_json)['speed']     # move_joints adds the effective time scale of the move
    assert (speed['override_min'], speed['override_avg']) == (50.0, 50.0) and speed['scale'] == pytest.approx(0.5)
    run(client, 'move_joints', {'pose': 'home'})
    publisher.publish(SpeedOverride(percent=10.0, cap_percent=50.0, test_percent=10.0))
    wait_until(lambda: client.taskweave_server.speed.fraction() == 0.1, what='the server to take the 10 % override')
    result, slow = timed(client, 'ready', timeout_s=3.0)   # about 8 s of wall time, 0.8 work s: no timeout, though the
    # budget is 3 work s (3 s of wall time without the override); the margin is wide so that a loaded runner is fine
    assert result.outcome == 0, result.message
    assert slow > 3.0   # it ran past what the timeout allows without the override
