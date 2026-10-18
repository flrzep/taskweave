"""The ROS fixtures give back their file descriptors: a module with dozens of them must not hit 'Too many open files'
(limit 1024). Runs the real ``system`` fixture of test_joint_motion (skill server, simulator, client, executor) 60 times
in one process, and checks the descriptors, threads and a fixture that fails half way through its setup."""

import gc
import os
import threading

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('tf2_ros')
pytest.importorskip('moveit_msgs')

from conftest import RosRig  # noqa: E402
from test_joint_motion import system  # noqa: E402, F401  (the fixture is used by name)

CYCLES = 60
WARM = 3                # the first cycles open the DDS participant of the session (about 23 descriptors), once
MARGIN = 6              # what the last cycle may still hold above the warm level
seen = []


def open_fds() -> int:
    gc.collect()
    return len(os.listdir('/proc/self/fd'))


@pytest.mark.skipif(not os.path.isdir('/proc/self/fd'), reason='counts descriptors through /proc')
@pytest.mark.parametrize('cycle', range(CYCLES))
def test_the_system_fixture_runs_again_and_again(system, cycle):  # noqa: F811
    sim, client = system
    assert client.get_name() == 'test_client' and sim.position
    seen.append((open_fds(), threading.active_count()))


@pytest.mark.skipif(not os.path.isdir('/proc/self/fd'), reason='counts descriptors through /proc')
def test_descriptors_and_threads_are_back_near_the_start():
    assert len(seen) == CYCLES, 'the cycles did not all run'
    warm_fds, warm_threads = seen[WARM]
    fds, threads = seen[-1]
    print(f'fds: first {seen[0][0]}, after {WARM + 1} cycles {warm_fds}, after {CYCLES} cycles {fds} (limit 1024)')
    assert fds <= warm_fds + MARGIN, f'{fds} descriptors after {CYCLES} fixtures, {warm_fds} after {WARM + 1}'
    assert threads <= warm_threads + 1, f'{threads} threads after {CYCLES} fixtures, {warm_threads} after {WARM + 1}'
    assert open_fds() <= warm_fds + MARGIN


def test_a_fixture_that_fails_in_its_setup_still_tears_down():
    before = threading.active_count()
    with pytest.raises(RuntimeError, match='setup failed'):
        with RosRig() as ros:
            ros.add(rclpy.create_node('fd_leak_failing_setup'))
            ros.spin(4)
            raise RuntimeError('setup failed')
    assert not ros.nodes and ros.executor is None
    assert threading.active_count() <= before
