import shutil
from pathlib import Path

import pytest

FIXTURE_CONFIG = Path(__file__).parent / 'fixtures' / 'config'


@pytest.fixture
def config_dir(tmp_path) -> Path:
    """A writable copy of the generic test config."""
    target = tmp_path / 'config'
    shutil.copytree(FIXTURE_CONFIG, target)
    return target


def wait_until(condition, timeout=30.0, interval=0.02, what='condition'):
    """Poll ``condition()`` (while a background executor spins the nodes) until it is truthy, and return its value.
    The deadline is generous on purpose: it only matters when something is really wrong, a loaded machine (several
    suites in parallel) just takes longer. Raises AssertionError naming ``what`` on timeout."""
    import time

    deadline = time.monotonic() + timeout
    while True:
        value = condition()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise AssertionError(f'timed out after {timeout:g} s waiting for {what}')
        time.sleep(interval)


def warm_up(handler):
    """Make the handler's own action clients talk to the live skill servers before the first mission. The handler
    makes its client at the first dispatch and refuses the step ('skill server ... is not available') if the server is
    not matched yet; and a server of the previous test (same names, gone but not yet forgotten by discovery) can make
    a client look ready too early. So prove it with a real goal: the 'wait' skill for 0 s, sent through the handler's
    client, retried until accepted."""
    import json

    from rclpy.action import ActionClient

    from taskweave_interfaces.action import RunSkill

    catalog = wait_until(handler.refresh_catalog, what='the skill catalog')
    for info in catalog.values():
        if info.action_name not in handler.dispatcher.clients:
            handler.dispatcher.clients[info.action_name] = ActionClient(
                handler, RunSkill, info.action_name, callback_group=handler.group)
        client = handler.dispatcher.clients[info.action_name]
        wait_until(client.server_is_ready, what=f'the action server of {info.name}')
        if info.name != 'wait':
            continue
        import time
        done = False
        while not done:
            future = client.send_goal_async(RunSkill.Goal(step_id='warm', skill='wait',
                                                          args_json=json.dumps({'seconds': 0})))
            until = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < until:
                time.sleep(0.02)
            done = future.done() and future.result().accepted
        wait_until(client.server_is_ready, what=f'the action server of {info.name} to stay ready')


class RosRig:
    """The nodes of one ROS test, one executor and the thread spinning it, torn down in the right order whatever
    happens (use as ``with RosRig() as rig:`` around the yield of a fixture, or around a test body).

    ``rig.add(node)`` registers a node the moment it exists, so a fixture that fails half way through its setup still
    destroys what it made. ``rig.spin(threads)`` starts the executor. On exit: shut the executor down, join its
    thread (nothing may still run a callback while nodes are destroyed), destroy the nodes in reverse order, drop
    the per-node singletons the production classes keep in class-level dicts (they hold the node, so the node would
    stay alive), and collect garbage so the DDS endpoints of the test are closed before the next test starts.
    A fixture without this leaked every executor, spin thread and node when its setup or test failed, and left it to
    the garbage collector otherwise."""

    SINGLETONS = (('taskweave_core.speed', 'SpeedOverride'),)

    def __init__(self):
        self.nodes = []
        self.executor = None
        self.thread = None

    def add(self, node):
        self.nodes.append(node)
        if self.executor is not None:
            self.executor.add_node(node)
        return node

    def spin(self, threads=8):
        import threading

        from rclpy.executors import MultiThreadedExecutor

        self.executor = MultiThreadedExecutor(num_threads=threads)
        for node in self.nodes:
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        return self.executor

    def close(self):
        import gc
        import importlib

        problems = []
        if self.executor is not None:
            self.executor.shutdown()
        if self.thread is not None:
            self.thread.join(10.0)
            if self.thread.is_alive():
                problems.append('the executor thread did not stop within 10 s')
        for node in reversed(self.nodes):
            try:
                node.destroy_node()
            except Exception as error:  # keep destroying the rest
                problems.append(f'destroy_node({node.get_name()}): {error}')
            for module, name in self.SINGLETONS:
                try:
                    getattr(importlib.import_module(module), name)._instances.pop(id(node), None)
                except (ImportError, AttributeError):
                    pass
        self.nodes, self.executor, self.thread = [], None, None
        gc.collect()
        assert not problems, '; '.join(problems)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


@pytest.fixture(scope='session', autouse=True)
def one_ros_context():
    """One rclpy context for the whole session. The ROS tests call rclpy.init()/shutdown() around every test; creating
    and destroying a DDS participant per test makes discovery slow and now and then lost on a loaded machine (an action
    server that stays invisible to a node of the same process), so init happens once and the per-test calls do
    nothing."""
    try:
        import rclpy
    except ImportError:
        yield
        return
    real_init, real_shutdown = rclpy.init, rclpy.shutdown
    real_init()
    rclpy.init = lambda *args, **kwargs: None
    rclpy.shutdown = lambda *args, **kwargs: None
    try:
        yield
    finally:
        rclpy.init, rclpy.shutdown = real_init, real_shutdown
        real_shutdown()
