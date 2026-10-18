import time

import pytest


def wait_until(condition, timeout=30.0, interval=0.02, what='condition'):
    """Poll ``condition()`` (while a background executor spins the nodes) until it is truthy, and return its value.
    The deadline is generous on purpose: it only matters when something is really wrong, a loaded machine (several
    suites in parallel) just takes longer. Raises AssertionError naming ``what`` on timeout."""
    deadline = time.monotonic() + timeout
    while True:
        value = condition()
        if value:
            return value
        if time.monotonic() >= deadline:
            raise AssertionError(f'timed out after {timeout:g} s waiting for {what}')
        time.sleep(interval)


def wait_ready(server, *clients):
    """A skill server is ready for its first goal when its joint command publisher is matched, joint states have
    arrived and the given service clients (``ik.client``, ``planner.validity``, ...) see their services. Without this
    a loaded machine starts the move before discovery is done: commands are lost, states are missing, a service looks
    absent and the skill answers differently (e.g. plans instead of moving straight)."""
    from taskweave_skills.joints import JointInterface

    joints = JointInterface._instances.get(id(server))
    if joints is not None:
        wait_until(lambda: joints.publisher.get_subscription_count() > 0, what='the simulator to take joint commands')
        wait_until(lambda: joints.snapshot(1.0), what='joint states at the skill server')
    for client in clients:
        wait_until(client.service_is_ready, what=f'service {client.srv_name}')


def connect(node, action_type, name, timeout=90.0, attempt_s=5.0):
    """An ActionClient whose server is visible. A client that has not found the server after ``attempt_s`` is replaced by
    a fresh one (new endpoints are announced again), which gets over a discovery that was lost under load; the deadline
    is generous."""
    from rclpy.action import ActionClient

    deadline = time.monotonic() + timeout
    while True:
        client = ActionClient(node, action_type, name)
        if client.wait_for_server(timeout_sec=attempt_s):
            return client
        client.destroy()
        assert time.monotonic() < deadline, f'action server {name} not found within {timeout:g} s'


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

    SINGLETONS = (('taskweave_core.speed', 'SpeedOverride'), ('taskweave_skills.joints', 'JointInterface'),
                  ('taskweave_skills.hold', 'HoldState'))

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
    server that stays invisible to a node of the same process), so init happens once and the per-test calls do nothing."""
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
