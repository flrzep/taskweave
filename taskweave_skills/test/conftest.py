import time


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
