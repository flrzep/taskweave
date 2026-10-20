"""Optional clock guard: recovers Nav2 and MoveIt after the simulation clock jumps back (Stop/Play, simulator restart).

Nav2 keeps TF history in sim time. After a jump back it treats every new transform as TF_OLD_DATA, plans from a stale
robot pose and does not follow its path. Three cases leave it dead until restarted:

- the clock jumped back;
- stale TF: after one late transform the controller's TF buffer stops at that sim time while TF itself runs on, and
  every goal aborts with a "failed to transform" error (one such log line is enough);
- a failed bringup: the costmap gave up waiting for TF before the sim published it, the lifecycle manager aborts and
  the navigator stays inactive while the clock runs.

In each case the guard resets the Nav2 lifecycle nodes and starts them again (fresh TF buffers and costmaps) once the
clock runs steadily, at most every ``cooldown_s``. A navigation goal that was running is aborted; the taskweave step
fails and follows its mission rules. move_group has no lifecycle and keeps the base where it was at the reset, so
every IK target in the base frame lands elsewhere: after a jump the guard ends its process once the clock is steady
and the launch file respawns it (the moveit launch of ``taskweave_bringup`` respawns it). An IK or planning call in
flight fails.

``taskweave_bringup`` starts it with ``clock_guard:=true``. Node and service names are parameters (defaults: the
nodes ``taskweave_bringup`` itself launches); ``nav2`` / ``move_group`` switch each part. The decisions are pure
functions below; the node only feeds them with the clock, logs and service answers.
"""

from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class Timing:
    jump_s: float = 0.5        # sim time going back by more than this is a reset
    settle_s: float = 2.0      # wall seconds of steady clock before anything is restarted
    cooldown_s: float = 20.0   # wall seconds between Nav2 restarts (a restart itself takes a few seconds)
    inactive_s: float = 15.0   # wall seconds the navigator may stay inactive while the clock runs
    steady_s: float = 1.0      # the clock counts as running when a message came this recently (wall seconds)


def is_jump(last: float | None, now: float, jump_s: float) -> bool:
    """Sim time ``now`` is more than ``jump_s`` before the last message's ``last``."""
    return last is not None and now < last - jump_s


def _steady(clock_at: float | None, now: float, timing: Timing) -> bool:
    return clock_at is not None and now - clock_at < timing.steady_s


def nav2_reset_due(pending_at: float | None, now: float, clock_at: float | None, restarted_at: float,
                   timing: Timing) -> bool:
    """A Nav2 restart requested at wall time ``pending_at`` may run now: settled, clock steady, cooled down."""
    return (pending_at is not None and now - pending_at >= timing.settle_s and _steady(clock_at, now, timing)
            and now - restarted_at >= timing.cooldown_s)


def move_group_restart_due(pending_at: float | None, now: float, clock_at: float | None, timing: Timing) -> bool:
    return pending_at is not None and now - pending_at >= timing.settle_s and _steady(clock_at, now, timing)


def inactive_update(since: float | None, active: bool, clock_runs: bool, now: float,
                    timing: Timing) -> tuple[float | None, bool]:
    """(new ``since``, restart?) for a navigator that is not active while the sim runs (failed bringup)."""
    if active or not clock_runs:
        return None, False
    since = now if since is None else since
    if now - since > timing.inactive_s:
        return None, True
    return since, False


def stale_tf_log(node: str, text: str, watch_node: str, watch_text: str) -> bool:
    """A log line of ``watch_node`` containing ``watch_text``: the controller lost TF. Empty ``watch_text``: off."""
    return bool(watch_text) and node == watch_node and watch_text in text


# ---- the node ---------------------------------------------------------------------------------------------------
def main(args=None):
    import rclpy
    import rclpy.executors
    from lifecycle_msgs.msg import State
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from rcl_interfaces.msg import Log
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=args)
    node = rclpy.create_node('clock_guard')
    defaults = {'nav2': True, 'move_group': True, 'lifecycle_manager': '/lifecycle_manager_navigation/manage_nodes',
                'navigator_state': '/bt_navigator/get_state', 'move_group_match': '__node:=move_group',
                'stale_tf_node': 'controller_server', 'stale_tf_text': 'Failed to transform end pose to global frame'}
    defaults.update({f.name: f.default for f in fields(Timing)})
    for key, value in defaults.items():
        node.declare_parameter(key, value)
    p = {key: node.get_parameter(key).value for key in defaults}
    timing = Timing(**{f.name: p[f.name] for f in fields(Timing)})
    log = node.get_logger()
    client = node.create_client(ManageLifecycleNodes, p['lifecycle_manager'])
    state_client = node.create_client(GetState, p['navigator_state'])
    s = {'last': None, 'clock_at': None, 'pending': None, 'restarted_at': -timing.cooldown_s, 'busy': False,
         'inactive_since': None, 'move_group_pending': None}

    def request(reason: str) -> None:
        if s['pending'] is None and not s['busy']:
            log.warning(f'{reason}: restarting Nav2')
            s['pending'] = time.monotonic()

    def on_clock(msg) -> None:
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        if is_jump(s['last'], now, timing.jump_s):
            s['pending'] = None  # restart from the jump, after the new time settles
            if p['nav2']:
                request(f'sim time went back {s["last"]:.1f} -> {now:.1f} s')
            s['move_group_pending'] = time.monotonic()
        s['last'], s['clock_at'] = now, time.monotonic()

    def on_log(msg) -> None:
        if p['nav2'] and stale_tf_log(msg.name, msg.msg, p['stale_tf_node'], p['stale_tf_text']):
            request(f'Nav2 controller lost TF ({p["stale_tf_node"]})')

    def send(command: int) -> None:
        future = client.call_async(ManageLifecycleNodes.Request(command=command))
        future.add_done_callback(lambda done: finished(command, done))

    def finished(command: int, future) -> None:
        if future.result() is None or not future.result().success:
            s['busy'] = False
            log.error(f'Nav2 lifecycle command {command} failed; will retry')
            request('previous Nav2 restart failed')
        elif command == ManageLifecycleNodes.Request.RESET:
            send(ManageLifecycleNodes.Request.STARTUP)
        else:
            s['busy'] = False
            log.info('Nav2 restarted with fresh TF buffers')

    def on_navigator_state(future) -> None:
        result, now = future.result(), time.monotonic()
        active = result is not None and result.current_state.id == State.PRIMARY_STATE_ACTIVE
        runs = s['clock_at'] is not None and now - s['clock_at'] < 2 * timing.steady_s
        s['inactive_since'], restart = inactive_update(s['inactive_since'], active, runs, now, timing)
        if restart:
            request(f'navigator is {result.current_state.label} while the sim runs (bringup failed?)')

    def poll_navigator() -> None:
        if p['nav2'] and not (s['busy'] or s['pending']) and state_client.service_is_ready():
            state_client.call_async(GetState.Request()).add_done_callback(on_navigator_state)

    def check() -> None:
        now = time.monotonic()
        if p['move_group'] and move_group_restart_due(s['move_group_pending'], now, s['clock_at'], timing):
            s['move_group_pending'] = None
            ended = subprocess.run(['pkill', '-f', p['move_group_match']], check=False).returncode == 0
            log.warning('sim time went back: restarting move_group' if ended
                        else 'sim time went back, but no move_group process to restart')
        if s['busy'] or not nav2_reset_due(s['pending'], now, s['clock_at'], s['restarted_at'], timing):
            return
        s['pending'] = None
        if not client.wait_for_service(timeout_sec=1.0):
            log.error(f'{p["lifecycle_manager"]} not available; restart the stack to recover Nav2')
            return
        s['busy'], s['restarted_at'] = True, now
        send(ManageLifecycleNodes.Request.RESET)

    node.create_subscription(Clock, '/clock', on_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(Log, '/rosout', on_log, 100)
    node.create_timer(0.5, check)
    node.create_timer(2.0, poll_navigator)
    log.info(f'watching /clock (nav2: {p["nav2"]}, move_group: {p["move_group"]})')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


def clock_guard_nodes(nav2: bool, moveit: bool, params_file: str = '') -> list:
    """The guard node; ``params_file`` (YAML, ROS parameter format) overrides node names and timing."""
    from launch_ros.actions import Node

    parameters: list = [{'nav2': nav2, 'move_group': moveit, 'use_sim_time': False}]  # wall time: it watches /clock
    if params_file:
        parameters.append(params_file)
    return [Node(package='taskweave_bringup', executable='clock_guard', name='clock_guard', output='screen',
                 respawn=True, parameters=parameters)]
