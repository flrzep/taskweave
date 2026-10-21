"""Read-only probe: which torso heights make a whole held-object line feasible (the path the move walks on the URDF
chains, not just the end point). No motion; only MoveIt's /compute_ik, /compute_fk, /check_state_validity with model
joint names, the robot's current joint state as the IK seed.

The held object starts at the held pose ``--from-pose`` (its last waypoint, base frame) with the arms solved for it at
``--start-height`` (the state after that move), then the torso goes to each candidate height with the arms keeping
their joints (what move_object does before a line) and the object's contact ``--contact`` is walked onto
``--onto`` (``<instance>/<contact>``) ``--offset`` back, from the stance ``--stance`` (a base target). Prints, per
height, whether the line exists and why not.

    python3 probe_line.py --config <cell config> --robot <name> --object <type> --grasp <point> \
        --from-pose <held pose> --onto <instance>/<contact> --contact <held contact> --offset 0.1 \
        --stance stance:<instance>/<variant> --base-z <m> --start-height 0.2
"""

import argparse
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from taskweave_core.config import ConfigStore
from taskweave_core.config.limits import joint_limits, urdf_path
from taskweave_skills.cartesian import PathError, linear_path, tcp_targets
from taskweave_skills.ik import IkClient, IkError, rpy_matrix
from taskweave_skills.points import onto_pose, target_of


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for name in ('config', 'robot', 'object', 'grasp', 'from-pose', 'onto', 'contact', 'stance'):
        parser.add_argument(f'--{name}', required=True)
    parser.add_argument('--offset', type=float, default=0.0)
    parser.add_argument('--then-offset', type=float, default=None,
                        help='after the line to --offset, a second line onto the contact at this offset (0 = on it)')
    parser.add_argument('--base-z', type=float, default=0.0, help='base frame origin above the cell floor (m)')
    parser.add_argument('--start-height', type=float, required=True)
    parser.add_argument('--tcp-speed', type=float, default=0.05)
    parser.add_argument('--tcp-turn', type=float, default=0.3)
    args = parser.parse_args()

    store = ConfigStore(args.config)
    robot = store.robot(args.robot)
    rclpy.init()
    node = rclpy.create_node('probe_line')
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    ik = IkClient(node, robot, limits=joint_limits(store, args.robot))
    ik.attach_chains(urdf_path(store, args.robot), (robot.data.get('planning') or {}).get('group'))
    for client in (ik.client, ik.validity):
        client.wait_for_service(timeout_sec=10.0)
    joint = ik.torso['joint']
    names = {n for group in robot.data['joints']['groups'].values() for n in group}
    seen = {}
    node.create_subscription(JointState, robot.data['joints']['state'],
                             lambda m: seen.update(zip(m.name, m.position, strict=False)), 10)
    deadline = time.monotonic() + 10.0
    while not any(n in seen for n in names) and time.monotonic() < deadline:
        time.sleep(0.1)
    time.sleep(0.5)
    current = {n: float(v) for n, v in seen.items() if n in names}

    stance = store.base_target(args.stance, args.robot)
    base_in_cell = rpy_matrix([stance.x, stance.y, args.base_z], [0.0, 0.0, stance.yaw])
    target, contact = store.instance_contact(args.onto)
    mine = store.object_contact(args.object, args.contact)
    goal = onto_pose(target, contact, mine, base_in_cell, args.offset)
    held = store.held_pose(args.robot, args.object, args.from_pose)
    way = held['path'][-1]
    start = rpy_matrix(way.xyz, way.rpy)
    grasp = store.object(args.object)['points'][args.grasp]
    grips = {key[:-5]: rpy_matrix(v['xyz'], v['rpy']) for key, v in grasp.items() if key.endswith('_tool')}
    tools_start = {h: target_of(start @ g) for h, g in grips.items()}
    goals = tcp_targets({h: target_of(goal @ g) for h, g in grips.items()})
    print(f'object start {np.round(start[:3, 3], 3).tolist()} -> goal {np.round(goal[:3, 3], 3).tolist()} in base; '
          f'torso now {current[joint]:.3f}, start state solved at {args.start_height}')
    try:
        arm = ik.solve(tools_start, {joint: args.start_height}, current=current, allow_flip=False)
    except IkError as error:
        raise SystemExit(f'the start pose has no IK at {args.start_height}: {error}') from error
    state0 = {**current, joint: args.start_height, **arm}
    armset = {n: state0[n] for n in arm}
    usable = []
    then = None
    if args.then_offset is not None:
        final = onto_pose(target, contact, mine, base_in_cell, args.then_offset)
        then = tcp_targets({h: target_of(final @ g) for h, g in grips.items()})
    for height in sorted({float(v) for v in ik.torso['candidates']} | {args.start_height}):
        state = {**state0, joint: height}
        starts = ik.tcps_at(goals, state)
        began = time.monotonic()
        try:
            path = linear_path(ik, starts, goals, armset, args.tcp_speed, args.tcp_turn, state=state)
            verdict = f'line ok ({len(path.points)} steps)'
            if args.then_offset is not None:       # the next move: from the end of this line onto the contact
                end = dict(zip(path.names, path.points[-1], strict=True))
                state2 = {**state, **end}
                second = ik.tcps_at(then, state2)
                try:
                    path2 = linear_path(ik, second, then, {n: state2[n] for n in end}, args.tcp_speed, args.tcp_turn,
                                        state=state2)
                    verdict += f'; then line ok ({len(path2.points)} steps)'
                    usable.append(height)
                except PathError as error:
                    verdict += f'; then NO line: {str(error)[:80]}'
            else:
                usable.append(height)
        except PathError as error:
            verdict = f'no line: {str(error)[:110]}'
        print(f'  {height:5.3f}  {verdict}  ({time.monotonic() - began:.1f} s)')
    near = min(usable, key=lambda h: (abs(h - args.start_height), h), default=None)
    print(f'heights usable for the whole move: {usable}; least torso travel from {args.start_height}: {near}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
