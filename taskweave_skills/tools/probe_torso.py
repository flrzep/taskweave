"""Read-only probe of the torso height search (IkClient.solve_with_torso) against a live MoveIt. No motion: it calls
only /compute_ik, /compute_fk and /check_state_validity with MODEL joint names (profile joints.groups; never base or
wheel joints) and the robot's current state from its joint state topic.

For a grasp point on an object whose origin is at (``--object`` x y z) in the stance frame (the robot at the origin,
x forward), with the tool pose ``--tool-xyz`` / ``--tool-rpy`` in the object frame, it prints for each hand alone:
every candidate height with its verdict (valid or the reason), the chosen height, the idle arm there and what the
old first-success list (torso_fallback) would take. Run on the ROS domain of the live system, from a sourced
workspace (the robot packs of the checkout in TASKWEAVE_ROBOT_PACKS):

    python3 probe_torso.py --config /workspace/frame-sim/config --robot <name> --object 0.8 0 0.6 \
        --tool-xyz 0 0 0.07 --tool-rpy 0 3.1416 -1.5708 [--approach 0.1] [--first-success 0.0 0.2 ...]
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
from taskweave_skills.ik import IkClient, IkError, rpy_matrix
from taskweave_skills.points import target_of


def model_state(node, names: set, topic: str, wait_s: float = 5.0) -> dict:
    """The current value of the joints in ``names`` (the model's own joints) from the joint state topic."""
    box = {}
    node.create_subscription(JointState, topic, lambda m: box.update(zip(m.name, m.position, strict=False)), 10)
    deadline = time.monotonic() + wait_s
    while not box and time.monotonic() < deadline:
        time.sleep(0.05)
    return {n: float(v) for n, v in box.items() if n in names}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--config', required=True)
    parser.add_argument('--robot', required=True)
    parser.add_argument('--object', type=float, nargs=3, required=True, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--tool-xyz', type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument('--tool-rpy', type=float, nargs=3, default=[0.0, 0.0, 0.0])
    parser.add_argument('--approach', type=float, default=0.1, help='pre-grasp distance back along the approach (m)')
    parser.add_argument('--first-success', type=float, nargs='*', default=[], help="the old list's heights")
    args = parser.parse_args()

    store = ConfigStore(args.config)
    robot = store.robot(args.robot)
    rclpy.init()
    node = rclpy.create_node('probe_torso')
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    ik = IkClient(node, robot, limits=joint_limits(store, args.robot))
    ik.attach_chains(urdf_path(store, args.robot), (robot.data.get('planning') or {}).get('group'))
    spec = ik.torso
    if not spec:
        raise SystemExit('the robot profile has no ik.torso')
    for client in (ik.client, ik.validity):
        client.wait_for_service(timeout_sec=10.0)
    names = {n for group in robot.data['joints']['groups'].values() for n in group}
    current = model_state(node, names, robot.data['joints']['state'])
    joint = spec['joint']
    if joint not in current:
        raise SystemExit(f'no state for {joint}: {sorted(current)}')
    now = current[joint]
    print(f'{joint} now {now:.3f}; {len(current)} model joints read; validity group {ik.validity_group!r}')

    box = np.eye(4)
    box[:3, 3] = args.object
    tool = box @ rpy_matrix(args.tool_xyz, args.tool_rpy)
    point, before = target_of(tool), target_of(tool, args.approach)
    print(f'TCP {point["tcp"]} approach {point["approach"]} closing {point["closing"]}')
    weight = float((spec.get('weights') or {}).get('torso', 1.0))
    for hand in robot.data['tool_frames']:
        idle = [t for t in ik.groups if t != hand]
        print(f'\n== {hand} hand alone (idle: {", ".join(idle)})')
        samples = [{hand: point}, {hand: before}]
        feasible = []
        for height in sorted({float(v) for v in spec['candidates']} | {now}):
            try:
                choice = ik._torso_height(samples, height, {}, current, joint, now, weight, lambda: False, None, True)
            except IkError as error:
                print(f'  {height:5.3f}  rejected: {error}')
                continue
            feasible.append(height)
            print(f'  {height:5.3f}  valid  cost {choice.cost:.3f}')
        try:
            best = ik.solve_with_torso(samples, {}, current)
        except IkError as error:
            print(f'  search: {error}')
            continue
        again = ik.solve_with_torso(samples, {}, current).height
        print(f'  repeat: {again:.3f} ({"same" if again == best.height else "DIFFERENT"})')
        arm = best.solution
        print(f'  search chose {best.height:.3f} (cost {best.cost:.3f}); valid heights {feasible}')
        print('  arm joints ' + ' '.join(f'{n}={v:.2f}' for n, v in sorted(arm.items())))
        for other in idle:
            state = {**current, joint: best.height}
            hand_at = ik.tcps_at([other], state).get(other)
            posture = {n: round(v, 2) for n, v in current.items() if n.startswith(other)}
            ok = ik.valid({joint: best.height, **ik.idle_joints([hand], current), **arm}, ik.groups[hand])
            where = '' if hand_at is None else f'hand at {np.round(hand_at[:3, 3], 3).tolist()}'
            print(f'  idle {other}: {where}, joints {posture}, full-state check valid={ok}')
        if args.first_success:
            for height in [None, *args.first_success]:
                fixed = {} if height is None else {joint: height}
                try:
                    ik.solve({hand: point}, fixed, current=current)
                except IkError:
                    continue
                at = now if height is None else height
                solved = ik.solve({hand: point}, fixed, current=current)
                ok = ik.valid({joint: at, **ik.idle_joints([hand], current), **solved}, ik.groups[hand])
                print(f'  old list takes {at:.3f}; full-state check valid={ok}')
                break
            else:
                print('  old list: nothing solves')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
