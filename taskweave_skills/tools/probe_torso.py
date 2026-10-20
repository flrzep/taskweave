"""Read-only probe of the torso height search (IkClient.solve_with_torso) against a live MoveIt. No motion: it calls
only /compute_ik, /compute_fk and /check_state_validity with MODEL joint names (profile joints.groups; never base or
wheel joints) and the robot's current state from its joint state topic.

The target is given in the robot's base frame: an object pose (``--object`` x y z yaw) and, per hand, the tool pose in
the object frame (``--tool`` hand x y z roll pitch yaw, as in an object's points). ``--offset`` stops that far back
along the approach; ``--approach`` adds the pre-grasp sample a free move to a point also checks. The probe prints what
the search decided for all given hands at once (height, rejected heights, skipped note) and how long it took
(``--repeat`` times, each from a cold memo). Run on the ROS domain of the live system, from a sourced workspace (the
robot packs of the checkout in TASKWEAVE_ROBOT_PACKS):

    python3 probe_torso.py --config /workspace/frame-sim/config --robot <name> --object 1.0 0 0.38 0 \
        --tool left 0.1 0.25 0.07 -1.9 0 -1.5 --tool right 0.1 -0.25 0.07 -1.9 0 -1.5 --offset 0.15 --elbow up
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
    parser.add_argument('--object', type=float, nargs=4, required=True, metavar=('X', 'Y', 'Z', 'YAW'))
    parser.add_argument('--tool', nargs=7, action='append', required=True,
                        metavar=('HAND', 'X', 'Y', 'Z', 'R', 'P', 'W'))
    parser.add_argument('--offset', type=float, default=0.0, help='stop this far back along the approach (m)')
    parser.add_argument('--approach', type=float, default=0.0, help='also check the pre-grasp this far back (m)')
    parser.add_argument('--elbow', default=None)
    parser.add_argument('--repeat', type=int, default=3)
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
    for client in (ik.client, ik.validity):
        client.wait_for_service(timeout_sec=10.0)
    if not ik.torso:
        raise SystemExit('the robot profile has no ik.torso')
    names = {n for group in robot.data['joints']['groups'].values() for n in group}
    current = model_state(node, names, robot.data['joints']['state'])
    joint = ik.torso['joint']
    print(f'{joint} now {current[joint]:.3f}; {len(current)} model joints read; validity group {ik.validity_group!r}')

    box = np.eye(4)
    c, s = np.cos(args.object[3]), np.sin(args.object[3])
    box[:2, :2] = [[c, -s], [s, c]]
    box[:3, 3] = args.object[:3]

    def targets(back: float) -> dict:
        return {hand: target_of(box @ rpy_matrix([float(v) for v in pose[:3]], [float(v) for v in pose[3:]]), back)
                for hand, *pose in args.tool}

    samples = [targets(args.offset)]
    if args.approach > 0:
        samples.append(targets(args.offset + args.approach))
    for hand, target in samples[0].items():
        print(f'{hand}: TCP {target["tcp"]} approach {target["approach"]}')
    for run in range(args.repeat):
        ik.memo = {}
        started = time.monotonic()
        try:
            choice = ik.solve_with_torso(samples, {}, current, elbow=args.elbow)
            mid = time.monotonic()
            ik.solve(samples[0], choice.fixed, current=current, elbow=args.elbow)   # the move's own solve: a memo hit
            again = time.monotonic() - mid
            report = choice.report()
            rejected = ', '.join(f'{r["height"]:g}: {r["reason"][:40]}' for r in report['rejected'])
            print(f'run {run}: {mid - started:.2f} s  height {report["height"]}  cost {report["cost"]}  '
                  f'(the move\'s solve after it {again:.2f} s)  {report.get("skipped", "")}  rejected [{rejected}]')
        except IkError as error:
            print(f'run {run}: {time.monotonic() - started:.2f} s  FAILED {str(error)[:300]}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
