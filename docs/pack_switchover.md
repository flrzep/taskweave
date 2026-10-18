# Switching the frame-sim host to cell pack + robot packs

Before: one config directory, `/data/frame-sim/config` (= `/workspace/frame-sim/config` in the containers), with whole
robot folders in `robots/<robot>/`. After: that directory is the cell pack (`pack.yaml`, schema 2) and keeps only the
robots' cell-specific files (`stances.yaml`, `held_poses.yaml`, `object_points.yaml`); profile, embodiment, moveit/,
navigation, sim, poses and pose_targets come from the installed `taskweave_robots` package
(`/workspace/frame-sim/ros/ws/install/taskweave_robots/share/taskweave_robots/<robot>/`). The files are the same
bytes, so the robot behaves the same; `taskweave config migrate` only removes a cell file when the robot pack has an
identical copy and keeps any file that differs as a cell override.

Every command below runs on the GPU host (`ssh root@100.97.65.111`) unless it says laptop.

## Before you start

1. **taskweave_skills/joints.py reads the URDF through the robot file search path.** On `main`, `MoveToPose.setup`
   builds the URDF path from the cell pack (`config.root / URDF.format(...)`). After the migration that file is gone
   from the cell pack, the URDF chains are silently not attached and straight-line arm moves change. Before deploying,
   `main` must have:

   ```diff
   -            from taskweave_core.config.limits import URDF
   -            urdf = config.root / URDF.format(robot=robot.name)
   +            from taskweave_core.config.limits import urdf_path
   +            urdf = urdf_path(config, robot.name)
   ```

   Check: `grep -rn "root / URDF" taskweave_*/taskweave_*` in the taskweave repo finds nothing.
2. **frame_sim scripts resolve robot files the same way.** Copy the changed frame_sim files from the laptop mirror to
   the host (they also work with the old layout, because the cell pack is searched first, so this can happen any
   time before the restarts). From the laptop, in `adient-uc1-frame/frame_sim`:

   ```bash
   for f in scripts/cellconfig.py ros/moveit.launch.py ros/nav2.launch.py ros/depth_cloud.py ros/frame_ros.launch.py \
            tools/solve_poses.py tools/model_to_urdf.py tools/export_collision.py tools/collision_matrix.py; do
     ssh root@100.97.65.111 "cp /data/frame-sim/$f /data/frame-sim/$f.pre-packs"   # for the rollback
     scp "$f" root@100.97.65.111:/data/frame-sim/$f
   done
   ssh root@100.97.65.111 'chown 1234:1234 /data/frame-sim/{scripts,ros,tools}/*.py && chmod g+rw /data/frame-sim/{scripts,ros,tools}/*.py'
   ```

   Diff the host copies against the mirror first if the host might have newer edits (the host copy is the truth).
3. **Nothing is running.** No mission in the HMI (state idle), no smoke run. Restarting frame-sim needs the user's OK.
4. **The host config repo is clean.** `git -C /data/frame-sim/config status` shows nothing, or commit/stash what is
   there first (on 16 Oct `tests/frame_cycle.reference.yaml` was modified), so the migration is one commit of its own.

## Switch-over

1. **Deploy** taskweave (laptop): `frame_sim/tools/deploy_taskweave.sh`. It builds every package, including
   `taskweave_robots`. The running processes keep the old code until the restart and still read the old layout.
   Check the robot packs are installed:

   ```bash
   docker exec frame-ros ls /workspace/frame-sim/ros/ws/install/taskweave_robots/share/taskweave_robots
   ```

2. **Dry run** of the migration against the host config:

   ```bash
   docker exec -u robot frame-ros bash -c 'source /opt/ros/jazzy/setup.bash && \
     source /workspace/frame-sim/ros/ws/install/setup.bash && \
     python3 -m taskweave_core.cli config migrate /workspace/frame-sim/config --name adient_uc1_frame --dry-run'
   ```

   Expected: `would remove robots/<robot>/<file> (same as .../taskweave_robots/<robot>/<file>)` for every
   robot-intrinsic file of galbot_s1, galbot_g1 and a2d, then `would write pack.yaml (schema_version 2)`. A line
   `keep robots/<robot>/<file> as a cell override: differs from ...` means the host file changed after the robot packs
   were copied (16 Oct, e.g. a pose saved in the HMI). Either put that file into `taskweave_robots` and deploy again
   (if it is a robot change), or accept the cell override (behaviour is unchanged either way).
3. **Migrate and restart frame-ros, back to back** (the running old code reads robot files from the cell pack on every
   access, so it breaks as soon as they are removed):

   ```bash
   docker exec -u robot frame-ros bash -c 'source /opt/ros/jazzy/setup.bash && \
     source /workspace/frame-sim/ros/ws/install/setup.bash && \
     python3 -m taskweave_core.cli config migrate /workspace/frame-sim/config --name adient_uc1_frame'
   cd /data/frame-sim/config && git add -A && \
     git commit -m "Cell pack: robot files come from taskweave_robots (taskweave config migrate)"
   docker restart frame-ros
   ```

   Run the migration as user robot (uid 1234) so new files keep the group-writable ownership.
   A unit (a file, or a folder such as `moveit/`) is removed completely or not at all: the migration first checks
   that every entry in it can be removed. If not (a root-owned `meshes/` subfolder, say) it prints `cannot remove
   <path>: <reason> (unit ... left as it is; ...)`, leaves that unit alone, finishes the other units, does not write
   pack.yaml and exits 1. Fix the permissions (or remove the unit as its owner) and run it again. A unit that an
   interrupted run left half removed (some of the pack's files left, all identical) is recognised as "same as the pack,
   partly removed" and removed on the rerun, not kept as a cell override. `config slim` checks the same before it
   removes a whole file.
4. **Check** in frame-ros:

   ```bash
   docker exec frame-ros bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/frame-sim/ros/ws/install/setup.bash && \
     python3 -m taskweave_core.cli config files /workspace/frame-sim/config galbot_s1 && \
     python3 -m taskweave_core.cli check /workspace/frame-sim/config'
   ```

   Every robot-intrinsic file shows `robot pack`, the cell-specific files `cell`; check ends with `config ok`. The HMI
   (http://100.97.65.111:8090) loads, the Robot tab shows the poses; move_group logs that it loaded the URDF
   (`docker logs frame-ros 2>&1 | grep -i move_group | tail`), Nav2 is up.
5. **Restart frame-sim** (only with the user's OK): `docker restart frame-sim`. Isaac reads the robot's sim.yaml and
   profile through `scripts/cellconfig.py` (no ROS there: the install share on the shared mount). Its log line
   `[frame-sim] built ... (robot galbot_s1, ...` must appear as before.
6. **Smoke test** in frame-ros:

   ```bash
   docker exec frame-ros bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/frame-sim/ros/ws/install/setup.bash && \
     cd /tmp && python3 -m taskweave_core.cli smoke run frame_cycle --hmi http://100.97.65.111:8090 \
     --reference /workspace/frame-sim/config/tests/frame_cycle.reference.yaml'
   ```

   It must pass like before the switch.

## After the switch

- HMI and teach edits of robot pack files (poses, navigation, the profile's ik.choose) cannot go to the installed
  share (every build overwrites it), so they create a cell override in `config/robots/<robot>/` that holds only the keys
  the edit changed (`~override: keys`, laid over the pack's file; pack updates to other keys still reach the cell;
  `~delete` removes a pack key); `taskweave config files` shows it as `override`. Whole-file overrides kept by the
  migration keep working; `taskweave config slim config/ --dry-run` shows how they would shrink to the differing keys,
  without `--dry-run` it does it (idempotent). A pack key an old copy lacks (a profile copy from before `arms`,
  `motion_tuning`, `grippers.*.closed_from`) is not overridden after the slim, so the pack's value applies (and later
  pack updates of it reach the cell); only an explicit `~delete` removes a pack key. Run the slim right after the
  migration: until then the whole-file copy replaces the pack's file and lacks those keys (no elbow control; `taskweave
  check` warns). Commit it in the host repo as usual. To make it
  part of the robot pack, merge its keys into `taskweave_robots/<robot>/` in the taskweave repo, deploy, and delete the
  override.
- Changing a robot's MoveIt config, sim.yaml or poses for good: edit `taskweave_robots/<robot>/` in the taskweave repo
  and deploy (frame-ros restart; frame-sim restart for sim.yaml).
- `system.yaml: robot_packs: [path]` or env `TASKWEAVE_ROBOT_PACKS` add writable robot pack locations searched
  before the installed share (edits then go there), e.g. a checkout of someone else's robot pack.
- The comment in `config/system.yaml` ("Each robot lives in config/robots/<robot>/") is outdated after the switch.

## Rollback

Config and taskweave go back together (the new taskweave refuses a schema-1 config, the old one cannot find robot
files outside the cell):

1. `cd /data/frame-sim/config && git revert --no-edit <migration commit>` (the robot files are back, pack.yaml gone).
2. Deploy the previous taskweave commit (laptop): `git -C taskweave worktree add ../tw-rollback <previous commit>` and
   `frame_sim/tools/deploy_taskweave.sh ../tw-rollback`. The old build ignores the left-over
   `install/taskweave_robots` (the cell pack has every file again).
3. `docker restart frame-ros` (and frame-sim, with the user's OK). The new frame_sim scripts work with the old layout,
   so they can stay; to restore them anyway: `for f in /data/frame-sim/{scripts,ros,tools}/*.pre-packs; do mv "$f"
   "${f%.pre-packs}"; done`.
