# Configuration

An application hands taskweave one config directory, the cell pack. Every file is YAML and is validated against a JSON
schema shipped with `taskweave_core` when it is loaded; errors name the file, the key and what was expected.

```
config/                  cell pack: everything about one cell
  pack.yaml              name, version, schema_version (3); without it: run `taskweave config migrate config/` (schema 2: `taskweave config
                         migrate-skills config/`, the old skills move_to_pose, go_to_point, gripper and the old
                         move_object become move_joints, move_gripper, grip, move_object and set_posture)
  system.yaml            optional robot_packs: [paths relative to the cell pack]
  cell/
    layout.yaml          fixtures and zones of the work cell (application-specific content)
    locations.yaml       named poses in the cell frame
    objects.yaml         placed object instances: object type and pose in the cell frame
  objects/
    <object>.yaml        object frame, mesh reference, taught tool poses (grasp, place, hang ...)
  robots/
    <model>/             the robot's cell-specific files, and per-key overrides of its robot pack's files
      stances.yaml       where this robot stands relative to an object (per object type, per instance)
      held_poses.yaml    where this robot holds an object of a type (move_object pose)
      object_points.yaml this robot's settings per object point (elbow side, grip opening)
  skills/
    <skill>.yaml         implementations of one skill (first match is served) and its parameters
  missions/
    <mission>.yaml       goal, robot, loop mode, defaults, steps
```

## Create, export and import a cell pack

```bash
taskweave config init CELL_DIR --robot PACK [--name NAME]   # new empty pack; CELL_DIR absent or empty; then taskweave check passes
taskweave config export CELL_DIR cell.tar.gz                # one file
taskweave config import cell.tar.gz CELL_DIR [--force]      # check, then replace
```

`init` writes `pack.yaml` (current schema), `system.yaml` (`robot`), empty `cell/locations.yaml` and
`cell/objects.yaml`, the robot's empty `stances.yaml`, `held_poses.yaml`, `object_points.yaml`, and the folders
`objects/`, `missions/`, `skills/`. The robot must be a robot pack taskweave finds (`TASKWEAVE_ROBOT_PACKS`, installed).

`export` packs the pack's files (no caches, `.git`, `state/`; if the folder is in a git repo, only files git does not
ignore; symlinks are skipped and listed) with a `manifest.yaml`: `name`, `schema_version`, `git_rev` (null without a
repo), `taskweave_version`, `date`, `files` (`path`, `sha256`, `size`).

`import` extracts into a temp folder next to the target, refuses unsafe paths (`..`, absolute), links and special
files, checks every sha256 and the file list against the manifest, refuses another schema version with a message, and
runs the same checks as `taskweave check`. Only then does it replace the target; a bad archive changes nothing.
- A plain folder is swapped: the old one is renamed `<dir>.bak-<yyyymmdd-hhmmss>`, the new one takes its place with the
  old folder's group and mode (setgid dirs, group-write files). A non-empty target needs `--force`.
- A folder inside a git work tree (the live host: `/data/frame-sim`) is not swapped, as that would break the repo:
  the files are copied in place (owners kept), files the archive lacks are removed, and exactly those paths are
  committed as `import <name> <rev>`; git history is the backup. Uncommitted changes in the pack stop the import,
  or with `--force` are committed first as `taskweave: snapshot before import`. Nothing is pushed.

The same functions are in `taskweave_core.config.packfile` (`init_cell`, `export_cell`, `import_cell`; errors are
`PackfileError`).

A robot pack holds one robot model, nothing about a cell (taskweave ships `taskweave_robots`, one folder per robot,
installed to `share/taskweave_robots/<model>/`):

```
<model>/
  pack.yaml              name (= the model), version, schema_version (1), description, optional requires: {taskweave: ">=0.1,<0.2"} (install.md, "Pack versions")
  profile.yaml           how the hardware is reached: roles -> topics, joints, frames, controllers
  embodiment.yaml        what the robot is and can do: capabilities, tools, reach, cameras, action spaces
  poses.yaml             named poses (home, carry); robot-specific by nature
  navigation.yaml        room kept around the robot while driving
  moveit/                URDF, SRDF, kinematics, joint limits, collision meshes
  sim.yaml, pose_targets.yaml   simulator and pose-solver inputs of the application
  skill_overrides.yaml   optional: this robot's own motion options and standard speeds per skill (over skills/<skill>.yaml)
```

`robots/<model>/<file>` is read from the cell pack if it has the file, else from the first robot pack that has it:
env `TASKWEAVE_ROBOT_PACKS` (paths separated by `os.pathsep`), `system.yaml` `robot_packs`, the installed
`taskweave_robots`. Each location is a robot pack or a folder of robot packs. Edits from the HMI and teaching go to
the cell pack for cell-specific files and files the cell pack already has, else to the robot pack the file came
from; the installed share is read-only (a rebuild overwrites it), so an edit of one of its files creates a cell
override. `taskweave config files config/` lists each robot file's source.

A cell override of a robot pack file (`profile`, `embodiment`, `navigation`, `sim`, `poses`, `pose_targets`) holds only
what the cell changes. It starts with `~override: keys` and is laid over the pack's file when loaded (the merged result
is what is validated), so later pack updates to other keys still reach the cell:

- mappings merge key by key; lists and scalars replace; in `poses.yaml` a pose is replaced whole
- `key: ~delete` removes that key of the pack's file (the only way to delete a pack key; only explicit edits write it)
- a file without `~override: keys` is a whole-file override (made before per-key overrides) and still replaces the pack's
  file; the next HMI/teach save turns it into a keyed one, `taskweave config slim config/ [--dry-run]` does it for all.
  A pack key such a copy lacks is read as never had (the copy predates it), not deleted: after the slim the pack's value
  applies (e.g. `arms` of an old profile copy). Until then the copy replaces the pack's file, so it lacks them too;
  `taskweave check` warns about an IK arm without an elbow description
- a configured robot pack location (env or `system.yaml`) that does not exist is an error, and so is a keyed override
  whose robot pack file no pack has; `config migrate` and `slim` leave entries that resolve (symlinks) into a robot pack
  location or outside the cell pack alone

Frames: poses are `{frame, xyz: [x, y, z], rpy: [r, p, y]}` (metres, radians) or `{frame, x, y, yaw}` for planar base
poses. `frame` is `cell` or a named object or fixture frame.

## Mission

```yaml
goal: Move parts from their stacks into the bin
# robot: <model>              # optional: only for robot-specific missions; omitted = any robot
loop: forever                 # once | forever
max_cycles: 72                # optional, with loop: forever
stop_when: bin_full           # optional, with loop: forever: world-state key that ends the mission when truthy
defaults:
  timeout_s: 120
  retries: 1                  # extra attempts after a failure
  on_failure: abort           # what to do when retries are used up: retry | replan | skip | abort
  max_replans: 1              # how often one step may trigger a replan per cycle
steps:
  - id: drive_to_stack        # optional; s1, s2, ... if missing
    instruction: Drive to the fullest stack
    skill: navigate_to
    args: {target: "stance:{fullest_stack}"}
  - instruction: Both hands onto the part
    skill: move_gripper
    args: {to: {point: "{fullest_stack}/grasp"}}
    retries: 2
    on_failure: replan
    replan_from: drive_to_stack   # on replan, continue from this step with fresh retries
```

Decision rules per step (falling back to `defaults`):

| Key | Meaning |
|---|---|
| `timeout_s` | The step fails with `TIMEOUT` after this time (sim time when `use_sim_time` is set) |
| `retries` | Extra attempts of the same step after `FAILED` or `TIMEOUT` |
| `on_failure` | When retries are used up: `retry` (start the step again with fresh retries), `replan` (ask the planner), `skip` (log and continue), `abort` (end the mission as failed). `retry` and `replan` count toward `max_replans` |
| `replan_from` | Step to continue from when the planner replans; default is the failed step |
| `max_replans` | Replans one step may trigger per cycle; after that the mission aborts |

`{name}` placeholders are resolved from the world state when the step starts; arguments that contain placeholders
are validated at that point instead of before the mission starts. [missions.md](missions.md) is the guide to writing
missions, [skills.md](skills.md) the reference of the skills and their arguments.

A step can include another mission instead of calling a skill: `{instruction, include: <mission>, with: {param:
value}}`. The included mission declares its parameters (`params: {frame: {point: instance, description: ...}}`; the
point kind lets the mission editor offer names) and uses them as `{param}` in its steps; placeholders that are not
parameters stay world-state placeholders. When the plan is made, the step is replaced by the included steps (ids
`<step>_<inner step>`), with the include step's `timeout_s`, `retries`, `on_failure` and `max_replans` if set, else
the included mission's defaults. Includes can nest (5 deep), not loop.

`facts` are worked out when each cycle is planned and filled into its steps as `{name}`, so they stay fixed for the
cycle while it changes the objects: `{fullest: <object type>, among: <glob of instance names>}` is the placed object
with the most items left (run-time object state; ties go to the first name). When a fact finds nothing, the mission is
complete, which ends a `loop: forever` mission once every matching stack is empty.

Point references: `stance:<instance>[/<variant>]` (where this robot stands at a placed object), `location:<name>`
or a bare name (cell location),
`<object>.<point>` (a point of an object type) or `<instance>/<point>` (that point on one placed object). A skill
declares which arguments are points in its argument schema with `"x-taskweave-point"`: `base_target`, `object_point`
(either form), `instance_point` (only `<instance>/<point>`, e.g. move_gripper `to.point`), `instance_contact`
(`<instance>/<contact>`, e.g. move_object `to.onto`), `joint_pose` / `joint_pose_only` / `tool_pose` (poses of
`poses.yaml`), `joint_group`, `held_pose`, `held_object` and `via_target`, so missing points are reported before
anything moves (`taskweave_core.validation.check_point`).

Contacts (`contacts:` in `objects/<object>.yaml`, `{xyz, rpy}` relative to the object) are frames where one object
meets another: a hook plate on a frame, the hook it hangs on. Their z axis is the direction the held object moves to
engage (down onto a hook). `move_object {to: <instance>/<contact>, contact: <held object's contact>, offset_m}` puts the
held object's contact onto the placed one's, `offset_m` short of it against that direction.

## Simulator data

`sim.yaml` and `pose_targets.yaml` are inputs of the simulator integration and of the pose solver, not of taskweave:
taskweave loads them as YAML and checks nothing else. Some of their values are paths into a simulation workspace:
`asset_file` in `galbot_s1/sim.yaml` and `model` in each `pose_targets.yaml` (the comments also cite the `reference/*.json`
geometry files). These are resolved relative to the **simulator workspace root**, which the simulator integration provides
(for example through an environment variable such as `TASKWEAVE_SIM_ROOT`), never relative to the robot pack. A robot pack
therefore does not change when the simulation workspace moves; only the simulator's root does. Other simulator keys follow
the simulator's own conventions (for example `galbot_g1/sim.yaml` `asset`, relative to the scene's assets root).

taskweave itself does not read `asset_file` or `model`: no code in this repository does, so the packs validate without any
simulation workspace.

## Robot profile

```yaml
name: a2d
base:
  type: holonomic             # holonomic | differential | fixed
  cmd_vel: /cmd_vel
  odom: /odom
  frame: base_link
  footprint: [0.60, 0.45]     # length, width in metres
  limits: {linear: 1.0, angular: 1.5}
arms:                         # per IK tool, the keys elbow and swivel (nothing else is read, see below)
  left:  {elbow: {shoulder: left_link2, elbow: left_link4, wrist: left_link6}, swivel: left_joint3}
  right: {elbow: {shoulder: right_link2, elbow: right_link4, wrist: right_link6}, swivel: right_joint3}
grippers:
  left:  {action: /left_gripper/command,  open: 0.0, closed: 1.0}
  right: {action: /right_gripper/command, open: 0.0, closed: 1.0}
cameras:
  head: {color: /head_camera/color/image_raw, depth: /head_camera/aligned_depth_to_color/image_raw}
teleop:
  deadman_required: true
motion_tuning:                # optional, every key defaults (schemas.MOTION_TUNING_DEFAULTS)
  elbow: {rate_rad: 0.03}
```

Skills ask for roles (`base`, `joints`, `grippers.left`, `tool_frames.left`, `ik`); only this file knows the hardware names.

`arms.<tool>.elbow` names the links at an IK arm's shoulder, elbow and wrist centres: the elbow side (`ik.choose.elbow`,
a point's `elbow`), straight lines and elbow swings use it. An arm without it has no elbow control (an elbow side is
ignored with a warning, an elbow swing fails with a message). Only `elbow` and `swivel` are read under `arms.<tool>`;
the schema does not reject other keys (an old profile may carry `group` or `tool_frame`), they are ignored, and
`taskweave check` warns about each so that a typo does not stay silent. `arms.<tool>.swivel` names the joint the pad's elbow
jog turns (e.g. the upper-arm roll): without a URDF the jog turns it and IK puts the TCP back (an arm without it has no
elbow jog then); with a URDF it only sets which way is positive. `motion_tuning` holds the step sizes and thresholds of
the arm motion code, grouped as `chain` (damped least-squares step), `line` (straight-line steps), `elbow` (elbow side
and swing), `continuation` (short moves continue the arm's configuration) and `plan` (planner retries, contact depth);
the schema documents each key and its default. They are read when the skills start (restart frame-ros after a change).

## Robot description (embodiment)

`robots/<model>/embodiment.yaml` says what the robot *is* and *can do*. It holds no topics or joint names (the schema
rejects unknown keys), so a simulated and a real robot of the same model share it while their profiles differ.
Unknown values are written as `~` and reported as unknown; they are measured, not guessed.

```yaml
name: a2d
summary: >-
  Wheeled bimanual humanoid: holonomic base, lift torso, two 7-DoF arms with parallel grippers,
  RGB-D head camera and one RGB-D camera per wrist.
capabilities: [base.holonomic, torso.lift, arms.bimanual, gripper.parallel, camera.head.rgbd, camera.wrist.rgbd]
base:  {footprint: [0.60, 0.45], max_speed: {linear: 1.0, angular: 1.5}}
torso: {lift_range_m: ~}
arms:
  left:  {dof: 7, reach_m: ~, payload_kg: ~, tool: left}
  right: {dof: 7, reach_m: ~, payload_kg: ~, tool: right}
tools:
  left:  {type: parallel_gripper, stroke_m: ~, finger_length_m: ~}
  right: {type: parallel_gripper, stroke_m: ~, finger_length_m: ~}
cameras:
  head:       {type: rgbd, hfov_deg: 69, range_m: [0.1, 10]}
  left_wrist: {type: rgbd, hfov_deg: 87, range_m: [0.07, 1.0], mounted_on: tools.left}
action_spaces:
  joint_pos_bimanual: {dims: 17, rate_hz: 30}
```

**Capability vocabulary** (owned by taskweave, `taskweave_core.embodiment.CAPABILITIES`): `base.holonomic`,
`base.differential`, `base.fixed`, `torso.lift`, `torso.pitch`, `head.pan_tilt`, `arms.single`, `arms.bimanual`,
`gripper.parallel`, `gripper.suction`, `hand.dexterous`, `camera.head.rgb`, `camera.head.rgbd`, `camera.wrist.rgb`,
`camera.wrist.rgbd`, `lidar.2d`, `lidar.3d`. Tool types: `parallel_gripper`, `suction`, `dexterous_hand`.

**Canonical TCP:** every tool's tool-centre point is the grasp centre between the fingertips (or the suction-cup
centre), with **z = approach direction** and **y = closing direction**. The profile maps each robot's flange to this
frame; taught tool poses use it, so they transfer between robots.

`taskweave check` reports contradictions between embodiment and profile (for example `base.holonomic` with a
differential base, or a tool without a gripper mapping) and implementation classes that cannot be loaded.

## Skill implementations

```yaml
# skills/pick_frame.yaml: a skill of the application (not one of taskweave_skills)
implementations:                    # the skill server serves the first one that fits the robot
  - name: pick_frame_vla_v1
    type: vla                       # script | vla | rl | other
    class: frame_policies.vla:PickFrame
    requires:
      capabilities: [arms.bimanual, camera.wrist.rgbd]
      action_space: joint_pos_bimanual
      cameras: [left_wrist, right_wrist]
      embodiments: [a2d]            # trained on A2D data
    params: {checkpoint: /models/pick_frame_v1}
  - name: pick_frame_scripted
    type: script
    class: frame_skills.pick:PickFrame
    requires: {capabilities: [arms.bimanual, gripper.parallel]}
approach_distance: 0.10             # shared by all implementations; `params` override per implementation
```

The skill server is started with skill *names*. It serves the first implementation whose `requires` fit the
robot's embodiment and logs why the others don't. A mission that uses a skill without a fitting implementation is
rejected with those reasons before anything moves.

## Locations

```yaml
waiting: {frame: cell, x: 2.0, y: 0.0, yaw: 0.0}
dropoff: {frame: cell, x: 0.75, y: 0.0, yaw: 3.1416}
```

Written by hand or by the teach tool (`taskweave teach location <name>`), which stores the current base pose.

## Object instances and stances

Where the robot stands to work on an object is robot-specific; where the object is, is not. So the cell lists the
placed objects, and each robot stores its stances *relative to the object*:

```yaml
# cell/objects.yaml (robot-independent; from config now, from perception later)
bay_2_stack_2: {object: frame_type2, frame: cell, x: 9.425, y: 1.10, z: 0.10, yaw: 1.5708}

# robots/<model>/stances.yaml (base pose in the object's frame: x, y, yaw)
objects:                      # per object type: every instance of the type uses these
  frame_type2:
    pick:  {x: -0.85, y: 0.0, yaw: 0.0}     # first variant = default
    place: {x: -0.70, y: 0.0, yaw: 0.0}
instances:                    # optional overrides for single instances
  bay_3_stack_3:
    pick:  {x: -0.85, y: 0.10, yaw: 0.0}
```

`stance:bay_2_stack_2` is the object's pose composed with the robot's default offset (`pick` above);
`stance:bay_2_stack_2/place` names a variant. Moving an object moves its stances; a second robot only needs its own
offsets. `taskweave teach stance bay_2_stack_2[/variant]` stores the current base pose as the offset for the object
type (all instances follow); add `--instance` to store it for that instance only. Tool poses on objects (below) use the
same object frames, so base, approach and grasp all follow one object pose.

## Objects

```yaml
name: frame_type2
mesh: assets/frame_type2.usd
points:
  side_rails:                 # taught with the teach tool
    tool_type: parallel_gripper
    grip_width_m: 0.03        # checked against the tool's stroke
    left_tool:  {frame: frame_type2, xyz: [-0.18,  0.235, 0.065], rpy: [0, 1.5708, 0]}
    right_tool: {frame: frame_type2, xyz: [-0.18, -0.235, 0.065], rpy: [0, 1.5708, 0]}
    approach_m: 0.10          # the last 10 cm of the way in are a straight line (else the skill's motion.approach_m)
contacts:                     # frames where this object meets another (move_object to.onto)
  base: {xyz: [0, 0, 0], rpy: [0, 0, 0]}
```

An object type may carry a `sim:` object (free-form, e.g. `sim: {box: [0.06, 0.06, 0.1], on_hold: pull_release}`): it is for the
simulator integration (what to draw, what a hold does), taskweave skills never read it.

`collision:` makes a held object a collision object for MoveIt (without it the free arm can plan through what the other
hand holds). One shape or a list, in metres, in the object frame:
`collision: {box: [0.06, 0.06, 0.10], xyz: [0, 0, 0.05]}` or `{cylinder: {radius: 0.03, height: 0.2}}`, each with an
optional `xyz` and `rpy` (radians) that place the shape. **A shape's origin is its centre** (a cylinder's axis is z), so a
box whose object frame is at its bottom centre needs `xyz: [0, 0, height / 2]`. When a grip closes on the object,
`grip` attaches the shapes to the first holding hand's tool frame link through the `/apply_planning_scene` service
(posed as the object is relative to that link at that moment); when the last hand opens, they are detached and removed
(also on a simulator reset). The links allowed to touch it are the holding hands' profile `grippers.<hand>.links` (a list;
default the hand's tool frame link). Profile `planning.scene_service` (default `/apply_planning_scene`) and
`planning.scene_timeout_s` (default 2.0) set the service and how long a call may take. If the service is missing or
refuses, `grip` logs a warning and carries on. An object type without `collision` sends nothing.

Tool poses are canonical-TCP poses relative to the object, so they hold for any robot that can reach them.
`<role>_tool` names the robot tool (`left_tool` is `tools.left`). Validation checks tool type and grip width against
the robot's tools.

To teach a point, jog the tools onto a placed object with the pad. Then press the `joy.teach` button in
`operator.yaml`, or run `taskweave teach point bay_2_stack_1/side_rails [--tool left]`. That stores each tool's
current TCP (profile `tool_frames`, read from TF in the cell frame) relative to the instance's top item, for the
object type. It keeps the point's other tools and metadata.

## Skill parameters

```yaml
# skills/navigate_to.yaml
max_linear: 0.6
max_angular: 1.0
position_tolerance: 0.03
yaw_tolerance: 0.05
stuck_timeout_s: 5.0
```

The arm skills (`move_joints`, `move_gripper`, `move_object`, `set_posture`, `grip`) use one layout: standard speeds in
`speed`, option defaults in `motion`, accuracy and timeouts in `tolerances` (shipped files: `docs/examples/skills/`;
older flat keys are renamed by `taskweave config migrate-skills`). `move_joints` moves the torso, head and arm
joints to a named joint pose or to given values. Joint groups for its `only` option come from the profile's `joints.groups`.

```yaml
# skills/move_joints.yaml
implementations:
  - {name: joint_targets, type: script, class: "taskweave_skills.joint_motion:MoveJoints"}
speed: {joint_rad_s: 1.2, joint_linear_m_s: 0.25}   # standard speeds, scaled by a step's motion.speed
motion: {planner: auto}
tolerances: {position: 0.02, settle_s: 0.3, stall_timeout_s: 3.0, state_timeout_s: 1.0}
```

```yaml
# robots/<robot>/profile.yaml
joints: {command: /joint_command, state: /joint_states, linear: [body_joint],
         groups: {torso: [body_joint], head: [head_joint1, head_joint2], left_arm: [left_joint1, ...]},
         limits: {body_joint: [0.0, 0.75]}}
```

`joints.limits` is optional: joint -> `[lower, upper]` (rad, m for a prismatic joint). It is laid over the limits of the
robot's URDF (`moveit/robot.urdf` of the pack), so it is the only source for a robot without a URDF and the place to narrow
a joint. `move_joints` checks values against it before anything moves, IK and the straight-line code keep clear of it
(`ik.choose.limit_margin_deg`), and plan validation checks named poses against it. A robot with limits from neither
source is not checked. `joints.groups` names the
groups for `move_joints only`; `<tool>_arm` groups it does not list come from the `ik.joints` regex and the URDF.

```yaml
# mission steps
- {id: lift, skill: move_joints, args: {pose: lift_high}}                        # a named joint pose
- {id: look, skill: move_joints, args: {joints: {head_joint1: 0.3, head_joint2: -0.2}}}
- {id: raise, skill: move_joints, args: {pose: home, only: [torso]}}             # the torso part of the pose only
- {id: tuck, skill: move_joints, args: {pose: tucked, motion: {speed: 0.5, via: [home]}}}   # half speed, via home
```
Joint values are checked against the robot's limits (URDF, profile `joints.limits`) before anything moves. While an
object is held, moves of arm joints are refused; torso and head moves are fine.
