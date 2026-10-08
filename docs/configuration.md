# Configuration

An application hands taskweave one config directory. Every file is YAML and is validated against a JSON schema shipped
with `taskweave_core` when it is loaded; errors name the file, the key and what was expected.

```
config/
  cell/
    layout.yaml          fixtures and zones of the work cell (application-specific content)
    locations.yaml       named poses in the cell frame
    objects.yaml         placed object instances: object type and pose in the cell frame
  objects/
    <object>.yaml        object frame, mesh reference, taught tool poses (grasp, place, hang ...)
  robots/
    <model>/
      profile.yaml       how the hardware is reached: roles -> topics, joints, frames, controllers
      embodiment.yaml    what the robot is and can do: capabilities, tools, reach, cameras, action spaces
      poses.yaml         named joint-space poses (home, carry); robot-specific by nature
      stances.yaml       where this robot stands relative to an object (per object type, per instance)
  skills/
    <skill>.yaml         implementations of one skill (first match is served) and its parameters
  missions/
    <mission>.yaml       goal, robot, loop mode, defaults, steps
```

Frames: poses are `{frame, xyz: [x, y, z], rpy: [r, p, y]}` (metres, radians) or `{frame, x, y, yaw}` for planar base
poses. `frame` is `cell` or a named object or fixture frame.

## Mission

```yaml
goal: Move frames from the bays to the Karakuri rack
robot: a2d                    # optional: only for robot-specific missions; omitted = any robot
loop: forever                 # once | forever
max_cycles: 72                # optional, with loop: forever
stop_when: bays_empty         # optional world-state flag that ends the mission
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
  - instruction: Pick the top frame with both hands
    skill: pick_frame
    args: {object: frame_type2, at: "{fullest_stack}", grasp: side_rails}
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
are validated at that point instead of before the mission starts.

Point references: `stance:<instance>[/<variant>]` (where this robot stands at a placed object), `location:<name>`
or a bare name (cell location),
`<object>.<point>` (object point). A skill declares which arguments are points in its argument schema with
`"x-taskweave-point": "base_target"` or `"object_point"`, so missing points are reported before anything moves.

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
arms:
  left:  {group: left_arm,  tool_frame: left_tool}
  right: {group: right_arm, tool_frame: right_tool}
  both:  {group: both_arms_lift}
grippers:
  left:  {action: /left_gripper/command,  open: 0.0, closed: 1.0}
  right: {action: /right_gripper/command, open: 0.0, closed: 1.0}
cameras:
  head: {color: /head_camera/color/image_raw, depth: /head_camera/aligned_depth_to_color/image_raw}
teleop:
  deadman_required: true
```

Skills ask for roles (`base`, `arms.both`, `grippers.left`, `cameras.head`); only this file knows the hardware names.

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
# skills/pick_frame.yaml
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
    approach: {axis: z, distance: 0.10}
```

Tool poses are canonical-TCP poses relative to the object, so they hold for any robot that can reach them.
`<role>_tool` names the robot tool (`left_tool` is `tools.left`). Validation checks tool type and grip width against
the robot's tools.

## Skill parameters

```yaml
# skills/navigate_to.yaml
max_linear: 0.6
max_angular: 1.0
position_tolerance: 0.03
yaw_tolerance: 0.05
stuck_timeout_s: 5.0
```
