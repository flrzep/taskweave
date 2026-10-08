# Configuration

An application hands taskweave one config directory. Every file is YAML and is validated against a JSON schema shipped
with `taskweave_core` when it is loaded; errors name the file, the key and what was expected.

```
config/
  cell/
    layout.yaml          fixtures and zones of the work cell (application-specific content)
    locations.yaml       named poses in the cell frame
  objects/
    <object>.yaml        object frame, mesh reference, taught tool poses (grasp, place, hang ...)
  robots/
    <model>/
      profile.yaml       roles -> hardware (joints, frames, controllers, topics, limits)
      poses.yaml         named joint-space poses (home, carry); robot-specific by nature
      stances.yaml       robot-specific base poses (e.g. where to stand in front of a fixture)
  skills/
    <skill>.yaml         parameters of one skill (speeds, tolerances, approach distances)
  missions/
    <mission>.yaml       goal, robot, loop mode, defaults, steps
```

Frames: poses are `{frame, xyz: [x, y, z], rpy: [r, p, y]}` (metres, radians) or `{frame, x, y, yaw}` for planar base
poses. `frame` is `cell` or a named object or fixture frame.

## Mission

```yaml
goal: Move frames from the bays to the Karakuri rack
robot: a2d                    # config/robots/a2d/
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
| `on_failure` | When retries are used up: `retry` (start again with fresh retries), `replan` (ask the planner), `skip` (log and continue), `abort` (stop the mission) |
| `replan_from` | Step to continue from when the planner replans; default is the failed step |
| `max_replans` | Replans one step may trigger per cycle; after that the mission aborts |

`{name}` placeholders are resolved from the world state when the step starts; arguments that contain placeholders
are validated at that point instead of before the mission starts.

Point references: `stance:<name>` (robot-specific stance), `location:<name>` or a bare name (cell location),
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

## Locations

```yaml
waiting: {frame: cell, x: 2.0, y: 0.0, yaw: 0.0}
dropoff: {frame: cell, x: 0.75, y: 0.0, yaw: 3.1416}
```

Written by hand or by the teach tool (`taskweave teach location <name>`), which stores the current base pose.

## Objects

```yaml
name: frame_type2
mesh: assets/frame_type2.usd
points:
  side_rails:                 # taught with the teach tool
    left_tool:  {frame: frame_type2, xyz: [-0.18,  0.235, 0.065], rpy: [0, 1.5708, 0]}
    right_tool: {frame: frame_type2, xyz: [-0.18, -0.235, 0.065], rpy: [0, 1.5708, 0]}
    approach: {axis: z, distance: 0.10}
```

Tool poses are relative to the object, so they hold for any robot that can reach them.

## Skill parameters

```yaml
# skills/navigate_to.yaml
max_linear: 0.6
max_angular: 1.0
position_tolerance: 0.03
yaw_tolerance: 0.05
stuck_timeout_s: 5.0
```
