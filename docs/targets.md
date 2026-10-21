# Targets

A **target** is a place the robot goes to. One format covers all of them: a base pose in the cell, where the base
stands at an object, a hand pose on an object, a carry pose, a joint pose, the frame where one object meets another.
One skill, `move`, goes to any of them ([skills.md](skills.md)), and one reference syntax names them in missions.

Every target says **what it moves** (`moves`) and **in which frame** it is given. The frame is not written: it comes from
the file the target is in.

This page is the reference of the format. Using targets in missions: [missions.md](missions.md). Where the files are:
[configuration.md](configuration.md). Coming from an older cell pack: [migration_targets.md](migration_targets.md).
The examples are the example cell, `docs/examples/cell` (robot `duo_bot`, a neutral test robot).

## The four files

| File | Holds | Frame |
|---|---|---|
| `cell/targets.yaml` | Targets in the cell: base poses such as `home`, `staging`. | the cell |
| `objects/<type>.yaml`, key `targets:` | Targets on an object type that any robot can use: hand points, contacts. | the object |
| `robots/<robot>/objects/<type>.yaml` | What one robot adds on a type: where its base stands (stances), its settings on the type's targets (elbow side, grip opening), and overrides for single instances. | the object |
| `robots/<robot>/targets.yaml` | Targets in the robot's base frame (tool poses, carry poses) and joint targets. The robot pack ships it; the cell overrides keys with `~override: keys`. | the base, or none for joints |

What each kind of target moves, and where the examples are:

| `moves` | Cell frame | Base frame | Object frame |
|---|---|---|---|
| `base` | `staging` | not used yet | `part_stack_1/pick` (a stance) |
| `hand` | format only, `move` rejects it | `present_both` | `part_stack_1/grasp` |
| `object` | format only, `move` rejects it | `carry` | `bin_1/seat` (a contact) |
| `joints` | no frame: `home_pose` | | |

"Format only" means the files accept it and `taskweave check` validates it, but `move` answers `REJECTED` until it is
built.

## The shape of a target

```yaml
<name>:                      # letters, digits, _ and -
  moves: base | hand | object | joints     # required
  # the pose of the kind (below): none for joints
  joints: {torso: 0.3, left_arm: [0.0, 0.0]}   # optional next to a pose; the whole target for moves: joints
```

- **Frame.** An explicit `frame:` is accepted if it is the one the file implies. Nothing else is needed.
- **Names** are unique per place: see [Names](#names-are-unique-per-place).
- **Comments** are kept by every writer: the HMI, `taskweave teach` and `taskweave config migrate`.

### `moves: base`

`x` and `y` in metres, `yaw` in radians, in the frame of the file.

```yaml
# example: cell/targets.yaml
home: {moves: base, x: 0.0, y: 0.0, yaw: 0.0}
staging: {moves: base, x: 1.0, y: 0.5, yaw: 1.5708}
```

A base target on an object type is a **stance**: where the base stands, relative to the object, so it follows the object
when the object moves. A robot has its own stances, so they are in the robot's file for the type:

```yaml
# example: robots/duo_bot/objects/part.yaml
# Where this robot stands relative to an object (base pose in the object's frame).
handled_from: pick
targets:
  pick: {moves: base, x: -0.85, y: 0.0, yaw: 0.0}
instances:                   # one placed object differs: its own pick position
  part_stack_2: {pick: {x: -0.95, y: 0.0, yaw: 0.0}}
```

An object type can have several stances; they are separate targets with their own names (`bin_1/place`, `bin_1/pick`).
There is no "first one is the default": a step always names the stance.

### `moves: hand`

Where the tool centre point (TCP) of each hand goes. The TCP is the grasp centre between the fingertips, z along the
approach and y along the closing direction. A hand that is missing from the target is not moved.

```yaml
# example: objects/part.yaml
name: part
targets:
  grasp:                          # taught with `taskweave teach hand <instance>/grasp`
    moves: hand
    tool_type: parallel_gripper
    grip_width_m: 0.03
    approach_m: 0.1               # the last 10 cm in a straight line
    left_tool: {xyz: [0.0, 0.2, 0.05], rpy: [0.0, 3.14159265, 0.0]}
    right_tool: {xyz: [0.0, -0.2, 0.05], rpy: [0.0, 3.14159265, 0.0]}
    alone:                        # the same point for one hand
      left_tool: {xyz: [0.0, 0.1, 0.05], rpy: [0.0, 3.14159265, 0.0]}
```

| Key | Meaning |
|---|---|
| `<tool>_tool` | Pose of that tool: `{xyz: [m, m, m], rpy: [rad, rad, rad]}`. `left_tool`, `right_tool`: the tool names of the robot profile. |
| `alone` | Replaces the pair's poses when one hand goes alone (`hands: [left]`). |
| `tool_type` | Checked against the robot's tools (`parallel_gripper`). |
| `grip_width_m` | Width of what is gripped, checked against the tool's stroke. |
| `approach_m` | Last part of a free move, straight along the approach. |
| `elbow` | Preferred elbow side: `down`, `up`, `out`, `in`, `back`, `forward`. |
| `grip_m` | How far each gripper stays open when it closes here: `{left: 0.06, right: 0.06}`. |
| `path` | Waypoints instead of the poses above (see [Paths](#paths)). |
| `joints` | Joint values applied first (see [The joints part](#the-joints-part)). |

`tool_type`, `grip_width_m` and `approach_m` describe the object, so they belong in `objects/<type>.yaml`. `elbow` and
`grip_m` are the robot's settings: put them in the robot's object file under the same name, without `moves`:

```yaml
# example: robots/duo_bot/objects/bin.yaml
handled_from: place
targets:
  place: {moves: base, x: -0.70, y: 0.0, yaw: 0.0}
  pick: {moves: base, x: -0.70, y: 0.0, yaw: 0.0}
  grasp: {elbow: out}           # this robot's setting on the bin's grasp point (objects/bin.yaml)
```

In the base frame, a hand target is a named tool pose of the robot (no object involved):

```yaml
# example: robots/duo_bot/targets.yaml
home_pose: {moves: joints, joints: {torso: 0.0, left_arm: [0.0, 0.0], right_arm: [0.0, 0.0]}}
present_left:
  moves: hand
  left_tool: {xyz: [0.6, 0.2, 0.4], rpy: [1.570796, 0.0, 1.570796]}
  elbow: out
present_both:
  moves: hand
  left_tool: {xyz: [0.6, 0.2, 0.4], rpy: [1.570796, 0.0, 1.570796]}
  right_tool: {xyz: [0.6, -0.2, 0.4], rpy: [1.570796, 0.0, 1.570796]}
  joints:
    torso: 0.1
# Where this robot holds an object of a type (`move` to carry).
carry:
  moves: object
  joints:
    torso: 0.1
  path:
  - {xyz: [0.7, 0.0, 0.1], rpy: [0.0, 0.0, 0.0]}
```

### `moves: object`

The pose of the origin of the held object, or a `path` of such poses. Two uses:

- **A carry pose**, in the robot's base frame (`carry` above). Optional `type: <object type>` limits it to held objects of
  that type. With nothing held, or an object of another type, `move` rejects it.
- **A contact**, in the object frame: where one object meets another. A bin's `seat` is where a part sits in it; the
  part's `base` is the part's own underside.

```yaml
# example: objects/bin.yaml
name: bin
targets:
  grasp:                          # a small bin the left hand lifts out by its rim
    moves: hand
    tool_type: parallel_gripper
    grip_width_m: 0.03
    approach_m: 0.1
    left_tool: {xyz: [0.0, 0.2, 0.25], rpy: [0.0, 3.14159265, 0.0]}
  seat: {moves: object, xyz: [0.0, 0.0, 0.2], rpy: [0.0, 0.0, 0.0], reference: base} # where a part sits in the bin
  base: {moves: object, xyz: [0.0, 0.0, 0.0], rpy: [0.0, 0.0, 0.0]} # the bin's own underside: where it stands on something
```

`move {to: bin_1/seat}` puts the **reference** of the held object onto the `seat`: the held object ends where its
reference target coincides with the target frame. `reference: base` names the target of the held object's type that goes
there (the part's `base`). Without `reference` the object's origin goes. A step can override it with the `reference:`
option. The z axis of the target is the direction the object moves to engage; `offset_m` stops that far short of it,
backwards along z.

### `moves: joints`

Joint values by group, no frame:

```yaml
# example: robots/duo_bot/targets.yaml
home_pose: {moves: joints, joints: {torso: 0.0, left_arm: [0.0, 0.0], right_arm: [0.0, 0.0]}}
lift_high: {moves: joints, joints: {torso: 0.2}}   # only the lift: the arms stay as they are
```

The example cell's file has `home_pose` and the hand and carry targets shown above; `lift_high` is added here to show a
one-group target.

## The joints part

Targets and missions name joint **groups**, never single joints, so the same file works on another robot. The groups are
in the robot profile (`joints.groups`): `torso`, `head`, `left_arm`, `right_arm` on the Galbot S1.

| Value | Meaning |
|---|---|
| a number | A group with one joint, or `torso` when the profile has `torso.joint`. |
| a list | One value per joint, in the order of the group. `null` leaves that joint where it is. |
| `auto` | `torso` only: search the torso height. Needs a hand or object pose in the same target or step. |
| `keep` | `torso` only: do not move it. |

The joints part is applied first, then the pose of the target is moved to. A target with only `joints:` is a joint
target. Head values are groups too (`head: [0.0, -0.3]`); `look_at` is reserved and not built.

## Paths

A hand, object or base target can give `path: [waypoint, ...]` instead of a single pose. Each waypoint is in the pose
form of the kind: `{x, y, yaw}`, `{xyz, rpy}` or `{<tool>_tool: ...}`. The robot visits them in order, each leg moved as
the step's mode says, and the last one is the target. `path` and a single pose together are an error.

```yaml
# example: cell/targets.yaml
home: {moves: base, x: 0.0, y: 0.0, yaw: 0.0}
staging: {moves: base, x: 1.0, y: 0.5, yaw: 1.5708}
via_door:                       # drive through the gap, then to the staging point
  moves: base
  path:
  - {x: 1.0, y: 0.0, yaw: 0.0}
  - {x: 1.0, y: 0.5, yaw: 1.5708}
```

## Objects: types, robots and instances

An object-frame target `<name>` on the placed object `<instance>` (of type `<T>`) for robot `<R>` is the **per-key merge**
of these layers, the later one winning:

1. `objects/<T>.yaml`, `targets.<name>`
2. `robots/<R>/objects/<T>.yaml`, `targets.<name>`
3. `objects/<T>.yaml`, `instances.<instance>.<name>`
4. `robots/<R>/objects/<T>.yaml`, `instances.<instance>.<name>`

- `left_tool`, `right_tool`, `x`, `y`, `yaw`, `xyz` and `rpy` replace as a whole. `grip_m`, `alone` and `joints` merge per
  hand or per group.
- A layer that has no `moves` only adds settings (the robot's `elbow` above). If two layers give `moves`, they must agree.
- A name that exists only in a robot layer needs `moves` (a stance). Settings for a name no layer defines are an error:
  `settings for an unknown target`.
- The same layout holds for a robot that has a stance for a type and nothing else: `robots/<robot>/objects/<type>.yaml`
  with only `pick`.

**Instances** override single keys for one placed object (`part_stack_2` above). The instance name is an entry of
`cell/objects.yaml`; it may also be one a mission creates with `grip ... place_as`.

**Stacks.** An object with `stack: {count, pitch}` is the pose of its bottom item. Every spatial target on it refers to
the **top item**, `z + (count - 1) * pitch`. The count is the run-time state (`state/objects.yaml`) that `grip close` lowers
after a grasp. With count 0 the instance is gone: `'part_stack_1' was taken in an earlier step or run`.

**`handled_from`.** A type with `yaw_symmetry_deg` (a frame that can lie the other way round) is handled in the turn
that suits the robot. `handled_from: <base target>` in the robot's object file says which stance decides that: the turn
whose `handled_from` stance is nearest the robot's base is used for all the type's targets. Without the key, and without
a symmetry, nothing is turned.

## Names are unique per place

A reference has no kind prefix, so a name has one meaning in its place:

- plain names: `cell/targets.yaml` and the robot's `targets.yaml` (robot pack plus the cell's override) together. The
  example cell calls its joint target `home_pose` because the cell already has a base target `home`;
- per object type: the type's targets and the robot's targets for it together. The same name in both files is one target
  with the robot's settings layered on it (2).

Two entries with one name in one place are an error: `target 'home' is in cell/targets.yaml and robots/duo_bot/targets.yaml:
rename one`.

## References

| Form | Names | Example |
|---|---|---|
| `<name>` | a cell, base-frame or joint target | `staging`, `home_pose`, `present_both`, `carry` |
| `<instance>/<name>` | an object-frame target of a placed object | `part_stack_1/pick`, `part_stack_1/grasp`, `bin_1/seat` |

A placeholder may stand in either part: `"{part}/pick"`. It is resolved when the step starts.

The resolution of `<instance>/<name>`: the instance must be a placed object (`cell/objects.yaml` plus the run-time
state). Its type gives the targets, the robot's layers are merged over them, then the target is composed with the
object's pose (a stack: its top item).

A hand target is also checked against the robot (tool type, grip width against the stroke) when a mission is validated.
A hand that holds something is not moved by a hand target: the step is `REJECTED`, see [skills.md](skills.md).

### Errors

Each message is one line and names the file to look at.

| Case | Message (shortened) |
|---|---|
| Name in two places | `target 'home' is in cell/targets.yaml and robots/duo_bot/targets.yaml: rename one` |
| Misspelt name | `unknown target 'stagin'; did you mean staging? (cell: 3, robot: 4; object targets need <instance>/<name>)` |
| Object target without an instance | `unknown target 'grasp': it is a target of part objects and needs an instance: <instance>/grasp, e.g. part_stack_1/grasp` |
| Unknown instance | `unknown target 'bin_9/pick': no object 'bin_9' in cell/objects.yaml, instances: ...` |
| Instance gone | `'part_stack_1' was taken in an earlier step or run (object state: state/objects.yaml); reset the sim ...` |
| Name not on the type | `object 'part' has no target 'grap' (it has: base, grasp, pick)` |
| Robot lacks it | `robot 'galbot_g1' has no 'pick' target for frame_type2 objects (robots/galbot_g1/objects/frame_type2.yaml)` |
| Wrong kind | `'staging' moves the base: ...` (a `via` item takes hand or joint targets; a hand option on a base target is rejected) |
| Old syntax | `'stance:part_stack_1/pick': the stance: prefix was removed in schema 4: run taskweave config migrate` |
| Old object form | `'part.grasp': the <object>.<point> form was removed in schema 4: use <instance>/<name>` |

## Teaching and editing

- `taskweave teach base NAME` saves the current base pose as a base target of the cell. `teach base INSTANCE/NAME` saves
  it as a stance of the object's type (`--instance`: for that instance only).
- `taskweave teach hand INSTANCE/NAME [--tool T]` saves the tools' current poses relative to the object (to its top item).
  `teach hand NAME` saves them in the robot's base frame as a robot target.
- `taskweave teach joints NAME [--tool GROUP]` saves the current joint values of the groups the target has, or of every
  group, as a joint target.
- Carry poses in the base frame (`moves: object`) and contacts are written by hand or in the HMI, not taught.

The HMI Teach tab edits the same files ([operator.md](operator.md)); every write keeps the comments and is checked
before it is saved.
