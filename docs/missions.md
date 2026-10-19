# Writing missions

A mission is one YAML file in the cell pack, `missions/<name>.yaml`: a goal and an ordered list of steps. Each step calls
a skill ([skills.md](skills.md)) or includes another mission. The task handler validates the whole mission before
anything moves, then runs the steps and applies the decision rules you wrote. Everything a mission names (stances,
points, poses, locations) comes from the cell pack and the robot pack ([configuration.md](configuration.md)).

## Anatomy of a mission file

```yaml
goal: Move parts from their stacks into the bin      # required, shown in the HMI and logs
robot: duo_bot                                       # optional: only for robot-specific missions; omitted = any robot
loop: forever                                        # once (default) | forever
max_cycles: 12                                       # optional, with loop: forever
stop_when: bin_full                                  # optional world-state flag that ends the mission
params: {part: {point: instance, description: "the placed part"}}   # only for blocks that others include
facts: {next_stack: {fullest: part, among: "part_stack_*"}}         # worked out once per cycle
defaults: {timeout_s: 120, retries: 1, on_failure: abort, max_replans: 1}
steps:
  - id: pick                                         # optional, s1, s2, ... if missing
    instruction: Pick the top part of {next_stack}   # required, natural language
    skill: navigate_to                               # a skill, or include: <mission> (never both)
    args: {target: "stance:{next_stack}"}
    timeout_s: 60                                    # per-step decision rules, else the defaults
    retries: 2
    on_failure: replan
    replan_from: pick
    max_replans: 2
```

| Key | Meaning |
|---|---|
| `goal` | What the mission is for. Required. |
| `robot` | Restricts the mission to one robot model. Leave it out so the same mission runs on any robot that can run its skills. |
| `loop` | `once`: run the steps once. `forever`: start over after the last step (the planner builds the plan again, so facts are worked out again). |
| `max_cycles` | With `loop: forever`: stop after this many cycles. |
| `stop_when` | With `loop: forever`: name of a world-state key; the mission is complete (ends cleanly) when it is truthy at the start of a cycle. |
| `defaults` | `timeout_s`, `retries`, `on_failure`, `max_replans` for every step that does not set its own. |
| `steps` | At least one step. |

### Step keys

| Key | Meaning |
|---|---|
| `id` | Name to refer to the step (`replan_from`) and to find it in logs and recordings. Unique in the plan. Missing: `s1`, `s2`, ... Steps of an included mission get `<include step id>_<inner id>`. |
| `instruction` | Required. What the step does in words. Also the input of language-conditioned skills (VLA). Placeholders work here too. |
| `skill` + `args` | The skill and its arguments; see [skills.md](skills.md). |
| `include` + `with` | Replace the step by another mission's steps, with its parameters filled from `with`. |
| `timeout_s` | The step fails with `TIMEOUT` after this long. Counted in work time: sim time scaled by the global speed override, so a step at 10 % gets ten times as long. |
| `retries` | Extra attempts of the same step after `FAILED` or `TIMEOUT`. |
| `on_failure` | When the retries are used up: `retry` (the step again with fresh retries), `replan` (ask the planner), `skip` (log and continue), `abort` (end the mission as failed). |
| `replan_from` | The step to continue from when the planner replans; default is the failed step. Must be a step id of the plan. |
| `max_replans` | How often one step may trigger a replan per cycle; after that the mission aborts. `retry` and `replan` both count. |

Pick `abort` for steps whose failure leaves the robot in an unknown state (holding something, arm out) and `skip` for
harmless ones (a final drive home). Use `replan_from` to restart from a safe point: the step that drives to the
object, not the one that grasps it.

### Speed override and test mode

The global speed override (README, "Speed override and test mode") scales every skill and, by 1/override, the step
timeouts. One exception: the Nav2 phase of `navigate_to` never drives slower than its param `min_override` (default 0.3,
set it in `skills/navigate_to.yaml`), because Nav2's controller cannot do the fine final approach into a stance at 10 %.
So test mode at 10 % = arm motion 10 %, Nav2 drives 30 %. The direct controller after the handover, `drive_straight`,
arms, torso and grippers follow the real override. The step result shows the Nav2 value as `data.speed.nav2_override`.

## Params, facts and placeholders

A `{name}` in a string of a step (`instruction` or any argument, at any depth) is a **placeholder**. It is filled from
the **world state** when the step starts; a string that is exactly one placeholder keeps the value's type
(`"{lane}"` can become the number `2`), inside a longer string it is formatted in (`"stance:{stack}"`). Arguments with
placeholders are validated when the step starts, not before the mission starts. A placeholder without a value fails
the step with `world state has no value for placeholder {name}`.

- **`facts`** are worked out when each cycle is planned and stay fixed for the cycle, although the cycle changes the
  objects. `{fullest: <object type>, among: <glob of instance names>}` is the placed object with the most items left
  (the run-time object state; ties go to the first name). A fact that finds nothing means the mission is complete: a
  `loop: forever` mission ends once every matching stack is empty.
- **`params`** make a mission a reusable block. `params: {part: {point: instance, description: ...}}` declares the
  parameter; the optional `point` kind (`base_target`, `joint_pose`, `tool_pose`, `object_point`, `instance_point`,
  `instance_contact`, `instance`) lets the HMI editor offer matching names. The steps use `{part}`.
- **`include` / `with`**: `{instruction, include: pick_part, with: {part: "{next_stack}"}}` replaces the step by the
  steps of `missions/pick_part.yaml`, with `{part}` filled from `with` (a literal name or a placeholder of the including
  mission). Placeholders that are not parameters stay world-state placeholders. The include step's `timeout_s`,
  `retries`, `on_failure` and `max_replans` apply to the inner steps if set, otherwise the included mission's defaults
  do. `with` must give every parameter of the block and no others. Includes nest 5 deep and must not loop. Only the included
  mission's steps and defaults count: its `loop`, `facts` and `stop_when` are ignored.

## Targets

Skill arguments name things in the config. A wrong name is reported before the mission starts.

| Target | Form | Used by | Defined in |
|---|---|---|---|
| Location | `staging` or `location:staging` | `navigate_to`, `drive_straight` `to` | `cell/locations.yaml` |
| Stance | `stance:<instance>` or `stance:<instance>/<variant>` | `navigate_to`, `drive_straight` `to` | `robots/<robot>/stances.yaml`, relative to the placed object `<instance>` of `cell/objects.yaml` |
| Point on a placed object | `<instance>/<point>`, e.g. `part_stack_1/grasp` | `move_gripper` `to.point` | `objects/<type>.yaml` `points:` (taught with `taskweave teach point`) |
| Contact | `<instance>/<contact>`, e.g. `bin_1/seat` | `move_object` `to.onto` (plus `contact`, the held object's own contact) | `objects/<type>.yaml` `contacts:` |
| Held pose | `carry` | `move_object` `to.pose` | `robots/<robot>/held_poses.yaml`, per object type |
| Tool pose | `present_both` | `move_gripper` `to.pose` | `robots/<robot>/poses.yaml` (entries with `tools:`) |
| Joint pose | `home` | `move_joints` `pose`, `motion.via` | `robots/<robot>/poses.yaml` (joint values) |
| Joint group | `left_arm`, `torso` | `move_joints` `only` | profile `joints.groups` |

- A **stance** is where the robot stands to work on an object, stored relative to the object type (or one instance), so
  moving the object moves the stance. The first variant is the default. `stance:fixture_1/work` is the robot's `work`
  variant for the `fixture` type. Missing for this robot: the step is rejected, the mission itself does not change.
- A **point** carries one tool pose per hand (`left_tool`, `right_tool`), canonical TCP poses relative to the object, so
  they hold on any robot that can reach them. `approach_m` is the straight last part of the way, `alone:` gives
  one-hand poses, `grip_width_m` is checked against the robot's tools. Arriving at a point with no offset makes closing
  the grippers there a **grasp**: the object is then held by those hands, until `grip` opens them.
- A **contact** is a frame where one object meets another (a part's `base` on a bin's `seat`). `move_object` puts the held
  object's `contact` onto the placed one's, `offset_m` short of it against its engage direction (z). Use the offset
  step first, then the same step without offset.
- **Placing without a contact.** `to.onto` needs a contact on the placed object (`contacts:` in its `objects/<type>.yaml`).
  If it has none, either put the object where you can name the spot: `move_object {to: {at: [x, y, z]}}` (robot frame: where the
  object origin goes, orientation kept) or `{to: {pose: <held pose>}}` (a named held pose of the robot for this type of
  object), or add a contact to the object type (a `{xyz, rpy}` frame, z = the direction the object moves to engage) and
  use `to.onto` as in [Example 4](#example-4-pick-up-a-bin-and-put-it-on-the-fixture).
- Where an object is put down, `grip open` with `place_as: <name>` records it as a placed object of that name, whose
  points exist from then on (validation knows a point of a `place_as` object only after that step).

## Motion options

The arm skills (`move_gripper`, `move_object`, `move_joints`, `set_posture`) take an optional `motion` object; `grip`
takes `speed`, `accel` and `tolerance`. The tables are in [skills.md](skills.md). The common ones:

| Option | Effect |
|---|---|
| `mode` | `free`: joint space, planned around obstacles, the last `approach_m` straight. `linear`: the TCPs on straight lines (`move_object` default). |
| `speed` | Fraction (0 to 1] of the skill's standard speed. The global override applies on top. `speed_abs` gives absolute speeds instead. |
| `accel` | Fraction of the standard acceleration. |
| `elbow` | Side the elbows keep or reach: `down`, `up`, `out`, `in`, `back`, `forward` (robot frame). |
| `planner` | `auto`: straight joint move if free, else an elbow swing, else the planner. `direct`: straight move only. `planned`: always the planner. |
| `approach_m` | Length of the straight approach in `free` mode; `0` plans all the way. |
| `torso` | `keep` or `auto` (try the configured torso heights when a target is out of reach). |
| `via` | Waypoints to pass (`move_joints`: joint pose names or `{joint: value}` maps). |
| `tolerance` | End accuracy. |

Defaults are layered, later wins: the skill yaml (`skills/<skill>.yaml`, `motion:`, `speed:`, `accel:`), the robot pack's
`skill_overrides.yaml`, the target's own data (a point's `approach_m` and `elbow`), then the step's `motion`. Put a
number in a mission only when this step differs from the usual; put the usual in config.

## Hands

`hands` selects which hands act. Values: a hand name from the robot profile (`left`), a list (`[left, right]`) or a
rule word.

| Rule | Meaning |
|---|---|
| `auto` | The hands the target defines (a point with `left_tool` and `right_tool` uses both); for a held object, the hands that hold it. Default of `move_gripper` and `move_object`. |
| `any_one` | One free hand that can take the target (`move_gripper` only). |
| `all` | Every hand. |

`grip` takes `auto` and `all`; `move_object` and `set_posture` take `auto` (the holding hands) and `all`; `move_gripper` takes all three.

A one-hand step on a two-hand point uses the point's `alone:` poses. Closing hands at a point makes them hold the object,
also one hand of a two-hand point; opening takes them out of the hold, and an object with no hands left is released.
`move_object` keeps every holding hand locked to the object; `set_posture` defaults to the holding hands, else all.

**The free arm moves while the other hand holds.** A hand that holds an object stays with it: `move_gripper` for that hand and `move_joints` on its arm joints (also as part of a whole-body pose or a `via`) are REJECTED, naming the holding hand. The other hand may `move_gripper`, `move_joints` its own arm group and `grip`; torso and head moves are allowed, and every hold follows them. The motion planner does not know the held object: the free arm's path is checked against the robot and the planning scene only, so the object itself is not avoided (objects have no collision shape in the config yet, `mesh` is the simulator's visual asset).

## The example cell

The examples run against `docs/examples/cell`, a cell pack of its own: `taskweave check docs/examples/cell` works on it as it
stands, and `taskweave_skills/test/test_docs_examples.py` validates every example mission against it.

| File | Holds |
|---|---|
| `pack.yaml` | name, version, `schema_version: 3` |
| `cell/locations.yaml`, `cell/objects.yaml` | `home`, `staging`; placed objects: `part_stack_1`, `part_stack_2` (stacks of `part`), `bin_1`, `fixture_1` |
| `objects/part.yaml` | point `grasp` (both hands, and `alone` for one), contact `base` |
| `objects/bin.yaml` | point `grasp` (left hand), contacts `seat` (where a part sits) and `base` |
| `objects/fixture.yaml` | point `handle` (a lever), contact `slot` (where something put on the fixture sits) |
| `robots/duo_bot/` | the robot: `profile.yaml`, `embodiment.yaml`, `poses.yaml` (`home`, `present_both`), `stances.yaml`, `held_poses.yaml` (`carry` for `part` and `bin`), `object_points.yaml` |

The robot is `duo_bot`, a neutral test robot (fixed base, a lift, two 2-joint arms, two parallel grippers, IK by a fake
MoveIt). Its files here are a copy of `taskweave_skills/test/fixtures/config/robots/duo_bot` (the test fixture the
skill tests run on), with joint groups added for `move_joints only`. It is not a shipped robot pack on purpose: the
tool poses, stances and carry poses are made up to fit the example objects and mean nothing for a real robot, and a pack in
`taskweave_robots` would suggest they do. To try the missions on a shipped pack (`galbot_s1`, ...), keep the cell files
and give the cell that robot's stances and held poses (`robots/<robot>/stances.yaml`, `held_poses.yaml`, taught on the
robot) as in [configuration.md](configuration.md).

## Example 1: drive, pick and place with two hands

Each step has a short instruction; the grasp is a pre-grasp offset, then the point itself, then `grip close`.

```yaml
goal: Pick a part from its stack with both hands and put it into the bin
loop: once
defaults: {timeout_s: 120, retries: 1, on_failure: abort}
steps:
  - id: tuck
    instruction: Arms in the driving pose
    skill: move_joints
    args: {pose: home}
  - id: to_part
    instruction: Drive to the pick stance at the part stack
    skill: navigate_to
    args: {target: "stance:part_stack_1"}
  - instruction: Open both grippers
    skill: grip
    args: {hands: all, action: open}
  - instruction: Pre-grasp 10 cm above the part
    skill: move_gripper
    args: {to: {point: part_stack_1/grasp, offset_m: 0.1}}
    retries: 2
    on_failure: replan
    replan_from: to_part
  - instruction: Onto the part
    skill: move_gripper
    args: {to: {point: part_stack_1/grasp}, motion: {mode: linear, speed: 0.5}}
  - instruction: Close both grippers on the part
    skill: grip
    args: {hands: all, action: close}
  - instruction: Lift the part 20 cm
    skill: move_object
    args: {to: {by: [0, 0, 0.2]}}
  - instruction: Into the carry pose
    skill: move_object
    args: {to: {pose: carry}}
  - id: to_bin
    instruction: Drive to the bin
    skill: navigate_to
    args: {target: "stance:bin_1/place"}
  - instruction: Part onto the seat, 5 cm short of it
    skill: move_object
    args: {to: {onto: bin_1/seat, contact: base, offset_m: 0.05}}
  - instruction: Down onto the seat
    skill: move_object
    args: {to: {onto: bin_1/seat, contact: base}}
  - instruction: Let go
    skill: grip
    args: {hands: all, action: open}
  - instruction: Arms in the driving pose
    skill: move_joints
    args: {pose: home}
```

Notes: the pre-grasp step retries twice and then replans from `to_part`, so the robot drives to the stance again.
`motion: {mode: linear, speed: 0.5}` makes the last move a straight line at half speed. `carry` is a held pose of the
robot. The bin step goes in two moves: 5 cm short of the seat, then onto it.

## Example 2: one hand

Hand selection with `hands: [left]` on a point that has a single tool pose, and `motion.elbow` for the arm shape.

```yaml
goal: Pull the lever of the fixture with the left hand and let go again
loop: once
defaults: {timeout_s: 90, retries: 0, on_failure: abort}
steps:
  - instruction: Drive to the work stance at the fixture
    skill: navigate_to
    args: {target: "stance:fixture_1/work"}
  - instruction: Open the left gripper
    skill: grip
    args: {hands: [left], action: open}
  - instruction: Left hand to the lever, 10 cm short of it
    skill: move_gripper
    args: {to: {point: fixture_1/handle, offset_m: 0.1}, hands: [left], motion: {elbow: out}}
  - instruction: Onto the lever
    skill: move_gripper
    args: {to: {point: fixture_1/handle}, hands: [left], motion: {mode: linear}}
  - instruction: Grip the lever
    skill: grip
    args: {hands: [left], action: close}
  - instruction: Pull 8 cm towards the robot
    skill: move_object
    args: {to: {by: [-0.08, 0, 0]}, motion: {speed: 0.3}}
  - instruction: Let go
    skill: grip
    args: {hands: [left], action: open}
  - instruction: Left hand back to the driving pose
    skill: move_joints
    args: {pose: home}
```

## Example 3: a block included by another mission

`pick_part.yaml` is a block with a parameter; `restock.yaml` includes it with a fact and then places the part.

```yaml
# Block (included by other missions, with: {part: <placed part>}): pick the part and hold it in the carry pose.
goal: Pick a part and hold it in the carry pose
params:
  part: {point: instance, description: "the placed part (cell/objects.yaml), e.g. part_stack_1"}
loop: once
defaults: {timeout_s: 120, retries: 0, on_failure: abort}
steps:
  - instruction: Drive to the pick stance at {part}
    skill: navigate_to
    args: {target: "stance:{part}"}
  - instruction: Open both grippers
    skill: grip
    args: {hands: all, action: open}
  - instruction: Pre-grasp above {part}
    skill: move_gripper
    args: {to: {point: "{part}/grasp", offset_m: 0.1}}
  - instruction: Onto the part
    skill: move_gripper
    args: {to: {point: "{part}/grasp"}}
  - instruction: Close both grippers
    skill: grip
    args: {hands: all, action: close}
  - instruction: Into the carry pose
    skill: move_object
    args: {to: {pose: carry}}
```

```yaml
# Each cycle takes a part from the fullest stack (fact next_stack, from the run-time object state), puts it into the
# bin and starts again. The mission is complete when no stack has parts left.
goal: Move every part of the stacks into the bin, fullest stack first
loop: forever
facts:
  next_stack: {fullest: part, among: "part_stack_*"}
defaults: {timeout_s: 120, retries: 0, on_failure: abort}
steps:
  - id: pick
    instruction: Pick the top part of {next_stack}
    include: pick_part
    with: {part: "{next_stack}"}
    on_failure: replan
  - instruction: Drive to the bin
    skill: navigate_to
    args: {target: "stance:bin_1/place"}
  - instruction: Part onto the seat
    skill: move_object
    args: {to: {onto: bin_1/seat, contact: base}}
  - instruction: Let go
    skill: grip
    args: {hands: all, action: open}
```

The planner works out `next_stack` at the start of each cycle, the include step becomes the six steps of `pick_part`
(ids `pick_s1` ... `pick_s6`) with `{part}` filled, and the mission ends when no `part_stack_*` has parts left.

## Example 4: pick up a bin and put it on the fixture

One hand, from a point on the bin to a contact on the fixture: the left hand takes the bin by its `grasp` point, carries
it, and puts its `base` contact onto the fixture's `slot`.

```yaml
goal: Take the bin with the left hand, put it onto the fixture, arm back to the driving pose
loop: once
defaults: {timeout_s: 120, retries: 1, on_failure: abort}
steps:
  - id: to_bin
    instruction: Drive to the pick stance at the bin
    skill: navigate_to
    args: {target: "stance:bin_1/pick"}
  - instruction: Open the left gripper
    skill: grip
    args: {hands: [left], action: open}
  - id: pregrasp
    instruction: Left hand 10 cm short of the bin's grasp point
    skill: move_gripper
    args: {to: {point: bin_1/grasp, offset_m: 0.1}, hands: [left]}
    retries: 2
    on_failure: replan
    replan_from: to_bin
  - instruction: Left hand straight onto the grasp point, at half speed
    skill: move_gripper
    args: {to: {point: bin_1/grasp}, hands: [left], motion: {mode: linear, speed: 0.5}}
  - instruction: Close the left gripper on the bin
    skill: grip
    args: {hands: [left], action: close}
  - instruction: Lift the bin 20 cm
    skill: move_object
    args: {to: {by: [0, 0, 0.2]}, hands: [left]}
  - instruction: Bin into the carry pose
    skill: move_object
    args: {to: {pose: carry}, hands: [left]}
  - id: to_fixture
    instruction: Drive to the work stance at the fixture
    skill: navigate_to
    args: {target: "stance:fixture_1/work"}
  - instruction: Bin onto the fixture slot, 5 cm short of it
    skill: move_object
    args: {to: {onto: fixture_1/slot, contact: base, offset_m: 0.05}, hands: [left]}
  - instruction: Bin down onto the fixture slot
    skill: move_object
    args: {to: {onto: fixture_1/slot, contact: base}, hands: [left]}
  - instruction: Let go of the bin with the left hand
    skill: grip
    args: {hands: [left], action: open}
  - id: tuck
    instruction: Arms back in the driving pose
    skill: move_joints
    args: {pose: home}
```

Notes: no `to.onto` without the `slot` contact on `objects/fixture.yaml`; `contact: base` is the bin's own contact, the
one that goes onto the slot. `hands: [left]` on `move_object` makes it explicit that only the left hand holds.

## Validate a mission

1. `taskweave check <cell>` (offline, no ROS graph) checks what needs no skill catalog: every file of the cell pack and of
   its robots (`robots/<robot>`, cell first, then the robot packs) loads against its schema, the profile and the
   embodiment agree (base type, tools; warnings for an IK arm without elbow description and for keys under `arms` that
   nothing reads), each `skills/<name>.yaml` loads and its classes import, and every mission expands (includes,
   parameters) into a plan. It does **not** check skill arguments or any name a step uses: a misspelt point, contact,
   stance, held pose or joint pose, a wrong `hands` value or a missing argument passes. Those need the skills'
   argument schemas, so they are checked by 2 (live catalog) or 3 (the same offline, in a test).
2. The HMI mission editor checks the arguments against the live skill catalog, as the task handler does before a start:
   skill exists and has an implementation for the robot, arguments fit the schema, points, stances, contacts and poses
   exist (a problem names the step and the argument). Starting an invalid mission is refused with the reasons.
3. In a test, run the same check offline: build the catalog from the skill classes' `args_schema`, then
   `validate_plan(Plan.from_mission(store.expanded_mission(name), name, robot=robot), catalog, store)`; see
   `taskweave_skills/test/test_docs_examples.py`, which does it for every example above.
4. `taskweave describe <cell> <robot>` prints the robot card and the skills it can run.

## Common errors

| Message (shortened) | Cause and fix |
|---|---|
| `the skill move_to_pose was removed, use move_joints ...` (also `go_to_point`, `gripper`) | The mission is from before the skill rework. `taskweave config migrate-skills <cell> [--dry-run]` rewrites missions, `operator.yaml` and `skills/`. |
| `move_object takes its target as to now ...` | Old `move_object` arguments (`by`, `pose`, `elbow` ...). Same migration: `to: {by: ...}`, the elbow argument became the skill `set_posture`. |
| `cell pack schema 2, this taskweave needs 3: run taskweave config migrate-skills <cell>` | Pack from an older layout: `taskweave config migrate <cell>` runs every step up to the current schema (1 to 2: robot packs, 2 to 3: skills); `migrate-skills` is the last step alone. |
| `no skill server offers it; available: ...` | The skill is not served: list it in the launch `skills:` argument and give it a `skills/<name>.yaml`. |
| `no implementation fits robot ... (...)` | The `requires` of every implementation in `skills/<name>.yaml` fail against the robot's embodiment; the reasons follow. Add an implementation or fix the capabilities. |
| `args.to.point: <x> names no placed object: use <instance>/<point>` | `move_gripper` takes `<instance>/<point>` (an entry of `cell/objects.yaml`), not `<object>.<point>`. |
| `unknown stance ...: no object X in cell/objects.yaml` / `... robot R has no stance for T objects` | The instance is not in `cell/objects.yaml`, or `robots/<robot>/stances.yaml` has no stance for its type: teach one (`taskweave teach stance <instance>`). |
| `unknown placed object ...` | `object:` of `move_object` (or a point's instance) is not in `cell/objects.yaml`. |
| `pose X places no tools (joint values only): use move_joints` / `pose X places tools: use move_gripper to.pose` | Joint poses go to `move_joints`, tool poses to `move_gripper`. |
| `robot ... has no held pose ...` | Add it to `robots/<robot>/held_poses.yaml` for the object type. |
| `robot ... has no joint group ...` | `move_joints only:` takes groups of the profile's `joints.groups`. |
| `step id ... is used more than once` / `replan_from ... is not a step of this plan` | Ids must be unique; `replan_from` names a step of the expanded plan (an include's inner steps are `<id>_<inner id>`). |
| `world state has no value for placeholder {name}` | The placeholder is not a parameter of the block, a fact or a world-state key; check the spelling and the `with:`. |
| An error naming a file and a key when a mission loads | The config schemas reject unknown keys and wrong types and name the file and the key; a newly added key needs the taskweave version that knows it. |
| `nothing is held: move_gripper to the object, then grip close` (REJECTED at run time) | `move_object` needs a hold: go to the point with `move_gripper` (no offset), then `grip close`. |
| `has no parameter ...` / missing parameters of an `include` | `with:` must give exactly the block's `params`. |
