# Writing missions

A mission is one YAML file in the cell pack, `missions/<name>.yaml`: a goal and an ordered list of steps. Each step calls
a skill ([skills.md](skills.md)) or includes another mission. The task handler validates the whole mission before
anything moves, then runs the steps and applies the decision rules you wrote. Everything a mission names (targets:
places the robot goes to, see [targets.md](targets.md)) comes from the cell pack and the robot pack
([configuration.md](configuration.md)).

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
    skill: move                                      # a skill, or include: <mission> (never both)
    args: {to: "{next_stack}/pick"}
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
timeouts. One exception: the Nav2 phase of a `move` to a base target never drives slower than its param `min_override`
(default 0.3, set it in `skills/move/base.yaml`), because Nav2's controller cannot do the fine final approach into a
stance at 10 %. So test mode at 10 % = arm motion 10 %, Nav2 drives 30 %. The direct controller after the handover, a
straight drive (`motion.mode: straight`), arms, torso and grippers follow the real override. The step result shows the Nav2 value as `data.speed.nav2_override`.

## Params, facts and placeholders

A `{name}` in a string of a step (`instruction` or any argument, at any depth) is a **placeholder**. It is filled from
the **world state** when the step starts; a string that is exactly one placeholder keeps the value's type
(`"{lane}"` can become the number `2`), inside a longer string it is formatted in (`"{stack}/pick"`). Arguments with
placeholders are validated when the step starts, not before the mission starts. A placeholder without a value fails
the step with `world state has no value for placeholder {name}`.

- **`facts`** are worked out when each cycle is planned and stay fixed for the cycle, although the cycle changes the
  objects. `{fullest: <object type>, among: <glob of instance names>}` is the placed object with the most items left
  (the run-time object state; ties go to the first name). A fact that finds nothing means the mission is complete: a
  `loop: forever` mission ends once every matching stack is empty.
- **`params`** make a mission a reusable block. `params: {part: {point: instance, description: ...}}` declares the
  parameter; the optional `point` kind (`instance`: a placed object, `target`: a target) lets the HMI editor offer
  matching names. The steps use `{part}`.
- **`include` / `with`**: `{instruction, include: pick_part, with: {part: "{next_stack}"}}` replaces the step by the
  steps of `missions/pick_part.yaml`, with `{part}` filled from `with` (a literal name or a placeholder of the including
  mission). Placeholders that are not parameters stay world-state placeholders. The include step's `timeout_s`,
  `retries`, `on_failure` and `max_replans` apply to the inner steps if set, otherwise the included mission's defaults
  do. `with` must give every parameter of the block and no others. Includes nest 5 deep and must not loop. Only the included
  mission's steps and defaults count: its `loop`, `facts` and `stop_when` are ignored.

## Targets

Skill arguments name targets in the config: places the robot goes to. A wrong name is reported before the mission starts.
The format of the files is in [targets.md](targets.md); here is how a mission names them.

| Reference | Names | Example in the example cell |
|---|---|---|
| `<name>` | a target of the cell (`cell/targets.yaml`), a base-frame target of the robot or a joint target (`robots/<robot>/targets.yaml`) | `staging`, `present_both`, `carry`, `home_pose` |
| `<instance>/<name>` | a target on a placed object of `cell/objects.yaml`: a stance, a hand point, a contact | `part_stack_1/pick`, `part_stack_1/grasp`, `bin_1/seat` |

What a target moves comes from the target itself (`moves`), and the same step form goes to all of them:

| The target moves | Example | `move` does |
|---|---|---|
| the base | `{to: part_stack_1/pick}` | drives there (Nav2 where the robot has it, else direct), `motion: {mode: straight}` in a straight line |
| a hand (or both) | `{to: part_stack_1/grasp, offset_m: 0.1}` | moves the tools to the pose, `offset_m` short of it |
| a held object | `{to: bin_1/seat}` or `{to: carry}` | moves the held object, the holding hands locked to it |
| joints | `{to: home_pose}` | moves the joint groups of the target |

- A **stance** is where the base stands to work on an object, stored relative to the object type (or one instance), so
  moving the object moves the stance. `fixture_1/work` is the robot's `work` stance for the `fixture` type. Missing for this
  robot: the step is rejected, the mission itself does not change. There is no default stance: a step names it.
- A **hand target** carries one tool pose per hand (`left_tool`, `right_tool`), canonical TCP poses relative to the object,
  so they hold on any robot that can reach them. `approach_m` is the straight last part of the way, `alone:` gives
  one-hand poses, `grip_width_m` is checked against the robot's tools. Arriving at a hand target on an object with no
  offset makes closing the grippers there a **grasp**: the object is then held by those hands, until `grip` opens them.
- A **contact** is an object target on an object: a frame where one object meets another (a part's `base` on a bin's
  `seat`). `move {to: bin_1/seat}` puts the held object's reference (the part's `base`, named in the target's `reference`)
  onto it, `offset_m` short of it against its engage direction (z). Use the offset step first, then the same step
  without offset.
- **Placing without a contact.** A `to` that is an object target needs a contact on the placed object (a target with
  `moves: object` in its `objects/<type>.yaml`). If it has none, put the object where you can name the spot:
  `move {object: held, at: [x, y, z]}` (robot frame: where the object origin goes, orientation kept) or `{to: <carry
  pose>}` (a named object target in the robot's base frame, for this type of object), or add a contact to the object
  type and use it as in [Example 4](#example-4-pick-up-a-bin-and-put-it-on-the-fixture).
- Where an object is put down, `grip open` with `place_as: <name>` records it as a placed object of that name, whose
  targets exist from then on (validation knows a target of a `place_as` object only after that step).

## The `move` skill

`move` is the one skill that moves the robot. A step takes one of six forms:

| Form | Required | Meaning |
|---|---|---|
| target | `to` | go to a target; its kind says what moves |
| hand relative | `hand` + `by` or `at` | move hands relative to now, or to a pose in the base frame |
| object relative | `object` + `by` or `at` | move the held object (`held`, or a placed object's name) relative to now |
| base straight | `base: {by: [forward, left]}` | drive straight, metres |
| elbow | `elbow` | swing the elbows to a side, the hands stay |
| joints | `joints` | joint group values, as in a target |

Which options apply depends on the kind of motion. A step with an option that does not apply is rejected before it runs
(`move base: option hands does not apply`). `taskweave check --missions` finds it offline.

| Option | base | straight | hand | object | elbow | joints |
|---|---|---|---|---|---|---|
| `hands` | - | - | yes | `auto`, `all` | yes | - |
| `offset_m` | - | - | yes | yes | - | - |
| `reference` | - | - | - | yes | - | - |
| `only` (groups of a joint target) | - | - | - | - | - | yes |
| `torso` | `keep`, a value | `keep`, a value | `auto`, `keep`, a value | `auto`, `keep`, a value | `keep` | - |
| `motion.mode` | `nav` | `straight` | `free`, `linear` | `linear`, `free` | - | - |

Argument tables: [skills.md](skills.md). The old skills `navigate_to`, `drive_straight`, `move_gripper`, `move_object`,
`set_posture` and `move_joints` are gone from cells of schema 4; the migration rewrites their steps
([migration_targets.md](migration_targets.md)).

## Motion options

`move` takes an optional `motion` object; `grip` takes `speed`, `accel` and `tolerance`. The tables are in
[skills.md](skills.md). The common ones:

| Option | Effect |
|---|---|
| `mode` | `free`: joint space, planned around obstacles, the last `approach_m` straight. `linear`: the TCPs on straight lines (default for a held object). For the base: `nav` (default) or `straight`. |
| `speed` | Fraction (0 to 1] of the standard speed. The global override applies on top. `speed_abs` gives absolute speeds instead. |
| `accel` | Fraction of the standard acceleration. |
| `elbow` | Side the elbows keep or reach: `down`, `up`, `out`, `in`, `back`, `forward` (robot frame). |
| `planner` | `auto`: straight joint move if free, else an elbow swing, else the planner. `direct`: straight move only. `planned`: always the planner. |
| `approach_m` | Length of the straight approach in `free` mode; `0` plans all the way. |
| `via` | Waypoints to pass: hand targets (hand move) or joint targets (joints move), by name. |
| `tolerance` | End accuracy. |

The torso is not in `motion` but a step option of its own, `torso: auto | keep | <value>`:

- `auto` chooses the torso height (robot profile `ik.torso`: least motion, reachable and collision free; else the first
  reachable of the `torso_fallback` heights). It is the default for hand targets and relative hand moves.
- `keep` leaves the torso where it is. It is the right choice for a move of a held object, which is why the migration
  writes it there.
- A number (or a list for several joints) sets it. The order of precedence is the step's `torso`, then the target's own
  `joints.torso`, then the default of the kind.
- For a base move, `torso` is the height before driving (default: the profile's `torso.drive_m`).

Defaults are layered, later wins: the skill yaml (`skills/move/<part>.yaml`, `motion:`, `speed:`, `accel:`), the robot
pack's `skill_overrides.yaml`, the target's own data (a hand target's `approach_m` and `elbow`), then the step's
`motion`. Put a number in a mission only when this step differs from the usual; put the usual in config.

## Hands

`hands` selects which hands act. Values: a hand name from the robot profile (`left`), a list (`[left, right]`) or a
rule word.

| Rule | Meaning |
|---|---|
| `auto` | The hands the target defines (a hand target with `left_tool` and `right_tool` uses both); for a held object, the hands that hold it. Default of `move`. |
| `any_one` | One hand: for `move` to a hand target the free hand that can take the target (see below); for `grip` the one hand at an object target. |
| `all` | Every hand. |

`grip` and `move` to a hand target take all three; a `move` of a held object and the elbow form take `auto` (the holding
hands) and `all`. `hands` is for the `to` form; the relative forms name their hands with `hand`.

`move` to a hand target with `any_one` chooses one hand, in this order:

1. Only free hands count: a hand that holds an object is never chosen.
2. Only hands the target can apply to: for an object target, a hand with an `alone:` pose or a pose in the pair; for a
   robot target in the base frame, a hand the target places.
3. Hands whose IK has a solution (the robot's `ik.choose` settings, no motion) come before those without.
4. Of those, the hand with the least joint motion from where the arm is now.
5. A tie goes to the first hand in the robot profile (`grippers` order), so the choice is repeatable.

The chosen hand is in the message and in the result data (`data.hands`, plus `data.choice` with every candidate's
motion or reason). If no hand can take the target the step is rejected with one reason per hand. `grip` with `any_one`
takes the hand that is at an object target (closing, `opening`), or for `open` the one hand that holds something; with
none or with both hands there it is rejected as ambiguous. `auto` on a target that only one free hand has a pose for (a
single `right_tool`) means that hand.

A one-hand step on a two-hand target uses the target's `alone:` poses. Closing hands at a hand target makes them hold the
object, also one hand of a two-hand target; opening takes them out of the hold, and an object with no hands left is
released. A `move` of a held object keeps every holding hand locked to it; the elbow form defaults to the holding hands,
else all.

**The free arm moves while the other hand holds.** A hand that holds an object stays with it: a hand move for that hand
and a joint move on its arm joints (also as part of a whole-body target or a `via`) are REJECTED, naming the holding hand.
The other hand may move, move its own arm group and `grip`; torso and head moves are allowed, and every hold follows them.
The motion planner does not know the held object: the free arm's path is checked against the robot and the planning scene
only, so the object itself is not avoided unless its type has a `collision` shape.

## The example cell

The examples run against `docs/examples/cell`, a cell pack of its own: `taskweave check docs/examples/cell` works on it as it
stands, and `taskweave_skills/test/test_docs_examples.py` validates every example mission against it.

| File | Holds |
|---|---|
| `pack.yaml` | name, version, `schema_version: 4` |
| `cell/targets.yaml`, `cell/objects.yaml` | base targets `home`, `staging`; placed objects: `part_stack_1`, `part_stack_2` (stacks of `part`), `bin_1`, `fixture_1` |
| `objects/part.yaml` | hand target `grasp` (both hands, and `alone` for one), object target `base` |
| `objects/bin.yaml` | hand target `grasp` (left hand), object targets `seat` (where a part sits) and `base` |
| `objects/fixture.yaml` | hand target `handle` (a lever), object target `slot` (where something put on the fixture sits) |
| `robots/duo_bot/` | the robot: `profile.yaml`, `embodiment.yaml`, `targets.yaml` (`home_pose`, `present_left`, `present_both`, `carry`), `objects/<type>.yaml` (its stances `pick`, `place`, `work`, and settings) |
| `skills/move.yaml`, `skills/move/*.yaml`, `skills/grip.yaml`, `skills/wait.yaml` | in `docs/examples/skills`: the skill files a cell pack starts from |

The robot is `duo_bot`, a neutral test robot (fixed base, a lift, two 2-joint arms, two parallel grippers, IK by a fake
MoveIt). Its files here are a copy of `taskweave_skills/test/fixtures/config_new/robots/duo_bot` (the test fixture the
skill tests run on), with joint groups added for the joints part of targets. It is not a shipped robot pack on purpose: the
tool poses, stances and carry poses are made up to fit the example objects and mean nothing for a real robot, and a pack in
`taskweave_robots` would suggest they do. To try the missions on a shipped pack (`galbot_s1`, ...), keep the cell files
and give the cell that robot's stances (`robots/<robot>/objects/<type>.yaml`, taught on the robot) as in
[configuration.md](configuration.md). The example cell is the migrated form of the schema 3 example (the migration's
golden test, [migration_targets.md](migration_targets.md)).

## Example 1: drive, pick and place with two hands

Each step has a short instruction; the grasp is a pre-grasp offset, then the point itself, then `grip close`.

```yaml
goal: Pick a part from its stack with both hands and put it into the bin
loop: once
defaults: {timeout_s: 120, retries: 1, on_failure: abort}
steps:
  - id: tuck
    instruction: Arms in the driving pose
    skill: move
    args: {to: home_pose}
  - id: to_part
    instruction: Drive to the pick position at the part stack
    skill: move
    args: {to: part_stack_1/pick}
  - instruction: Open both grippers
    skill: grip
    args: {hands: all, action: open}
  - instruction: Pre-grasp 10 cm above the part
    skill: move
    args: {to: part_stack_1/grasp, offset_m: 0.1}
    retries: 2
    on_failure: replan
    replan_from: to_part
  - instruction: Onto the part
    skill: move
    args: {to: part_stack_1/grasp, motion: {mode: linear, speed: 0.5}}
  - instruction: Close both grippers on the part
    skill: grip
    args: {hands: all, action: close}
  - instruction: Lift the part 20 cm
    skill: move
    args: {object: held, by: [0, 0, 0.2], torso: keep}
  - instruction: Into the carry pose
    skill: move
    args: {to: carry, torso: keep}
  - id: to_bin
    instruction: Drive to the bin
    skill: move
    args: {to: bin_1/place}
  - instruction: Part onto the seat, 5 cm short of it
    skill: move
    args: {to: bin_1/seat, offset_m: 0.05, torso: keep}
  - instruction: Down onto the seat
    skill: move
    args: {to: bin_1/seat, torso: keep}
  - instruction: Let go
    skill: grip
    args: {hands: all, action: open}
  - instruction: Arms in the driving pose
    skill: move
    args: {to: home_pose}
```

Notes: the pre-grasp step retries twice and then replans from `to_part`, so the robot drives to the stance again.
`motion: {mode: linear, speed: 0.5}` makes the last move a straight line at half speed. `carry` is an object target in
the robot's base frame, `home_pose` a joint target. `torso: keep` leaves the torso alone while the part is held. The bin
step goes in two moves: 5 cm short of the seat, then onto it.

## Example 2: one hand

Hand selection with `hands: [left]` on a hand target that has a single tool pose, and `motion.elbow` for the arm shape.

```yaml
goal: Pull the lever of the fixture with the left hand and let go again
loop: once
defaults: {timeout_s: 90, retries: 0, on_failure: abort}
steps:
  - instruction: Drive to the work position at the fixture
    skill: move
    args: {to: fixture_1/work}
  - instruction: Open the left gripper
    skill: grip
    args: {hands: [left], action: open}
  - instruction: Left hand to the lever, 10 cm short of it
    skill: move
    args: {to: fixture_1/handle, offset_m: 0.1, hands: [left], motion: {elbow: out}}
  - instruction: Onto the lever
    skill: move
    args: {to: fixture_1/handle, hands: [left], motion: {mode: linear}}
  - instruction: Grip the lever
    skill: grip
    args: {hands: [left], action: close}
  - instruction: Pull 8 cm towards the robot
    skill: move
    args: {object: held, by: [-0.08, 0, 0], motion: {speed: 0.3}, torso: keep}
  - instruction: Let go
    skill: grip
    args: {hands: [left], action: open}
  - instruction: Left hand back to the driving pose
    skill: move
    args: {to: home_pose}
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
  - instruction: Drive to the pick position at {part}
    skill: move
    args: {to: '{part}/pick'}
  - instruction: Open both grippers
    skill: grip
    args: {hands: all, action: open}
  - instruction: Pre-grasp above {part}
    skill: move
    args: {to: "{part}/grasp", offset_m: 0.1}
  - instruction: Onto the part
    skill: move
    args: {to: "{part}/grasp"}
  - instruction: Close both grippers
    skill: grip
    args: {hands: all, action: close}
  - instruction: Into the carry pose
    skill: move
    args: {to: carry, torso: keep}
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
    skill: move
    args: {to: bin_1/place}
  - instruction: Part onto the seat
    skill: move
    args: {to: bin_1/seat, torso: keep}
  - instruction: Let go
    skill: grip
    args: {hands: all, action: open}
```

The planner works out `next_stack` at the start of each cycle, the include step becomes the six steps of `pick_part`
(ids `pick_s1` ... `pick_s6`) with `{part}` filled, and the mission ends when no `part_stack_*` has parts left.

## Example 4: pick up a bin and put it on the fixture

One hand, from a hand target on the bin to a contact on the fixture: the left hand takes the bin by its `grasp` point,
carries it, and puts its `base` onto the fixture's `slot`.

```yaml
goal: Take the bin with the left hand, put it onto the fixture, arm back to the driving pose
loop: once
defaults: {timeout_s: 120, retries: 1, on_failure: abort}
steps:
  - id: to_bin
    instruction: Drive to the pick position at the bin
    skill: move
    args: {to: bin_1/pick}
  - instruction: Open the left gripper
    skill: grip
    args: {hands: [left], action: open}
  - id: pregrasp
    instruction: Left hand 10 cm short of the bin's grasp point
    skill: move
    args: {to: bin_1/grasp, offset_m: 0.1, hands: [left]}
    retries: 2
    on_failure: replan
    replan_from: to_bin
  - instruction: Left hand straight onto the grasp point, at half speed
    skill: move
    args: {to: bin_1/grasp, hands: [left], motion: {mode: linear, speed: 0.5}}
  - instruction: Close the left gripper on the bin
    skill: grip
    args: {hands: [left], action: close}
  - instruction: Lift the bin 20 cm
    skill: move
    args: {object: held, by: [0, 0, 0.2], hands: [left], torso: keep}
  - instruction: Bin into the carry pose
    skill: move
    args: {to: carry, hands: [left], torso: keep}
  - id: to_fixture
    instruction: Drive to the work position at the fixture
    skill: move
    args: {to: fixture_1/work}
  - instruction: Bin onto the fixture slot, 5 cm short of it
    skill: move
    args: {to: fixture_1/slot, offset_m: 0.05, hands: [left], torso: keep}
  - instruction: Bin down onto the fixture slot
    skill: move
    args: {to: fixture_1/slot, hands: [left], torso: keep}
  - instruction: Let go of the bin with the left hand
    skill: grip
    args: {hands: [left], action: open}
  - id: tuck
    instruction: Arms back in the driving pose
    skill: move
    args: {to: home_pose}
```

Notes: there is no step onto the fixture without the `slot` target on `objects/fixture.yaml`; its `reference: base` names
the bin's own contact, the one that goes onto the slot. `hands: [left]` on a move of the held object makes it explicit
that only the left hand holds.

## Validate a mission

1. `taskweave check <cell>` (offline, no ROS graph) checks what needs no skill catalog: every file of the cell pack and of
   its robots (`robots/<robot>`, cell first, then the robot packs) loads against its schema, the profile and the
   embodiment agree (base type, tools; warnings for an IK arm without elbow description and for keys under `arms` that
   nothing reads), each `skills/<name>.yaml` loads and its classes import, every target name is unique in its place,
   and every mission expands (includes, parameters) into a plan. It does **not** check skill arguments or any name a
   step uses: a misspelt target, a wrong `hands` value or a missing argument passes. `taskweave check <cell> --missions`
   also validates every mission's plan against the skills the cell serves (needs ROS and the skill packages); the live
   catalog (2) and a test (3) do the same.
2. The HMI mission editor checks the arguments against the live skill catalog, as the task handler does before a start:
   skill exists and has an implementation for the robot, arguments fit the schema, targets exist and fit the form of the
   step (a problem names the step and the argument). Starting an invalid mission is refused with the reasons.
3. In a test, run the same check offline: build the catalog from the skill classes' `args_schema`, then
   `validate_plan(Plan.from_mission(store.expanded_mission(name), name, robot=robot), catalog, store)`; see
   `taskweave_skills/test/test_docs_examples.py`, which does it for every example above.
4. `taskweave describe <cell> <robot>` prints the robot card and the skills it can run.

## Common errors

| Message (shortened) | Cause and fix |
|---|---|
| `the skill navigate_to was replaced by move on a cell of schema 4 (targets): write the step as move {...}` (also `drive_straight`, `move_gripper`, `move_object`, `set_posture`, `move_joints`) | The mission is from before schema 4. `taskweave config migrate <cell> [--dry-run]` rewrites missions, `operator.yaml` and `skills/` ([migration_targets.md](migration_targets.md)). |
| `'stance:part_stack_1/pick': the stance: prefix was removed in schema 4: run taskweave config migrate` | An old reference (`stance:`, `location:`, `pose:`, `<object>.<point>`). Write `<instance>/<name>` or `<name>`. |
| `cell pack schema 3, this taskweave needs 4: run taskweave config migrate <cell>` | Pack from an older layout: `taskweave config migrate <cell>` runs every step up to the current schema. |
| `unknown target 'wating'; did you mean waiting? ...` | Misspelt or missing target. The message counts the cell and robot targets; object targets need `<instance>/<name>`. |
| `unknown target 'grasp': it is a target of part objects and needs an instance: <instance>/grasp ...` | A target of an object type written without the object. |
| `unknown target 'bin_9/pick': no object 'bin_9' in cell/objects.yaml, instances: ...` | The instance is not in `cell/objects.yaml` (or it was removed from the run-time state). |
| `object 'part' has no target 'grap' (it has: ...)` / `robot 'R' has no 'pick' target for T objects (robots/R/objects/T.yaml)` | The type has no such target, or this robot has no stance for it: teach one (`taskweave teach base <instance>/pick`). |
| `target 'set_down' is in cell/targets.yaml and robots/R/targets.yaml: rename one` | A plain name has two meanings. Rename one of them. |
| `'waiting' moves the base: ...` / `move base: option hands does not apply` | The option or the form does not fit the kind of target (the table in [The `move` skill](#the-move-skill)). |
| `move hand: left holds frame_1: move the object (...), or open the gripper first` | A hand target for a hand that holds something. Move the held object, or `grip open`. |
| `'carry' is a base-frame object target for frame_type2: a held frame_type2 is needed, nothing is held` | A carry pose for an object that is not held. `grip close` on it first. |
| `move: look_at is not implemented yet: set head through joints` | `look_at` is reserved. Use `joints: {head: [...]}`. |
| `no skill server offers it; available: ...` | The skill is not served: list it in the launch `skills:` argument (`wait,move,grip`) and give it a `skills/<name>.yaml`. |
| `no implementation fits robot ... (...)` / `this robot cannot do <kind>: ...` | The `requires` of every implementation in `skills/<name>.yaml` (for `move`: of the part's `skills/move/<part>.yaml`) fail against the robot's embodiment; the reasons follow. |
| `robot ... has no joint group ...` | A joints part or `only:` names a group the profile's `joints.groups` lacks. |
| `step id ... is used more than once` / `replan_from ... is not a step of this plan` | Ids must be unique; `replan_from` names a step of the expanded plan (an include's inner steps are `<id>_<inner id>`). |
| `world state has no value for placeholder {name}` | The placeholder is not a parameter of the block, a fact or a world-state key; check the spelling and the `with:`. |
| An error naming a file and a key when a mission loads | The config schemas reject unknown keys and wrong types and name the file and the key; a newly added key needs the taskweave version that knows it. |
| `nothing is held: move to a hand target, then grip close` (REJECTED at run time) | A move of the held object needs a hold: go to the hand target with `move` (no offset), then `grip close`. |
| `has no parameter ...` / missing parameters of an `include` | `with:` must give exactly the block's `params`. |
