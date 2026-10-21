# Migrating a cell pack to targets (schema 3 to 4)

taskweave 0.3 reads cell packs of schema 4 and robot packs of schema 2: every place the robot goes to is a
[target](targets.md), and the six motion skills are one skill, `move`. A pack from an older release is converted by one
command. This page says what it does, what it prints and how to go back.
(The earlier move, from one config directory to cell pack and robot packs, is in [pack_switchover.md](pack_switchover.md).)

## Run it

```bash
taskweave config migrate /path/to/cell --dry-run     # print what would happen, change nothing
taskweave config migrate /path/to/cell               # do it
taskweave check /path/to/cell --missions             # every file and every mission's plan
```

- `migrate` takes a pack of any older schema up to the current one (1 to 2, 2 to 3, 3 to 4). Schema 3 to 4 is the new
  step.
- Everything is built in memory first and checked. Then each file is written next to its place and renamed. `pack.yaml`
  is written last, so an interrupted run leaves the pack at the old schema and the next run finishes the job.
- A second run changes nothing: `cell pack schema 4, nothing to do`.
- Comments and key order stay. Step ids, instructions, `include`, `with`, `defaults`, `facts` and `params` stay.
- An old taskweave refuses a schema 4 cell (`update taskweave`), and taskweave 0.3 refuses an older cell
  (`run taskweave config migrate <dir>`).

Robot packs are migrated too: the robot packs the cell uses (found through `system.yaml` `robot_packs`,
`TASKWEAVE_ROBOT_PACKS`, `--robot-packs DIR`) when they are writable. An installed read-only pack is only reported
(`FLAG ... migrate it where it comes from`). A robot pack, or a folder of them, can also be migrated on its own:

```bash
taskweave config migrate /path/to/robot_pack_or_folder_of_packs [--dry-run]
```

It turns `poses.yaml` into `targets.yaml` and sets `schema_version: 2`. The cell step reads the old robot files to learn
which poses are joint poses and which place tools, so run the cell first, or both in one go.

## What changes

| Old | New |
|---|---|
| `cell/locations.yaml` | `cell/targets.yaml`, each entry `moves: base` |
| `robots/<r>/stances.yaml` | `robots/<r>/objects/<type>.yaml`, `targets:` (`moves: base`), `instances:`, and `handled_from` = the old first variant |
| `robots/<r>/object_points.yaml` | the settings (`elbow`, `grip_m`) on the same `targets:` of `robots/<r>/objects/<type>.yaml` |
| `objects/<type>.yaml` `points:` | `targets:`, each `moves: hand` |
| `objects/<type>.yaml` `contacts:` | `targets:`, each `moves: object` (with a `reference` where a step named one) |
| `robots/<r>/poses.yaml` | `robots/<r>/targets.yaml`: joint poses `moves: joints` with group values, tool poses `moves: hand` |
| `robots/<r>/held_poses.yaml` | the same `targets.yaml`: `moves: object`, `type`, `path` |
| `skills/{navigate_to,drive_straight,move_gripper,move_object,set_posture,move_joints}.yaml` | `skills/move/{base,straight,hand,object,posture,joints}.yaml` (same content) and a new `skills/move.yaml` |
| `robots/<r>/skill_overrides.yaml` keys | renamed to `move.base`, `move.hand`, ... |
| mission steps and `operator.yaml` actions | `skill: move`, see below |
| `pack.yaml` | `schema_version: 4` |

Tool poses written as `tcp`, `approach` and `closing` become `xyz` and `rpy`. The migration rebuilds the rotation matrix and
checks it against the old one (1e-6). Joint poses name groups instead of joints; a pose joint that is in no group of the
profile stops the migration and is named.

### Steps

Each step is rewritten as a `move` step. Examples from the example cell:

| Before | After |
|---|---|
| `navigate_to {target: "stance:part_stack_1"}` | `move {to: part_stack_1/pick}` |
| `navigate_to {target: staging}` | `move {to: staging}` |
| `drive_straight {to: "stance:fixture_1/work"}` | `move {to: fixture_1/work, motion: {mode: straight}}` |
| `drive_straight {distance_m: -0.5}` | `move {base: {by: [-0.5, 0]}}` |
| `move_gripper {to: {point: part_stack_1/grasp, offset_m: 0.1}}` | `move {to: part_stack_1/grasp, offset_m: 0.1}` |
| `move_gripper {to: {pose: present_both}}` | `move {to: present_both}` |
| `move_object {to: {by: [0, 0, 0.2]}}` | `move {object: held, by: [0, 0, 0.2], torso: keep}` |
| `move_object {to: {pose: carry}}` | `move {to: carry, torso: keep}` |
| `move_object {to: {onto: bin_1/seat, contact: base}}` | `move {to: bin_1/seat, torso: keep}` and `reference: base` on the target |
| `set_posture {elbow: out}` | `move {elbow: out}` |
| `move_joints {pose: home}` | `move {to: home}` |

- `stance:<instance>` without a variant becomes `<instance>/<first variant>`: the stance that was the default.
- **`torso: keep`** is written on every object step that did not say `motion.torso: auto`. `move_object` used to keep the
  torso and `move` chooses it for hand and object targets, so this keeps the robot doing what it did. Remove them one by
  one, with the smoke test as judge.
- A contact that steps used becomes the `reference` of the target when every step names the same one.
- Run-time objects: a step such as `"stance:{frame}"` with a placeholder for an object a mission creates at run time
  (`place_as`) gets the stance name every other value of the same `params` entry maps to, and a `FLAG`.
- Skill names in the smoke reference file are not rewritten: the smoke test reads old skill names as aliases of
  `move.<kind>` ([skills.md](skills.md)).

## What it prints

One line per change, with a word first:

| Word | Meaning |
|---|---|
| `rename` | A name clashed once the kind prefix was gone, and one of them got a new name. Steps follow the new name. |
| `move`, `rewrite`, `write`, `remove` | A file moved, rewritten in place, newly written, removed. |
| `held` | A held pose became an object target; says whether it got a `type`. |
| `drop` | A joint twin (`<pose>_joints`) that nothing used was dropped. |
| `step`, `action` | A mission step or `operator.yaml` action: old form, then new form. |
| `param` | A mission parameter's point kind became `target`. |
| `comment` | A comment still names an old file or skill. Listed, not edited. |
| `FLAG` | Something was not mapped cleanly: check it by hand. Printed again at the end. |
| `note` (dry run only) | What a real run would stop at, for example a file that cannot be written. |

Example, the example cell before schema 4, with `--dry-run` (shortened):

```
cell pack schema 3 -> 4 (dry run: nothing is changed)
rename  pose duo_bot home (clashes with the location home) -> home_pose
move    cell/locations.yaml -> cell/targets.yaml  (2 base target(s), moves: base)
rewrite objects/part.yaml  (points, contacts -> targets: grasp, base)
write   robots/duo_bot/objects/part.yaml  (stances pick)
FLAG    robots/duo_bot/object_points.yaml box rim: dropped, the settings are for no point (box is no object type; check warned)
held    duo_bot carry -> carry (moves: object, no type: the same for part, bin)
step    pick_and_place to_part: navigate_to {target: 'stance:part_stack_1'} -> move {to: part_stack_1/pick}
move    skills/navigate_to.yaml -> skills/move/base.yaml
comment objects/part.yaml:4: grasp: # taught with `taskweave teach point <instance>/grasp`
would write 21 file(s), would remove 11; 1 flag(s); 15 comment(s) name an old file or skill
```

A migration that cannot map something at all stops before it changes anything and names the file or step. Typical causes:
a pose joint that is in no joint group, a step that names a stance nobody can resolve, a file that is not writable (or a
link out of the pack).

### Name clashes

A reference has no prefix any more, so a name has one meaning in its place ([targets.md](targets.md)). When two entries
with one name end up in the same place, the one that is not the older concept is renamed:

| Clash | New name |
|---|---|
| a stance named like a hand or contact target of the same object type | `<name>_stance` |
| a held pose named like a plain target | `<name>_held` |
| a pose named like a cell location | `<name>_pose` |
| a contact named like a point of the same type | `<name>_contact` |

Every rename is printed and every step that used the name follows it. In the example cell the joint pose `home`
clashes with the cell location `home`, so the pose is `home_pose`.

## After the migration

1. `taskweave check /path/to/cell --missions` must say `config ok`: zero errors, zero warnings is the goal.
2. Read the `FLAG` lines. Each one names a file and what to decide.
3. Read the `comment` lines: comments that still say `stances.yaml` or `move_gripper` are not edited, change them when you
   next touch the file.
4. Run a mission on the simulator, or the smoke test, before the real robot. Launch with `skills:=wait,move,grip`
   (README, Usage).
5. Commit the result as one commit (`git add` the files, not `-A` if others work in the tree).

## Roll back

The migration moves files with their history, so one `git revert` of the migration commit undoes it:

```bash
git -C /path/to/cell revert <migration commit>
```

Then run the taskweave version the pack had (0.2.x) and restart the task stack. Without git, keep a copy of the pack
before the first run (`taskweave config export CELL_DIR cell.tar.gz` writes one file); `import` restores it.

New config keys (`targets:`, `moves`) are unknown to taskweave 0.2, which refuses to load them. So update taskweave first,
restart the stack, then migrate.
