# Operator guide

For the people who run missions on a robot cell. You do not need to write code or read the source. You need the cell
pack of your cell (a folder), a running task stack (someone has installed and launched taskweave: [install.md](install.md)),
and, to type commands, a terminal where the workspace is sourced so that `taskweave` works. `taskweave --help` lists
the commands. Writing and editing missions is in [missions.md](missions.md); this guide is about running them.

The examples use the example cell of `docs/examples/` (robot `duo_bot`, a neutral test robot). Your cell has its own
names. One real cell for comparison: the Galbot S1 frame cell, a two-arm robot with a torso and a differential base.

## 1. Concepts

| Term | What it is |
|---|---|
| **Cell pack** | The folder of one work cell: where things stand (`cell/`), the kinds of objects (`objects/`), the missions (`missions/`), skill settings (`skills/`) and, per robot, its stances, held poses and object points (`robots/<robot>/`). It describes the cell, not the robot. |
| **Robot pack** | One robot model: how its hardware is reached, what it can do, its named poses, its MoveIt and Nav2 files. One robot pack serves many cells. |
| **Mission** | A list of steps in `missions/<name>.yaml` that reaches a goal, for example "move the parts from the stacks into the bin". It runs once or loops until the work is done (`loop`, `max_cycles`, `stop_when`). |
| **Step** | One line of the mission: a sentence (`instruction`), one skill with its arguments, and rules for what to do if it goes wrong (`timeout_s`, `retries`, `on_failure`). |
| **Skill** | One thing the robot can do, in operator terms: a verb. The task handler hands each step to the skill, the skill does it, and reports success or why not. Skills know the robot only through its pack, so the same step works on another robot. |

Shipped skills (arguments: [skills.md](skills.md)):

| Skill | What the robot does |
|---|---|
| `navigate_to` | Drives the base to a named location or to its stance at an object and stops there, facing the stored direction. With Nav2 it plans around obstacles; without it drives a direct path. |
| `drive_straight` | Drives the base a short distance forward or back (no obstacle avoidance), for example up to a rack and out again. |
| `move_joints` | Moves arms, torso or head to a named joint pose such as `home`, or to given joint values. |
| `move_gripper` | Moves one or both grippers (the tool centre points, TCP) to a point on an object, a named tool pose, or by an offset from where they are. |
| `grip` | Opens or closes the fingers. Closing at an object point makes the hand hold that object; opening lets go. |
| `move_object` | Moves the object the robot holds, with the holding hands locked to it: by an offset, to a held pose such as `carry`, or onto a contact of another object. |
| `set_posture` | Swings the elbows to a side while the hands stay where they are. |
| `wait` | Waits for a number of seconds (not slowed by the speed override). |

Four more terms you will meet in missions, messages and the teach tools:

- **Point.** A place on an object that a hand goes to, written `<instance>/<point>`, for example `part_stack_1/grasp`.
  It holds one tool pose per hand and is stored relative to the object, so it follows the object when the object moves.
  Points are taught (`taskweave teach point`) or edited in `objects/<type>.yaml`.
- **Stance.** Where the base stands to work on an object, written `stance:<instance>` (first variant) or
  `stance:<instance>/<variant>`. It is stored relative to the object, per robot. Locations (`home`, `staging`) are
  fixed base poses of the cell.
- **Held pose.** A named arm shape for carrying a certain type of object (`carry`), per robot.
- **Placed object.** An object instance of the cell (`part_stack_1`, `bin_1`, `fixture_1`), listed in `cell/objects.yaml`
  with its position (stacks also with their item count). While missions run, taskweave keeps a **run-time object
  state** on top of that file: items taken off a stack, objects gone, objects put down (`grip open` with `place_as`
  creates a new placed object whose points exist from then on). This state is what lets a looping mission finish when
  every stack is empty. It is not in the cell pack (section 7).

### The HMI

The task stack serves a web page, the HMI: `http://<host>:8090` in a browser on a laptop or a phone (port 8090 by
default; a second stack on the same host uses another port, for example 8091). There is **no login**: anyone who can
reach the address can start missions and move the robot. Keep it on a trusted network (the frame cell: the Tailscale
address of the robot host only), never on a public one.

The bar at the top shows the robot the stack drives, the pad's mode (`pad: drive`, or the arm it jogs) and a dot: green
while the page has a live connection to the stack, red when it lost it (the page reconnects by itself). A pad action
that changed something, such as a taught point, shows there for a moment.

| Tab | What it is for |
|---|---|
| **Run** | Start, pause, resume and abort a mission; its status, its plan step by step, and the robot live in the cell (3D). |
| **Missions** | Read and edit the mission files, checked before they are saved (writing missions: [missions.md](missions.md)). Move a step by dragging its handle (the dots left of its number, also by touch), with its ↑ ↓ buttons, or with Alt + ↑ / ↓ on the handle; a `replan_from` keeps pointing at the same step. |
| **Robot** | The robot now: its position, grippers and joints, in 3D with the room the planner keeps around it; below, the navigation and arm settings. |
| **Teach** | Locations, stances, poses, points and carry poses: where the robot drives to, stands and puts its hands. |
| **Record** | Record a run (bag, the simulator's videos) and download the recordings. Only there when the stack has a recordings folder. |
| **Log** | The step events of the last runs, newest last. |

While a mission runs, everything that changes points or settings is locked (the buttons are greyed out and a note says
why): teach and edit between runs.

## 2. Running a mission

Check first that the stack is up: `taskweave skills` lists the skills of the running system (an error or an empty list
means the task handler or skill servers are not running), and `taskweave status` shows `[idle]`.

### From the HMI: the Run tab

1. Pick the mission under **Start a mission** and press **Start**. The answer appears under it: `Started <plan id>`, or
   why it was refused (the same reasons as `rejected: ...` below).
2. **Pause** stops the robot where it is; the same button then reads **Resume**. **Abort** ends the mission. Both work
   like the commands below. A pad can do the same if the cell's `operator.yaml` maps buttons to it (the frame cell: options
   = pause / resume, share = abort).
3. **Start at a step:** before starting, open the plan (**List**) and tap a step: the mission's first cycle then begins
   there, with the robot and the objects as they are now. Tap it again to clear it. Use it to repeat the end of a
   mission after a fix, not as a shortcut through a mission whose first steps set things up.

The plan shows as a **Graph** (the steps and where a failure leads: retry, replan, skip, abort) or a **List**, each step
with its outcome once it ran. The 3D view below the status shows the robot where the stack believes it is, in the cell
with its fixtures, and the taught locations, objects and points. **Robot** frames the robot, **Cell** the whole cell,
**Follow** keeps the camera on the robot as it drives, **Labels** shows the names (greyed out while nothing is taught).
A note `no robot pose` means the stack has no position of the robot: the 3D view and every drive are off until it
has one again.

The **speed strip** in the top bar (on every tab) sets the global speed override and test mode, also while a mission
runs (section 3).

### From the command line

```bash
taskweave run restock          # start missions/restock.yaml; prints "started <plan id>" or "rejected: <why>"
taskweave status               # one line now
taskweave status --follow      # keep printing as the status changes (Ctrl+C to stop; it does not stop the mission)
taskweave pause                # stop the robot at once, keep the place in the mission
taskweave resume               # carry on
taskweave abort                # end the mission
```

`taskweave run` only starts the mission and returns. It exits 1 and prints `rejected: ...` if the mission does not
validate (section 7) or `a mission is already active; abort it first`. `--goal TEXT` overrides the goal text.

A status line reads:

```
[running] plan_3 cycle 2 step 4/9: s4 move_gripper "Pre-grasp 10 cm above the part" attempt 1 running 45% 12.3s
    last error: s4 (move_gripper) failed: ...
```

State is one of `idle`, `running`, `paused`, `succeeded`, `failed`, `aborted`. The step number counts within the
current cycle of the plan; `attempt` is the try of that step. `last error` stays until the next failure, so a
`running` line with an old error is normal after a retry that worked.

`pause`, `resume` and `abort` print `ok` or `not possible (state: ...)`, exit code 0 or 1. Pause cancels the running
skill (the robot stops where it is) and resume runs **the same step again from its start**: a half-finished arm move
is planned again from where the arm is. Abort ends the mission as `aborted`. The robot keeps whatever it holds
and wherever it stands; nothing is put away for you.

### What happens when a step goes wrong

Every step ends with one of these results; the message comes with it in the status and in the log.

| Result | What it means to you | What the handler does |
|---|---|---|
| `SUCCEEDED` | The step did what it said. | Next step. |
| `REJECTED` | The step was refused **before the robot moved**: wrong argument, a name that does not exist (point, stance, pose), the robot is holding something that stops it, nothing is held when something must be. A mission file or the cell config is wrong, or the world is not as the mission assumes. Trying again changes nothing. | No retries. It applies `on_failure` at once. |
| `FAILED` | The step started and did not get there: IK found no solution, a planner failed, the base made no progress, Nav2 did not reach the goal, the gripper closed on nothing, a position check missed. | Retries first (`retries`), then `on_failure`. |
| `TIMEOUT` | No result within `timeout_s` (plus a grace of 2 s). The handler cancels the skill. | Same as `FAILED`. |
| `CANCELED` | The skill was stopped by pause or abort. | Not a failure: pause waits, abort ends. |

Order of decisions: result is `FAILED` or `TIMEOUT`, so retry while `attempt <= retries`; when the retries are used up (or at
once for `REJECTED`), apply the step's `on_failure`:

| `on_failure` | Effect |
|---|---|
| `abort` | The mission ends as `failed`. The robot stops where it is. |
| `skip` | Logged, and the mission carries on with the next step. |
| `retry` | The step starts again with fresh retries. Counts against `max_replans`. |
| `replan` | The planner makes the plan again and the mission continues from `replan_from` (default the failed step), for example from the drive to the object. Counts against `max_replans`. |

When one step has used `max_replans` restarts, the mission ends as `failed` (`... max_replans (N) reached`). A failed
mission says why in `last error`, as `<step id> (<skill>) <result>: <message>`. Messages starting with `plan rejected:`
mean the plan did not validate before it started, nothing moved.

A `loop: forever` mission starts the next cycle after the last step; it ends as `succeeded` with `mission complete`
when the planner finds nothing left to do (`stop_when`, `max_cycles`, or no stack with items left).

### On screen

The status card at the top of the Run tab is the same information as `taskweave status`, made to be read from a few
metres away:

| On screen | Same as in the status line |
|---|---|
| The badge and the coloured rail: `IDLE` grey, `RUNNING` blue, `PAUSED` amber with stripes, `SUCCEEDED` green, `FAILED` and `ABORTED` red | The state in brackets. |
| The big line: the mission's goal, or `No mission running` | The goal. |
| `cycle n` at the top right | `cycle n`. |
| Under the goal: the step (`step 4/9 · move_gripper`), its instruction, a progress bar and the attempt and elapsed time | `step 4/9: s4 move_gripper "..." attempt 1 running 45% 12.3s`. |
| A red box under it | `last error`, as `<step id> (<skill>) <result>: <message>`. Like the status line, it stays until the next failure: after a retry that worked it is still there while the mission runs on. |

In the plan, each step that ran is marked with its result: `SUCCEEDED`, `FAILED` (the attempt that failed, then the
retry), `REJECTED`, `TIMEOUT`, `CANCELED`. A step marked `REJECTED` at once, before anything moved, means the mission or
the cell config is wrong (the message names the argument), not the robot. The Log tab lists the same step events with
their times, for reading back what happened after the run.

### The Robot tab

What the robot is doing now. At the top a 3D view of the robot alone, its joints live, with the room the path planner
keeps around it drawn on the floor: the chassis, the envelope (the robot in its driving pose, arms included), the
padding and the inflation zone, and an arrow to the front. **Fit** shows it from the front, **Top** straight down,
**Areas** hides the floor drawing. Below:

- **Robot pose:** where the base is in the cell (x, y, heading) and the placed object nearest to it.
- **Grippers:** each gripper's finger opening in mm, with **Open** and **Close**. They move at once.
- **Joints:** every joint against its limits (red within 10° of one), grouped by torso, head and arms. A value set here
  is sent at once and **without a collision check**: for getting an arm out of a place the pad cannot reach, not for
  normal work.
- **Settings** (closed by default): **Navigation** (the clearances and driving speeds of the path planner, applied to
  it at once; the floor drawing above follows your numbers while you type) and **Arm solutions** (which elbow side the
  robot prefers when several arm shapes reach a target).

### Teaching in the HMI

The Teach tab shows the robot's position at the top and one list at a time:

| List | What it stores | How to teach it |
|---|---|---|
| **Locations** | Base poses of the cell (`cell/locations.yaml`) | Drive the robot there (pad, or a mission step), **+ Location**, a name, **Set from robot**, **Save**. **Drive here** drives to a saved one. |
| **Stances** | Where the robot stands at a type of object, relative to it, per robot | In the card of the object type: drive the robot to where it should stand at one of the placed objects, choose that object, **Set from robot**, **Save**. Every placed object of the type follows. A per-object override is for the one object that needs another stance. The card's **Placed** view lists the placed objects (read-only: they come from `cell/objects.yaml`). |
| **Poses** | Arm shapes relative to the robot (`poses.yaml`), such as `home` | Put the arms there with the pad, **+ Pose**, a name, then **Capture from robot** (or the pad's L1 + L3). **Move here** moves the robot into it. |
| **Points** | Where a gripper goes on an object (a grasp, a place), relative to the object type | Choose at which placed object to teach (**at**: the nearest one by default), put the gripper there with the pad, **+ Point**, **Capture from robot** or L1 + L3. **Go here** drives the gripper there. |
| **Carry poses** | How the robot carries an object it holds (`move_object` `pose:`) | Typed in as joint values and waypoints; **Test** needs the robot holding such an object. |

An entry that is open is the one the pad's teach press (L1 + L3) goes to: open the one you mean before you press.
Every entry shows which missions use it; one that a mission uses cannot be renamed or deleted. **Numbers** shows and
edits the stored values, **Advanced** a point's elbow side, straight approach and finger opening. In a number field
↑ / ↓ (on a laptop also the small arrows at its right end) change a position by 1 mm and an angle by 1°, with Shift
by 10 mm or 10°; decimals are written with a dot in every browser language (a typed comma becomes one). **Delete**
needs a second tap. Saved values are used from the next move or mission start.

While a move started here runs (**Drive here**, **Move here**, **Go here**, a carry pose's **Test**), a bar in the top
bar shows what it goes to and for how long, on every tab. Its red **Abort** stops it at once with one press, as does
Escape (unless you are typing in a field); the entry then says `aborted`. It never stops a mission: that is the Run
tab's Abort.

### Recording a run

The Record tab records what happens, with or without a mission:

1. Choose a mission (or none: then it records what you do), and if you like a name and **Stop after cycles**.
2. Choose what to record: **ROS bag** (with **Camera images in the bag**, by far the largest part), **Videos** (the
   simulator's fixed and following views) and **From the start (reset the sim)**: the simulator is reset and the mission
   starts from the clean world, as the smoke test does.
3. **Start recording**. **Stop recording** ends it; the recording then shows **finishing** while the simulator encodes its
   videos and the camera videos are made from the bag. That takes minutes for a long run, Stop is greyed out
   meanwhile, and the recording is not complete until it says it has ended.

Each finished recording opens to its videos (to play in the page) and its files. **Download all (.tar, size)** saves the
whole recording, bag included, as one file into the browser's downloads; single videos and data files download one by
one from their links. **Keep** protects a recording from being replaced: by default a new recording replaces the older
ones, and the tab says how many are kept and how much disk is free. A recording that runs out of disk space is refused
before it starts.

## 3. Speed

**Standard speed** is what the cell was set up for: the speeds and accelerations in `skills/<skill>.yaml`, the robot
pack's `skill_overrides.yaml` and the point's own data. At 100 % the robot moves exactly like that.

| Control | Where | Effect |
|---|---|---|
| Per-step speed | `motion: {speed: 0.5}` in the step (fraction of the skill's standard speed, 0 to 1); `grip` has its own `speed` | Slows that step only. Use it for a careful approach. |
| **Global override** | 1 to 100 %, one value for the whole robot | Scales every skill, on top of the step's own speed. 50 % on a step with `speed: 0.5` is 25 % of standard. Applies at once to a step that is running; lowering takes effect immediately, raising ramps up over about half a second. |
| **Test mode** | A switch with a start value (default 10 %) and a cap (default 50 %) | Run starts: if the override is above the start value it is lowered to it. While test mode is on, the override cannot be set above the cap (it is clamped, with a note). Run ends: the override goes back to what it was, but only if test mode lowered it and nobody changed it since. |

The override belongs to the task handler and is not saved: after a restart of the task handler it is 100 % and test
mode is off. Defaults for the start value and the cap are the handler parameters `speed_test_percent` (10) and
`speed_cap_percent` (50).

**In the HMI**: the speed strip in the top bar shows the override in effect. The presets (10, 50, 100 %), the slider
(1 to 100 %) and − / + (5 % steps) set it; **Test mode** switches test mode on and off and shows the cap while it is
on (the strip turns amber). When the handler clamps or refuses a value, its note shows under the strip. The strip is
greyed out while the task handler is not reachable.

**From a terminal**:

```bash
ros2 service call /taskweave/task_handler/set_speed_override taskweave_interfaces/srv/SetSpeedOverride "{percent: 50}"
ros2 service call /taskweave/task_handler/set_speed_override taskweave_interfaces/srv/SetSpeedOverride \
    "{set_test_mode: true, test_mode: true, test_percent: 10}"
```

`percent: 0` means "leave unchanged". A value outside 1-100 is refused with a message. Use test mode for a first run of
any new or changed mission, with the people and the cell in the state you want when it goes wrong.

Both ways set the same value in the handler: what you set from the terminal shows in the HMI's strip, and applies to
runs started from either.

**Timeouts scale with the speed.** A step's `timeout_s` is counted in *work time*, not wall time: time passes at the
effective override. At 10 % a step gets ten times as long; at 100 % it is exactly the stated time. Retries and
`max_replans` are not scaled. `wait` is not slowed by the override, so its seconds are real seconds.

**Base floor for Nav2.** The Nav2 part of `navigate_to` never drives below a floor, because Nav2's controller cannot do
the fine final approach into a stance at 10 %. The floor is `min_override` in `skills/navigate_to.yaml` (default 0.3,
that is 30 %; `0` switches the floor off). In test mode at 10 % the arms and torso move at 10 % while Nav2 drives at 30 %.
The direct controller that takes over for the last metres, `drive_straight`, the arms, the torso and the grippers
follow the real override. Each `navigate_to` result carries the value used as `data.speed.nav2_override`. If 30 % is
too fast for a first run, lower the floor in the skill file, `taskweave config reload`, and expect worse stance
accuracy; do not switch it off for production.

## 4. Hands

Skills that act with the grippers take `hands`:

| Value | Meaning |
|---|---|
| a hand name (`left`) or a list (`[left, right]`) | Exactly these hands. |
| `all` | Every hand of the robot. |
| `auto` | The hands the target defines (a point with both a `left_tool` and a `right_tool` pose uses both), or for a held object the hands that hold it. The default of `move_gripper` and `move_object`. |
| `any_one` | One hand, chosen by the robot (below). |

**What `any_one` chooses** (`move_gripper`): only a free hand that can apply to the target (for a point: a hand with an
`alone` pose or a pose in the pair); among those, hands for which IK has a solution come first, then the hand with
the least joint motion from where the arm is now, then the first hand in the robot profile, so the choice repeats.
`grip any_one` takes the one hand that is at an object point (closing) or the one hand that holds something (opening);
with none or both it is rejected as ambiguous, with `hands any_one is ambiguous: left and right ...: name the hand`.

**Seeing which hand it chose.** The step's message names the hand, and the step result data has `hands` (the hand used)
and `choice` (every candidate with its joint motion, or the reason it was ruled out). In the status line the step shows
as `move_gripper "..."` so use the log or the HMI step details to read the result data. If no hand can take the
target the step is `REJECTED` with `hands any_one: no hand can take it (...)`, one reason per hand.

**Holding rules.** Closing hands that are at an object point makes them hold that object; opening takes them out
of it; an object with no hand left is released. While a hand holds:

- That arm is locked to the object. `move_gripper` for it and `move_joints` on its arm joints (also inside a `via` or a
  whole-body pose) are `REJECTED`, with a message naming the holding hand. Open the hand first (`grip open`) or move the
  object with `move_object`, which keeps every holding hand locked to it.
- The pad jogs the object, not the arm: a stick on a holding hand's tool moves the held object (the same axes as the
  arm jog; the object turns about its own origin) and every holding hand follows, keeping its grip. Two hands step
  together or not at all: if one hand reaches a limit or its reach, or the step would collide, neither moves (the HMI
  notice says why). The elbow trigger still swings that arm's elbow with the object fixed. Opening the gripper ends the
  hold and the tool jogs the arm again. The jog state is on `/taskweave/jog/state`; a skill that uses the held object
  re-reads its pose from the hands first, so a jogged object is where it really is.
- The free arm can move: `move_gripper`, `move_joints` of its own arm group and `grip` work. Torso and head may move too; the
  hold follows them.
- The free arm's path is planned without the held object, so the object itself is not avoided.
- `move_object` with nothing held is rejected with `nothing is held: move_gripper to the object, then grip close`.

## 5. After a config change

Edit the file, validate it (section 6), then make the running system read it. `taskweave config reload` calls the
`reload_config` service of the task handler and of every skill server, and prints per process what was reloaded and what
still needs a restart:

```bash
taskweave config reload                       # every file that changed since the process last loaded
taskweave config reload missions/restock.yaml # only these files (paths relative to their pack)
taskweave config reload --hmi http://<host>:8090   # the same through the HMI, where ROS is not reachable
```

```
/taskweave/task_handler: reloaded
  reloaded      cell/locations.yaml
  needs restart navigation.yaml: the driving envelope is also laid into the Nav2 launch
```

Exit code 1 means a process answered `busy` (a mission or a skill goal is active: wait or abort, then try again), `failed`
(a changed file does not load, nothing was applied: fix it, validate, again) or no process answered at all. A file that
needs a restart is listed on every call until the restart.

| File | Effect of a change |
|---|---|
| `missions/*.yaml` | Read when the next mission starts. Not between steps or cycles of the mission that is running. |
| `cell/locations.yaml`, `cell/objects.yaml`, `objects/*.yaml`, `robots/<robot>/stances.yaml`, `held_poses.yaml`, `object_points.yaml` | Live: read on every use, effective at once (also in a running mission, from its next step). |
| `poses.yaml`, `skill_overrides.yaml` of a robot (robot pack, or the cell's copy under `robots/<robot>/`) | Live. |
| `tests/*.reference.yaml` | Live (read by the smoke test only). |
| `skills/<name>.yaml`: parameters (speeds, tolerances, `min_override`, implementation `params`) | `taskweave config reload`, between goals; the next goal uses them. |
| `skills/<name>.yaml`: the implementation `name` or `class`, or a parameter a connection is made from (topics and action name of the Nav2 skill) | Restart the skill servers (the frame-ros / task stack container). |
| `navigation.yaml` | Driving limits are live, but the envelope is also laid into the Nav2 launch: restart the task stack for Nav2 to see it. |
| `profile.yaml`, `embodiment.yaml`, `operator.yaml`, `system.yaml`, `pack.yaml` (cell or robot) | Restart the task stack: read at launch (profile `base.limits` alone reload with the skill servers). |
| `moveit/**`, `nav2_params.yaml` | Restart the task stack: MoveIt and Nav2 read them at launch. |
| `system.yaml` keys the simulator reads, `cell/layout.yaml`, `sim/scene.yaml`, `sim.yaml`, `pose_targets.yaml` of a robot | Restart the simulator; taskweave does not read them. |
| Handler parameters `speed_test_percent`, `speed_cap_percent` | Restart the task handler. (The override itself is live.) |

"Restart the task stack" is the container or launch that runs the task handler, skill servers, HMI, Nav2 and MoveIt (the frame cell:
`frame-ros`). "Restart the simulator" is the process that simulates the robot (the frame cell: `frame-sim`). The
`reload` column of `taskweave config files <cell> --json` gives the same values per file: `live`, `mission_start`,
`reload_service` (`taskweave config reload`), `launch` (restart the task stack), `sim` (restart the simulator). **Do not
restart anything while a mission or a recording is running**: check `taskweave status` is `idle`, `succeeded`, `failed` or
`aborted` first, and ask whoever shares the robot. A restart of the simulator also resets the world (section 7).

The HMI has no general config editor yet. It edits missions (Missions tab), the taught points (Teach tab) and the
navigation and arm settings (Robot tab), each checked before it is saved; every other file is edited as text and
reloaded as above.

## 6. Checks

```bash
taskweave check /path/to/cell                  # every file of the cell pack and its robots, offline, no ROS needed
taskweave config validate missions/restock.yaml --cell /path/to/cell   # one file, as it is on disk
taskweave describe /path/to/cell <robot>       # the robot card and the skills it can run
```

- **`taskweave check <cell>`** lists every file as `ok`, `warn` or an error, and exits 0 only if the pack is valid. It
  loads each file against its schema, checks that profile and embodiment agree, that each skill file loads and each
  mission expands (includes, parameters) into a plan. It does **not** check the arguments of a step or the names it
  uses: a misspelt point, stance or pose passes `check` and is found when the mission starts (the live catalogue
  rejects it with the step and the argument) or at run time. Run it after every edit and before every mission after
  a pack was changed.
- **`taskweave config validate PATH`** checks one file the same way and also against the rest of the pack (point, stance and skill
  names, mission ids, `replan_from`, includes). A relative path needs `--cell` (and `--pack robot:<name>` for a robot pack
  file); `--json` gives machine-readable problems with line and column.
- **Smoke test.** A regression test of a whole mission, run in the simulator from a clean start, compared with a reference
  run you saved when the mission was known good. It needs the HMI's recording interface, so it runs only where a simulator is
  attached, and only while nobody else is using the robot (it resets the world and takes minutes).

```bash
taskweave smoke run restock --hmi http://<hmi>:8090 --save-reference tests/restock.reference.yaml   # once, when good
taskweave smoke run restock --hmi http://<hmi>:8090 --reference tests/restock.reference.yaml
```

  The table has one row per step: step, instruction, `ref s` (duration in the reference), `now s` (duration now, simulator
  time), `Δ%`, `error` (the largest position error the skill reported) and `result`. A row passes when the step ran in the
  reference's order, ended `SUCCEEDED`, took no more than the reference plus 10 % plus 1 s (25 % for `navigate_to`), and its
  error is at most the reference's plus 10 % plus 0.002. Missing steps and steps that are not in the reference fail.
  Exit code 0 passed, 1 failed, 2 could not run (HMI unreachable, recording refused). Read a failure from the first
  failing row: `FAILED`/`TIMEOUT` is a mission or robot problem, a `Δ%` far above the limits on every row means the
  simulator or the machine is slow (compare under the same load), one row alone means that step changed. A report
  is written next to the reference (`<name>.report.yaml`). Re-record the reference when you change a mission on purpose.
  Without `--reference` the run passes when every step succeeded.
- **Release check.** The generic checks are the tests of the repository. A cell usually adds its own: the frame cell has
  `frame_sim/tools/release_check.sh`, a script that its maintainers run before handing a version to the floor. It is
  specific to that cell (its containers, its missions) and is not part of taskweave: if your cell has such a script,
  run it the same way and read what it prints; if it does not, `taskweave check`, `taskweave config validate` on the
  files you changed and the smoke test are the minimum.

## 7. Troubleshooting

Read `taskweave status` first: the state and `last error` say which row below applies.

| Symptom | Likely cause | What to do |
|---|---|---|
| The robot does not move, status `idle` | No mission started, or `taskweave run` was rejected. | `taskweave run <mission>` and read its answer. `rejected: a mission is already active; abort it first` means another run: `taskweave abort`. |
| `taskweave run` or `status` hangs or says no service | The task handler is not running, or your terminal is on another ROS domain. | Check `ROS_DOMAIN_ID` is the robot's, the workspace is sourced, and the task stack is up (`taskweave skills`). Do not start a second task handler. |
| `rejected: ...` right at the start, or `plan rejected: ...` | The mission refers to something that does not exist (skill, point, stance, pose, mission id) or has a wrong argument. The message names the step and the argument. | Fix the mission or the cell file named; `taskweave check`; `taskweave config reload`; run again. |
| `skill 'x' is not in the catalog` / `no skill server offers it` | The skill is not served by this stack, or its server is down. | The skill must be in the launch `skills:` list and have a `skills/<name>.yaml` in the cell. Restart the task stack after fixing. |
| Running, but the robot stands still | A step is waiting: a `wait`, a very slow override (an override of 1-5 % with the timeouts scaled up), or the skill server waits for MoveIt/Nav2. | Look at `step` and the elapsed time in `status`. Check the speed override (section 3). Paused? State `paused` needs `taskweave resume`. |
| Step `REJECTED` with `unknown object point '...': expected <instance>/<point>, instances in cell/objects.yaml: ...` | A point is written without its object (`grasp` instead of `part_stack_1/grasp`), the instance name is misspelt, or the instance is not there any more: taken away in the run-time object state or never created (a `place_as` step did not run). The list in the message is what exists now. | Fix the spelling or the mission order. If an object should exist but the list lacks it, the object state is out of date: see "sim was reset" below. `'<name>' was taken in an earlier step or run (object state: state/objects.yaml)` is a single object that was picked, or a stack at count 0: `taskweave state show` lists what is taken, `taskweave state clear` puts it all back (or reset the sim). |
| Step `FAILED` with `no IK solution for ...` | The target is out of reach from the robot's current stance, torso height or elbow side, or a wrong stance. | Check the stance (is the base where it should be?), re-teach the point, try `motion: {torso: auto}` or a different `elbow`. Not a MoveIt outage. |
| `no IK service /compute_ik (is MoveIt running?)`, `no FK service /compute_fk`, `collision check (/check_state_validity) did not answer`, `planning service ... did not answer` | MoveIt is not running, still starting, or on another ROS domain. | Check the MoveIt process is up (`ros2 service list \| grep -e compute_ik -e check_state_validity`, same `ROS_DOMAIN_ID` as the stack). Wait a minute after a launch. If it is still missing, restart the task stack (check status first). A probe from a terminal must use the live domain. Services such as `/check_state_validity` only know the robot's model joint names, not base or wheel joints. |
| `Nav2 is not running (no ... action server)` | Nav2 is not up or the map is missing. | Check the Nav2 launch part of the stack; restart the task stack. |
| `Nav2 did not reach <target>: ... (1.20 m away)`, or `no progress for N s, X m to go` | Nav2 is stuck: something blocks the route, localisation is wrong, or the stance is unreachable. `no progress` is the direct controller that takes over at the end. | Check the area is clear and the robot's position is right (HMI map or `taskweave status`). With `retries: 1` the retry often works. If it recurs, the stance or the Nav2 settings need attention: tell the person who maintains the cell. |
| Nav2 drives slowly or "creeps" at a low override | By design: at test mode 10 % Nav2 drives 30 % (the `min_override` floor), the arms 10 %. Timeouts are scaled by the override, so slow is expected. | Raise the override for the drive, or accept it. If the step times out anyway, raise `timeout_s` of that step. If it stops short, see Nav2 above. |
| After a simulator Stop/Play (or restart) the robot does not move, TF_OLD_DATA warnings, `no IK solution` with the target far ahead | Sim time went back. A TF buffer that still holds the old run's transforms drops the new ones as old data and answers with the old pose; Nav2 and MoveIt keep theirs too. taskweave's own nodes clear their buffers at the jump (log line `clock jumped by ... s: TF buffers cleared`), Nav2 and MoveIt do not. | Wait a few seconds, then retry. `teach` refuses to save for `after_jump_s` after a jump and when its TF or base pose is older than `max_tf_age_s` (the message says which): retry once the robot publishes again. Start the stack with `clock_guard:=true` (`taskweave_bringup` launch argument; node names and timing in a YAML given as `clock_guard_params:=`): it resets the Nav2 lifecycle nodes and restarts move_group after a jump. Without it, restart Nav2 and MoveIt (check `taskweave status` first), and abort a running mission: it cannot continue across a reset. |
| The simulator was reset or restarted and the objects are not as before | A reset ends every hold (open hands) and clears the run-time object state, so stacks are full again and moved objects are back. The robot is back at its start pose. | Start the mission from the beginning (the robot must be where the first step expects it). Nothing to repair, but a mission that was interrupted halfway does not resume across a reset. The state file is `objects.yaml` in `$TASKWEAVE_STATE_DIR` (default: a `state/` folder next to the cell pack folder); `taskweave state show [CELL_DIR]` prints it, `taskweave state clear [CELL_DIR]` deletes it (the same as deleting the file). Running processes re-read the file on their next use (by modification time), so no reload or restart is needed; `state clear` does not end a hold, a sim reset does. |
| A held object was lost (dropped, or the sim was reset), and `move_object` says `nothing is held: ...` | The hold registry says the hands are open. Reasons: the grip failed, the hand was opened, a sim reset. After a drop the robot does not know. | `taskweave abort`. Bring the robot to a safe pose (`move_joints home` from a one-step mission), put the object back by hand or reset the cell, then start the mission from the beginning. `... where is the held object now?` means the position of the held object could not be read: same steps. |
| `move_gripper` / `move_joints` `REJECTED` naming a holding hand | That hand still holds an object. | `grip open` (with `place_as` if the object is put down) before moving that arm, or use `move_object`. |
| Disk full, or the logs are huge, on the host that runs the containers | Container logs grow without limit by default (`docker logs` of a stack that prints status and warnings for days), plus recordings (bags, videos) and the smoke test's `--bag` / `--video`. | `df -h`, `docker system df`, then delete recordings you no longer need. Cap the logs: in the compose service `logging: {driver: json-file, options: {max-size: "50m", max-file: "3"}}`, or `docker run --log-opt max-size=50m --log-opt max-file=3`; it applies to a container when it is created again, not to a running one. Do not delete a running container's log file by hand. |
| CI (or a slow machine) fails with timeouts but passes again on a rerun | The tests use real nodes and fixed deadlines; a slow runner misses them. | Set `TASKWEAVE_TEST_TIME_SCALE` to a number above 1 (CI uses 4), which stretches the skill tests' deadlines: `TASKWEAVE_TEST_TIME_SCALE=4 python3 -m pytest -q test`. A test that fails at 4 as well is a real failure. |

When you report a problem, send: the mission and the step id, the `taskweave status` line with `last error`, what the
robot was doing, and the time. The log of the task handler and skill servers has the same lines with more detail.
