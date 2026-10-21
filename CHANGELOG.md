# Changelog

## 0.2.2 — 2026-10-21

### Torso chosen by IK
- `motion: {torso: auto}` searches torso heights (`ik.torso` profile keys): the current height first when its arm cost
  is acceptable (`accept_cost`), else outward over a grid; both hands at one height; the full robot collision-checked
  (idle arm included); a global-IK fallback at the nearest heights when no height solves locally; straight lines
  checked along their real path; a pre-grasp / pre-approach offset target also has to serve the line onto the point.
  Cost: arm cost + torso travel + optional wrist bend (`weights.wrist`). `data.torso` reports the choice.
- `move_object` with `motion: {torso: auto}` chooses one height for all holding hands; the hold follows the torso.
- Profile `torso: {joint, drive_m, tolerance_m}`: `navigate_to` and `drive_straight` bring the torso to the drive
  height first (left alone while holding).
- Galbot S1: `home` no longer sets the torso; `ik.torso` grid 0-0.75 m, `torso.drive_m` 0.494.

### Clock jumps (sim Stop + Play)
- One clock-jump helper clears the TF buffers of tcp_jog, teach, operator_joy and the skill server.
- `teach` refuses to save a point, stance or location while its TF is stale or right after a jump (the old age check
  went negative after a jump and let a pre-reset base pose through).
- Optional `clock_guard` node in `taskweave_bringup` (`clock_guard:=true`): resets Nav2 and restarts move_group after
  a jump.

### Fixes
- A folder without pack.yaml and without cell folders says "no cell pack here" instead of asking to migrate.

## 0.2.1 — 2026-10-20

### Fixes
- **HMI: planning stalls from the 3D view (PR #7).** The HMI asked MoveIt for the full planning scene every 10 s, which
  slowed arm moves that planned at that moment by up to 1.4 s. The scene is now fetched only while a 3D view asks, and
  the geometry only when the object names change.
- **Held objects with collision shapes on the Galbot S1:** `galbot_s1` lists its gripper links (`grippers.<hand>.links`)
  as touch links, so a held object's shape no longer collides with the fingers (robot packs version 0.2.1).
- **Version check:** `taskweave_version()` reads the running source tree's `package.xml` first, then the installed copy
  this file belongs to; an older installed taskweave no longer decides it.
- **A taken object** gets its own message: "'x' was taken in an earlier step or run (object state); reset the sim or run
  `taskweave state clear`", instead of the list of instances.

### New
- **Sim clock jump (Stop/Play):** one helper, `taskweave_core.clock_jump`, clears every node's TF buffer and calls
  reset hooks; `teach` refuses to save from stale TF or a pre-jump base pose (`max_tf_age_s`, `after_jump_s`); the
  optional `clock_guard:=true` bringup node resets Nav2 and restarts move_group after a jump.
- `taskweave state show|clear` for the run-time object state.
- The smoke report records how each step moved (`path` direct/planned, planner time, trajectory points) and names the
  path kinds when a step is out of tolerance.
- Install guide: the gaps a clean-host install of 0.2.0 found.

## 0.2.0 — 2026-10-20

### Breaking changes
- **Reload value renamed.** The catalogue `reload` value of `skills/*.yaml` is now `reload_service` (was `skill_server`).
  Any tool or saved file that used the old value must change.
- **`taskweave check` output changed.** One line per file (`ok`, `FAIL` or `warn`, with `file:line` and the message),
  then a summary: `N file(s) with problems`, or `config ok (skill arguments are checked against a running catalog)`.
  The old `ok    <file>` / `N problem(s)` lines are gone; scripts that parse them must change. Exit code 1 still means
  an error.
- **Packs need `requires: {taskweave: ">=0.2,<0.3"}`.** The example cell and the bundled robot packs now require it;
  a pack requiring another range is refused by this release (the error names both versions). Robot packs are version
  0.2.0.
- **New run-time dependency:** `python3-packaging` (`exec_depend` of `taskweave_core`). Install it on hosts that run
  the core from source.
- **Interfaces:** new `ReloadConfig` service and `RestartItem` message in `taskweave_interfaces` (additive).
- **Limitations of 0.1.0 resolved:** arm selection exists (`hands: any_one` in `move_gripper` and `grip`), so the
  "no arm auto-selection" limitation is gone; the other limitations of 0.1.0 still apply.

### HMI (PRs #1–#4, #6)
- New layout: tabs Run · Missions · Robot · Teach · Record · Log; Teach in sub-tabs (locations, stances, poses,
  points, carry poses); a new visual design with light and dark themes.
- Read-only 3D view of the robot model, the planning-scene collision objects, placed objects and points (Robot tab,
  and the cell view in Run).
- Speed strip in the top bar: global speed override and test mode.
- Mimic joints shown correctly in the robot model.
- Operator guide `docs/operator.md`: running missions, speed, hands, reload vs restart, checks, troubleshooting, and
  the HMI tabs.

### Robot packs
- New pack `galbot_s1_g1gripper` (Galbot S1 with the G1 gripper, PR #5).

### Arm selection and one-hand work (P3)
- `hands: any_one` for `move_gripper` and `grip`: a free, IK-feasible hand with the least motion; the profile order
  breaks ties; a repeatable hand ranks before one that needed the global IK search; a FAILED target (TF) is passed on.
- Free arm moves while the other hand holds: only motions of holding hands and their arms are refused. IK
  collision-checks the free arm against the other arms' current joints.
- A holding hand is not a reached hand in `grip close any_one`; `grip open` with several holding hands names them in an
  ambiguity.
- Motions within a tolerance of the least are equal, and the profile order decides. The probe counts the torso and the
  pose's fixed joints in `motion_rad`.
- `move_gripper` via-poses: a pose with nothing for the target hands adds no leg (was a crash); via legs take only the
  target's hands.
- `move_joints` on a holding gripper joint is refused like the arm joints.

### Held objects
- Held objects are collision objects in MoveIt's planning scene.
- A planned motion keeps the holding arms where they are; holds are followed however a motion ends, not only on
  success; `move_object`'s lift segment follows the other holds.
- Moving a held object drops the stale reach of hands that are not in the hold.
- A hold records the object type, so a taken single object can be placed (`place_as`) and moved again; the type is
  published only when known. `grip open` with `place_as` rejects when two holds would be released.
- A hold that cannot be registered (no TF) fails `grip close` instead of reporting holding.
- Object types may carry a free-form `sim:` key for the simulator integration.

### Config API
- **Catalogue** (`config/catalogue.py`): one table of every config file kind (pattern, schema, reload value,
  editability). `taskweave config files`, `config schema` (JSON Schema export), and `config validate <file>` (one file as on disk); problems are
  structured (`path`, `pack`, `where`, `line`, `col`, `message`, `severity`).
- **Save, delete, create, save_mission, set_robot** go through core: a stale base hash raises `SaveConflict`; errors
  write nothing; one lock per pack around compare, validate and write; atomic write keeping owner, group and mode;
  robot-pack edits are written as overrides into the cell pack; the written path is committed when the pack is a git
  work tree. Commits use umask 002 and warn about a shared repo without `core.sharedRepository`.
- **`config save`, `config history`, `config restore`** (CLI) use the same core calls; history and restore read git.
- **Init, export, import:** `taskweave config init <dir> --robot <pack>` (a cell pack from nothing, passes `check`);
  `config export <cell> f.tar.gz`; `config import f.tar.gz <cell> [--force]` (checked before anything changes; folder
  swap with `.bak-<time>`, or a commit in a git tree; pack lock and rollback on failure).
- **Reload without a restart:** `<node>/reload_config` on the task handler and every skill server;
  `taskweave config reload [--ros | --hmi URL]` prints what was reloaded and what needs a restart. Skill yaml parameters
  and `base.limits` apply live. Skills read their parameters in `configure`, split off `setup`.
- An invalid `system.yaml` is a problem of that file and can be fixed by a save; dry-run messages name the real files.

### Pack versions
- `requires: {taskweave: <PEP 440 specifier>}` in a cell or robot `pack.yaml`. Loading and `taskweave check` refuse a
  pack that does not admit the installed taskweave and name both versions; a missing `requires` is allowed.
- The example cell and the bundled robot packs require `>=0.2,<0.3`; `config migrate` writes robot pack version 0.2.0.

### Container image
- `Dockerfile` (ROS 2 Jazzy, all packages built from the checkout, entrypoint sourcing ROS and the install,
  `taskweave` on the PATH), `docker/compose.example.yaml`, `.dockerignore`, and a CI job that builds the image without
  pushing it. Nothing is published. See `docs/install.md`, "Container".

### CLI on PATH
- `taskweave` is a wrapper in `bin/` (installed through `data_files`), so it is on the PATH after sourcing the install.
  The console scripts stay in `lib/taskweave_core` for `ros2 run`. `python3 -m taskweave_core.cli` is unchanged.

### CI and test robustness
- The skills ROS tests are independent of the runner's load; wall-clock tolerances scale with
  `TASKWEAVE_TEST_TIME_SCALE` (CI: 4).
- Per-node singletons are dropped on `destroy_node`, with a guard against id reuse (also for `HoldCollision`).
- `test_pack_requires` cases live in their own folders (the pack.yaml mtime cache), and the shipped packs are tested
  against the package version.

### Fixes
- Record tab: Download all (the whole recording, bag included, as one tar); a stopped recording shows that it is
  finishing and Stop is disabled meanwhile.
- Save and delete hash the file again right before the write (no lost update).
- Import into a git tree: writable check first, rollback on failure.
- Review findings of the P3 and P4 reviews (`any_one` ranking, probe motion, hold reach, registry,
  `teach_then_drive_back` checks against the taught point, `test_move_follows_the_planned_path` records the detour).
- Docs: the skill step format in `docs/skills.md` and the taskweave-on-PATH notes in `docs/install.md`.

## 0.1.0 (2026-10-19)

First release. taskweave runs robot tasks on ROS 2 Jazzy, driven entirely by YAML. All six packages
(`taskweave_interfaces`, `taskweave_core`, `taskweave_skills`, `taskweave_hmi`, `taskweave_robots`,
`taskweave_bringup`) carry version 0.1.0. Install: [docs/install.md](docs/install.md).

### What it is
- **Planner, task handler, skills.** A planner turns a goal into steps (today a mission file), the task handler
  validates the plan, runs the steps and applies retry / replan / skip / abort, and skills do one step each behind the
  `RunSkill` action. Skills never name hardware: they reach it through the roles of a robot profile.
- **Config packs.** A *robot pack* holds one robot model (profile, embodiment, poses, navigation, MoveIt files). A
  *cell pack* holds one work cell (layout, objects, missions, skills, per-robot stances and points) and overrides a
  robot pack's files key by key (`~override: keys`), so one model serves many cells. `taskweave_robots` bundles
  `galbot_s1`, `galbot_g1` and `a2d` for internal use only (see each pack's `LICENSE-NOTE.md`). A self-contained
  example cell is in `docs/examples/cell`.
- **Skills:** `move_joints`, `move_gripper`, `move_object` (linear or free motion, grip-drift check), `set_posture`,
  `grip` (with a per-hand hold registry), `navigate_to`, `drive_straight`, `wait`. Reference: `docs/skills.md`
  (generated from the skill classes).
- **Global speed override** (1-100 %, with a test mode) scales every skill. Commanded speeds are capped by the robot's
  velocity limits. Nav2 drives are slowed by scaling the controller accelerations and, through a `cmd_vel` speed gate,
  the commanded speed; Nav2 time allowances stretch by 1/override. The Nav2 phase of `navigate_to` never drives below a
  configurable floor (`min_override`, default 0.3).
- **Web HMI** (`taskweave_hmi`): run (live status, pause / resume / abort, speed override), teach (save locations,
  stances, grasp points and poses from the live robot), robot (card, joints, tool jog), record (recordings of a run),
  missions (validated mission editor).
- **Config tools** (`taskweave` command): `check` (offline validation of a cell pack), `config migrate` (bring an older
  cell pack to this schema, atomic and re-runnable), `config migrate-skills` (schema 2 -> 3, the current skill model),
  `config slim` (whole-file overrides to per-key overrides), `config files` (where each robot file comes from),
  plus `skills`, `describe`, `run`, `status`, `pause`, `resume`, `abort`, `teach`.
- **Smoke test** (`taskweave smoke run | compare`): replays a mission in the simulator from a clean start and checks
  every step and its duration against a reference run (per-skill tolerances).
- **`taskweave_bringup`**: robot-agnostic launch of taskweave, the HMI, and optionally Nav2 and MoveIt, generated
  from the robot packs (`taskweave_bringup.launch.py`, `taskweave_nav2.launch.py`, `taskweave_moveit.launch.py`).
- **Engineering rules enforced by tests:** no robot, cell or object names and no tuning numbers in code
  (`test_agnostic.py`), schemas reject unknown keys, every schema change comes with a migration, CI runs ruff and
  `colcon build` / `colcon test` in `ros:jazzy-ros-base`.

### Known limitations
- Nav2 drives keep an override floor: the MPPI controller cannot creep, so a very low override does not slow a drive
  below `min_override`.
- `hands: any_one` works in `move_gripper` only.
- No arm auto-selection yet: a step names its hands.
- The HMI cannot edit every config file yet; the rest is edited as YAML.
- Offline `taskweave check` does not check point and contact names against the objects; that happens at plan
  validation on the running system.
- There is no VLM planner; only the mission-file planner exists.
- Package licences are still `TBD`, and the bundled Galbot / A2D packs are not for redistribution.
- Nav2 and MoveIt need maps, a robot and a clock from the application; the bundled launch files do not start a
  simulator or hardware.
