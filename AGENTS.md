# AGENTS.md: working on taskweave

For AI agents and new developers. Users of taskweave start with [README.md](README.md).

## What this is
taskweave runs robot tasks on ROS 2 Jazzy, driven entirely by YAML.
- A **planner** turns a goal into steps (today: a mission file; later: a VLM).
- The **task handler** validates the plan, runs the steps, applies retry / replan / skip / abort and publishes status.
- **Skills** do one step each behind the `RunSkill` action and address the robot through roles of its profile.
- **Config** holds every name, point, number and rule: a **cell pack** (the work cell) plus **robot packs** (one per model).
- A global **speed override** (1-100 %, test mode) scales every skill; a **smoke test** replays a mission against a reference.

## Repo map
| Path | Contents |
|---|---|
| `taskweave_interfaces/` | msgs, srvs, `RunSkill` action (CMake package) |
| `taskweave_core/taskweave_core/` | `config/` (store, schemas, packs, overlay, catalogue, check, migrate*, slim), `planner.py`, `executive.py`, `task_handler.py`, `skill.py` (base class), `skill_server.py`, `validation.py`, `motion.py` (motion option resolution), `speed.py`, `speed_gate.py`, `smoke.py`, `teach.py`, `operator_joy.py`, `cli.py` |
| `taskweave_skills/taskweave_skills/` | the generic skills: `basic` (wait), `navigate`, `drive`, `joint_motion`, `gripper_motion`, `object_motion`; helpers `joints`, `ik`, `chain`, `cartesian`, `motion_plan`, `hold`, `points`, `tcp_jog`, `nav2` |
| `taskweave_hmi/taskweave_hmi/` | web HMI (`server.py`, `web/*.js`): status, control, mission editor, teach, record |
| `taskweave_bringup/` | launch files `taskweave_bringup.launch.py` (taskweave, HMI, optional Nav2/MoveIt), `taskweave_nav2.launch.py`, `taskweave_moveit.launch.py`; `taskweave_bringup/` (`nav2.py`, `moveit.py`, `hmi.py`, `robot_files.py`: the logic, testable without launching), `config/nav2_params.yaml` (default Nav2 tuning) |
| `taskweave_robots/<robot>/` | robot packs (`pack.yaml`, `profile.yaml`, `embodiment.yaml`, `poses.yaml`, `moveit/` ...), installed to `share/taskweave_robots/` |
| `docs/` | `install.md`, `architecture.md`, `configuration.md`, `missions.md`, `skills.md` (generated), `pack_switchover.md`, `examples/` |
| `tools/gen_skills_doc.py` | generates `docs/skills.md` (needs ROS sourced, see "Add a skill" step 6) |
| `*/test/` | pytest suites per package; config fixtures in `*/test/fixtures/config` |

## Build
A user's install from a tag (clean machine, example cell, own packs, upgrade) is in [docs/install.md](docs/install.md); keep it working when you change packages, launch files or the CLI.
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/tw_ws/src && cd ~/tw_ws/src && git clone https://github.com/flrzep/taskweave.git
cd ~/tw_ws && rosdep install --from-paths src -y --ignore-src
colcon build && source install/setup.bash
```
After sourcing the install, `taskweave` is on the PATH (`taskweave_core/bin/taskweave`, a wrapper installed to `bin/`;
the console scripts themselves sit in `lib/taskweave_core` for `ros2 run`). `python3 -m taskweave_core.cli` is the same command.
`taskweave_skills` needs MoveIt messages (`moveit_msgs`) on the system; the task handler needs `jsonschema`, `pyyaml`,
`numpy`.

## Run the tests
Each package has a `test/` folder. The ROS tests start real nodes, so:
```bash
source /opt/ros/jazzy/setup.bash && source ~/tw_ws/install/setup.bash
export ROS_DOMAIN_ID=77                       # never the domain of a live robot
export TASKWEAVE_ROBOT_PACKS=$PWD/taskweave_robots   # the robot packs of this checkout (tests read the source)
export TASKWEAVE_STRICT_AGNOSTIC=zero         # test_agnostic: any robot/cell name or tuning number in code fails
for p in taskweave_core taskweave_hmi taskweave_skills taskweave_bringup taskweave_robots; do
  (cd $p && python3 -m pytest -q test)        # or: colcon test --packages-select $p
done
```
- Tests that need ROS skip themselves without `rclpy` (`pytest.importorskip`). `docs/` and `tools/` must be in the
  checkout for the doc tests (`test_skills_doc.py`, `test_docs_examples.py`); they skip when the folders are missing.
- ROS tests use `RosRig` and `wait_until` from the package's `test/conftest.py`: `with RosRig() as rig:` registers
  nodes, spins one executor and tears everything down in the right order; `wait_until(cond, what=...)` polls with a
  generous deadline (never `time.sleep` for discovery); `wait_ready` / `connect` wait for servers. Look at
  `taskweave_skills/test/test_joints_ros.py` for a skill test and `taskweave_core/test/test_ros_integration.py` for the
  task handler.
- Config fixtures use neutral robots (`demo_bot`; `duo_bot`, `diff_bot`, `omni_bot`, `arm_bot`) and a `box` object. Copy a fixture
  into `tmp_path`, never edit it in place.
- Lint: `ruff check .` from the repo root (`ruff.toml`: line length 120, single quotes, rules E F W I B UP). CI runs it
  pinned; run the same version anywhere you have Python: `python -m pip install --user ruff==0.17.1`, then
  `python -m ruff check .`. It needs no ROS, so a laptop will do; the ROS container may have no pip. Keep new code
  clean; the repo is not auto-formatted, so do not run `ruff format` over existing files.

### A new ROS package
`package.xml` (every runtime dependency as `exec_depend` with its rosdep key: CI runs `rosdep install`), `setup.py` /
`setup.cfg` / `resource/<name>` (copy `taskweave_hmi`), a `test/` folder, and its name in: the test loop above,
`ruff.toml` `known-first-party`, and `PACKAGES` in `taskweave_core/test/test_agnostic.py` if it holds code that must stay
robot-agnostic. CI builds and tests every package it finds (`colcon build`, `colcon test`), so it needs no workflow
change. Launch files go to `share/<name>/launch` through `data_files` in `setup.py`; keep their logic in the package (as
`taskweave_bringup` does) so tests can build launch descriptions without launching.

## Rules
1. **No robot, cell or object names and no tuning numbers in code.** `taskweave_core/test/test_agnostic.py` scans
   `taskweave_core`, `taskweave_skills`, `taskweave_hmi` and `taskweave_bringup` for forbidden words (robot, cell and object names), unnamed
   numbers in the skill and motion modules, module tuning constants and fixed joint indices. `zero` mode fails on any
   finding. Numbers go to config: a skill yaml, a robot's `motion_tuning`, a schema `default`.
2. **Config-driven.** A new behaviour is a config key with a schema and a default, not a constant. Schemas live in
   `taskweave_core/taskweave_core/config/schemas.py` and reject unknown keys.
3. **Skills are robot-agnostic.** They reach hardware only through the robot profile (roles: `base`, `joints`,
   `grippers`, `tool_frames`, `ik`) and the embodiment, never joint or topic names.
4. **A schema change needs a migration and a `schema_version` bump** (cell pack: `packs.CELL_SCHEMA`, robot pack:
   `packs.ROBOT_SCHEMA`; migrations in `config/migrate.py` `MIGRATIONS`, a test per migration). New optional keys with
   a default need neither; renamed or moved ones do. A deployed build rejects unknown keys, so deploy the code before
   the config that uses a new key.
5. **Fail before moving.** Anything checkable statically (skills, args, points) is checked in `validation.py`.
6. **Speed override.** Skills run on work time (`ctx.work_now()`, `ctx.work_clock()`), never on plain `time`, so the
   override scales moves and timeouts. Two clocks exist on the skill context: node time (`ctx.elapsed()`, `ctx.sleep()`:
   sim time when `use_sim_time` is set, not scaled) and work time (`ctx.work_now()`: node time integrated at the
   effective override, which is also what the step timeout counts). The one exception is `wait`: it runs on node time on
   purpose (the spec says waiting is not scaled), so ten seconds stay ten seconds at 10 %. A skill that moves, or waits
   for a move, uses work time; use node time only for something that really takes wall or sim time (a sensor delay).
7. Python 3.12, type hints where they help, docstrings that say why, line length 120.

## How-tos

### Add a skill
1. Class in `taskweave_skills/taskweave_skills/<module>.py`, subclass of `taskweave_core.skill.Skill`: `name`,
   `description` (written for a planner), `args_schema` (JSON schema; `additionalProperties: false`; mark points with
   `x-taskweave-point`, units with `x-taskweave-unit`; reuse `hands_schema`, `motion_schema(keys)`, `VEC3` from
   `config/schemas.py`), `needs`, `achieves`, `default_timeout_s`, `setup(node, config, robot, params)`,
   `execute(ctx, args)` returning `SkillResult.succeeded/failed/rejected`. Check `ctx.should_stop` in loops and return
   `ctx.interrupted()`. Implement `stop()` so cancel leaves the hardware safe. Clocks: rule 6.
   **Discovery:** `taskweave_skills.shipped_skills()` finds every `Skill` subclass with a `name` in any module of the
   package, so the class needs no registration. The docs generator and the doc tests use that one list.
2. A new point kind needs a branch in `validation.check_point`.
3. Skill yaml, two copies that must stay equal (`test_migrate_skills` checks it): `docs/examples/skills/<name>.yaml`
   and the string in `taskweave_core/taskweave_core/config/skill_templates.py` (add it to `TEMPLATES`). **Every
   shipped skill has both**, also one with no parameters (`wait`); `test_skills_doc.py` fails without them. Content:
   `implementations: [{name, type: script, class: "module:Class", requires: ...}]`, speeds, tolerances. `requires`
   lists `capabilities` (embodiment vocabulary), `embodiments`, `cameras`, `action_space`.
4. Serving: the skill server serves the names listed in the launch argument `skills:`. Add the name to the list in the
   README "Usage" launch command (the one place that lists them; `test_skills_doc.py` checks that it names every
   shipped skill), and give a cell that uses it a `skills/<name>.yaml` (start from the example above).
5. Tests: schema validation (`oneof_problems` from `taskweave_core/taskweave_core/validation.py` checks the `oneOf` variants of a
   schema, see `test_tool_pose.py`), the argument checks, a ROS test with `RosRig` against a fake robot (see
   `test_joints_ros.py`), and a speed test if it moves.
6. Docs: add an example step to `docs/examples/skill_steps.yaml` (validated by `test_docs_examples.py`), then regenerate
   the reference. The generator imports the skill modules, so it needs ROS sourced and `taskweave_core` and
   `taskweave_skills` importable (no running graph, no robot):
   ```bash
   source /opt/ros/jazzy/setup.bash && source ~/tw_ws/install/setup.bash     # the built workspace; or, unbuilt:
   export PYTHONPATH=$PWD/taskweave_core:$PWD/taskweave_skills:$PYTHONPATH   # (taskweave_interfaces must be built)
   python3 tools/gen_skills_doc.py            # write docs/skills.md; --check only compares, --stdout prints
   ```
   The tests fail on a stale `docs/skills.md`. Without ROS the import fails with a missing `rclpy` or `moveit_msgs`.

   `docs/examples/skill_steps.yaml` (no schema file; `test_docs_examples.py` and `tools/gen_skills_doc.py` read it):
   - top level: one key per skill, the skill's name; its value is the list of example steps, in the order shown.
   - each step: `instruction` (required; the text the reference shows), `skill` (required; must equal the key), `args`
     (checked against the skill's `args_schema`, with the robot's hands filled in).
   - mission-step keys are allowed too: `id`, `timeout_s`, `retries`, `on_failure` (`retry | replan | skip | abort`),
     `replan_from`, `max_replans` (the `STEP` schema in `taskweave_core/taskweave_core/config/schemas.py`). `include` and
     `with` are not (a step with `include` is not a skill call).
   - the test builds a one-step mission from each step and runs `validate_plan` against `docs/examples/cell`.

### Robot capabilities (`embodiment.CAPABILITIES`)
The words of a robot's `embodiment.yaml` `capabilities` and of a skill implementation's `requires.capabilities`
(`taskweave_core/taskweave_core/embodiment.py`; the schemas take their enum from it, so an unknown word is rejected).
An implementation fits a robot when every `requires` word is in its embodiment (`embodiment.match`, which also checks
`requires.embodiments`, `action_space` and `cameras`).

| Capability | Meaning |
|---|---|
| `base.holonomic` | base translates in any direction and rotates |
| `base.differential` | base drives forward/backward and rotates |
| `base.fixed` | base does not move |
| `torso.lift` | torso height can change |
| `torso.pitch` | torso can lean forward |
| `head.pan_tilt` | head camera can be pointed |
| `arms.single` | one manipulator arm |
| `arms.bimanual` | two manipulator arms that can work together |
| `gripper.parallel` | parallel two-finger gripper |
| `gripper.suction` | suction gripper |
| `hand.dexterous` | multi-finger hand |
| `camera.head.rgb` | colour camera on the head or body |
| `camera.head.rgbd` | colour and depth camera on the head or body |
| `camera.wrist.rgb` | colour camera on a wrist or tool |
| `camera.wrist.rgbd` | colour and depth camera on a wrist or tool |
| `lidar.2d` | planar laser scanner |
| `lidar.3d` | 3D lidar |

The base word must be the one for the profile's `base.type` (`embodiment.check_profile`; `taskweave check` reports a
mismatch, and a second base word is a contradiction too).

### Add a robot pack
A pack is a folder `taskweave_robots/<robot>/`. The packs are data only: nothing about a cell (stances, held poses,
object points belong to the cell pack).
1. `pack.yaml`: `name` (= the folder), `version`, `schema_version: 1`, `description`. Required. Add a `LICENSE-NOTE.md`
   (copy one from `galbot_s1/`): the bundled galbot and a2d packs are vendor-derived, internal use only (licence line in
   `taskweave_robots/package.xml`; the test `test_every_pack_has_a_licence_note` checks each pack has one).
2. `profile.yaml` (how the hardware is reached) and `embodiment.yaml` (what the robot is), both required:
   - profile required: `name`, `base` with `type` (`holonomic | differential | fixed`), `cmd_vel`, `odom`, `frame`.
   - embodiment required: `name`, `summary`, `capabilities` (words of `embodiment.CAPABILITIES`, e.g. `base.differential`,
     `arms.bimanual`, `gripper.parallel`; the base capability must match `base.type`, `check` reports a mismatch);
     optional `base`, `torso`, `arms`, `tools`, `cameras` descriptions (unknown values `~`). Each `tools` entry needs
     a `grippers.<name>` in the profile.
   - `poses.yaml` with a `home` pose: optional for loading, but `test_robot_packs.py` runs the generic mission with it.
3. What each optional part enables (all are optional; a pack with none of them is valid, but limited):

   | Profile / pack part | Enables |
   |---|---|
   | `joints` (`command`, `state`; `linear`, `groups`, `limits`) | `move_joints` and named joint poses. `groups`: `move_joints only`. `limits`: joint -> `[lower, upper]` laid over the URDF's, checked before a move (docs/configuration.md); none from either source = unchecked |
   | `grippers` | `grip`, hands for every arm skill |
   | `tool_frames` + `ik` (+ `arms.<tool>.elbow`) | `move_gripper`, `move_object`, `set_posture`, tool poses in `poses.yaml`, elbow control. Needs `moveit/` and a MoveIt `GetPositionIK` service behind `ik.service` |
   | `planning` + `moveit/` (URDF, SRDF, `kinematics.yaml`, `joint_limits.yaml`, collision meshes) | collision-free paths (`motion.planner`), URDF-based straight lines, velocity limits |
   | `navigation.yaml`, `base.footprint`, `base.limits` | the driving envelope for Nav2, base speed caps |
   | `motion_tuning`, `skill_overrides.yaml` | arm motion step sizes and this robot's speeds per skill (all default) |
   | `sim.yaml`, `pose_targets.yaml` | inputs of the simulator application, no taskweave schema |

   A pack **without** `ik` and `moveit/` is valid: it drives, takes joint poses and grips, but `move_gripper` /
   `move_object` reject (no `ik`) and nothing is planned around obstacles. Add the parts as the hardware needs them.
4. Install: the CMake install copies every folder at install time, so a new pack folder is picked up by the next
   `colcon build --packages-select taskweave_robots` (no reconfigure, no GLOB). If an older build tree still ignores
   it, remove `build/taskweave_robots` once. The tests read the source folder, so they need no build; hidden and
   cache folders next to the packs are ignored.
5. Validate: `python3 -m pytest taskweave_robots/test` (every pack gets the generic mission), and with a cell
   `taskweave check <cell>`, `taskweave describe <cell> <robot>` and `taskweave config files <cell> <robot>`.
   `docs/examples/cell` is a ready cell (robot `duo_bot` inside it): `taskweave check docs/examples/cell`.

### Start MoveIt for IK (a robot pack with `moveit/`)
1. The MoveIt files go in `taskweave_robots/<robot>/moveit/` (a cell's `robots/<robot>/moveit/` wins over it):
   `robot.urdf` (its presence turns move_group on; `meshes/` is referenced as `filename="meshes/..."` and made absolute
   at launch), `robot.srdf` (the planning groups, one per arm, and the disabled collision pairs), `kinematics.yaml` (the
   IK solver per group, `robot_description_kinematics`), `joint_limits.yaml` (optional: speeds and accelerations that
   planned trajectories are timed with). `taskweave_robots/galbot_s1/moveit/` is a complete example.
2. The profile's `ik:` section names the groups: `groups: {<tool>: <planning group>}`, `joints: '{tool}_joint[1-7]'` (the
   joints IK may command). Optional: `service` (default `/compute_ik`), `timeout_s`, `seeds`, `avoid_collisions`,
   `choose` (see `taskweave_skills/taskweave_skills/ik.py`).
3. Start it: `ros2 launch taskweave_bringup taskweave_bringup.launch.py config_dir:=<cell> robot:=<robot> moveit:=true`,
   or alone `ros2 launch taskweave_bringup taskweave_moveit.launch.py config_dir:=<cell> robot:=<robot>` (its
   `use_sim_time` defaults to true; `use_sim_time:=false` on hardware). `taskweave_bringup/taskweave_bringup/moveit.py`
   builds the parameters. move_group respawns on exit.
4. move_group serves `/compute_ik` (the `ik` skills call it) and `/check_state_validity`. It has no MoveIt controllers
   (`allow_trajectory_execution: False`): taskweave's joint skills execute the motion. Check with
   `ros2 service list | grep -E "compute_ik|check_state_validity"`, on the robot's own ROS domain.

### Add a cell pack (an application's config)
1. `pack.yaml` with `name`, `version`, `schema_version: 3`; `system.yaml` with `robot_packs: [..]` if packs live outside
   the install.
2. `cell/layout.yaml`, `cell/locations.yaml`, `cell/objects.yaml` (placed instances; `stack: {count, pitch}`).
3. `objects/<type>.yaml`: points with taught tool poses (`taskweave teach point <instance>/<point>`) and `contacts`.
4. `robots/<robot>/stances.yaml`, `held_poses.yaml`, `object_points.yaml`: what this robot does at this cell.
5. `skills/<skill>.yaml` for each skill the missions use (start from `docs/examples/skills/`),
   `missions/<name>.yaml`.
6. From an older layout: `taskweave config migrate <cell>` (runs every migration up to schema 3; `migrate-skills`
   is the 2 to 3 step alone).
   Check: `taskweave check <cell>`. Details: [docs/configuration.md](docs/configuration.md).

### Add a skill to a live cell
1. Copy the shipped template `docs/examples/skills/<skill>.yaml` (every shipped skill has one) to `<cell>/skills/<skill>.yaml`.
   `implementations` is a list of `{name, type: script | vla | rl | other, class: "module:Class", requires: {capabilities,
   embodiments, cameras, action_space}, params}`; the skill server serves the first one that fits the robot. Other
   top-level keys (`speed`, `tolerances`, `motion`, ...) are shared by all implementations and read by the skill.
2. It is served only when its name is in the skill server's `skills:=` launch argument (the list in README "Usage").
3. What checks it: `taskweave check <cell>` validates the file against `SKILL_PARAMS` in
   `taskweave_core/taskweave_core/config/schemas.py` (an implementation rejects unknown keys, a bad `type` or `class`) and
   imports every class (`cli._importable`). `taskweave describe <cell> <robot>` lists the skill, or the reasons its
   `requires` do not fit. It does not check the skill's arguments (see "Write a mission").
4. Restart: the skill server reads `skills/<skill>.yaml` when it starts, so a new or changed file needs a restart of the
   node that serves it (on the live rig, `frame-ros`, after the rules in CLAUDE.md). A build that predates a new key
   rejects the file, so deploy the code first and the config after (rule 4). The launch log prints `ERROR ... not served`
   for a skill whose file is missing or whose `requires` the robot cannot meet.

### The config API (catalogue, schemas, checks)
`config/catalogue.py` holds **one table** (`KINDS`): per kind of file its pattern, schema, `reload` (what a change needs)
and editability; a kind's id is its schema id. `ConfigStore.files()`, `file_kinds()`, `json_schema()`, `enum_values()`,
`file_hash()` read it. `config/check.py` has `validate_file` (schema, then `CROSS[kind]`, one function per kind),
`check_pack`, `dry_run_pack`; problems are dicts `{path, pack, where (JSON pointer), line, col, message, severity}`.
A new config file: add a `Kind` (with a schema or a `no_schema_reason`), a `CROSS` entry if it refers to other files,
`ENUM_HINTS` for names that depend on the pack; `test_catalogue.py` checks the table. A new schema key: optional only, and
`json_schema` exports it with no further work.

### Write a mission
See [docs/missions.md](docs/missions.md). Put it in `<cell>/missions/<name>.yaml`. Validate in layers:
- `taskweave check <cell>` (offline; `ConfigStore.check_pack()` returns the same as problem dicts) checks the schema and the
  cross-file rules of every file of the pack and its robot packs (config API below), and that every mission expands.
  It does **not** check skill arguments, nor whether a point, contact, stance or pose a step names exists:
  those need the skills' argument schemas.
- The HMI editor and the task handler check them against the live skill catalog before a start.
- To catch a misspelt point, contact, stance or pose name before anything moves: start the skills and read the refusal
  (`taskweave run <mission>` prints `rejected:` with the reason; the HMI editor shows it too). Offline, the test
  `taskweave_skills/test/test_docs_examples.py` shows the Python check: `validate_plan(plan, catalog(store), store)` with
  the catalog built from the skill classes.
- A test does the same offline: `test_docs_examples.py` builds the catalog from the skill classes and runs
  `validate_plan` on every example. Add a mission that teaches something to `docs/examples/missions/` and to the list in
  that test (the docs must show it verbatim); it runs against `docs/examples/cell` (robot `duo_bot`, a copy of the
  skills test fixture's).

### Speed override and test mode
- Service `/taskweave/task_handler/set_speed_override` (`SetSpeedOverride`: `percent`, `test_percent`, `set_test_mode`,
  `test_mode`), parameter `speed_override_percent`, latched topic `/taskweave/speed_override` (`SpeedOverride`).
  `OverrideState` in `speed.py` holds the rules (range 1-100, test mode latch: lower the override to `test_percent` at
  run start, clamp to `cap_percent` while on, restore at run end only if it lowered it).
- Skills follow it through `SpeedOverride.for_node` and `ctx.work_clock()`; base drives that Nav2 plans use `speed_gate`.
- Test: `test_speed*.py` in `taskweave_core/test` and `taskweave_skills/test`.

### Smoke test
`taskweave smoke run <mission> --hmi http://<hmi>:8090 [--reference <file>]` talks to the HMI's HTTP API only (no ROS
where it runs), resets the sim, runs the mission and compares every step with a saved reference run (outcome, duration,
max error). `--save-reference` records one; `taskweave smoke compare` works offline on saved runs. Exit 0 passed,
1 failed, 2 could not run. Code: `taskweave_core/taskweave_core/smoke.py`, tests: `test_smoke.py`.

## Docs and commits
- `docs/skills.md` is generated: never edit it by hand. `docs/configuration.md` and `docs/architecture.md` are edited
  by hand and must match the code (grep a key before documenting it).
- Stage files explicitly (`git add <files>`), not `git add -A`. Do not push without asking.
