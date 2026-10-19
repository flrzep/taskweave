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
| `taskweave_core/taskweave_core/` | `config/` (store, schemas, packs, overlay, migrate*, slim), `planner.py`, `executive.py`, `task_handler.py`, `skill.py` (base class), `skill_server.py`, `validation.py`, `motion.py` (motion option resolution), `speed.py`, `speed_gate.py`, `smoke.py`, `teach.py`, `operator_joy.py`, `cli.py` |
| `taskweave_skills/taskweave_skills/` | the generic skills: `basic` (wait), `navigate`, `drive`, `joint_motion`, `gripper_motion`, `object_motion`; helpers `joints`, `ik`, `chain`, `cartesian`, `motion_plan`, `hold`, `points`, `tcp_jog`, `nav2` |
| `taskweave_hmi/taskweave_hmi/` | web HMI (`server.py`, `web/*.js`): status, control, mission editor, teach, record |
| `taskweave_robots/<robot>/` | robot packs (`pack.yaml`, `profile.yaml`, `embodiment.yaml`, `poses.yaml`, `moveit/` ...), installed to `share/taskweave_robots/` |
| `docs/` | `architecture.md`, `configuration.md`, `missions.md`, `skills.md` (generated), `pack_switchover.md`, `examples/` |
| `tools/gen_skills_doc.py` | generates `docs/skills.md` |
| `*/test/` | pytest suites per package; config fixtures in `*/test/fixtures/config` |

## Build
```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/tw_ws/src && cd ~/tw_ws/src && git clone https://github.com/flrzep/taskweave.git
cd ~/tw_ws && rosdep install --from-paths src -y --ignore-src
colcon build && source install/setup.bash
```
`taskweave_skills` needs MoveIt messages (`moveit_msgs`) on the system; the task handler needs `jsonschema`, `pyyaml`,
`numpy`.

## Run the tests
Each package has a `test/` folder. The ROS tests start real nodes, so:
```bash
source /opt/ros/jazzy/setup.bash && source ~/tw_ws/install/setup.bash
export ROS_DOMAIN_ID=77                       # never the domain of a live robot
export TASKWEAVE_ROBOT_PACKS=$PWD/taskweave_robots   # the robot packs of this checkout (tests read the source)
export TASKWEAVE_STRICT_AGNOSTIC=zero         # test_agnostic: any robot/cell name or tuning number in code fails
for p in taskweave_core taskweave_hmi taskweave_skills taskweave_robots; do
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
- Lint: `ruff check .` (`ruff.toml`: line length 120, single quotes, rules E F W I B UP). Keep new code clean; the repo is not auto-formatted, so do not run `ruff format` over existing files.

## Rules
1. **No robot, cell or object names and no tuning numbers in code.** `taskweave_core/test/test_agnostic.py` scans
   `taskweave_core`, `taskweave_skills` and `taskweave_hmi` for forbidden words (robot, cell and object names), unnamed
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
   override scales moves and timeouts.
7. Python 3.12, type hints where they help, docstrings that say why, line length 120.

## How-tos

### Add a skill
1. Class in `taskweave_skills/taskweave_skills/<module>.py`, subclass of `taskweave_core.skill.Skill`: `name`,
   `description` (written for a planner), `args_schema` (JSON schema; `additionalProperties: false`; mark points with
   `x-taskweave-point`, units with `x-taskweave-unit`; reuse `hands_schema`, `motion_schema(keys)`, `VEC3` from
   `config/schemas.py`), `needs`, `achieves`, `default_timeout_s`, `setup(node, config, robot, params)`,
   `execute(ctx, args)` returning `SkillResult.succeeded/failed/rejected`. Check `ctx.should_stop` in loops and return
   `ctx.interrupted()`. Implement `stop()` so cancel leaves the hardware safe.
2. A new point kind needs a branch in `validation.check_point`.
3. Skill yaml: `docs/examples/skills/<name>.yaml` and the matching entry in `config/skill_templates.py` (keep them in
   step: `test_migrate_skills`) with `implementations: [{name, type: script, class: "module:Class", requires: ...}]`,
   speeds, tolerances. `requires` lists `capabilities` (embodiment vocabulary), `embodiments`, `cameras`, `action_space`.
4. Tests: schema validation (`oneof_problems` helper for `oneOf`), the argument checks, a ROS test with `RosRig`
   against a fake robot (see `test_joints_ros.py`), and a speed test if it moves.
5. Docs: add an example step to `docs/examples/skill_steps.yaml`, then regenerate the reference:
   `python3 tools/gen_skills_doc.py` (needs ROS sourced, no running graph). The tests fail on a stale `docs/skills.md`.

### Add a robot pack
1. Folder `taskweave_robots/<robot>/` with `pack.yaml` (`name` = folder, `version`, `schema_version: 1`, `description`).
2. `profile.yaml` (base type `holonomic | differential | fixed`, topics, `joints`, `grippers`, `tool_frames`, `ik`,
   optional `planning`, `motion_tuning`), `embodiment.yaml` (capabilities, tools, arms), `poses.yaml` (joint poses and
   tool poses, e.g. `home`), optional `navigation.yaml`, `sim.yaml`, `pose_targets.yaml`, `skill_overrides.yaml`.
3. `moveit/` for arms with a motion planner: URDF, SRDF, `kinematics.yaml`, `joint_limits.yaml`, collision meshes.
4. Validate: `python3 -m pytest taskweave_robots/test` (every pack gets the generic mission), and with a cell
   `taskweave check <cell>`, `taskweave describe <cell> <robot>` and `taskweave config files <cell> <robot>`.
   No cell content in a pack: stances, held poses and object points belong to the cell.

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

### Write a mission
See [docs/missions.md](docs/missions.md). Put it in `<cell>/missions/<name>.yaml`; validate with `taskweave check <cell>`
and in the HMI editor, which checks arguments against the live skill catalog. Add a regression example to
`docs/examples/missions/` if it teaches something (`test_docs_examples.py` validates those against the skills).

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
