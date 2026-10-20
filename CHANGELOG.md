# Changelog

## Unreleased

- **Reload without a restart.** `<node>/reload_config` (`ReloadConfig`) on the task handler and every skill server
  re-reads the config between missions and goals; `taskweave config reload [--ros | --hmi URL]` calls them all and
  prints what was reloaded and what needs a restart. Skill yaml parameters and `base.limits` apply live; the catalogue
  value of `skills/*.yaml` is now `reload_service` (was `skill_server`). Skills read their parameters in
  `Skill.configure`, split off `setup`. See `docs/configuration.md`, "Reload without a restart".
- **Container image.** A `Dockerfile` (ROS 2 Jazzy, all packages built from the checkout, entrypoint sourcing ROS and
  the install, `taskweave` on the PATH), `docker/compose.example.yaml` and a CI job that builds the image without
  pushing it. Nothing is published. See `docs/install.md`, "Container".
- **Pack versions.** A cell or robot `pack.yaml` may carry `requires: {taskweave: ">=0.1,<0.2"}` (a PEP 440
  specifier). Loading and `taskweave check` refuse a pack that does not admit the installed taskweave and name both
  versions; a missing `requires` is allowed. The example cell and the bundled robot packs require `>=0.1,<0.2`. New
  run-time dependency: `python3-packaging`.

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
