# taskweave

Config-driven task planning and execution for ROS 2 robots.

A **planner** turns a goal into steps, a **task handler** runs them and decides what happens when one fails, and
**skills** do the work behind a single ROS 2 action. Every location, grasp point, speed and decision rule comes from
YAML. Robot hardware is described in one profile, so the same mission runs on a different robot.

Skills can be scripts today and learned policies (VLA, RL) tomorrow; the planner can be a mission file today and a
VLM tomorrow. Neither change touches the rest of the system.

## How it fits together

```mermaid
flowchart LR
    goal([Goal]) --> planner[Planner<br/>mission file · VLM]
    planner -- Plan --> handler[Task handler<br/>validate · run · decide]
    handler -- RunSkill --> skills[Skills<br/>script · VLA · RL]
    skills --> control[Robot control<br/>/cmd_vel · ros2_control · grippers]
    config[(config/*.yaml)] -.-> planner
    config -.-> handler
    config -.-> skills
    operator[Operator<br/>pause · resume · abort] -.-> handler
```

| Layer | Responsibility | Never does |
|---|---|---|
| Planner | Goal → plan (ordered steps, each with `instruction`, `skill`, `args`) | Call controllers |
| Task handler | Validate the plan, run steps, apply retry / replan / skip / abort, publish status | Know how a skill works |
| Skills | Execute one step behind `RunSkill`, address robot *roles* via the robot profile | Know about plans or other steps |
| Config | Every point, number, name and rule | Contain code |

Each step carries a natural-language `instruction` as well as a structured `skill` + `args`, so the same plan works
for scripted skills and for language-conditioned policies.

## Decision flow

```mermaid
flowchart TD
    goal([Goal]) --> planner[Planner] --> plan[Plan]
    plan --> valid{Plan valid?}
    valid -- no: reason --> planner
    valid -- yes --> next[Next step]
    next --> run[Dispatch RunSkill] --> skill[Skill]
    skill --> outcome{Outcome?}
    outcome -- succeeded --> world[Update world state]
    world --> more{More steps?}
    more -- yes --> next
    more -- no --> loop{Loop mission?}
    loop -- yes --> planner
    loop -- no --> done([Done])
    outcome -- failed / timeout --> retries{Retries left?}
    retries -- yes --> run
    retries -- no --> policy{on_failure}
    policy -- replan --> planner
    policy -- skip --> world
    policy -- abort --> abort([Abort · safe stop])
```

Every decision reads its rule from the mission file (`timeout_s`, `retries`, `on_failure`, `replan_from`,
`max_replans`, `loop`, `stop_when`).

## A mission

```yaml
goal: Move frames from the bays to the Karakuri rack
robot: a2d                    # selects config/robots/a2d/
loop: forever                 # once | forever
stop_when: bays_empty         # world-state flag that ends the mission
defaults: {timeout_s: 120, retries: 1, on_failure: abort}
steps:
  - instruction: Drive to the fullest stack
    skill: navigate_to
    args: {target: "stance:{fullest_stack}"}
  - instruction: Pick the top frame with both hands
    skill: pick_frame
    args: {object: frame_type2, at: "{fullest_stack}", grasp: side_rails}
    retries: 2
    on_failure: replan
```

Placeholders such as `{fullest_stack}` are resolved from the world state when the step starts.

## Repository layout

| Path | Contents |
|---|---|
| [`taskweave_interfaces/`](taskweave_interfaces) | ROS 2 interfaces: `RunSkill` action, `Plan`, `PlanStep`, `TaskStatus`, `SkillInfo`, `ListSkills`, `StartMission` |
| [`taskweave_core/`](taskweave_core) | Config loader and schemas, planners, task handler, skill server base class, CLI |
| [`taskweave_skills/`](taskweave_skills) | Generic, robot-agnostic skills: `wait`, `navigate_to`, `drive_straight`, `move_joints`, `move_gripper`, `move_object`, `set_posture`, `grip` |
| [`taskweave_hmi/`](taskweave_hmi) | Web HMI: live mission status and control, validated mission editor |
| [`docs/architecture.md`](docs/architecture.md) | Layers, interfaces, decision rules, design rules |
| [`docs/configuration.md`](docs/configuration.md) | Config tree and file formats |

Application-specific parts (a cell's layout, its objects and missions, a robot profile, cell-specific skills) live in
the application repository and are passed to taskweave as a config directory.

## Status

Early development. The interfaces are defined; the core packages are being built step by step:

- [x] Interfaces (`taskweave_interfaces`)
- [x] Config loader and schemas, robot profile and embodiment, capability vocabulary
- [x] Mission-file planner, executive (decision rules) and task handler
- [x] Skill server with implementation selection, `wait`, `navigate_to`
- [x] Teaching named base poses; `describe_robot` for planners
- [x] Joint skills `move_joints` (named poses) and `grip` (open/close, holding detection)
- [x] Joystick buttons for pause/abort and manual skill calls (`operator.yaml`)
- [x] Web HMI: live status, start/pause/resume/abort, mission editor with validation
- [x] Teaching tool poses (pad or CLI), `move_gripper`, Nav2 and MoveIt adapters
- [ ] VLM planner

## Usage

```bash
ros2 launch taskweave_core taskweave.launch.py config_dir:=/path/to/config robot:=a2d skills:=wait,navigate_to
ros2 run taskweave_core taskweave check /path/to/config     # validate all config files offline
ros2 run taskweave_core taskweave describe                  # robot card + skills of the running system
ros2 run taskweave_core taskweave run frame_cycle           # start a mission
ros2 run taskweave_core taskweave status --follow
ros2 run taskweave_core taskweave pause | resume | abort
ros2 run taskweave_core taskweave teach location bay_2      # save the current base pose
ros2 run taskweave_core taskweave teach point bay_2_stack_1/rails_top   # save the tools' TCPs on an object
ros2 run taskweave_hmi hmi_server --ros-args -p config_dir:=/path/to/config -p robot:=a2d -p host:=<address>
```

The HMI (default port 8090) has no login: bind it to a trusted network only (for example a Tailscale address).

### Mission smoke test

`taskweave smoke` runs a mission in the simulator from a clean start and checks every step against a stored
reference run. It talks to the HMI over HTTP only (no ROS needed where it runs): the Record tab's API resets the sim,
starts the mission and stops it after `--cycles`; the step events of `/api/state` give each step's outcome, duration
(sim time) and the `max error` a skill reports. Bag and video are off unless `--bag` / `--video` is given.

```bash
taskweave smoke run frame_cycle --hmi http://<hmi>:8090 --save-reference config/tests/frame_cycle.reference.yaml
taskweave smoke run frame_cycle --hmi http://<hmi>:8090 --reference config/tests/frame_cycle.reference.yaml
taskweave smoke compare config/tests/frame_cycle.reference.yaml run.yaml   # offline, a saved run (--save-run)
```

A step passes when it runs in the reference's order with outcome `SUCCEEDED`, its duration is within `--tolerance`
(default ±10 %) plus `--slack` seconds (1.0) of the reference, and its max error is at most reference × (1 +
tolerance) + `--error-slack` (0.002). Missing steps fail, and so do steps that are not in the reference unless
`--allow-extra`. Duration tolerance is per skill: `navigate_to` gets 25 % (Nav2 drives vary more than arm moves),
overridable with `--skill-tolerance SKILL=FRACTION` or `limits: {tolerance_by_skill: {...}}` in the reference. The
command prints a table (step, instruction, ref s, now s, Δ%, error, result), writes a report (`<name>.report.yaml`
next to the reference, or `--report FILE`) and exits 0 (passed), 1 (failed) or 2 (could not run: HMI unreachable,
recording rejected). Without `--reference` it passes when every step succeeded.
The reference is just a saved run: re-record it when a mission changes on purpose.

## Build

ROS 2 Jazzy.

```bash
mkdir -p ~/tw_ws/src && cd ~/tw_ws/src && git clone https://github.com/flrzep/taskweave.git
cd ~/tw_ws && rosdep install --from-paths src -y --ignore-src
colcon build && source install/setup.bash
colcon test && colcon test-result --verbose
```
