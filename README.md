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
| [`taskweave_skills/`](taskweave_skills) | Generic, robot-agnostic skills: `wait`, `navigate_to`, `go_to_point` |
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
- [ ] Teaching tool poses, `go_to_point`, Nav2 and MoveIt adapters
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
```

## Build

ROS 2 Jazzy.

```bash
mkdir -p ~/tw_ws/src && cd ~/tw_ws/src && git clone https://github.com/flrzep/taskweave.git
cd ~/tw_ws && rosdep install --from-paths src -y --ignore-src
colcon build && source install/setup.bash
colcon test && colcon test-result --verbose
```
