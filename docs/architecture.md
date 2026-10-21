# Architecture

taskweave separates *what* a robot should do (plans), *when and in which order* (task handler) and *how* (skills).
Each layer talks to the next through one fixed interface, so any layer can be replaced without touching the others.

## Layers

### Planner
Turns a goal into a `Plan`: an ordered list of `PlanStep`s. Each step has

- `instruction`: natural language, always set. Language-conditioned policies (VLAs) read this.
- `skill` + `args`: structured call into the skill catalog. Scripted skills read this.
- optional decision rules: `timeout_s`, `retries`, `on_failure`.

Planner interface (Python, in `taskweave_core`):

```python
class Planner(ABC):
    def make_plan(self, world: Mapping[str, Any], cycle: int) -> Plan | None: ...      # None: mission complete
    def replan(self, plan: Plan, failed_index: int, reason: str,
               world: Mapping[str, Any]) -> tuple[Plan, int] | None: ...               # (new plan, step to resume at)
```

| Implementation | Status |
|---|---|
| `MissionFilePlanner`: loads `config/missions/<name>.yaml`; `replan` restarts the mission from the world state | first |
| VLM planner: prompts a model with goal, world state, skill catalog, named targets and optionally a camera image; output must be a valid `Plan` | later |

### Task handler
A ROS 2 node that

1. validates a plan before anything moves: every skill exists in the catalog, every `args` object matches the skill's
   JSON schema, every referenced target exists in config and fits the step (a hand option on a base target is refused);
2. resolves placeholders (`{fullest_stack}`) from the world state when a step starts;
3. sends the step to the skill through `RunSkill` and forwards feedback to `TaskStatus`;
4. applies the decision rules on failure or timeout: `retry` → `replan` / `skip` / `abort`;
5. loops the mission according to `loop`;
6. handles operator commands (`/taskweave/pause`, `/taskweave/resume`, `/taskweave/abort`) at any time by canceling the active skill;
7. publishes episode markers (step start / end with instruction and outcome) so runs can be recorded as training data.

It knows nothing about how a skill works.

### Skills
A skill executes one step. Every implementation serves the same action, `taskweave_interfaces/action/RunSkill`, and
registers a `SkillInfo` (name, description, JSON schema of its arguments, needs, achieves) that ends up in the catalog
(`ListSkills`).

- Skills address the robot through **roles** from the robot profile (`base`, `arms.left`, `tool: right`, `grippers.both`),
  never joint names or topic names.
- Skills read their parameters from `config/skills/<skill>.yaml`.
- Swapping a scripted skill for a VLA or RL policy means serving the same skill name with the same arguments.

### Robot description
Each robot has two files: the **profile** (how its hardware is reached: topics, joints, controllers) and the
**embodiment** (what it is and can do: capabilities from a fixed vocabulary, tools with the canonical TCP, reach,
cameras, action spaces). The embodiment drives three decisions:

1. **Which skills a robot can run.** Each skill lists implementations with `requires`; the skill server serves the
   first that fits, and plans using a skill with no fitting implementation are rejected.
2. **What a planner may ask for.** `describe_robot` (service `/taskweave/describe_robot`, CLI `taskweave describe`)
   returns a robot card (summary, capabilities, limits, cameras, known targets by kind) plus the skill catalog filtered to this
   robot. This is the input for a VLM planner.
3. **Whether a grasp fits.** Hand targets on objects carry `tool_type` and `grip_width_m`, checked against the robot's tools.

### Configuration
All application knowledge is data: cell layout, targets (named places, stances, hand points, carry and joint poses), objects, robot profile and
embodiment, skill implementations and parameters, missions. See [configuration.md](configuration.md).

## Interfaces

| Interface | Type | Used by |
|---|---|---|
| `RunSkill` | action | task handler → skill |
| `Plan`, `PlanStep` | msg | planner → task handler; published on `/taskweave/plan` |
| `TaskStatus` | msg | task handler → operator UIs, recorders; `/taskweave/status` |
| `SkillInfo`, `ListSkills` | msg, srv | skill catalog → task handler, planners |
| `StartMission` | srv | operator / MES → task handler |
| `DescribeRobot` | srv | planners → task handler (robot card + filtered catalog) |
| `TeachPoint` | srv | operator → teach node (save the current base pose, tool poses or joint values as a target, by name) |
| `SpeedOverride` | msg | task handler → skills: the global speed override and test mode (latched, `/taskweave/speed_override`) |
| `SetSpeedOverride` | srv | operator → task handler: the single write path of the override (`/taskweave/task_handler/set_speed_override`) |
| `/taskweave/pause`, `/taskweave/resume`, `/taskweave/abort` | `std_srvs/Trigger` | operator → task handler |

`RunSkill` outcomes: `SUCCEEDED`, `FAILED` (ran but missed the goal), `TIMEOUT`, `CANCELED` (by the task handler),
`REJECTED` (invalid arguments or unmet preconditions; nothing moved).

## Design rules

1. **Nothing hard-coded.** Numbers, names, targets and rules live in config. Code only holds behaviour.
2. **One kind of place, in task space.** A [target](targets.md) says what it moves and gets its frame from its file: base
   poses in the cell, stances and hand points relative to an object, carry poses relative to the robot. One skill,
   `move`, goes to any of them. A joint target is for a pose that has no task-space meaning (`home`) and names joint
   groups, never joints.
3. **Two files per robot.** How the hardware is reached (profile) is separate from what the robot can do
   (embodiment). A new robot needs both plus its robot-specific hints; missions, cell config and the targets on objects
   stay unchanged. Skills it cannot run are filtered out and reported, not attempted.
4. **Fail before moving.** Anything that can be checked statically (skills, arguments, targets) is checked when the plan
   is validated.
5. **Every step is observable.** Status and episode markers are published for every step and attempt.
