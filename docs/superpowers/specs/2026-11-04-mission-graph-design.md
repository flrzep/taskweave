# Mission graph: schema v2, execution semantics and the planner API

Status: design, agreed with the user on 4 Nov 2026. The HMI session builds the graph editor and the Run dashboard
against this; the core session implements the core part (sections 2 to 5). Nothing here changes an existing key,
so no schema bump or migration is needed (AGENTS.md rule 4).

## 1. Why

Missions are a flat list of steps. The HMI's Run tab already draws that list as a state machine from the plan, and
operators program it in a form. Three things do not fit the list any more:

- **Loops, parallel branches and groups.** A cell cycle is "repeat while a stack has parts: pick, carry (base drives
  while the arm holds), drop". Today the loop is a whole-mission rule, a group only an include flattened away, and
  parallel does not exist.
- **AI as a first-class citizen.** Policy skills (VLA/RL) are already an implementation type. An AI planning module
  that writes a mission or a part of one needs a schema it can target, a description of the robot, and a validation
  loop with paths it can act on.
- **A graph editor.** Nodes with ports, collapsible groups, loops and lanes need a tree-shaped mission with stable
  paths back to the file and line each node came from.

The stored form stays readable YAML: people diff it, keep it in git on the host and edit it by hand. The graph is a
view of the YAML and edits it through operator-like calls (section 6). Arbitrary wiring between nodes is deliberately
not supported: a mission is a structured program, and a structured tree has one canonical layout.

## 2. Schema v2

A step is **exactly one** of `skill`, `include`, `group`, `repeat`, `parallel`. The first two exist. The three new
ones are *blocks*: they carry `steps` (or `branches`) and the same rule keys a step has.

```yaml
goal: Empty every stack into the bin
params: {...}                # unchanged (a mission with params is an include, not startable)
facts:  {...}                # unchanged
defaults: {...}              # unchanged
loop: forever                # unchanged; the HMI renders it as an outer repeat
steps:
  - id: empty_stacks
    instruction: Repeat while a stack has parts
    repeat: {while: next_stack}          # one of: while: <fact>, until: <world key>, times: N
    max_cycles: 50                       # a bound for while/until; required with them
    steps:
      - id: pick
        instruction: Pick the top part of {next_stack}
        include: pick_part               # a linked group: shared file, shown with a link badge
        with: {part: "{next_stack}"}
      - id: carry
        instruction: Carry to the bin
        parallel:
          - id: drive
            instruction: Drive to the bin
            steps:
              - {instruction: Drive, skill: move, args: {to: bin_1/place}}
          - id: hold
            instruction: Keep the part level
            steps:
              - {instruction: Hold level, skill: hold_level}
        join: all                        # all (default) | first
      - id: drop
        instruction: Let go
        group: true                      # an inline group: local, collapsible, no own file
        steps:
          - {instruction: Open, skill: grip, args: {hands: all, action: open}}
  - id: grasp_loose
    instruction: Pick up the loose part with the policy
    skill: pick_loose                    # a policy skill (implementation vla) is an ordinary step
    when: loose_part_seen                # guard: skipped when the world key is falsy
    on_failure: replan
    replan_from: empty_stacks
```

### 2.1 Keys

| Key | On | Type | Meaning |
|---|---|---|---|
| `group` | block | `true` | an inline group of `steps`; only structure and a collapse unit in the graph |
| `repeat` | block | object | `{while: <fact name>}`, `{until: <world-state key>}` or `{times: N>=1}`; exactly one |
| `max_cycles` | repeat | int >= 1 | bound for `while`/`until` (required with them; ignored with `times`) |
| `parallel` | block | list of branches | each branch: `{id, instruction, steps: [...]}`; 2 or more |
| `join` | parallel | `all` \| `first` | default `all`: the block ends when every branch ended; `first`: when one branch succeeded, the others are stopped |
| `steps` | group, repeat | list of steps | at least 1 |
| `when` | any step or block | world-state key (NAME) | guard: run only when the key is truthy, else outcome `skipped` |
| `id`, `instruction`, `timeout_s`, `retries`, `on_failure`, `replan_from`, `max_replans` | block | as on a step | `timeout_s` on a block bounds the whole block; `retries` restarts the block |

Constraints checked by the schema (`schemas.py`) and `check.py`:

- `oneOf` over `skill` / `include` / `group` / `repeat` / `parallel`; `with` only with `include`, `args` only with `skill`.
- `join` only on `parallel`; `max_cycles` only on `repeat`; `steps` only on `group` and `repeat`.
- Ids unique over the whole tree, including branch ids and ids inside includes after expansion
  (an include's inner ids stay prefixed `<include id>_<inner id>` as today).
- `replan_from` must name an **ancestor block or an earlier sibling at any level above or equal**; never a step
  inside a parallel sibling branch or a later step.
- Parallel branches must lock **disjoint resources** (`step_info.py` computes a step's resources; a branch's set is
  the union of its steps'). A skill whose resources are unknown makes the branch unschedulable: a structured problem.
- Nesting depth of blocks and includes together at most 5, as for includes today.

### 2.2 Catalogue and docs

- `catalogue.py`: the mission template stays `{goal, steps}`; `ENUM_HINTS` gains `join`, `on_failure` on blocks.
- `docs/missions.md`: a section per construct; `docs/examples/missions/` gets one documented example per construct
  (`test_docs_examples.py` validates them against `docs/examples/cell`).

## 3. Expansion and the plan the HMI sees

- `expand_includes` keeps the **tree**. An include becomes a block of kind `include` with the included file's steps
  as children (placeholders from `with` filled as today).
- Every node carries `origin`: `{file: <mission or include file>, path: "steps/0/steps/1/branches/0/steps/0"}`, the
  JSON-pointer-like path **in its own file**, plus `include: [chain of include ids]` as today. The HMI uses it to map a
  node back to a line and to route an edit to the right file.
- The plan message and `/api/missions/<name>/plan` carry the tree: `{plan_id, goal, robot, loop, steps: [node]}`,
  `node = step_dict | {id, instruction, kind: group|repeat|parallel|include, origin, rule keys..., steps|branches}`.
  `Plan.msg` gains a `tree_json` string (the flat `steps` stays for older readers until the HMI is switched).
- Facts and params are filled at the same points as today (`_fill`); nothing new flows between nodes.

## 4. Execution semantics (executive)

The executive walks the tree; the flat list becomes a stack of frames.

- **group**: runs its steps in order. A failed step applies its own rule first; a failure that escapes applies the
  group's rule (`retry` restarts the group, `skip` skips the rest of it, `replan`, `abort`).
- **repeat**: evaluates its condition **before each pass**: `while` recomputes the fact (as `make_plan` does per cycle
  today; empty ends the loop), `until` reads the world key, `times` counts. `max_cycles` ends it. Each pass is a
  `pass` counter in the status. Mission-level `loop` keeps today's behaviour.
- **parallel**: dispatches each branch's first step at once; each branch proceeds independently (one action goal
  per branch). `join: all` waits for all; `join: first` stops the others (cancel their running action) when one
  succeeds. A failed branch stops its siblings, then the block's `on_failure` applies. Speed, pause and abort apply
  to every running branch; pause waits for all of them to settle.
- **when**: evaluated at the node's start; falsy records outcome `skipped` (an episode `step_finished` with
  `outcome: SKIPPED`) and moves on.
- **replan**: `replan_from` restarts at that node; counting against `max_replans` as today.
- **Status** (`TaskStatus`, `/api/state.status`): gains `active: [{path, step_id, skill, attempt, status, progress,
  elapsed_s}]` (several entries during a parallel block), `passes: {<repeat id>: n}`, and keeps `step` as the first
  active entry for older readers. Episodes gain `path`.
- **Resources**: the dispatcher already runs one skill at a time; concurrent branches need the `RosDispatcher` to hold
  several goals (keyed by path). The skill server already serves each skill on its own action server.

Tests: `test_executive.py` with `FakeDispatcher` for every rule above (a fixture mission per construct); a
`test_include.py` case for a tree that keeps origin paths; `test_validation_paths.py` for the new structured
problems (disjoint resources, replan target).

## 5. Planner API

The AI planner writes **only the schema** and never bypasses validation. It is a closed loop: generate, validate,
fix, up to N rounds, then the operator reviews.

| Endpoint | Returns |
|---|---|
| `GET /api/schema/mission` | the v2 JSON schema (from `catalogue.py`) |
| `GET /api/robot/card` | `DescribeRobot`: `card` text, `embodiment`, `skills: [SkillInfo]` (description, args schema, needs, achieves, implementation), and the target groups of `/api/points` |
| `POST /api/missions/validate` `{mission}` | `{problems: [{path, message, origin}], notes}` for an unsaved body (the per-name form exists) |
| `POST /api/plan/propose` `{goal, mission?, select?: path, failed?: {plan_id, path, error}}` | `{mission, patch: [json-patch ops on the tree], rationale, rounds, problems: []}` |

- `propose` without `mission` generates a whole mission; with `mission` and `select` it replaces the subtree at
  `select` (an empty group, or a gap: `select` may end in `/-`); with `failed` it proposes a repair (used later by
  the Run page).
- Core module `taskweave_core/planner_llm.py`: builds the prompt from the robot card, the skill list and the schema,
  calls the model, validates with `validate_plan_structured`, feeds the problems back, stops after `rounds` (3).
  The model and key come from the cell config (`cell/planner.yaml`: `{model, api_key_env, rounds}`), absent =
  endpoint answers 503 "no planner configured".
- Every node's `instruction` is natural language already and doubles as the planner's per-node rationale.

## 6. Mission edits from the graph (HMI server)

`POST /api/missions/<name>/edit {ops: [...]}` applies operator-style edits on the ruamel tree in place (comments and
key order kept), validates, saves, and answers `{mission, problems, undo}`. Ops: `insert {at: path, step}`,
`remove {at}`, `move {from, to}`, `wrap {from, to, as: group|repeat|parallel, ...}`, `unwrap {at}`,
`set {at, key, value}`. An op whose `at` lies inside an include is applied to the include's file (the reply names
the file). Undo as `undo.py`. `missions.py` `save` must write `params` and `facts` (today `TOP_LEVEL_ORDER` lacks
them).

## 7. Out of scope here

Conditions beyond the `when` guard (if/else branches), data ports between nodes, a free node canvas, run-time
acceptance of a patched plan mid-run (the API allows it; the executive hook comes after the editor).
