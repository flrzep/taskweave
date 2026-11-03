# One step of a plan. Produced by a planner, executed by the task handler through RunSkill.

string step_id          # unique within the plan
string instruction      # natural-language description (always set)
string skill            # skill name from the catalog
string args_json        # arguments as JSON; may contain placeholders such as "{fullest_stack}"
float32 timeout_s       # 0 = mission default
int32 retries           # -1 = mission default
string on_failure       # "retry" | "replan" | "skip" | "abort"; empty = mission default
string kind             # a move step: "navigate" | "straight" | "hand" | "posture" | "joints"; else empty
string[] resources      # what the step uses ("base", "hands", "torso", "joints"), filled when the plan is validated
string origin_json      # where the step came from, JSON: {"include": [names of the includes, outermost first; [] for the mission's own steps], "step": index in the innermost mission's own steps, "top": index of the step, or of the include step it came from, in the top mission's own steps}; empty = not from a mission
string replan_from      # on a replan, the step id the plan restarts from; empty = the step itself
int32 max_replans       # replans (and on_failure retries) the step may use before the plan fails; mission default applied
