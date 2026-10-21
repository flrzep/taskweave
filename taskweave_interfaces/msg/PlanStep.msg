# One step of a plan. Produced by a planner, executed by the task handler through RunSkill.

string step_id          # unique within the plan
string instruction      # natural-language description (always set)
string skill            # skill name from the catalog
string args_json        # arguments as JSON; may contain placeholders such as "{fullest_stack}"
float32 timeout_s       # 0 = mission default
int32 retries           # -1 = mission default
string on_failure       # "retry" | "replan" | "skip" | "abort"; empty = mission default
string kind             # a move step: "base" | "straight" | "hand" | "object" | "posture" | "joints"; else empty
string[] resources      # what the step uses ("base", "hands", "torso", "joints"), filled when the plan is validated
