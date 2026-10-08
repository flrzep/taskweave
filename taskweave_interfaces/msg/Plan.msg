# An ordered list of steps for one goal.

string plan_id          # unique per plan, e.g. "frame_cycle-0007"
string goal             # the goal the plan was made for (natural language)
string robot            # robot profile the plan targets, e.g. "a2d"
string loop             # "once" | "forever" | "until_empty"
PlanStep[] steps
