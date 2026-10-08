# Catalog entry for one skill. This is what planners (including a VLM planner) see.

string name             # e.g. "navigate_to"
string description      # one or two sentences, written for a planner to read
string args_schema_json # JSON Schema of the args object
string[] needs          # preconditions in plain words, e.g. "gripper empty"
string[] achieves       # effects in plain words, e.g. "robot at target"
string implementation   # "script" | "vla" | "rl" | ...
string action_name      # RunSkill action server that executes it
