# Published by the task handler on every state change and periodically while running.

builtin_interfaces/Time stamp

uint8 IDLE = 0
uint8 RUNNING = 1
uint8 PAUSED = 2
uint8 SUCCEEDED = 3
uint8 FAILED = 4
uint8 ABORTED = 5
uint8 state

string plan_id
string goal
uint32 cycle            # loop iteration, starting at 1
int32 step_index        # index into the plan, -1 when no step is active
uint32 step_count
PlanStep step           # the active step with placeholders resolved
uint32 attempt          # 1 for the first try, increments on retry
string step_status      # latest skill feedback status
float32 step_progress   # latest skill feedback progress, -1 if unknown
float32 step_elapsed_s
string last_error       # reason for the most recent failure, empty if none
