# Global speed override, published latched by the task handler on /taskweave/speed_override on every change.
# Skills follow it (taskweave_core.speed.SpeedOverride); trajectories are computed at 100 % and played back slower.

float32 percent         # 1-100: scales speeds (accelerations by its square on the phase clock), timeouts by 1/percent
bool test_mode          # test mode latch: a run starts at test_percent, requests above cap_percent are clamped
float32 test_percent    # the override a run starts at in test mode (default 10)
float32 cap_percent     # the highest override allowed while test mode is on (default 50)
