"""ROS names used by taskweave. One place, so nodes, CLI and docs agree."""

NAMESPACE = '/taskweave'
START_MISSION = f'{NAMESPACE}/start_mission'
PAUSE = f'{NAMESPACE}/pause'
RESUME = f'{NAMESPACE}/resume'
ABORT = f'{NAMESPACE}/abort'
LIST_SKILLS = f'{NAMESPACE}/list_skills'
STATUS = f'{NAMESPACE}/status'
PLAN = f'{NAMESPACE}/plan'
EPISODES = f'{NAMESPACE}/episodes'
TEACH_POINT = f'{NAMESPACE}/teach_point'
DESCRIBE_ROBOT = f'{NAMESPACE}/describe_robot'
OPERATOR_MODE = f'{NAMESPACE}/operator_mode'  # pad mode: drive or a tool name (std_msgs/String, latched)
OPERATOR_NOTICE = f'{NAMESPACE}/operator_notice'  # what a pad press did, for the HMI (String, JSON {text, ok})
TEACH_TARGET = f'{NAMESPACE}/teach_target'  # the arm position open in the HMI (String, latched; JSON, '' = none)
TEACH_REQUEST = f'{NAMESPACE}/teach_request'  # pad teach press for the HMI's target (String, JSON {tools})
SPEED_OVERRIDE = f'{NAMESPACE}/speed_override'  # global speed override (SpeedOverride, latched)
SET_SPEED_OVERRIDE = f'{NAMESPACE}/task_handler/set_speed_override'  # its single write path (SetSpeedOverride)
SKILL_ACTION_PREFIX = f'{NAMESPACE}/skills/'
SKILL_SERVER_CATALOG_SUFFIX = '/list_skills'  # each skill server serves <node>/list_skills
RELOAD_CONFIG = f'{NAMESPACE}/task_handler/reload_config'  # ReloadConfig; the skill servers serve <node>/reload_config
RELOAD_CONFIG_SUFFIX = '/reload_config'


def skill_action(skill: str) -> str:
    return SKILL_ACTION_PREFIX + skill
