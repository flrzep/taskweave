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
SKILL_ACTION_PREFIX = f'{NAMESPACE}/skills/'
SKILL_SERVER_CATALOG_SUFFIX = '/list_skills'  # each skill server serves <node>/list_skills


def skill_action(skill: str) -> str:
    return SKILL_ACTION_PREFIX + skill
