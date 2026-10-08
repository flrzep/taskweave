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
SKILL_ACTION_PREFIX = f'{NAMESPACE}/skills/'
SKILL_SERVER_CATALOG_SUFFIX = '/list_skills'  # each skill server serves <node>/list_skills


def skill_action(skill: str) -> str:
    return SKILL_ACTION_PREFIX + skill
