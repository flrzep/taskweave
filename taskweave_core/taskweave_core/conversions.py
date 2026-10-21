"""Conversions between taskweave's Python types and taskweave_interfaces messages."""

from __future__ import annotations

import json

from taskweave_interfaces.msg import Plan as PlanMsg
from taskweave_interfaces.msg import PlanStep, SkillInfo, TaskStatus

from .executive import Status
from .plan import Plan, Step


def step_to_msg(step: Step | None) -> PlanStep:
    msg = PlanStep()
    if step is None:
        return msg
    msg.step_id, msg.instruction, msg.skill = step.id, step.instruction, step.skill
    msg.args_json = json.dumps(step.args, sort_keys=True)
    msg.timeout_s, msg.retries, msg.on_failure = float(step.timeout_s), int(step.retries), step.on_failure
    if hasattr(msg, 'resources'):   # taskweave_interfaces of this version (older installs lack the two fields)
        msg.kind, msg.resources = step.kind or '', list(step.resources)
    return msg


def plan_to_msg(plan: Plan) -> PlanMsg:
    msg = PlanMsg()
    msg.plan_id, msg.goal, msg.robot, msg.loop = plan.plan_id, plan.goal, plan.robot, plan.loop
    msg.steps = [step_to_msg(step) for step in plan.steps]
    return msg


def status_to_msg(status: Status, stamp, now: float) -> TaskStatus:
    msg = TaskStatus()
    msg.stamp = stamp
    msg.state = status.state.value
    msg.plan_id, msg.goal, msg.cycle = status.plan_id, status.goal, max(status.cycle, 0)
    msg.step_index, msg.step_count = status.step_index, status.step_count
    msg.step = step_to_msg(status.step)
    msg.attempt = max(status.attempt, 0)
    msg.step_status, msg.step_progress = status.step_status, float(status.step_progress)
    msg.step_elapsed_s = float(now - status.step_started) if status.step else 0.0
    msg.last_error = status.last_error
    return msg


def skill_info_to_dict(info: SkillInfo) -> dict:
    return {'name': info.name, 'description': info.description,
            'args_schema': json.loads(info.args_schema_json or '{}'), 'needs': list(info.needs),
            'achieves': list(info.achieves), 'implementation': info.implementation}
