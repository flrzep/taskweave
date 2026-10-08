"""Basic skills that need no hardware."""

from taskweave_core.skill import Skill, SkillResult


class Wait(Skill):
    name = 'wait'
    description = 'Wait for the given number of seconds (simulation time when use_sim_time is set).'
    args_schema = {
        'type': 'object',
        'required': ['seconds'],
        'properties': {'seconds': {'type': 'number', 'minimum': 0}},
        'additionalProperties': False,
    }
    achieves = ['the given time has passed']

    def execute(self, ctx, args):
        seconds = float(args['seconds'])
        end = ctx.elapsed() + seconds
        while ctx.elapsed() < end:
            if not ctx.sleep(min(0.1, end - ctx.elapsed())):
                return ctx.interrupted()
            ctx.feedback(min(1.0, ctx.elapsed() / seconds) if seconds else 1.0, 'waiting')
        return SkillResult.succeeded()
