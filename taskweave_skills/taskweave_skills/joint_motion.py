"""move_joints: joint groups (torso, head, arms) to a named joint pose or to given joint values (skill-api-spec 1.4).

Replaces ``move_to_pose`` for joint poses. It reuses MoveToPose's setup, planner path (``_go``), IK client and the
phase-clock playback of ``_move`` (so the global speed override scales it); this module adds the target forms,
``only`` (joint groups), ``via`` waypoints, joint limits, the step's speed options and the hold rule.

Robot roles (profile.yaml): ``joints.groups`` {group: [joint, ...]} (torso, head, left_arm, ...; ``<tool>_arm`` groups
come from the ik joints regex when not listed), ``joints.limits`` or the URDF for the limits. Skill yaml
(skills/move_joints.yaml): ``speed: {joint_rad_s, joint_linear_m_s}`` (standard speeds), ``accel`` (unused: the step's
fraction stretches ``min_duration_s``), ``motion: {planner, ...}`` defaults, ``tolerances: {position, settle_s,
stall_timeout_s, state_timeout_s}``; the old flat Params keys work as well.
"""

from __future__ import annotations

from dataclasses import replace

from taskweave_core import motion
from taskweave_core.config.limits import arm_group_names, joint_groups, joint_limits, joint_problems
from taskweave_core.config.schemas import motion_schema
from taskweave_core.skill import SkillResult

from .joints import _PoseMotion
from .motion_plan import Trajectory

MOTION_KEYS = ('speed', 'speed_abs', 'accel', 'planner', 'tolerance', 'via')   # spec 1.4: always free, no mode
YAML_SPEEDS = {'joint_rad_s': 'max_joint_speed', 'joint_linear_m_s': 'max_linear_speed'}
YAML_TOLERANCES = {'position': 'position_tolerance'}   # settle_s, stall_timeout_s, state_timeout_s keep their names
YAML_ONLY = ('accel', 'motion', 'cell_frame', 'torso_fallback')   # new-layout keys this skill does not feed to Params


def _schema() -> dict:
    motion_options = motion_schema(MOTION_KEYS)
    motion_options['properties']['via'] = {   # joint poses and joint maps, passed in order
        'type': 'array',
        'items': {'anyOf': [{'type': 'string', 'x-taskweave-point': 'joint_pose_only'},
                            {'type': 'object', 'minProperties': 1, 'additionalProperties': {'type': 'number'}}]},
        'description': 'Waypoints to pass, in order: a joint pose name (poses.yaml) or a {joint: value} map.'}
    return {
        'type': 'object', 'additionalProperties': False,
        'oneOf': [{'title': 'Named pose', 'required': ['pose']}, {'title': 'Joint values', 'required': ['joints']}],
        'x-taskweave-resources': ['joints'],
        'properties': {
            'pose': {'type': 'string', 'x-taskweave-point': 'joint_pose_only',
                     'description': 'A named pose of this robot (robots/<robot>/poses.yaml) with joint values only '
                                    '(e.g. home, lift_high).'},
            'joints': {'type': 'object', 'minProperties': 1, 'additionalProperties': {'type': 'number'},
                       'description': '{joint name: value} in rad (metres for prismatic joints).'},
            'only': {'type': 'array', 'minItems': 1, 'uniqueItems': True,
                     'items': {'type': 'string', 'x-taskweave-point': 'joint_group'},
                     'description': 'Move only these joint groups of the target (profile joints.groups, e.g. torso, '
                                    'head, left_arm); the target\'s other joints stay as they are.'},
            'motion': motion_options}}


class _Rejected(Exception):
    """Invalid target or an unfit situation: nothing moved."""


class MoveJoints(_PoseMotion):
    name = 'move_joints'
    description = ('Move joint groups (torso, head, an arm) to a named joint pose or to given joint values and hold '
                   'them. `only` restricts the move to some groups, `motion.via` passes waypoints first, '
                   '`motion.speed` slows the step. With a motion planner the path avoids collisions; without one it '
                   'is a straight '
                   'joint-space move. A hand that holds an object stays with it: a move of its arm joints is REJECTED, '
                   'naming the hand. The other arm, torso and head may move (the holding arm keeps its joints and '
                   'the object moves with it).')
    args_schema = _schema()
    needs = ['the motion path is free of obstacles']
    achieves = ['joints at the target']

    def configure(self, params):
        super().configure(self._joint_params(params))
        self.base_p = self.p

    @staticmethod
    def _joint_params(params: dict) -> dict:
        """Params of the skill yaml: the new layout (speed/tolerances tables) mapped onto the Params names."""
        result = {k: v for k, v in params.items() if k not in YAML_ONLY and k not in ('speed', 'tolerances')}
        for key, name in YAML_SPEEDS.items():   # other speeds (tcp_m_s, ...) belong to the hand skills
            if key in (params.get('speed') or {}):
                result.setdefault(name, params['speed'][key])
        for key, value in (params.get('tolerances') or {}).items():
            result.setdefault(YAML_TOLERANCES.get(key, key), value)
        return result

    # ---- run ---------------------------------------------------------------------
    def execute(self, ctx, args):
        values = motion.values(motion.resolve(self.config, self.robot.name, self.name, args))
        try:
            legs, label = self._legs(args, values.get('via') or [])
            self._check(legs)
        except _Rejected as error:
            return SkillResult.rejected(str(error))
        self._refresh_ik()
        holds = self.hold.held_all()
        refusal = self._holding_refusal(joints={n for leg in legs for n in leg})
        if refusal is not None:
            return refusal
        self.hold.leave(list(self.hold.reached_hands()))   # the hands are no longer at an object point
        self._path_used, self._time_scale = 'direct', 1.0
        self.p = self._step_params(values)
        try:
            worst = 0.0
            for index, leg in enumerate(legs):
                step = label if index == len(legs) - 1 else f'{label} via {index + 1}'
                result, error = self._leg(ctx, leg, step, values.get('planner', 'auto'))
                if result is not None:
                    return result
                worst = max(worst, error)
        finally:
            self.p = self.base_p
            # the torso may have moved, however the motion ended (failed, interrupted): each object went with its hands
            failed = self._follow_holds(holds, label)
        if failed is not None:
            return failed
        stats = ctx.work.stats()
        data = {'hands': self._hands(legs), 'mode': 'free', 'path': self._path_used, 'max_error': round(worst, 4),
                'speed': {**stats, 'scale': round(self._time_scale * stats['override_avg'] / 100.0, 4)}}
        if 'pose' in args:
            data['world_state'] = {'arm_pose': args['pose']}
        return SkillResult.succeeded(f'at {label} (max error {worst:.3f})', **data)

    def _leg(self, ctx, goal: dict, label: str, planner: str):
        """One move to ``goal``. ``direct``: the straight joint move only, refused if the planner finds it blocked.
        ``auto`` / ``planned``: ``_go`` (the planner, which goes straight when that is free; without a planner a
        straight move). Returns (failure or None, worst joint error)."""
        if planner != 'direct':
            return self._go(ctx, goal, label)
        start = self._start()
        if start is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)'), 0
        if self.planner is not None and not self.planner.free(start, goal, lambda: ctx.should_stop):
            return SkillResult.failed(f'{label}: the straight joint move is blocked (planner: direct)'), 0
        result, final = self._move(ctx, goal, label)
        if result is not None:
            return result, 0
        return None, max(abs(goal[n] - final[n]) for n in goal)

    def _move(self, ctx, goal, label, stall_ok=False, path=None):
        """MoveToPose's playback; a planned path is timed at the step's speed (the planner's own timing is 100 %)."""
        if path is not None:
            self._path_used = 'planned'
            if self._time_scale != 1.0:
                path = Trajectory(path.names, [t / self._time_scale for t in path.times], path.points)
        return super()._move(ctx, goal, label, stall_ok, path)

    def _step_params(self, values: dict):
        """Params for this step: the standard speeds scaled by ``speed`` / replaced by ``speed_abs``, ``accel`` as a
        fraction of the minimum-duration ramp, ``tolerance``. Sets ``_time_scale`` (the rad/s speed ratio)."""
        base, fraction, absolute = self.base_p, values.get('speed'), values.get('speed_abs') or {}
        speeds = {name: motion.effective_speed(getattr(base, name), None, fraction, absolute.get(key))
                  for key, name in YAML_SPEEDS.items()}
        self._time_scale = speeds['max_joint_speed'] / base.max_joint_speed
        return replace(base, **speeds, min_duration_s=base.min_duration_s / values.get('accel', 1.0),
                       position_tolerance=values.get('tolerance', base.position_tolerance))

    # ---- targets -----------------------------------------------------------------
    def _legs(self, args: dict, via: list) -> tuple[list[dict], str]:
        """The joint goals to move to in order (``via`` waypoints, then the target), ``only`` applied, and a label."""
        if ('pose' in args) == ('joints' in args):
            raise _Rejected('give exactly one of: pose, joints')
        goal, label = self._joint_target(args['pose']) if 'pose' in args else (
            {n: float(v) for n, v in args['joints'].items()}, 'joints')
        goals = [self._joint_target(item.removeprefix('pose:'))[0] if isinstance(item, str)
                 else {n: float(v) for n, v in item.items()} for item in via]
        only = args.get('only')
        return [*(leg for leg in (self._only(g, only, False) for g in goals) if leg), self._only(goal, only)], label

    def _joint_target(self, pose: str) -> tuple[dict, str]:
        try:
            named = self.config.named_pose(self.robot.name, pose)
        except KeyError as error:
            raise _Rejected(str(error.args[0])) from error
        if named['tools']:
            raise _Rejected(f'pose {pose} places tools: use move_gripper to.pose')
        return named['joints'], pose

    def _only(self, goal: dict, only, strict: bool = True) -> dict:
        """``goal`` restricted to the joints of the groups ``only``; none left: REJECTED, or {} if not ``strict``
        (a via waypoint outside the groups is skipped)."""
        if not only:
            return goal
        groups = joint_groups(self.config, self.robot.name)
        missing = [g for g in only if g not in groups]
        if missing:
            raise _Rejected(f'robot has no group {", ".join(missing)} (has: {", ".join(sorted(groups)) or "none"})')
        keep = {joint for g in only for joint in groups[g]}
        kept = {n: v for n, v in goal.items() if n in keep}
        if not kept and strict:
            raise _Rejected(f'no joints of the target are in {", ".join(only)}')
        return kept

    def _check(self, legs: list[dict]) -> None:
        """Unknown joints and values outside the joint limits are REJECTED before anything moves."""
        limits = joint_limits(self.config, self.robot.name)
        known = set(limits) or set(self.joints.snapshot(self.p.state_timeout_s) or ())
        linear = set(self.p.linear_joints)
        for leg in legs:
            unknown = [n for n in leg if known and n not in known]
            if unknown:
                raise _Rejected(f'unknown joint {", ".join(unknown)}')
            problems = joint_problems(leg, limits, linear)
            if problems:
                raise _Rejected('outside the joint limits: ' + '; '.join(problems))

    def _arm_joint_names(self, joints: dict) -> set[str]:
        """The joints of ``joints`` that move a hand: members of the ``<tool>_arm`` groups, and the ik joints regex. A
        robot with neither groups nor ik: every joint (as MoveToPose counts them)."""
        groups = joint_groups(self.config, self.robot.name)
        arms = {joint for g in arm_group_names(self.config, self.robot.name) for joint in groups.get(g, ())}
        if self.ik is not None:
            arms |= set(self._arm_joints(joints))
        elif not groups:
            return set(joints)
        return arms & set(joints)

    def _hands(self, legs: list[dict]) -> list[str]:
        """The tools whose arm moved."""
        groups = joint_groups(self.config, self.robot.name)
        touched = {n for leg in legs for n in leg}
        return sorted(g.removesuffix('_arm') for g in arm_group_names(self.config, self.robot.name)
                      if touched & set(groups.get(g, ())))
