"""move_object and set_posture: the held-object skills of the new skill model (skill-api-spec 1.2, 1.3, 2, 3).

``move_object`` moves the object the robot holds with the hands that hold it (one or two), every hand keeping its grip
on it. The target is one of: an offset from where the object is (``by``, with turn and tilt), a position (``at``), a
named held pose of the robot for this kind of object (``pose``), or a contact of a placed object (``onto`` +
``contact``). ``motion.mode``:

    linear   (default) the hands on straight lines, the object rigid between them (the old move_object).
    free     joint space: IK for every hand's goal, then the path of a free move (straight when free, elbow swing first,
             else the planner). With two or more hands the sampled path is checked by FK: the distances between the
             hands must stay within ``grip_drift_m`` (skill yaml, default 0.02 m) of what they are now, else the move
             fails before anything moves and the message says to use linear.

``set_posture`` changes the arm shape while the hands stay: the elbows of the holding hands (else of all hands) swing
toward a side along the arms' null space. A held object does not move. It fails before moving when the swing would stop
further than ``tolerance_deg`` from the side.

In a cell in the targets layout these classes are ``move.carry`` and ``move.posture`` (``skills/move/carry.yaml``,
``skills/move/posture.yaml``). ``Move`` resolves the targets and gives them in ``args[RESOLVED]``: ``held`` (a carry
pose: its ``joints`` and ``path`` of Pose3D, ``args.to.pose`` names it) or ``onto`` (a contact: the placed ``instance``,
its ``pose`` and the held object's own contact ``mine``; ``args.to.onto`` and ``to.contact`` name them). A hand move of
a holding hand is a carry too, where the hand goes decides where the object goes (its grasp kept): ``hand`` (a hand of
the hold; ``args.to`` by / at as a hand move takes them: frame, turn, tilt, rpy_deg) or ``hand_target`` (a hand target
as ``move.hand`` gets it) with ``hands`` (the hands of the hold the target has: their goals must agree within
``grip_drift_m``); a target with waypoints is carried along them.

Torso look-ahead (P6-10b): a carry with ``torso: auto`` run by the executive (``ctx.context``; ``Move`` gives the part
the steps ahead as ``ahead``) plans the torso heights of its whole hold window before it moves (torso_window predicts
the steps, torso_plan plans them; profile ``ik.torso.lookahead``) and keeps the plan for the later steps of the window:
each takes its planned height while the state is the planned one, re-checks it at that height, and plans again from
where it is otherwise (a retry, a start mid-plan, a jog). A plan that proves the hold impossible (``Plan.certain``)
refuses the step before it moves; one stopped by the time budget, capped or not certain falls back to the per-step
choice. A change step refines its planned height when it runs (ik.torso.refine). ``data.torso_plan`` says what happened.

Both reuse the helpers of the joint skills (_onto, _swing_plan, the linear and free move paths, joints.py) and
the motion options of gripper_motion (skill yaml, robot override, step: ``taskweave_core.motion``). The old
move_object (flat args by / at / pose / to + contact / elbow) is gone: ``taskweave config migrate-skills``
rewrites missions and skills/move_object.yaml, set_posture.yaml is new.
"""

from __future__ import annotations

import copy
import time

import numpy as np

from taskweave_core import motion
from taskweave_core.config.schemas import ELBOW_SIDES, ROBOT_PROFILE, VEC3, default, hands_schema, motion_schema
from taskweave_core.skill import SkillResult

from .gripper_motion import RESOURCES, MoveGripper, Target, hands_arg
from .joints import _ObjectMotion
from .parts import RESOLVED, in_move

OBJECT_MODE = 'linear'                    # the mode of move_object whose yaml has no ``motion.mode``
HOLD_RULES = ('auto', 'all')              # hands words of the held-object skills: the hold's hands, all hands
LOOKAHEAD = {key: default(ROBOT_PROFILE, 'ik', 'torso', 'lookahead', key)        # ik.torso.lookahead defaults
             for key in ('enabled', 'budget_s', 'max_checks', 'torso_m', 'object_m', 'branch_rad')}
# ik.torso.lookahead.refine has no fixed default: it falls back to ik.torso.refine (F17), then REFINE
REFINE = default(ROBOT_PROFILE, 'ik', 'torso', 'refine')                          # ik.torso.refine default


class _HeldMotion(MoveGripper, _ObjectMotion):
    """Shared setup of the held-object skills: the motion settings of move_gripper (speeds, accel, torso fallback) and
    the held-object helpers of the old move_object."""

    hand_rules = HOLD_RULES

    def setup(self, node, config, robot, params):
        super().setup(node, config, robot, params)
        self.args_schema = copy.deepcopy(self.args_schema)
        self.args_schema['properties']['hands'] = hands_schema(self.hands, self.hand_rules)

    def _unfit_for(self, mode: str, planner: str):
        """A REJECTED result if this robot cannot do what the options ask for."""
        if self.ik is None:
            return SkillResult.rejected(f'mode {mode} needs the profile ik section')
        if planner == 'planned' and self.planner is None:
            return SkillResult.rejected('planner planned needs the profile planning section')
        return None

    def _watch(self, ctx, hands: list, goals: dict, label: str, m: dict):
        """After the move: a hand that ended further than ``motion.tolerance`` (m) from its goal fails the step."""
        miss = self._misses(Target(label, hands, {hand: _target_of(goal) for hand, goal in goals.items()}),
                            m.get('tolerance'))
        return SkillResult.failed(f'{label}: {miss}') if miss else None


def _target_of(goal: np.ndarray) -> dict:
    from .points import target_of
    return target_of(goal)


class MoveObject(_HeldMotion):
    name = 'move_object'
    description = ('Move the object held with the grippers (grip close at its point first) with the hands that hold '
                   'it: every hand keeps its grip, locked in orientation and distance to the object. Target (to): by '
                   'an offset in the robot frame (x forward, y left, z up) with optional turn about the vertical and '
                   'tilt (far side up); at a position (where the object origin goes, orientation kept); pose, a '
                   'named held pose of this robot for this kind of object (e.g. carry: torso first, then straight '
                   'lines); or onto a contact of a placed object (the held object contact goes there, offset_m short '
                   'of it). motion.mode linear: straight lines (default); free: joint-space path with a planner, '
                   'with two hands checked so the hands do not drift apart (otherwise use linear).')
    args_schema = {
        'type': 'object',
        'required': ['to'],
        'additionalProperties': False,
        RESOURCES: ['hands', 'torso'],
        'properties': {
            'to': {'description': 'Where the held object goes.', 'oneOf': [
                {'title': 'by an offset', 'type': 'object', 'required': ['by'], 'additionalProperties': False,
                 'properties': {
                     'by': {**VEC3, 'description': '[x, y, z] in metres, robot frame (x forward, y left, z up).'},
                     'turn_deg': {'type': 'number',
                                  'description': 'Turn about the vertical through the object, degrees.'},
                     'tilt_deg': {'type': 'number',
                                  'description': 'Tilt about the robot y axis through the object, far side up, '
                                                 'degrees.'}}},
                {'title': 'to a position', 'type': 'object', 'required': ['at'], 'additionalProperties': False,
                 'properties': {
                     'at': {**VEC3, 'description': '[x, y, z] in metres, robot frame: where the object origin goes. '
                                                   'Orientation kept.'}}},
                {'title': 'held pose', 'type': 'object', 'required': ['pose'], 'additionalProperties': False,
                 'properties': {
                     'pose': {'type': 'string', 'x-taskweave-point': 'target', 'x-taskweave-target-kinds': ['object'],
                              'description': 'A held pose of this robot for the object type '
                                             '(robots/<robot>/held_poses.yaml), e.g. carry.'}}},
                {'title': 'onto a contact', 'type': 'object', 'required': ['onto', 'contact'],
                 'additionalProperties': False,
                 'properties': {
                     'onto': {'type': 'string', 'x-taskweave-point': 'target', 'x-taskweave-target-kinds': ['object'],
                              'description': '<instance>/<contact>: where on a placed object the held object goes.'},
                     'contact': {'type': 'string', 'description': "The held object's contact that goes there."},
                     'offset_m': {'type': 'number', 'minimum': 0, 'x-taskweave-unit': 'm',
                                  'description': 'Stop this far before the contact, against its engage '
                                                 'direction.'}}}]},
            'object': {'type': 'string', 'x-taskweave-point': 'held_object',
                       'description': 'Only needed when several objects are held: which one (placed-object name).'},
            'hands': hands_schema((), HOLD_RULES),
            'motion': motion_schema(['mode', 'speed', 'speed_abs', 'accel', 'planner', 'tolerance', 'torso']),
        },
    }
    needs = ['an object held with the grippers']
    achieves = ['the held object moved']
    default_timeout_s = 60.0
    ahead = None            # Move: a function giving the steps after this one (Move._ahead), while a step runs

    # ---- running -------------------------------------------------------------
    def execute(self, ctx, args):
        to = args['to']
        self._look = None
        hold = self._hold(args)
        if isinstance(hold, SkillResult):
            return hold
        m = motion.values(motion.resolve(self.config, self.robot.name, self.name, args))
        mode, planner = m.get('mode', OBJECT_MODE), m.get('planner', 'auto')
        unfit = self._unfit_for(mode, planner)
        if unfit is not None:
            return unfit
        resolved = args.get(RESOLVED) or {}
        goal, legs = None, None
        if resolved.get('hand') or resolved.get('hand_target'):
            legs = self._hand_goals(hold, to, resolved)
            if isinstance(legs, SkillResult):
                return legs
        elif 'pose' not in to:
            goal = self._goal(hold, to, resolved.get('onto'))
            if isinstance(goal, SkillResult):
                return goal
        self._refresh_ik()
        self.torso_reports = []            # per move: the torso height chosen and the rejected ones (ik.torso)
        refused = self._lookahead(ctx, hold, args, m)
        if refused is not None:
            return refused
        saved = self.p, self.tcp_speed, self.tcp_turn, self.plan_scale
        self._speeds(m)
        try:
            if legs is not None:
                result = self._carry_legs(ctx, hold, legs, m, mode, planner)
            elif goal is None:
                result = self._carry_pose(ctx, hold, to['pose'], m, mode, planner, resolved.get('held'))
            else:
                result = self._carry(ctx, hold, goal[0], goal[1], m, mode, planner, ahead_pose=goal[2])
        finally:
            self.p, self.tcp_speed, self.tcp_turn, self.plan_scale = saved
        if isinstance(result, SkillResult):
            return self._with_plan(self._with_torso(result))
        label, worst, kind = result
        torso = {'torso': self.torso_reports} if self.torso_reports else {}
        height = f', torso {self.torso_reports[-1]["height"]:g}' if self.torso_reports else ''
        planned = ' planned' if self.torso_reports and self.torso_reports[-1].get('planned') else ''
        return self._with_plan(SkillResult.succeeded(
            f'moved {label} (max error {worst:.3f}{height}{planned})', world_state={'held_object': hold['object']},
            held_object=hold['object'], hands=hold['hands'], mode=mode, path=kind, max_error=float(worst), motion=m,
            **torso))

    # ---- torso look-ahead (P6-10b) ----------------------------------------------------------------------
    def _with_plan(self, result: SkillResult) -> SkillResult:
        """The result with ``data.torso_plan`` (what the look-ahead did for this step), if it did anything."""
        report = (getattr(self, '_look', None) or {}).get('report')
        if report and 'torso_plan' not in result.data:
            result.data['torso_plan'] = report
        return result

    def _lookahead(self, ctx, hold: dict, args: dict, m: dict):
        """This held step's height from the torso plan of its hold window (``self._look['height']``; none: the
        per-step choice). Only with the executive's context, torso auto, ik.torso and ik.torso.lookahead.enabled. The
        plan made at an earlier step of the window is kept (``self.kept``): this step takes its height while the state
        is the planned one (Kept.mismatch), a change step refines it (``_refined``); else, or with no plan for this
        window, the window is planned from here. A REJECTED result when the plan proves the hold impossible."""
        self._look = None
        context = getattr(ctx, 'context', None) or {}
        spec = self.ik.torso if self.ik is not None else None
        tune = {**LOOKAHEAD, **((spec or {}).get('lookahead') or {})}
        if not context.get('plan_id') or not spec or not tune['enabled'] or m.get('torso', 'keep') != 'auto':
            return None
        current = self._start()
        if current is None or spec['joint'] not in current:
            return None
        sid = getattr(ctx, 'step_id', '') or f'step {int(context.get("step_index", 0)) + 1}'
        key = (context.get('plan_id'), context.get('cycle'), hold['object'])
        self._look = look = {'id': sid, 'key': key, 'call': args, 'tune': tune, 'replanned': False, 'height': None}
        kept = (getattr(self, 'kept', None) or {}).get(key)
        if kept is not None and kept.covers(sid, [str(a.get('id', '')) for a in context.get('ahead') or []]):
            if kept.plan.certain:    # proven impossible (the reach bound): a retry gets the same refusal
                look['report'] = {**kept.plan.report(), 'source': 'kept', 'from': kept.first}
                refused = SkillResult.rejected(kept.plan.message)
                refused.data['torso_plan'] = look['report']
                return refused
            height = kept.plan.heights.get(sid)
            if height is None:       # the plan ends before this step (cut short, stopped, capped): as without it
                look['report'] = {'source': 'per step', 'why': f'the plan made at {kept.first} does not reach {sid}'}
                return None
            why = kept.mismatch(sid, current, hold, spec['joint'], self.ik.joint_pattern, tune)
            if why is None:
                look['report'] = {'source': 'kept', 'from': kept.first, 'height': round(height, 4)}
                look['height'] = self._refined(ctx, kept, sid, current, height)
                return None
            return self._plan_window(ctx, 'replanned', why)
        return self._plan_window(ctx, 'planned')

    def _plan_window(self, ctx, source: str, why: str = ''):
        """Plan the torso over the hold window from this step and the state now (``self._look``): the window
        predicted (torso_window.predict, the steps ahead from ``self.ahead``), the plan within ik.torso.lookahead
        budget_s and max_checks, kept for the later steps. A REJECTED result if the plan is certain, else None."""
        from .torso_plan import TorsoPlan
        from .torso_window import Kept, predict

        look = self._look
        hold, current = self._hold(look['call']), self._start()
        if isinstance(hold, SkillResult) or current is None:
            return None
        try:
            base_now = self._base_in_cell()
        except LookupError:
            base_now = None
        to = look['call'].get('to') or {}
        label = to.get('pose') or to.get('onto') or ' '.join(k for k in ('by', 'at') if k in to)
        window = predict(self, hold, {'id': look['id'], 'label': label, 'call': look['call']},
                         self.ahead() if callable(self.ahead) else [], base_now)
        world = self._world(base_now) if any(step.base is not None for step in window.steps) else None
        count = len(window.steps)
        index = {step.id: k for k, step in enumerate(window.steps, start=1)}
        ctx.feedback(0.0, f'planning the torso over {count} held moves')
        spec, tune = self.ik.torso, look['tune']
        began = time.monotonic()      # a compute budget (IK service calls take wall time), not a motion time

        def stop():
            return ctx.should_stop or time.monotonic() - began > float(tune['budget_s'])
        checks = self._torso_checks(ctx, world, lambda step: ctx.feedback(
            0.0, f'planning the torso over {count} held moves: {index.get(step.id, 0)}/{count}'), stop)
        band = getattr(checks, 'band', None)
        bands = [band(step) for step in window.steps] if band else []    # F17: heights only the global search found
        grid = [*spec['candidates'], *(band[0] for band in bands if band)]
        memory = self.__dict__.setdefault('torso_memory', {})
        planner = TorsoPlan(grid, tolerance=self.p.position_tolerance, memory=memory,
                            max_checks=int(tune['max_checks']))
        plan = planner.plan(window.steps, float(current[spec['joint']]), current, checks,
                            name=f'the hold of {hold["object"]}', should_stop=stop)
        if not plan.certain and window.later:     # after a posture move: the reach bound only (no IK, no time)
            plan = planner.hopeless(window.later, float(current[spec['joint']]), checks,
                                    name=f'the hold of {hold["object"]}') or plan
        self.kept = {look['key']: Kept(plan, window, current, look['id'], source, world)}
        report = {**plan.report(), 'source': source, **({'why': why} if why else {}),
                  **({'end': window.end} if window.end else {}), **({'until': window.cut} if window.cut else {})}
        look.update(report=report, replanned=look['replanned'] or source == 'replanned',
                    height=plan.heights.get(look['id']) if plan.ok else None)
        if plan.certain:      # before anything moved: REJECTED; a later step of the hold, things moved: FAILED
            refused = (SkillResult.failed if source == 'replanned' else SkillResult.rejected)(plan.message)
            refused.data['torso_plan'] = report
            return refused
        return None

    def _torso_checks(self, ctx, world=None, progress=None, stop=None):
        """The checks a torso plan of this skill calls (torso_plan.SkillChecks); ``stop``: default the step's."""
        from .torso_plan import SkillChecks

        return SkillChecks(self, stop or (lambda: ctx.should_stop), world=world, progress=progress)

    def _world(self, base_now):
        """torso_window.World from MoveIt's robot state now (its virtual joint), or None: the checks of a later
        step then see the base where it stands now."""
        from .collision import HoldCollision
        from .torso_window import World

        found = HoldCollision.for_node(self.node, self.robot, self.hold).world_joint()
        if found is None or base_now is None:
            return None
        joint, frame, virtual = found
        try:
            return World(joint, virtual, self.hold.transform(frame or self.cell_frame, self.robot.base['frame']))
        except LookupError:
            return None

    def _refined(self, ctx, kept, sid: str, current: dict, height: float) -> float:
        """At a change step of the kept plan (not the step that planned: its time was the budget's), F17's refine
        between the planned height and the height before (TorsoPlan.refine, ik.torso.lookahead.refine halvings, one
        finest step of margin from a failing height) over the steps that keep it, predicted again from the real hold
        and base (the kept window was predicted where the plan was made), within the lookahead budget (cut short:
        the planned height). The kept plan takes the refined height for them. The height to take."""
        from .torso_plan import TorsoPlan
        from .torso_window import predict

        tune = self._look['tune']
        halvings = int(tune.get('refine', self.ik.torso.get('refine', REFINE)))
        change = next((c for c in kept.plan.changes if c['at'] == sid), None)
        if change is None or sid == kept.first or halvings <= 0 or not callable(self.ahead):
            return height
        hold = self._hold(self._look['call'])
        if isinstance(hold, SkillResult):
            return height
        try:
            base_now = self._base_in_cell()
        except LookupError:
            return height
        window = predict(self, hold, {'id': sid, 'label': change['at'], 'call': self._look['call']}, self.ahead(),
                         base_now)
        later = {c['at'] for c in kept.plan.changes} - {sid}
        run = []
        for step in window.steps:           # the steps that keep the change's height: to the next change
            if step.id in later or step.id not in kept.plan.heights:
                break
            run.append(step)
        began = time.monotonic()

        def stop():
            return ctx.should_stop or time.monotonic() - began > float(tune['budget_s'])
        world = self._world(base_now) if any(step.base is not None for step in run) else None
        planner = TorsoPlan([*self.ik.torso['candidates'], height], tolerance=self.p.position_tolerance,
                            memory=self.__dict__.setdefault('torso_memory', {}))
        found = planner.refine(run, height, current, self._torso_checks(ctx, world, stop=stop),
                               float(current[self.ik.torso['joint']]), halvings, stop)
        if found is None:
            return height
        better, states = found
        for step in run:
            kept.plan.heights[step.id] = better
        kept.plan.states.update(states)
        change['to'] = round(better, 4)
        after = [c for c in kept.plan.changes if c['at'] in later and kept.window.ids.index(c['at']) >
                 kept.window.ids.index(sid)]
        if after:                            # the next change now starts from the refined height
            min(after, key=lambda c: kept.window.ids.index(c['at']))['from'] = round(better, 4)
        self._look['report'] |= {'height': round(better, 4), 'refined': {'from': round(height, 4),
                                                                         'to': round(better, 4)}}
        return better

    def _ride(self, ctx, chosen, label: str, planner: str, planned: bool):
        """_set_torso; a planned height rides as its plan checked it: the arms keep their joints while the column
        carries the held hands (``keep_arms``, joints._widen), not re-solved by IK."""
        self.keep_arms = planned
        try:
            return self._set_torso(ctx, chosen, label, planner)
        finally:
            self.keep_arms = False

    def _planned_choice(self, ctx, search):
        """(choice, planned) of a held leg: at the planned height when there is one, checked here (``search(h)``);
        if that fails, the window is planned again once from here; with no planned height left, the per-step choice
        (``search(None)``). A re-plan that proves the hold impossible fails the step."""
        look = getattr(self, '_look', None)
        while look is not None and look.get('height') is not None:
            chosen = search(look['height'])
            if not isinstance(chosen, SkillResult):
                chosen.skipped = 'planned by the torso look-ahead'
                return chosen, True
            if look['replanned']:
                break
            refused = self._plan_window(ctx, 'replanned', f'{look["height"]:g} failed here: {chosen.message}')
            if refused is not None:
                return SkillResult.failed(refused.message, torso_plan=look.get('report')), False
        if look is not None:
            look['height'] = None
        return search(None), False

    def dry_run(self, ctx, args):
        """Resolve the target and solve IK for the held object at it, move nothing: the joint goal the move would
        send (``Move``'s ``dry_run``). A carry pose is solved at its last waypoint, with its joints; the arms are
        solved for every hand of the hold. ``data.joint_goal`` has the joints set and the arms' solution."""
        from .ik import IkError, rpy_matrix

        to = args['to']
        hold = self._hold(args)
        if isinstance(hold, SkillResult):
            return hold
        m = motion.values(motion.resolve(self.config, self.robot.name, self.name, args))
        mode, planner = m.get('mode', OBJECT_MODE), m.get('planner', 'auto')
        unfit = self._unfit_for(mode, planner)
        if unfit is not None:
            return unfit
        resolved = args.get(RESOLVED) or {}
        joints: dict = {}
        if resolved.get('hand') or resolved.get('hand_target'):
            legs = self._hand_goals(hold, to, resolved)
            if isinstance(legs, SkillResult):
                return legs
            moved, label = legs[-1]
        elif 'pose' in to:
            held = resolved.get('held')
            if held is None:
                return SkillResult.rejected(f'a dry run needs the resolved held pose {to["pose"]}')
            joints, label = dict(held['joints']), f'{hold["object"]} to {to["pose"]}'
            if not held['path']:
                return SkillResult.succeeded(f'{label}: joint goal resolved (nothing moved)', dry_run=True,
                                             hands=hold['hands'], joint_goal=joints, legs=[])
            moved = rpy_matrix(held['path'][-1].xyz, held['path'][-1].rpy)
        else:
            goal = self._goal(hold, to, resolved.get('onto'))
            if isinstance(goal, SkillResult):
                return goal
            moved, label = goal[0], goal[1]
        tools = {hand: _target_of(moved @ pose) for hand, pose in hold['tcp_in_object'].items()}
        self._refresh_ik()
        start = self._start()
        if start is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)')
        leg = Target(label, list(tools), tools, joints=joints)
        if m.get('torso', 'keep') == 'auto' and self._searches(leg):
            chosen = self._torso_choice(ctx, leg, m, mode, 0.0, label, flip=False)
            if isinstance(chosen, SkillResult):
                return chosen
            goal_joints = {**chosen.fixed, **chosen.solution}
        else:
            try:
                goal_joints = {**joints, **self.ik.solve(tools, joints, lambda: ctx.should_stop, current=start,
                                                         allow_flip=False, elbow=m.get('elbow'))}
            except IkError as error:
                return SkillResult.failed(f'{label}: {error}')
        return SkillResult.succeeded(f'{label}: IK solved (nothing moved)', dry_run=True, hands=hold['hands'],
                                     mode=mode, joint_goal=goal_joints,
                                     legs=[{'label': label, 'joint_goal': goal_joints}])

    def _hold(self, args: dict):
        """The hold to move: the one ``args.object`` names, or the only one (or the one the named hands are in). A
        REJECTED result if there is none, several, or ``hands`` is not exactly the hands of the hold."""
        self.hold.reanchor()      # the arms may have moved since the stored pose (jogging)
        holds = self.hold.held_all()
        if not holds:
            return SkillResult.rejected('nothing is held: move to a hand target, then grip close'
                                        if in_move(self) else
                                        'nothing is held: move_gripper to the object, then grip close')
        rule, name = hands_arg(args.get('hands')), args.get('object')
        named = None if rule in HOLD_RULES else list(rule)
        if name:
            hold = next((h for h in holds if h['object'] == name), None)
            if hold is None:
                return SkillResult.rejected(f'{name} is not held (held: {", ".join(h["object"] for h in holds)})')
        else:
            candidates = [h for h in holds if named is None or set(h['hands']) & set(named)]
            if not candidates:
                text = '; '.join(f'{h["object"]} by {" and ".join(h["hands"])}' for h in holds)
                return SkillResult.rejected(f'{" and ".join(named)} hold nothing (held: {text})')
            if len(candidates) > 1:
                return SkillResult.rejected('several objects held: name one with object:')
            hold = candidates[0]
        if named is not None and set(named) != set(hold['hands']):
            others = [hand for hand in hold['hands'] if hand not in named]
            if others:
                return SkillResult.rejected(f'{hold["object"]} is also held by {" and ".join(others)}: include them '
                                            'or open them')
            return SkillResult.rejected(f'{" and ".join(h for h in named if h not in hold["hands"])} does not hold '
                                        f'{hold["object"]}: hands must be the hands that hold it '
                                        f'({" and ".join(hold["hands"])})')
        return hold

    def _hand_goals(self, hold: dict, to: dict, resolved: dict):
        """[(the held object's pose in the base frame, a label) per leg] of a carry by a hand (see the module doc):
        where the hand goes, the held object goes with its grasp kept. A REJECTED / FAILED result if the target has no
        pose for the hands of the hold, or two hands of the hold would pull the object apart."""
        from .cartesian import matching_flip, tcp_targets
        from .ik import rpy_matrix
        from .points import moved_tcp

        grips = hold['tcp_in_object']
        now = {hand: hold['object_in_base'] @ grip for hand, grip in grips.items()}
        if resolved.get('hand'):
            hand = resolved['hand']
            if hand not in grips:
                return SkillResult.rejected(f'{hand} does not hold {hold["object"]}')
            if 'at' in to:
                goal = now[hand].copy()
                if to.get('rpy_deg') is not None:
                    goal = rpy_matrix(to['at'], np.radians(to['rpy_deg']))
                goal[:3, 3] = to['at']
                label = f'{hold["object"]} with {hand} to {[round(float(v), 3) for v in to["at"]]}'
            else:
                goal = moved_tcp(now[hand], to['by'], to.get('frame', 'robot'), float(to.get('turn_deg', 0.0)),
                                 float(to.get('tilt_deg', 0.0)))
                label = f'{hold["object"]} with {hand} by {[round(float(v), 3) for v in to["by"]]}' + \
                    (f', turn {to["turn_deg"]:g} deg' if to.get('turn_deg') else '') + \
                    (f', tilt {to["tilt_deg"]:g} deg' if to.get('tilt_deg') else '')
            return [(goal @ np.linalg.inv(grips[hand]), label)]
        hands = [hand for hand in resolved.get('hands') or [] if hand in grips]
        if not hands:
            return SkillResult.rejected(f'the hand target has no pose for the hands that hold {hold["object"]} '
                                        f'({" and ".join(hold["hands"])})')
        target = self._store_target(resolved['hand_target'], hands)
        if isinstance(target, SkillResult):
            return target
        legs = []
        for leg in [*target.waypoints, target]:
            goals = matching_flip(tcp_targets(leg.tools), now, self.hold.tool_frames)   # the grasp turned as it is
            poses = [goals[hand] @ np.linalg.inv(grips[hand]) for hand in hands]
            apart = max(float(np.linalg.norm(pose[:3, 3] - poses[0][:3, 3])) for pose in poses)
            if apart > self.p.grip_drift_m:
                return SkillResult.rejected(f'{leg.label}: the poses of {" and ".join(hands)} would put '
                                            f'{hold["object"]} {apart:.3f} m apart (more than grip_drift_m '
                                            f'{self.p.grip_drift_m:g}): name one hand with hands')
            legs.append((poses[0], f'{hold["object"]} with {" and ".join(hands)} to {leg.label}'))
        return legs

    def _carry_legs(self, ctx, hold: dict, legs: list, m: dict, mode: str, planner: str):
        """Carry the held object through ``legs`` [(4x4 base frame, label)], one move each: (label, worst, kind) of the
        last or a failure result."""
        hand, step = hold['hands'][0], None
        for moved, label in legs:
            step = self._carry(ctx, self.hold.hold_of(hand) or hold, moved, label, m, mode, planner)
            if isinstance(step, SkillResult):
                return step
        return step

    def _goal(self, hold: dict, to: dict, onto: dict | None = None):
        """(the held object's pose in the base frame after the move, a label, the pose the move after it goes to
        (onto with an offset: the contact itself) or None), or a REJECTED / FAILED result. ``onto``: what ``Move``
        resolved of an ``onto`` target (see the module doc)."""
        if 'onto' in to:
            extra = {RESOLVED: onto} if onto else {}
            moved = self._onto(hold, {'to': to['onto'], 'contact': to['contact'], 'offset_m': to.get('offset_m', 0.0),
                                      **extra})
            if isinstance(moved, SkillResult):
                return moved
            final = None
            if to.get('offset_m'):
                final = self._onto(hold, {'to': to['onto'], 'contact': to['contact'], 'offset_m': 0.0, **extra})
                final = final if isinstance(final, np.ndarray) else None
            return moved, f'{hold["object"]} {to["contact"]} to {to["onto"]}' + \
                (f' (-{to["offset_m"]:g} m)' if to.get('offset_m') else ''), final
        from .points import moved_object

        moved = moved_object(hold['object_in_base'], to.get('by'), to.get('at'), float(to.get('turn_deg', 0.0)),
                             float(to.get('tilt_deg', 0.0)))
        return moved, f'{hold["object"]} ' + (f'to {[round(float(v), 3) for v in to["at"]]}' if 'at' in to else
                                              f'by {[round(float(v), 3) for v in to["by"]]}') + \
            (f', {to["turn_deg"]:g} deg' if to.get('turn_deg') else '') + \
            (f', tilted {to["tilt_deg"]:g} deg' if to.get('tilt_deg') else ''), None

    def _carry(self, ctx, hold: dict, moved: np.ndarray, label: str, m: dict, mode: str, planner: str,
               ahead_pose=None):
        """Move the held object to ``moved`` (4x4, base frame) with every hand of the hold: (label, worst joint error,
        path kind) or a failure result. Updates the hold afterwards."""
        goals = {hand: moved @ pose for hand, pose in hold['tcp_in_object'].items()}
        tools = {hand: _target_of(goal) for hand, goal in goals.items()}
        chosen, leg = None, Target(label, list(goals), tools)
        if m.get('torso', 'keep') == 'auto' and self._searches(leg):
            # one height for every hand of the hold (the torso carries them all), the held object goes with it
            ahead = {hand: _target_of(ahead_pose @ pose) for hand, pose in hold['tcp_in_object'].items()} \
                if ahead_pose is not None else None      # the contact itself, after an offset: the next line
            chosen, planned = self._planned_choice(ctx, lambda height: self._torso_choice(
                ctx, leg, m, mode, 0.0, label, flip=False, ahead=ahead, height=height))
            if isinstance(chosen, SkillResult):
                return chosen
            self.torso_reports.append({'leg': label, **chosen.report(), **({'planned': True} if planned else {})})
        if mode == 'linear':
            if chosen is not None:     # set the height first, the line keeps it; the object moved with the lift
                result = self._ride(ctx, chosen, label, planner, planned)
                failed = self._follow_holds([hold], label)
                if result is not None or failed is not None:
                    return result or failed
            result, worst = self._linear(ctx, {}, label, goals=goals)
            kind = 'linear'
        else:
            self.drift_hands = tuple(goals) if len(goals) > 1 else ()
            try:      # the hands' IK goals keep the grasp as it is (no flipped fingers on a held object)
                result, worst = self._reach(ctx, chosen.fixed if chosen is not None else {}, tools, label, flip=False,
                                            planner=planner)
            finally:
                self.drift_hands = ()
            kind = self.path_kind
        if result is not None:
            return result
        failure = self._watch(ctx, list(goals), goals, label, m)
        if failure is not None:
            return failure
        self.hold.moved(moved, hold['object'])
        return label, worst, kind

    def _torso_choice_path(self, ctx, hold: dict, waypoints: list, m: dict, mode: str, label: str,
                           height: float | None = None):
        """One torso height for every waypoint of a held pose's path (4x4 object poses, base frame): each waypoint is
        solved for all hands (the last one is the main sample), and in mode linear the legs are walked one after the
        other as the moves will (a leg that collides or runs out of reach at a height rejects it). ``height``: only
        this one is checked (planned by the torso look-ahead). A TorsoChoice or a FAILED result with the rejected
        heights."""
        from .cartesian import tcp_targets
        from .ik import IkError, Samples

        current = self._start()
        if current is None:
            return SkillResult.failed(f'no joint states on {self.joints.state_topic} (robot or sim not running?)')
        legs = [{hand: wp @ grip for hand, grip in hold['tcp_in_object'].items()} for wp in waypoints]
        tools = [{hand: _target_of(goal) for hand, goal in leg.items()} for leg in legs]
        samples = Samples([tools[-1], *tools[:-1]])
        if mode == 'linear':
            goals = [tcp_targets(t) for t in tools]
            samples.verify = lambda height: self._walk_lines(goals, current, height)[0]
        ctx.feedback(0.0, 'choosing the torso height')
        if height is not None and mode == 'linear':      # planned: walked as the moves will go
            return self._walked(goals, current, height, label)
        try:
            if height is not None:
                return self.ik.check_height(samples, height, current, {}, elbow=m.get('elbow'), allow_flip=False,
                                            should_stop=lambda: ctx.should_stop)
            return self.ik.solve_with_torso(samples, {}, current, lambda: ctx.should_stop, elbow=m.get('elbow'),
                                            allow_flip=False, held=True)
        except (LookupError, IkError) as error:
            return SkillResult.failed(f'{label}: {error}', torso=[{'leg': label, 'height': None,
                                                                   'rejected': getattr(error, 'rejected', [])}])

    def _carry_pose(self, ctx, hold: dict, name: str, m: dict, mode: str, planner: str, held: dict | None = None):
        """Held pose ``name`` for the held object's type: its joints (the lift, not the arms') first, then the
        waypoints one after the other, each like any other move of the object. ``held``: ``{joints, path}`` as
        ``Move`` resolved the target (else the robot's held pose of that name)."""
        from .ik import rpy_matrix

        kind = self.hold.type_of(hold, self.config.object_instances())
        if kind is None:
            return SkillResult.rejected(f'the held {hold["object"]} is not a placed object: its type is unknown')
        try:
            pose = held if held is not None else self.config.held_pose(self.robot.name, kind, name)
        except KeyError as error:
            return SkillResult.rejected(str(error.args[0]))
        arm = self._arm_joints(pose['joints'])
        if arm:
            return SkillResult.rejected(f'held pose {name} sets arm joints {arm}: only joints the held object moves '
                                        'with (e.g. a lift) can be set')
        label, worst, kind = f'{hold["object"]} to {name}', 0.0, 'direct'
        hand = hold['hands'][0]
        if self.ik is not None and self.ik.torso and self.ik.torso['joint'] in pose['joints']:
            m = {**m, 'torso': 'keep'}       # the pose sets the torso: the waypoints keep it
        elif m.get('torso', 'keep') == 'auto' and pose['path'] and self._searches(
                Target(label, list(hold['hands']), {h: {} for h in hold['hands']})):
            # one height for the whole path: every waypoint reachable, every leg a real line, then the legs keep it
            waypoints = [rpy_matrix(w.xyz, w.rpy) for w in pose['path']]
            chosen, planned = self._planned_choice(ctx, lambda height: self._torso_choice_path(
                ctx, hold, waypoints, m, mode, label, height=height))
            if isinstance(chosen, SkillResult):
                return chosen
            self.torso_reports.append({'leg': label, **chosen.report(), **({'planned': True} if planned else {})})
            result = self._ride(ctx, chosen, label, planner, planned)
            failed = self._follow_holds([hold], label)
            if result is not None or failed is not None:
                return result or failed
            m = {**m, 'torso': 'keep'}
        if pose['joints']:
            others = [h for h in self.hold.held_all() if h['object'] != hold['object']]
            result, worst = self._go(ctx, pose['joints'], f'{label} (joints)', planner=planner)
            failed = self._follow_holds(others, f'{label} (joints)')   # the lift moved the other holds' base frame too
            if result is not None:
                return result
            if failed is not None:
                return failed
            kind = self.path_kind
            try:    # the arms kept their joints: the object moved with them
                tcp = self.hold.tcp_poses([hand])[hand]
            except LookupError as error:
                return SkillResult.failed(f'{label}: where is the held object now? {error}')
            self.hold.moved(tcp @ np.linalg.inv(hold['tcp_in_object'][hand]), hold['object'])
        for index, waypoint in enumerate(pose['path'], start=1):
            hold = self.hold.hold_of(hand)
            step = self._carry(ctx, hold, rpy_matrix(waypoint.xyz, waypoint.rpy),
                               f'{label} ({index}/{len(pose["path"])})', m, mode, planner)
            if isinstance(step, SkillResult):
                return step
            _, worst, kind = step
        return label, worst, kind


class SetPosture(_HeldMotion):
    name = 'set_posture'
    description = ('Change the arm shape while the hands stay where they are: the elbows swing toward a side '
                   '(robot frame, out = away from the body) along the arms\' null space, as far as joint limits and '
                   'collisions allow. Default hands: the hands that hold something, else all. A held object does not '
                   'move. Fails before moving if the elbows would stop more than tolerance_deg off the side.')
    args_schema = {
        'type': 'object',
        'required': ['elbow'],
        'additionalProperties': False,
        RESOURCES: ['hands'],
        'properties': {
            'elbow': {'enum': ELBOW_SIDES,
                      'description': 'The hands and any held object stay; the arms swing their elbows toward this '
                                     'side (robot frame, out = away from the body), as far as joint limits and '
                                     'collisions allow.'},
            'tolerance_deg': {'type': 'number', 'exclusiveMinimum': 0, 'x-taskweave-unit': 'deg',
                              'description': 'Fail if the elbows would stop further than this from the side '
                                             '(default: skill config elbow_tolerance_deg).'},
            'hands': hands_schema((), HOLD_RULES),
            'motion': motion_schema(['speed', 'speed_abs', 'accel', 'tolerance']),
        },
    }
    needs = ['the hands stay in reach of where they are']
    achieves = ['the elbows toward the given side, the hands unchanged']
    default_timeout_s = 60.0

    def execute(self, ctx, args):
        side, rule = args['elbow'], hands_arg(args.get('hands'))
        holding = [hand for hand in self.hands if hand in self.hold.hand_holds]
        hands = (holding or list(self.hands)) if rule == 'auto' else list(self.hands) if rule == 'all' else list(rule)
        label = f'elbows of {" and ".join(hands)} {side}'
        m = motion.values(motion.resolve(self.config, self.robot.name, self.name, args))
        self._refresh_ik()
        saved = self.p, self.tcp_speed, self.tcp_turn, self.plan_scale
        self._speeds(m)
        try:
            try:
                starts = self.hold.tcp_poses(hands)
            except LookupError as error:
                return SkillResult.failed(f'{label}: where are the hands? {error}')
            failure, path, report, swung = self._swing_plan(ctx, hands, side, label, args.get('tolerance_deg'))
            if failure is not None:
                return failure
            data = {'hands': hands, 'mode': 'null_space', 'motion': m,
                    'report': {hand: {'from_deg': a, 'to_deg': b} for hand, (a, b, _) in report.items()}}
            if path is None:
                return SkillResult.succeeded(f'{label}: already as far as they go ({swung})', path='none', **data)
            goal = dict(zip(path.names, path.points[-1], strict=True))
            result, _ = self._move(ctx, goal, label, path=path)
            if result is not None:
                return result
            self._check_hands(starts, label)
            failure = self._watch(ctx, hands, starts, label, m)
            if failure is not None:
                return failure
        finally:
            self.p, self.tcp_speed, self.tcp_turn, self.plan_scale = saved
        return SkillResult.succeeded(f'{label}: {swung}', path='swing', **data)

