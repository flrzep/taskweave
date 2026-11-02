"""move: the one skill that moves the robot, in a cell in the targets layout (cell schema 4 and 5).

A step says what moves and where to (``args_schema``): ``to`` a target (its kind says what moves: the base, a hand, a
held object or joints), ``hand`` relative to now (``by`` / ``at``; ``hand: holding`` names the hands that hold),
``base`` straight by a distance, ``elbow`` to a side, ``joints`` to group values. ``Move`` resolves what the step asks
for (``ConfigStore.target``), checks the options that kind of motion takes (taskweave_core ``MOVE_OPTIONS``) and hands
the step to the module that does it, one of six parts (``parts.py``). The operator sees five commands, the kinds
(``data.kind``; labels: taskweave_core ``MOVE_LABELS``):

    command                  kind      step forms                                      part
    navigate to              navigate  base target (motion.mode nav, the default)      move.navigate  NavigateTo / Nav2
    move straight            straight  base target with motion.mode straight, base     move.straight  DriveStraight
    move to target           hand      hand target, hand + by / at                     move.hand      MoveGripper
                                       object target, hand: holding + by / at; a hand  move.carry     MoveObject
                                       target or hand + by / at of a holding hand
    change posture           posture   elbow                                           move.posture   SetPosture
    move to joint positions  joints    joint target, joints                            move.joints    MoveJoints

Carrying: a hand move whose hands hold something moves the held object with them (``move.carry``), a two-hand hold as
one (``hand: left, by`` with both hands on a frame moves the frame). An object target is where the held object goes;
``hand: holding`` + by / at moves the held object relative to now (cell schema 4 wrote ``object: held``). A hand target
or a relative move that names a holding hand (the hand goes there, the object keeps its grasp) needs the collision
shape of the held object's type (``objects/<type>.yaml`` ``collision:``): without one the step is REJECTED.

Each part is built from ``skills/move/<part>.yaml`` (implementations with ``requires``, parameters) like a skill of its
own; one that has no implementation for the robot answers REJECTED "this robot cannot do <part> moves" and the others
work. The parts are the classes of the former motion skills. ``Move`` gives a part what it resolved in
``args[RESOLVED]`` (``parts.py``), so it hands over the same targets the old skills made of their names.

The result is the part's, with ``kind`` (one of the five), ``module`` (``move.<part>``), ``target`` and, for a carry,
``carrying`` (the held instance) added to its data, and its message prefixed with the command and what moved ("move to
target · carrying frame_1 · hook"). ``dry_run: true`` resolves and solves, moves nothing and answers
``data.joint_goal`` (the joints at the end of the move, ``data.legs`` on the way): for a preview.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field

from taskweave_core import motion
from taskweave_core.config import Pose2D
from taskweave_core.config.limits import group_values
from taskweave_core.config.poses import Pose3D
from taskweave_core.config.schemas import ELBOW_SIDES, VEC3, hands_schema, motion_schema
from taskweave_core.config.targets import (
    FORM_OF_TARGET,
    HOLDING,
    MOVES_WORD,
    RELATIVE_KEYS,
    RELATIVE_OPTIONS,
    TOOL_SUFFIX,
    TORSO,
    GroupError,
    kind_of,
    move_label,
    move_option_problems,
)
from taskweave_core.executive import Outcome
from taskweave_core.skill import Skill, SkillResult, load_skill_class

from .parts import PARTS, RESOLVED, part_name

HAND_RULES = ('auto', 'any_one', 'all')
OPTIONS = ('hands', 'offset_m', 'reference', 'only', 'torso', 'tolerance_deg', 'motion')   # taskweave_core MOVE_OPTIONS
ORIGIN = Pose3D((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 'object')


def _schema() -> dict:
    options = motion_schema(('mode', 'speed', 'speed_abs', 'accel', 'elbow', 'planner', 'approach_m', 'via',
                             'tolerance'))
    props = options['properties']
    props['mode'] = {'enum': ['nav', 'straight', 'free', 'linear'],
                     'description': 'nav (default) / straight: a base target. free (default) / linear: a hand '
                                    'target. linear (default) / free: an object target. Not for the other forms.'}
    props['via'] = {'type': 'array', 'items': {'type': 'string', 'x-taskweave-point': 'target',
                                                'x-taskweave-target-kinds': ['hand', 'joints']},
                    'description': 'Waypoints to pass, in order: hand targets (hand move) or joint targets (joints '
                                   'move).'}
    either = [{'required': ['by']}, {'required': ['at']}]
    exclusive = {'x-taskweave-exclusive': ['by', 'at']}   # one of the two (the root's `not` enforces it)
    return {
        'type': 'object',
        'additionalProperties': False,
        'oneOf': [
            {'title': 'Go to a target', 'required': ['to']},
            {'title': 'Move hands relative to now', 'required': ['hand'], 'anyOf': either,
             **exclusive},
            {'title': 'Move a held object relative to now (cell schema 4; now hand: holding)',
             'required': ['object'], 'not': {'anyOf': [{'required': ['to']}, {'required': ['hand']}]},
             'anyOf': either, **exclusive},
            {'title': 'Drive the base straight', 'required': ['base']},
            {'title': 'Elbow side', 'required': ['elbow']},
            {'title': 'Joint values', 'required': ['joints']},
        ],
        'not': {'required': ['by', 'at'], 'x-taskweave-exclusive': ['by', 'at']},   # by or at, not both
        'x-taskweave-resources': ['base', 'hands', 'torso', 'joints'],
        'properties': {
            'to': {'type': 'string', 'x-taskweave-point': 'target',
                   'x-taskweave-target-kinds': ['base', 'hand', 'object', 'joints'],
                   'description': 'A target: <name> (a cell, base or joint target) or <instance>/<name> (a target '
                                  'of a placed object). What moves (base, hands, held object, joints) comes from '
                                  'the target.'},
            'hand': {'anyOf': [{'type': 'string'},
                               {'type': 'array', 'items': {'type': 'string'}, 'minItems': 1, 'uniqueItems': True}],
                     'x-taskweave-widget': 'hands',
                     'description': 'Which hands move relative to now (with by or at): a hand name, a list, all, or '
                                    'holding (the hands that hold, with the held object). A hand that holds carries '
                                    'what it holds.'},
            'object': {'type': 'string',
                       'description': 'Which held object (a placed-object name) when several are held, with `to` or '
                                      '`hand: holding`. Cell schema 4 wrote `object: held` + by / at for '
                                      '`hand: holding` + by / at.'},
            'base': {'type': 'object', 'required': ['by'], 'additionalProperties': False,
                     'properties': {'by': {'type': 'array', 'items': {'type': 'number'}, 'minItems': 2, 'maxItems': 2,
                                           'description': '[forward, left] in metres: a straight drive keeping '
                                                          'the heading (left only on a holonomic base).'}}},
            'elbow': {'anyOf': [{'enum': ELBOW_SIDES}, {'type': 'object', 'minProperties': 1,
                                                        'additionalProperties': {'enum': ELBOW_SIDES}}],
                      'description': 'The elbows swing toward this side, the hands stay: one side for the hands of '
                                     '`hands` (default: the holding hands, else all), or {hand: side}.'},
            'joints': {'type': 'object', 'minProperties': 1,
                       'additionalProperties': {'anyOf': [{'type': 'number'}, {'enum': ['keep']},
                                                          {'type': 'array',
                                                           'items': {'type': ['number', 'null']}}]},
                       'description': 'Joint group values: a number, or a list in the group\'s joint order '
                                      '(null leaves that joint where it is).'},
            'hands': hands_schema(),
            'offset_m': {'type': 'number', 'minimum': 0, 'x-taskweave-unit': 'm',
                         'description': 'Stop this far short of a hand or object target, back along its approach '
                                        '(engage) direction.'},
            'reference': {'type': 'string',
                          'description': 'An object target: which target of the held object\'s type goes there '
                                         '(default: the target\'s own reference, else the object\'s origin).'},
            'by': {**VEC3, 'description': '[x, y, z] in metres relative to now (hand or object).'},
            'at': {**VEC3, 'description': '[x, y, z] in metres, robot frame (hand: exactly one hand).'},
            'rpy_deg': {**VEC3, 'description': 'With `at`, hand: orientation in degrees, robot frame.'},
            'frame': {'enum': ['robot', 'tool'], 'default': 'robot',
                      'description': 'With `by`, hand: robot (x forward, y left, z up) or tool (along the TCP axes).'},
            'turn_deg': {'type': 'number', 'description': 'With `by`: turn about the vertical (or the approach axis '
                                                          'with frame tool), degrees.'},
            'tilt_deg': {'type': 'number', 'description': 'With `by`: tilt about the robot y axis, degrees.'},
            'only': {'type': 'array', 'minItems': 1, 'uniqueItems': True,
                     'items': {'type': 'string', 'x-taskweave-point': 'joint_group'},
                     'description': 'A joint target: move only these groups of it.'},
            'torso': {'anyOf': [{'enum': ['auto', 'keep']}, {'type': 'number'},
                                {'type': 'array', 'items': {'type': ['number', 'null']}}],
                      'description': 'The torso for this move: auto (choose a height; hand and object targets), '
                                     'keep (do not move it), or a value (a number, or a list for several joints).'},
            'tolerance_deg': {'type': 'number', 'exclusiveMinimum': 0, 'x-taskweave-unit': 'deg',
                              'description': 'Elbow: fail if the elbows would stop further than this from the side.'},
            'motion': options,
            'look_at': {'type': 'string', 'description': 'Reserved (the head looks at a target): not implemented '
                                                         'yet, set the head through joints.'},
            'dry_run': {'type': 'boolean', 'default': False,
                        'description': 'Resolve and solve IK, move nothing: the result has data.joint_goal.'},
        },
    }


@dataclass
class Plan:
    """What a step resolved to: the part that moves (its kind: ``kind_of(part)``) and the calls to make."""
    part: str                       # navigate, straight, hand, carry, posture, joints
    desc: str                       # 'waiting': what the messages say after the command
    target: str | None = None       # the reference as resolved
    calls: list[dict] = field(default_factory=list)       # the args of each call of the part, in order
    before: dict | None = None      # a joints leg first (base target with a joints part): args of move.joints
    dry: dict = field(default_factory=dict)               # what a dry run of a base or straight move answers
    carrying: str | None = None     # a carry: the held instance

    @property
    def kind(self) -> str:
        return kind_of(self.part)


class Move(Skill):
    name = 'move'
    description = ('Move the robot. Forms: `to` a target (its kind says what moves: a base target drives the base, a '
                   'hand target moves hands, an object target moves the held object, a joint target moves joints); '
                   '`hand` + `by` / `at` moves hands relative to now (`hand: holding`: the hands that hold, with the '
                   'held object); `base: {by: [forward, left]}` drives straight; `elbow` swings the elbows to a side '
                   'while the hands stay; `joints` moves joint groups to values. A hand that holds an object carries '
                   'it (both hands of a two-hand hold move as one); a hand target or a relative move of a holding '
                   "hand needs the collision shape of the held object's type. Options: hands, offset_m, reference, "
                   'only, torso (auto / keep / a value), tolerance_deg, motion (mode, speed, accel, planner, via '
                   '...); which apply depends on the kind of motion. dry_run resolves and solves without moving.')
    args_schema = _schema()
    needs = ['the target is in reach of where the robot is']
    achieves = ['the thing that moves is at the target']
    default_timeout_s = 120.0

    # ---- setup ----------------------------------------------------------------------------------------------
    def __copy__(self):
        """A copy for the skill server's try-out of a new configuration (``configure`` on it must change nothing)."""
        clone = type(self).__new__(type(self))
        clone.__dict__.update(self.__dict__)
        clone._trial = True
        return clone

    def setup(self, node, config, robot, params):
        self._trial = False
        self.modules: dict = {}
        self.unavailable: dict[str, list[str]] = {}
        self._part_params: dict[str, dict] = {}
        self._used: list = []
        super().setup(node, config, robot, params)
        for part in PARTS:
            key = part_name(part)
            implementation, reasons = config.select_implementation(key, robot.name)
            if implementation is None:
                self.unavailable[part] = reasons
                node.get_logger().info(f'{key}: not available: {"; ".join(reasons)}')
                continue
            module = load_skill_class(implementation['class'])()
            module.name = key
            module.implementation = f'{implementation["type"]}:{implementation["name"]}'
            self._part_params[part] = config.skill_params(key, implementation)
            try:
                module.setup(node, config, robot, dict(self._part_params[part]))
            except (KeyError, ValueError) as error:   # the robot lacks what the part is built on (no joints, ...)
                self.unavailable[part] = [f'the robot profile does not fit: {error.args[0] if error.args else error}']
                node.get_logger().info(f'{key}: not available: {self.unavailable[part][0]}')
                continue
            self.modules[part] = module
        self.hands = list(robot.data.get('grippers', {}))
        self.args_schema = self.schema_for(self.hands)
        served = ', '.join(f'{part} ({module.implementation})' for part, module in self.modules.items())
        node.get_logger().info(f'move parts: {served or "none"}')

    def configure(self, params):
        """The parts' parameters again (``skills/move/<part>.yaml``), tried on copies first. A parameter a part made a
        connection from, and its implementation, keep the values of the start (the part would need a restart)."""
        super().configure(params)
        if not getattr(self, 'modules', None):
            return
        fresh = {}
        for part, module in self.modules.items():
            key = part_name(part)
            implementation = self.config.select_implementation(key, self.robot.name)[0]
            new = dict(self.config.skill_params(key, implementation))
            old = self._part_params[part]
            for name in module.startup_params:    # a connection made at start keeps its value
                if new.get(name) != old.get(name):
                    self.node.get_logger().warning(f'{key}: {name} changed, it takes effect at the next restart')
                if name in old:
                    new[name] = old[name]
                else:
                    new.pop(name, None)
            probe = copy.copy(module)
            probe.robot = self.robot
            probe.configure(dict(new))
            fresh[part] = new
        if self._trial:
            return
        for part, module in self.modules.items():
            module.robot = self.robot
            module.configure(dict(fresh[part]))

    @classmethod
    def schema_for(cls, hands: list[str]) -> dict:
        """The args schema with the robot's hands in ``hand`` and ``hands`` (the class has the generic one)."""
        schema = copy.deepcopy(cls.args_schema)
        props = schema['properties']
        props['hands'] = hands_schema(hands, HAND_RULES)
        props['hand'] = {'anyOf': [{'enum': ['all', HOLDING, *hands]},
                                   {'type': 'array', 'items': {'enum': hands}, 'minItems': 1, 'uniqueItems': True}],
                         'x-taskweave-widget': 'hands', 'description': props['hand']['description']}
        return schema

    def stop(self) -> None:
        for module in self._used:
            module.stop()

    def default_timeout(self, args: dict) -> float:
        """The timeout of the part this step is for (nav 120 s, the others 60 s)."""
        part = None
        if isinstance(args.get('to'), str):
            try:
                target = self.config.target(args['to'], self.robot.name)
            except KeyError:
                return self.default_timeout_s
            part = ('straight' if (args.get('motion') or {}).get('mode') == 'straight' else 'navigate') \
                if target.kind == 'base' else FORM_OF_TARGET[target.kind]
        elif args.get('hand') == HOLDING:
            part = 'carry'
        else:
            part = next((p for form, p in (('hand', 'hand'), ('object', 'carry'), ('base', 'straight'),
                                           ('elbow', 'posture'), ('joints', 'joints')) if form in args), None)
        module = self.modules.get(part)
        return float(module.default_timeout_s) if module is not None else self.default_timeout_s

    # ---- run ------------------------------------------------------------------------------------------------
    def execute(self, ctx, args):
        plan = self._plan(args)
        if isinstance(plan, SkillResult):
            return plan
        module = self.modules[plan.part]
        self._used = [module]
        if args.get('dry_run'):
            return self._dry(ctx, plan, module)
        if plan.before is not None:
            joints = self.modules['joints']
            self._used.append(joints)
            first = joints.execute(ctx, plan.before)
            if first.outcome != Outcome.SUCCEEDED:
                return self._finish(first, plan, module, prefix='joints of ')
        result = None
        for number, call in enumerate(plan.calls, start=1):
            result = module.execute(ctx, call)
            if result.outcome != Outcome.SUCCEEDED:
                if len(plan.calls) > 1:
                    result.message = f'leg {number}/{len(plan.calls)}: {result.message}'
                return self._finish(result, plan, module)
        if len(plan.calls) > 1:
            result.message = f'{len(plan.calls)} legs: {result.message}'
        return self._finish(result, plan, module)

    def _data(self, plan: Plan) -> dict:
        """What every result of a plan has in its data: the kind, the module that ran, the target, what it carries."""
        return {'kind': plan.kind, 'module': part_name(plan.part), 'target': plan.target,
                **({'carrying': plan.carrying} if plan.carrying else {})}

    def _finish(self, result: SkillResult, plan: Plan, module, prefix: str = '') -> SkillResult:
        """The part's result with the command and what moved in the message ("move to target · carrying frame_1 ·
        hook"), and ``kind``, ``module``, ``target`` (a carry: ``carrying``) in the data."""
        hands = result.data.get('hands')
        named = f' ({", ".join(hands)})' if plan.part in ('hand', 'carry', 'posture') and hands else ''
        impl = ''
        if plan.part == 'navigate':
            impl = ' (nav2)' if hasattr(module, 'nav') else ' (direct)'
        if result.outcome == Outcome.REJECTED:   # nothing moved: the kind is what matters
            result.message = f'{move_label(plan.kind)}: {result.message}'
        else:
            result.message = f'{move_label(plan.kind)} · {prefix}{plan.desc}{named}{impl}: {result.message}'
        result.data.update(self._data(plan))
        return result

    def _dry(self, ctx, plan: Plan, module) -> SkillResult:
        """``dry_run``: what the move would do, nothing moves."""
        if plan.part in ('navigate', 'straight'):
            return SkillResult.succeeded(f'{move_label(plan.kind)} · {plan.desc}: resolved (nothing moved)',
                                         dry_run=True, **self._data(plan), **plan.dry)
        if plan.part == 'posture' or not hasattr(module, 'dry_run'):
            return SkillResult.rejected(f'{move_label(plan.kind)}: dry_run is not available for this kind of motion')
        return self._finish(module.dry_run(ctx, plan.calls[0]), plan, module)

    # ---- resolving a step -----------------------------------------------------------------------------------
    def _reject(self, text: str, form: str = '') -> SkillResult:
        """REJECTED with the command of ``form`` (a row of the option table; carry is move to target) in front."""
        return SkillResult.rejected(f'{move_label(kind_of(form))}: {text}' if form else f'move: {text}')

    def _plan(self, args: dict):
        """The Plan of a step, or a REJECTED / FAILED result saying what is wrong."""
        if args.get('look_at') is not None:
            return self._reject('look_at is not implemented yet: set head through joints')
        if 'hand' in args and 'hands' in args and args['hand'] != HOLDING:   # holding: hands picks the hold
            return self._reject('hand and hands are two spellings: give one (hand names the hands of a relative '
                                'move, hands filters the hands of a target)')
        options = {key: args[key] for key in OPTIONS if key in args}
        extra = [key for key in RELATIVE_OPTIONS if key in args]
        if 'to' in args:
            return self._plan_target(args, options, extra)
        if 'hand' in args:
            return self._plan_relative(args, options, extra, 'carry' if args['hand'] == HOLDING else 'hand')
        if 'object' in args:     # cell schema 4: {object: held, by / at} is {hand: holding, by / at}
            return self._plan_relative(args, options, extra, 'carry')
        if 'base' in args:
            return self._plan_base_by(args, options, extra)
        if 'elbow' in args:
            return self._plan_posture(args, options, extra)
        if 'joints' in args:
            return self._plan_joints(args, options, extra)
        return self._reject('give one of: to, hand, object, base, elbow, joints')

    def _part(self, part: str):
        """The part's module, or a REJECTED result if this robot has none for it."""
        if part in self.modules:
            return self.modules[part]
        reasons = '; '.join(self.unavailable.get(part, [])) or 'not configured'
        return self._reject(f'this robot cannot do {part} moves: {reasons}')

    def _options(self, form: str, options: dict, extra: list, ref: str | None = None, kind: str | None = None,
                 allowed_extra=(), relative: bool = False):
        """A REJECTED result if an option does not apply to this kind of motion (taskweave_core ``MOVE_OPTIONS``,
        which options each kind takes), else None."""
        problems = move_option_problems(form, options, relative, ref is not None)
        problems += [f'move {form}: option {key} does not apply' for key in extra if key not in allowed_extra]
        speed_abs = (options.get('motion') or {}).get('speed_abs')
        if speed_abs is not None and form in ('navigate', 'straight'):
            problems.append(f'move {form}: motion.speed_abs does not apply (the base takes motion.speed)')
        if not problems:
            return None
        text = problems[0].removeprefix(f'move {form}: ')
        if form in ('navigate', 'straight') and options.get('torso') == 'auto':
            text = 'torso auto needs a hand or object target'
        if ref is not None:
            return self._reject(f'{ref!r} moves {MOVES_WORD[kind or form]}; {text}')
        return self._reject(text, form)

    def _near(self) -> Pose2D | None:
        """Where the robot's base is in the cell (odometry, else TF): which turn of a symmetric object it handles."""
        for part in ('navigate', 'straight'):
            module = self.modules.get(part)
            pose = module._current() if module is not None and hasattr(module, '_current') else None
            if pose is not None:
                return pose
        module = next((m for m in self.modules.values() if hasattr(m, 'hold')), None)
        if module is None:
            return None
        try:
            matrix = module.hold.transform(getattr(module, 'cell_frame', 'world'), self.robot.base['frame'])
        except LookupError:
            return None
        return Pose2D(float(matrix[0, 3]), float(matrix[1, 3]), math.atan2(matrix[1, 0], matrix[0, 0]))

    def _torso_default(self, part: str) -> str | None:
        """``auto`` for a hand move or a carry unless the part's own configuration says otherwise."""
        if part not in ('hand', 'carry'):
            return None
        if 'torso' in motion.values(motion.resolve(self.config, self.robot.name, part_name(part))):
            return None
        return 'auto'

    def _torso_values(self, value, kind: str):
        """The joint values of a torso given as a number or a list ({joint: value}), or a REJECTED result."""
        try:
            return group_values(self.config, self.robot.name, {TORSO: value})
        except GroupError as error:
            return self._reject(str(error), kind)

    def _torso(self, part: str, step, target_joints: dict, kind: str):
        """The torso of a hand or object move, by the order: the step's ``torso``, the target's ``joints.torso``, the
        kind's default. Returns ``(word, joints)``: ``word`` auto / keep, or None when nothing says (the part's own
        default holds); ``joints`` the joint values a number or list sets ({joint: value}). A number of the step is
        the height to keep (``word`` keep); a number in the target is part of its joints and leaves the part's
        default as it is (an old pose with a lift behaved so). A REJECTED result if the value does not fit."""
        if step is not None:
            if isinstance(step, str):
                return step, {}
            values = self._torso_values(step, kind)
            return (values, {}) if isinstance(values, SkillResult) else ('keep', values)
        value = target_joints.get(TORSO)
        if value is None or isinstance(value, str):
            return value or self._torso_default(part), {}
        values = self._torso_values(value, kind)
        return (values, {}) if isinstance(values, SkillResult) else (self._torso_default(part), values)

    def _group_values(self, joints: dict, kind: str, skip_torso: bool = True):
        """The joint values of a joints part (without the torso entry: ``_torso`` decides that), or a REJECTED."""
        rest = {group: value for group, value in joints.items() if group != TORSO or not skip_torso}
        try:
            return group_values(self.config, self.robot.name, rest)
        except GroupError as error:
            return self._reject(str(error), kind)

    # ---- the forms --------------------------------------------------------------------------------------------
    def _plan_target(self, args: dict, options: dict, extra: list):
        ref = args['to']
        try:
            target = self.config.target(ref, self.robot.name, self._near())
        except KeyError as error:
            return self._reject(str(error.args[0]) if error.args else str(error))
        kind = target.kind
        if kind in ('hand', 'object') and target.frame == 'cell':
            return self._reject(f'cell-frame {kind} targets are not supported yet (target {ref!r})', 'hand')
        if kind == 'base' and target.frame == 'base':
            return self._reject(f'base-frame base targets are not supported yet: use base: {{by: [...]}} '
                                f'(target {ref!r})', 'navigate')
        form = ('straight' if (options.get('motion') or {}).get('mode') == 'straight' else 'navigate') \
            if kind == 'base' else FORM_OF_TARGET[kind]
        refused = self._options(form, options, extra, ref, kind)
        if refused is not None:
            return refused
        return getattr(self, f'_target_{form}')(args, options, target)

    def _call_motion(self, options: dict, **more) -> dict:
        """The ``motion`` of a call: the step's, without what is Move's (``via`` is resolved) plus ``more``."""
        result = {key: value for key, value in (options.get('motion') or {}).items() if key not in ('via',)}
        result.update({key: value for key, value in more.items() if value is not None})
        return result

    def _speeds(self, options: dict) -> dict:
        motion_options = options.get('motion') or {}
        return {key: motion_options[key] for key in ('speed', 'accel') if key in motion_options}

    def _base_legs(self, target, ref: str, kind: str):
        """The poses of a base target in the cell frame, one per waypoint (the last the target)."""
        count = len(target.poses)
        return [(pose, ref if number == count else f'{ref} ({number}/{count})')
                for number, pose in enumerate(target.poses, start=1)]

    def _drive_plan(self, part: str, options: dict, target, ref: str, args: dict):
        module = self._part(part)
        if isinstance(module, SkillResult):
            return module
        step = options.get('torso')
        value = step if step is not None else target.joints.get(TORSO)
        if value == 'auto':
            return self._reject('torso auto needs a hand or object target', part)
        torso, extra = None, {}           # None: the profile's drive height
        if value == 'keep':
            torso = 'keep'
        elif value is not None:
            values = self._torso_values(value, part)
            if isinstance(values, SkillResult):
                return values
            if step is not None:
                torso = values            # the drive module moves the torso there before it drives
            else:
                torso, extra = 'keep', values   # the target's joints leg sets it: the drive leaves it
        rest = self._group_values(target.joints, part)    # a joints part: the other groups move first
        if isinstance(rest, SkillResult):
            return rest
        rest = {**rest, **extra}
        before = None
        if rest:
            joints = self._part('joints')
            if isinstance(joints, SkillResult):
                return joints
            before = {'joints': dict(rest), 'motion': self._speeds(options)}
        speeds = self._speeds(options)
        calls = []
        for pose, label in self._base_legs(target, ref, part):
            resolved = {'pose': pose, 'torso': torso, **speeds}
            calls.append({'target': label, RESOLVED: resolved} if part == 'navigate' else
                         {'to': label, RESOLVED: resolved})
        dry = {'pose': _pose_dict(target.poses[-1]), 'poses': [_pose_dict(p) for p in target.poses],
               'torso': torso, 'joint_goal': dict(rest)}
        text = f'{"to " if part == "straight" else ""}{ref}'
        return Plan(part, text, ref, calls, before, dry)

    def _target_navigate(self, args: dict, options: dict, target):
        return self._drive_plan('navigate', options, target, args['to'], args)

    def _target_straight(self, args: dict, options: dict, target):
        return self._drive_plan('straight', options, target, args['to'], args)

    def _resolve_via(self, entries: list, kinds: tuple, context: str, kind: str):
        """The ``motion.via`` entries as resolved targets with their joint values: [{reference, target, joints}], or a
        REJECTED result."""
        items = []
        near = self._near()
        for entry in entries:
            try:
                found = self.config.target(entry, self.robot.name, near, kinds=kinds, context=context)
            except KeyError as error:
                return self._reject(str(error.args[0]) if error.args else str(error), kind)
            if found.frame == 'cell':
                return self._reject(f'cell-frame {found.kind} targets are not supported yet (target {entry!r})', kind)
            values = self._group_values(found.joints, kind, skip_torso=False)
            if isinstance(values, SkillResult):
                return values
            items.append({'reference': entry, 'target': found, 'joints': values})
        return items

    # ---- carrying ---------------------------------------------------------------------------------------------
    def _registry(self):
        """The hold registry the parts share (hold.py), or None (no part that knows it)."""
        module = next((self.modules[p] for p in ('carry', 'hand') if p in self.modules), None)
        return getattr(module, 'hold', None)

    def _carrying(self, hands: list[str], form: str):
        """What a hand move of ``hands`` carries: None (no hand holds anything: a hand move), the hold (every hand
        holds the same object: the whole hold moves) or a REJECTED result. The held object's type must have a collision
        shape (``objects/<type>.yaml`` ``collision:``): a hand moved while holding is planned around it."""
        registry = self._registry()
        holding = dict(registry.hand_holds) if registry is not None else {}
        busy = [hand for hand in hands if hand in holding]
        if not busy:
            return None
        free = [hand for hand in hands if hand not in holding]
        text = ', '.join(f'{hand} holds {holding[hand]}' for hand in busy)
        if free:
            return self._reject(f'{text}, {" and ".join(free)} holds nothing: move them in two steps (hand: holding '
                                'names the holding hands)', form)
        objects = sorted({holding[hand] for hand in busy})
        if len(objects) > 1:
            return self._reject(f'{text}: name the hands of one held object', form)
        hold = next(h for h in registry.held_all() if h['object'] == objects[0])
        kind = registry.type_of(hold, self.config.object_instances())
        if kind is None:
            return self._reject(f'{text}: its type is unknown, so is its collision shape (a hand moves while holding '
                                'only around a known shape; move it with hand: holding or an object target)', form)
        if not self.config.object(kind).get('collision'):
            return self._reject(f'{text}: a hand moves while holding only when the held object has a collision shape: '
                                f'objects/{kind}.yaml has no collision: (add one, or move it with hand: holding or an '
                                'object target)', form)
        return hold

    def _carry_options(self, options: dict, target_joints: dict | None = None):
        """A REJECTED result for what a hand move takes but a carry does not (via legs, a joints part), else None."""
        motion_options = options.get('motion') or {}
        for key in ('via', 'approach_m'):
            if motion_options.get(key):
                return self._reject(f'motion.{key} does not apply while the hands hold something (a carry)', 'hand')
        if target_joints:
            return self._reject('joint values (a joints part of the target, a torso height) do not apply while the '
                                f'hands hold something: {", ".join(target_joints)} (torso auto or keep do)', 'hand')
        return None

    def _target_hands(self, options: dict, target) -> list[str] | None:
        """The hands a hand target moves (the ``hands`` option over the hands the target has), None for any_one (it
        picks a free hand)."""
        rule = options.get('hands', 'auto')
        if rule == 'any_one':
            return None
        if rule == 'auto':
            return [key[:-len(TOOL_SUFFIX)] for key, pose in target.poses[-1].items()
                    if key.endswith(TOOL_SUFFIX) and pose is not None]
        return list(self.hands) if rule == 'all' else [rule] if isinstance(rule, str) else list(rule)

    def _target_hand(self, args: dict, options: dict, target):
        hands = self._target_hands(options, target)
        hold = self._carrying(hands, 'hand') if hands else None
        if isinstance(hold, SkillResult):
            return hold
        part = 'carry' if hold is not None else 'hand'
        module = self._part(part)
        if isinstance(module, SkillResult):
            return module
        ref = args['to']
        word, torso_joints = self._torso(part, options.get('torso'), target.joints, 'hand')
        if isinstance(word, SkillResult):
            return word
        joints = self._group_values(target.joints, 'hand')
        if isinstance(joints, SkillResult):
            return joints
        joints = {**joints, **torso_joints}
        data = {key: target.data[key] for key in motion.TARGET_KEYS if target.data.get(key) is not None}
        if hold is not None:      # the hands that hold go to the target, the held object with them
            refused = self._carry_options(options, joints)
            if refused is not None:
                return refused
            mine = [hand for hand in hands if hand in hold['hands']]
            call = {'to': {'hand_target': ref}, 'object': hold['object'],
                    'motion': self._call_motion(options, torso=word),
                    RESOLVED: {'hand_target': {'target': target, 'offset_m': float(options.get('offset_m', 0.0)),
                                               'joints': {}, 'data': data, 'via': []}, 'hands': mine}}
            return Plan('carry', f'carrying {hold["object"]} · {ref}', ref, [call], carrying=hold['object'])
        via = self._resolve_via((options.get('motion') or {}).get('via') or [], ('hand', 'joints'),
                                'via takes hand or joint targets', 'hand')
        if isinstance(via, SkillResult):
            return via
        call = {'motion': self._call_motion(options, torso=word),
                RESOLVED: {'target': target, 'offset_m': float(options.get('offset_m', 0.0)), 'joints': joints,
                           'data': data, 'via': via}}
        if 'hands' in options:
            call['hands'] = options['hands']
        return Plan('hand', ref, ref, [call])

    def _hold_of(self, module, args: dict):
        """The hold a carry is for (the part's own rules: nothing held, several, hands), or a REJECTED."""
        found = module._hold(args)
        if isinstance(found, SkillResult):
            return self._reject(found.message, 'carry')
        return found

    def _object_args(self, args: dict, options: dict) -> dict:
        call = {}
        for key in ('hands', 'object'):
            if key in args and not (key == 'object' and args[key] == 'held'):
                call[key] = args[key]
        return call

    def _target_carry(self, args: dict, options: dict, target):
        module = self._part('carry')
        if isinstance(module, SkillResult):
            return module
        ref = args['to']
        hold = self._hold_of(module, self._object_args(args, options))
        if isinstance(hold, SkillResult):
            return hold
        held_type = module.hold.type_of(hold, self.config.object_instances())
        problem = target.needs_held(held_type)
        if problem:
            return self._reject(problem, 'carry')
        word, torso_joints = self._torso('carry', options.get('torso'), target.joints, 'carry')
        if isinstance(word, SkillResult):
            return word
        call = {**self._object_args(args, options), 'motion': self._call_motion(options, torso=word)}
        desc = f'carrying {hold["object"]} · {ref}'
        if target.frame == 'base':
            if options.get('reference'):
                return self._reject(f'option reference does not apply to {ref!r}, a base-frame object target (it '
                                    'names a part of the held object for a target on an object)', 'carry')
            if options.get('offset_m'):
                return self._reject(f'option offset_m does not apply to {ref!r}, a base-frame object target (it '
                                    'stops short of a target on an object)', 'carry')
            joints = self._group_values(target.joints, 'carry')
            if isinstance(joints, SkillResult):
                return joints
            held = {'joints': {**joints, **torso_joints}, 'path': list(target.poses)}
            call.update({'to': {'pose': ref}, RESOLVED: {'held': held}})
            return Plan('carry', desc, ref, [call], carrying=hold['object'])
        if target.joints:
            return self._reject(f'{ref!r} has a joints part: not supported on an object target in an object\'s '
                                'frame', 'carry')
        name = options.get('reference') or target.reference_target
        mine = ORIGIN
        if name:
            if held_type is None:
                return self._reject(f'the held {hold["object"]} is not a placed object: its type is unknown '
                                    f'(reference {name!r})', 'carry')
            try:
                found = self.config.type_target(held_type, name, self.robot.name)
            except KeyError as error:
                return self._reject(f'reference {name!r}: {error.args[0]}', 'carry')
            if found.kind != 'object':
                return self._reject(f'reference {name!r} of {held_type} objects moves {MOVES_WORD[found.kind]}: '
                                    'it must be an object target', 'carry')
            mine = found.pose
        call.update({'to': {'onto': ref, 'contact': name or 'origin', 'offset_m': float(options.get('offset_m', 0.0))},
                     RESOLVED: {'onto': {'instance': target.instance, 'pose': target.pose, 'mine': mine}}})
        return Plan('carry', desc, ref, [call], carrying=hold['object'])

    def _target_joints(self, args: dict, options: dict, target):
        module = self._part('joints')
        if isinstance(module, SkillResult):
            return module
        ref = args['to']
        goal = self._group_values(target.joints, 'joints', skip_torso=False)
        if isinstance(goal, SkillResult):
            return goal
        via = self._resolve_via((options.get('motion') or {}).get('via') or [], ('joints',),
                                'via of a joint move takes joint targets', 'joints')
        if isinstance(via, SkillResult):
            return via
        call = {'pose': ref, 'motion': self._call_motion(options),
                RESOLVED: {'goal': goal, 'via': [item['joints'] for item in via]}}
        if 'only' in options:
            call['only'] = options['only']
        return Plan('joints', ref, ref, [call])

    def _plan_relative(self, args: dict, options: dict, extra: list, which: str):
        """A hand move relative to now (``which`` hand), or a carry (``which`` carry: ``hand: holding`` or the old
        ``object`` form, the held object moved as the old move_object did). A hand move whose hands hold something is
        a carry too: the hand goes by / to where the step says, the held object keeps its grasp."""
        form = which
        hands, rule = None, args.get('hand')
        if which == 'hand':
            if rule in ('auto', 'any_one'):
                return self._reject(f'hand {rule!r}: name the hands (a name, a list, all or holding)', 'hand')
            hands = rule if isinstance(rule, list) else self.hands if rule == 'all' else [rule]
        else:
            module = self._part('carry')
            if isinstance(module, SkillResult):
                return module
        refused = self._options(form, options, [], None, None, relative=True)
        if refused is not None:
            return refused
        keys = RELATIVE_KEYS[which]
        use = 'by' if 'by' in args else 'at'
        if 'by' in args and 'at' in args:
            return self._reject('give by or at, not both', which)
        if not any(key in args for key in ('by', 'at')):
            return self._reject('give by or at', which)
        stray = [key for key in extra if key not in keys[use]]
        if stray:
            return self._reject(f'option {stray[0]} does not apply to {use}', which)
        if isinstance(options.get('torso'), int | float | list):
            return self._reject('a relative move takes torso auto or keep (a value needs a target or joints)', which)
        to = {key: args[key] for key in keys[use] if key in args}
        if 'at' in to and which == 'hand' and len(hands) != 1:
            return self._reject('at needs exactly one hand', 'hand')
        hold = self._carrying(hands, 'hand') if which == 'hand' else None
        if isinstance(hold, SkillResult):
            return hold
        if which == 'hand':
            module = self._part('carry' if hold is not None else 'hand')
            if isinstance(module, SkillResult):
                return module
        part = 'carry' if hold is not None else which
        word = options.get('torso') if options.get('torso') in ('auto', 'keep') else self._torso_default(part)
        call: dict = {'to': to, 'motion': self._call_motion(options, torso=word)}
        if which == 'carry':
            held = self._hold_of(module, self._object_args(args, options))
            if isinstance(held, SkillResult):
                return held
            call.update(self._object_args(args, options))
            return Plan('carry', f'carrying {held["object"]} · {use}', None, [call], carrying=held['object'])
        named = ", ".join(rule if isinstance(rule, list) else [rule])
        if hold is not None:      # the named hand goes by / to, the whole hold with it
            refused = self._carry_options(options)
            if refused is not None:
                return refused
            call.update({'object': hold['object'], RESOLVED: {'hand': hands[0]}})
            return Plan('carry', f'carrying {hold["object"]} · hand {named} {use}', None, [call],
                        carrying=hold['object'])
        call['hands'] = rule
        return Plan('hand', f'hand {named} {use}', None, [call])

    def _plan_base_by(self, args: dict, options: dict, extra: list):
        module = self._part('straight')
        if isinstance(module, SkillResult):
            return module
        refused = self._options('straight', options, extra)
        if refused is not None:
            return refused
        forward, sideways = (float(v) for v in args['base']['by'])
        torso = options.get('torso')
        if isinstance(torso, int | float | list):
            torso = self._torso_values(torso, 'straight')
            if isinstance(torso, SkillResult):
                return torso
        resolved = {'torso': torso, **self._speeds(options)}
        call = {'distance_m': forward, RESOLVED: resolved}
        if sideways:
            call['sideways_m'] = sideways
        return Plan('straight', f'by {forward:+g} m' + (f', {sideways:+g} m sideways' if sideways else ''),
                    None, [call], dry={'by': [forward, sideways], 'torso': torso, 'joint_goal': {}})

    def _plan_posture(self, args: dict, options: dict, extra: list):
        module = self._part('posture')
        if isinstance(module, SkillResult):
            return module
        refused = self._options('posture', options, extra)
        if refused is not None:
            return refused
        elbow = args['elbow']
        base = {'motion': self._call_motion(options)}
        if 'tolerance_deg' in options:
            base['tolerance_deg'] = options['tolerance_deg']
        if isinstance(elbow, dict):
            if 'hands' in options:
                return self._reject('elbow names its hands: drop hands', 'posture')
            sides: dict[str, list[str]] = {}
            for hand, side in elbow.items():
                if hand not in self.hands:
                    return self._reject(f'elbow: unknown hand {hand!r} (hands: {", ".join(self.hands)})', 'posture')
                sides.setdefault(side, []).append(hand)
            calls = [{**base, 'elbow': side, 'hands': hands} for side, hands in sides.items()]
            text = 'elbow ' + ', '.join(f'{hand} {side}' for hand, side in elbow.items())
        else:
            calls = [{**base, 'elbow': elbow, **({'hands': options['hands']} if 'hands' in options else {})}]
            text = f'elbow {elbow}'
        return Plan('posture', text, None, calls)

    def _plan_joints(self, args: dict, options: dict, extra: list):
        module = self._part('joints')
        if isinstance(module, SkillResult):
            return module
        refused = self._options('joints', options, extra)
        if refused is not None:
            return refused
        if 'only' in options:
            return self._reject('option only applies to a joint target (to:)', 'joints')
        joints = args['joints']
        word = joints.get(TORSO)
        if word == 'auto':
            return self._reject('torso auto needs a hand or object target', 'joints')
        values = self._group_values(joints, 'joints')
        if isinstance(values, SkillResult):
            return values
        if word is not None and not isinstance(word, str):
            lift = self._torso_values(word, 'joints')
            if isinstance(lift, SkillResult):
                return lift
            values.update(lift)
        via = self._resolve_via((options.get('motion') or {}).get('via') or [], ('joints',),
                                'via of a joint move takes joint targets', 'joints')
        if isinstance(via, SkillResult):
            return via
        if not values:
            return self._reject('joints names no joint to move (keep only)', 'joints')
        call = {'joints': values, 'motion': self._call_motion(options),
                RESOLVED: {'goal': values, 'via': [item['joints'] for item in via]}}
        return Plan('joints', ', '.join(f'{group} {_shown(value)}' for group, value in joints.items()), None, [call])


def _shown(value) -> str:
    return f'{value:g}' if isinstance(value, int | float) else str(value)


def _pose_dict(pose: Pose2D) -> dict:
    return {'x': round(float(pose.x), 4), 'y': round(float(pose.y), 4), 'yaw': round(float(pose.yaw), 4)}

