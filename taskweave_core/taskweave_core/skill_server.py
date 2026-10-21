"""ROS node that hosts skills. Each skill gets a RunSkill action server at /taskweave/skills/<name>.

Parameters:
  config_dir (string)    taskweave config directory
  robot (string)         robot name (config/robots/<robot>/profile.yaml and embodiment.yaml)
  skills (string[])      skill names to serve

For every skill name the server reads skills/<name>.yaml and serves the first implementation whose ``requires``
fits the robot's embodiment; it logs why the others do not fit. A skill without a fitting implementation is not
served (missions using it are then rejected by validation with the same reasons).

Service <node>/reload_config (ReloadConfig): re-reads the config and applies, between goals, what the skills read per
goal: the parameters of skills/<name>.yaml and base.limits of the profile. A changed implementation, a profile or
embodiment part bound at start (topics, joint interface, IK, planning, hands) or a skill's ``startup_params`` is
reported as needs_restart. While a goal is active the answer is ``busy``.

Which skills are served depends on the cell pack. A cell in the targets layout (schema 4) is launched with ``wait``,
``move`` and ``grip``: the motion skills of the old layout (navigate_to, drive_straight, move_gripper, move_object,
set_posture, move_joints) are what ``move`` does, so asking for any of them serves ``move`` (once). An older cell
serves the names it is asked for, and ``move`` is not served there (it needs targets): ``served_names``.

A skill with parts (``move``) keeps the parts' configuration in ``skills/<skill>/<part>.yaml``; a change to one
reloads the skill (``skill_of``).

Run: ros2 run taskweave_core skill_server --ros-args -p config_dir:=... -p robot:=... -p skills:="['...']"
"""

from __future__ import annotations

import copy
import importlib
import json
import threading
import traceback
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from taskweave_interfaces.action import RunSkill
from taskweave_interfaces.msg import RestartItem, SkillInfo
from taskweave_interfaces.srv import ListSkills, ReloadConfig

from . import names
from .config import ConfigError, ConfigStore, RobotProfile
from .config.reload import Reloader
from .executive import Outcome
from .skill import Skill, SkillContext, SkillResult
from .speed import SpeedOverride
from .validation import validate_args

OLD_MOTION_SKILLS = ('navigate_to', 'drive_straight', 'move_gripper', 'move_object', 'set_posture', 'move_joints')
TARGET_SKILLS = ('wait', 'move', 'grip')   # what the launch of a cell in the targets layout asks for


def served_names(requested: list[str], targets: bool) -> tuple[list[str], list[str]]:
    """(the skills to serve, notes on what was left out or folded) for the names asked for (the ``skills``
    parameter) and a cell pack in the targets layout (``targets``) or not. In the targets layout the motion skills of
    the old layout are folded into ``move`` (the launch lists ``wait,move,grip``); other skills are served as asked.
    Before the targets layout ``move`` cannot be served, and the old skills are. Order of the request is kept."""
    notes: list[str] = []
    served: list[str] = []
    for name in requested:
        if targets and name in OLD_MOTION_SKILLS:
            notes.append(f'{name}: replaced by move in the targets layout (cell schema 4), serving move')
            name = 'move'
        elif not targets and name == 'move':
            notes.append('move: needs a cell pack in the targets layout (cell schema 4): run taskweave config migrate; '
                         'not served')
            continue
        if name not in served:
            served.append(name)
    return served, notes


def skill_of(path: str) -> str:
    """The skill a skill config file belongs to: ``skills/<skill>.yaml``, or ``skills/<skill>/<part>.yaml``."""
    file = Path(path)
    return file.parent.name if file.parent.name != 'skills' else file.stem


def load_skill_class(path: str) -> type[Skill]:
    module_name, _, class_name = path.partition(':')
    if not class_name:
        raise ValueError(f'skill {path!r} must be given as "package.module:ClassName"')
    try:
        cls = getattr(importlib.import_module(module_name), class_name)
    except AttributeError:
        from .config.migrate_skills import removed_class_message

        raise ValueError(removed_class_message(module_name, class_name) or f'{path}: no such class') from None
    if not (isinstance(cls, type) and issubclass(cls, Skill)):
        raise TypeError(f'{path} is not a taskweave Skill')
    return cls


class SkillServer(Node):
    def __init__(self, skills: list[Skill] | None = None, **kwargs):
        super().__init__(kwargs.pop('node_name', 'skill_server'), **kwargs)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('robot', '')
        self.declare_parameter('skills', [''])
        config_dir = self.get_parameter('config_dir').value
        robot_name = self.get_parameter('robot').value
        if not config_dir or not robot_name:
            raise ValueError('parameters config_dir and robot are required')
        self.config = ConfigStore(config_dir)
        self.robot = self.config.robot(robot_name)
        implementations: dict[str, dict] = {}
        if skills is None:
            skills = []
            requested, notes = served_names([n for n in self.get_parameter('skills').value if n],
                                            self.config.targets_format())
            for note in notes:
                self.get_logger().info(note)
            for skill_name in requested:
                implementation, reasons = self.config.select_implementation(skill_name, robot_name)
                for reason in reasons:
                    self.get_logger().info(f'{skill_name}: not using {reason}')
                if implementation is None:
                    self.get_logger().error(f'{skill_name}: no implementation fits robot {robot_name!r}; not served')
                    continue
                skill = load_skill_class(implementation['class'])()
                if skill.name != skill_name:
                    raise ValueError(f'{implementation["class"]} implements {skill.name!r}, not {skill_name!r}')
                skill.implementation = f'{implementation["type"]}:{implementation["name"]}'
                implementations[skill_name] = implementation
                skills.append(skill)
        self._impl, self._params = implementations, {}
        self._active, self._lock = 0, threading.Lock()    # goals running; held while a reload applies
        try:
            self._embodiment = self.config.embodiment(robot_name)
        except ConfigError:
            self._embodiment = None
        self.group = ReentrantCallbackGroup()
        self.speed = SpeedOverride.for_node(self)  # the handler's global override (latched topic)
        self.skills: dict[str, Skill] = {}
        self._servers = []
        for skill in skills:
            if not skill.name or skill.name in self.skills:
                raise ValueError(f'skill name {skill.name!r} is empty or duplicated')
            self._params[skill.name] = self.config.skill_params(skill.name, implementations.get(skill.name))
            skill.setup(self, self.config, self.robot, self._params[skill.name])
            self.skills[skill.name] = skill
            self._servers.append(ActionServer(
                self, RunSkill, names.skill_action(skill.name),
                execute_callback=lambda handle, s=skill: self._execute(s, handle),
                goal_callback=lambda goal: GoalResponse.ACCEPT,
                cancel_callback=lambda handle: CancelResponse.ACCEPT,
                callback_group=self.group))
        self.create_service(ListSkills, f'{self.get_name()}{names.SKILL_SERVER_CATALOG_SUFFIX}', self._list,
                            callback_group=self.group)
        self.create_service(ReloadConfig, f'{self.get_name()}{names.RELOAD_CONFIG_SUFFIX}', self._reload,
                            callback_group=self.group)
        self.reloader = Reloader(self.config, robot_name, {'live', 'reload_service', 'launch'}, {}, self._concerns,
                                 self._apply)
        served = ', '.join(f'{name} ({skill.implementation})' for name, skill in self.skills.items())
        self.get_logger().info(f'skills for robot {robot_name!r}: {served or "none"}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def info(self, skill: Skill) -> SkillInfo:
        return SkillInfo(name=skill.name, description=skill.description,
                         args_schema_json=json.dumps(skill.args_schema), needs=list(skill.needs),
                         achieves=list(skill.achieves), implementation=skill.implementation,
                         action_name=names.skill_action(skill.name))

    def _list(self, request, response):
        response.skills = [self.info(skill) for skill in self.skills.values()]
        return response

    # ---- reload_config ---------------------------------------------------------
    def _concerns(self, entry: dict) -> bool:
        """Skill yamls of the skills served here, the profile and embodiment, and the live files (the store reads
        those per use)."""
        if entry['kind'] in ('skill', 'skill_part'):
            return skill_of(entry['path']) in self.skills
        return entry['kind'] in ('profile', 'embodiment') or 'live' in entry['reload']

    def _apply(self, entries: list[dict]) -> list[tuple[str, str]]:
        """Take the changed profile, embodiment and skill yamls into effect. Everything is built and tried on copies
        first, so a value that does not load changes nothing (raises). Returns what only a restart applies."""
        refused, name = [], self.robot.name
        robot, old = self.robot, self.robot.data
        profile = next((e for e in entries if e['kind'] == 'profile'), None)
        if profile:     # base.limits is read per goal; the rest of the profile is bound when the server starts
            new = self.config.robot(name).data
            robot = RobotProfile(name, {**old, 'base': {**_bound(old, 'base'), **_limits_of(new['base'])}})
            rest = sorted(k for k in {*old, *new} if _bound(old, k) != _bound(new, k))
            if rest:
                refused.append((profile['path'], f'{", ".join(rest)}: topics, joint interface, IK, planning and hands '
                                                 'are bound when the skill servers start'))
        embodiment = next((e for e in entries if e['kind'] == 'embodiment'), None)
        if embodiment and self.config.embodiment(name) != self._embodiment:
            refused.append((embodiment['path'], 'the skill implementations were chosen from it when the skill '
                                                'servers started'))
        changed = {skill_of(e['path']): e['path'] for e in entries if e['kind'] in ('skill', 'skill_part')}
        plan = {}
        for skill in self.skills.values():
            if skill.name not in changed and robot is self.robot:
                continue
            params = self._params[skill.name]
            if skill.name in changed:
                path, impl = changed[skill.name], self._impl.get(skill.name)
                new_impl = self.config.select_implementation(skill.name, name)[0] if impl else None
                if impl and (new_impl is None or
                             (new_impl['name'], new_impl['class']) != (impl['name'], impl['class'])):
                    refused.append((path, 'the implementation (name, class) was chosen when the skill servers '
                                          'started'))
                    continue
                params = dict(self.config.skill_params(skill.name, new_impl))
                for key in skill.startup_params:    # a connection made at start keeps its value
                    if params.get(key) != self._params[skill.name].get(key):
                        refused.append((path, f'{key}: a connection made when the skill server started'))
                        params.pop(key, None)
                        if key in self._params[skill.name]:
                            params[key] = self._params[skill.name][key]
            trial = copy.copy(skill)
            trial.robot = robot
            trial.configure(dict(params))
            plan[skill.name] = params
        self.robot = robot
        for skill_name, params in plan.items():
            self.skills[skill_name].robot = robot
            self.skills[skill_name].configure(dict(params))
            self._params[skill_name] = params
        return refused

    def _reload(self, request, response):
        response.node = self.get_name()
        with self._lock:
            if self._active:
                response.status, response.message = 'busy', f'{self._active} skill goal(s) active'
                return response
            try:
                response.reloaded, restart = self.reloader.reload(list(request.paths))
            except Exception as error:  # a file that does not load: nothing was applied
                response.status, response.message = 'failed', f'{type(error).__name__}: {error}'
                self.get_logger().warning(f'reload_config failed: {response.message}')
                return response
        response.needs_restart = [RestartItem(path=path, what=what) for path, what in restart]
        response.status = 'reloaded'
        self.get_logger().info(f'reload_config: {len(response.reloaded)} reloaded, {len(restart)} need a restart')
        return response

    def _execute(self, skill: Skill, handle) -> RunSkill.Result:
        with self._lock:
            self._active += 1
        try:
            return self._execute_goal(skill, handle)
        finally:
            with self._lock:
                self._active -= 1

    def _execute_goal(self, skill: Skill, handle) -> RunSkill.Result:
        goal = handle.request
        result = self._run(skill, goal, handle)
        if result.outcome != Outcome.SUCCEEDED:
            try:
                skill.stop()
            except Exception:  # stopping must never mask the original outcome
                self.get_logger().error(f'{skill.name}.stop() failed:\n{traceback.format_exc()}')
        message = RunSkill.Result(outcome=result.outcome.value, message=result.message,
                                  data_json=json.dumps(result.data))
        if result.outcome == Outcome.SUCCEEDED:
            handle.succeed()
        elif result.outcome == Outcome.CANCELED and handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.abort()
        text = f'{skill.name} [{goal.step_id}] {result.outcome.name}{": " + result.message if result.message else ""}'
        if result.outcome == Outcome.SUCCEEDED:  # one call site per severity: rclpy refuses to mix them
            self.get_logger().info(text)
        else:
            self.get_logger().warning(text)
        return message

    def _run(self, skill: Skill, goal, handle) -> SkillResult:
        try:
            args = json.loads(goal.args_json or '{}')
        except json.JSONDecodeError as error:
            return SkillResult.rejected(f'args_json is not valid JSON: {error}')
        try:
            problems = validate_args(args, skill.args_schema, self.config, self.robot.name)
        except ConfigError as error:
            problems = [str(error)]
        if problems:
            return SkillResult.rejected('; '.join(problems))
        timeout = goal.timeout_s if goal.timeout_s > 0 else _default_timeout(skill, args)

        def publish(progress: float, status: str) -> None:
            handle.publish_feedback(RunSkill.Feedback(progress=float(progress), status=status))

        ctx = SkillContext(self.now, timeout, lambda: handle.is_cancel_requested, publish, goal.instruction,
                           self.get_logger(), speed=self.speed)
        try:
            result = skill.execute(ctx, args)
        except Exception as error:  # a crashing skill is a failed step, not a dead server
            self.get_logger().error(f'{skill.name} raised:\n{traceback.format_exc()}')
            result = SkillResult.failed(f'{type(error).__name__}: {error}')
        if not isinstance(result, SkillResult):
            result = SkillResult.failed(f'{skill.name}.execute returned {type(result).__name__}, not SkillResult')
        ctx.work_now()
        speed = result.data.setdefault('speed', {})   # the override this step ran at (Record, smoke test)
        for key, value in ctx.work.stats().items():   # a skill may add keys of its own (nav2_override)
            speed.setdefault(key, value)
        return result


def _default_timeout(skill: Skill, args: dict) -> float:
    try:
        return float(skill.default_timeout(args))
    except Exception:  # noqa: BLE001 - a timeout guess must not fail the step: the skill's own check follows
        return float(skill.default_timeout_s)


def _limits_of(base: dict) -> dict:
    return {'limits': base['limits']} if 'limits' in base else {}


def _bound(profile: dict, key: str):
    """A profile part as the skills bind it at start: all of it, but base.limits (read per goal)."""
    value = profile.get(key)
    return {k: v for k, v in value.items() if k != 'limits'} if key == 'base' else value


def spin(node: Node) -> None:
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        spin(SkillServer())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

