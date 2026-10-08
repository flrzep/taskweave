"""ROS node that hosts skills. Each skill gets a RunSkill action server at /taskweave/skills/<name>.

Parameters:
  config_dir (string)    taskweave config directory
  robot (string)         robot profile name (config/robots/<robot>/profile.yaml)
  skills (string[])      skills to load as "package.module:ClassName"

Run: ros2 run taskweave_core skill_server --ros-args -p config_dir:=... -p robot:=... -p skills:="['...']"
"""

from __future__ import annotations

import importlib
import json
import traceback

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from taskweave_interfaces.action import RunSkill
from taskweave_interfaces.msg import SkillInfo
from taskweave_interfaces.srv import ListSkills

from . import names
from .config import ConfigError, ConfigStore
from .executive import Outcome
from .skill import Skill, SkillContext, SkillResult
from .validation import validate_args


def load_skill_class(path: str) -> type[Skill]:
    module_name, _, class_name = path.partition(':')
    if not class_name:
        raise ValueError(f'skill {path!r} must be given as "package.module:ClassName"')
    cls = getattr(importlib.import_module(module_name), class_name)
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
        if skills is None:
            paths = [p for p in self.get_parameter('skills').value if p]
            skills = [load_skill_class(path)() for path in paths]
        self.group = ReentrantCallbackGroup()
        self.skills: dict[str, Skill] = {}
        self._servers = []
        for skill in skills:
            if not skill.name or skill.name in self.skills:
                raise ValueError(f'skill name {skill.name!r} is empty or duplicated')
            skill.setup(self, self.config, self.robot, self.config.skill_params(skill.name))
            self.skills[skill.name] = skill
            self._servers.append(ActionServer(
                self, RunSkill, names.skill_action(skill.name),
                execute_callback=lambda handle, s=skill: self._execute(s, handle),
                goal_callback=lambda goal: GoalResponse.ACCEPT,
                cancel_callback=lambda handle: CancelResponse.ACCEPT,
                callback_group=self.group))
        self.create_service(ListSkills, f'{self.get_name()}{names.SKILL_SERVER_CATALOG_SUFFIX}', self._list,
                            callback_group=self.group)
        self.get_logger().info(f'skills for robot {robot_name!r}: {", ".join(self.skills) or "none"}')

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

    def _execute(self, skill: Skill, handle) -> RunSkill.Result:
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
        level = self.get_logger().info if result.outcome == Outcome.SUCCEEDED else self.get_logger().warning
        level(f'{skill.name} [{goal.step_id}] {result.outcome.name}{": " + result.message if result.message else ""}')
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
        timeout = goal.timeout_s if goal.timeout_s > 0 else skill.default_timeout_s

        def publish(progress: float, status: str) -> None:
            handle.publish_feedback(RunSkill.Feedback(progress=float(progress), status=status))

        ctx = SkillContext(self.now, timeout, lambda: handle.is_cancel_requested, publish, goal.instruction,
                           self.get_logger())
        try:
            result = skill.execute(ctx, args)
        except Exception as error:  # a crashing skill is a failed step, not a dead server
            self.get_logger().error(f'{skill.name} raised:\n{traceback.format_exc()}')
            return SkillResult.failed(f'{type(error).__name__}: {error}')
        if not isinstance(result, SkillResult):
            return SkillResult.failed(f'{skill.name}.execute returned {type(result).__name__}, not SkillResult')
        return result


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

