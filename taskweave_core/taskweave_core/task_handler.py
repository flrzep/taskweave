"""ROS node around the executive: starts missions, dispatches steps to skills, publishes status.

Parameters:
  config_dir (string)        taskweave config directory
  robot (string)             robot profile this handler drives; missions for other robots are rejected
  timeout_grace_s (double)   extra time after a step's timeout before the handler cancels it (default 2.0)
  status_rate_hz (double)    periodic status publishing while a mission is active (default 2.0)
  speed_override_percent (double)  global speed override 1-100 (default 100; not persisted). Set through the
                             service set_speed_override (the single write path; setting the parameter goes through it)
  speed_test_percent, speed_cap_percent (double)  test mode: a run starts at test (default 10), cap (default 50)
                             is the highest override while it is on

Interfaces: see taskweave_core.names (start_mission, pause, resume, abort, list_skills, status, plan, episodes).
"""

from __future__ import annotations

import json
import threading

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from taskweave_interfaces.action import RunSkill
from taskweave_interfaces.msg import Plan as PlanMsg
from taskweave_interfaces.msg import SpeedOverride as SpeedOverrideMsg
from taskweave_interfaces.msg import TaskStatus
from taskweave_interfaces.srv import DescribeRobot, ListSkills, SetSpeedOverride, StartMission

from . import names
from .config import ConfigError, ConfigStore
from .conversions import plan_to_msg, skill_info_to_dict, status_to_msg
from .embodiment import robot_card
from .executive import ACTIVE, Executive, Outcome, Status
from .plan import Plan, Step
from .planner import MissionFilePlanner
from .skill_server import spin
from .speed import MAX_PERCENT, MIN_PERCENT, OverrideState
from .validation import validate_plan
from .world_state import WorldState

CATALOG_TIMEOUT_S = 2.0


class RosDispatcher:
    """Sends steps to RunSkill action servers and reports results back to the executive."""

    def __init__(self, node: TaskHandler):
        self.node = node
        self.clients: dict[str, ActionClient] = {}
        self.goals: dict[int, object] = {}
        self.pending_cancel: set[int] = set()
        self.lock = threading.Lock()

    def dispatch(self, token: int, step: Step, attempt: int) -> None:
        info = self.node.catalog.get(step.skill)
        if info is None:
            self._report(token, Outcome.REJECTED, f'skill {step.skill!r} is not in the catalog')
            return
        client = self.clients.get(info.action_name)
        if client is None:
            client = ActionClient(self.node, RunSkill, info.action_name, callback_group=self.node.group)
            self.clients[info.action_name] = client
        if not client.server_is_ready():
            self._report(token, Outcome.REJECTED, f'skill server for {step.skill!r} is not available')
            return
        goal = RunSkill.Goal(step_id=step.id, skill=step.skill, instruction=step.instruction,
                             args_json=json.dumps(step.args), timeout_s=float(step.timeout_s))
        future = client.send_goal_async(
            goal, feedback_callback=lambda msg: self.node.executive.on_feedback(
                token, msg.feedback.progress, msg.feedback.status))
        future.add_done_callback(lambda f: self._accepted(token, f))

    def cancel(self, token: int) -> None:
        with self.lock:
            handle = self.goals.get(token)
            if handle is None:
                self.pending_cancel.add(token)
                return
        handle.cancel_goal_async()

    def _accepted(self, token: int, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._report(token, Outcome.REJECTED, 'goal rejected by the skill server')
            return
        with self.lock:
            self.goals[token] = handle
            cancel_now = token in self.pending_cancel
            self.pending_cancel.discard(token)
        if cancel_now:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(lambda f: self._finished(token, f))

    def _finished(self, token: int, future) -> None:
        with self.lock:
            self.goals.pop(token, None)
        result = future.result().result
        try:
            data = json.loads(result.data_json or '{}')
        except json.JSONDecodeError:
            data = {}
        self.node.executive.on_result(token, Outcome(result.outcome), result.message, data)

    def _report(self, token: int, outcome: Outcome, message: str) -> None:
        # Never call back into the executive from inside its own dispatch call.
        threading.Thread(target=self.node.executive.on_result, args=(token, outcome, message), daemon=True).start()


class TaskHandler(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'task_handler'), **kwargs)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('robot', '')
        self.declare_parameter('timeout_grace_s', 2.0)
        self.declare_parameter('status_rate_hz', 2.0)
        self.declare_parameter('speed_override_percent', MAX_PERCENT)
        self.declare_parameter('speed_test_percent', 10.0)
        self.declare_parameter('speed_cap_percent', 50.0)
        config_dir = self.get_parameter('config_dir').value
        self.robot = self.get_parameter('robot').value
        if not config_dir or not self.robot:
            raise ValueError('parameters config_dir and robot are required')
        self.config = ConfigStore(config_dir)
        self.config.robot(self.robot)  # fail at startup if the profile or embodiment is missing or invalid
        self.config.embodiment(self.robot)
        self.group = ReentrantCallbackGroup()
        self.world = WorldState()
        self.catalog: dict[str, object] = {}
        self.dispatcher = RosDispatcher(self)
        self.executive: Executive | None = None
        self._published_plan: Plan | None = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(TaskStatus, names.STATUS, latched)
        self.plan_pub = self.create_publisher(PlanMsg, names.PLAN, latched)
        self.episode_pub = self.create_publisher(String, names.EPISODES, 50)
        self.create_service(StartMission, names.START_MISSION, self._start, callback_group=self.group)
        self.create_service(Trigger, names.PAUSE, self._trigger('pause'), callback_group=self.group)
        self.create_service(Trigger, names.RESUME, self._trigger('resume'), callback_group=self.group)
        self.create_service(Trigger, names.ABORT, self._trigger('abort'), callback_group=self.group)
        self.create_service(ListSkills, names.LIST_SKILLS, self._list_skills, callback_group=self.group)
        self.create_service(DescribeRobot, names.DESCRIBE_ROBOT, self._describe, callback_group=self.group)
        self._setup_speed(latched)
        self.create_timer(0.1, self._tick, callback_group=self.group)
        self.create_timer(1.0 / self.get_parameter('status_rate_hz').value, self._publish_status,
                          callback_group=self.group)
        self._publish_status(Status())
        self.get_logger().info(f'task handler ready for robot {self.robot!r}, config {self.config.root}')

    # ---- helpers -------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def refresh_catalog(self) -> dict:
        """Ask every skill server for its skills (services named */list_skills of type ListSkills)."""
        catalog = {}
        for service, types in self.get_service_names_and_types():
            if service == names.LIST_SKILLS or 'taskweave_interfaces/srv/ListSkills' not in types:
                continue
            client = self.create_client(ListSkills, service, callback_group=self.group)
            try:
                if not client.wait_for_service(timeout_sec=CATALOG_TIMEOUT_S):
                    continue
                done = threading.Event()
                future = client.call_async(ListSkills.Request())
                future.add_done_callback(lambda _, event=done: event.set())
                if done.wait(CATALOG_TIMEOUT_S) and future.result() is not None:
                    for info in future.result().skills:
                        if info.name in catalog:
                            self.get_logger().warning(f'skill {info.name!r} is offered twice; using {service}')
                        catalog[info.name] = info
            finally:
                self.destroy_client(client)
        self.catalog = catalog
        return catalog

    def _validate(self, plan: Plan) -> list[str]:
        problems = []
        if plan.robot != self.robot:
            problems.append(f'plan is for robot {plan.robot!r}, this handler drives {self.robot!r}')
        schemas = {name: json.loads(info.args_schema_json or '{}') for name, info in self.catalog.items()}
        return problems + validate_plan(plan, schemas, self.config)

    def _publish_status(self, status: Status | None = None) -> None:
        status = status or (self.executive.status if self.executive else Status())
        self.status_pub.publish(status_to_msg(status, self.get_clock().now().to_msg(), self.now()))

    def _on_status(self, status: Status) -> None:
        self._publish_status(status)
        if self._run_active and status.state not in ACTIVE:
            self._speed_run(False)
        if self.executive and self.executive.plan and self._published_plan is not self.executive.plan:
            self._published_plan = self.executive.plan
            self.plan_pub.publish(plan_to_msg(self.executive.plan))

    def _on_episode(self, event: dict) -> None:
        self.episode_pub.publish(String(data=json.dumps(event, default=str)))

    def _tick(self) -> None:
        if self.executive:
            self.executive.tick()

    # ---- speed override (skill API spec 4.2) -----------------------------------------
    def _setup_speed(self, latched) -> None:
        percent = float(self.get_parameter('speed_override_percent').value)
        if not MIN_PERCENT <= percent <= MAX_PERCENT:
            raise ValueError(f'speed_override_percent {percent:g} is outside {MIN_PERCENT:g}-{MAX_PERCENT:g}')
        self.speed = OverrideState(percent, float(self.get_parameter('speed_test_percent').value),
                                   float(self.get_parameter('speed_cap_percent').value))
        self._speed_lock, self._speed_syncing, self._run_active = threading.RLock(), False, False
        self.speed_pub = self.create_publisher(SpeedOverrideMsg, names.SPEED_OVERRIDE, latched)
        self.create_service(SetSpeedOverride, names.SET_SPEED_OVERRIDE, self._set_speed, callback_group=self.group)
        if hasattr(self, 'add_pre_set_parameters_callback'):
            self.add_pre_set_parameters_callback(self._speed_pre_set)
        self.add_on_set_parameters_callback(self._speed_param)
        self._publish_speed()

    def speed_request(self, percent: float = 0.0, test_mode: bool | None = None,
                      test_percent: float = 0.0) -> tuple[bool, str, float]:
        """The single write path of the override: the service, the parameter and run start/end go through it."""
        with self._speed_lock:
            result = self.speed.request(percent, test_mode, test_percent)
            if result[0]:
                self._speed_changed()
            return result

    def _speed_changed(self) -> None:
        """Publish the override and keep the parameter in step (without running its callback's rules again)."""
        if self.get_parameter('speed_override_percent').value != self.speed.percent:
            self._speed_syncing = True
            try:
                self.set_parameters([Parameter('speed_override_percent', value=float(self.speed.percent))])
            finally:
                self._speed_syncing = False
        self._publish_speed()

    def _publish_speed(self) -> None:
        s = self.speed
        self.speed_pub.publish(SpeedOverrideMsg(percent=float(s.percent), test_mode=s.test_mode,
                                                test_percent=float(s.test_percent), cap_percent=float(s.cap_percent)))

    def _set_speed(self, request, response):
        response.ok, response.message, response.percent = self.speed_request(
            float(request.percent), bool(request.test_mode), float(request.test_percent))
        self.get_logger().info(f'speed override: {response.message}')
        return response

    def _speed_pre_set(self, parameters: list) -> list:
        """In test mode a parameter value above the cap is clamped, like a service request."""
        if self._speed_syncing or not self.speed.test_mode:
            return parameters
        cap = self.speed.cap_percent
        return [Parameter(p.name, value=float(cap))
                if p.name == 'speed_override_percent' and isinstance(p.value, float) and p.value > cap else p
                for p in parameters]

    def _speed_param(self, parameters: list) -> SetParametersResult:
        if self._speed_syncing:
            return SetParametersResult(successful=True)
        for p in parameters:
            if p.name != 'speed_override_percent':
                continue
            if not isinstance(p.value, float) or not MIN_PERCENT <= p.value <= MAX_PERCENT:
                return SetParametersResult(successful=False,
                                           reason=f'speed_override_percent must be {MIN_PERCENT:g}-{MAX_PERCENT:g}')
            if self.speed.test_mode and p.value > self.speed.cap_percent:  # no pre-set callback in this rclpy
                return SetParametersResult(successful=False, reason=f'test mode: at most {self.speed.cap_percent:g}')
            with self._speed_lock:
                self.speed.request(p.value)
                self._publish_speed()
        return SetParametersResult(successful=True)

    def _speed_run(self, started: bool) -> None:
        """Test mode: lower the override at run start, restore it at run end if it lowered it."""
        with self._speed_lock:
            self._run_active = started
            changed = self.speed.run_started() if started else self.speed.run_ended()
            if changed:
                self._speed_changed()
                self.get_logger().info(f'speed override {self.speed.percent:g} % (test mode, run '
                                       f'{"start" if started else "end"})')

    # ---- services ---------------------------------------------------------------
    def _start(self, request, response):
        if self.executive and self.executive.status.state in ACTIVE:
            response.accepted, response.message = False, 'a mission is already active; abort it first'
            return response
        try:
            self.config.reload()  # pick up config edits between missions
            planner = MissionFilePlanner(self.config, request.mission, request.goal or None, self.robot)
        except ConfigError as error:
            response.accepted, response.message = False, str(error)
            return response
        self.refresh_catalog()
        self.executive = Executive(planner, self.dispatcher, self.world, validate=self._validate, clock=self.now,
                                   on_status=self._on_status, on_episode=self._on_episode,
                                   timeout_grace_s=self.get_parameter('timeout_grace_s').value,
                                   speed=lambda: self.speed.percent / 100.0)
        self._speed_run(True)
        status = self.executive.start(request.start_step)
        response.accepted = status.state in ACTIVE or status.state.name == 'SUCCEEDED'
        response.message = status.last_error if not response.accepted else f'started {status.plan_id}'
        response.plan_id = status.plan_id
        at = f' from step {request.start_step}' if request.start_step else ''
        self.get_logger().info(f'mission {request.mission!r}{at}: {response.message}')
        return response

    def _trigger(self, command: str):
        def handle(request, response):
            if self.executive is None:
                response.success, response.message = False, 'no mission'
                return response
            response.success = getattr(self.executive, command)()
            response.message = self.executive.status.state.name.lower()
            return response
        return handle

    def _list_skills(self, request, response):
        response.skills = list(self.refresh_catalog().values())
        return response

    def _describe(self, request, response):
        embodiment = self.config.embodiment(self.robot)
        response.skills = list(self.refresh_catalog().values())
        response.card = robot_card(embodiment, [skill_info_to_dict(s) for s in response.skills],
                                   self.config.point_names(self.robot))
        response.embodiment_json = json.dumps(embodiment)
        return response


def main(args=None):
    rclpy.init(args=args)
    try:
        spin(TaskHandler())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
