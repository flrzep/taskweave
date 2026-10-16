"""taskweave web HMI server: a ROS bridge plus a small HTTP server (standard library only).

Parameters:
  config_dir (string)   taskweave config directory (missions are read and written here)
  robot (string)        robot the running task handler drives
  host (string)         address to bind; bind the trusted network only (e.g. the tailnet address)
  port (int)            default 8090
  recordings_dir (string)  where the Record tab keeps recordings (bag, sim videos); empty: no Record tab

HTTP API (JSON):
  GET  /api/state                  robot, status, plan with per-step outcomes, recent events
  GET  /api/events                 server-sent events: status | plan | episode
  GET  /api/missions               mission list (invalid files are listed with their error)
  GET  /api/missions/<name>        mission as JSON; GET /api/template for a new one
  PUT  /api/missions/<name>        validate and save; 422 with {problems} if invalid, nothing written;
                                   ?rename_from=<old> saves mission <old> as <name> (old file removed)
  DELETE /api/missions/<name>      remove the mission file (a running mission keeps its plan)
  POST /api/missions/<name>/validate
  GET  /api/missions/<name>/plan   the plan a start would run now (includes expanded, facts filled): step ids to start at
  POST /api/start {mission[, start_step]}  start a mission (at a step id: the first cycle begins there, robot and
                                   objects as they are); POST /api/pause | /api/resume | /api/abort
  GET  /api/skills                 skill catalog;  GET /api/points  named points for argument pickers
  GET  /api/points/all             locations, stance offsets and placed objects, with the missions using each point
  PUT  /api/points/location/<name>                          {x, y, yaw[, rename]}  create, update or rename
  PUT  /api/points/stance/<objects|instances>/<owner>/<variant>  {x, y, yaw[, rename]}  (offset in the object frame)
  DELETE the same addresses;  422 with {problems} if refused, nothing written
  GET  /api/robot_pose             current base pose in the cell and relative to every placed object
  POST /api/points/drive {target}  run navigate_to now (no mission running): location name or stance:<object>/<variant>
  GET  /api/robot                  joint poses (with the missions using each), navigation clearance, footprint
  GET  /api/robot/joints           current joint positions
  GET  /api/robot/tools            current tool targets {tool: {tcp, approach, closing}} (MoveIt forward kinematics)
  PUT  /api/robot/poses/<name>     {joints[, rename, create]};  DELETE the same
  PUT  /api/robot/navigation       {envelope: {front, back, left, right}, padding, inflation_radius, driving}: checked
                                   against the robot's base, saved and applied to the running costmaps
                                   (parameter nav_costmaps) and controller (nav_controller)
  POST /api/robot/move {pose}      run move_to_pose now (no mission running); returns the outcome when it ends
  POST /api/robot/reach {joints, tools, search: [tool]}   per tool: ok, message, warnings, nearest reachable target
  PUT  /api/robot/joints {joints: {name: value}}  send joint targets now (no mission running; within the limits)
  GET  /api/teach/arm               arm positions: robot poses and object points as one list (arm.py)
  PUT  /api/teach/arm/<robot|type>/<name>      {hands, joints, elbow, approach_m[, rename]};  DELETE the same
  POST /api/teach/arm/<robot|type>/<name>/capture {at, tools, form}  save the robot's current arm there (also creates)
  POST /api/teach/arm/<robot|type>/<name>/test {at}  move there now (move_to_pose / go_to_point at placed object <at>)
  PUT  /api/teach/target {scope, name, at, form} or {}   the position a pad teach press saves into
  PUT  /api/teach/held/<type>/<name> {joints, path, rename}; DELETE the same   held poses (move_object pose:)
  POST /api/teach/held/<type>/<name>/test   move the held object there now (move_object)
  GET  /api/recordings              {status, recordings, free_bytes} (404 without recordings_dir; recording.py)
  POST /api/recordings/start {mission, name, cycles, reset, bag, camera, video, quality, max_min};  POST .../stop
  DELETE /api/recordings/<id>;  GET /recordings/<id>/<file>  a recording's video or data file (byte ranges)
Missions and points are edited between runs (409 while a mission runs) and are used at the next start.
"""

from __future__ import annotations

import json
import queue
import shutil
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlsplit

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from taskweave_core import names
from taskweave_core.config import ConfigError, ConfigStore, Pose2D
from taskweave_core.planner import MissionFilePlanner
from taskweave_core.teach import MAX_POSE_AGE_S, yaw_from_quaternion
from taskweave_interfaces.action import RunSkill
from taskweave_interfaces.msg import Plan as PlanMsg
from taskweave_interfaces.msg import TaskStatus
from taskweave_interfaces.srv import ListSkills, StartMission, TeachPoint
from taskweave_skills.ik import pose_matrix, tool_target
from taskweave_skills.nav2 import nav2_parameters

from . import arm, held, missions, points, recording, robot

WEB = Path(__file__).parent / 'web'
STATES = ['idle', 'running', 'paused', 'succeeded', 'failed', 'aborted']
CALL_TIMEOUT_S = 5.0
STATIC_FILES = ('app.js', 'robot.js', 'teach.js', 'held.js', 'graph.js', 'record.js', 'style.css')
MANUAL_SKILL_TIMEOUT_S = 120.0
DRIVE_TIMEOUT_S = 300.0  # sim time; a drive across the cell in a slow sim
NAV_COSTMAPS = ['/local_costmap/local_costmap', '/global_costmap/global_costmap']  # Nav2 defaults
NAV_CONTROLLER = '/controller_server'
FK_SERVICE = '/compute_fk'  # MoveIt move_group
OUTCOMES = ['succeeded', 'failed', 'timeout', 'canceled', 'rejected']
CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
                 '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.mp4': 'video/mp4',
                 '.csv': 'text/csv; charset=utf-8', '.json': 'application/json', '.log': 'text/plain; charset=utf-8'}
CHUNK = 1 << 20


def step_dict(step) -> dict:
    return {'id': step.step_id, 'instruction': step.instruction, 'skill': step.skill,
            'args': json.loads(step.args_json or '{}'), 'timeout_s': step.timeout_s, 'retries': step.retries,
            'on_failure': step.on_failure}


def preview_plan(config: ConfigStore, mission: str, robot: str) -> dict:
    """The plan a start of ``mission`` would run now, shaped like the live plan (``plan_id`` ``<mission>-preview``);
    no steps when there is nothing to do (e.g. every stack is empty)."""
    plan = MissionFilePlanner(config, mission, None, robot).make_plan({}, 1)
    if plan is None:
        return {'plan_id': f'{mission}-preview', 'goal': '', 'robot': robot, 'loop': '', 'steps': [],
                'message': 'nothing to do now'}
    return {'plan_id': f'{mission}-preview', 'goal': plan.goal, 'robot': plan.robot, 'loop': plan.loop,
            'steps': [{'id': step.id, 'instruction': step.instruction, 'skill': step.skill, 'args': step.args,
                       'timeout_s': step.timeout_s, 'retries': step.retries, 'on_failure': step.on_failure}
                      for step in plan.steps]}


def status_dict(msg: TaskStatus) -> dict:
    return {'state': STATES[msg.state], 'plan_id': msg.plan_id, 'goal': msg.goal, 'cycle': msg.cycle,
            'step_index': msg.step_index, 'step_count': msg.step_count,
            'step': step_dict(msg.step) if msg.step_index >= 0 else None, 'attempt': msg.attempt,
            'step_status': msg.step_status, 'step_progress': msg.step_progress,
            'step_elapsed_s': msg.step_elapsed_s, 'last_error': msg.last_error}


def ros_parameter(name: str, value) -> Parameter:
    if isinstance(value, bool):
        return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value))
    if isinstance(value, int):
        return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value))
    if isinstance(value, float):
        return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
    return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=str(value)))


class Busy(Exception):
    """The request would change config while a mission runs."""


class Bridge(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'hmi_server'), **kwargs)
        for name, default in (('config_dir', ''), ('robot', ''), ('host', ''), ('port', 8090),
                              ('nav_costmaps', NAV_COSTMAPS), ('nav_controller', NAV_CONTROLLER),
                              ('recordings_dir', ''), ('recording_bag_exclude', recording.BAG_EXCLUDE)):
            self.declare_parameter(name, default)
        self.config = ConfigStore(self.get_parameter('config_dir').value)
        self.robot = self.get_parameter('robot').value
        if not self.robot:
            raise ValueError('parameter robot is required')
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.status: dict = status_dict(TaskStatus())
        self.plan: dict | None = None
        self.outcomes: dict[str, dict] = {}   # step id -> latest step event for the current plan and cycle
        self.events: deque = deque(maxlen=200)
        self.listeners: list[queue.Queue] = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskStatus, names.STATUS, self._status, latched, callback_group=self.group)
        self.create_subscription(PlanMsg, names.PLAN, self._plan, latched, callback_group=self.group)
        self.create_subscription(String, names.EPISODES, self._episode, 50, callback_group=self.group)
        self.pad_mode = 'drive'
        self.create_subscription(String, names.OPERATOR_MODE, self._pad_mode, latched, callback_group=self.group)
        self.create_subscription(String, names.OPERATOR_NOTICE, self._notice, 10, callback_group=self.group)
        self.start_client = self.create_client(StartMission, names.START_MISSION, callback_group=self.group)
        self.skills_client = self.create_client(ListSkills, names.LIST_SKILLS, callback_group=self.group)
        self.triggers = {cmd: self.create_client(Trigger, getattr(names, cmd.upper()), callback_group=self.group)
                         for cmd in ('pause', 'resume', 'abort')}
        self.pose: Pose2D | None = None
        self.pose_stamp = 0.0
        self.odom_topic = self.config.robot(self.robot).base['odom']
        self.create_subscription(Odometry, self.odom_topic, self._odom, 10, callback_group=self.group)
        self.joints: dict[str, float] = {}
        self.joints_stamp = 0.0
        joints = self.config.robot(self.robot).data.get('joints', {})
        if joints.get('state'):
            self.create_subscription(JointState, joints['state'], self._joint_states, 10, callback_group=self.group)
        self.param_clients = {
            node: self.create_client(SetParameters, f'{node}/set_parameters', callback_group=self.group)
            for node in [*self.get_parameter('nav_costmaps').value, self.get_parameter('nav_controller').value]}
        self.skill_lock = threading.Lock()  # one manual skill at a time
        self.skill_clients: dict[str, ActionClient] = {}
        self.fk_client = None  # created on first use (MoveIt)
        self.ik = None  # IkClient, created on first reach check
        self.reach_lock = threading.Lock()
        # teaching: the arm position open in the HMI receives pad teach presses (operator_joy)
        self.teach_client = self.create_client(TeachPoint, names.TEACH_POINT, callback_group=self.group)
        self.teach_target: dict | None = None
        self.teach_target_pub = self.create_publisher(String, names.TEACH_TARGET, latched)
        self.teach_target_pub.publish(String(data=''))
        self.notice_pub = self.create_publisher(String, names.OPERATOR_NOTICE, 10)
        self.create_subscription(String, names.TEACH_REQUEST, self._teach_request, 10, callback_group=self.group)
        self.joint_command = self.create_publisher(JointState, joints['command'], 10) if joints.get('command') else None
        directory = self.get_parameter('recordings_dir').value
        exclude = self.get_parameter('recording_bag_exclude').value
        self.recorder = recording.Recorder(self, directory, exclude) if directory else None

    # ---- ROS -> state ------------------------------------------------------------
    def _broadcast(self, kind: str, data) -> None:
        message = json.dumps({'type': kind, 'data': data})
        with self.lock:
            for listener in list(self.listeners):
                try:
                    listener.put_nowait(message)
                except queue.Full:  # a client that stopped reading (e.g. a background tab): blocking here would
                    self.listeners.remove(listener)  # hold the lock for good; end its stream, EventSource reconnects
                    with listener.mutex:
                        listener.queue.clear()
                    listener.put_nowait(None)

    def _status(self, msg: TaskStatus) -> None:
        status = status_dict(msg)
        with self.lock:
            self.status = status
        self._broadcast('status', status)

    def _plan(self, msg: PlanMsg) -> None:
        plan = {'plan_id': msg.plan_id, 'goal': msg.goal, 'robot': msg.robot, 'loop': msg.loop,
                'steps': [step_dict(step) for step in msg.steps]}
        with self.lock:
            if self.plan is None or self.plan['plan_id'] != plan['plan_id']:
                self.outcomes = {}
            self.plan = plan
        self._broadcast('plan', plan)

    def _odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self.pose = Pose2D(p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.pose_stamp = time.monotonic()

    def robot_pose(self) -> Pose2D:
        if self.pose is None or time.monotonic() - self.pose_stamp > MAX_POSE_AGE_S:
            raise TimeoutError(f'no current robot pose on {self.odom_topic}')
        return self.pose

    def _joint_states(self, msg: JointState) -> None:
        self.joints = {**self.joints, **dict(zip(msg.name, msg.position, strict=False))}
        self.joints_stamp = time.monotonic()

    def joint_positions(self) -> dict[str, float]:
        if not self.joints or time.monotonic() - self.joints_stamp > MAX_POSE_AGE_S:
            raise TimeoutError('no current joint states')
        return dict(self.joints)

    def check_idle(self) -> None:
        with self.lock:
            state = self.status['state']
        if state in ('running', 'paused'):
            raise Busy(f'a mission is {state}; change settings when it has finished or was aborted')

    def pose_args(self) -> set:
        try:
            return robot.pose_args(self.catalog())
        except TimeoutError:
            return robot.DEFAULT_POSE_ARGS

    def apply_navigation(self, settings: dict) -> dict[str, str]:
        """Set clearance and driving on the running path planner; node -> 'applied' or the reason it was not."""
        values = nav2_parameters(settings, self.config.robot(self.robot).base)
        result = {}
        for node, client in self.param_clients.items():
            params = values['controller'] if node == self.get_parameter('nav_controller').value else values['costmap']
            if not params:
                continue
            try:
                request = SetParameters.Request(parameters=[ros_parameter(k, v) for k, v in params.items()])
                failed = [r.reason for r in self.call(client, request).results if not r.successful]
                result[node] = 'applied' if not failed else '; '.join(failed)
            except TimeoutError as error:
                result[node] = str(error)
        return result

    def tool_targets(self) -> dict:
        """Current tool targets (TCP, approach, closing in the base frame) from the IK planner's forward kinematics."""
        from moveit_msgs.msg import RobotState
        from moveit_msgs.srv import GetPositionFK

        profile = self.config.robot(self.robot).data
        tools = {tool: profile['tool_frames'][tool] for tool in profile.get('ik', {}).get('groups', {})
                 if tool in profile.get('tool_frames', {})}
        if not tools:
            raise TimeoutError('the robot profile has no IK tools')
        if self.fk_client is None:
            self.fk_client = self.create_client(GetPositionFK, FK_SERVICE, callback_group=self.group)
        request = GetPositionFK.Request(fk_link_names=[frame['link'] for frame in tools.values()],
                                        robot_state=RobotState(is_diff=True))
        request.header.frame_id = self.config.robot(self.robot).base['frame']
        response = self.call(self.fk_client, request)
        if response.error_code.val != 1:
            raise TimeoutError(f'forward kinematics failed (code {response.error_code.val})')
        links = zip(response.fk_link_names, response.pose_stamped, strict=False)
        poses = {link: stamped.pose for link, stamped in links}
        return {tool: tool_target(pose_matrix(poses[frame['link']].position, poses[frame['link']].orientation), frame)
                for tool, frame in tools.items()}

    def reach(self, body: dict) -> dict:
        """Can the robot put its tools on these targets? Per tool: ok, message, warnings, and with ``search`` the
        nearest reachable target (TCP moved along a base axis). Solved like move_to_pose solves a pose."""
        from taskweave_skills.ik import IkClient, IkError, target_warnings

        profile = self.config.robot(self.robot)
        if not profile.data.get('ik'):
            raise TimeoutError('the robot profile has no ik section')
        joints = {name: float(value) for name, value in (body.get('joints') or {}).items()}
        try:
            self.joint_positions()
        except TimeoutError as error:  # MoveIt places the robot from live TF: nothing to solve against
            raise TimeoutError(f'{error}: the robot (or sim) is not running, start it to check reach') from error
        with self.reach_lock:
            if self.ik is None:
                from taskweave_core.config.limits import joint_limits

                self.ik = IkClient(self, profile, limits=joint_limits(self.config, self.robot))
            result = {}
            for tool, target in (body.get('tools') or {}).items():
                entry = {'ok': False, 'message': '', 'warnings': [], 'suggestion': None}
                try:
                    entry['warnings'] = target_warnings(target)
                    self.ik.solve({tool: target}, joints)
                    entry['ok'] = True
                except IkError as error:
                    entry['message'] = str(error)
                    if tool in body.get('search', []):
                        entry['suggestion'] = self.ik.nearest_reachable(tool, target, joints)
                except (KeyError, ValueError, TypeError) as error:
                    entry['message'] = f'invalid target: {error}'
                result[tool] = entry
            return result

    def run_skill(self, skill: str, args: dict, timeout_s: float = MANUAL_SKILL_TIMEOUT_S) -> dict:
        """Run one skill directly (no mission), e.g. move to a joint pose to check it. Blocks until it ends."""
        self.check_idle()
        if not self.skill_lock.acquire(blocking=False):
            raise Busy('another manual skill is running')
        try:
            action = self.skill_clients.get(skill)
            if action is None:
                action = self.skill_clients[skill] = ActionClient(self, RunSkill, names.skill_action(skill),
                                                                  callback_group=self.group)
            if not action.wait_for_server(timeout_sec=CALL_TIMEOUT_S):
                raise TimeoutError(f'skill {skill} is not served')
            goal = RunSkill.Goal(step_id='hmi', skill=skill, args_json=json.dumps(args), timeout_s=timeout_s,
                                 instruction=f'{skill} from the HMI')
            sent = action.send_goal_async(goal)
            deadline = time.monotonic() + CALL_TIMEOUT_S
            while not sent.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not sent.done() or not sent.result().accepted:
                raise TimeoutError(f'{skill} did not accept the goal')
            result = sent.result().get_result_async()
            deadline = time.monotonic() + timeout_s + CALL_TIMEOUT_S
            while not result.done() and time.monotonic() < deadline:
                time.sleep(0.1)
            if not result.done():
                raise TimeoutError(f'{skill} did not finish in {timeout_s:g} s')
            outcome = result.result().result
            return {'outcome': OUTCOMES[outcome.outcome], 'message': outcome.message}
        finally:
            self.skill_lock.release()

    def set_teach_target(self, target: dict | None) -> None:
        with self.lock:
            self.teach_target = target or None
        self.teach_target_pub.publish(String(data=json.dumps(target) if target else ''))

    def _teach_request(self, msg: String) -> None:
        """A pad teach press (operator_joy): save the robot's arm into the position open in the HMI."""
        with self.lock:
            target = self.teach_target
        if not target:
            return
        tools = json.loads(msg.data or '{}').get('tools') or []
        label = f'{target["scope"]} {target["name"]}'
        try:
            file = self.capture(target['scope'], target['name'], {'at': target.get('at'), 'tools': tools,
                                                                  'form': target.get('form')})
            notice = {'text': f'taught {label} -> {file}', 'ok': True, 'taught': target}
        except (arm.ArmError, Busy, TimeoutError, KeyError, ConfigError) as error:
            notice = {'text': f'teach {label} failed: {error}', 'ok': False}
        self.notice_pub.publish(String(data=json.dumps(notice)))

    def capture(self, scope: str, name: str, body: dict) -> str:
        """Save the robot's current arm as an arm position: a robot pose from the joint states and forward
        kinematics, an object point through the teach node at placed object ``at`` (default: the nearest)."""
        self.check_idle()
        if scope == arm.ROBOT:
            try:
                targets = self.tool_targets()
            except TimeoutError:
                targets = None
            return arm.capture_pose(self.config, self.robot, name, body, self.joint_positions(), targets,
                                    self.pose_args())
        at = body.get('at') or self.nearest_placed(scope)
        instance = self.config.object_instances().get(at)
        if instance is None or instance.object != scope:
            raise arm.ArmError(f'{at} is not a placed {scope}')
        response = self.call(self.teach_client, TeachPoint.Request(name=f'{at}/{name}', kind='object_point',
                                                                   tools=list(body.get('tools') or [])))
        if not response.success:
            raise arm.ArmError(response.message)
        grip = arm.closed_grips(self.config, self.robot, list(body.get('tools') or []), self.joint_positions())
        if grip:  # grippers closed on the part when taught: they close that far at this point
            points.save_object_point(self.config, scope, name, {'grip_m': grip}, self.robot)
        try:
            return str(Path(response.file).relative_to(self.config.root))
        except ValueError:
            return response.file

    def nearest_placed(self, object_type: str) -> str:
        return arm.nearest_placed(self.config, object_type,
                                  points.relative_poses(self.config, self.robot_pose())['relative'])

    def test_position(self, scope: str, name: str, body: dict) -> dict:
        if scope == arm.ROBOT:
            return self.run_skill('move_to_pose', {'pose': name})
        at = body.get('at') or self.nearest_placed(scope)
        return self.run_skill('go_to_point', {'point': f'{at}/{name}'})

    def send_joints(self, values: dict) -> dict:
        """Joint targets straight to the robot (the HMI's joint panel): within the limits, no mission running."""
        from taskweave_core.config.limits import joint_limits, joint_problems

        self.check_idle()
        if self.joint_command is None:
            raise arm.ArmError('the robot profile has no joints.command topic')
        try:
            values = {str(k): float(v) for k, v in values.items()}
        except (TypeError, ValueError):
            raise arm.ArmError('joint values must be numbers') from None
        current = self.joint_positions()
        unknown = [joint for joint in values if joint not in current]
        if unknown:
            raise arm.ArmError(f'not in the joint states: {", ".join(unknown)}')
        linear = set(self.config.robot(self.robot).data.get('joints', {}).get('linear', []))
        problems = joint_problems(values, joint_limits(self.config, self.robot), linear)
        if problems:
            raise arm.ArmError('outside the joint limits: ' + '; '.join(problems))
        self.joint_command.publish(JointState(name=list(values), position=list(values.values())))
        return values

    def _episode(self, msg: String) -> None:
        event = json.loads(msg.data)
        with self.lock:
            if event.get('event') == 'step_started' and event.get('step_id') in self.outcomes \
                    and self.outcomes[event['step_id']].get('cycle') != event.get('cycle'):
                self.outcomes = {}  # a new cycle starts over
            self.outcomes[event.get('step_id')] = event
            self.events.append(event)
        self._broadcast('episode', event)

    def _pad_mode(self, msg: String) -> None:
        with self.lock:
            self.pad_mode = msg.data
        self._broadcast('pad_mode', msg.data)

    def _notice(self, msg: String) -> None:
        self._broadcast('notice', json.loads(msg.data))

    def snapshot(self) -> dict:
        with self.lock:
            return {'robot': self.robot, 'status': self.status, 'plan': self.plan, 'outcomes': dict(self.outcomes),
                    'events': list(self.events)[-50:], 'pad_mode': self.pad_mode}

    # ---- service calls -------------------------------------------------------------
    def call(self, client, request):
        if not client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
            raise TimeoutError(f'{client.srv_name} not available - is the task handler running?')
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(CALL_TIMEOUT_S) or future.result() is None:
            raise TimeoutError(f'{client.srv_name}: no response')
        return future.result()

    def catalog(self) -> list[dict]:
        response = self.call(self.skills_client, ListSkills.Request())
        return [{'name': s.name, 'description': s.description, 'args_schema': json.loads(s.args_schema_json or '{}'),
                 'needs': list(s.needs), 'achieves': list(s.achieves), 'implementation': s.implementation}
                for s in response.skills]

    def schemas(self) -> dict:
        return {skill['name']: skill['args_schema'] for skill in self.catalog()}


class Handler(BaseHTTPRequestHandler):
    bridge: Bridge = None  # set by serve()
    server_version = 'taskweave-hmi'

    def log_message(self, fmt, *args):  # keep the ROS log readable
        pass

    # ---- helpers ----------------------------------------------------------------------
    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _json(self, data, status: int = HTTPStatus.OK) -> None:
        self._send(status, json.dumps(data).encode(), 'application/json')

    def _body(self) -> dict:
        length = int(self.headers.get('Content-Length') or 0)
        return json.loads(self.rfile.read(length) or b'{}')

    def _file(self, file: Path) -> None:
        """A file, streamed, with byte ranges (a video player seeks with them)."""
        size = file.stat().st_size
        try:
            span = recording.byte_range(self.headers.get('Range'), size)
        except ValueError:
            self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
            self.send_header('Content-Range', f'bytes */{size}')
            self.end_headers()
            return
        start, end = span or (0, size - 1)
        self.send_response(HTTPStatus.PARTIAL_CONTENT if span else HTTPStatus.OK)
        self.send_header('Content-Type', CONTENT_TYPES.get(file.suffix, 'application/octet-stream'))
        self.send_header('Content-Length', str(max(end - start + 1, 0)))
        self.send_header('Accept-Ranges', 'bytes')
        if span:
            self.send_header('Content-Range', f'bytes {start}-{end}/{size}')
        self.end_headers()
        try:
            with file.open('rb') as handle:
                handle.seek(start)
                left = end - start + 1
                while left > 0:
                    chunk = handle.read(min(CHUNK, left))
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    left -= len(chunk)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _recordings(self, method: str, route: list[str]):
        recorder = self.bridge.recorder
        if recorder is None:
            return self._json({'error': 'recording is not set up (parameter recordings_dir)'}, HTTPStatus.NOT_FOUND)
        if method == 'GET' and not route:
            free = shutil.disk_usage(recorder.dir).free if recorder.dir.is_dir() else None
            return self._json({'status': recorder.status(), 'recordings': recorder.recordings(), 'free_bytes': free})
        if method == 'POST' and route == ['start']:
            self.bridge.config.reload()
            return self._json(recorder.start(self._body(), self.bridge.config.mission_names()))
        if method == 'POST' and route == ['stop']:
            return self._json(recorder.stop())
        if method == 'DELETE' and len(route) == 1:
            recorder.delete(unquote(route[0]))
            self.bridge.get_logger().info(f'HMI deleted recording {route[0]}')
            return self._json({'deleted': route[0]})
        return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)

    def _route(self, method: str) -> None:
        bridge, path = self.bridge, self.path.split('?', 1)[0].rstrip('/') or '/'
        parts = [p for p in path.split('/') if p]
        try:
            if method == 'GET' and (path == '/' or (len(parts) == 1 and parts[0] in STATIC_FILES)):
                file = WEB / ('index.html' if path == '/' else parts[0])
                return self._send(HTTPStatus.OK, file.read_bytes(), CONTENT_TYPES[file.suffix])
            if method == 'GET' and len(parts) == 3 and parts[0] == 'recordings' and bridge.recorder is not None:
                return self._file(bridge.recorder.file(unquote(parts[1]), unquote(parts[2])))
            if parts[:1] != ['api']:
                return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
            route = parts[1:]
            if route[:1] == ['recordings']:
                return self._recordings(method, route[1:])
            if method == 'GET' and route == ['state']:
                return self._json(bridge.snapshot())
            if method == 'GET' and route == ['events']:
                return self._events()
            if method == 'GET' and route == ['missions']:
                bridge.config.reload()
                return self._json(missions.list_missions(bridge.config))
            if method == 'GET' and route == ['template']:
                return self._json(missions.template(bridge.robot))
            if len(route) == 2 and route[0] == 'missions':
                name = missions.check_name(route[1])
                if method == 'GET':
                    bridge.config.reload()
                    return self._json(bridge.config.mission(name))
                if method == 'PUT':
                    old = parse_qs(urlsplit(self.path).query).get('rename_from', [None])[0]
                    path = missions.save(name, self._body(), bridge.config, bridge.schemas(), bridge.robot,
                                         rename_from=old)
                    renamed = f' (renamed from {old})' if old and old != name else ''
                    bridge.get_logger().info(f'HMI saved mission {name}{renamed} -> {path}')
                    return self._json({'saved': name})
                if method == 'DELETE':
                    path = missions.delete(name, bridge.config)
                    bridge.get_logger().info(f'HMI deleted mission {name} ({path})')
                    return self._json({'deleted': name})
            if method == 'GET' and len(route) == 3 and route[0] == 'missions' and route[2] == 'plan':
                bridge.config.reload()
                return self._json(preview_plan(bridge.config, missions.check_name(route[1]), bridge.robot))
            if method == 'POST' and len(route) == 3 and route[0] == 'missions' and route[2] == 'validate':
                problems = missions.validate(self._body(), bridge.config, bridge.schemas(), bridge.robot)
                return self._json({'problems': problems})
            if method == 'POST' and route == ['start']:
                body = self._body()
                response = bridge.call(bridge.start_client, StartMission.Request(
                    mission=body['mission'], start_step=str(body.get('start_step') or '')))
                status = HTTPStatus.OK if response.accepted else HTTPStatus.UNPROCESSABLE_ENTITY
                return self._json({'accepted': response.accepted, 'message': response.message,
                                   'plan_id': response.plan_id}, status)
            if method == 'POST' and len(route) == 1 and route[0] in bridge.triggers:
                response = bridge.call(bridge.triggers[route[0]], Trigger.Request())
                return self._json({'success': response.success, 'state': response.message})
            if method == 'GET' and route == ['skills']:
                return self._json(bridge.catalog())
            if method == 'GET' and route == ['points']:
                return self._json(bridge.config.point_names(bridge.robot))
            if method == 'GET' and route == ['points', 'all']:
                return self._json(points.overview(bridge.config, bridge.robot))
            if method in ('PUT', 'DELETE') and len(route) >= 3 and route[0] == 'points':
                bridge.check_idle()
                kind, address = route[1], [unquote(p) for p in route[2:]]
                if method == 'PUT':
                    file = points.save(bridge.config, bridge.robot, kind, address, self._body())
                else:
                    file = points.delete(bridge.config, bridge.robot, kind, address)
                bridge.get_logger().info(f'HMI {"saved" if method == "PUT" else "deleted"} {kind} '
                                         f'{"/".join(address)} -> {file}')
                return self._json({'saved' if method == 'PUT' else 'deleted': '/'.join(address), 'file': file})
            if method == 'GET' and route == ['robot_pose']:
                return self._json(points.relative_poses(bridge.config, bridge.robot_pose()))
            if method == 'GET' and route == ['robot']:
                return self._json(robot.overview(bridge.config, bridge.robot, bridge.pose_args()))
            if method == 'GET' and route == ['robot', 'joints']:
                return self._json(bridge.joint_positions())
            if method == 'GET' and route == ['robot', 'tools']:
                return self._json(bridge.tool_targets())
            if method in ('PUT', 'DELETE') and len(route) == 3 and route[:2] == ['robot', 'poses']:
                bridge.check_idle()
                name = unquote(route[2])
                if method == 'PUT':
                    file = robot.save_pose(bridge.config, bridge.robot, name, self._body(), bridge.pose_args())
                else:
                    file = robot.delete_pose(bridge.config, bridge.robot, name, bridge.pose_args())
                bridge.get_logger().info(f'HMI {"saved" if method == "PUT" else "deleted"} pose {name} -> {file}')
                return self._json({'pose': name, 'file': file})
            if method == 'PUT' and route == ['robot', 'ik']:
                bridge.check_idle()
                file = robot.save_ik_choice(bridge.config, bridge.robot, self._body())
                bridge.get_logger().info(f'HMI saved arm solutions -> {file}')
                return self._json({'file': file})
            if method == 'PUT' and len(route) == 4 and route[0] == 'objects' and route[2] == 'points':
                bridge.check_idle()
                object_type, point = unquote(route[1]), unquote(route[3])
                file = points.save_object_point(bridge.config, object_type, point, self._body(), bridge.robot)
                bridge.get_logger().info(f'HMI saved object point {object_type}/{point} -> {file}')
                return self._json({'saved': f'{object_type}/{point}', 'file': file})
            if method == 'PUT' and route == ['robot', 'navigation']:
                bridge.check_idle()
                settings = self._body()
                file = robot.save_navigation(bridge.config, bridge.robot, settings)
                applied = bridge.apply_navigation(settings)
                bridge.get_logger().info(f'HMI saved navigation clearance -> {file}; {applied}')
                return self._json({'file': file, 'applied': applied})
            if method == 'POST' and route == ['robot', 'reach']:
                return self._json(bridge.reach(self._body()))
            if method == 'POST' and route == ['points', 'drive']:
                target = str(self._body()['target'])
                return self._json(bridge.run_skill('navigate_to', {'target': target}, timeout_s=DRIVE_TIMEOUT_S))
            if method == 'PUT' and route == ['robot', 'joints']:
                return self._json({'sent': bridge.send_joints(self._body().get('joints') or {})})
            if method == 'GET' and route == ['teach', 'arm']:
                return self._json(arm.overview(bridge.config, bridge.robot, bridge.pose_args()))
            if method == 'PUT' and route == ['teach', 'target']:
                body = self._body()
                target = {k: body.get(k) for k in ('scope', 'name', 'at', 'form')} if body.get('name') else None
                bridge.set_teach_target(target)
                return self._json({'target': target})
            if len(route) >= 4 and route[:2] == ['teach', 'arm']:
                scope, name = unquote(route[2]), unquote(route[3])
                if len(route) == 4 and method in ('PUT', 'DELETE'):
                    bridge.check_idle()
                    if method == 'PUT':
                        file = arm.save(bridge.config, bridge.robot, scope, name, self._body(), bridge.pose_args())
                    else:
                        file = arm.delete(bridge.config, bridge.robot, scope, name, bridge.pose_args())
                    bridge.get_logger().info(f'HMI {"saved" if method == "PUT" else "deleted"} arm position '
                                             f'{scope}/{name} -> {file}')
                    return self._json({'file': file})
                if len(route) == 5 and method == 'POST' and route[4] == 'capture':
                    file = bridge.capture(scope, name, self._body())
                    bridge.get_logger().info(f'HMI captured arm position {scope}/{name} -> {file}')
                    return self._json({'file': file})
                if len(route) == 5 and method == 'POST' and route[4] == 'test':
                    return self._json(bridge.test_position(scope, name, self._body()))
            if len(route) >= 4 and route[:2] == ['teach', 'held']:
                obj, name = unquote(route[2]), unquote(route[3])
                if len(route) == 4 and method in ('PUT', 'DELETE'):
                    bridge.check_idle()
                    if method == 'PUT':
                        file = held.save(bridge.config, bridge.robot, obj, name, self._body())
                    else:
                        file = held.delete(bridge.config, bridge.robot, obj, name)
                    bridge.get_logger().info(f'HMI {"saved" if method == "PUT" else "deleted"} held pose '
                                             f'{obj}/{name} -> {file}')
                    return self._json({'file': file})
                if len(route) == 5 and method == 'POST' and route[4] == 'test':
                    return self._json(bridge.run_skill(held.SKILL, {held.ARG: name}))
            if method == 'POST' and route == ['robot', 'move']:
                return self._json(bridge.run_skill('move_to_pose', {'pose': self._body()['pose']}))
            return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
        except missions.MissionError as error:
            return self._json({'problems': error.problems}, HTTPStatus.UNPROCESSABLE_ENTITY)
        except (points.PointError, robot.RobotSettingsError, arm.ArmError, held.HeldError,
                recording.RecordingError) as error:
            return self._json({'problems': [str(error)]}, HTTPStatus.UNPROCESSABLE_ENTITY)
        except Busy as error:
            return self._json({'problems': [str(error)]}, HTTPStatus.CONFLICT)
        except ConfigError as error:
            return self._json({'problems': [str(error)]}, HTTPStatus.UNPROCESSABLE_ENTITY)
        except (TimeoutError, KeyError, json.JSONDecodeError) as error:
            return self._json({'error': str(error)}, HTTPStatus.SERVICE_UNAVAILABLE
                              if isinstance(error, TimeoutError) else HTTPStatus.BAD_REQUEST)

    def _events(self) -> None:
        listener: queue.Queue = queue.Queue(maxsize=500)
        with self.bridge.lock:
            self.bridge.listeners.append(listener)
        try:
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(f'data: {json.dumps({"type": "state", "data": self.bridge.snapshot()})}\n\n'.encode())
            self.wfile.flush()
            while True:
                try:
                    message = listener.get(timeout=15)
                    if message is None:  # dropped by _broadcast: this client fell too far behind
                        break
                    self.wfile.write(f'data: {message}\n\n'.encode())
                except queue.Empty:
                    self.wfile.write(b': keep-alive\n\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with self.bridge.lock:
                if listener in self.bridge.listeners:
                    self.bridge.listeners.remove(listener)

    def do_GET(self):  # noqa: N802 - http.server API
        self._route('GET')

    def do_PUT(self):  # noqa: N802
        self._route('PUT')

    def do_POST(self):  # noqa: N802
        self._route('POST')

    def do_DELETE(self):  # noqa: N802
        self._route('DELETE')


def serve(bridge: Bridge) -> ThreadingHTTPServer:
    Handler.bridge = bridge
    host, port = bridge.get_parameter('host').value, int(bridge.get_parameter('port').value)
    httpd = ThreadingHTTPServer((host, port), Handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    bridge.get_logger().info(f'HMI on http://{host or "0.0.0.0"}:{httpd.server_address[1]} for robot {bridge.robot!r}')
    return httpd


def main(args=None):
    rclpy.init(args=args)
    bridge = Bridge()
    httpd = serve(bridge)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(bridge)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        httpd.shutdown()
        executor.shutdown()
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



