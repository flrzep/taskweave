"""taskweave web HMI server: a ROS bridge plus a small HTTP server (standard library only).

Parameters:
  config_dir (string)   taskweave config directory (missions are read and written here)
  robot (string)        robot the running task handler drives
  host (string)         address to bind; bind the trusted network only (e.g. the tailnet address)
  port (int)            default 8090
  recordings_dir (string)  where the Record tab keeps recordings (bag, sim videos); empty: no Record tab
  recordings_keep (int)    recordings kept (default 1: each new one replaces the last; 0: all), besides marked ones
  recording_min_free_gb (double)  free disk a recording needs to start (default 30)
  recording_post (string)  command run after each recording, {dir} = its folder (e.g. camera videos from the bag)

HTTP API (JSON):
  GET  /api/state[?since=<seq>]    robot, status, plan with per-step outcomes, the latest 50 events; with since: the
                                   events after that seq (each event has a seq; event_seq = the latest, events_from =
                                   the oldest still held, 200 are)
  GET  /api/events                 server-sent events: status | plan | episode | speed | manual | pad_mode | notice |
                                   points (a location, stance, pose, point or carry pose was saved, deleted or taught,
                                   here or by the pad: GET /api/points again; data null)
  GET  /api/missions               mission list (invalid files are listed with their error)
  GET  /api/missions/<name>        mission as JSON; GET /api/template for a new one
  PUT  /api/missions/<name>        validate and save; 422 with {problems} if invalid, nothing written;
                                   ?rename_from=<old> saves mission <old> as <name> (old file removed)
  DELETE /api/missions/<name>      remove the mission file (a running mission keeps its plan)
  POST /api/missions/<name>/validate  {problems, places}; places (also in a 422 of PUT): per problem of a step
                                   {problem, step, path, include}: the step of the mission's own steps and the path in
                                   it (['args', 'to'], ['skill']), so the editor marks the step and the field
  GET  /api/missions/<name>/plan   the plan a start would run now (includes expanded, facts filled): step ids to
                                   start at; each step with kind (a move step's kind of motion, '') and resources,
                                   as the live plan's steps
  GET  /api/speed                  the global speed override {percent, test_mode, test_percent, cap_percent, presets,
                                   min, max, available}; also in /api/state (speed) and as the live event speed
  POST /api/speed {percent, test_mode, test_percent}  set it through the task handler (each key optional; also while
                                   a mission runs): {ok, message, percent}, 422 when refused (e.g. out of range)
  POST /api/start {mission[, start_step]}  start a mission (at a step id: the first cycle begins there, robot and
                                   objects as they are); POST /api/pause | /api/resume | /api/abort
  GET  /api/skills                 skill catalog (move: x-taskweave-move, the options per kind of motion)
  GET  /api/points                 named points for argument pickers (the store's
                                   lists plus 'held poses' and 'via targets'; on a cell pack in the targets layout
                                   also 'base targets', 'hand targets', 'object targets', 'joint targets')
  GET  /api/points/all             locations, stance offsets and placed objects, with the missions using each point
  PUT  /api/points/location/<name>                          {x, y, yaw[, rename]}  create, update or rename
  PUT  /api/points/stance/<objects|instances>/<owner>/<variant>  {x, y, yaw[, rename]}  (offset in the object frame)
  DELETE the same addresses;  422 with {problems} if refused, nothing written (on a targets cell: 404 with
                                   {problems} when the target, object type or instance is not there)
  GET  /api/robot_pose             current base pose in the cell and relative to every placed object
  POST /api/points/drive {target}  run navigate_to now (no mission running): location name or stance:<object>/<variant>
                                   (when the skill catalog has move: move {to: <name> or <object>/<variant>})
  Manual skills (drive, pose and point tests, held poses; one at a time, never while a mission runs) answer when they
                                   end: {outcome, message, aborted}. While one runs it is in /api/state (manual) and
                                   the live event manual: {skill, label, since (server epoch s), elapsed_s}, else null
  POST /api/skill/abort            cancel the running manual skill: {ok, skill}; 409 {ok: false, message} when none
                                   runs. Never a mission (that is /api/abort). Its own request then answers outcome
                                   canceled, aborted true
  GET  /api/robot                  joint poses (with the missions using each), navigation clearance, footprint
  GET  /api/robot/joints           current joint positions
  GET  /api/robot/tools            current tool targets {tool: {tcp, approach, closing}} (MoveIt forward kinematics)
  GET  /api/robot/model            {robot, urdf, root_link, tool_frames, grippers} for the 3D view: the URDF with mesh
                                   files at /robot/meshes/<file> (404 without a URDF; scene.py)
  GET  /api/robot/base             the base pose in the cell from TF {frame, child, xyz, quat: [w, x, y, z], stamp}
                                   (503 without the transform)
  GET  /api/scene                  {frame, objects, skipped, placed, locations, points, updated}: MoveIt's world
                                   collision objects ([] and updated null without move_group) and the config's
                                   placed objects, locations and object points (scene.py). Only while it is asked for:
                                   the object names at most every 10 s, the geometry when they change
  PUT  /api/robot/poses/<name>     {joints[, rename, create]};  DELETE the same
  PUT  /api/robot/navigation       {envelope: {front, back, left, right}, padding, inflation_radius, driving}: checked
                                   against the robot's base, saved and applied to the running costmaps
                                   (parameter nav_costmaps) and controller (nav_controller)
  POST /api/robot/move {pose}      go to a named pose now (no mission running: move_joints, or move_gripper for a pose
                                   with tool targets); returns the outcome when it ends
  POST /api/robot/reach {joints, tools, search: [tool]}   per tool: ok, message, warnings, nearest reachable target
  PUT  /api/robot/joints {joints: {name: value}}  send joint targets now (no mission running; within the limits)
  GET  /api/teach/arm               arm positions: robot poses and object points as one list (arm.py)
  PUT  /api/teach/arm/<robot|type>/<name>      {hands, joints, elbow, approach_m[, rename]};  DELETE the same
  POST /api/teach/arm/<robot|type>/<name>/capture {at, tools, form[, instance_override]}  save the robot's current arm
                                   there (also creates): {file, message} (the teach node's answer for a point)
  POST /api/teach/arm/<robot|type>/<name>/test {at}  move there now (a robot pose as /api/robot/move; an object point:
                                   move_gripper to <at>/<name>, at the placed object <at>)
  PUT  /api/teach/target {scope, name, at, form} or {}   the position a pad teach press saves into
  PUT  /api/teach/held/<type>/<name> {joints, path, rename}; DELETE the same   held poses (move_object to: {pose})
  POST /api/teach/held/<type>/<name>/test   move the held object there now (move_object; move {to} once served)
  POST /api/teach/held/<type>/<name>/capture   teach it from the held object (targets layout: the teach node's object
                                   kind; 422 with its answer while it cannot be taught yet)
  GET  /api/recordings              {status, recordings, free_bytes} (404 without recordings_dir; recording.py)
  POST /api/recordings/start {mission, name, cycles, reset, bag, camera, video, quality, max_min};  POST .../stop
  DELETE /api/recordings/<id>;  PUT /api/recordings/<id>/keep {keep}  exempt it from replacement
  GET  /recordings/<id>/<file>     a recording's video or data file (byte ranges)
  GET  /recordings/<id>.tar        the whole recording (bag included) as one tar, streamed (finished recordings)
  GET  /robot/meshes/<file>        an STL file from the URDF's meshes/ folder (plain .stl file names only)
  GET  /vendor/<path>              the 3D view's vendored libraries (three.js, urdf-loader) below web/vendor/
Missions and points are edited between runs (409 while a mission runs) and are used at the next start.
"""

from __future__ import annotations

import json
import queue
import shutil
import tarfile
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
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from taskweave_core import names, tf_util
from taskweave_core.config import ConfigError, ConfigStore, Pose2D, TargetError
from taskweave_core.speed import MAX_PERCENT, MIN_PERCENT, PRESETS, SpeedOverride
from taskweave_core.teach import MAX_POSE_AGE_S, yaw_from_quaternion
from taskweave_interfaces.action import RunSkill
from taskweave_interfaces.msg import Plan as PlanMsg
from taskweave_interfaces.msg import SpeedOverride as SpeedOverrideMsg
from taskweave_interfaces.msg import TaskStatus
from taskweave_interfaces.srv import ListSkills, SetSpeedOverride, StartMission, TeachPoint
from taskweave_skills.ik import pose_matrix, tool_target
from taskweave_skills.nav2 import nav2_parameters

from . import arm, held, missions, points, recording, robot, scene, targets

WEB = Path(__file__).parent / 'web'
STATES = ['idle', 'running', 'paused', 'succeeded', 'failed', 'aborted']
CALL_TIMEOUT_S = 5.0
STATIC_FILES = ('app.js', 'robot.js', 'teach.js', 'held.js', 'graph.js', 'record.js', 'viewer3d.js', 'style.css',
                'viewer3d-demo.html')  # the 3D viewer alone, full window: for testing it
VENDOR = WEB / 'vendor'  # the 3D view's libraries, served below /vendor/
MANUAL_SKILL_TIMEOUT_S = 120.0  # work time (sim time x speed override), as the skill's own timeout
DRIVE_TIMEOUT_S = 300.0  # a drive across the cell
NAV_COSTMAPS = ['/local_costmap/local_costmap', '/global_costmap/global_costmap']  # Nav2 defaults
NAV_CONTROLLER = '/controller_server'
FK_SERVICE = '/compute_fk'  # MoveIt move_group
SCENE_SERVICE = '/get_planning_scene'  # MoveIt move_group
SCENE_CHECK_S = 10.0  # while a 3D view asks: how often move_group's object names are compared (cheap); the geometry
                      # (every mesh of the cell, a stall of move_group's scene monitor) only when they change
SCENE_GIVE_UP_S = 30.0  # a fetch with no answer by then is dropped (move_group gone mid-call)
SPEED_CHECK_S = 2.0  # how often the HMI checks that the task handler serves set_speed_override (greys the control out)
OUTCOMES = ['succeeded', 'failed', 'timeout', 'canceled', 'rejected']
CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
                 '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.mp4': 'video/mp4',
                 '.csv': 'text/csv; charset=utf-8', '.json': 'application/json', '.log': 'text/plain; charset=utf-8',
                 '.yaml': 'text/plain; charset=utf-8', '.stl': 'model/stl', '.txt': 'text/plain; charset=utf-8',
                 '.md': 'text/plain; charset=utf-8'}
CHUNK = 1 << 20


def step_dict(step) -> dict:
    """A PlanStep message as the HMI's plan JSON; ``kind`` (a move step's kind of motion, '' for none) and
    ``resources`` are filled when the task handler validated the plan (a message without them: '' and [])."""
    return {'id': step.step_id, 'instruction': step.instruction, 'skill': step.skill,
            'args': json.loads(step.args_json or '{}'), 'timeout_s': step.timeout_s, 'retries': step.retries,
            'on_failure': step.on_failure, 'kind': getattr(step, 'kind', '') or '',
            'resources': list(getattr(step, 'resources', None) or [])}


def preview_plan(config: ConfigStore, mission: str, robot: str, catalog: dict | None = None) -> dict:
    """The plan a start of ``mission`` would run now (missions.preview_plan), with kind and resources per step."""
    return missions.preview_plan(config, mission, robot, catalog)


def teach_fields() -> set[str]:
    """The request fields of the teach service as built (TeachPoint before cell schema 4 has no instance_override)."""
    return set(TeachPoint.Request.get_fields_and_field_types())


def status_dict(msg: TaskStatus) -> dict:
    return {'state': STATES[msg.state], 'plan_id': msg.plan_id, 'goal': msg.goal, 'cycle': msg.cycle,
            'step_index': msg.step_index, 'step_count': msg.step_count,
            'step': step_dict(msg.step) if msg.step_index >= 0 else None, 'attempt': msg.attempt,
            'step_status': msg.step_status, 'step_progress': msg.step_progress,
            'step_elapsed_s': msg.step_elapsed_s, 'last_error': msg.last_error}


def speed_dict(msg: SpeedOverrideMsg | None, available: bool) -> dict:
    """The speed override as the web UI shows it; percent None until the task handler's first (latched) message.
    float32 fields are rounded, so 33.3 does not read 33.29999923."""
    values = {key: None if msg is None else round(float(getattr(msg, key)), 1)
              for key in ('percent', 'test_percent', 'cap_percent')}
    return {**values, 'test_mode': msg is not None and bool(msg.test_mode), 'presets': list(PRESETS),
            'min': MIN_PERCENT, 'max': MAX_PERCENT, 'available': available}


def speed_request(body: dict) -> SetSpeedOverride.Request:
    """The SetSpeedOverride request for a POST /api/speed body: percent, test_percent (numbers) and test_mode (true or
    false), each optional (left out: unchanged). The range is the task handler's to check (its message says why);
    ValueError for values of the wrong type or not above 0 (0 means 'unchanged' to the service)."""
    request = SetSpeedOverride.Request()
    for key in ('percent', 'test_percent'):
        value = body.get(key)
        if value is None:
            continue
        if isinstance(value, bool) or not isinstance(value, int | float) or not value > 0:
            raise ValueError(f'{key}: a number, {MIN_PERCENT:g}-{MAX_PERCENT:g} (percent)')
        setattr(request, key, float(value))
    if body.get('test_mode') is not None:
        if not isinstance(body['test_mode'], bool):
            raise ValueError('test_mode: true or false')
        request.set_test_mode, request.test_mode = True, body['test_mode']
    return request


def await_skill(done, work, limit_s: float, served, sleep=time.sleep) -> str:
    """Wait for a manual skill's result: '' when ``done()``, else why it was given up. The deadline ``limit_s`` is on
    ``work()``, work seconds (sim time scaled by the speed override, as the skill times itself), so a slow sim or a
    low override does not give up on a skill that is still running; a pause stops both. Given up also when the skill
    server is gone (``served()`` false)."""
    while not done():
        if work() > limit_s:
            return f'did not finish in {limit_s:g} s'
        if not served():
            return 'skill server gone'
        sleep(0.1)
    return ''


def args_text(args: dict) -> str:
    """Skill args on one line (a manual skill's label when its caller names none): key=value, nested ones as JSON."""
    return ' '.join(f'{key}={json.dumps(value) if isinstance(value, dict | list) else value}'
                    for key, value in args.items())


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
    skills_client = None   # the task handler's skill list service (set in __init__)
    served_skills: list[str] | None = None   # the skill names of the catalog's last answer

    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'hmi_server'), **kwargs)
        for name, default in (('config_dir', ''), ('robot', ''), ('host', ''), ('port', 8090),
                              ('nav_costmaps', NAV_COSTMAPS), ('nav_controller', NAV_CONTROLLER),
                              ('recordings_dir', ''), ('recording_bag_exclude', recording.BAG_EXCLUDE),
                              ('recordings_keep', 1), ('recording_min_free_gb', 30.0), ('recording_post', '')):
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
        self.event_seq = 0
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
        # the running manual skill, for the browser and for Abort (under self.lock): what it is, its goal handle once
        # the skill server accepted it, and whether Abort was pressed (also before the handle exists)
        self.manual: dict | None = None
        self.manual_handle = None
        self.manual_abort = False
        self.speed = SpeedOverride.for_node(self)  # manual skills wait on work time, like the skills
        # the speed override control: the full message (with test mode and cap) and whether the handler serves it
        self.speed_client = self.create_client(SetSpeedOverride, names.SET_SPEED_OVERRIDE, callback_group=self.group)
        self.speed_msg: SpeedOverrideMsg | None = None
        self.speed_state = speed_dict(None, False)
        self.create_subscription(SpeedOverrideMsg, names.SPEED_OVERRIDE, self._speed_update, latched,
                                 callback_group=self.group)
        self.create_timer(SPEED_CHECK_S, self._speed_update, callback_group=self.group)
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
        self.recorder = recording.Recorder(
            self, directory, self.get_parameter('recording_bag_exclude').value,
            keep=self.get_parameter('recordings_keep').value,
            min_free_gb=self.get_parameter('recording_min_free_gb').value,
            post=self.get_parameter('recording_post').value) if directory else None
        # 3D view: the base pose from TF; MoveIt's collision objects fetched when /api/scene asks (HTTP threads read
        # the copy), never without a 3D view open
        self.tf, self.tf_listener = tf_util.buffer(self)  # cleared when sim time jumps back
        self.cell_frame = self._cell_frame()
        self.collision: dict = {'objects': [], 'skipped': 0, 'updated': None}
        self.scene_sent = 0.0  # monotonic time of the fetch in flight, 0: none
        self.scene_checked = 0.0  # monotonic time of the last names check
        self.scene_names = None  # the object names the geometry copy is of (None: no copy yet)
        try:
            from moveit_msgs.srv import GetPlanningScene
        except ImportError:  # no MoveIt installed: no collision objects
            self.scene_client = None
        else:
            self.scene_client = self.create_client(GetPlanningScene, SCENE_SERVICE, callback_group=self.group)

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

    def _speed_update(self, msg: SpeedOverrideMsg | None = None) -> None:
        """A new override from the task handler, or (timer, no message) a check that it still serves the service;
        broadcast when what the control shows changed."""
        available = self.speed_client.service_is_ready()
        with self.lock:
            if msg is not None:
                self.speed_msg = msg
            state = speed_dict(self.speed_msg, available)
            changed, self.speed_state = state != self.speed_state, state
        if changed:
            self._broadcast('speed', state)

    def set_speed(self, request: SetSpeedOverride.Request) -> dict:
        """Set the override through the task handler, its single write path. Not refused while a mission runs: that
        is when it matters, the skills follow the latched topic at once."""
        response = self.call(self.speed_client, request)
        self.get_logger().info(f'HMI speed override: {response.message}')
        return {'ok': response.ok, 'message': response.message, 'percent': round(float(response.percent), 1)}

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

    def _cell_frame(self) -> str:
        """The cell frame as the teach node takes it: the hand motion's cell_frame (skills/move/hand.yaml, else
        move_gripper's), else 'world'."""
        try:
            return targets.module_config(self.config, 'move_gripper', 'hand').get('cell_frame', 'world')
        except (ConfigError, KeyError):
            return 'world'

    def frame_pose(self, frame: str) -> scene.Transform | None:
        """Where ``frame`` is in the cell frame now (TF): position, quaternion x, y, z, w; None if TF lacks it."""
        from tf2_ros import TransformException

        try:
            t = self.tf.lookup_transform(self.cell_frame, frame, Time()).transform
        except TransformException:
            return None
        return ((t.translation.x, t.translation.y, t.translation.z),
                (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w))

    def base_transform(self) -> dict:
        """The robot base's pose in the cell from TF (the 3D view places the robot model with it)."""
        from tf2_ros import TransformException

        child = self.config.robot(self.robot).base['frame']
        try:
            stamped = self.tf.lookup_transform(self.cell_frame, child, Time())
        except TransformException as error:
            raise TimeoutError(f'no transform {self.cell_frame} -> {child}: {error}') from error
        t, stamp = stamped.transform, stamped.header.stamp
        return {'frame': self.cell_frame, 'child': child,
                'xyz': [t.translation.x, t.translation.y, t.translation.z],
                'quat': [t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z],
                'stamp': stamp.sec + stamp.nanosec * 1e-9}

    def _fetch_scene(self) -> None:
        """Called by /api/scene: at most every SCENE_CHECK_S, ask move_group for its world object names; the geometry
        follows only when they differ from the copy's (_names_received). Nothing is asked while no 3D view is open."""
        from moveit_msgs.msg import PlanningSceneComponents

        if self.scene_client is None:
            return
        if not self.scene_client.service_is_ready():
            self.collision, self.scene_names = {'objects': [], 'skipped': 0, 'updated': None}, None
            return
        now = time.monotonic()
        if self.scene_sent and now - self.scene_sent < SCENE_GIVE_UP_S:
            return  # the last one is still on its way
        if now - self.scene_checked < SCENE_CHECK_S:
            return
        self.scene_checked = now
        self._ask_scene(PlanningSceneComponents.WORLD_OBJECT_NAMES, self._names_received)

    def _ask_scene(self, components: int, done) -> None:
        from moveit_msgs.srv import GetPlanningScene

        request = GetPlanningScene.Request()
        request.components.components = components
        self.scene_sent = time.monotonic()
        self.scene_client.call_async(request).add_done_callback(done)

    def _scene_answer(self, future):
        """The response of a scene request, or None (failed: the last copy is kept, the next check tries again)."""
        self.scene_sent = 0.0
        try:
            return future.result()
        except Exception as error:  # noqa: BLE001 - keep the last copy; the next check tries again
            self.get_logger().warning(f'{SCENE_SERVICE}: {error}')
            return None

    def _names_received(self, future) -> None:
        from moveit_msgs.msg import PlanningSceneComponents

        response = self._scene_answer(future)
        if response is None:
            return
        names = sorted(o.id for o in response.scene.world.collision_objects)
        if names != self.scene_names:
            self._ask_scene(PlanningSceneComponents.WORLD_OBJECT_GEOMETRY, self._scene_received)

    def _scene_received(self, future) -> None:
        response = self._scene_answer(future)
        if response is None:
            return
        world = response.scene.world.collision_objects
        objects, skipped = scene.collision_objects(world, self.cell_frame, self.frame_pose)
        self.collision = {'objects': objects, 'skipped': skipped, 'updated': time.time()}
        self.scene_names = sorted(o.id for o in world)

    def scene_view(self) -> dict:
        """The 3D view's scene: the last fetched collision objects and the config's points (scene.py). Asking for it
        keeps the copy current (_fetch_scene); the first answer of a newly opened view may still be the old copy."""
        self._fetch_scene()
        collision = self.collision
        return {'frame': self.cell_frame, 'objects': collision['objects'], 'skipped': collision['skipped'],
                **scene.config_scene(self.config), 'updated': collision['updated']}

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
        nearest reachable target (TCP moved along a base axis). Solved like move_gripper solves a named pose."""
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

    def run_skill(self, skill: str, args: dict, timeout_s: float = MANUAL_SKILL_TIMEOUT_S,
                  label: str | None = None) -> dict:
        """Run one skill directly (no mission), e.g. move to a joint pose to check it. Blocks until it ends; shown to
        the browser meanwhile (``label``: what it goes to, default the args) and stopped by ``abort_skill``."""
        self.check_idle()
        if not self.skill_lock.acquire(blocking=False):
            raise Busy('another manual skill is running')
        try:
            with self.lock:
                self.manual = {'skill': skill, 'label': label or args_text(args), 'since': time.time()}
                self.manual_handle, self.manual_abort = None, False
                manual = self.manual_view()
            self._broadcast('manual', manual)
            action = self.skill_clients.get(skill)
            if action is None:
                action = self.skill_clients[skill] = ActionClient(self, RunSkill, names.skill_action(skill),
                                                                  callback_group=self.group)
            if not action.wait_for_server(timeout_sec=CALL_TIMEOUT_S):
                raise TimeoutError(f'skill {skill} is not served')
            goal = RunSkill.Goal(step_id='hmi', skill=skill, args_json=json.dumps(args), timeout_s=timeout_s,
                                 instruction=f'{skill} from the HMI')
            with self.lock:
                aborted = self.manual_abort
            if aborted:  # Abort while the skill server was looked for: nothing sent, nothing moved
                return {'outcome': 'canceled', 'message': 'aborted before it started', 'aborted': True}
            sent = action.send_goal_async(goal)
            deadline = time.monotonic() + CALL_TIMEOUT_S
            while not sent.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not sent.done() or not sent.result().accepted:
                raise TimeoutError(f'{skill} did not accept the goal')
            handle = sent.result()
            with self.lock:
                self.manual_handle, aborted = handle, self.manual_abort
            if aborted:  # Abort came while the goal was on its way: cancel it now that there is a handle
                handle.cancel_goal_async()
            result = handle.get_result_async()
            clock = self.speed.clock(self.now_s())
            problem = await_skill(result.done, lambda: clock(self.now_s()), timeout_s + CALL_TIMEOUT_S,
                                  action.server_is_ready)
            if problem:  # stop it: the lock is released below, and a next manual skill must not run alongside
                handle.cancel_goal_async()
                deadline = time.monotonic() + CALL_TIMEOUT_S
                while not result.done() and time.monotonic() < deadline:
                    time.sleep(0.05)
                raise TimeoutError(f'{skill} {problem}; cancelled')
            outcome = result.result().result
            with self.lock:
                aborted = self.manual_abort
            return {'outcome': OUTCOMES[outcome.outcome], 'message': outcome.message, 'aborted': aborted}
        finally:
            with self.lock:
                self.manual, self.manual_handle, self.manual_abort = None, None, False
            self._broadcast('manual', None)
            self.skill_lock.release()

    def manual_view(self) -> dict | None:
        """The running manual skill as the browser shows it (call under self.lock). elapsed_s lets the browser count
        on from its own clock, whatever the server's clock says."""
        if self.manual is None:
            return None
        return {**self.manual, 'elapsed_s': round(time.time() - self.manual['since'], 1)}

    def abort_skill(self) -> dict | None:
        """Abort the running manual skill (POST /api/skill/abort): cancel its goal or, before the skill server has
        accepted it, have run_skill cancel it as soon as it has (or not send it at all). None when no manual skill
        runs. Missions are not touched: they are the task handler's, with their own abort."""
        with self.lock:
            if self.manual is None:
                return None
            manual, handle, self.manual_abort = dict(self.manual), self.manual_handle, True
        if handle is not None:
            handle.cancel_goal_async()
        pending = '' if handle is not None else ' (before its goal was accepted)'
        self.get_logger().info(f'HMI aborted manual skill {manual["skill"]} {manual["label"]}{pending}')
        return {'ok': True, 'skill': manual['skill']}

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def set_teach_target(self, target: dict | None) -> None:
        with self.lock:
            self.teach_target = target or None
        self.teach_target_pub.publish(String(data=json.dumps(target) if target else ''))

    def teach(self, kind: str, name: str, tools=(), instance_override: bool = False):
        """Ask the teach node to save the robot's current pose as target ``name`` of ``kind`` (base, hand, joints,
        object): the request of the cell's layout (targets.teach_request). Refused: an ArmError with the node's
        message (``hand 'tray_1/H05': ...``)."""
        try:
            fields = targets.teach_request(self.config, teach_fields(), kind, name, tools, instance_override)
        except ValueError as error:
            raise arm.ArmError(str(error)) from error
        response = self.call(self.teach_client, TeachPoint.Request(**fields))
        if not response.success:
            raise arm.ArmError(response.message)
        return response

    def _teach_request(self, msg: String) -> None:
        """A pad teach press (operator_joy): save the robot's arm into the position open in the HMI."""
        with self.lock:
            target = self.teach_target
        if not target:
            return
        tools = json.loads(msg.data or '{}').get('tools') or []
        label = f'{target["scope"]} {target["name"]}'
        try:
            result = self.capture(target['scope'], target['name'], {'at': target.get('at'), 'tools': tools,
                                                                    'form': target.get('form')})
            notice = {'text': f'taught {result["message"] or label} -> {result["file"]}', 'ok': True,
                      'taught': target}
            self.points_changed()
        except (arm.ArmError, Busy, TimeoutError, KeyError, ConfigError) as error:
            notice = {'text': f'teach {label} failed: {error}', 'ok': False}
        self.notice_pub.publish(String(data=json.dumps(notice)))

    def points_changed(self) -> None:
        """Tell every browser that the named points changed (a save from any browser or the pad): the mission
        editor's argument pickers fetch /api/points again."""
        self._broadcast('points', None)

    def capture(self, scope: str, name: str, body: dict) -> dict:
        """Save the robot's current arm as an arm position: a robot pose from the joint states and forward
        kinematics, an object point (a hand target of the type) through the teach node at placed object ``at``
        (default: the nearest; ``instance_override``: for that instance only). ``{file, message}``: the file written
        and the teach node's answer ('' for a robot pose)."""
        self.check_idle()
        if scope == arm.ROBOT:
            try:
                tool_targets = self.tool_targets()
            except TimeoutError:
                tool_targets = None
            return {'file': arm.capture_pose(self.config, self.robot, name, body, self.joint_positions(),
                                             tool_targets, self.pose_args()), 'message': ''}
        at = body.get('at') or self.nearest_placed(scope)
        instance = self.config.object_instances().get(at)
        if instance is None or instance.object != scope:
            raise arm.ArmError(f'{at} is not a placed {scope}')
        response = self.teach('hand', f'{at}/{name}', list(body.get('tools') or []),
                              bool(body.get('instance_override')))
        grip = arm.closed_grips(self.config, self.robot, list(body.get('tools') or []), self.joint_positions())
        if grip:  # grippers closed on the part when taught: they close that far at this point
            points.save_object_point(self.config, scope, name, {'grip_m': grip}, self.robot)
        return {'file': self.relative(response.file), 'message': response.message}

    def capture_held(self, name: str) -> dict:
        """Teach a held pose (an object target in the base frame) from where the held object is now: the teach
        node's ``object`` kind, which answers that it cannot be taught yet (the HMI shows its message)."""
        self.check_idle()
        if not targets.active(self.config):
            raise held.HeldError('a held pose cannot be captured: type in its numbers and try it with Test')
        try:
            response = self.teach('object', name)
        except arm.ArmError as error:
            raise held.HeldError(str(error)) from error
        return {'file': self.relative(response.file), 'message': response.message}

    def relative(self, file: str) -> str:
        """A file the teach node wrote, relative to the config folder when it is in it."""
        try:
            return Path(file).relative_to(self.config.root).as_posix()
        except ValueError:
            return file

    def nearest_placed(self, object_type: str) -> str:
        return arm.nearest_placed(self.config, object_type,
                                  points.relative_poses(self.config, self.robot_pose())['relative'])

    def go_to_pose(self, name: str) -> dict:
        """Run the skill that takes the robot to named pose ``name``: ``move {to}`` when the catalog has move, else
        move_gripper for one with tool targets (IK), else move_joints."""
        if self.serves_move():
            try:
                self.config.target(name, self.robot, kinds=('joints', 'hand'),
                                   context='a robot pose is a joint or hand target')
            except TargetError as error:
                raise arm.ArmError(str(error)) from error
            return self.run_skill(targets.MOVE, targets.move_args(name), label=name)
        try:
            tools = self.config.named_pose(self.robot, name)['tools']
        except KeyError as error:
            raise arm.ArmError(str(error.args[0])) from error
        if tools:
            return self.run_skill('move_gripper', {'to': {'pose': name}}, label=name)
        return self.run_skill('move_joints', {'pose': name}, label=name)

    def test_position(self, scope: str, name: str, body: dict) -> dict:
        if scope == arm.ROBOT:
            return self.go_to_pose(name)
        at = body.get('at') or self.nearest_placed(scope)
        if self.serves_move():
            return self.run_skill(targets.MOVE, targets.move_args(f'{at}/{name}', hands=body.get('hands'),
                                                                  offset_m=body.get('offset_m')),
                                  label=f'{at}/{name}')
        return self.run_skill('move_gripper', {'to': {'point': f'{at}/{name}'}}, label=f'{at}/{name}')

    def point_names(self) -> dict[str, list[str]]:
        """Named points for argument pickers (points.picker_names): the store's lists, held poses, via targets and,
        on a targets cell, the target groups."""
        return points.picker_names(self.config, self.robot)

    def serves_move(self) -> bool:
        """The manual skills run ``move {to}``: the skill catalog has move. Asked live; without an answer the last
        one counts, without any what the cell pack configures. A bridge without a node (tests) has only the last
        answer, else runs the old skills."""
        if self.skills_client is None:
            return targets.MOVE in (self.served_skills or ())
        try:
            names = [skill['name'] for skill in self.catalog()]
        except TimeoutError:
            names = self.served_skills
        return targets.serves_move(names, self.config)

    def drive(self, target: str) -> dict:
        """Drive to a location or stance now (POST /api/points/drive): ``move {to}`` with the reference as move
        takes it, else navigate_to with the old reference."""
        if self.serves_move():
            reference = targets.target_reference(self.config, self.robot, target)
            return self.run_skill(targets.MOVE, targets.move_args(reference), timeout_s=DRIVE_TIMEOUT_S, label=target)
        return self.run_skill('navigate_to', {'target': target}, timeout_s=DRIVE_TIMEOUT_S, label=target)

    def test_held(self, obj: str, name: str) -> dict:
        skill, args = held.test_request(name, self.serves_move())
        return self.run_skill(skill, args, label=f'{obj} {name}')

    def send_joints(self, values: dict) -> dict:
        """Joint targets straight to the robot (the HMI's joint panel): within the limits, no mission running."""
        from taskweave_core.config.limits import joint_limits, joint_problems, mimic_joints

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
        mimics = mimic_joints(self.config, self.robot)
        followers = [f'{joint} follows {mimics[joint]}' for joint in values if joint in mimics]
        if followers:
            raise arm.ArmError('set the joint they follow: ' + '; '.join(followers))
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
            self.event_seq += 1
            event['seq'] = self.event_seq
            self.outcomes[event.get('step_id')] = event
            self.events.append(event)
        if getattr(self, 'recorder', None) is not None:
            self.recorder.episode(event)
        self._broadcast('episode', event)

    def _pad_mode(self, msg: String) -> None:
        with self.lock:
            self.pad_mode = msg.data
        self._broadcast('pad_mode', msg.data)

    def _notice(self, msg: String) -> None:
        self._broadcast('notice', json.loads(msg.data))

    def snapshot(self, since: int | None = None) -> dict:
        with self.lock:
            events = list(self.events)
            return {'robot': self.robot, 'status': self.status, 'plan': self.plan, 'outcomes': dict(self.outcomes),
                    'events': events[-50:] if since is None else [e for e in events if e['seq'] > since],
                    'event_seq': self.event_seq, 'events_from': events[0]['seq'] if events else self.event_seq + 1,
                    'pad_mode': self.pad_mode, 'speed': self.speed_state, 'manual': self.manual_view()}

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
        self.served_skills = [s.name for s in response.skills]
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

    def _archive(self, rec_id: str) -> None:
        """A recording as an uncompressed tar, written while it is read (bag and videos are compressed already; no
        copy on the box's disk). No Content-Length: the connection closes at the end."""
        _folder, members = self.bridge.recorder.archive(rec_id)
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'application/x-tar')
        self.send_header('Content-Disposition', f'attachment; filename="{rec_id}.tar"')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.close_connection = True
        try:
            with tarfile.open(fileobj=self.wfile, mode='w|') as tar:
                for path, name in members:
                    tar.add(path, arcname=name, recursive=False)
        except (BrokenPipeError, ConnectionResetError):
            pass  # the browser cancelled the download
        self.bridge.get_logger().info(f'HMI sent recording {rec_id} ({len(members)} files)')

    def _recordings(self, method: str, route: list[str]):
        recorder = self.bridge.recorder
        if recorder is None:
            return self._json({'error': 'recording is not set up (parameter recordings_dir)'}, HTTPStatus.NOT_FOUND)
        if method == 'GET' and not route:
            free = shutil.disk_usage(recorder.dir).free if recorder.dir.is_dir() else None
            return self._json({'status': recorder.status(), 'recordings': recorder.recordings(), 'free_bytes': free,
                               'keep': recorder.keep, 'min_free_gb': recorder.min_free_gb})
        if method == 'POST' and route == ['start']:
            self.bridge.config.reload()
            return self._json(recorder.start(self._body(), self.bridge.config.mission_names()))
        if method == 'POST' and route == ['stop']:
            return self._json(recorder.stop())
        if method == 'PUT' and len(route) == 2 and route[1] == 'keep':
            return self._json({'keep': recorder.set_keep(unquote(route[0]), bool(self._body().get('keep')))})
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
                if not file.is_file():  # listed but not there (e.g. viewer3d.js in a build without the 3D view)
                    return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
                return self._send(HTTPStatus.OK, file.read_bytes(), CONTENT_TYPES[file.suffix])
            if (method == 'GET' and len(parts) == 2 and parts[0] == 'recordings' and parts[1].endswith('.tar')
                    and bridge.recorder is not None):
                return self._archive(unquote(parts[1])[:-len('.tar')])
            if method == 'GET' and len(parts) >= 2 and parts[0] == 'vendor':
                file = scene.vendor_file(VENDOR, [unquote(p) for p in parts[1:]])
                return self._file(file) if file else self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
            if method == 'GET' and len(parts) == 3 and parts[:2] == ['robot', 'meshes']:
                file = scene.mesh_file(bridge.config, bridge.robot, unquote(parts[2]))
                return self._file(file) if file else self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
            if method == 'GET' and len(parts) == 3 and parts[0] == 'recordings' and bridge.recorder is not None:
                return self._file(bridge.recorder.file(unquote(parts[1]), unquote(parts[2])))
            if parts[:1] != ['api']:
                return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
            route = parts[1:]
            if route[:1] == ['recordings']:
                return self._recordings(method, route[1:])
            if method == 'GET' and route == ['state']:
                since = parse_qs(urlsplit(self.path).query).get('since', [''])[0]
                if since and not since.isdigit():
                    return self._json({'error': 'since: an event seq (whole number)'}, HTTPStatus.BAD_REQUEST)
                return self._json(bridge.snapshot(int(since) if since else None))
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
                try:
                    catalog = bridge.schemas()   # the other skills' resources; a move step's kind needs none
                except TimeoutError:
                    catalog = {}
                return self._json(preview_plan(bridge.config, missions.check_name(route[1]), bridge.robot, catalog))
            if method == 'POST' and len(route) == 3 and route[0] == 'missions' and route[2] == 'validate':
                problems, places = missions.check(self._body(), bridge.config, bridge.schemas(), bridge.robot)
                return self._json({'problems': problems, 'places': places})
            if method == 'GET' and route == ['speed']:
                return self._json(bridge.speed_state)  # replaced on change, never changed in place
            if method == 'POST' and route == ['speed']:
                try:
                    request = speed_request(self._body())
                except ValueError as error:  # a body that is not JSON, too
                    return self._json({'error': str(error)}, HTTPStatus.BAD_REQUEST)
                result = bridge.set_speed(request)
                return self._json(result, HTTPStatus.OK if result['ok'] else HTTPStatus.UNPROCESSABLE_ENTITY)
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
                return self._json(targets.annotate_catalog(bridge.catalog()))
            if method == 'GET' and route == ['points']:
                return self._json(bridge.point_names())
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
                bridge.points_changed()
                return self._json({'saved' if method == 'PUT' else 'deleted': '/'.join(address), 'file': file})
            if method == 'GET' and route == ['robot_pose']:
                return self._json(points.relative_poses(bridge.config, bridge.robot_pose()))
            if method == 'GET' and route == ['robot']:
                return self._json(robot.overview(bridge.config, bridge.robot, bridge.pose_args()))
            if method == 'GET' and route == ['robot', 'joints']:
                return self._json(bridge.joint_positions())
            if method == 'GET' and route == ['robot', 'tools']:
                return self._json(bridge.tool_targets())
            if method == 'GET' and route == ['robot', 'model']:
                model = scene.robot_model(bridge.config, bridge.robot)
                if model is None:
                    error = f'robot {bridge.robot} has no URDF (robots/{bridge.robot}/moveit/robot.urdf)'
                    return self._json({'error': error}, HTTPStatus.NOT_FOUND)
                return self._json(model)
            if method == 'GET' and route == ['robot', 'base']:
                return self._json(bridge.base_transform())
            if method == 'GET' and route == ['scene']:
                return self._json(bridge.scene_view())
            if method in ('PUT', 'DELETE') and len(route) == 3 and route[:2] == ['robot', 'poses']:
                bridge.check_idle()
                name = unquote(route[2])
                if method == 'PUT':
                    file = robot.save_pose(bridge.config, bridge.robot, name, self._body(), bridge.pose_args())
                else:
                    file = robot.delete_pose(bridge.config, bridge.robot, name, bridge.pose_args())
                bridge.get_logger().info(f'HMI {"saved" if method == "PUT" else "deleted"} pose {name} -> {file}')
                bridge.points_changed()
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
                bridge.points_changed()
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
                return self._json(bridge.drive(str(self._body()['target'])))
            if method == 'POST' and route == ['skill', 'abort']:
                aborted = bridge.abort_skill()
                if aborted is None:
                    return self._json({'ok': False, 'message': 'no manual skill is running'}, HTTPStatus.CONFLICT)
                return self._json(aborted)
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
                    bridge.points_changed()
                    return self._json({'file': file})
                if len(route) == 5 and method == 'POST' and route[4] == 'capture':
                    result = bridge.capture(scope, name, self._body())
                    bridge.get_logger().info(f'HMI captured arm position {scope}/{name} -> {result["file"]}')
                    bridge.points_changed()
                    return self._json(result)
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
                    bridge.points_changed()
                    return self._json({'file': file})
                if len(route) == 5 and method == 'POST' and route[4] == 'test':
                    return self._json(bridge.test_held(obj, name))
                if len(route) == 5 and method == 'POST' and route[4] == 'capture':
                    result = bridge.capture_held(name)
                    bridge.points_changed()
                    return self._json(result)
            if method == 'POST' and route == ['robot', 'move']:
                return self._json(bridge.go_to_pose(self._body()['pose']))
            return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
        except missions.MissionError as error:
            return self._json({'problems': error.problems, 'places': error.places}, HTTPStatus.UNPROCESSABLE_ENTITY)
        except (points.PointError, robot.RobotSettingsError, arm.ArmError, held.HeldError,
                recording.RecordingError) as error:
            return self._json({'problems': [str(error)]}, HTTPStatus.UNPROCESSABLE_ENTITY)
        except Busy as error:
            return self._json({'problems': [str(error)]}, HTTPStatus.CONFLICT)
        except ConfigError as error:
            return self._json({'problems': [str(error)]}, HTTPStatus.UNPROCESSABLE_ENTITY)
        except TargetError as error:  # a target, object type or instance a Teach edit names is not there
            return self._json({'problems': [str(error)]}, HTTPStatus.NOT_FOUND)
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



