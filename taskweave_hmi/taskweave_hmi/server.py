"""taskweave web HMI server: a ROS bridge plus a small HTTP server (standard library only).

Parameters:
  config_dir (string)   taskweave config directory (missions are read and written here)
  robot (string)        robot the running task handler drives
  host (string)         address to bind; bind the trusted network only (e.g. the tailnet address)
  port (int)            default 8090

HTTP API (JSON):
  GET  /api/state                  robot, status, plan with per-step outcomes, recent events
  GET  /api/events                 server-sent events: status | plan | episode
  GET  /api/missions               mission list (invalid files are listed with their error)
  GET  /api/missions/<name>        mission as JSON; GET /api/template for a new one
  PUT  /api/missions/<name>        validate and save; 422 with {problems} if invalid, nothing written
  POST /api/missions/<name>/validate
  POST /api/start {mission}        start a mission; POST /api/pause | /api/resume | /api/abort
  GET  /api/skills                 skill catalog;  GET /api/points  named points for argument pickers
  GET  /api/points/all             locations, stance offsets and placed objects, with the missions using each point
  PUT  /api/points/location/<name>                          {x, y, yaw[, rename]}  create, update or rename
  PUT  /api/points/stance/<objects|instances>/<owner>/<variant>  {x, y, yaw[, rename]}  (offset in the object frame)
  DELETE the same addresses;  422 with {problems} if refused, nothing written
  GET  /api/robot_pose             current base pose in the cell and relative to every placed object
Missions and points are edited between runs (409 while a mission runs) and are used at the next start.
"""

from __future__ import annotations

import json
import queue
import threading
import time
from collections import deque
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote

import rclpy
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger

from taskweave_core import names
from taskweave_core.config import ConfigError, ConfigStore, Pose2D
from taskweave_core.teach import MAX_POSE_AGE_S, yaw_from_quaternion
from taskweave_interfaces.msg import Plan as PlanMsg
from taskweave_interfaces.msg import TaskStatus
from taskweave_interfaces.srv import ListSkills, StartMission

from . import missions, points

WEB = Path(__file__).parent / 'web'
STATES = ['idle', 'running', 'paused', 'succeeded', 'failed', 'aborted']
CALL_TIMEOUT_S = 5.0
CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
                 '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml'}


def step_dict(step) -> dict:
    return {'id': step.step_id, 'instruction': step.instruction, 'skill': step.skill,
            'args': json.loads(step.args_json or '{}'), 'timeout_s': step.timeout_s, 'retries': step.retries,
            'on_failure': step.on_failure}


def status_dict(msg: TaskStatus) -> dict:
    return {'state': STATES[msg.state], 'plan_id': msg.plan_id, 'goal': msg.goal, 'cycle': msg.cycle,
            'step_index': msg.step_index, 'step_count': msg.step_count,
            'step': step_dict(msg.step) if msg.step_index >= 0 else None, 'attempt': msg.attempt,
            'step_status': msg.step_status, 'step_progress': msg.step_progress,
            'step_elapsed_s': msg.step_elapsed_s, 'last_error': msg.last_error}


class Busy(Exception):
    """The request would change config while a mission runs."""


class Bridge(Node):
    def __init__(self, **kwargs):
        super().__init__(kwargs.pop('node_name', 'hmi_server'), **kwargs)
        for name, default in (('config_dir', ''), ('robot', ''), ('host', ''), ('port', 8090)):
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
        self.start_client = self.create_client(StartMission, names.START_MISSION, callback_group=self.group)
        self.skills_client = self.create_client(ListSkills, names.LIST_SKILLS, callback_group=self.group)
        self.triggers = {cmd: self.create_client(Trigger, getattr(names, cmd.upper()), callback_group=self.group)
                         for cmd in ('pause', 'resume', 'abort')}
        self.pose: Pose2D | None = None
        self.pose_stamp = 0.0
        self.odom_topic = self.config.robot(self.robot).base['odom']
        self.create_subscription(Odometry, self.odom_topic, self._odom, 10, callback_group=self.group)

    # ---- ROS -> state ------------------------------------------------------------
    def _broadcast(self, kind: str, data) -> None:
        message = json.dumps({'type': kind, 'data': data})
        with self.lock:
            for listener in list(self.listeners):
                listener.put(message)

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

    def check_idle(self) -> None:
        with self.lock:
            state = self.status['state']
        if state in ('running', 'paused'):
            raise Busy(f'a mission is {state}; edit points when it has finished or was aborted')

    def _episode(self, msg: String) -> None:
        event = json.loads(msg.data)
        with self.lock:
            if event.get('event') == 'step_started' and event.get('step_id') in self.outcomes \
                    and self.outcomes[event['step_id']].get('cycle') != event.get('cycle'):
                self.outcomes = {}  # a new cycle starts over
            self.outcomes[event.get('step_id')] = event
            self.events.append(event)
        self._broadcast('episode', event)

    def snapshot(self) -> dict:
        with self.lock:
            return {'robot': self.robot, 'status': self.status, 'plan': self.plan, 'outcomes': dict(self.outcomes),
                    'events': list(self.events)[-50:]}

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

    def _route(self, method: str) -> None:
        bridge, path = self.bridge, self.path.split('?', 1)[0].rstrip('/') or '/'
        parts = [p for p in path.split('/') if p]
        try:
            if method == 'GET' and (path == '/' or (len(parts) == 1 and parts[0] in ('app.js', 'style.css'))):
                file = WEB / ('index.html' if path == '/' else parts[0])
                return self._send(HTTPStatus.OK, file.read_bytes(), CONTENT_TYPES[file.suffix])
            if parts[:1] != ['api']:
                return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
            route = parts[1:]
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
                    path = missions.save(name, self._body(), bridge.config, bridge.schemas(), bridge.robot)
                    bridge.get_logger().info(f'HMI saved mission {name} -> {path}')
                    return self._json({'saved': name})
            if method == 'POST' and len(route) == 3 and route[0] == 'missions' and route[2] == 'validate':
                problems = missions.validate(self._body(), bridge.config, bridge.schemas(), bridge.robot)
                return self._json({'problems': problems})
            if method == 'POST' and route == ['start']:
                response = bridge.call(bridge.start_client, StartMission.Request(mission=self._body()['mission']))
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
            return self._json({'error': 'not found'}, HTTPStatus.NOT_FOUND)
        except missions.MissionError as error:
            return self._json({'problems': error.problems}, HTTPStatus.UNPROCESSABLE_ENTITY)
        except points.PointError as error:
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
                    self.wfile.write(f'data: {message}\n\n'.encode())
                except queue.Empty:
                    self.wfile.write(b': keep-alive\n\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            with self.bridge.lock:
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



