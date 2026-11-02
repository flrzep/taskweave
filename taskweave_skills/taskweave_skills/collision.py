"""A held object in MoveIt's planning scene: the object type's ``collision`` shapes, attached to the hold's first link.

``HoldCollision.attach`` (grip close) sends an AttachedCollisionObject through /apply_planning_scene, posed where the
object is relative to the link at that moment, and ``detach`` (open, simulator reset) detaches and removes it. The
touch links (that may touch the object) are the grippers' ``links`` of the holding hands (profile ``grippers.<hand>``),
else their tool frame link. An object type without ``collision`` sends nothing. A scene call that fails (no service,
no answer, refused) only logs a warning: a grip never fails because of it.

``in_scene`` says whether a held object's shapes are attached in the scene now: attached at the grip, and still among
the scene's attached objects (/get_planning_scene; a respawned move_group has lost them). ``move`` asks it before a
hand of a hold moves to a hand target or relative to now.

Profile ``planning`` keys: ``scene_service`` (default /apply_planning_scene), ``scene_get_service`` (default
/get_planning_scene), ``scene_timeout_s`` (default 2.0, the longest a scene call may take).
"""

from __future__ import annotations

import time

import numpy as np

from taskweave_core.node_cache import for_node

from .ik import quaternion, rpy_matrix

POLL_S = 0.02   # wait for a scene answer in steps of this
DEFAULTS = {'scene_service': '/apply_planning_scene', 'scene_get_service': '/get_planning_scene',
            'scene_timeout_s': 2.0}


def shapes_of(collision) -> list[dict]:
    """An object type's ``collision`` value (one shape or a list) as a list; none: empty."""
    return [] if not collision else (collision if isinstance(collision, list) else [collision])


def primitive(shape: dict):
    """(SolidPrimitive, 4x4 offset in the object frame) of one config shape."""
    from shape_msgs.msg import SolidPrimitive
    if 'box' in shape:
        solid = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in shape['box']])
    else:  # SolidPrimitive.CYLINDER dimensions: [height, radius]
        solid = SolidPrimitive(type=SolidPrimitive.CYLINDER,
                               dimensions=[float(shape['cylinder']['height']), float(shape['cylinder']['radius'])])
    return solid, rpy_matrix(shape.get('xyz', [0, 0, 0]), shape.get('rpy', [0, 0, 0]))


def touch_links(robot_data: dict, hands) -> list[str]:
    """Links that may touch a held object: the ``hands``' gripper links (profile ``grippers``), else their tool frame
    link."""
    grippers, frames = robot_data.get('grippers', {}), robot_data.get('tool_frames', {})
    return list(dict.fromkeys(link for hand in hands
                              for link in (grippers.get(hand, {}).get('links') or [frames[hand]['link']])))


class HoldCollision:
    _instances: dict = {}

    @classmethod
    def for_node(cls, node, robot, hold) -> HoldCollision:
        return for_node(cls._instances, node, lambda n: cls(n, robot, hold))

    def __init__(self, node, robot, hold):
        self.node, self.robot, self.hold = node, robot, hold
        spec = {**DEFAULTS, **(robot.data.get('planning') or {})}
        self.service, self.timeout = spec['scene_service'], float(spec['scene_timeout_s'])
        self.get_service = spec['scene_get_service']
        self.client = None                   # made at the first call: a robot without collision shapes never needs it
        self.get_client = None
        self.attached: dict[str, str] = {}   # object -> link it hangs on
        self.shapes: dict[str, list[dict]] = {}   # object -> its shapes (the object may be gone from the cell by now)
        hold.on_reset.append(self.detach_all)

    def _links(self, hands) -> list[str]:
        """Links that may touch the object: the hands' gripper links (else the tool frame link)."""
        return touch_links({**self.robot.data, 'tool_frames': self.hold.tool_frames}, hands)

    def attach(self, name: str, shapes: list[dict] | None = None) -> None:
        """Attach the shapes of held object ``name`` (an existing attachment is replaced: the hands changed).
        Without ``shapes``: those of the last attach."""
        shapes = shapes or self.shapes.get(name)
        hold = next((h for h in self.hold.held_all() if h['object'] == name), None)
        if not shapes or hold is None:
            return
        try:
            link = self.hold.tool_frames[hold['hands'][0]]['link']
            object_in_link = np.linalg.inv(self.hold.transform(self.hold.base, link)) @ hold['object_in_base']
            message = self._attached(name, link, shapes, object_in_link, self._links(hold['hands']))
        except Exception as error:  # noqa: BLE001 - e.g. no TF: the grip stands
            self._warn(f'cannot attach {name} to the planning scene: {error}')
            return
        if name in self.attached:
            self.detach(name)
        if self._apply(self._scene(message), f'attach {name}'):
            self.attached[name], self.shapes[name] = link, shapes

    def detach(self, name: str) -> None:
        """Detach held object ``name`` and remove it from the scene (nothing if it was never attached)."""
        from moveit_msgs.msg import CollisionObject
        link = self.attached.pop(name, None)
        self.shapes.pop(name, None)
        if link is None:
            return
        self._apply(self._scene(self._remove(name, link)), f'detach {name}')   # it becomes a world object ...
        scene = self._scene(None)
        scene.world.collision_objects = [CollisionObject(id=name, operation=CollisionObject.REMOVE)]
        self._apply(scene, f'remove {name}')                                   # ... that is removed now

    def detach_all(self) -> None:
        for name in list(self.attached):
            self.detach(name)

    def in_scene(self, name: str) -> str | None:
        """None if the shapes of held object ``name`` are attached in the planning scene now, else why not."""
        if name not in self.attached:
            return 'its shape was not attached at the grip'
        from moveit_msgs.msg import PlanningSceneComponents
        from moveit_msgs.srv import GetPlanningScene
        try:
            if self.get_client is None:
                from rclpy.callback_groups import ReentrantCallbackGroup
                self.get_client = self.node.create_client(GetPlanningScene, self.get_service,
                                                          callback_group=ReentrantCallbackGroup())
            request = GetPlanningScene.Request()
            request.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            answer, why = self._call(self.get_client, self.get_service, request)
        except Exception as error:  # noqa: BLE001 - a scene that cannot be read is not known to hold the shape
            answer, why = None, str(error)
        if answer is None:
            return f'cannot read the planning scene: {why}'
        if name not in {a.object.id for a in answer.scene.robot_state.attached_collision_objects}:
            return 'the planning scene has lost it (move_group restarted?)'
        return None

    # ---- messages -----------------------------------------------------------------
    @staticmethod
    def _attached(name, link, shapes, object_in_link, touch_links):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        obj = CollisionObject(id=name, operation=CollisionObject.ADD)
        obj.header.frame_id = link
        for shape in shapes:
            solid, offset = primitive(shape)
            pose = object_in_link @ offset
            w, x, y, z = quaternion(pose[:3, :3])
            msg = Pose()
            msg.position.x, msg.position.y, msg.position.z = (float(v) for v in pose[:3, 3])
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
            obj.primitives.append(solid)
            obj.primitive_poses.append(msg)
        return AttachedCollisionObject(link_name=link, object=obj, touch_links=touch_links)

    @staticmethod
    def _remove(name, link):
        from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
        return AttachedCollisionObject(link_name=link,
                                       object=CollisionObject(id=name, operation=CollisionObject.REMOVE))

    @staticmethod
    def _scene(attached):
        from moveit_msgs.msg import PlanningScene
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        if attached is not None:
            scene.robot_state.attached_collision_objects = [attached]
        return scene

    # ---- service ------------------------------------------------------------------
    def _apply(self, scene, what: str) -> bool:
        """Send a scene diff; True if MoveIt applied it. Failures are logged, never raised."""
        from moveit_msgs.srv import ApplyPlanningScene
        try:
            if self.client is None:
                from rclpy.callback_groups import ReentrantCallbackGroup
                self.client = self.node.create_client(ApplyPlanningScene, self.service,
                                                      callback_group=ReentrantCallbackGroup())
            answer, why = self._call(self.client, self.service, ApplyPlanningScene.Request(scene=scene))
            if answer is None:
                self._warn(f'cannot {what} in the planning scene: {why}')
                return False
            if not answer.success:
                self._warn(f'cannot {what} in the planning scene: {self.service} refused it')
                return False
            return True
        except Exception as error:  # noqa: BLE001 - never fail a grip over the scene
            self._warn(f'cannot {what} in the planning scene: {error}')
            return False

    def _call(self, client, service: str, request):
        """(answer, None), or (None, why) when the service is missing or does not answer within the timeout."""
        if not client.wait_for_service(timeout_sec=self.timeout):
            return None, f'no service {service}'
        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout
        while not future.done() and time.monotonic() < deadline:   # the executor's other threads run the reply
            time.sleep(POLL_S)
        if not future.done():
            future.cancel()
            return None, f'no answer from {service} in {self.timeout:g} s'
        return future.result(), None

    def _warn(self, text: str) -> None:
        self.node.get_logger().warning(text)
