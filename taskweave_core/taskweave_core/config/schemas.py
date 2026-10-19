"""JSON Schemas (draft 2020-12) for every taskweave config file.

Kept as Python data so they are versioned with the code that reads them. Application-specific content
(e.g. a cell layout) is not described here; taskweave only validates the files it interprets itself.
"""

from ..embodiment import CAPABILITIES, TOOL_TYPES

FINITE = {'type': 'number'}
ELBOW_SIDES = ['down', 'up', 'out', 'in', 'back', 'forward']  # preferred elbow side, in the robot's frame (ik.py)
NAME = {'type': 'string', 'pattern': r'^[A-Za-z_][A-Za-z0-9_\-]*$'}


def default(schema: dict, *path: str):
    """The documented default of a setting: the ``default`` annotation at ``path`` in ``schema`` (property names, '*'
    for any name: additionalProperties), e.g. ``default(ROBOT_PROFILE, 'ik', 'choose', 'limit_margin_deg')``."""
    for key in path:
        schema = schema['additionalProperties'] if key == '*' else schema['properties'][key]
    return schema['default']


# robot profile motion_tuning: how the arm motion code steps (taskweave_skills chain.py, cartesian.py, ik.py,
# motion_plan.py). Every key is optional; these defaults are the values the motion code was tuned with (Oct 2026).
MOTION_TUNING_DEFAULTS = {
    'chain': {                    # one damped least-squares step on a URDF chain (chain.Chain.solve_step)
        'max_step_rad': 0.08,     # the largest joint change one step may make (rad or m); larger steps are scaled down
        'damping': 0.05,          # lambda at a singularity
        'singular': 0.04,         # below this smallest singular value the damping fades in
        'limit_margin': 1e-3,     # stop this far short of a joint limit (rad or m)
    },
    'line': {                     # a straight-line step on the chain (cartesian.chain_step)
        'limit_push_rad': 0.02,   # the most a step moves a joint back from a limit through the null space
        'on_line_m': 5e-4,        # a step is done when the flange is this close to the line point (position)
        'on_line_rad': 5e-3,      # ... and orientation
        'step_iterations': 8,     # solver iterations per step at most
        'push_iterations': 3,     # while a joint is within the limit margin, at least this many (null-space push)
        'min_step_s': 1e-3,       # waypoints of straight lines and elbow swings at least this far apart (s)
    },
    'elbow': {                    # elbow side and swing (cartesian.elbow_push, elbow_swing_path; profile arms.*.elbow)
        'rate_rad': 0.03,         # the most a step swings the elbow toward the preferred side (null space)
        'settled_rad': 0.1,       # the elbow is on its side when it is this close (deviation)
        'swing_max_steps': 200,   # steps per leg of an elbow swing at most
        'via_out_deg': 150.0,     # an elbow this far off its side (nearly opposite) swings toward out first ...
        'via_out_until_deg': 120.0,   # ... until it is this close (never through the body)
        'probe_rad': 0.01,        # step along the self-motion that tells which way round the elbow swings
        'progress_rad': 1e-4,     # a swing step must bring the elbow at least this much closer to its side
        'limit_edge_parts': 5,    # closer than limit_margin_deg / this to a limit, a swing moves a joint only away
    },
    'continuation': {             # a short move continues the arm's configuration (ik.IkClient._continue)
        'max_m': 0.25,            # targets at most this far from the flange now ...
        'max_deg': 45.0,          # ... and turned at most this much are walked to on the chain first
        'elbow_deg': 90.0,        # a continuation ending with the elbow further off its side loses to the IK search
        'jump_deg': 120.0,        # a joint turning more than this for a short move (no continuation) is logged
    },
    'plan': {                     # collision-free joint motion (motion_plan.MotionPlanner)
        'tries': 3,               # planning requests when MoveIt's answer is worth a retry
        'deeper_m': 0.005,        # an escape from a start in contact may push a contact no deeper than this more
        'unknown_depth_m': 0.3,   # FCL's depth between meshes is sometimes meters off: deeper than this says nothing
    },
}


def _tuning_schema(defaults: dict) -> dict:
    return {'type': 'object', 'additionalProperties': False, 'properties': {
        key: _tuning_schema(value) if isinstance(value, dict)
        else {'type': 'integer', 'minimum': 1, 'default': value} if isinstance(value, int)
        else {'type': 'number', 'minimum': 0, 'default': value} for key, value in defaults.items()}}


MOTION_TUNING = _tuning_schema(MOTION_TUNING_DEFAULTS)
IK_WEIGHTS = {'limits': 2.0, 'elbow': 1.0, 'motion': 0.2, 'symmetry': 0.5}   # ik.choose.weights defaults
# profile arms.<tool>.elbow: the links at the arm's shoulder, elbow and wrist centres (the elbow side is the swivel of
# the elbow about the shoulder-wrist line); an arm without it has no elbow control
ELBOW = {'type': 'object', 'required': ['shoulder', 'elbow', 'wrist'], 'additionalProperties': False,
         'properties': {key: {'type': 'string', 'minLength': 1} for key in ('shoulder', 'elbow', 'wrist')}}

POSE2D = {
    'type': 'object',
    'required': ['x', 'y', 'yaw'],
    'properties': {'frame': {'type': 'string'}, 'x': FINITE, 'y': FINITE, 'yaw': FINITE},
    'additionalProperties': False,
}

POSE3D = {
    'type': 'object',
    'required': ['xyz', 'rpy'],
    'properties': {
        'frame': {'type': 'string'},
        'xyz': {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3},
        'rpy': {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3},
    },
    'additionalProperties': False,
}

ON_FAILURE = {'enum': ['retry', 'replan', 'skip', 'abort']}

STEP = {
    'type': 'object',
    'required': ['instruction'],
    # a skill call, or another mission's steps (include, with its params filled in from `with`)
    'oneOf': [{'required': ['skill'], 'not': {'required': ['include']}},
              {'required': ['include'], 'not': {'anyOf': [{'required': ['skill']}, {'required': ['args']}]}}],
    'properties': {
        'id': NAME,
        'instruction': {'type': 'string', 'minLength': 1},
        'skill': NAME,
        'include': NAME,
        'with': {'type': 'object', 'propertyNames': NAME},
        'args': {'type': 'object'},
        'timeout_s': {'type': 'number', 'exclusiveMinimum': 0},
        'retries': {'type': 'integer', 'minimum': 0},
        'on_failure': ON_FAILURE,
        'replan_from': NAME,
        'max_replans': {'type': 'integer', 'minimum': 0},
    },
    'additionalProperties': False,
}

MISSION = {
    'type': 'object',
    'required': ['goal', 'steps'],
    'properties': {
        'goal': {'type': 'string', 'minLength': 1},
        'robot': NAME,
        'loop': {'enum': ['once', 'forever']},
        'max_cycles': {'type': 'integer', 'minimum': 1},
        'stop_when': {'type': 'string', 'minLength': 1},
        # parameters a mission takes when another one includes it: name -> what it is (the point kind, if any, lets
        # the mission editor offer the right names)
        'params': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {
            'type': 'object', 'additionalProperties': False,
            'properties': {'description': {'type': 'string'},
                           'point': {'enum': ['base_target', 'joint_pose', 'tool_pose', 'object_point',
                                              'instance_point', 'instance_contact', 'instance']}}}},
        # facts each cycle starts from, filled into the steps as {name} (MissionFilePlanner): fullest = the
        # placed object
        # of this type with the most items left, among instance names matching the glob. When a fact finds nothing
        # (e.g. every stack is empty) the mission is complete.
        'facts': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {
            'type': 'object', 'required': ['fullest'], 'additionalProperties': False,
            'properties': {'fullest': NAME, 'among': {'type': 'string', 'minLength': 1}}}},
        'defaults': {
            'type': 'object',
            'properties': {
                'timeout_s': {'type': 'number', 'exclusiveMinimum': 0},
                'retries': {'type': 'integer', 'minimum': 0},
                'on_failure': ON_FAILURE,
                'max_replans': {'type': 'integer', 'minimum': 0},
            },
            'additionalProperties': False,
        },
        'steps': {'type': 'array', 'items': STEP, 'minItems': 1},
    },
    'additionalProperties': False,
}

ROBOT_PROFILE = {
    'type': 'object',
    'required': ['name', 'base'],
    'properties': {
        'name': NAME,
        'base': {
            'type': 'object',
            'required': ['type', 'cmd_vel', 'odom', 'frame'],
            'properties': {
                'type': {'enum': ['holonomic', 'differential', 'fixed']},
                'cmd_vel': {'type': 'string'},
                'odom': {'type': 'string'},
                'frame': {'type': 'string'},
                'footprint': {'type': 'array', 'items': FINITE, 'minItems': 2, 'maxItems': 2},
                'limits': {
                    'type': 'object',
                    'properties': {'linear': {'type': 'number', 'exclusiveMinimum': 0},
                                   'angular': {'type': 'number', 'exclusiveMinimum': 0}},
                    'additionalProperties': False,
                },
            },
        },
        'joints': {
            'type': 'object',
            'required': ['command', 'state'],
            'properties': {'command': {'type': 'string'}, 'state': {'type': 'string'},
                           'linear': {'type': 'array', 'items': {'type': 'string'}},  # prismatic joints (m)
                           # joint groups for move_joints `only` (torso, head, left_arm, ...): group -> its joints;
                           # <tool>_arm groups the profile does not list come from the ik joints regex and the URDF
                           'groups': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {
                               'type': 'array', 'minItems': 1, 'items': {'type': 'string'}, 'uniqueItems': True}}},
        },
        'grippers': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'properties': {'joint': {'type': 'string'}, 'open': FINITE, 'closed': FINITE,
                               'action': {'type': 'string'},
                               # joint distance from open beyond which the gripper is closed on a part (HMI capture)
                               'closed_from': {'type': 'number', 'exclusiveMinimum': 0, 'default': 0.1},
                               # [joint, finger opening m] rows, measured: a point's grip_m -> joint (gripper.py)
                               'opening_m': {'type': 'array', 'minItems': 2, 'items': {
                                   'type': 'array', 'items': FINITE, 'minItems': 2, 'maxItems': 2}}},
            },
        },
        'tool_frames': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'required': ['link', 'xyz', 'rpy'],
                'properties': {'link': {'type': 'string'},
                               'xyz': {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3},
                               'rpy': {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3},
                               # the grasp is the same turned 180 deg about the approach: IK may use either
                               'flip': {'type': 'boolean'}},
                'additionalProperties': False,
            },
        },
        # inverse kinematics for tool targets: a GetPositionIK service (MoveIt) and the planning group per tool
        'ik': {
            'type': 'object',
            'required': ['groups', 'joints'],
            'properties': {
                'service': {'type': 'string'},
                'timeout_s': {'type': 'number', 'exclusiveMinimum': 0},
                'fallback_timeout_s': {'type': 'number', 'exclusiveMinimum': 0},
                'groups': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {'type': 'string'}},
                'joints': {'type': 'string'},   # regex of the joints IK may move, {tool} = tool name
                # groups with a global solver, tried when no start below reaches the target
                'fallback_groups': {'type': 'object', 'propertyNames': NAME,
                                    'additionalProperties': {'type': 'string'}},
                # known-good postures IK starts from, in order (then the current state)
                'seeds': {'type': 'array', 'items': {'type': 'object', 'additionalProperties': FINITE}},
                'avoid_collisions': {'type': 'boolean'},  # reject solutions in collision (robot + planning scene)
                # deprecated (use arms.<tool>.swivel): the elbow jog's swivel joint, {tool} = tool name
                'swivel_joint': {'type': 'string'},
                # tool -> the tool whose joints it mirrors with the same values (names with the prefix swapped)
                'mirror': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {'type': 'string'}},
                # pick the best of several solutions (taskweave_skills/ik.py): joint-limit room, elbow side, move size
                'choose': {
                    'type': 'object',
                    'properties': {
                        'elbow': {'oneOf': [{'enum': ELBOW_SIDES},
                                            {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3}]},
                        # deprecated (use arms.<tool>.elbow): shoulder, elbow, wrist links, {tool} = tool name
                        'elbow_links': {'type': 'array', 'items': {'type': 'string'}, 'minItems': 3, 'maxItems': 3},
                        'limit_margin_deg': {'type': 'number', 'minimum': 0, 'default': 25.0},
                        'weights': {'type': 'object',
                                    'properties': {k: {'type': 'number', 'minimum': 0, 'default': v}
                                                   for k, v in IK_WEIGHTS.items()},
                                    'additionalProperties': False},
                        'fk_service': {'type': 'string'},
                        'starts': {'type': 'integer', 'minimum': 0},
                        'start_timeout_s': {'type': 'number', 'exclusiveMinimum': 0, 'default': 0.05},
                    },
                    'additionalProperties': False,
                },
            },
            'additionalProperties': False,
        },
        # collision-free joint motion: a GetMotionPlan service (MoveIt) and the group that covers a pose's joints
        'planning': {
            'type': 'object',
            'required': ['group'],
            'properties': {
                'service': {'type': 'string'},
                'group': {'type': 'string'},
                'pipeline': {'type': 'string'},
                'planner': {'type': 'string'},
                'timeout_s': {'type': 'number', 'exclusiveMinimum': 0},
                'attempts': {'type': 'integer', 'minimum': 1},
                'velocity_scale': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 1},
                'acceleration_scale': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 1},
                'joints': {'type': 'string'},   # regex: the group's joints; those not in a goal are held
                'direct': {'type': 'boolean'},  # a straight joint move when it is free (taskweave_skills.motion_plan)
                'validity_service': {'type': 'string'},
                **{k: {'type': 'number', 'exclusiveMinimum': 0}
                   for k in ('direct_step_rad', 'escape_max_rad', 'escape_speed', 'goal_tolerance_rad',
                             'check_timeout_s', 'answer_margin_s')},
            },
            'additionalProperties': False,
        },
        # arm roles by name (e.g. group, tool_frame); per IK tool the links of its elbow (ELBOW) and the swivel joint:
        # the joint the pad's elbow jog turns (without a URDF: then IK puts the TCP back; with one: its positive way)
        'arms': {'type': 'object', 'propertyNames': NAME,
                 'additionalProperties': {'type': 'object', 'properties': {
                     'elbow': ELBOW, 'swivel': {'type': 'string', 'minLength': 1}}}},
        'motion_tuning': MOTION_TUNING,
        'torso': {'type': 'object'},
        'cameras': {'type': 'object'},
        'teleop': {'type': 'object'},
    },
}

# robots/<model>/poses.yaml: named poses. Either joint name -> value (rad or m), or tool targets solved by IK at run
# time: per tool (profile tool_frames) the canonical TCP position and its approach (TCP z) and closing (TCP y)
# directions in the base frame, plus joints set directly (e.g. torso lift, head).
VECTOR3 = {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3}
TOOL_TARGET = {
    'type': 'object',
    'required': ['tcp', 'approach', 'closing'],
    'properties': {'tcp': VECTOR3, 'approach': VECTOR3, 'closing': VECTOR3},
    'additionalProperties': False,
}
JOINT_MAP = {'type': 'object', 'minProperties': 1, 'additionalProperties': FINITE}
TOOL_POSE = {
    'type': 'object',
    'required': ['tools'],
    'properties': {
        'joints': {'type': 'object', 'additionalProperties': FINITE},
        'tools': {'type': 'object', 'propertyNames': NAME, 'minProperties': 1, 'additionalProperties': TOOL_TARGET},
        'elbow': {'enum': ELBOW_SIDES},   # preferred elbow side for this pose (else the profile's ik.choose.elbow)
    },
    'additionalProperties': False,
}
JOINT_POSES = {
    'type': 'object',
    'propertyNames': NAME,
    'additionalProperties': {'oneOf': [JOINT_MAP, TOOL_POSE]},
}

POINTS2D = {'type': 'object', 'additionalProperties': POSE2D, 'propertyNames': NAME}

# robots/<model>/navigation.yaml: how much room a path planner keeps around the robot while it drives. The envelope
# covers the robot in its driving pose (arms included), as distances from the base frame; padding and inflation as
# in Nav2 (padding grows the envelope, inflation adds a cost gradient beyond it).
POSITIVE = {'type': 'number', 'exclusiveMinimum': 0}
ENVELOPE = {
    'type': 'object',
    'required': ['front', 'back', 'left', 'right'],
    'properties': {side: POSITIVE for side in ('front', 'back', 'left', 'right')},
    'additionalProperties': False,
}
NAVIGATION = {
    'type': 'object',
    'required': ['envelope'],
    'properties': {
        'envelope': ENVELOPE,
        'carry_envelope': ENVELOPE,   # while the robot holds an object: the robot plus the object (navigate_to)
        'padding': {'type': 'number', 'minimum': 0},
        'inflation_radius': POSITIVE,
        # how the planner may drive: speed limits (m/s, rad/s), sideways only for a holonomic base (checked against
        # the robot profile), prefer_forward = weight for facing along the path and not reversing (0 = no preference)
        'driving': {
            'type': 'object',
            'properties': {
                'max_forward': POSITIVE,
                'max_reverse': {'type': 'number', 'minimum': 0},
                'max_turn': POSITIVE,
                'sideways': {'type': 'boolean'},
                'max_sideways': {'type': 'number', 'minimum': 0},
                'prefer_forward': {'type': 'number', 'minimum': 0},
                # the path angle weight with prefer_forward 0: follow the path, any heading (Nav2 PathAngleCritic)
                'any_heading_weight': {'type': 'number', 'minimum': 0, 'default': 2.0},
            },
            'additionalProperties': False,
        },
    },
    'additionalProperties': False,
}

# cell/objects.yaml: placed object instances, name -> object type and pose in the cell frame
OBJECT_INSTANCES = {
    'type': 'object',
    'propertyNames': NAME,
    'additionalProperties': {
        'type': 'object',
        'required': ['object', 'x', 'y', 'yaw'],
        'properties': {'object': NAME, 'frame': {'type': 'string'}, 'x': FINITE, 'y': FINITE, 'z': FINITE,
                       'yaw': FINITE,
                       # a stack of nested items from z up: count items pitch apart; points refer to the top one
                       'stack': {'type': 'object', 'required': ['count', 'pitch'], 'additionalProperties': False,
                                 'properties': {'count': {'type': 'integer', 'minimum': 0}, 'pitch': FINITE}}},
        'additionalProperties': False,
    },
}

# <state dir>/objects.yaml: what happened to placed objects at run time, over cell/objects.yaml (ConfigStore
# object_instances): {count: n} items left on a stack, {gone: true} taken away, or a whole entry for an object put down
OBJECT_STATE = {
    'type': 'object',
    'propertyNames': NAME,
    'additionalProperties': {
        'type': 'object',
        'properties': {**OBJECT_INSTANCES['additionalProperties']['properties'],
                       'count': {'type': 'integer', 'minimum': 0}, 'gone': {'const': True}},
        'additionalProperties': False,
    },
}

# robots/<model>/stances.yaml: base poses relative to objects. Named variants (e.g. pick, place) per object type,
# optionally overridden per object instance. The first variant of a type is its default.
STANCE_VARIANTS = {'type': 'object', 'propertyNames': NAME, 'additionalProperties': POSE2D, 'minProperties': 1}
STANCES = {
    'type': 'object',
    'properties': {
        'objects': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': STANCE_VARIANTS},
        'instances': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': STANCE_VARIANTS},
    },
    'additionalProperties': False,
}

OBJECT = {
    'type': 'object',
    'required': ['name'],
    'properties': {
        'name': NAME,
        'mesh': {'type': 'string'},
        # the object is handled the same when turned by this angle about its vertical axis (e.g. 180: a frame lying
        # the other way round is approached and grasped as if it lay the usual way); 360 / angle turns are equivalent
        'yaw_symmetry_deg': {'enum': [90, 120, 180]},
        # a placed object of this type is a place the next one comes to (e.g. a carrier waiting on a conveyor): taking
        # it, or gripping it to work it, leaves the placed object where it is
        'replenished': {'type': 'boolean'},
        'points': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'properties': {
                    'approach': {'type': 'object'},
                    'tool_type': {'enum': sorted(TOOL_TYPES)},
                    'grip_width_m': {'type': 'number', 'exclusiveMinimum': 0},
                    # go_to_point: the last part of the way in a straight line (else the skill's approach_m)
                    'approach_m': {'type': 'number', 'minimum': 0},
                    # preferred elbow side at this point, robot frame (else the robot profile's ik.choose.elbow)
                    'elbow': {'enum': ELBOW_SIDES},
                    # single-hand grasps of the same point (move_gripper hands: [<hand>]): <hand>_tool poses that take
                    # the place of the pair's pose for that hand when it goes alone
                    'alone': {'type': 'object', 'propertyNames': {'pattern': r'.+_tool$'},
                              'additionalProperties': POSE3D},
                },
                'additionalProperties': POSE3D,
            },
        },
        # frames on the object where it meets another (a hook plate, the hook it goes on): z = the direction the held
        # object moves to engage (e.g. down onto a hook); move_object puts a held object's contact onto a placed one's
        'contacts': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': POSE3D},
    },
}

# robots/<model>/object_points.yaml: how this robot takes points of object types (object -> point -> settings): the
# elbow side for its arm, and how far each gripper stays open when it closes at the point (grip_m, finger opening in
# m, e.g. a rail's thickness; the profile's opening_m table gives the joint); the points themselves stay
# robot-independent in objects/<object>.yaml
ROBOT_OBJECT_POINTS = {
    'type': 'object', 'propertyNames': NAME,
    'additionalProperties': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {
        'type': 'object', 'additionalProperties': False, 'properties': {
            'elbow': {'enum': ELBOW_SIDES},
            'grip_m': {'type': 'object', 'propertyNames': NAME,
                       'additionalProperties': {'type': 'number', 'minimum': 0}},
        }}},
}

# robots/<model>/held_poses.yaml: where this robot holds an object of a type, by name (move_object pose): joints
# outside the arms first (e.g. the torso lift; the object moves with the arms), then straight lines through the
# waypoints: the object's origin in the robot's base frame (xyz m, rpy rad)
HELD_POSES = {
    'type': 'object', 'propertyNames': NAME,
    'additionalProperties': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {
        'type': 'object', 'additionalProperties': False,
        'properties': {'joints': {'type': 'object', 'additionalProperties': FINITE},
                       'path': {'type': 'array', 'items': POSE3D}}}},
}

OPERATOR = {
    'type': 'object',
    'properties': {
        'joy': {
            'type': 'object',
            'required': ['topic'],
            'properties': {
                'topic': {'type': 'string'},
                'pause_resume_button': {'type': 'integer', 'minimum': 0},
                'abort_button': {'type': 'integer', 'minimum': 0},
                # pad modes: drive (pad passed on to drive_topic) or jog one tool's TCP (operator_joy.py)
                'drive_topic': {'type': 'string'},
                'drive_button': {'type': 'integer', 'minimum': 0},
                'arms': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {
                    'type': 'object', 'required': ['button', 'twist'],
                    'properties': {'button': {'type': 'integer', 'minimum': 0}, 'twist': {'type': 'string'}},
                    'additionalProperties': False}},
                'jog': {'type': 'object', 'additionalProperties': False, 'properties': {
                    **{k: {'type': 'integer', 'minimum': 0}
                       for k in ('enable_button', 'fast_button', 'tilt_axis', 'elbow_axis')},
                    **{k: {'type': 'number', 'exclusiveMinimum': 0}
                       for k in ('speed', 'fast_speed', 'turn', 'fast_turn', 'elbow_turn', 'fast_elbow_turn')}}},
                # teach object points from the pad (operator_joy.py): button + <instance>/<point>
                'teach': {'type': 'object', 'required': ['button'], 'additionalProperties': False, 'properties': {
                    'button': {'type': 'integer', 'minimum': 0}, 'point': {'type': 'string'},
                    # held together with button, so a stray press stores nothing (default: jog.enable_button)
                    'hold_button': {'type': 'integer', 'minimum': 0}}},
                'actions': {
                    'type': 'array',
                    'items': {
                        'type': 'object',
                        'required': ['button', 'skill'],
                        'properties': {'button': {'type': 'integer', 'minimum': 0}, 'skill': NAME,
                                       'args': {'type': 'object'}, 'arm_args': {'type': 'object'}},
                        'additionalProperties': False,
                    },
                },
            },
            'additionalProperties': False,
        },
    },
    'additionalProperties': False,
}

REQUIRES = {
    'type': 'object',
    'properties': {
        'capabilities': {'type': 'array', 'items': {'enum': sorted(CAPABILITIES)}},
        'action_space': {'type': 'string'},
        'cameras': {'type': 'array', 'items': {'type': 'string'}},
        'embodiments': {'type': 'array', 'items': NAME},
    },
    'additionalProperties': False,
}

# skills/<skill>.yaml: implementations (first match is served) plus parameters shared by all of them
SKILL_PARAMS = {
    'type': 'object',
    'properties': {
        'implementations': {
            'type': 'array',
            'minItems': 1,
            'items': {
                'type': 'object',
                'required': ['name', 'type', 'class'],
                'properties': {
                    'name': NAME,
                    'type': {'enum': ['script', 'vla', 'rl', 'other']},
                    'class': {'type': 'string', 'pattern': r'^[A-Za-z_][\w.]*:[A-Za-z_]\w*$'},
                    'requires': REQUIRES,
                    'params': {'type': 'object'},
                },
                'additionalProperties': False,
            },
        },
    },
}

UNKNOWN_OR_NUMBER = {'type': ['number', 'null'], 'minimum': 0}
RANGE = {'oneOf': [{'type': 'null'},
                   {'type': 'array', 'items': FINITE, 'minItems': 2, 'maxItems': 2}]}

# robots/<model>/embodiment.yaml: what the robot is and can do. No topics or joint names.
EMBODIMENT = {
    'type': 'object',
    'required': ['name', 'summary', 'capabilities'],
    'properties': {
        'name': NAME,
        'summary': {'type': 'string', 'minLength': 1},
        'capabilities': {'type': 'array', 'items': {'enum': sorted(CAPABILITIES)}, 'uniqueItems': True},
        'base': {
            'type': 'object',
            'properties': {
                'footprint': {'oneOf': [{'type': 'null'},
                                        {'type': 'array', 'items': FINITE, 'minItems': 2, 'maxItems': 2}]},
                'max_speed': {'type': 'object',
                              'properties': {'linear': UNKNOWN_OR_NUMBER, 'angular': UNKNOWN_OR_NUMBER},
                              'additionalProperties': False},
            },
            'additionalProperties': False,
        },
        'torso': {
            'type': 'object',
            'properties': {'lift_range_m': RANGE, 'pitch_range_rad': RANGE},
            'additionalProperties': False,
        },
        'arms': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'properties': {'dof': {'type': 'integer', 'minimum': 1}, 'reach_m': UNKNOWN_OR_NUMBER,
                               'payload_kg': UNKNOWN_OR_NUMBER, 'tool': NAME},
                'additionalProperties': False,
            },
        },
        'tools': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'required': ['type'],
                'properties': {'type': {'enum': sorted(TOOL_TYPES)}, 'stroke_m': UNKNOWN_OR_NUMBER,
                               'finger_length_m': UNKNOWN_OR_NUMBER},
                'additionalProperties': False,
            },
        },
        'cameras': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'required': ['type'],
                'properties': {'type': {'enum': ['rgb', 'rgbd', 'depth']}, 'hfov_deg': UNKNOWN_OR_NUMBER,
                               'range_m': RANGE, 'mounted_on': {'type': 'string'}},
                'additionalProperties': False,
            },
        },
        'action_spaces': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'required': ['dims', 'rate_hz'],
                'properties': {'dims': {'type': 'integer', 'minimum': 1},
                               'rate_hz': {'type': 'number', 'exclusiveMinimum': 0},
                               'description': {'type': 'string'}},
                'additionalProperties': False,
            },
        },
    },
    'additionalProperties': False,
}

# pack.yaml at the root of a cell pack or a robot pack (packs.py); a robot pack's name is the robot
PACK = {
    'type': 'object',
    'required': ['name', 'version', 'schema_version'],
    'properties': {
        'name': NAME,
        'version': {'type': ['string', 'integer']},             # the pack's own version, e.g. 0.1.0
        'schema_version': {'type': 'integer', 'minimum': 1},    # its layout, checked against packs.py
        'description': {'type': 'string'},
    },
    'additionalProperties': False,
}

# system.yaml: the installation. taskweave reads robot_packs (robot pack locations, relative to the cell pack, searched
# after env TASKWEAVE_ROBOT_PACKS); other keys belong to the application (e.g. robot: the robot it drives)
SYSTEM = {
    'type': 'object',
    'properties': {
        'robot': NAME,
        'robot_packs': {'type': 'array', 'items': {'type': 'string', 'minLength': 1}},
    },
}


# ---- motion options shared by the arm skills (skill-api-spec 1.0, 2) --------------------------------------------
# Skills embed MOTION (or motion_schema(keys) for a subset) as the ``motion`` argument. ``default`` is a hint for the
# editor and is never written to a mission; the real defaults come from motion.resolve (skill yaml, robot override,
# target data). x-taskweave-show-if names sibling keys of the same object only.
MOTION_SPEED_KEYS = ('tcp_m_s', 'tcp_rad_s', 'joint_rad_s', 'joint_linear_m_s')  # standard and absolute speeds
MOTION_PROPERTIES = {
    'mode': {'enum': ['free', 'linear'], 'default': 'free',
             'description': 'free: joint space, planned around obstacles. linear: the TCP on a straight line.'},
    'speed': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 1, 'default': 1, 'x-taskweave-unit': 'fraction',
              'description': 'Fraction of the skill standard speed (1 = standard). The global speed override applies '
                             'on top.'},
    'speed_abs': {'type': 'object', 'minProperties': 1, 'additionalProperties': False,
                  'properties': {key: {'type': 'number', 'exclusiveMinimum': 0} for key in MOTION_SPEED_KEYS},
                  'description': 'Absolute speeds instead of the fraction. Capped by the robot limits.'},
    'accel': {'type': 'number', 'exclusiveMinimum': 0, 'maximum': 1, 'default': 1, 'x-taskweave-unit': 'fraction',
              'description': 'Fraction of the standard acceleration.'},
    'elbow': {'enum': ELBOW_SIDES,
              'description': "Elbow side kept or reached (robot frame). Default: the point's, else the robot's."},
    'planner': {'enum': ['auto', 'direct', 'planned'], 'default': 'auto',
                'description': 'auto: straight joint move if free, else elbow swing first, else planner. direct: '
                               'straight joint move only. planned: always the planner.'},
    'approach_m': {'type': 'number', 'minimum': 0, 'x-taskweave-unit': 'm', 'x-taskweave-show-if': {'mode': ['free']},
                   'description': 'free mode: the last part of the way, straight along the approach. 0 = planned all '
                                  'the way.'},
    'torso': {'enum': ['keep', 'auto'], 'default': 'keep',
              'description': 'auto: if the target is out of reach, try the configured torso heights. keep: never move '
                             'the torso.'},
    'via': {'type': 'array', 'items': {'type': 'string', 'x-taskweave-point': 'via_target'},
            'description': 'Reserved. Waypoints to pass: "pose:<joint pose>" or "<instance>/<point>".'},
    'tolerance': {'type': 'number', 'exclusiveMinimum': 0,
                  'description': 'End accuracy: m for hand skills, rad or m for joints.'},
}
MOTION_GROUP = 'Motion (advanced)'


def motion_schema(keys=None) -> dict:
    """The ``motion`` argument schema with only ``keys`` (default: all of MOTION_PROPERTIES). A fresh copy, so a
    skill may add hints (e.g. a different ``default``)."""
    import copy

    keys = list(MOTION_PROPERTIES) if keys is None else list(keys)
    unknown = [key for key in keys if key not in MOTION_PROPERTIES]
    if unknown:
        raise KeyError(f'unknown motion option {", ".join(unknown)}; known: {", ".join(MOTION_PROPERTIES)}')
    return {'type': 'object', 'additionalProperties': False, 'x-taskweave-group': MOTION_GROUP,
            'properties': {key: copy.deepcopy(MOTION_PROPERTIES[key]) for key in keys}}


MOTION = motion_schema()

# robots/<model>/skill_overrides.yaml (robot pack, optional): per skill, this robot's own motion options and standard
# speeds on top of skills/<skill>.yaml (motion.resolve, layer 2). Same keys as the skill yaml's motion/speed/accel.
_SKILL_SPEEDS = {'type': 'object', 'additionalProperties': False,
                 'properties': {key: POSITIVE for key in MOTION_SPEED_KEYS + ('base_m_s', 'base_rad_s')}}
SKILL_OVERRIDES = {
    'type': 'object', 'propertyNames': NAME,
    'additionalProperties': {
        'type': 'object', 'additionalProperties': False,
        'properties': {
            'motion': {'type': 'object', 'additionalProperties': False,
                       'properties': {key: MOTION_PROPERTIES[key] for key in MOTION_PROPERTIES
                                      if key not in ('speed_abs', 'via')}},
            'speed': _SKILL_SPEEDS,
            'accel': {'type': 'object', 'additionalProperties': {'type': 'number', 'exclusiveMinimum': 0}},
        },
    },
}


# ---- hands and vectors shared by the gripper skills (skill-api-spec 1.0, 3.1) ------------------------------------
VEC3 = {'type': 'array', 'items': {'type': 'number'}, 'minItems': 3, 'maxItems': 3, 'x-taskweave-widget': 'vec3'}
HAND_RULES = ('auto', 'any_one', 'all')


def hands_schema(hand_names=(), rules=HAND_RULES) -> dict:
    """The ``hands`` argument: a rule, a hand name or a list of names. ``hand_names``: the robot's hands (profile
    grippers), filled in by the skill's setup; ``rules``: the rule words the skill takes."""
    names = list(hand_names)
    return {'anyOf': [{'enum': [*rules, *names]},
                      {'type': 'array', 'items': {'enum': names}, 'minItems': 1, 'uniqueItems': True}],
            'x-taskweave-widget': 'hands',
            'description': 'Which hands: a hand name or a list of names; "auto": the hands the target defines (or, '
                           'for a held object, the hands holding it); "any_one": one free hand that can take the '
                           'target (move_gripper: an IK-feasible hand with the least joint motion, named in the '
                           'result; grip: the one hand at an object point); "all": every hand.'}
