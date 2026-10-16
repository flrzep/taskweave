"""JSON Schemas (draft 2020-12) for every taskweave config file.

Kept as Python data so they are versioned with the code that reads them. Application-specific content
(e.g. a cell layout) is not described here; taskweave only validates the files it interprets itself.
"""

from ..embodiment import CAPABILITIES, TOOL_TYPES

FINITE = {'type': 'number'}
ELBOW_SIDES = ['down', 'up', 'out', 'in', 'back', 'forward']  # preferred elbow side, in the robot's frame (ik.py)
NAME = {'type': 'string', 'pattern': r'^[A-Za-z_][A-Za-z0-9_\-]*$'}

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
                           'point': {'enum': ['base_target', 'joint_pose', 'object_point', 'instance_point',
                                              'instance_contact', 'instance']}}}},
        # facts each cycle starts from, filled into the steps as {name} (MissionFilePlanner): fullest = the placed object
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
                           'linear': {'type': 'array', 'items': {'type': 'string'}}},  # prismatic joints (m)
        },
        'grippers': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'properties': {'joint': {'type': 'string'}, 'open': FINITE, 'closed': FINITE,
                               'action': {'type': 'string'},
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
                # the joint the pad's elbow jog turns before IK puts the TCP back ({tool} = tool name; default:
                # the third joint the joints pattern matches, the upper-arm roll of a 7-joint arm)
                'swivel_joint': {'type': 'string'},
                # tool -> the tool whose joints it mirrors with the same values (names with the prefix swapped)
                'mirror': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {'type': 'string'}},
                # pick the best of several solutions (taskweave_skills/ik.py): joint-limit room, elbow side, move size
                'choose': {
                    'type': 'object',
                    'properties': {
                        'elbow': {'oneOf': [{'enum': ELBOW_SIDES},
                                            {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3}]},
                        'elbow_links': {'type': 'array', 'items': {'type': 'string'}, 'minItems': 3, 'maxItems': 3},
                        'limit_margin_deg': {'type': 'number', 'minimum': 0},
                        'weights': {'type': 'object',
                                    'properties': {k: {'type': 'number', 'minimum': 0}
                                                   for k in ('limits', 'elbow', 'motion', 'symmetry')},
                                    'additionalProperties': False},
                        'fk_service': {'type': 'string'},
                        'starts': {'type': 'integer', 'minimum': 0},
                        'start_timeout_s': {'type': 'number', 'exclusiveMinimum': 0},
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
                   for k in ('direct_step_rad', 'escape_max_rad', 'escape_speed')},
            },
            'additionalProperties': False,
        },
        'arms': {'type': 'object'},
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
        # a placed object of this type is a place the next one comes to (e.g. a hanger waiting on a conveyor): taking
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
            'grip_m': {'type': 'object', 'propertyNames': NAME, 'additionalProperties': {'type': 'number', 'minimum': 0}},
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
