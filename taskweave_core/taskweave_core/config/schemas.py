"""JSON Schemas (draft 2020-12) for every taskweave config file.

Kept as Python data so they are versioned with the code that reads them. Application-specific content
(e.g. a cell layout) is not described here; taskweave only validates the files it interprets itself.
"""

from ..embodiment import CAPABILITIES, TOOL_TYPES

FINITE = {'type': 'number'}
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
    'required': ['instruction', 'skill'],
    'properties': {
        'id': NAME,
        'instruction': {'type': 'string', 'minLength': 1},
        'skill': NAME,
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
                               'action': {'type': 'string'}},
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
                               'rpy': {'type': 'array', 'items': FINITE, 'minItems': 3, 'maxItems': 3}},
                'additionalProperties': False,
            },
        },
        'arms': {'type': 'object'},
        'torso': {'type': 'object'},
        'cameras': {'type': 'object'},
        'teleop': {'type': 'object'},
    },
}

# robots/<model>/poses.yaml: named joint-space poses, joint name -> value (rad or m)
JOINT_POSES = {
    'type': 'object',
    'propertyNames': NAME,
    'additionalProperties': {'type': 'object', 'minProperties': 1, 'additionalProperties': FINITE},
}

POINTS2D = {'type': 'object', 'additionalProperties': POSE2D, 'propertyNames': NAME}

# robots/<model>/navigation.yaml: how much room a path planner keeps around the robot while it drives. The envelope
# covers the robot in its driving pose (arms included), as distances from the base frame; padding and inflation as
# in Nav2 (padding grows the envelope, inflation adds a cost gradient beyond it).
POSITIVE = {'type': 'number', 'exclusiveMinimum': 0}
NAVIGATION = {
    'type': 'object',
    'required': ['envelope'],
    'properties': {
        'envelope': {
            'type': 'object',
            'required': ['front', 'back', 'left', 'right'],
            'properties': {side: POSITIVE for side in ('front', 'back', 'left', 'right')},
            'additionalProperties': False,
        },
        'padding': {'type': 'number', 'minimum': 0},
        'inflation_radius': POSITIVE,
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
                       'yaw': FINITE},
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
        'points': {
            'type': 'object',
            'propertyNames': NAME,
            'additionalProperties': {
                'type': 'object',
                'properties': {
                    'approach': {'type': 'object'},
                    'tool_type': {'enum': sorted(TOOL_TYPES)},
                    'grip_width_m': {'type': 'number', 'exclusiveMinimum': 0},
                },
                'additionalProperties': POSE3D,
            },
        },
    },
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
                'actions': {
                    'type': 'array',
                    'items': {
                        'type': 'object',
                        'required': ['button', 'skill'],
                        'properties': {'button': {'type': 'integer', 'minimum': 0}, 'skill': NAME,
                                       'args': {'type': 'object'}},
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
