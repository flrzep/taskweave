"""JSON Schemas (draft 2020-12) for every taskweave config file.

Kept as Python data so they are versioned with the code that reads them. Application-specific content
(e.g. a cell layout) is not described here; taskweave only validates the files it interprets itself.
"""

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
    'required': ['goal', 'robot', 'steps'],
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
        'arms': {'type': 'object'},
        'torso': {'type': 'object'},
        'grippers': {'type': 'object'},
        'cameras': {'type': 'object'},
        'teleop': {'type': 'object'},
    },
}

POINTS2D = {'type': 'object', 'additionalProperties': POSE2D, 'propertyNames': NAME}

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
                'properties': {'approach': {'type': 'object'}},
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
            },
            'additionalProperties': False,
        },
    },
    'additionalProperties': False,
}

SKILL_PARAMS = {'type': 'object'}
