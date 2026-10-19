"""Plain descriptions of launch nodes, to compare launch descriptions without running them: package, executable,
name, respawn, remappings and parameters with every substitution performed."""

from __future__ import annotations

import yaml
from launch.substitution import Substitution
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def plain(text: str):
    """launch_ros writes a parameter string as YAML text (a quoted scalar for odd ones): read the value back."""
    try:
        return yaml.safe_load(text)
    except yaml.YAMLError:
        return text


def perform(value, context):
    """A substitution, or a sequence of them (one string), as the string it stands for; containers recursively."""
    if isinstance(value, Substitution):
        return value.perform(context)
    if isinstance(value, ParameterValue):
        return perform(value.value, context)
    if isinstance(value, ParameterFile):
        return {'file': perform(value.param_file, context)}
    if isinstance(value, dict):
        return {perform(k, context): perform(v, context) for k, v in value.items()}
    if isinstance(value, list | tuple) and value and all(isinstance(v, Substitution) for v in value):
        return plain(''.join(v.perform(context) for v in value))
    if isinstance(value, list | tuple):
        return [perform(v, context) for v in value]
    return value


def describe(node, context) -> dict:
    parameters = node._Node__parameters or ()
    return {
        'package': perform(node._Node__package, context),
        'executable': perform(node._Node__node_executable, context),
        'name': perform(node._Node__node_name, context),
        'respawn': bool(node._ExecuteLocal__respawn) if isinstance(node._ExecuteLocal__respawn, bool) else
        perform(node._ExecuteLocal__respawn, context),
        'respawn_delay': node._ExecuteLocal__respawn_delay,
        'remappings': [(perform(a, context), perform(b, context)) for a, b in node._Node__remappings or ()],
        'parameters': [perform(p, context) for p in parameters],
    }
