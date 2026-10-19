"""Robot description: what a robot is and can do, independent of how its hardware is reached.

``robots/<model>/embodiment.yaml`` holds no topics or joint names (that is the profile's job), so a simulated and
a real robot of the same model share it. taskweave owns the vocabulary below, so descriptions of different robots
use the same words and implementation requirements match exactly.

Canonical TCP: every tool's tool-centre point is the grasp centre between the fingertips (or the suction-cup
centre), with **z = approach direction** (pointing out of the tool, towards the object) and **y = closing
direction** of the fingers. The robot profile maps each robot's flange to this frame; taught tool poses use it.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Any

CAPABILITIES = {
    'base.holonomic': 'base can translate in any direction and rotate',
    'base.differential': 'base drives forward/backward and rotates',
    'base.fixed': 'base does not move',
    'torso.lift': 'torso height can change',
    'torso.pitch': 'torso can lean forward',
    'head.pan_tilt': 'head camera can be pointed',
    'arms.single': 'one manipulator arm',
    'arms.bimanual': 'two manipulator arms that can work together',
    'gripper.parallel': 'parallel two-finger gripper',
    'gripper.suction': 'suction gripper',
    'hand.dexterous': 'multi-finger hand',
    'camera.head.rgb': 'colour camera on the head or body',
    'camera.head.rgbd': 'colour and depth camera on the head or body',
    'camera.wrist.rgb': 'colour camera on a wrist or tool',
    'camera.wrist.rgbd': 'colour and depth camera on a wrist or tool',
    'lidar.2d': 'planar laser scanner',
    'lidar.3d': '3D lidar',
}

TOOL_TYPES = {'parallel_gripper', 'suction', 'dexterous_hand'}
BASE_CAPABILITY = {'holonomic': 'base.holonomic', 'differential': 'base.differential', 'fixed': 'base.fixed'}


def _number(value: Any, unit: str = '', digits: int = 2) -> str:
    return 'unknown' if value is None else f'{value:.{digits}f}{unit}'.rstrip()


def match(requires: Mapping[str, Any] | None, embodiment: Mapping[str, Any]) -> list[str]:
    """Reasons why an implementation's ``requires`` does not fit the robot; empty means it fits."""
    requires = requires or {}
    reasons = []
    missing = sorted(set(requires.get('capabilities', [])) - set(embodiment.get('capabilities', [])))
    if missing:
        reasons.append('missing capabilities ' + ', '.join(missing))
    space = requires.get('action_space')
    if space and space not in embodiment.get('action_spaces', {}):
        reasons.append(f'no action space {space!r}')
    cameras = sorted(set(requires.get('cameras', [])) - set(embodiment.get('cameras', {})))
    if cameras:
        reasons.append('missing cameras ' + ', '.join(cameras))
    allowed = requires.get('embodiments')
    if allowed and embodiment['name'] not in allowed:
        reasons.append(f'trained for {", ".join(allowed)}, not {embodiment["name"]}')
    return reasons


def check_profile(embodiment: Mapping[str, Any], profile: Mapping[str, Any]) -> list[str]:
    """Contradictions between a robot's embodiment and its profile."""
    problems = []
    base_type = profile.get('base', {}).get('type')
    expected = BASE_CAPABILITY.get(base_type)
    caps = set(embodiment.get('capabilities', []))
    if expected and expected not in caps:
        problems.append(f'profile base type {base_type!r} but embodiment lacks capability {expected!r}')
    for other in set(BASE_CAPABILITY.values()) - {expected}:
        if other in caps:
            problems.append(f'embodiment says {other!r} but the profile base type is {base_type!r}')
    for role in embodiment.get('tools', {}):
        if role not in profile.get('grippers', {}) and role not in profile.get('tools', {}):
            problems.append(f'tool {role!r} has no hardware mapping in the profile (grippers.{role})')
    return problems


ARM_KEYS = ('elbow', 'swivel')    # what taskweave reads under profile arms.<tool>


def profile_warnings(profile: Mapping[str, Any]) -> list[str]:
    """Things in a profile that load but cost a function: an IK arm without an elbow description (profile
    arms.<tool>.elbow, or the deprecated ik.choose.elbow_links) has no elbow control (an old whole-file copy of a
    robot pack profile, see taskweave config slim, typically lacks it); a key under arms.<tool> that nothing reads (the
    schema lets it pass: a typo like ``swivle`` would otherwise be silent)."""
    ik = profile.get('ik') or {}
    arms = profile.get('arms') or {}
    warnings = [f'arms.{tool}.{key}: not read by taskweave (ignored); the keys of an arm are {", ".join(ARM_KEYS)}'
                for tool, arm in arms.items() for key in arm if key not in ARM_KEYS]
    if (ik.get('choose') or {}).get('elbow_links'):
        return warnings
    return warnings + [
        f'ik arm {tool!r} has no elbow description (profile arms.{tool}.elbow): no elbow control, and its '
        'straight lines and elbow swings are unavailable; an old copy of the robot pack profile? '
        '(taskweave config slim)' for tool in ik.get('groups') or {} if not (arms.get(tool) or {}).get('elbow')]


def check_grasp(point: Mapping[str, Any], embodiment: Mapping[str, Any]) -> list[str]:
    """Check an object point (tool_type, grip_width_m, tool poses) against the robot's tools."""
    problems = []
    tools = embodiment.get('tools', {})
    roles = [key for key in point if key.endswith('_tool')]
    for key in roles:
        role = key[:-len('_tool')]
        tool = tools.get(role)
        if tool is None:
            problems.append(f'needs tool {role!r}, robot has {", ".join(tools) or "none"}')
            continue
        wanted = point.get('tool_type')
        if wanted and tool.get('type') != wanted:
            problems.append(f'needs a {wanted} on {role!r}, robot has {tool.get("type")}')
        width, stroke = point.get('grip_width_m'), tool.get('stroke_m')
        if width is not None and stroke is not None and width > stroke:
            problems.append(f'grip width {width:g} m exceeds the {role!r} stroke {stroke:g} m')
    return problems


def robot_card(embodiment: Mapping[str, Any], skills: Iterable[Mapping[str, Any]],
               points: Mapping[str, Iterable[str]] | None = None) -> str:
    """Text description of the robot and the skills it can run, for a planner such as a VLM.

    ``skills``: dicts with name, description, args_schema, needs, achieves, implementation.
    ``points``: optional named points by kind, e.g. {'locations': [...], 'stances': [...], 'object points': [...]}.
    """
    e = embodiment
    lines = [f'Robot: {e["name"]}', e.get('summary', '').strip(), '',
             'Capabilities: ' + ', '.join(e.get('capabilities', []))]
    base = e.get('base')
    if base:
        speed = base.get('max_speed', {})
        footprint = base.get('footprint')
        size = f'{footprint[0]:.2f} x {footprint[1]:.2f} m' if footprint else 'unknown'
        lines.append(f'Base: footprint {size}, max {_number(speed.get("linear"), " m/s")}, '
                     f'{_number(speed.get("angular"), " rad/s")}')
    torso = e.get('torso')
    if torso:
        lift = torso.get('lift_range_m')
        lines.append('Torso: lift ' + (f'{lift[0]:.2f}-{lift[1]:.2f} m' if lift else 'range unknown'))
    for role, arm in e.get('arms', {}).items():
        lines.append(f'Arm {role}: {arm.get("dof", "?")}-DoF, reach {_number(arm.get("reach_m"), " m")}, '
                     f'payload {_number(arm.get("payload_kg"), " kg", 1)}, tool {arm.get("tool", "-")}')
    for role, tool in e.get('tools', {}).items():
        lines.append(f'Tool {role}: {tool.get("type")}, stroke {_number(tool.get("stroke_m"), " m", 3)}, '
                     f'finger length {_number(tool.get("finger_length_m"), " m", 3)}')
    for role, camera in e.get('cameras', {}).items():
        span = camera.get('range_m')
        where = f', on {camera["mounted_on"]}' if camera.get('mounted_on') else ''
        lines.append(f'Camera {role}: {camera.get("type")}, {camera.get("hfov_deg", "?")} deg horizontal'
                     + (f', {span[0]:g}-{span[1]:g} m' if span else '') + where)
    for name, space in e.get('action_spaces', {}).items():
        lines.append(f'Action space {name}: {space.get("dims")} dims at {space.get("rate_hz")} Hz')
    if points:
        lines += ['', 'Known points:']
        lines += [f'- {kind}: {", ".join(sorted(names)) or "none"}' for kind, names in points.items()]
    lines += ['', 'Skills (call with skill name and args):']
    for skill in sorted(skills, key=lambda s: s['name']):
        schema = skill.get('args_schema') or {}
        required = set(schema.get('required', []))
        args = ', '.join(
            f'{key}{"" if key in required else "?"}: {spec.get("description") or spec.get("type", "any")}'
            for key, spec in schema.get('properties', {}).items())
        lines.append(f'- {skill["name"]}({args}) [{skill.get("implementation", "")}]: {skill.get("description", "")}')
        if skill.get('needs'):
            lines.append(f'    needs: {"; ".join(skill["needs"])}')
        if skill.get('achieves'):
            lines.append(f'    achieves: {"; ".join(skill["achieves"])}')
    return '\n'.join(lines).replace('\n\n\n', '\n\n')
