"""Robot and cell data for the launch files, found the way taskweave finds it (ConfigStore robot file search path):
the cell pack's ``robots/<robot>/<file>`` first, then the robot packs (env TASKWEAVE_ROBOT_PACKS, system.yaml
``robot_packs``, the installed ``taskweave_robots``). Nothing here knows a path of a particular robot or cell.
"""

from __future__ import annotations

from pathlib import Path

import yaml

from taskweave_core.config import ConfigStore, overlay


def open_store(config_dir: str | Path) -> ConfigStore:
    return ConfigStore(config_dir)


def read_yaml(path: Path | str) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding='utf-8')) or {}


def base_profile(store: ConfigStore, robot: str) -> dict:
    """The ``base`` section of the robot's profile (cell override laid over the robot pack's file)."""
    return store.robot(robot).base


def navigation(store: ConfigStore, robot: str) -> dict:
    """navigation.yaml of the robot; ``{}`` when neither the cell pack nor a robot pack has one."""
    return store.navigation(robot)


def moveit_folder(store: ConfigStore, robot: str) -> Path | None:
    """The robot's ``moveit/`` folder (cell pack copy, else robot pack) if it holds a robot.urdf, else None."""
    folder = store.robot_file(robot, 'moveit')
    return folder if (folder / 'robot.urdf').is_file() else None


def layered_file(store: ConfigStore, robot: str, name: str, default: Path, explicit: str | Path = '') -> dict:
    """A YAML file with a packaged default that a robot pack or the cell pack may replace, whole or as a keyed override.

    Order: ``explicit`` (a launch argument), else ``robots/<robot>/<name>`` of the cell pack, else a robot pack's
    ``<name>``, else ``default``. A file that starts with ``~override: keys`` is laid over the next lower layer (the
    robot pack's file, else the default) with taskweave's overlay rules; any other file replaces it."""
    lower = next((p.path / name for p in store.robot_packs(robot) if (p.path / name).is_file()), default)
    chosen = Path(explicit) if str(explicit) else store.robot_file(robot, name)
    if not chosen.is_file():
        chosen = lower
    data = read_yaml(chosen)
    if overlay.is_keyed(data):
        data = overlay.overlay(read_yaml(lower), data, name)
    return data


def source_of(store: ConfigStore, robot: str, name: str, default: Path, explicit: str | Path = '') -> str:
    """Where :func:`layered_file` reads from (for the launch log)."""
    if str(explicit):
        return str(explicit)
    found = store.robot_file(robot, name)
    return str(found) if found.is_file() else str(default)
