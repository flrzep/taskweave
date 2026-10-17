"""Cell packs and robot packs: where config files live.

A cell pack is one cell's config (the ConfigStore root): cell/, objects/, missions/, skills/, sim/, tests/,
system.yaml, operator.yaml and robots/<robot>/ with the robot's cell-specific files (stances, held poses, object
points). A robot pack is one robot model, nothing about a cell: profile, embodiment, moveit/, navigation, sim, poses,
pose targets. Both have a pack.yaml {name, version, schema_version[, description]}; a robot pack's name is the robot.

A robot file is looked up in the cell pack's robots/<robot>/ first (cell-specific files, and overrides of robot pack
files: keyed, only the keys the cell changes laid over the pack's file, or a whole file; overlay.py), then in the
robot packs, in this order: env TASKWEAVE_ROBOT_PACKS
(os.pathsep-separated), system.yaml robot_packs (relative to the cell pack), the installed taskweave_robots share.
Each location is a robot pack (its pack.yaml names the robot) or a folder of robot packs (<location>/<robot>/).
The installed share is read-only: edits of its files are written to the cell pack as keyed overrides (a rebuild would
overwrite them there).
"""

from __future__ import annotations

import functools
import os
from dataclasses import dataclass
from pathlib import Path

import yaml
from jsonschema import Draft202012Validator

from . import schemas

CELL_SCHEMA = 2         # cell pack layout this taskweave reads (1: one config directory with whole robot folders)
ROBOT_SCHEMA = 1        # robot pack layout this taskweave reads
PACK_FILE = 'pack.yaml'
ROBOT_PACKS_ENV = 'TASKWEAVE_ROBOT_PACKS'
INSTALLED_PACKAGE = 'taskweave_robots'
ROBOT_PACK_FILES = ('profile.yaml', 'embodiment.yaml', 'moveit', 'navigation.yaml', 'sim.yaml', 'poses.yaml',
                    'pose_targets.yaml', 'skill_overrides.yaml')    # robot-intrinsic: what the robot pack holds
# (moveit/ is one unit)
CELL_ROBOT_FILES = ('stances.yaml', 'held_poses.yaml', 'object_points.yaml')  # robot data about this cell


def _error(path, message: str, key: str = ''):
    from .store import ConfigError  # store imports this module

    return ConfigError(path, message, key)


_packs: dict[Path, tuple[int, dict]] = {}   # pack.yaml -> (mtime, data): robot files are resolved on every load


def read_pack(folder: Path) -> dict | None:
    """<folder>/pack.yaml, validated; None if the folder has none."""
    path = Path(folder) / PACK_FILE
    try:
        stamp = path.stat().st_mtime_ns
    except OSError:
        return None
    if path in _packs and _packs[path][0] == stamp:
        return _packs[path][1]
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except yaml.YAMLError as error:
        raise _error(path, f'invalid YAML: {error}') from error
    errors = sorted(Draft202012Validator(schemas.PACK).iter_errors(data), key=lambda e: list(e.absolute_path))
    if errors:
        raise _error(path, errors[0].message, '.'.join(str(p) for p in errors[0].absolute_path))
    _packs[path] = (stamp, data)
    return data


def cell_schema(folder: Path) -> int:
    """The cell pack's schema_version; 1 without a pack.yaml (a config directory from before packs)."""
    pack = read_pack(folder)
    return int(pack['schema_version']) if pack else 1


def check_cell_schema(folder: Path) -> None:
    version = cell_schema(folder)
    if version < CELL_SCHEMA:
        raise _error(Path(folder) / PACK_FILE, f'cell pack schema {version}, this taskweave needs {CELL_SCHEMA}: '
                                               f'run taskweave config migrate {folder}')
    if version > CELL_SCHEMA:
        raise _error(Path(folder) / PACK_FILE, f'cell pack schema {version}, this taskweave reads schema '
                                               f'{CELL_SCHEMA}: update taskweave')


@functools.lru_cache(maxsize=1)
def installed_location() -> Path | None:
    """share/taskweave_robots of the installed package (ament index); None without ROS or the package."""
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(INSTALLED_PACKAGE))
    except Exception:  # no ament_index_python, package not installed
        return None


@dataclass(frozen=True)
class Location:
    """A place robot packs are found: a robot pack itself or a folder of them."""

    path: Path
    origin: str         # env, system.yaml, installed
    writable: bool


def locations(cell: Path, configured: list[str] | None) -> list[Location]:
    """Robot pack locations in search order: env, system.yaml robot_packs (``configured``), installed share."""
    found = [Location(Path(p).expanduser().resolve(), 'env', True)
             for p in os.environ.get(ROBOT_PACKS_ENV, '').split(os.pathsep) if p.strip()]
    found += [Location((Path(cell) / p).expanduser().resolve(), 'system.yaml', True) for p in configured or []]
    installed = installed_location()
    if installed is not None:
        found.append(Location(installed, 'installed', False))
    return found


@dataclass(frozen=True)
class RobotPack:
    path: Path          # the robot's folder: <path>/<file>
    name: str
    location: Location

    @property
    def label(self) -> str:
        return f'robot pack {self.name} ({self.location.origin}: {self.path})'


def robot_pack(location: Location, robot: str) -> RobotPack | None:
    """The robot's pack at a location, schema checked; None if the location has none for the robot."""
    folder, pack = location.path, read_pack(location.path)
    if pack is None:
        folder = location.path / robot
        if not folder.is_dir():
            return None
        pack = read_pack(folder)
        if pack is None:
            raise _error(folder / PACK_FILE, f'robot pack {robot} has no {PACK_FILE}')
    elif pack['name'] != robot:
        return None
    if pack['name'] != robot:
        raise _error(folder / PACK_FILE, f'name is {pack["name"]!r}, expected {robot!r}', 'name')
    version = int(pack['schema_version'])
    if version != ROBOT_SCHEMA:
        hint = f'run taskweave config migrate {folder}' if version < ROBOT_SCHEMA else 'update taskweave'
        raise _error(folder / PACK_FILE, f'robot pack schema {version}, this taskweave reads schema {ROBOT_SCHEMA}: '
                                         f'{hint}')
    return RobotPack(folder, robot, location)


def robot_packs(cell: Path, configured: list[str] | None, robot: str) -> list[RobotPack]:
    """Every pack of the robot, in search order."""
    return [p for p in (robot_pack(loc, robot) for loc in locations(cell, configured)) if p is not None]
