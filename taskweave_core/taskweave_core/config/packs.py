"""Cell packs and robot packs: where config files live.

A cell pack is one cell's config (the ConfigStore root): cell/, objects/, missions/, skills/, sim/, tests/,
system.yaml, operator.yaml and robots/<robot>/ with the robot's cell-specific files (stances, held poses, object
points). A robot pack is one robot model, nothing about a cell: profile, embodiment, moveit/, navigation, sim, poses,
pose targets. Both have a pack.yaml {name, version, schema_version[, description][, requires: {taskweave: <spec>}]};
a robot pack's name is the robot.

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
from packaging.specifiers import InvalidSpecifier, SpecifierSet
from packaging.version import InvalidVersion, Version

from . import schemas

CELL_SCHEMA = 3         # cell pack layout this taskweave reads (1: one config directory with whole robot folders,
                        # 2: robot packs, old skills move_to_pose / go_to_point / gripper / old move_object)
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


_packs: dict[Path, tuple[tuple, dict]] = {}   # pack.yaml -> ((mtime, size), data)


def read_pack(folder: Path) -> dict | None:
    """<folder>/pack.yaml, validated; None if the folder has none."""
    path = Path(folder) / PACK_FILE
    try:
        info = path.stat()
        stamp = (info.st_mtime_ns, info.st_size)    # not mtime alone: a coarse clock misses a quick rewrite
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


def taskweave_version() -> str:
    """The version of the taskweave_core that is running: the package.xml of its source tree, else the metadata of the
    installed copy this file belongs to (not just any installed taskweave_core: an older one may come first on the
    path), else any installed one."""
    import re

    here = Path(__file__).resolve()
    xml = here.parents[2] / 'package.xml'
    if xml.is_file():
        found = re.search(r'<version>([^<]+)</version>', xml.read_text(encoding='utf-8'))
        if found:
            return found.group(1).strip()
    try:
        from importlib.metadata import distributions, version

        for dist in distributions():
            if (dist.metadata['Name'] or '').replace('-', '_') == 'taskweave_core' and \
                    here.is_relative_to(Path(dist.locate_file('')).resolve()):
                return dist.version
        return version('taskweave_core')
    except Exception:  # metadata missing or unreadable
        return 'unknown'


def check_requires(pack: dict, path: Path, label: str) -> None:
    """Refuse a pack whose ``requires.taskweave`` does not admit this taskweave. No ``requires``: always fine."""
    spec = (pack.get('requires') or {}).get('taskweave')
    if spec is None:
        return
    key = 'requires.taskweave'
    try:
        specifier = SpecifierSet(spec)
    except InvalidSpecifier as error:
        raise _error(path, f'{label}: requires.taskweave {spec!r} is not a version specifier like ">=0.2,<0.3" '
                           f'({error})', key) from error
    have = taskweave_version()
    try:
        ok = specifier.contains(Version(have), prereleases=True)
    except InvalidVersion:
        return      # no usable own version (unknown): nothing to compare
    if not ok:
        mine = f' {pack["version"]}' if 'version' in pack else ''
        raise _error(path, f'{label}{mine} requires taskweave {spec}, this is taskweave {have}: use a matching '
                           f'taskweave release or a matching pack version', key)


def cell_schema(folder: Path) -> int:
    """The cell pack's schema_version; 1 without a pack.yaml (a config directory from before packs)."""
    pack = read_pack(folder)
    return int(pack['schema_version']) if pack else 1


def check_cell_schema(folder: Path) -> None:
    pack = read_pack(folder)
    if pack:
        check_requires(pack, Path(folder) / PACK_FILE, f'cell pack {pack["name"]}')
    elif not any((Path(folder) / part).exists() for part in ('cell', 'objects', 'missions', 'robots')):
        # no pack.yaml and none of an old cell's folders: not a cell at all (not one to migrate)
        raise _error(Path(folder), 'no cell pack here: pass the cell pack folder (the one with pack.yaml, e.g. config)')
    version = cell_schema(folder)
    if version < CELL_SCHEMA:
        command = 'migrate-skills' if version == 2 else 'migrate'   # 2 -> 3 is the skill migration (migrate_skills.py)
        raise _error(Path(folder) / PACK_FILE, f'cell pack schema {version}, this taskweave needs {CELL_SCHEMA}: '
                                               f'run taskweave config {command} {folder}')
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


def locations(cell: Path, configured: list[str] | None, planned: tuple[str, ...] = (),
              first: tuple[Path, ...] = ()) -> list[Location]:
    """Robot pack locations in search order: env, system.yaml robot_packs (``configured``), installed share. A
    configured location that does not exist is an error (a typo would silently hide a robot pack, and the files
    would come from an older one further down); ``planned`` system.yaml entries may not exist yet (migrate);
    ``first``: folders searched before all others (a dry run of an edit, ConfigStore.dry_run_pack)."""
    found = [Location(Path(path), 'dry run', True) for path in first]
    for entry in os.environ.get(ROBOT_PACKS_ENV, '').split(os.pathsep):
        if entry.strip():
            found.append(_location(Path(entry).expanduser().resolve(), 'env', f'env {ROBOT_PACKS_ENV}', cell))
    for entry in configured or []:
        path = (Path(cell) / entry).expanduser().resolve()
        found.append(_location(path, 'system.yaml', 'system.yaml robot_packs', cell, entry in planned))
    installed = installed_location()
    if installed is not None:
        found.append(Location(installed, 'installed', False))
    return found


def _location(path: Path, origin: str, where: str, cell: Path, may_not_exist: bool = False) -> Location:
    if not path.exists() and not may_not_exist:
        raise _error(path, f'robot pack location does not exist (configured in {where}; cell pack {cell})')
    return Location(path, origin, True)


def foreign(path: Path, cell: Path, search: list[Location], base: Path | None = None) -> str | None:
    """Why a tool that rewrites or deletes cell pack entries (config migrate, slim) must leave ``path`` alone, or None:
    through a symlink it leads outside the cell pack, into a robot pack location, or to the very file it is
    compared with (``base``). Changing it would change the robot pack, not the cell pack."""
    real, home = Path(path).resolve(), Path(cell).resolve()
    for location in search:
        if real.is_relative_to(location.path) and not home.is_relative_to(location.path):
            return f'resolves to {real}, inside robot pack location {location.path}'
    if not real.is_relative_to(home):
        return f'resolves to {real}, outside the cell pack'
    if base is not None and real == Path(base).resolve():
        return f'resolves to the robot pack file {real}'
    return None


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
    check_requires(pack, folder / PACK_FILE, f'robot pack {robot}')
    version = int(pack['schema_version'])
    if version != ROBOT_SCHEMA:
        hint = f'run taskweave config migrate {folder}' if version < ROBOT_SCHEMA else 'update taskweave'
        raise _error(folder / PACK_FILE, f'robot pack schema {version}, this taskweave reads schema {ROBOT_SCHEMA}: '
                                         f'{hint}')
    return RobotPack(folder, robot, location)


def robot_packs(cell: Path, configured: list[str] | None, robot: str, first: tuple[Path, ...] = ()) -> list[RobotPack]:
    """Every pack of the robot, in search order."""
    return [p for p in (robot_pack(loc, robot) for loc in locations(cell, configured, first=first)) if p is not None]


def list_packs(cell: Path, configured: list[str] | None, first: tuple[Path, ...] = ()) -> list[dict]:
    """Every robot pack at every location: ``{name, version, description, location, writable}`` (``location``: the
    pack's folder; a pack whose pack.yaml is invalid has ``error`` instead of version and description)."""
    found = []
    for location in locations(cell, configured, first=first):
        folders = [location.path] if (location.path / PACK_FILE).is_file() else sorted(
            p for p in location.path.glob('*') if (p / PACK_FILE).is_file()) if location.path.is_dir() else []
        for folder in folders:
            entry = {'name': folder.name, 'location': str(folder), 'writable': location.writable}
            try:
                pack = read_pack(folder) or {}
                entry.update(name=pack.get('name', folder.name), version=pack.get('version'),
                             description=pack.get('description', ''))
            except Exception as error:  # noqa: BLE001 - a broken pack is listed, not hidden
                entry['error'] = str(error)
            found.append(entry)
    return found
