"""Cell pack migrations: ``taskweave config migrate CELL_DIR [--robot-packs DIR] [--name NAME] [--dry-run]``.

Each migration takes a cell pack from one schema_version to the next; they run in order up to packs.CELL_SCHEMA,
print what they do (with --dry-run only that) and are idempotent: a second run finds nothing left to do.

1 -> 2: split one config directory into a cell pack and robot packs. Per robot-intrinsic file in robots/<robot>/
(packs.ROBOT_PACK_FILES; moveit/ as one unit), compared with the robot pack the store would read after the move:
  same content       removed from the cell pack
  different content  kept in the cell pack, as a cell override of the robot pack's file (nothing changes)
  no pack has it     moved to <--robot-packs>/<robot>/ (pack.yaml written, the folder added to system.yaml
                     robot_packs if it is not on the search path), or kept without --robot-packs
Cell-specific files (stances, held poses, object points) stay. Last, the cell pack's pack.yaml is written.
"""

from __future__ import annotations

import filecmp
import os
import re
import shutil
from pathlib import Path

import yaml

from . import packs

ROBOT_PACK_VERSION = '0.1.0'


def _same(a: Path, b: Path) -> bool:
    """Same file content, or the same files with the same content below two folders."""
    if a.is_dir() != b.is_dir():
        return False
    if not a.is_dir():
        return filecmp.cmp(a, b, shallow=False)
    files = sorted(p.relative_to(a) for p in a.rglob('*') if p.is_file())
    return files == sorted(p.relative_to(b) for p in b.rglob('*') if p.is_file()) and all(
        filecmp.cmp(a / f, b / f, shallow=False) for f in files)


def _system_robot_packs(cell: Path) -> list[str]:
    path = cell / 'system.yaml'
    return list((yaml.safe_load(path.read_text(encoding='utf-8')) or {}).get('robot_packs', [])) if path.is_file() else []


def _add_robot_packs(cell: Path, entry: str) -> None:
    """Append ``entry`` to system.yaml robot_packs, comments kept."""
    from ruamel.yaml import YAML
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    path, handler = cell / 'system.yaml', YAML()
    document = (handler.load(path.read_text(encoding='utf-8')) if path.is_file() else None) or CommentedMap()
    if not isinstance(document.get('robot_packs'), list):
        document['robot_packs'] = CommentedSeq()
        document.yaml_set_comment_before_after_key('robot_packs', 'robot pack locations, relative to this cell pack '
                                                                  '(taskweave config migrate)')
    document['robot_packs'].append(entry)
    with path.open('w', encoding='utf-8') as stream:
        handler.dump(document, stream)


def split_robot_packs(cell: Path, robot_packs: Path | None, dry_run: bool, say) -> None:
    """Migration 1 -> 2 without the pack.yaml (see the module docstring)."""
    do = 'would ' if dry_run else ''
    configured = _system_robot_packs(cell)
    target = robot_packs.expanduser().resolve() if robot_packs else None
    on_path = target is not None and any(loc.path == target for loc in packs.locations(cell, configured))
    entry = os.path.relpath(target, cell).replace(os.sep, '/') if target is not None else None
    moved = False
    for folder in sorted(p for p in (cell / 'robots').glob('*') if p.is_dir()):
        robot = folder.name
        search = packs.locations(cell, configured + ([entry] if target is not None and not on_path else []))
        robot_packs_found = [p for p in (packs.robot_pack(loc, robot) for loc in search) if p is not None]
        gone = set()    # entries removed or moved out of the folder
        for name in packs.ROBOT_PACK_FILES:
            here = folder / name
            if not here.exists():
                continue
            label = f'robots/{robot}/{name}{"/" if here.is_dir() else ""}'
            base = next((p.path / name for p in robot_packs_found if (p.path / name).exists()), None)
            if base is not None and _same(here, base):
                say(f'{do}remove  {label}  (same as {base})')
                gone.add(name)
                if not dry_run:
                    shutil.rmtree(here) if here.is_dir() else here.unlink()
            elif base is not None:
                say(f'keep    {label}  as a cell override: differs from {base}')
            elif target is not None:
                pack = target / robot
                say(f'{do}move    {label}  -> {pack / name}')
                gone.add(name)
                moved = True
                if not dry_run:
                    if not (pack / packs.PACK_FILE).is_file():
                        pack.mkdir(parents=True, exist_ok=True)
                        (pack / packs.PACK_FILE).write_text(yaml.safe_dump(
                            {'name': robot, 'version': ROBOT_PACK_VERSION, 'schema_version': packs.ROBOT_SCHEMA,
                             'description': f'{robot} robot files, moved out of cell pack {cell.name} by taskweave '
                                            'config migrate'}, sort_keys=False, width=1000), encoding='utf-8')
                    shutil.move(str(here), str(pack / name))
            else:
                say(f'keep    {label}  no robot pack has it (move it out with --robot-packs DIR)')
        if gone and not {p.name for p in folder.iterdir()} - gone:
            say(f'{do}remove  robots/{robot}/  (empty)')
            if not dry_run:
                folder.rmdir()
    if moved and not on_path:
        say(f'{do}add     {entry}  to system.yaml robot_packs')
        if not dry_run:
            _add_robot_packs(cell, entry)


def write_cell_pack(cell: Path, name: str, version: int, dry_run: bool, say) -> None:
    pack = packs.read_pack(cell) or {}
    name = re.sub(r'[^A-Za-z0-9_\-]', '_', name)
    name = name if re.match(r'[A-Za-z_]', name) else f'cell_{name}'
    text = ('# cell pack: everything about this cell; the robots come from robot packs (taskweave config migrate)\n'
            + yaml.safe_dump({'name': pack.get('name', name), 'version': pack.get('version', 1),
                              'schema_version': version}, sort_keys=False))
    say(f'{"would " if dry_run else ""}write   pack.yaml  (schema_version {version})')
    if not dry_run:
        (cell / packs.PACK_FILE).write_text(text, encoding='utf-8')


MIGRATIONS = {1: split_robot_packs}  # from schema_version -> function(cell, robot_packs, dry_run, say)


def migrate(cell: str | Path, robot_packs: str | Path | None = None, name: str | None = None, dry_run: bool = False,
            say=print) -> int:
    """Run the migrations from the cell pack's schema_version up to packs.CELL_SCHEMA. Returns an exit code."""
    cell = Path(cell).expanduser().resolve()
    if not cell.is_dir():
        say(f'{cell}: no such directory')
        return 2
    version = packs.cell_schema(cell)
    if version > packs.CELL_SCHEMA:
        say(f'{cell}: cell pack schema {version} is newer than this taskweave ({packs.CELL_SCHEMA}): update taskweave')
        return 2
    if version == packs.CELL_SCHEMA:
        say(f'{cell}: cell pack schema {version}, nothing to do')
        return 0
    say(f'{cell}: cell pack schema {version} -> {packs.CELL_SCHEMA}{" (dry run: nothing is changed)" if dry_run else ""}')
    for step in range(version, packs.CELL_SCHEMA):
        MIGRATIONS[step](cell, Path(robot_packs) if robot_packs else None, dry_run, say)
        write_cell_pack(cell, name or cell.name, step + 1, dry_run, say)
    return 0
