"""Cell pack migrations: ``taskweave config migrate CELL_DIR [--robot-packs DIR] [--name NAME] [--dry-run]``.

Each migration takes a cell pack from one schema_version to the next; they run in order up to packs.CELL_SCHEMA,
print what they do (with --dry-run only that) and are idempotent: a second run finds nothing left to do.
``taskweave config migrate-skills CELL_DIR [--dry-run]`` runs only the 2 -> 3 one (migrate_skills.py).
``taskweave config migrate PACK_DIR`` where PACK_DIR is a robot pack (pack.yaml and profile.yaml) or a folder of robot
packs brings those packs to the robot pack schema 2 (poses.yaml -> targets.yaml, migrate_targets.py).

1 -> 2: split one config directory into a cell pack and robot packs. Per robot-intrinsic file in robots/<robot>/
(packs.ROBOT_PACK_FILES; moveit/ as one unit), compared with the robot pack the store would read after the move:
  same content       removed from the cell pack (a unit an interrupted run left half removed, i.e. a folder with
                     only some of the pack's files, all identical, is removed too)
  different content  kept in the cell pack, as a cell override of the robot pack's file (nothing changes)
  no pack has it     moved to <--robot-packs>/<robot>/ (pack.yaml written, the folder added to system.yaml
                     robot_packs if it is not on the search path), or kept without --robot-packs
Cell-specific files (stances, held poses, object points) stay. Before a unit is removed or moved, every entry in it
(and the folders that hold them) is checked to be removable; if one is not (root-owned, read-only), nothing of that
unit is touched, it is named, the other units go on, the exit code is 1 and pack.yaml is not written: fix the
permissions and run again. Last, the cell pack's pack.yaml is written.

2 -> 3: the new skill model: missions, operator.yaml, skills/*.yaml and joint-and-tool poses rewritten for move_joints,
move_gripper, grip, move_object (args ``to``) and set_posture (migrate_skills.py).

3 -> 4: targets: locations, stances, object points and contacts, poses and held poses become targets in four files, the
six motion skills become the skill ``move``; writable robot packs are migrated too
(migrate_targets.py, migrate_move.py).
"""

from __future__ import annotations

import filecmp
import os
import re
import shutil
from pathlib import Path

import yaml

from . import packs

ROBOT_PACK_VERSION = '0.2.0'


def _relation(a: Path, b: Path) -> str:
    """How ``a`` (the cell copy) relates to ``b`` (the pack copy): 'same' (same content, or the same files with the
    same content below two folders), 'subset' (a folder with only some of the pack files, each identical: a unit left
    half removed) or 'different'."""
    if a.is_dir() != b.is_dir():
        return 'different'
    if not a.is_dir():
        return 'same' if filecmp.cmp(a, b, shallow=False) else 'different'
    mine = {p.relative_to(a) for p in a.rglob('*') if p.is_file()}
    theirs = {p.relative_to(b) for p in b.rglob('*') if p.is_file()}
    if not mine <= theirs or not all(filecmp.cmp(a / f, b / f, shallow=False) for f in mine):
        return 'different'
    return 'same' if mine == theirs else 'subset'


def _entry_blocker(path: Path) -> str | None:
    """Why ``path`` cannot be removed (or moved away), None if it can: no write access to the folder that holds it,
    a sticky folder of someone else, a folder we cannot enter, a read-only file (Windows)."""
    parent = path.parent
    if not os.access(parent, os.W_OK | os.X_OK):
        return f'no write permission on {parent}'
    if os.name == 'nt':
        if not path.is_symlink() and not path.is_dir() and not os.access(path, os.W_OK):
            return 'file is read-only'
        return None
    info = parent.stat()
    if info.st_mode & 0o1000 and os.geteuid() not in (0, info.st_uid, path.lstat().st_uid):
        return f'{parent} is sticky and the entry belongs to another user'
    if path.is_dir() and not path.is_symlink() and not os.access(path, os.R_OK | os.W_OK | os.X_OK):
        return f'no read/write permission on {path}'
    return None


def removal_blocker(unit: Path) -> tuple[Path, str] | None:
    """The first entry of ``unit`` (a file, or a folder with all that is below it) that cannot be removed, with the
    reason; None if all can."""
    candidates = [unit]
    if unit.is_dir() and not unit.is_symlink():
        candidates += sorted(unit.rglob('*'))
    for path in candidates:
        reason = _entry_blocker(path)
        if reason:
            return path, reason
    return None


def cannot_remove(blocker: tuple[Path, str], label: str, say) -> None:
    say(f'cannot remove {blocker[0]}: {blocker[1]} (unit {label} left as it is; fix permissions or remove it as its '
        'owner)')


def _remove(path: Path) -> None:
    shutil.rmtree(path) if path.is_dir() and not path.is_symlink() else path.unlink()


def _system_robot_packs(cell: Path) -> list[str]:
    path = cell / 'system.yaml'
    if not path.is_file():
        return []
    return list((yaml.safe_load(path.read_text(encoding='utf-8')) or {}).get('robot_packs', []))


def _entry(target: Path, cell: Path) -> str:
    """The system.yaml robot_packs entry for ``target``: relative to the cell pack, absolute where there is no
    relative path (another Windows drive)."""
    try:
        return os.path.relpath(target, cell).replace(os.sep, '/')
    except ValueError:
        return target.as_posix()


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


def split_robot_packs(cell: Path, robot_packs: Path | None, dry_run: bool, say) -> list[str]:
    """Migration 1 -> 2 without the pack.yaml (see the module docstring). Returns the units (labels) that had to go
    but could not."""
    stuck: list[str] = []
    do = 'would ' if dry_run else ''
    configured = _system_robot_packs(cell)
    target = robot_packs.expanduser().resolve() if robot_packs else None
    on_path = target is not None and any(loc.path == target for loc in packs.locations(cell, configured))
    entry = _entry(target, cell) if target is not None else None
    moved = False
    for folder in sorted(p for p in (cell / 'robots').glob('*') if p.is_dir()):
        robot = folder.name
        adding = [entry] if target is not None and not on_path else []
        search = packs.locations(cell, configured + adding, tuple(adding))
        robot_packs_found = [p for p in (packs.robot_pack(loc, robot) for loc in search) if p is not None]
        gone = set()    # entries removed or moved out of the folder
        keep_folder = False
        for name in packs.ROBOT_PACK_FILES:
            here = folder / name
            if not here.exists():
                continue
            label = f'robots/{robot}/{name}{"/" if here.is_dir() else ""}'
            base = next((p.path / name for p in robot_packs_found if (p.path / name).exists()), None)
            foreign = packs.foreign(here, cell, search + ([packs.Location(target, 'env', True)] if target else []),
                                    base)
            # a symlink into a robot pack (or out of the cell pack): deleting or moving it would hit that
            if foreign:
                say(f'skip    {label}  {foreign} (not touched)')
                keep_folder = True
                continue
            relation = _relation(here, base) if base is not None else 'none'
            if relation in ('same', 'subset'):
                blocker = removal_blocker(here)
                if blocker:
                    cannot_remove(blocker, label, say)
                    stuck.append(label)
                    keep_folder = True
                    continue
                say(f'{do}remove  {label}  (same as {base}'
                    + (', partly removed already' if relation == 'subset' else '')
                    + ')')
                gone.add(name)
                if not dry_run:
                    _remove(here)
            elif base is not None:
                say(f'keep    {label}  as a cell override: differs from {base}')
            elif target is not None:
                pack = target / robot
                blocker = removal_blocker(here)
                if blocker:
                    cannot_remove(blocker, label, say)
                    stuck.append(label)
                    keep_folder = True
                    continue
                say(f'{do}move    {label}  -> {pack / name}')
                gone.add(name)
                moved = True
                if not dry_run:
                    if not (pack / packs.PACK_FILE).is_file():
                        pack.mkdir(parents=True, exist_ok=True)
                        (pack / packs.PACK_FILE).write_text(yaml.safe_dump(
                            {'name': robot, 'version': ROBOT_PACK_VERSION, 'schema_version': packs.ROBOT_SCHEMA_OLDEST,
                             'description': f'{robot} robot files, moved out of cell pack {cell.name} by taskweave '
                                            'config migrate'}, sort_keys=False, width=1000), encoding='utf-8')
                    shutil.move(str(here), str(pack / name))
            else:
                say(f'keep    {label}  no robot pack has it (move it out with --robot-packs DIR)')
        if gone and not keep_folder and not {p.name for p in folder.iterdir()} - gone:
            say(f'{do}remove  robots/{robot}/  (empty)')
            if not dry_run:
                reason = _entry_blocker(folder)
                if reason:
                    say(f'cannot remove {folder}: {reason} (empty folder left as it is)')
                else:
                    folder.rmdir()
    if moved and not on_path:
        say(f'{do}add     {entry}  to system.yaml robot_packs')
        if not dry_run:
            _add_robot_packs(cell, entry)
    return stuck


def write_cell_pack(cell: Path, name: str, version: int, dry_run: bool, say) -> None:
    pack = packs.read_pack(cell) or {}
    name = re.sub(r'[^A-Za-z0-9_\-]', '_', name)
    name = name if re.match(r'[A-Za-z_]', name) else f'cell_{name}'
    text = ('# cell pack: everything about this cell; the robots come from robot packs (taskweave config migrate)\n'
            + yaml.safe_dump({'name': pack.get('name', name), 'version': pack.get('version', 1),
                              'schema_version': version}, sort_keys=False))
    say(f'{"would " if dry_run else ""}write   pack.yaml  (schema_version {version})')
    if not dry_run:
        with (cell / packs.PACK_FILE).open('w', encoding='utf-8', newline='') as stream:    # LF on Windows too
            stream.write(text)


def target_model(cell: Path, robot_packs: Path | None, dry_run: bool, say) -> list[str]:
    from .migrate_targets import migrate_targets

    return migrate_targets(cell, robot_packs, dry_run, say)     # RuntimeError: a name it cannot map, nothing changed


def skill_model(cell: Path, robot_packs: Path | None, dry_run: bool, say) -> list[str]:
    from .migrate_skills import migrate_skills

    migrate_skills(cell, robot_packs, dry_run, say)
    return []   # no unit it could not touch: a file it cannot write stops it before anything changes (RuntimeError)


# from schema_version -> function(cell, robot_packs, dry_run, say) -> units it could not remove (pack.yaml waits)
MIGRATIONS = {1: split_robot_packs, 2: skill_model, 3: target_model}
LATEST = max(MIGRATIONS) + 1    # the schema the chain ends at (packs.CELL_SCHEMA)


def migrate(cell: str | Path, robot_packs: str | Path | None = None, name: str | None = None, dry_run: bool = False,
            say=print, latest: int = LATEST) -> int:
    """Run the migrations from the cell pack's schema_version up to packs.CELL_SCHEMA (``latest``: stop at this
    schema). Returns an exit code."""
    cell = Path(cell).expanduser().resolve()
    if not cell.is_dir():
        say(f'{cell}: no such directory')
        return 2
    from .migrate_targets import migrate_robot_packs_only, robot_pack_folders

    if robot_pack_folders(cell) and not any((cell / part).exists() for part in ('cell', 'objects', 'missions')):
        return migrate_robot_packs_only(cell, dry_run, say)     # a robot pack or a folder of them, not a cell pack
    version = packs.cell_schema(cell)
    if version > packs.CELL_SCHEMA:
        say(f'{cell}: cell pack schema {version} is newer than this taskweave ({packs.CELL_SCHEMA}): update taskweave')
        return 2
    if version >= latest:
        say(f'{cell}: cell pack schema {version}, nothing to do')
        return 0
    dry = ' (dry run: nothing is changed)' if dry_run else ''
    say(f'{cell}: cell pack schema {version} -> {latest}{dry}')
    pack = cell / packs.PACK_FILE
    blocked = pack.is_symlink() or (not os.access(pack, os.W_OK) if pack.exists() else not os.access(cell, os.W_OK))
    if blocked:     # pack.yaml is written last: without it a run would leave a half-migrated cell
        say(f'{pack.name} is not writable (or is a symbolic link): nothing was changed. Fix the permissions of '
            f'{pack} and run taskweave config migrate again.')
        if not dry_run:
            return 1
    for step in range(version, latest):
        try:
            stuck = MIGRATIONS[step](cell, Path(robot_packs) if robot_packs else None, dry_run, say)
        except RuntimeError as error:   # a file cannot be written: the migration checked before it touched anything
            say(f'{cell}: {error}')
            return 2
        except OSError as error:        # a write or remove failed anyway (a file changed meanwhile, a full disk)
            say(f'{cell}: migration step {step} stopped: {error}. The cell may be half-migrated: fix the cause and run '
                'taskweave config migrate again; it finishes what is left.')
            return 1
        if stuck:
            say(f'pack.yaml not written: still to remove: {", ".join(stuck)}. Fix the permissions (or remove them as '
                'their owner) and run taskweave config migrate again; it finishes what is left.')
            return 1
        try:
            write_cell_pack(cell, name or cell.name, step + 1, dry_run, say)
        except OSError as error:
            say(f'{cell}: pack.yaml could not be written: {error}. Fix the permissions and run taskweave config '
                'migrate again.')
            return 1
    return 0


def migrate_skills_only(cell: str | Path, dry_run: bool = False, say=print) -> int:
    """``taskweave config migrate-skills``: the 2 -> 3 migration alone (a cell pack of schema 1 needs ``migrate``)."""
    path = Path(cell).expanduser().resolve()
    version = packs.cell_schema(path) if path.is_dir() else 0
    if version == 1:
        say(f'{path}: cell pack schema 1: run taskweave config migrate {cell} first (it ends with this migration)')
        return 2
    return migrate(path, None, None, dry_run, say, latest=3)   # 2: the skill migration; 3: nothing to do
