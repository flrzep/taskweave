"""``taskweave config slim CELL_DIR [--dry-run]``: rewrite whole-file cell overrides of robot pack files as keyed
overrides (overlay.py), i.e. only the keys that differ from the robot pack's file. What a robot loads changes only for pack keys an old whole-file copy lacks: it gets the pack's.

Per robots/<robot>/<file> of overlay.MERGED_FILES that a robot pack also has:
  whole file, differs   rewritten as a keyed override; prints the keys it drops (equal to the pack's). Pack keys the
                        copy lacks are not overridden (the pack's apply): the copy predates them, it did not delete them
  whole file, same      removed (the pack's file says the same)
  keyed already         left alone
Idempotent: a second run finds only keyed overrides.
"""

from __future__ import annotations

from pathlib import Path

from . import overlay, packs
from .migrate import cannot_remove, removal_blocker
from .store import ConfigError, ConfigStore


def slim(cell: str | Path, dry_run: bool = False, say=print) -> int:
    """Slim every whole-file override in the cell pack. Returns an exit code (2: not a cell pack, 1: unreadable file or one that cannot be removed)."""
    cell = Path(cell).expanduser().resolve()
    if not cell.is_dir():
        say(f'{cell}: no such directory')
        return 2
    store, do, code, count = ConfigStore(cell), 'would ' if dry_run else '', 0, 0
    search = packs.locations(cell, store.system().get('robot_packs'))
    for folder in sorted(p for p in (cell / 'robots').glob('*') if p.is_dir()):
        robot = folder.name
        for file in overlay.MERGED_FILES:
            path, base = folder / file, store.override_base(robot, file)
            if base is None or not path.is_file():
                continue
            label = f'robots/{robot}/{file}'
            foreign = packs.foreign(path, cell, search, base)    # a link into a robot pack: never touched
            if foreign:
                say(f'skip    {label}  {foreign} (not touched)')
                continue
            try:
                data, pack = store._read_yaml(path), store._read_yaml(base) or {}
            except ConfigError as error:
                say(f'error   {label}: {error}')
                code = 1
                continue
            if overlay.is_keyed(data) or not isinstance(data, dict):
                say(f'keep    {label}  ({"keyed already" if overlay.is_keyed(data) else "not a mapping"})')
                continue
            changed = overlay.delta(pack, data, overlay.DEPTH.get(file), deleted=False)
            dropped = overlay.same_paths(pack, data, overlay.DEPTH.get(file))
            if not changed:
                blocker = removal_blocker(path)
                if blocker:
                    cannot_remove(blocker, label, say)
                    code = 1
                    continue
                say(f'{do}remove  {label}  (same as {base})')
                if not dry_run:
                    path.unlink()
            else:
                keys = overlay.key_paths(changed, overlay.DEPTH.get(file))
                say(f'{do}slim    {label}  keeps {len(keys)} key(s): {", ".join(keys)}')
                say(f'          drops {len(dropped)} equal to {base}: {", ".join(dropped) or "-"}')
                if not dry_run:
                    document = store.keyed_override(robot, file)[0]
                    from .store import _yaml_handler

                    with path.open('w', encoding='utf-8') as stream:
                        _yaml_handler().dump(document, stream)
            count += 1
    say(f'{count} whole-file override(s) {"to slim" if dry_run else "slimmed"}' if count else 'nothing to slim')
    return code
