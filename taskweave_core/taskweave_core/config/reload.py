"""Which config files changed since a process loaded them, and what each change needs: the logic behind the
``reload_config`` service of the task handler and the skill servers (no ROS in here).

A ``Reloader`` remembers a hash of every config file of its robot. ``reload(paths)`` re-reads the store, finds the
changed files and sorts them by the catalogue's ``reload`` values: a value the process applies (``applies``) makes the
file *reloaded*; a kind in ``restarts`` is *needs_restart*, with the reason. The process's ``apply`` callback gets the
files to take into effect and returns those it could not (path, what); it raises before changing anything if a new
value does not load. A file that needs a restart stays reported on every call until the process restarts.
"""

from __future__ import annotations

from collections.abc import Callable

from . import catalogue


class Reloader:
    def __init__(self, store, robot: str, applies: set[str], restarts: dict[str, str],
                 mine: Callable[[dict], bool] = lambda entry: True,
                 apply: Callable[[list[dict]], list[tuple[str, str]]] | None = None):
        """``applies``: reload values this process applies. ``restarts``: kind -> why a change needs a restart.
        ``mine``: whether a catalogue entry (of this robot) concerns this process at all."""
        self.store, self.robot, self.applies, self.restarts, self.mine, self.apply = (
            store, robot, applies, restarts, mine, apply)
        self.seen = self._snapshot()

    def _snapshot(self) -> dict[tuple[str, str], tuple[dict, str | None]]:
        found = {}
        for entry in self.store.files(hashes=False):
            if entry['robot'] in (None, self.robot) and self.mine(entry):
                found[entry['pack'], entry['path']] = (entry, catalogue.file_hash(self.store, entry['path'],
                                                                                   entry['pack']))
        return found

    def reload(self, paths=()) -> tuple[list[str], list[tuple[str, str]]]:
        """Re-read the store; returns (reloaded paths, [(path, what) that need a restart]). ``paths`` limits it to
        those files. Raises ConfigError (or what ``apply`` raises) with nothing applied."""
        self.store.reload()
        now = self._snapshot()
        changed = [key for key in {**self.seen, **now} if (not paths or key[1] in paths)
                   and now.get(key, (None, None))[1] != self.seen.get(key, (None, None))[1]]
        entries = [(now.get(key) or self.seen[key])[0] for key in changed]
        live = [e for e in entries if self.applies & set(e['reload'])]
        restart = [(e['path'], self.restarts[e['kind']]) for e in entries if e['kind'] in self.restarts]
        refused = self.apply(live) if self.apply and live else []   # not taken into effect
        restart += refused
        stuck = {path for path, _ in restart}
        reloaded = []
        for key, entry in zip(changed, entries, strict=True):
            if entry in live and entry['path'] not in {path for path, _ in refused}:
                reloaded.append(entry['path'])
            if entry['path'] in stuck:
                continue    # stays unseen: reported again until a restart
            if key in now:
                self.seen[key] = now[key]
            else:
                self.seen.pop(key, None)
        return sorted(set(reloaded)), sorted(set(restart))
