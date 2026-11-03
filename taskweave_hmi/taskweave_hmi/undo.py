"""Undo of a Teach save in the HMI. No ROS dependency.

A target write (``ConfigStore.save_target``, also the teach node's) is not a git commit, so ``store.history`` /
``store.restore`` cannot name the revision a save made. The HMI keeps, for each save it makes, the text the changed
files had before (in memory, the latest ``KEEP`` saves) and undoes it with ``ConfigStore.save`` (``delete`` for a file
the save created), with the hash the save left as ``base``: a file that changed since (another save, the pad, an editor)
is a conflict, nothing is written. Only files of the cell pack, chosen by the server (``teach_files``), are snapshot;
the browser names a save by its id only.
"""

from __future__ import annotations

import secrets
import threading
from collections import OrderedDict
from collections.abc import Callable, Iterable

from taskweave_core.config import ConfigStore, SaveConflict, SaveError

KEEP = 20  # saves that can be undone (the latest ones)


class UndoError(ValueError):
    """An undo that cannot be done; the message says why."""


def teach_files(robot: str, object_type: str | None = None) -> list[str]:
    """The cell pack files a Teach save can write: the cell's and the robot's targets, and for an object type its file
    and the robot's settings on it."""
    files = ['cell/targets.yaml', f'robots/{robot}/targets.yaml']
    if object_type:
        files += [f'objects/{object_type}.yaml', f'robots/{robot}/objects/{object_type}.yaml']
    return files


class Undo:
    """The latest saves, each ``{path: (text before or None, hash after)}`` by an id."""

    def __init__(self, keep: int = KEEP):
        self.keep = keep
        self.saves: OrderedDict[str, dict] = OrderedDict()
        self.lock = threading.Lock()

    @staticmethod
    def _text(store: ConfigStore, path: str) -> str | None:
        file = store.root / path
        return file.read_text(encoding='utf-8') if file.is_file() else None

    def run(self, store: ConfigStore, paths: Iterable[str], write: Callable, label: str):
        """``write()`` with the files ``paths`` (relative to the cell pack) noted before: ``(its result, the undo:
        {id, label, files} or None when no file changed)``."""
        before = {path: self._text(store, path) for path in dict.fromkeys(paths)}
        result = write()
        changed = {path: (text, store.file_hash(path)) for path, text in before.items()
                   if self._text(store, path) != text}
        if not changed:
            return result, None
        key = secrets.token_hex(6)
        with self.lock:
            self.saves[key] = {'label': label, 'files': changed}
            while len(self.saves) > self.keep:
                self.saves.popitem(last=False)
        return result, {'id': key, 'label': label, 'files': sorted(changed)}

    def undo(self, store: ConfigStore, key: str) -> dict:
        """Put the files of save ``key`` back as they were before it: ``{label, files}``. UndoError when it is not
        held (undone already, too old, the HMI restarted) or a file changed since (nothing is written then)."""
        with self.lock:
            entry = self.saves.get(str(key))
        if entry is None:
            raise UndoError('nothing to undo: that save is undone already, too old, or the HMI restarted')
        files = entry['files']
        changed = [path for path, (_text, after) in files.items() if store.file_hash(path) != after]
        if changed:
            raise UndoError(f'{", ".join(changed)} changed since that save: undo by hand (the editor or git)')
        message = f'HMI: undo {entry["label"]}'
        done = []
        # the robot's files first: its settings on a type's target go before the target they refer to
        order = sorted(files, key=lambda path: (not path.startswith('robots/'), path))
        try:
            for path in order:
                text, after = files[path]
                if text is None:
                    store.delete(path, base=after, message=message)
                else:
                    store.save(path, text, base=after, message=message)
                done.append(path)
        except (SaveConflict, SaveError) as error:
            undone = f' ({", ".join(done)} undone already)' if done else ''
            raise UndoError(f'{path}: {error}{undone}') from error
        finally:
            store.reload()
        with self.lock:
            self.saves.pop(str(key), None)
        return {'label': entry['label'], 'files': sorted(files)}
