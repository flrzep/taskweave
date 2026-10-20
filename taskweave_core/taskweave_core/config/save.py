"""Writing config files through core: ``ConfigStore.save`` / ``delete`` / ``restore`` / ``history`` / ``show``, the
mission and system.yaml robot setters. The HMI never writes YAML itself.

A save is: lock the cell pack, compare ``base`` with the file's current hash, validate (the file, then the pack with
the file in place; refuse on errors, warnings pass), write atomically (temp file, fsync, rename; owner, group and mode
of an existing file kept, a new file gets the directory's group and g+w), commit exactly that path if the pack is in
a git work tree. Everything lands in the cell pack: a robot pack file (``pack='robot:<name>'``) is written as a keyed
override (overlay.py) of the cell pack, the robot pack stays untouched.
"""

from __future__ import annotations

import contextlib
import io
import os
import re
import socket
import stat
import subprocess
import tempfile
from pathlib import Path, PurePosixPath

import yaml

from . import catalogue, overlay, packs, schemas
from .store import _pop, _yaml_handler

try:
    import fcntl
except ImportError:     # Windows: no cross-process lock (the pack is edited on the robot's host)
    fcntl = None

LOCK_NAME = '.taskweave.lock'


class SaveError(Exception):
    """A save or delete refused: ``problems`` are check.py problems (``severity`` error | warning), nothing written."""

    def __init__(self, problems: list[dict], message: str = ''):
        self.problems = problems
        errors = [p for p in problems if p['severity'] == 'error']
        super().__init__(message or f'{len(errors)} error(s), nothing written: '
                         + '; '.join(f'{p["path"]}{p["where"]}: {p["message"]}' for p in errors[:3]))


class SaveConflict(Exception):
    """The file changed since ``base`` (or exists while no base was given): ``current_hash`` and ``current_text`` are
    the file now (None: it does not exist), for the editor's diff."""

    def __init__(self, current_hash: str | None, current_text: str | None, message: str = ''):
        self.current_hash, self.current_text = current_hash, current_text
        super().__init__(message or 'the file changed since it was read')


def _problem(path: str, pack: str, message: str) -> dict:
    return {'path': path, 'pack': pack, 'where': '', 'line': None, 'col': None, 'message': message,
            'severity': 'error'}


def _problem_key(p: dict) -> tuple:
    return p['path'], p['pack'], p['where'], p['message'], p['severity']


# ---- lock ------------------------------------------------------------------------------------------------------------
@contextlib.contextmanager
def pack_lock(root: Path):
    """flock on a lock file in the cell pack (next to it if the pack is read-only), shared by every process."""
    root = Path(root)
    lock = root / LOCK_NAME if os.access(root, os.W_OK) else root.with_name(f'.{root.name}.taskweave.lock')
    try:
        fd = os.open(lock, os.O_RDWR | os.O_CREAT, 0o664)
    except PermissionError:     # made by someone else, not writable to us: flock needs only a readable file
        fd = os.open(lock, os.O_RDONLY)
    else:
        with contextlib.suppress(OSError, AttributeError):
            os.fchmod(fd, 0o664 | 0o020)
    try:
        if fcntl:
            fcntl.flock(fd, fcntl.LOCK_EX)
        _ignore_lock(lock)
        yield lock
    finally:
        os.close(fd)    # releases the lock


# ---- paths and routing -----------------------------------------------------------------------------------------------
def _split_pack(pack: str) -> str | None:
    if pack == 'cell':
        return None
    if pack.startswith('robot:') and len(pack) > 6:
        return pack[6:]
    raise ValueError(f"pack is 'cell' or 'robot:<name>', not {pack!r}")


def _clean(path: str) -> str:
    clean = PurePosixPath(path)
    if clean.is_absolute() or '..' in clean.parts or not clean.parts:
        raise SaveError([_problem(str(path), 'cell', 'not a path inside the pack')])
    return clean.as_posix()


def _target(path: str, pack: str) -> str:
    """The file a change lands in, relative to the cell pack: the file itself, or the cell override of a robot file."""
    robot = _split_pack(pack)
    return path if robot is None else f'robots/{robot}/{path}'


def _kind(path: str, pack: str):
    kind = catalogue.kind_of(path, pack)
    if kind is None or not kind.editable or not path.endswith(catalogue.YAML_SUFFIXES):
        reason = 'not a YAML file' if kind and kind.editable else kind.editable_reason if kind else (
            'not a file taskweave reads')
        raise SaveError([_problem(path, pack, f'cannot be edited here: {reason}')])
    return kind


def _text(path: Path) -> str | None:
    return path.read_text(encoding='utf-8', errors='replace') if path.is_file() else None


def _current_text(store, path: str, pack: str) -> str | None:
    """The file as the editor shows it: the file, or for a robot file the whole file in effect (override applied)."""
    robot = _split_pack(pack)
    if robot is None:
        return _text(store.root / path)
    if store.override_base(robot, path) is not None:
        return yaml.safe_dump(store.keyed_override(robot, path)[1], sort_keys=False)
    return _text(store.robot_file(robot, path))


# ---- building the new text -------------------------------------------------------------------------------------------
def _sync(node, data):
    """``node`` (ruamel) made equal to ``data`` in place, keeping the comments and order of what stays; returns the
    node to use (a different value replaces a scalar)."""
    from ruamel.yaml.comments import CommentedMap, CommentedSeq

    if overlay.same(json_plain(node), data):
        return node
    if isinstance(node, CommentedMap) and isinstance(data, dict):
        for key in [k for k in node if k not in data]:
            _pop(node, key)
        for key, value in data.items():
            node[key] = _sync(node[key], value) if key in node else value
        return node
    if isinstance(node, CommentedSeq) and isinstance(data, list):
        del node[len(data):]
        for i, value in enumerate(data):
            node.append(value) if i >= len(node) else node.__setitem__(i, _sync(node[i], value))
        return node
    return data


def json_plain(value):
    import json

    return json.loads(json.dumps(value))


def _dump(document) -> str:
    stream = io.StringIO()
    _yaml_handler().dump(document, stream)
    return stream.getvalue()


def _document(text: str | None):
    from ruamel.yaml.comments import CommentedMap

    document = _yaml_handler().load(text) if text else None
    return CommentedMap() if document is None else document


def _new_text(store, path: str, pack: str, content) -> str:
    """The text to write for ``content`` (text: as given; data: merged into the loaded file, comments kept)."""
    robot = _split_pack(pack)
    if robot is not None and store.override_base(robot, path) is not None:
        wanted = yaml.safe_load(content) if isinstance(content, str) else json_plain(content)
        if not isinstance(wanted, dict):
            raise SaveError([_problem(path, pack, 'a keyed override of a mapping file needs a mapping')])
        return _dump(store.keyed_override(robot, path, lambda document: _sync(document, wanted))[0])
    if isinstance(content, str):
        return content
    existing = _text(store.root / path if robot is None else store.robot_file(robot, path))
    return _dump(_sync(_document(existing), json_plain(content)))


# ---- writing ---------------------------------------------------------------------------------------------------------
def _makedirs(folder: Path) -> None:
    """Missing folders, each with its parent's group and mode (setgid and group write follow the parent)."""
    if folder.exists():
        return
    _makedirs(folder.parent)
    folder.mkdir()
    parent = folder.parent.stat()
    with contextlib.suppress(OSError, AttributeError):
        os.chown(folder, -1, parent.st_gid)
    os.chmod(folder, stat.S_IMODE(parent.st_mode))


def _write(target: Path, data: bytes) -> None:
    """Atomically: temp file next to it, fsync, rename. Existing file: its owner, group and mode; new file: the
    directory's group, 664 less the umask but g+w."""
    _makedirs(target.parent)
    if target.is_symlink():
        raise SaveError([_problem(target.name, 'cell', 'is a symbolic link: not written through')])
    info = target.stat() if target.exists() else None
    fd, temp = tempfile.mkstemp(dir=target.parent, prefix=f'.{target.name}.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if info is not None:
            uid, gid, mode = info.st_uid, info.st_gid, stat.S_IMODE(info.st_mode)
        else:
            umask = os.umask(0)
            os.umask(umask)
            uid, gid, mode = -1, target.parent.stat().st_gid, (0o666 & ~umask) | 0o020
        with contextlib.suppress(OSError, AttributeError):
            try:
                os.chown(temp, uid, gid)
            except PermissionError:     # not the owner (not root): the group at least
                os.chown(temp, -1, gid)
        os.chmod(temp, mode)
        os.replace(temp, target)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(temp)
        raise
    with contextlib.suppress(OSError, AttributeError):
        dir_fd = os.open(target.parent, os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)


# ---- git -------------------------------------------------------------------------------------------------------------
def _git(cwd: Path, *args: str, config: tuple = ()) -> subprocess.CompletedProcess:
    cmd = ['git', *(a for c in config for a in ('-c', c)), *args]
    return subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, encoding='utf-8', errors='replace')


def _repo(folder: Path) -> Path | None:
    try:
        run = _git(folder, 'rev-parse', '--show-toplevel')
    except OSError:     # no git
        return None
    return Path(run.stdout.strip()) if run.returncode == 0 and run.stdout.strip() else None


def _author(author: str | None) -> tuple[str, str]:
    author = author or f'taskweave HMI <hmi@{socket.gethostname()}>'
    found = re.fullmatch(r'\s*(.*?)\s*<([^>]*)>\s*', author)
    return (found.group(1), found.group(2)) if found else (author.strip(), f'{author.strip()}@{socket.gethostname()}')


def _last_rev(repo: Path, target: Path) -> str | None:
    run = _git(repo, 'log', '-1', '--format=%H', '--', str(target))
    return run.stdout.strip() or None if run.returncode == 0 else None


def _ignore_lock(lock: Path) -> None:
    """The lock file is no change of the pack: keep it out of ``git status`` (local exclude, no tracked file)."""
    try:
        repo = _repo(lock.parent)
        if repo is None or not lock.resolve().is_relative_to(repo.resolve()):
            return
        if _git(repo, 'check-ignore', '-q', str(lock)).returncode == 0:
            return
        exclude = Path(_git(repo, 'rev-parse', '--git-path', 'info/exclude').stdout.strip())
        exclude = exclude if exclude.is_absolute() else repo / exclude
        exclude.parent.mkdir(parents=True, exist_ok=True)
        with exclude.open('a', encoding='utf-8') as stream:
            stream.write(f'/{lock.resolve().relative_to(repo.resolve()).as_posix()}\n')
    except OSError:
        pass


def _commit(target: Path, message: str, author: str | None, removed: bool = False) -> tuple[str | None, list]:
    """Commit exactly ``target`` if it is in a git work tree: (rev, warnings). A git failure is a warning."""
    repo = _repo(target.parent if target.parent.exists() else target.parent.parent)
    if repo is None:
        return None, []
    name, email = _author(author)
    config = (f'user.name={name}', f'user.email={email}')
    try:
        stage = (_git(repo, 'rm', '-q', '--cached', '--ignore-unmatch', '--', str(target)) if removed
                 else _git(repo, 'add', '--', str(target)))
        if stage.returncode:
            return None, [f'not committed: git {stage.args[1]}: {stage.stderr.strip()}']
        if _git(repo, 'diff', '--cached', '--quiet', '--', str(target)).returncode == 0:
            return _last_rev(repo, target), []     # nothing changed
        run = _git(repo, 'commit', '-q', '--only', '-m', message, f'--author={name} <{email}>', '--', str(target),
                   config=config)
        if run.returncode:
            return None, [f'not committed: {(run.stderr or run.stdout).strip()}']
        return _git(repo, 'rev-parse', 'HEAD').stdout.strip() or None, []
    except OSError as error:
        return None, [f'not committed: {error}']


# ---- save and delete -------------------------------------------------------------------------------------------------
def _refresh(store) -> None:
    store.reload()
    packs._packs.clear()


def _check_base(store, path: str, pack: str, base: str | None, what: str) -> str | None:
    current = store.file_hash(path, pack)
    if current is not None and base is None:
        raise SaveConflict(current, _current_text(store, path, pack), f'{path} exists: {what} needs its hash as base')
    if base != current and not (base is None and current is None):
        raise SaveConflict(current, _current_text(store, path, pack))
    return current


def save(store, path: str, content, pack: str = 'cell', base: str | None = None, message: str | None = None,
         author: str | None = None) -> dict:
    """Write ``content`` (YAML text, as given; or data, merged into the loaded file keeping its comments and order:
    it is the whole new content, keys it lacks are removed) to ``path`` of ``pack`` ('cell' or 'robot:<name>', whose
    file is written as an override in the cell pack).

    ``base``: the hash the editor read (``file_hash``). A file that exists needs it (None raises SaveConflict
    too); None is for a new file. Raises SaveConflict (current hash and text) when the file changed, SaveError (with
    the problems) when the result has errors; the file is then untouched. Returns ``{path, pack, hash, rev (commit
    or None), reload, warnings (messages and problems), override (the cell file written for a robot file)}``."""
    path = _clean(path)
    kind, target = _kind(path, pack), _target(path, pack)
    with pack_lock(store.root):
        _check_base(store, path, pack, base, 'a save')
        text = _new_text(store, path, pack, content)
        warnings = _validate(store, path, pack, content, target, text, kind)
        file = store.root / target
        if not (file.is_file() and file.read_bytes() == text.encode('utf-8')):
            _write(file, text.encode('utf-8'))
        _refresh(store)
        rev, notes = _commit(file, message or f'HMI: edit {target}', author)
        return {'path': path, 'pack': pack, 'hash': store.file_hash(path, pack), 'rev': rev,
                'reload': list(kind.reload), 'warnings': warnings + notes,
                'override': target if pack != 'cell' else None}


def _validate(store, path, pack, content, target, text, kind) -> list:
    """Refuse (SaveError) on errors in the file or new ones in the pack with it in place; the warnings."""
    from . import check

    if kind.schema is None:
        own, new = [], []
        if path.endswith(catalogue.YAML_SUFFIXES):
            own = check.validate_file(store, path, content, pack)    # at least: is it YAML
    else:
        own = check.validate_file(store, path, content, pack)
        known = {_problem_key(p) for p in check.check_pack(store)}
        skip = (target, 'cell') if pack == 'cell' or own else None
        new = [p for p in check.dry_run_pack(store, target, text, 'cell')
               if _problem_key(p) not in known and (p['path'], p['pack']) != skip]
    problems = own + new
    if any(p['severity'] == 'error' for p in problems):
        raise SaveError(problems)
    return problems


def delete(store, path: str, pack: str = 'cell', base: str | None = None, message: str | None = None,
           author: str | None = None) -> dict:
    """Remove ``path`` (a robot file: its cell override, so the robot pack's file is in effect again). ``base`` is
    required, as for a save. Problems the pack has afterwards (a mission that included it) are returned as
    warnings. Returns ``{path, pack, hash (None), rev, reload, warnings, override}``."""
    from . import check

    path = _clean(path)
    kind, target = _kind(path, pack), _target(path, pack)
    with pack_lock(store.root):
        file = store.root / target
        if not file.is_file():
            raise SaveError([_problem(path, pack, 'there is no such file to delete' if pack == 'cell' else
                                      'no cell override of this robot pack file: nothing to delete')])
        _check_base(store, path, pack, base, 'a delete')
        before = {_problem_key(p) for p in check.check_pack(store)}
        file.unlink()
        _refresh(store)
        warnings = [p for p in check.check_pack(store) if _problem_key(p) not in before]
        rev, notes = _commit(file, message or f'HMI: delete {target}', author, removed=True)
        return {'path': path, 'pack': pack, 'hash': store.file_hash(path, pack), 'rev': rev,
                'reload': list(kind.reload), 'warnings': warnings + notes,
                'override': target if pack != 'cell' else None}


def create(store, kind: str, name: str = '', robot: str | None = None, content=None, message: str | None = None,
           author: str | None = None) -> dict:
    """A new file of ``kind`` (catalogue.new_file): ``content`` or the kind's template, saved without a base."""
    path, template = catalogue.new_file(kind, name, robot)
    return save(store, path, template if content is None else content, 'cell', None, message, author)


def save_mission(store, name: str, content, base: str | None = None, message: str | None = None,
                 author: str | None = None) -> dict:
    """Save missions/<name>.yaml, validated as a mission (schema, includes, step ids; a plan must build)."""
    if not re.fullmatch(schemas.NAME['pattern'], name or ''):
        raise SaveError([_problem(f'missions/{name}.yaml', 'cell', 'mission names: letters, digits, "_" and "-", '
                                                                    'starting with a letter')])
    return save(store, f'missions/{name}.yaml', content, 'cell', base, message, author)


def set_robot(store, robot: str, base: str | None = None, message: str | None = None,
              author: str | None = None) -> dict:
    """Set ``robot`` in system.yaml (a restart is needed: reload says launch). ``base`` None: the file as it is now."""
    known = sorted({p['name'] for p in store.robot_packs() if 'error' not in p} | set(store.robot_names()))
    if robot not in known:
        raise SaveError([_problem('system.yaml', 'cell', f'no robot {robot!r}; robots: {", ".join(known)}')])
    file = store.root / 'system.yaml'
    current = (yaml.safe_load(file.read_text(encoding='utf-8')) or {}) if file.is_file() else {}
    if base is None:
        base = store.file_hash('system.yaml')
    return save(store, 'system.yaml', {**current, 'robot': robot}, 'cell', base, message or f'HMI: robot {robot}',
                author)


# ---- history ---------------------------------------------------------------------------------------------------------
def _history_target(store, path: str, pack: str) -> Path:
    return catalogue.file_path(store, _clean(path), pack)


def history(store, path: str, pack: str = 'cell', limit: int = 50) -> list[dict]:
    """The commits that changed ``path`` (git log --follow), newest first: ``{rev, time (ISO), author, message}``;
    [] without a repo, git or history. A robot pack file has the history of the pack file (a cell override: its own
    path in the cell pack)."""
    target = _history_target(store, path, pack)
    repo = _repo(target.parent) if target.parent.is_dir() else None
    if repo is None:
        return []
    run = _git(repo, 'log', '--follow', f'-n{int(limit)}', '--format=%H%x1f%aI%x1f%an <%ae>%x1f%B%x1e', '--',
               str(target))
    if run.returncode:
        return []
    rows = (r.strip('\n').split('\x1f') for r in run.stdout.split('\x1e') if r.strip())
    return [{'rev': r[0].strip(), 'time': r[1], 'author': r[2], 'message': r[3].strip()} for r in rows]


def show(store, path: str, rev: str, pack: str = 'cell') -> str:
    """The text of ``path`` at ``rev``. ValueError if there is no such revision or the file was not in it."""
    target = _history_target(store, path, pack)
    repo = _repo(target.parent) if target.parent.is_dir() else None
    if repo is None:
        raise ValueError(f'{path} is not in a git repository')
    if not re.fullmatch(r'[0-9A-Za-z][0-9A-Za-z_./~^-]*', rev or ''):
        raise ValueError(f'not a revision: {rev!r}')
    relative = target.resolve().relative_to(repo.resolve()).as_posix()
    run = _git(repo, 'show', f'{rev}:{relative}')
    if run.returncode:
        raise ValueError(f'{path} at {rev}: {run.stderr.strip()}')
    return run.stdout


def restore(store, path: str, rev: str, pack: str = 'cell', base: str | None = None,
            author: str | None = None) -> dict:
    """Save the text of ``path`` at ``rev`` as a new change: ``base`` as in save (a conflict is raised the same
    way), message ``restore <path> to <rev[:8]>``."""
    return save(store, path, show(store, path, rev, pack), pack, base, f'restore {path} to {rev[:8]}', author)
