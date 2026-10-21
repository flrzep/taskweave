"""A cell pack as one file: create an empty one (init), pack it (export), take one in (import).

The archive is a .tar.gz of the pack's files plus ``manifest.yaml`` (name, schema_version, git rev, taskweave
version, date, files with sha256). Import extracts into a temp dir next to the target, checks paths, links and hashes,
validates it like ``taskweave check``, and only then replaces the target: a plain folder is swapped (the old one stays
as ``<dir>.bak-<time>``), a git work tree gets a commit (the old pack is history, the repo around it stays intact).
A bad archive never touches the current pack.
"""

from __future__ import annotations

import contextlib
import datetime
import hashlib
import io
import os
import re
import shutil
import stat
import subprocess
import tarfile
import tempfile
from pathlib import Path, PurePosixPath

import yaml

from . import packs
from .save import pack_lock

MANIFEST = 'manifest.yaml'
SKIP_DIRS = {'.git', '__pycache__', 'state'}
SKIP_SUFFIXES = ('.pyc', '.swp', '~')
SKIP_NAMES = {'.DS_Store'}
LAYOUT_DIRS = ('cell', 'objects', 'missions', 'skills')
EMPTY_FILES = ('cell/locations.yaml', 'cell/objects.yaml')
ROBOT_FILES = packs.CELL_ROBOT_FILES
MAX_BYTES = 1 << 30     # extracted size cap


class PackfileError(Exception):
    """A pack file problem the user can act on (one clear message)."""


# ---- helpers ------------------------------------------------------------------
def _name(text: str) -> str:
    text = re.sub(r'[^A-Za-z0-9_\-]', '_', text)
    return text if re.match(r'[A-Za-z_]', text) else f'cell_{text}'


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def _git(cwd: Path, *args: str, stdin: str | None = None, check: bool = True) -> str:
    try:
        run = subprocess.run(['git', *args], cwd=cwd, input=stdin, capture_output=True, text=True, encoding='utf-8')
    except OSError as error:
        raise PackfileError(f'git not usable: {error}') from error
    if check and run.returncode:
        raise PackfileError(f'git {args[0]} failed: {run.stderr.strip() or run.stdout.strip()}')
    return run.stdout if run.returncode == 0 else ''


def _is_git(folder: Path) -> bool:
    try:
        return subprocess.run(['git', 'rev-parse', '--is-inside-work-tree'], cwd=folder, capture_output=True,
                              text=True).stdout.strip() == 'true'
    except OSError:
        return False


def _skipped(rel: PurePosixPath) -> bool:
    return (any(part in SKIP_DIRS for part in rel.parts[:-1]) or rel.name in SKIP_NAMES
            or rel.name.endswith(SKIP_SUFFIXES))


def _pack_files(cell: Path) -> tuple[list[str], list[str]]:
    """(files, skipped symlinks) of the pack as posix paths relative to ``cell``: the git work tree's files that are
    not ignored if the folder is in a repo, else every file; never caches, state/ or .git."""
    if _is_git(cell):
        listed = _git(cell, 'ls-files', '-co', '--exclude-standard', '-z').split('\0')
    else:
        listed = [p.relative_to(cell).as_posix() for p in cell.rglob('*')]
    files, links = [], []
    for name in sorted(n for n in listed if n):
        rel = PurePosixPath(name)
        if _skipped(rel) or rel.name == MANIFEST and len(rel.parts) == 1:
            continue
        path = cell / name
        if path.is_symlink():
            links.append(name)
        elif path.is_file():
            files.append(name)
    return files, links


def _read_pack(folder: Path) -> dict:
    try:
        pack = packs.read_pack(folder)
    except Exception as error:
        raise PackfileError(f'{folder}: {error}') from error
    if pack is None:
        raise PackfileError(f'{folder} is not a cell pack: no {packs.PACK_FILE}')
    return pack


def _taskweave_version() -> str:
    from importlib import metadata

    for name in ('taskweave_core', 'taskweave-core'):
        with contextlib.suppress(metadata.PackageNotFoundError):
            return metadata.version(name)
    return 'unknown'


# ---- init -----------------------------------------------------------------------
def init_cell(cell: str | Path, robot: str, name: str | None = None) -> Path:
    """A minimal valid cell pack in ``cell`` (absent or empty): pack.yaml at the current schema, system.yaml naming the
    robot, empty cell/locations.yaml and cell/objects.yaml, the robot's empty stances / held poses / object points,
    and the objects/, missions/, skills/ folders. The robot must be a robot pack taskweave finds (env, installed)."""
    cell = Path(cell).expanduser().resolve()
    if cell.exists() and (not cell.is_dir() or any(cell.iterdir())):
        raise PackfileError(f'{cell} exists and is not an empty folder')
    if not re.fullmatch(packs.schemas.NAME['pattern'], robot):
        raise PackfileError(f'invalid robot name {robot!r}')
    try:
        found = packs.robot_packs(cell, None, robot)
    except Exception as error:
        raise PackfileError(str(error)) from error
    if not found:
        raise PackfileError(f'no robot pack {robot!r} found (env {packs.ROBOT_PACKS_ENV}, installed taskweave_robots)')
    # the files below are the old point files: the oldest layout until init writes the targets files (the migration)
    pack = {'name': _name(name or cell.name), 'version': 1, 'schema_version': packs.CELL_SCHEMA_OLDEST}
    files = {packs.PACK_FILE: '# cell pack: everything about this cell; the robot comes from its robot pack\n'
                              + yaml.safe_dump(pack, sort_keys=False),
             'system.yaml': yaml.safe_dump({'robot': robot}), **{f: '{}\n' for f in EMPTY_FILES},
             **{f'robots/{robot}/{f}': '{}\n' for f in ROBOT_FILES}}
    cell.mkdir(parents=True, exist_ok=True)
    for folder in LAYOUT_DIRS:
        (cell / folder).mkdir(exist_ok=True)
    for rel, text in files.items():
        (cell / rel).parent.mkdir(parents=True, exist_ok=True)
        with (cell / rel).open('w', encoding='utf-8', newline='') as stream:    # LF on Windows too
            stream.write(text)
    return cell


# ---- export ---------------------------------------------------------------------
def export_cell(cell: str | Path, file: str | Path) -> dict:
    """Write the pack in ``cell`` to ``file`` (.tar.gz); returns the manifest."""
    cell, file = Path(cell).expanduser().resolve(), Path(file).expanduser().resolve()
    if not cell.is_dir():
        raise PackfileError(f'{cell} is not a folder')
    pack = _read_pack(cell)
    files, links = _pack_files(cell)
    rev = None
    if _is_git(cell):
        rev = _git(cell, 'rev-parse', 'HEAD', check=False).strip() or None
    manifest = {'name': pack['name'], 'schema_version': int(pack['schema_version']), 'git_rev': rev,
                'taskweave_version': _taskweave_version(),
                'date': datetime.datetime.now(datetime.UTC).strftime('%Y-%m-%dT%H:%M:%SZ'),
                'files': [{'path': f, 'sha256': _sha256(cell / f), 'size': (cell / f).stat().st_size} for f in files]}
    if links:
        manifest['skipped_links'] = links
    data = yaml.safe_dump(manifest, sort_keys=False).encode()
    file.parent.mkdir(parents=True, exist_ok=True)
    handle, temp = tempfile.mkstemp(dir=file.parent, prefix=f'.{file.name}.')
    try:
        with os.fdopen(handle, 'wb') as raw, tarfile.open(fileobj=raw, mode='w:gz') as archive:
            info = tarfile.TarInfo(MANIFEST)
            info.size, info.mtime, info.mode = len(data), int(datetime.datetime.now().timestamp()), 0o644
            archive.addfile(info, io.BytesIO(data))
            for name in files:
                archive.add(cell / name, arcname=name, recursive=False, filter=_plain_info)
        os.replace(temp, file)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(temp)
        raise
    return manifest


def _plain_info(info: tarfile.TarInfo) -> tarfile.TarInfo:
    """No owner or group in the archive (the importer sets them), mode normalised."""
    info.uid = info.gid = 0
    info.uname = info.gname = ''
    info.mode = 0o755 if info.mode & stat.S_IXUSR else 0o644
    return info


# ---- import ---------------------------------------------------------------------
def _safe_path(name: str) -> PurePosixPath:
    rel = PurePosixPath(name)
    if (not name or '\\' in name or '\0' in name or rel.is_absolute() or any(p in ('..', '') for p in name.split('/'))
            or re.match(r'^[A-Za-z]:', name)):
        raise PackfileError(f'unsafe path in archive: {name!r}')
    return rel


def _extract(file: Path, into: Path) -> dict:
    """Extract regular files only into ``into``; returns the manifest after checking paths, links and hashes."""
    try:
        archive = tarfile.open(file, 'r:gz')
    except (tarfile.TarError, OSError, EOFError) as error:
        raise PackfileError(f'{file} is not a readable cell pack archive: {error}') from error
    total = 0
    try:
        with archive:
            for member in archive:
                if member.isdir():
                    _safe_path(member.name.rstrip('/'))
                    continue
                if not member.isreg():
                    raise PackfileError(f'archive member {member.name!r} is a link or special file; refused')
                rel = _safe_path(member.name)
                total += member.size
                if total > MAX_BYTES:
                    raise PackfileError('archive is too large')
                target = into.joinpath(*rel.parts)
                target.parent.mkdir(parents=True, exist_ok=True)
                source = archive.extractfile(member)
                with target.open('wb') as stream:
                    shutil.copyfileobj(source, stream)
    except (tarfile.TarError, OSError, EOFError) as error:
        raise PackfileError(f'{file} is damaged: {error}') from error
    path = into / MANIFEST
    if not path.is_file():
        raise PackfileError(f'{file} has no {MANIFEST}: not a cell pack archive')
    try:
        manifest = yaml.safe_load(path.read_text(encoding='utf-8'))
        listed = {e['path']: e['sha256'] for e in manifest['files']}
        manifest['name'], manifest['schema_version']
    except (yaml.YAMLError, KeyError, TypeError) as error:
        raise PackfileError(f'bad {MANIFEST}: {error!r}') from error
    present = {p.relative_to(into).as_posix() for p in into.rglob('*') if p.is_file()} - {MANIFEST}
    if present != set(listed):
        diff = sorted(present ^ set(listed))
        raise PackfileError(f'archive files and {MANIFEST} differ: {", ".join(diff[:5])}')
    for rel, digest in listed.items():
        if _sha256(into / rel) != digest:
            raise PackfileError(f'checksum mismatch: {rel}')
    path.unlink()
    return manifest


def _validate(folder: Path, manifest: dict) -> None:
    """Schema first (clear message), then the same check as ``taskweave check``."""
    version = manifest['schema_version']
    if not packs.CELL_SCHEMA_OLDEST <= version <= packs.CELL_SCHEMA:
        hint = ('migrate it with the taskweave that wrote it' if version < packs.CELL_SCHEMA_OLDEST
                else 'update taskweave')
        raise PackfileError(f'incompatible cell pack: schema {version}, this taskweave reads schema '
                            f'{packs.CELL_SCHEMA} ({hint})')
    pack = _read_pack(folder)
    if int(pack['schema_version']) != version:
        raise PackfileError(f'{MANIFEST} says schema {version}, pack.yaml says {pack["schema_version"]}')
    from ..cli import check

    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out):
            code = check(str(folder))
    except Exception as error:
        raise PackfileError(f'pack check failed: {error}') from error
    if code:
        bad = [line.strip() for line in out.getvalue().splitlines() if 'FAIL' in line]
        raise PackfileError('pack check failed: ' + '; '.join(bad))


def _tree_files(folder: Path) -> list[str]:
    return sorted(p.relative_to(folder).as_posix() for p in folder.rglob('*') if p.is_file())


def _fit(folder: Path, like: Path) -> list[str]:
    """Give the new tree the group and mode of the old one (setgid and g+w kept); returns warnings."""
    info = like.stat()
    dir_mode = stat.S_IMODE(info.st_mode)
    group_write = bool(dir_mode & stat.S_IWGRP)
    warnings = []
    for path in [folder, *sorted(folder.rglob('*'))]:
        try:
            if hasattr(os, 'chown') and path.stat().st_gid != info.st_gid:
                os.chown(path, -1, info.st_gid)
        except OSError as error:
            if not warnings:
                warnings.append(f'group not set to the old one: {error}')
        if path.is_dir():
            os.chmod(path, dir_mode)
        else:
            os.chmod(path, 0o644 | (0o020 if group_write else 0) | (0o111 & path.stat().st_mode))
    return warnings


def _commit_identity(cwd: Path) -> list[str]:
    configured = _git(cwd, 'config', 'user.name', check=False).strip()
    return [] if configured else ['-c', 'user.name=taskweave', '-c', 'user.email=taskweave@localhost']


def _writable(cell: Path, rel: str, exists: bool) -> bool:
    target = cell / rel
    if exists and not os.access(target, os.W_OK):
        return False
    folder = target.parent
    while not folder.exists():
        folder = folder.parent
    return os.access(folder, os.W_OK | os.X_OK)


def _import_git(source: Path, cell: Path, manifest: dict, force: bool) -> dict:
    """Replace the pack files of the work tree folder ``cell`` by ``source``'s and commit exactly those paths. Every
    target is checked writable first; a failure while applying restores the touched files (HEAD), drops new ones."""
    old = _pack_files(cell)[0]
    new = _tree_files(source)
    removed = [f for f in old if f not in new]
    touched = [*new, *removed]
    locked = [f for f in touched if not _writable(cell, f, (cell / f).exists())]
    if locked:
        raise PackfileError(f'not writable, nothing changed: {", ".join(locked[:5])}')
    dirty = [line for line in _git(cell, 'status', '--porcelain', '-z', '--', '.').split('\0') if line]
    ident = _commit_identity(cell)
    if dirty and not force:
        raise PackfileError(f'{cell} has uncommitted changes; commit them or use --force (they are committed first)')
    if dirty:   # keep them in history before they are replaced
        _git(cell, *ident, 'add', '-A', '--', '.')
        _git(cell, *ident, 'commit', '-m', 'taskweave: snapshot before import', '--', '.')
    tracked = set(_git(cell, 'ls-files', '-z', '--', '.').split('\0'))
    created = [f for f in new if not (cell / f).exists()]
    try:
        return _apply_git(source, cell, manifest, ident, new, removed, touched)
    except BaseException as error:
        problem = _rollback(cell, [f for f in touched if f in tracked], created)
        raise PackfileError(f'import failed: {error}; ' + (f'the pack could not be fully restored ({problem}): '
                                                          f'git status shows what is left' if problem
                                                          else 'the pack is restored, nothing committed')) from error


def _rollback(cell: Path, tracked: list[str], created: list[str]) -> str:
    """Put the tracked files back from HEAD, drop the new ones; '' or what went wrong."""
    problems = []
    try:
        if tracked:
            _git(cell, 'checkout', 'HEAD', '--pathspec-from-file=-', '--pathspec-file-nul',
                 stdin='\0'.join(tracked) + '\0')
        if created:
            _git(cell, 'rm', '-q', '-f', '--cached', '--ignore-unmatch', '--pathspec-from-file=-',
                 '--pathspec-file-nul', stdin='\0'.join(created) + '\0')
    except PackfileError as error:
        problems.append(str(error))
    for rel in created:
        try:
            (cell / rel).unlink(missing_ok=True)
        except OSError as error:
            problems.append(str(error))
    return '; '.join(problems)


def _apply_git(source: Path, cell: Path, manifest: dict, ident: list, new: list, removed: list, touched: list) -> dict:
    for rel in removed:
        (cell / rel).unlink()
    for rel in new:
        target = cell / rel
        existed = target.exists()
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source / rel, target)   # in place: an existing file keeps owner and mode
        if not existed and cell.stat().st_mode & stat.S_IWGRP:
            os.chmod(target, stat.S_IMODE(target.stat().st_mode) | stat.S_IWGRP)
    paths = '\0'.join(touched) + '\0'
    spec = ['--pathspec-from-file=-', '--pathspec-file-nul']
    _git(cell, 'add', '-A', *spec, stdin=paths)
    rev = manifest.get('git_rev') or 'no rev'
    message = f'import {manifest["name"]} {rev}'
    if set(_git(cell, 'diff', '--cached', '--name-only', '--relative', '-z', '--', '.').split('\0')) & set(touched):
        _git(cell, *ident, 'commit', '-m', message, '--only', *spec, stdin=paths)
        commit = _git(cell, 'rev-parse', 'HEAD').strip()
    else:
        commit = None   # identical to what is there
    return {'mode': 'git', 'commit': commit, 'backup': None, 'warnings': []}


def import_cell(file: str | Path, cell: str | Path, force: bool = False) -> dict:
    """Take the pack archive ``file`` into ``cell``; returns a report (name, schema_version, mode, backup, commit,
    files, warnings). ``cell`` must be absent, empty or a clean git work tree unless ``force``. Any problem raises
    PackfileError before the current pack is touched."""
    file, cell = Path(file).expanduser().resolve(), Path(cell).expanduser().resolve()
    if not file.is_file():
        raise PackfileError(f'{file} not found')
    exists = cell.exists()
    if exists and not cell.is_dir():
        raise PackfileError(f'{cell} is not a folder')
    occupied = exists and any(cell.iterdir())
    git_target = occupied and _is_git(cell)     # history is the backup: no --force needed while the pack is clean
    if occupied and not force and not git_target:
        raise PackfileError(f'{cell} is not empty: use --force to replace it (the old pack is kept as a backup)')
    cell.parent.mkdir(parents=True, exist_ok=True)
    work = Path(tempfile.mkdtemp(dir=cell.parent, prefix=f'.{cell.name}.import-'))
    try:
        manifest = _extract(file, work)
        _validate(work, manifest)
        report = {'name': manifest['name'], 'schema_version': manifest['schema_version'],
                  'git_rev': manifest.get('git_rev'), 'files': len(manifest['files'])}
        if git_target:
            with pack_lock(cell):
                return {**report, **_import_git(work, cell, manifest, force)}
        warnings, backup = [], None
        if occupied:
            warnings = _fit(work, cell)
            stamp = f'{datetime.datetime.now():%Y%m%d-%H%M%S}'
            backup = cell.with_name(f'{cell.name}.bak-{stamp}')
            for n in range(1, 100):
                if not backup.exists():
                    break
                backup = cell.with_name(f'{cell.name}.bak-{stamp}-{n}')
            os.rename(cell, backup)
        else:
            os.chmod(work, 0o777 & ~_umask())
            if exists:
                cell.rmdir()
        try:
            os.rename(work, cell)
        except OSError:
            if backup:
                os.rename(backup, cell)     # put the old pack back
            raise
        return {**report, 'mode': 'swap' if backup else 'new', 'backup': str(backup) if backup else None,
                'commit': None, 'warnings': warnings}
    finally:
        shutil.rmtree(work, ignore_errors=True)


def _umask() -> int:
    mask = os.umask(0)
    os.umask(mask)
    return mask
