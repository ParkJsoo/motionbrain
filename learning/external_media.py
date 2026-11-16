"""Explicit, hash-bound reads of relocated media; never authorizes writes."""
from contextlib import contextmanager
from contextvars import ContextVar
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import plistlib
import stat
import subprocess

_ACTIVE = ContextVar('motionbrain_external_media', default=None)
_DIRECTORY = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW
_MEDIA = ('.bgra', '.lzbitmap', '.png', '.jpg', '.jpeg', '.mkv', '.mov', '.mp4', '.webm', '.avi', '.bgra.gz')


def _parts(value):
    if not isinstance(value, str) or not value or '\\' in value or '\x00' in value:
        raise ValueError('unsafe_media_path')
    parts = value.split('/')
    if PurePosixPath(value).is_absolute() or any(x in ('', '.', '..') for x in parts):
        raise ValueError('unsafe_media_path')
    return parts


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate_media_receipt_key')
        result[key] = value
    return result


def _mount_fd(mount, expected_uuid, logical):
    if mount.resolve(strict=True) != mount or mount.is_symlink():
        raise ValueError('media_mount_symlink')
    info = plistlib.loads(subprocess.check_output(
        ['diskutil', 'info', '-plist', str(mount)], timeout=15))
    if (info.get('VolumeUUID') != expected_uuid or info.get('MountPoint') != str(mount)
            or info.get('Internal') is not False):
        raise ValueError('media_volume_identity_changed')
    fd = os.open(mount, _DIRECTORY)
    if os.fstat(fd).st_dev == logical.stat().st_dev:
        os.close(fd)
        raise ValueError('media_volume_not_external')
    return fd


@contextmanager
def external_media_reads(receipt_path, expected_sha256, expected_volume_uuid):
    """Enable only receipt-listed media leaves in this execution context.

    Stored JSON, code, models, root links, and intermediate directory links keep
    their existing rejection policy. Callers must supply the expected receipt
    digest and volume identity independently of the receipt's contents.
    """
    raw = Path(receipt_path).read_bytes()
    if hashlib.sha256(raw).hexdigest() != expected_sha256:
        raise ValueError('media_receipt_sha256_changed')
    receipt = json.loads(raw, object_pairs_hook=_unique)
    if receipt.get('schemaVersion') != 'motionbrain.external_media_receipt.v1':
        raise ValueError('media_receipt_schema')
    logical = Path(receipt['logicalRoot'])
    mount = Path(receipt['mountPoint'])
    if (not logical.is_absolute() or logical.resolve(strict=True) != logical
            or not mount.is_absolute() or receipt['volumeUuid'] != expected_volume_uuid):
        raise ValueError('media_root_identity')
    prefix = _parts(receipt['payloadPrefix'])
    entries = receipt['files']
    if not isinstance(entries, dict) or not entries:
        raise ValueError('media_receipt_files')
    for name, item in entries.items():
        _parts(name)
        if (not name.lower().endswith(_MEDIA) or type(item.get('bytes')) is not int
                or item['bytes'] < 0 or not isinstance(item.get('sha256'), str)
                or len(item['sha256']) != 64
                or any(c not in '0123456789abcdef' for c in item['sha256'])):
            raise ValueError('media_receipt_entry')
    fd = _mount_fd(mount, expected_volume_uuid, logical)
    token = _ACTIVE.set(dict(logical=logical, mount=mount, prefix=prefix, entries=entries,
                             fd=fd, device=os.fstat(fd).st_dev))
    try:
        yield
    finally:
        _ACTIVE.reset(token)
        os.close(fd)


def read_media_alias(path):
    """Read one registered leaf through a pinned mount FD and verify its bytes."""
    active = _ACTIVE.get()
    if active is None:
        raise ValueError('external_media_context_required')
    path = Path(path).absolute()
    if path.parent.resolve(strict=True) != path.parent or not path.is_symlink():
        raise ValueError('media_alias_must_be_leaf')
    relative = path.relative_to(active['logical']).as_posix()
    parts = _parts(relative)
    record = active['entries'].get(relative)
    if record is None:
        raise ValueError('media_alias_not_registered')
    target = active['mount'].joinpath(*active['prefix'], *parts)
    if os.readlink(path) != str(target):
        raise ValueError('media_alias_target_changed')
    # Reject stale/replaced mounts even if the receipt itself is unchanged.
    if active['mount'].stat().st_dev != active['device']:
        raise ValueError('media_mount_changed')
    parent = os.dup(active['fd'])
    fd = None
    try:
        for component in [*active['prefix'], *parts[:-1]]:
            child = os.open(component, _DIRECTORY, dir_fd=parent)
            os.close(parent)
            parent = child
        fd = os.open(parts[-1], os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent)
        before = os.fstat(fd)
        if (not stat.S_ISREG(before.st_mode) or before.st_dev != active['device']
                or before.st_size != record['bytes']):
            raise ValueError('media_payload_identity')
        with os.fdopen(fd, 'rb') as stream:
            fd = None
            raw = stream.read(record['bytes'] + 1)
            after = os.fstat(stream.fileno())
        if ((before.st_size, before.st_mtime_ns, before.st_ino) !=
                (after.st_size, after.st_mtime_ns, after.st_ino)
                or len(raw) != record['bytes']
                or hashlib.sha256(raw).hexdigest() != record['sha256']):
            raise ValueError('media_payload_sha256_changed')
        return raw
    finally:
        if fd is not None:
            os.close(fd)
        os.close(parent)
