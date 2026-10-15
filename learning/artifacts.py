"""Immutable local evidence storage. This module never opens a network connection.

Each event is a separate durable file. A contiguous event prefix cannot prove that
an unseen tail ever existed; callers must require their own terminal run record.
SHA-256 detects accidental changes, not malicious edits to both data and manifests.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import threading
import uuid

MAX_IMAGE_BYTES = 2 * 1024 * 1024
_SAFE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")
_SHA = re.compile(r"[0-9a-f]{64}\Z")
_EVENT = re.compile(r"([0-9]{8})\.json\Z")
_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW


class StorageError(RuntimeError):
    """Evidence could not be safely stored or read; callers must retain failure."""


def _parts(relative):
    if not isinstance(relative, str) or not relative or "\\" in relative:
        raise StorageError("unsafe artifact reference")
    parts = relative.split("/")
    if PurePosixPath(relative).is_absolute() or any(not _SAFE.fullmatch(p) or p in (".", "..") for p in parts):
        raise StorageError("unsafe artifact reference")
    return parts


def _identifier(value):
    if not isinstance(value, str) or not _SAFE.fullmatch(value) or value in (".", ".."):
        raise StorageError("unsafe identifier")
    return value


def _parent_fd(root, relative, create=False):
    parts = _parts(relative)
    fd = os.open(root, _DIRECTORY_FLAGS)
    try:
        for component in parts[:-1]:
            if create:
                try:
                    os.mkdir(component, mode=0o700, dir_fd=fd)
                    os.fsync(fd)
                except FileExistsError:
                    pass
            child = os.open(component, _DIRECTORY_FLAGS, dir_fd=fd)
            os.close(fd)
            fd = child
        return fd, parts[-1]
    except BaseException:
        os.close(fd)
        raise


def _read(root, relative):
    parent = fd = None
    try:
        parent, name = _parent_fd(root, relative)
        fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent)
        import stat
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise StorageError("artifact is not a regular file")
        with os.fdopen(fd, "rb") as source:
            fd = None
            return source.read()
    except (OSError, ValueError) as exc:
        raise StorageError(f"cannot read {relative}: {exc}") from exc
    finally:
        if fd is not None:
            os.close(fd)
        if parent is not None:
            os.close(parent)


def _decode_json(raw):
    def unique_pairs(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate JSON key")
            result[key] = value
        return result
    def reject_constant(value):
        raise ValueError(f"invalid JSON constant: {value}")
    def finite_float(value):
        parsed = float(value)
        if not math.isfinite(parsed):
            raise ValueError("nonfinite JSON number")
        return parsed
    return json.loads(raw.decode("utf-8"), object_pairs_hook=unique_pairs,
                      parse_constant=reject_constant, parse_float=finite_float)


class ArtifactStore:
    def __init__(self, run_dir: Path):
        try:
            requested = Path(run_dir)
            if not requested.name or requested.name in (".", ".."):
                raise StorageError("a new run directory is required")
            requested.parent.mkdir(parents=True, exist_ok=True)
            self.root = requested.parent.resolve() / requested.name
            self.root.mkdir(mode=0o700, exist_ok=False)
            parent = os.open(self.root.parent, _DIRECTORY_FLAGS)
            try:
                os.fsync(parent)
            finally:
                os.close(parent)
        except (OSError, ValueError) as exc:
            raise StorageError(f"cannot create new run directory: {exc}") from exc
        self._sequence = 0
        self._event_lock = threading.Lock()

    def prepare_parent(self, relative: str):
        """Durably prepare directories before capture, without creating evidence.

        The normal writer still fsyncs each file and its parent and verifies
        readback. Empty prepared directories never prove a completed capture.
        """
        parent = None
        try:
            parent, _ = _parent_fd(self.root, relative, create=True)
        except (OSError, ValueError) as exc:
            raise StorageError(f"cannot prepare {relative}: {exc}") from exc
        finally:
            if parent is not None:
                os.close(parent)

    def _write(self, relative, content):
        _parts(relative)
        parent = fd = None
        temporary = ".pending-" + uuid.uuid4().hex
        digest = hashlib.sha256(content).hexdigest()
        try:
            parent, name = _parent_fd(self.root, relative, create=True)
            fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
                         0o600, dir_fd=parent)
            with os.fdopen(fd, "wb") as target:
                fd = None
                target.write(content)
                target.flush()
                os.fsync(target.fileno())
            # Hardlink is an atomic create-if-absent, unlike rename/replace.
            os.link(temporary, name, src_dir_fd=parent, dst_dir_fd=parent,
                    follow_symlinks=False)
            os.unlink(temporary, dir_fd=parent)
            os.fsync(parent)
            stored = _read(self.root, relative)
            if len(stored) != len(content) or hashlib.sha256(stored).hexdigest() != digest:
                raise StorageError(f"readback mismatch: {relative}")
            return {"artifactRef": relative, "sha256": digest, "bytes": len(content)}
        except (OSError, ValueError, TypeError) as exc:
            raise StorageError(f"cannot write {relative}: {exc}") from exc
        finally:
            if fd is not None:
                os.close(fd)
            if parent is not None:
                try:
                    os.unlink(temporary, dir_fd=parent)
                except FileNotFoundError:
                    pass
                except OSError as exc:
                    raise StorageError(f"cannot remove pending artifact: {exc}") from exc
                finally:
                    os.close(parent)

    def write_json(self, relative: str, data):
        try:
            encoded = (json.dumps(data, ensure_ascii=False, allow_nan=False,
                                  sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        except (ValueError, TypeError, OverflowError, UnicodeError, RecursionError) as exc:
            raise StorageError(f"cannot serialize JSON: {exc}") from exc
        return self._write(relative, encoded)

    def save_image(self, transition_id, phase, frame: bytes, expected_sha256):
        transition_id, phase = _identifier(transition_id), _identifier(phase)
        if not isinstance(frame, bytes) or not 0 < len(frame) <= MAX_IMAGE_BYTES:
            raise StorageError("image must be nonempty bytes within 2 MiB")
        if not isinstance(expected_sha256, str) or not _SHA.fullmatch(expected_sha256):
            raise StorageError("expected image SHA-256 is required")
        descriptor = self._write(f"images/{transition_id}/{phase}.jpg", frame)
        descriptor.update(expectedSha256=expected_sha256,
                          hashMatches=descriptor["sha256"] == expected_sha256)
        return descriptor

    def append(self, kind, transition_id, payload):
        _identifier(kind)
        _identifier(transition_id)
        with self._event_lock:
            sequence = self._sequence + 1
            if sequence > 99999999:
                raise StorageError("event sequence exhausted")
            descriptor = self.write_json(f"events/{sequence:08d}.json", {
                "sequence": sequence, "kind": kind, "transitionId": transition_id, "payload": payload})
            self._sequence = sequence
            return descriptor


def verify_artifacts(root, descriptors):
    """Return errors for unsafe, missing, or changed immutable artifacts."""
    errors = []
    if not isinstance(descriptors, (list, tuple)):
        return ["artifact descriptors must be a list"]
    for index, descriptor in enumerate(descriptors):
        label = f"artifact[{index}]"
        try:
            if not isinstance(descriptor, dict):
                raise StorageError("descriptor must be an object")
            reference, digest, size = (descriptor.get(k) for k in ("artifactRef", "sha256", "bytes"))
            _parts(reference)
            label = reference
            if not isinstance(digest, str) or not _SHA.fullmatch(digest):
                raise StorageError("invalid SHA-256")
            if type(size) is not int or size < 0:
                raise StorageError("invalid byte count")
            content = _read(root, reference)
            if len(content) != size:
                errors.append(f"{label}: byte_count_mismatch")
            if hashlib.sha256(content).hexdigest() != digest:
                errors.append(f"{label}: sha256_mismatch")
        except (StorageError, OSError, ValueError) as exc:
            errors.append(f"{label}: {exc}")
    return errors


def read_json_artifact(root, descriptor):
    """Verify and decode the same bytes through a no-symlink read."""
    if not isinstance(descriptor, dict):
        raise StorageError("descriptor must be an object")
    try:
        content = _read(root, descriptor["artifactRef"])
        if (type(descriptor.get("bytes")) is not int or len(content) != descriptor["bytes"]
                or hashlib.sha256(content).hexdigest() != descriptor.get("sha256")):
            raise StorageError("JSON artifact integrity mismatch")
        return _decode_json(content)
    except (KeyError, ValueError, TypeError, UnicodeError, RecursionError) as exc:
        raise StorageError(f"invalid JSON artifact: {exc}") from exc


def describe_artifact(root, relative):
    """Fingerprint an existing local file; this alone does not establish trust."""
    content = _read(root, relative)
    return {"artifactRef": relative, "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}


def read_events(root):
    """Read a verified structural event prefix; absent terminal evidence is unknown.

    No missing-tail guarantee is possible from directory contents alone. The
    collector must check run/transition terminal events and artifact descriptors.
    """
    events, errors = [], []
    root_fd = events_fd = None
    try:
        root_fd = os.open(root, _DIRECTORY_FLAGS)
        try:
            events_fd = os.open("events", _DIRECTORY_FLAGS, dir_fd=root_fd)
        except FileNotFoundError:
            return [], []
        names = sorted(os.listdir(events_fd))
        expected = 1
        for name in names:
            match = _EVENT.fullmatch(name)
            if not match or int(match.group(1)) != expected:
                errors.append(f"events/{name}: noncontiguous_or_invalid_sequence")
                break
            try:
                event = _decode_json(_read(root, f"events/{name}"))
                if not isinstance(event, dict) or type(event.get("sequence")) is not int or event["sequence"] != expected:
                    raise StorageError("event sequence mismatch")
                _identifier(event.get("kind"))
                _identifier(event.get("transitionId"))
                if "payload" not in event:
                    raise StorageError("missing event payload")
            except (StorageError, ValueError, UnicodeError, RecursionError) as exc:
                errors.append(f"events/{name}: {exc}")
                break
            events.append(event)
            expected += 1
    except (OSError, ValueError) as exc:
        errors.append(f"events: {exc}")
    finally:
        if events_fd is not None:
            os.close(events_fd)
        if root_fd is not None:
            os.close(root_fd)
    return events, errors
