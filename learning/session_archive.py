"""Offline FFV1 copies of closed manual sessions; never deletes source data."""
import ctypes
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

from learning.lossless_frame import SUFFIX, decode, library

SCHEMA = 'motionbrain.session_archive.v1'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def file_record(path):
    digest = hashlib.sha256()
    count = 0
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
            count += len(chunk)
    return dict(bytes=count, sha256=digest.hexdigest())


def safe(root, name):
    if root.is_symlink():
        raise ValueError('archive_symlink')
    root = root.resolve()
    p = Path(name)
    if p.is_absolute() or not p.parts or any(x in ('.', '..') for x in p.parts):
        raise ValueError('unsafe_archive_path')
    target = root / p
    if target.resolve() != target.absolute():
        raise ValueError('archive_symlink')
    return target


def inventory(root):
    result = {}
    for p in sorted(root.rglob('*')):
        if p.is_symlink():
            raise ValueError('source_symlink')
        if p.is_dir():
            continue
        if not p.is_file():
            raise ValueError('source_not_regular')
        result[p.relative_to(root).as_posix()] = file_record(p)
    return result


def encode(raw):
    src = ctypes.create_string_buffer(raw)
    dst = ctypes.create_string_buffer(len(raw) + 65536)
    n = library().compression_encode_buffer(dst, len(dst), src, len(raw), None, 0x702)
    if not n:
        raise ValueError('lzbitmap_encode_failed')
    return dst.raw[:n]


def checked(data, record):
    if len(data) != record['bytes'] or sha(data) != record['sha256']:
        raise ValueError('archive_hash_mismatch')
    return data


def load(root):
    root = Path(root).resolve()
    m = json.loads(safe(root, 'manifest.json').read_bytes())
    if m['schemaVersion'] != SCHEMA or not 1 <= len(m['frames']) <= 1000:
        raise ValueError('archive_contract')
    width, height = m['width'], m['height']
    if type(width) is not int or type(height) is not int or not (1 <= width <= 1920 and 1 <= height <= 1080):
        raise ValueError('archive_dimensions')
    names = [f['path'] for f in m['frames']]
    if len(set(names)) != len(names) or any(n not in m['files'] for n in names):
        raise ValueError('archive_frame_index')
    for name, record in m['files'].items():
        safe(root / 'files', name)
        if name not in names:
            checked(safe(root / 'files', name).read_bytes(), record)
    if file_record(safe(root, 'frames.mkv')) != m['video']:
        raise ValueError('archive_hash_mismatch')
    return root, m


def pixels(root, m, ffmpeg):
    cmd = [ffmpeg, '-nostdin', '-v', 'error', '-i', str(root / 'frames.mkv'),
           '-map', '0:v:0', '-f', 'rawvideo', '-pix_fmt', 'bgra', '-threads', '2', 'pipe:1']
    size = m['width'] * m['height'] * 4
    with tempfile.TemporaryFile() as errors:
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=errors)
        try:
            for frame in m['frames']:
                parts = bytearray()
                while len(parts) < size:
                    chunk = p.stdout.read(size - len(parts))
                    if not chunk:
                        raise ValueError('archive_video_truncated')
                    parts.extend(chunk)
                raw = bytes(parts)
                if sha(raw) != frame['pixelSha256']:
                    raise ValueError('archive_pixel_hash')
                yield frame['path'], checked(encode(raw), m['files'][frame['path']])
            if p.stdout.read(1) or p.wait() != 0:
                raise ValueError('archive_video_length_or_decode')
        finally:
            if p.poll() is None:
                p.kill()
            p.wait()
            p.stdout.close()


def verify(archive, ffmpeg='ffmpeg'):
    root, m = load(archive)
    for _ in pixels(root, m, ffmpeg):
        pass
    return dict(frames=len(m['frames']), files=len(m['files']),
                originalBytes=sum(r['bytes'] for r in m['files'].values()),
                archiveBytes=sum(p.stat().st_size for p in root.rglob('*') if p.is_file()))


def publish(stage, destination):
    # Exclusive directory creation prevents replacement of an existing archive.
    destination.mkdir(mode=0o700)
    # Completion manifest is installed last; partial copies are never complete archives.
    paths = sorted(stage.iterdir(), key=lambda p: p.name == 'manifest.json')
    for p in paths:
        os.rename(p, destination / p.name)


def create(session, destination, ffmpeg='ffmpeg'):
    session, destination = Path(session).resolve(), Path(destination).absolute()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(destination)
    if destination.resolve().is_relative_to(session):
        raise ValueError('archive_inside_source')
    state = json.loads((session / 'session.json').read_bytes())
    if state.get('schemaVersion') != 'motionbrain.manual_collection.v1' or state.get('phase') not in ('completed', 'failed'):
        raise ValueError('closed_manual_session_required')
    before = inventory(session)
    paths = sorted(n for n in before if n.endswith(SUFFIX))
    if not 1 <= len(paths) <= 1000:
        raise ValueError('archive_frame_count')
    frames = []
    dimensions = None
    for name in paths:
        meta = json.loads(safe(session, name[:-len(SUFFIX)] + '.json').read_bytes())
        shape = (meta['width'], meta['height'])
        if (dimensions is not None and dimensions != shape) or meta['rawByteLength'] != shape[0] * shape[1] * 4:
            raise ValueError('archive_frame_dimensions')
        if not (1 <= shape[0] <= 1920 and 1 <= shape[1] <= 1080):
            raise ValueError('archive_frame_dimensions')
        dimensions = shape
        checked(safe(session, name).read_bytes(), dict(bytes=meta['storedByteLength'], sha256=meta['imageSha256']))
        frames.append(dict(path=name, pixelSha256=meta['pixelSha256']))
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.session-archive-', dir=destination.parent) as tmp:
        stage = Path(tmp)
        for name in before:
            if name in paths:
                continue
            target = safe(stage / 'files', name)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(checked(safe(session, name).read_bytes(), before[name]))
        width, height = dimensions
        cmd = [ffmpeg, '-nostdin', '-v', 'error', '-f', 'rawvideo', '-pixel_format', 'bgra',
               '-video_size', f'{width}x{height}', '-framerate', '1', '-i', 'pipe:0', '-an',
               '-c:v', 'ffv1', '-level', '3', '-coder', '1', '-context', '1',
               '-pix_fmt', 'bgra', '-threads', '2', str(stage / 'frames.mkv')]
        with tempfile.TemporaryFile() as errors:
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=errors)
            try:
                for frame in frames:
                    stored = checked(safe(session, frame['path']).read_bytes(), before[frame['path']])
                    raw = decode(stored, width * height * 4, frame['pixelSha256'])
                    checked(encode(raw), before[frame['path']])
                    p.stdin.write(raw)
                p.stdin.close()
                if p.wait() != 0:
                    raise ValueError('archive_video_encode_failed')
            finally:
                if p.poll() is None:
                    p.kill()
                p.wait()
                p.stdin.close()
        m = dict(schemaVersion=SCHEMA, width=width, height=height, files=before, frames=frames,
                 video=file_record(stage / 'frames.mkv'),
                 timeline='index_only_original_timestamps_in_files')
        (stage / 'manifest.json').write_text(json.dumps(m, ensure_ascii=False, indent=2) + '\n')
        result = verify(stage, ffmpeg)
        if inventory(session) != before:
            raise ValueError('source_changed_during_archive')
        publish(stage, destination)
    return result


def restore(archive, destination, ffmpeg='ffmpeg'):
    root, m = load(archive)
    destination = Path(destination).absolute()
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(destination)
    if destination.resolve().is_relative_to(root):
        raise ValueError('restore_inside_archive')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.session-restore-', dir=destination.parent) as tmp:
        stage = Path(tmp)
        frame_names = {f['path'] for f in m['frames']}
        for name, record in m['files'].items():
            if name not in frame_names:
                target = safe(stage, name)
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(checked(safe(root / 'files', name).read_bytes(), record))
        for name, data in pixels(root, m, ffmpeg):
            target = safe(stage, name)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
        if inventory(stage) != m['files']:
            raise ValueError('restored_inventory_mismatch')
        publish(stage, destination)
    return dict(restoredFiles=len(m['files']), restoredFrames=len(m['frames']))
