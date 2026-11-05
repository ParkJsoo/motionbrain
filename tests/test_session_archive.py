import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

from learning import session_archive as archive


@unittest.skipUnless(sys.platform == 'darwin' and shutil.which('ffmpeg'), 'macOS and FFmpeg required')
class SessionArchiveTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.source = self.root / 'source'
        self.camera = self.source / 'native-camera'
        self.camera.mkdir(parents=True)
        (self.source / 'session.json').write_text(json.dumps(dict(
            schemaVersion='motionbrain.manual_collection.v1', phase='completed')))
        for i in range(3):
            raw = bytes((x + i * 7) % 256 for x in range(64 * 48 * 4))
            data = archive.encode(raw)
            name = f'frame-{i:04d}'
            (self.camera / (name + '.bgra.lzbitmap')).write_bytes(data)
            (self.camera / (name + '.json')).write_text(json.dumps(dict(
                width=64, height=48, rawByteLength=len(raw), storedByteLength=len(data),
                imageSha256=hashlib.sha256(data).hexdigest(), pixelSha256=hashlib.sha256(raw).hexdigest(),
                pts=dict(value=i * 250, timescale=1000))))
        self.destination = self.root / 'archive'

    def test_complete_byte_exact_roundtrip_preserves_source_and_timestamps(self):
        before = archive.inventory(self.source)
        result = archive.create(self.source, self.destination)
        self.assertEqual(result['frames'], 3)
        self.assertEqual(archive.verify(self.destination), result)
        restored = self.root / 'restored'
        archive.restore(self.destination, restored)
        self.assertEqual(self.destination.stat().st_mode & 0o777, 0o700)
        self.assertEqual(restored.stat().st_mode & 0o777, 0o700)
        self.assertEqual(archive.inventory(restored), before)
        self.assertEqual(archive.inventory(self.source), before)
        self.assertEqual(json.loads((restored / 'native-camera/frame-0002.json').read_text())['pts']['value'], 500)

    def test_corrupted_video_or_metadata_never_publishes_restore(self):
        archive.create(self.source, self.destination)
        for path in [self.destination / 'frames.mkv', self.destination / 'files/session.json']:
            original = path.read_bytes()
            path.write_bytes(b'corrupt')
            with self.assertRaisesRegex(ValueError, 'hash_mismatch'):
                archive.restore(self.destination, self.root / 'restored')
            self.assertFalse((self.root / 'restored').exists())
            path.write_bytes(original)

    def test_wrong_pixel_hash_even_with_valid_video_hash_is_rejected(self):
        archive.create(self.source, self.destination)
        p = self.destination / 'manifest.json'
        m = json.loads(p.read_text()); m['frames'][0]['pixelSha256'] = '0' * 64
        p.write_text(json.dumps(m))
        with self.assertRaisesRegex(ValueError, 'pixel_hash'):
            archive.restore(self.destination, self.root / 'restored')
        self.assertFalse((self.root / 'restored').exists())

    def test_traversal_and_symlinks_rejected(self):
        (self.source / 'linked').symlink_to(self.source / 'session.json')
        with self.assertRaisesRegex(ValueError, 'symlink'):
            archive.create(self.source, self.destination)
        (self.source / 'linked').unlink()
        archive.create(self.source, self.destination)
        p = self.destination / 'manifest.json'
        m = json.loads(p.read_text()); m['files']['../outside'] = dict(bytes=0, sha256=archive.sha(b''))
        p.write_text(json.dumps(m))
        with self.assertRaisesRegex(ValueError, 'unsafe_archive_path'):
            archive.restore(self.destination, self.root / 'restored')
        self.assertFalse((self.root / 'outside').exists())

    def test_existing_destinations_and_active_sessions_preserved(self):
        self.destination.mkdir()
        with self.assertRaises(FileExistsError):
            archive.create(self.source, self.destination)
        self.destination.rmdir()
        (self.source / 'session.json').write_text(json.dumps(dict(
            schemaVersion='motionbrain.manual_collection.v1', phase='recording')))
        with self.assertRaisesRegex(ValueError, 'closed_manual'):
            archive.create(self.source, self.destination)
        self.assertFalse(self.destination.exists())

    def test_source_mutation_during_verification_aborts_publication(self):
        original = archive.verify
        def mutate(*args):
            result = original(*args)
            (self.source / 'new-evidence.json').write_text('{}')
            return result
        with patch.object(archive, 'verify', side_effect=mutate):
            with self.assertRaisesRegex(ValueError, 'source_changed'):
                archive.create(self.source, self.destination)
        self.assertFalse(self.destination.exists())
        self.assertTrue((self.source / 'new-evidence.json').exists())

    def test_missing_encoder_keeps_originals_and_cleans_stage(self):
        before = archive.inventory(self.source)
        with self.assertRaises(FileNotFoundError):
            archive.create(self.source, self.destination, ffmpeg='/no-such-encoder')
        self.assertEqual(archive.inventory(self.source), before)
        self.assertEqual(sorted(p.name for p in self.root.iterdir()), ['source'])

    def test_nonidentical_lzbitmap_reconstruction_is_rejected(self):
        before = archive.inventory(self.source)
        with patch.object(archive, 'encode', return_value=b'noncanonical'):
            with self.assertRaisesRegex(ValueError, 'hash_mismatch'):
                archive.create(self.source, self.destination)
        self.assertEqual(archive.inventory(self.source), before)
        self.assertFalse(self.destination.exists())

    def test_prune_and_in_place_recovery_preserve_all_original_bytes(self):
        before = archive.inventory(self.source)
        archive.create(self.source, self.destination)
        result = archive.prune(self.destination, self.source)
        self.assertEqual(result['removedFrames'], 3)
        self.assertEqual(archive.prune(self.destination, self.source)['removedFrames'], 0)
        remaining = archive.inventory(self.source)
        self.assertEqual(remaining, {n: r for n, r in before.items() if not n.endswith(archive.SUFFIX)})
        self.assertEqual(archive.restore_missing(self.destination, self.source)['restoredFrames'], 3)
        self.assertEqual(archive.inventory(self.source), before)
        self.assertEqual(archive.restore_missing(self.destination, self.source)['restoredFrames'], 0)

    def test_corruption_or_sync_failure_prevents_any_pruning(self):
        before = archive.inventory(self.source)
        archive.create(self.source, self.destination)
        with patch.object(archive, 'sync_archive', side_effect=OSError('disk sync failed')):
            with self.assertRaises(OSError):
                archive.prune(self.destination, self.source)
        self.assertEqual(archive.inventory(self.source), before)
        (self.destination / 'frames.mkv').write_bytes(b'corrupt')
        with self.assertRaisesRegex(ValueError, 'hash_mismatch'):
            archive.prune(self.destination, self.source)
        self.assertEqual(archive.inventory(self.source), before)

    def test_changed_metadata_or_frame_rejected_before_prune_and_restore(self):
        archive.create(self.source, self.destination)
        for path in [self.source / 'session.json', self.camera / 'frame-0000.bgra.lzbitmap']:
            original = path.read_bytes(); path.write_bytes(b'changed')
            for action in [archive.prune, archive.restore_missing]:
                with self.assertRaisesRegex(ValueError, 'source_differs'):
                    action(self.destination, self.source)
            self.assertEqual(path.read_bytes(), b'changed')
            path.write_bytes(original)

    def test_interrupted_pruning_resumes_and_restores_every_frame(self):
        before = archive.inventory(self.source)
        archive.create(self.source, self.destination)
        unlink = Path.unlink
        def interrupt(path, *args, **kwargs):
            if path.name == 'frame-0001.bgra.lzbitmap':
                raise OSError('interrupted deletion')
            return unlink(path, *args, **kwargs)
        with patch.object(Path, 'unlink', interrupt):
            with self.assertRaisesRegex(OSError, 'interrupted deletion'):
                archive.prune(self.destination, self.source)
        self.assertFalse((self.camera / 'frame-0000.bgra.lzbitmap').exists())
        self.assertTrue((self.camera / 'frame-0001.bgra.lzbitmap').exists())
        self.assertEqual(archive.prune(self.destination, self.source)['removedFrames'], 2)
        archive.restore_missing(self.destination, self.source)
        self.assertEqual(archive.inventory(self.source), before)

    def test_archive_mutation_during_verification_prevents_removal(self):
        before = archive.inventory(self.source)
        archive.create(self.source, self.destination)
        original = archive.verify
        def mutate(*args):
            result = original(*args)
            (self.destination / 'frames.mkv').write_bytes(b'changed')
            return result
        with patch.object(archive, 'verify', side_effect=mutate):
            with self.assertRaisesRegex(ValueError, 'archive_or_source_changed'):
                archive.prune(self.destination, self.source)
        self.assertEqual(archive.inventory(self.source), before)

    def test_concurrent_archive_operations_are_rejected(self):
        archive.create(self.source, self.destination)
        with archive.archive_lock(self.destination):
            with self.assertRaises(BlockingIOError):
                archive.prune(self.destination, self.source)

    def test_archive_sync_includes_every_ancestor_directory(self):
        archive.create(self.source, self.destination)
        with patch.object(archive, 'sync_directory') as sync:
            archive.sync_archive(self.destination)
        seen = {call.args[0] for call in sync.call_args_list}
        self.assertTrue(set(self.destination.parents).issubset(seen))

    def test_interrupted_in_place_recovery_can_resume(self):
        before = archive.inventory(self.source)
        archive.create(self.source, self.destination)
        archive.prune(self.destination, self.source)
        link = archive.os.link
        def interrupt(origin, target):
            if target.name == 'frame-0001.bgra.lzbitmap':
                raise OSError('interrupted recovery')
            return link(origin, target)
        with patch.object(archive.os, 'link', side_effect=interrupt):
            with self.assertRaisesRegex(OSError, 'interrupted recovery'):
                archive.restore_missing(self.destination, self.source)
        self.assertEqual(archive.restore_missing(self.destination, self.source)['restoredFrames'], 2)
        self.assertEqual(archive.inventory(self.source), before)
