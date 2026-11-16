import hashlib
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore, StorageError, _read
from learning.external_media import external_media_reads


class ExternalMediaTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name).resolve()
        self.local = self.base/'local'
        self.mount = self.base/'external'
        self.local.mkdir()
        (self.mount/'payload/session').mkdir(parents=True)
        (self.local/'session').mkdir()
        self.raw = b'bound pixels'
        self.target = self.mount/'payload/session/frame.bgra'
        self.target.write_bytes(self.raw)
        self.alias = self.local/'session/frame.bgra'
        self.alias.symlink_to(self.target)
        self.receipt = self.base/'receipt.json'
        self.data = dict(schemaVersion='motionbrain.external_media_receipt.v1',
                         logicalRoot=str(self.local), mountPoint=str(self.mount),
                         volumeUuid='test-volume', payloadPrefix='payload',
                         files={'session/frame.bgra': dict(bytes=len(self.raw),
                               sha256=hashlib.sha256(self.raw).hexdigest())})
        self.write_receipt()
        # Only OS volume identification is mocked; all FD/path/hash reads are real.
        self.mock_mount = patch('learning.external_media._mount_fd',
                                side_effect=lambda mount, uuid, logical: os.open(mount, os.O_RDONLY|os.O_DIRECTORY))
        self.mock_mount.start()
        self.addCleanup(self.mock_mount.stop)

    def write_receipt(self):
        self.receipt.write_text(json.dumps(self.data))
        self.digest = hashlib.sha256(self.receipt.read_bytes()).hexdigest()

    def context(self):
        return external_media_reads(self.receipt, self.digest, 'test-volume')

    def test_default_still_rejects_link_and_context_is_scoped(self):
        with self.assertRaises(StorageError): _read(self.local/'session', 'frame.bgra')
        with self.context():
            self.assertEqual(_read(self.local/'session', 'frame.bgra'), self.raw)
            with self.context(): self.assertEqual(_read(self.local, 'session/frame.bgra'), self.raw)
        with self.assertRaises(StorageError): _read(self.local/'session', 'frame.bgra')

    def test_same_size_and_mtime_corruption_is_rejected(self):
        before = self.target.stat()
        self.target.write_bytes(b'wrong pixels')
        os.utime(self.target, ns=(before.st_atime_ns, before.st_mtime_ns))
        with self.context(), self.assertRaises(StorageError):
            _read(self.local, 'session/frame.bgra')

    def test_changed_alias_target_is_rejected(self):
        other = self.mount/'other.bgra';other.write_bytes(self.raw)
        self.alias.unlink();self.alias.symlink_to(other)
        with self.context(), self.assertRaises(StorageError):
            _read(self.local, 'session/frame.bgra')

    def test_intermediate_and_root_links_remain_rejected(self):
        middle = self.local/'middle';middle.symlink_to(self.local/'session', target_is_directory=True)
        with self.context():
            for root, relative in [(self.local, 'middle/frame.bgra'), (middle, 'frame.bgra')]:
                with self.assertRaises(StorageError): _read(root, relative)

    def test_external_intermediate_link_is_rejected(self):
        actual = self.mount/'actual';self.target.parent.rename(actual)
        (self.mount/'payload/session').symlink_to(actual, target_is_directory=True)
        with self.context(), self.assertRaises(StorageError):
            _read(self.local, 'session/frame.bgra')

    def test_unregistered_leaf_and_metadata_not_allowed(self):
        other = self.local/'session/other.bgra';other.symlink_to(self.target)
        with self.context(), self.assertRaises(StorageError): _read(self.local, 'session/other.bgra')
        self.data['files']={'session/state.json':dict(bytes=len(self.raw),sha256=hashlib.sha256(self.raw).hexdigest())}
        self.write_receipt()
        with self.assertRaises(ValueError):
            with self.context(): pass

    def test_receipt_and_volume_binding_are_required(self):
        self.receipt.write_text(self.receipt.read_text()+' ')
        with self.assertRaises(ValueError):
            with self.context(): pass
        self.write_receipt()
        with self.assertRaises(ValueError):
            with external_media_reads(self.receipt,self.digest,'different-volume'): pass

    def test_writer_does_not_follow_registered_alias(self):
        store = ArtifactStore(self.local/'writer')
        target = self.mount/'payload/writer/frame.bgra'
        target.parent.mkdir()
        target.write_bytes(self.raw)
        (self.local/'writer/frame.bgra').symlink_to(target)
        self.data['files']['writer/frame.bgra'] = dict(self.data['files']['session/frame.bgra'])
        self.write_receipt()
        with self.context(), self.assertRaises(StorageError):
            store._write('frame.bgra', b'replacement')
        self.assertEqual(target.read_bytes(), self.raw)

    def test_ordinary_local_metadata_still_reads(self):
        (self.local/'session/state.json').write_bytes(b'{"ok":true}')
        with self.context():
            self.assertEqual(_read(self.local/'session','state.json'),b'{"ok":true}')

    def test_wrong_size_and_unsafe_receipt_paths_rejected(self):
        self.target.write_bytes(self.raw+b'x')
        with self.context(), self.assertRaises(StorageError): _read(self.local,'session/frame.bgra')
        self.data['files']['../frame.bgra']=self.data['files'].pop('session/frame.bgra');self.write_receipt()
        with self.assertRaises(ValueError):
            with self.context(): pass


class VolumeBindingTests(unittest.TestCase):
    def test_os_volume_identity_and_same_filesystem_rejected(self):
        import plistlib
        from learning.external_media import _mount_fd
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory).resolve()
            mount, local = base/'external', base/'local'
            mount.mkdir();local.mkdir()
            good = dict(VolumeUUID='expected', MountPoint=str(mount), Internal=False)
            for changed in [dict(VolumeUUID='wrong'), dict(MountPoint='/different'), dict(Internal=True)]:
                payload = dict(good, **changed)
                with self.subTest(changed=changed), patch('learning.external_media.subprocess.check_output', return_value=plistlib.dumps(payload)):
                    with self.assertRaisesRegex(ValueError, 'media_volume_identity_changed'):
                        _mount_fd(mount, 'expected', local)
            with patch('learning.external_media.subprocess.check_output', return_value=plistlib.dumps(good)):
                with self.assertRaisesRegex(ValueError, 'media_volume_not_external'):
                    _mount_fd(mount, 'expected', local)
            alias = base/'alias';alias.symlink_to(mount, target_is_directory=True)
            with self.assertRaisesRegex(ValueError, 'media_mount_symlink'):
                _mount_fd(alias, 'expected', local)
