import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools import run_dual_view_policy as runner


class CaptureStorageTest(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.repo = Path(temp.name).resolve()
        self.capture = self.repo / '.local' / 'live-captures'
        self.capture.mkdir(parents=True)
        patcher = patch.object(runner, 'ROOT', self.repo)
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_observe_locally_without_external_mount(self):
        with patch.object(runner, '_mount_fd', side_effect=AssertionError):
            with runner.capture_parent(dict(captureStorage='local', captureRoot=str(self.capture))) as root:
                self.assertEqual(root, self.capture)

    def test_local_rejects_redirected_roots(self):
        link = self.repo / '.local' / 'alias'
        link.symlink_to(self.capture)
        for path in (self.repo, self.repo / '.local', link):
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, 'local_capture_root_changed'):
                with runner.capture_parent(dict(captureStorage='local', captureRoot=str(path))):
                    self.fail()

    def test_external_default_checks_bound_volume(self):
        fd = os.open(self.repo, os.O_RDONLY)
        with patch.object(runner, '_mount_fd', return_value=fd) as check:
            with runner.capture_parent(dict(captureRoot=str(self.capture), mount=str(self.repo), volumeUuid='id')):
                check.assert_called_once_with(self.repo, 'id', self.repo)
        with self.assertRaises(OSError):
            os.fstat(fd)
