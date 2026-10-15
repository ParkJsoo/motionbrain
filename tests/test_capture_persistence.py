"""Per-frame parallel persistence: durability barriers, failure and no next-frame queue."""
import json
import subprocess
import sys
import textwrap
import threading
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from learning.artifacts import ArtifactStore, StorageError, read_events, verify_artifacts
from learning.fast_probe import run_fast_probe
import test_fast_observation as fixtures


class PreparationTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory(); self.addCleanup(tmp.cleanup)
        self.base = Path(tmp.name); self.store = ArtifactStore(self.base/'run')

    def test_preparation_is_empty_idempotent_and_not_a_completed_capture(self):
        self.store.prepare_parent('images/0/source.jpg')
        self.store.prepare_parent('images/0/source.jpg')
        self.assertEqual(list((self.store.root/'images/0').iterdir()), [])
        self.assertEqual(read_events(self.store.root), ([], []))
        ref = self.store.save_image('0', 'source', fixtures.jpeg(), fixtures.metadata(fixtures.jpeg())['imageSha256'])
        self.assertEqual(verify_artifacts(self.store.root, [ref]), [])
        with self.assertRaises(StorageError):
            self.store.save_image('0', 'source', fixtures.jpeg(), ref['sha256'])

    def test_traversal_symlink_and_directory_fsync_failure_fail_closed(self):
        (self.store.root/'link').symlink_to(self.base, target_is_directory=True)
        for path in ('../outside/file', '/absolute/file', 'link/file'):
            with self.subTest(path=path), self.assertRaises(StorageError):
                self.store.prepare_parent(path)
        with patch('learning.artifacts.os.fsync', side_effect=OSError('fixture sync error')):
            with self.assertRaises(StorageError): self.store.prepare_parent('new/file.json')


class PersistenceTest(unittest.TestCase):
    setUp = fixtures.FastProbeTest.setUp
    run_probe = fixtures.FastProbeTest.run_probe

    def test_receipt_and_image_overlap_callbacks_stay_on_caller_and_join_before_measurement(self):
        image_entered, receipt_entered = threading.Event(), threading.Event()
        original_image, original_json = ArtifactStore.save_image, ArtifactStore.write_json
        caller = threading.get_ident()
        callback_threads = []
        self.transport.recorder = lambda _: callback_threads.append(threading.get_ident())
        def image(store, index, phase, body, digest):
            if index != 'anchor':
                image_entered.set()
                self.assertTrue(receipt_entered.wait(2))
            return original_image(store, index, phase, body, digest)
        def write(store, path, data):
            if data.get('path') == '/capture':
                self.assertTrue(image_entered.wait(2))
                receipt_entered.set()
            if path.startswith('measurements/'):
                index = int(Path(path).stem)
                self.assertTrue((store.root/f'images/{index}/source.jpg').exists())
            return original_json(store, path, data)
        with patch.object(ArtifactStore, 'save_image', image), patch.object(ArtifactStore, 'write_json', write):
            report = self.run_probe()
        self.assertEqual(report['status'], 'completed', report)
        self.assertEqual(set(callback_threads), {caller})
        self.assertFalse(any(t.name.startswith('capture-image') for t in threading.enumerate()))
        self.assertEqual(verify_artifacts(self.root, report['artifacts']), [])

    def test_receipt_failure_still_joins_and_retains_current_original(self):
        original = ArtifactStore.write_json
        def fail(store, path, data):
            if data.get('path') == '/capture': raise OSError('receipt write failed')
            return original(store, path, data)
        with patch.object(ArtifactStore, 'write_json', fail): report = self.run_probe()
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['completedSamples'], 0)
        self.assertEqual(self.capture_count, 1)
        self.assertEqual((self.root/'images/0/source.jpg').read_bytes(), self.body)
        self.assertTrue(any(r['artifactRef']=='images/0/source.jpg' for r in report['artifacts']))
        self.assertFalse(any(t.name.startswith('capture-image') for t in threading.enumerate()))

    def test_receipt_interrupt_retains_image_and_uncompleted_capture_attempt(self):
        original = ArtifactStore.write_json
        def stop(store, path, data):
            if data.get('path') == '/capture': raise KeyboardInterrupt()
            return original(store, path, data)
        with patch.object(ArtifactStore, 'write_json', stop): report = self.run_probe()
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(report['completedSamples'], 0)
        self.assertTrue((self.root/'images/0/source.jpg').exists())
        kinds = [e['kind'] for e in read_events(self.root)[0]]
        self.assertEqual(kinds, ['started', 'capture_requested', 'finished'])

    def test_image_interrupt_retains_receipt_and_never_starts_next_capture(self):
        original = ArtifactStore.save_image
        def stop(store, index, phase, body, digest):
            if index != 'anchor': raise KeyboardInterrupt()
            return original(store, index, phase, body, digest)
        with patch.object(ArtifactStore, 'save_image', stop): report = self.run_probe()
        self.assertEqual(report['status'], 'interrupted')
        self.assertEqual(self.capture_count, 1)
        receipts = [json.loads(p.read_text()) for p in (self.root/'raw').glob('*.json')]
        self.assertEqual(sum(r.get('path')=='/capture' for r in receipts), 1)
        self.assertFalse(any(t.name.startswith('capture-image') for t in threading.enumerate()))

    def test_preparation_failure_is_before_capture(self):
        with patch.object(ArtifactStore, 'prepare_parent', side_effect=StorageError('fixture')):
            report = self.run_probe()
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(self.capture_count, 0)
        self.assertEqual(report['motorCommandsSent'], 0)

    def test_serial_reference_preserves_measurement_and_image_bytes(self):
        first = self.run_probe(parallel_persistence=False)
        old_root = self.root
        self.setUp()
        second = self.run_probe()
        self.assertEqual(first['results'], second['results'])
        for i in range(2):
            self.assertEqual((old_root/f'images/{i}/source.jpg').read_bytes(),
                             (self.root/f'images/{i}/source.jpg').read_bytes())
        self.assertEqual(second['motorCommandsSent'], 0)
        self.assertEqual(first['acceptedMeasurements'], second['acceptedMeasurements'])

    def test_real_sigint_during_future_wait_collects_completed_image(self):
        # Signal only this isolated child, never the test runner or hardware.
        script = r"""
import hashlib, json, os, signal, tempfile, threading, time
from pathlib import Path
from unittest.mock import patch
from learning.artifacts import ArtifactStore, verify_artifacts
from learning.fast_probe import _persist_capture
with tempfile.TemporaryDirectory() as temp:
    store = ArtifactStore(Path(temp)/'run')
    body = b'original bytes'
    receipt = {'status':200, 'sha256':hashlib.sha256(body).hexdigest()}
    artifacts = []
    image_started = threading.Event()
    original = store.save_image
    def slow(*args):
        image_started.set()
        time.sleep(.2)
        return original(*args)
    timers = []
    def record(_):
        assert image_started.wait(2)
        timer = threading.Timer(.03, lambda: os.kill(os.getpid(), signal.SIGINT))
        timers.append(timer); timer.start()
    try:
        with patch.object(store, 'save_image', slow):
            _persist_capture(store, 0, receipt, body, [receipt], record, artifacts, clock=time, parallel=True)
        raise AssertionError('interruption was lost')
    except KeyboardInterrupt:
        pass
    for timer in timers: timer.join()
    assert len(artifacts)==1, artifacts
    assert artifacts[0]['artifactRef']=='images/0/source.jpg'
    assert not verify_artifacts(store.root, artifacts)
    assert not any(t.name.startswith('capture-image') for t in threading.enumerate())
    print('verified')
"""
        for receipt_failure in (False, True):
            scenario = script
            if receipt_failure:
                scenario = scenario.replace('timers.append(timer); timer.start()',
                                            "timers.append(timer); timer.start(); raise OSError('receipt failure')")
                scenario = scenario.replace('except KeyboardInterrupt:', 'except OSError:')
            result = subprocess.run([sys.executable, '-c', textwrap.dedent(scenario)],
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), 'verified')

    def test_non_boolean_parallel_option_rejected(self):
        with self.assertRaises(ValueError): self.run_probe(parallel_persistence=1)
