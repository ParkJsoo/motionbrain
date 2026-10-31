"""Validate local exporter calls with a recording SDK double, no installation."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from learning.visual_action_examples import run_synthetic
from learning.visual_action_collection import audit_collection
from learning.visual_action_rerun import export_recording


class RerunExportTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name);run_synthetic(self.root/'source')
        self.sdk=Mock();self.rec=self.sdk.RecordingStream.return_value
    def test_local_save_uses_one_host_timeline_and_records_command_results(self):
        target=self.root/'rrd/view.rrd'
        with patch('socket.socket',side_effect=AssertionError('network forbidden')):
            r=export_recording(self.root/'source',target,sdk=self.sdk)
        self.assertFalse(r['externalUpload']);self.rec.save.assert_called_once_with(str(target))
        self.rec.connect.assert_not_called();self.sdk.spawn.assert_not_called()
        self.rec.flush.assert_called_once();self.rec.disconnect.assert_called_once()
        self.assertTrue(all(c.args==('host_ingress',) for c in self.rec.set_time.call_args_list))
        paths=[c.args[0] for c in self.rec.log.call_args_list]
        self.assertIn('camera/image',paths);self.assertIn('camera/image/points',paths);self.assertIn('action/events',paths)
    def test_integrity_failure_creates_no_sink(self):
        next((self.root/'source/raw').glob('*.json')).write_text('{}')
        with self.assertRaises(ValueError):export_recording(self.root/'source',self.root/'rrd/view.rrd',sdk=self.sdk)
        self.sdk.RecordingStream.assert_not_called()
    def test_existing_output_is_preserved(self):
        target=self.root/'view.rrd';target.write_bytes(b'original')
        with self.assertRaises(FileExistsError):export_recording(self.root/'source',target,sdk=self.sdk)
        self.assertEqual(target.read_bytes(),b'original')
    def test_preserved_but_rejected_frames_create_no_sink(self):
        for scenario in ('corrupt_frame','missing_frame'):
            with self.subTest(scenario=scenario):
                source=self.root/scenario;run_synthetic(source,scenario)
                audit=audit_collection(source)
                self.assertTrue(audit['integrityOk'])
                self.assertTrue(audit['summary']['errors'])
                target=self.root/(scenario+'-rrd')/'view.rrd'
                with self.assertRaisesRegex(ValueError,'validated_timeline_required'):
                    export_recording(source,target,sdk=self.sdk)
                self.sdk.RecordingStream.assert_not_called()
                self.assertFalse(target.parent.exists())
    def test_valid_failed_action_remains_replayable(self):
        source=self.root/'unknown';run_synthetic(source,'unknown_result')
        result=export_recording(source,self.root/'unknown-rrd/view.rrd',sdk=self.sdk)
        self.assertFalse(result['recordingComplete'])
        self.rec.save.assert_called_once()
        self.rec.disconnect.assert_called_once()
