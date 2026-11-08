"""고정 ROI·같은 촬영 첫 기준과 실제 transport 원본의 결합을 검증한다.

기존 native byte/clock 감사는 기존 recorder/comparison 테스트가 담당한다.
이 파일은 그 읽기 경계만 합성 영상으로 대체하며 실제 카메라/소켓을 열지 않는다.
"""
import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from learning import arm_session_reference as ref
from learning.artifacts import ArtifactStore
from learning.supervised_collection import SupervisedCollection, plan_digest, validate_plan
from learning.supervised_transport import SequencePort
from tests import test_arm_motion_observation as observation_fixture
from tests.test_supervised_collection import Camera, plan as fixture_plan
from tests.test_supervised_transport import Clock, Controller


def load_cli(name):
    path = Path(__file__).resolve().parents[1] / 'tools' / name
    spec = importlib.util.spec_from_file_location(name.replace('.py', ''), path)
    cli = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cli)
    return cli


class SessionReferenceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        fixture = observation_fixture.ArmMotionObservationTest()
        fixture.setUp()
        a, b = fixture.frames(1.)
        self.images = []
        for image in (a, b):
            canvas = np.zeros((1080, 1920), dtype=np.uint8)
            canvas[:480, :640] = image
            self.images.append(canvas)
        region = dict(movingPolygon=fixture.moving, backgroundPolygons=fixture.backgrounds, forwardImageSign=-1)
        self.template = dict(schemaVersion=ref.TEMPLATE_SCHEMA, imageSize=[1920, 1080],
                             regions=dict(upper=region, forearm=region))
        self.raw = json.dumps(self.template).encode()
        self.clock = Clock()
        self.store = ArtifactStore(self.root / 'run')
        self.device = Controller(self.clock)
        self.device.boot = 'a' * 32
        self.port = SequencePort('http://127.0.0.1:8181', 'synthetic-unused', self.device.boot,
                                 self.store, clock=self.clock)
        self.port.opener = self.device
        self.plan = fixture_plan('shoulder', 'down', (350,), 35)
        self.plan['armReferenceTemplateSha256'] = ref.sha(self.raw)
        owner = self

        class SyntheticCamera(Camera):
            def start(self):
                super().start()
                self.frames.append(owner.frame(0, self.clock.monotonic()))

            def wait_ready(self, timeout):
                self.clock.sleep(.01)

        self.camera = SyntheticCamera(self.clock)

    def frame(self, index, at):
        native = self.store.root / 'native-camera'
        native.mkdir(exist_ok=True)
        metadata, pixels = b'synthetic-metadata-' + bytes([index]), b'synthetic-pixels-' + bytes([index])
        meta_name, image_name = f'frame-{index:04d}.json', f'frame-{index:04d}.bgra.lzbitmap'
        (native / meta_name).write_bytes(metadata)
        (native / image_name).write_bytes(pixels)
        return dict(frameIndex=index, frameId=f"{self.plan['sessionId']}:{index}",
                    metadataFile=meta_name, metadataSha256=ref.sha(metadata),
                    imageFile=image_name, imageSha256=ref.sha(pixels), pixelSha256=ref.sha(pixels),
                    width=1920, height=1080, sourceEarliest=at, sourceLatest=at + .001,
                    stdoutReadAt=at + .002, persistenceMs=1, convertedHostPtsSeconds=at,
                    qualityFlags=[])

    def native_image(self, ev, camera, frame):
        verified = {key: frame[key] for key in ('frameIndex', 'frameId', 'pixelSha256', 'width', 'height',
                                               'sourceEarliest', 'sourceLatest')}
        verified.update(videoMirrored=False, videoRotationAngle=0, configurationSha256='c' * 64,
                        persistedAt=frame['stdoutReadAt'] + .001)
        return self.images[min(frame['frameIndex'], 1)], verified

    def run_collection(self, registered=True):
        plan = copy.deepcopy(self.plan)
        if not registered:
            del plan['armReferenceTemplateSha256']
        with patch.object(ref, 'native_image', side_effect=self.native_image):
            runner = SupervisedCollection(plan, self.camera, self.port, self.store, clock=self.clock,
                                          arm_reference_template=self.raw if registered else None)
            report = runner.run(plan_digest(plan))
        if report['status'] == 'completed':
            # The fake camera has no reader thread; append one endpoint to its
            # synthetic final report after the real transport/collector completed.
            report['frames'].append(self.frame(1, report['trials'][0]['quietVerifiedUntil'] - .05))
            (self.store.root / 'report.json').write_text(json.dumps(report))
        return report, plan

    @contextlib.contextmanager
    def offline_reader(self):
        def audited_camera(ev, plan, report):
            return report['frames'], dict(viewId=plan['camera']['viewId'])
        with patch.object(ref, 'native_image', side_effect=self.native_image), \
             patch.object(ref, 'audit_camera', side_effect=audited_camera):
            yield

    def compare(self, plan, **kwargs):
        with self.offline_reader():
            return ref.compare_run(self.store.root, self.raw, plan_digest(plan), **kwargs)

    def test_template_schema_hash_and_regions_are_fixed(self):
        self.assertEqual(ref.load_template(self.raw, ref.sha(self.raw)), self.template)
        for mutate in (
            lambda t: t.update(extra=True),
            lambda t: t['regions']['upper'].update(forwardImageSign=True),
            lambda t: t['regions']['upper'].update(backgroundPolygons=[]),
            lambda t: t['regions']['upper']['backgroundPolygons'].__setitem__(0, t['regions']['upper']['movingPolygon']),
        ):
            bad = copy.deepcopy(self.template)
            mutate(bad)
            with self.assertRaises(ValueError):ref.load_template(json.dumps(bad).encode())
        with self.assertRaisesRegex(ValueError, 'hash_changed'):ref.load_template(self.raw, '0' * 64)
        with self.assertRaises(ValueError):ref.load_template(b'{"schemaVersion":0,"schemaVersion":1}')

    def test_optional_plan_pin_changes_digest_and_keeps_default_contract(self):
        self.assertEqual(validate_plan(self.plan), self.plan)
        default = copy.deepcopy(self.plan)
        del default['armReferenceTemplateSha256']
        self.assertNotEqual(plan_digest(default), plan_digest(self.plan))
        bad = copy.deepcopy(self.plan)
        bad['pulses'] *= 2
        with self.assertRaises(ValueError):validate_plan(bad)
        with self.assertRaisesRegex(ValueError, 'template_hash'):validate_plan(dict(self.plan, armReferenceTemplateSha256='bad'))

    def test_bad_template_fails_before_any_device_or_camera(self):
        with self.assertRaisesRegex(ValueError, 'hash_changed'):
            SupervisedCollection(self.plan, self.camera, self.port, self.store, clock=self.clock,
                                 arm_reference_template=self.raw + b' ')
        self.assertFalse(self.camera.started)
        self.assertEqual(self.device.requests, [])

    def test_baseline_quality_failure_preserves_source_and_starts_no_action(self):
        with patch.object(ref, 'baseline_quality', side_effect=ValueError('arm_reference_baseline_quality:upper:test')):
            report, _ = self.run_collection()
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['summary']['not_started'], 1)
        self.assertTrue(report['finalIdleOffVerified'])
        self.assertTrue(self.camera.closed)
        self.assertFalse(any(payload is not None for _, payload in self.device.requests))
        self.assertTrue((self.store.root / 'arm-reference-template.json').exists())
        self.assertTrue((self.store.root / 'native-camera/frame-0000.json').exists())
        self.assertFalse((self.store.root / 'arm-reference.json').exists())

    def test_reference_registration_and_real_http_audit(self):
        report, plan = self.run_collection()
        self.assertEqual(report['status'], 'completed')
        self.assertEqual(report['armReference']['frame']['frameIndex'], 0)
        self.assertEqual((self.store.root / 'arm-reference-template.json').read_bytes(), self.raw)
        result, _ = self.compare(plan)
        self.assertTrue(result['preActionReferenceVerified'])
        self.assertTrue(result['httpAudit']['fullTraceVerified'])
        self.assertEqual(result['sensorSamples'][0], result['reference']['firstFrameSensorSample'])
        self.assertEqual(result['summary'], dict(requested=2, valid=2, rejected=0))
        self.assertEqual(result['nativeFrames'][0]['frameIndex'], 0)
        self.assertGreater(result['regions']['upper']['observation']['projectedDeltaDeg'], .8)
        self.assertFalse(result['jointZeroCalibrated'])
        self.assertFalse(result['controlEligible'])

    def test_offline_registration_is_explicit_and_never_pre_action(self):
        _, plan = self.run_collection(registered=False)
        with self.assertRaisesRegex(ValueError, 'explicit_offline_registration'):self.compare(plan)
        result, _ = self.compare(plan, allow_offline_registration=True)
        self.assertFalse(result['preActionReferenceVerified'])
        self.assertEqual(result['reference']['mode'], 'offline_reconstruction_not_pre_action')
        self.assertIsNone(result['reference']['registeredAt'])
        self.assertFalse((self.store.root / 'arm-reference.json').exists())

    def test_receipt_and_run_off_brackets_cannot_be_relabelled(self):
        report, plan = self.run_collection()
        path = self.store.root / 'trial-001-result.json'
        trial = json.loads(path.read_text())
        trial['offAt'] += 10
        path.write_text(json.dumps(trial))
        with self.assertRaisesRegex(ValueError, 'receipt_report_changed'):self.compare(plan)
        report['trials'][0] = trial
        (self.store.root / 'report.json').write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'post_window|bracket'):self.compare(plan)

    def test_registration_cannot_be_moved_after_first_write(self):
        report, plan = self.run_collection()
        registration = report['armReference']
        registration['registeredAt'] = report['trials'][0]['dispatchBefore'] + .01
        receipt = self.store.root / 'arm-reference.json'
        receipt.write_text(json.dumps(registration))
        report['armReferenceSha256'] = ref.sha(receipt.read_bytes())
        (self.store.root / 'report.json').write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, 'registered_after_write'):self.compare(plan)

    def test_registration_changed_source_or_template_is_rejected(self):
        report, plan = self.run_collection()
        native = self.store.root / 'native-camera/frame-0000.bgra.lzbitmap'
        native.write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError, 'frame_bytes_changed'):self.compare(plan)
        (self.store.root / 'arm-reference-template.json').write_bytes(self.raw + b' ')
        with self.assertRaisesRegex(ValueError, 'saved_template_changed'):self.compare(plan)

    def test_first_frame_and_post_quiet_window_have_no_fallback(self):
        trial = dict(dispatchBefore=10., offAt=11., quietVerifiedUntil=14.04)
        before = dict(frameIndex=0, frameId='s:0', sourceEarliest=9.7, sourceLatest=9.8)
        after = dict(frameIndex=1, frameId='s:1', sourceEarliest=13.89, sourceLatest=13.91)
        tail = dict(frameIndex=2, frameId='s:2', sourceEarliest=14.05, sourceLatest=14.06)
        chosen, proof = ref.select_endpoints([before, after, tail], trial, 3., verified_quiet_end=14.04)
        self.assertEqual(chosen, [before, after])
        self.assertLess(proof['afterSourceElapsedBoundsSeconds'][1], 3.)
        with self.assertRaisesRegex(ValueError, 'endpoint_frames_missing'):
            ref.select_endpoints([before, tail], trial, 3., verified_quiet_end=14.04)
        with self.assertRaisesRegex(ValueError, 'endpoint_frames_missing'):
            ref.select_endpoints([dict(before, sourceLatest=10.), after], trial, 3., verified_quiet_end=14.04)

    def test_missing_nearby_first_frame_sensor_starts_no_action(self):
        original_start = self.camera.start
        def late_start():
            self.clock.sleep(1.)
            original_start()
        self.camera.start = late_start
        self.camera.wait_ready = lambda timeout: self.clock.sleep(2.)
        report, _ = self.run_collection()
        self.assertEqual(report['status'], 'failed')
        self.assertEqual(report['summary']['not_started'], 1)
        self.assertIn('sensor_frame_time_gap_exceeded', report['errors'][0]['reason'])
        self.assertFalse(any(payload is not None for _, payload in self.device.requests))
        self.assertTrue(self.camera.closed)

    def test_quiet_receipt_cannot_extend_raw_http_verified_window(self):
        report, plan = self.run_collection()
        _, _, proof = ref.audit_trial_http(ref.PinnedEvidence(self.store.root), report, plan)
        trial = report['trials'][0]
        trial['quietVerifiedUntil'] += .2
        trial['postEnd'] = trial['quietVerifiedUntil']
        (self.store.root / 'report.json').write_text(json.dumps(report))
        (self.store.root / 'trial-001-result.json').write_text(json.dumps(trial))
        with self.assertRaisesRegex(ValueError, 'quiet_receipt_http_bracket_changed'):
            self.compare(plan)
        # Even a receipt bound later than a frame cannot authorize that frame
        # beyond the independent raw output-read end bound.
        with self.assertRaisesRegex(ValueError, 'endpoint_frames_missing'):
            ref.select_endpoints([
                dict(frameIndex=0, frameId='s:0', sourceEarliest=9.7, sourceLatest=9.8),
                dict(frameIndex=1, frameId='s:1', sourceEarliest=14.05, sourceLatest=14.06)],
                dict(dispatchBefore=10., offAt=11., quietVerifiedUntil=14.4), 3., verified_quiet_end=14.04)
        self.assertLess(proof['quietOutputsVerifiedThrough'], trial['quietVerifiedUntil'])

    def test_repeated_source_reads_cannot_replace_pinned_hash(self):
        path = self.store.root / 'one.json'
        path.write_bytes(b'{}')
        evidence = ref.PinnedEvidence(self.store.root)
        evidence.raw('one.json')
        path.write_bytes(b'{"changed":true}')
        with self.assertRaisesRegex(ValueError, 'source_changed_between_reads'):
            evidence.raw('one.json')

    def test_template_cli_pins_raw_bytes_without_capture_or_http(self):
        cli = load_cli('collect_supervised_actions.py')
        helper = self.root / 'helper'
        helper.write_bytes(b'synthetic')
        template = self.root / 'template.json'
        template.write_bytes(self.raw)
        path = self.root / '.local/plan.json'
        args = ['prepare', '--helper', str(helper), '--output-plan', str(path),
                '--arm-reference-template', str(template), '--joint', 'shoulder', '--direction', 'down',
                '--percent', '35', '--durations-ms', '350', '--session-id', 'session',
                '--capture-group', 'group', '--device-id', 'synthetic', '--view-id', 'side',
                '--pose-context', 'middle', '--boot-id', 'a' * 32, '--controller-url', 'http://127.0.0.1']
        with patch.object(cli, 'ROOT', self.root), patch('subprocess.Popen') as capture, \
             patch('urllib.request.OpenerDirector.open') as http, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(args), 0)
            saved = json.loads(path.read_text())
            self.assertEqual(saved['armReferenceTemplateSha256'], ref.sha(self.raw))
            self.assertEqual(cli.main(['validate', '--plan', str(path), '--helper', str(helper),
                                       '--arm-reference-template', str(template)]), 0)
        capture.assert_not_called()
        http.assert_not_called()

    def test_comparison_cli_writes_new_private_artifacts_only(self):
        _, plan = self.run_collection()
        cli = load_cli('compare_supervised_arm_video.py')
        # Move the synthetic run under the CLI's permitted private tree.
        private = self.root / '.local'
        private.mkdir()
        run = private / 'input'
        self.store.root.rename(run)
        self.store.root = run
        template = self.root / 'template.json'
        template.write_bytes(self.raw)
        output = private / 'output'
        args = ['--run', str(run), '--output', str(output), '--arm-reference-template', str(template),
                '--expected-template-sha256', ref.sha(self.raw), '--expected-plan-sha256', plan_digest(plan)]
        with patch.object(cli, 'ROOT', self.root), self.offline_reader(), \
             patch('subprocess.Popen') as capture, patch('urllib.request.OpenerDirector.open') as http, \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(args), 0)
            self.assertTrue((output / 'comparison.json').exists())
            self.assertTrue((output / 'before.png').exists())
            with self.assertRaisesRegex(ValueError, 'new_separate_output'):cli.main(args)
            with self.assertRaisesRegex(ValueError, 'canonical_private_path'):
                cli.main([*args[:3], str(self.root / 'public'), *args[4:]])
        capture.assert_not_called()
        http.assert_not_called()
