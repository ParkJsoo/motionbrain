"""사후 대조에서 시각 간격·변조·출처 혼합·탈락 분모를 보존한다."""
import copy
import importlib.util
import io
import json
import shutil
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.arm_motion_comparison import (SCHEMA, LEARNING_SHORT_SCHEMA, compare_manifest,
                                            nearest_sensor, sensor_delta, sha, pinned_frame, _native)
from learning.supervised_collection import plan_digest


class MotionComparisonTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        (self.root/'learning').mkdir()
        source_root=Path(__file__).resolve().parents[1]
        for name in ['arm_motion_observation.py','arm_motion_comparison.py']:
            shutil.copyfile(source_root/'learning'/name,self.root/'learning'/name)
        self.case = self.root / '.local/case'; self.case.mkdir(parents=True)
        self.scope = dict(schemaVersion=LEARNING_SHORT_SCHEMA, context={'view': 'synthetic'},
                          controllerBootId='a'*32)
        self.result = dict(schemaVersion=LEARNING_SHORT_SCHEMA, scopeSha256=plan_digest(self.scope))
        (self.case/'scope.json').write_text(json.dumps(self.scope))
        self.raw = json.dumps(self.result).encode(); (self.case/'result.json').write_bytes(self.raw)
        self.manifest = dict(schema=SCHEMA, regions=dict(imageSize=[640, 480],
            movingPolygon=[[10,10],[80,10],[80,80],[10,80]],
            backgroundPolygons=[[[100,10],[150,10],[150,80],[100,80]],[[200,10],[250,10],[250,80],[200,80]]],
            forwardImageSign=1), referenceView=dict(contextSha256=plan_digest(self.scope['context']),
            controllerBootId='a'*32, configurationSha256='b'*64, videoMirrored=False, videoRotationAngle=0),
            cases=[dict(root='.local/case', resultSha256=sha(self.raw))])

    def test_nearest_is_host_time_diagnostic_not_optical_synchronization(self):
        rows=[dict(at=10., angleDeg=194.),dict(at=10.3,angleDeg=195.)]
        out=nearest_sensor(rows,10.25)
        self.assertEqual(out['angleDeg'],195.)
        self.assertAlmostEqual(out['hostTimeGapSeconds'],.05)
        self.assertFalse(out['opticalSynchronizationVerified'])
        with self.assertRaisesRegex(ValueError,'time_gap'):nearest_sensor(rows,12.)

    def test_bad_times_angles_and_wrap_are_not_silently_interpolated(self):
        for rows in [[],[dict(at=True,angleDeg=194)], [dict(at=1,angleDeg=float('nan'))],
                     [dict(at=2,angleDeg=194),dict(at=1,angleDeg=195)],
                     [dict(at=1,angleDeg=194),dict(at=1,angleDeg=195)]]:
            with self.assertRaises(ValueError):nearest_sensor(rows,1)
        with self.assertRaisesRegex(ValueError,'wrap'):
            sensor_delta({'angleDeg':359.},{'angleDeg':1.})

    def test_changed_result_is_a_rejected_case_not_removed_from_denominator(self):
        m=copy.deepcopy(self.manifest);m['cases'][0]['resultSha256']='0'*64
        r=compare_manifest(m,self.root)
        self.assertEqual(r['summary']['requested'],1)
        self.assertEqual(r['summary']['accepted'],0)
        self.assertEqual(r['summary']['rejected'],1)
        self.assertEqual(r['cases'][0]['rejectedReason'],'result_hash_changed')
        self.assertFalse(r['m4DirectionCalibrated']);self.assertFalse(r['controlEligible'])

    def test_wrong_camera_context_or_controller_boot_fails_before_native_reader(self):
        for field in ['contextSha256','controllerBootId']:
            m=copy.deepcopy(self.manifest);m['referenceView'][field]='0'*64
            with patch('learning.arm_motion_comparison._native',side_effect=AssertionError('must not read')):
                r=compare_manifest(m,self.root)
            self.assertEqual(r['cases'][0]['rejectedReason'],'comparison_context_or_boot_changed')

    def test_private_path_symlink_escape_and_missing_file_are_retained(self):
        for path in ['../public',str(self.case),'.local/missing']:
            m=copy.deepcopy(self.manifest);m['cases'][0]['root']=path
            self.assertEqual(compare_manifest(m,self.root)['summary']['rejected'],1)
        (self.root/'.local/escape').symlink_to(self.root, target_is_directory=True)
        m=copy.deepcopy(self.manifest);m['cases'][0]['root']='.local/escape'
        self.assertEqual(compare_manifest(m,self.root)['cases'][0]['rejectedReason'],'private_case_path_required')
        (self.case/'result.json').unlink()
        (self.case/'result.json').symlink_to(self.case/'scope.json')
        self.assertEqual(compare_manifest(self.manifest,self.root)['summary']['rejected'],1)

    def test_duplicate_cases_and_unbounded_series_are_rejected(self):
        m=copy.deepcopy(self.manifest);m['cases']*=2
        with self.assertRaisesRegex(ValueError,'duplicate'):compare_manifest(m,self.root)
        for stride in [True,-1,33]:
            with self.assertRaisesRegex(ValueError,'stride'):compare_manifest(self.manifest,self.root,series_stride=stride)

    def test_alias_paths_duplicate_hashes_and_logical_episodes_cannot_inflate_count(self):
        (self.root/'.local/alias').symlink_to(self.case, target_is_directory=True)
        for alias in ['.local/case/.','.local/case/','.local/alias']:
            m=copy.deepcopy(self.manifest)
            m['cases'].append(dict(root=alias,resultSha256='1'*64))
            with self.assertRaisesRegex(ValueError,'duplicate_case'):compare_manifest(m,self.root)
        other=self.root/'.local/other';other.mkdir()
        m=copy.deepcopy(self.manifest);m['cases'].append(dict(root='.local/other',resultSha256=sha(self.raw)))
        with self.assertRaisesRegex(ValueError,'duplicate_case'):compare_manifest(m,self.root)
        m['cases'][1]['resultSha256']='1'*64
        row=dict(id='same-trial',episodeIdentity=['a'*32,123],valid=False,newPhysicalCommands=0)
        with patch('learning.arm_motion_comparison.audit_case',return_value=row):
            with self.assertRaisesRegex(ValueError,'duplicate_physical_episode'):compare_manifest(m,self.root)

    def test_cli_private_new_output_preserves_rejections(self):
        path=Path(__file__).resolve().parents[1]/'tools/compare_arm_video_sensor.py'
        spec=importlib.util.spec_from_file_location('motion_comparison_cli',path)
        cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
        manifest=self.root/'manifest.json';manifest.write_text(json.dumps(self.manifest))
        output=self.root/'.local/output'
        with patch.object(cli,'ROOT',self.root),redirect_stderr(io.StringIO()),redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit):cli.main(['--manifest',str(manifest),'--output',str(self.root/'public')])
            self.assertEqual(cli.main(['--manifest',str(manifest),'--output',str(output)]),1)
            report=json.loads((output/'comparison.json').read_text())
            self.assertEqual(report['summary']['rejected'],1)
            with self.assertRaises(SystemExit):cli.main(['--manifest',str(manifest),'--output',str(output)])

    def test_endpoint_missing_duplicate_and_changed_time_cannot_escape_pinned_result(self):
        frame=dict(frameIndex=1,frameId='synthetic:1',sourceEarliest=10.,sourceLatest=10.001)
        self.assertEqual(pinned_frame([frame],1,'synthetic:1'),frame)
        for frames in [[],[frame,frame],[dict(frame,frameIndex=2,frameId='synthetic:2')]]:
            with self.assertRaises(ValueError):pinned_frame(frames,1,'synthetic:1')
        shifted=dict(frame,sourceEarliest=10.01,sourceLatest=10.011)
        source=dict(root='external',ingressFile='host-ingress/0001.json')
        with patch('learning.arm_motion_comparison._read',return_value=json.dumps(shifted).encode()), \
             patch('learning.arm_motion_comparison._external_frame') as external:
            with self.assertRaisesRegex(ValueError,'native_summary_frame_changed'):
                _native(self.case,source,self.scope,expected=frame)
            external.assert_not_called()


if __name__=='__main__':unittest.main()
