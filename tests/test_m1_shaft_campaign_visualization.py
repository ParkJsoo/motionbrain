"""합성·저장 파일만으로 campaign 표시와 사전 예측 증거를 검사한다."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import m1_shaft_learning as learning
from tests.test_m1_shaft_learning import scalar_sample
from tests.test_grasp_observation import Camera, Clock
from tools.visualization import m1_shaft_campaign as m


class CampaignViewTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name); self.local = self.root/'.local'; self.runtime = self.local/'runtime'
        self.runtime.mkdir(parents=True); self.source = self.local/'session'; self.pair = self.source/'gripper-pair'; self.pair.mkdir(parents=True)
        self.patch = patch.object(m, 'ROOT', self.root); self.patch.start(); self.addCleanup(self.patch.stop)
        self.session = b'{"phase":"completed"}'; (self.source/'session.json').write_bytes(self.session)
        self.source_sha = hashlib.sha256(self.session).hexdigest()
        clock = Clock(); camera = Camera(clock)
        from learning.gripper_camera_dataset import read_frame
        self.frames = {}
        for side in ('before', 'after'):
            import urllib.request
            response = camera.open(urllib.request.Request('http://unused.invalid'), 1)
            metadata = dict(sessionId='synthetic', hostClockId='synthetic', clockDomain='python_monotonic',
                requestStartedAt=1., receivedAt=2., persistedAt=3., headers=response.headers,
                sha256=hashlib.sha256(response.body).hexdigest())
            (self.pair/f'esp32-{side}.jpg').write_bytes(response.body)
            self.write(self.pair/f'esp32-{side}.json', metadata)
            self.frames[side] = read_frame(self.pair, side, 'synthetic')[0]
        self.audit = dict(source=dict(path=str(self.source), sha256=self.source_sha),
            visual=dict(status='verified', row=dict(sessionId='synthetic', evidence=dict(commandAt=20.))),
            execution=dict(status='verified', offVerified=True))
        self.plan = dict(steps=[dict(phase='preparation',command=dict(joint='gripper',direction='open',percent=40,durationMs=350)),dict(phase='evaluation')], sourceHashes={'historical-code.py':'a'*64})
        self.write(self.runtime/'plan.json', self.plan)

    def write(self, path, value): path.write_text(json.dumps(value))

    def prospective_record(self):
        model = learning.fit([scalar_sample('s1','train',40.),scalar_sample('s2','train',60.)])
        row = scalar_sample('eval', 'eval', 55.); row.update(sourceHash=self.source_sha,
            sessionId='synthetic', frameIds=[self.frames['before']['frameId'],self.frames['after']['frameId']])
        self.audit['visual']['row'].update(inputs=dict(command=copy.deepcopy(row['command'])))
        self.audit['visual']['row']['evidence'].update(controllerBootId=row['episode'][0], commandId=row['episode'][1])
        p = dict(preparedAt=10., modelSha256=learning.digest(model), command=row['command'], strictRejection=None,
            frame=dict(frame=self.frames['before']), shaftSeparationPx=55., noChange=55.,
            shaftRidge=learning.predict(model,55.,row['command'],row['context']),
            commandMean=learning.predict(model,55.,row['command'],row['context'],baseline=True),
            observation=dict(pixelSha256=self.frames['before']['pixelSha256'],state=[55.],shaftSeparationPx=55.,
                annotation=dict(frameId=self.frames['before']['frameId'],pixelSha256=self.frames['before']['pixelSha256'])))
        record = dict(phase='evaluation',status='completed',transition=row,rawAudit=self.audit,prospective=p,
            observation=dict(perFrame={s:dict(self.frames[s],shaftSeparationPx=55. if s=='before' else row['afterShaftPx']) for s in self.frames}))
        return model, record

    def test_prospective_proof_requires_time_identity_state_hash_prediction_and_no_overlap(self):
        model, record = self.prospective_record(); frames={s:dict(status='verified') for s in self.frames}
        self.assertTrue(m._prospective(record,model,learning.digest(model),frames)['prospectiveVerified'])
        mutations = [lambda d:d['prospective'].update(preparedAt=21.),
            lambda d:d['prospective']['observation'].update(pixelSha256='bad'),
            lambda d:d['prospective']['observation']['annotation'].update(frameId='bad'),
            lambda d:d['prospective'].update(shaftSeparationPx=54.),
            lambda d:d['prospective'].update(modelSha256='bad'),
            lambda d:d['prospective'].update(shaftRidge=999.),
            lambda d:d['rawAudit']['visual']['row']['inputs']['command'].update(percent=39),
            lambda d:d['transition'].update(captureGroup='train')]
        for mutate in mutations:
            bad=copy.deepcopy(record);mutate(bad)
            with self.subTest(mutate=mutate), self.assertRaises(ValueError): m._prospective(bad,model,learning.digest(model),frames)

    def test_preparation_raw_pair_and_pending_denominator_without_fabricated_points(self):
        record = dict(step=1,phase='preparation',status='preparation_recorded',rawAudit=self.audit,
            collection=dict(request=dict(approvalPlanSha256=learning.digest(self.plan),command=self.plan['steps'][0]['command'])),
            prospective=dict(observation=None,command=self.plan['steps'][0]['command']),afterObservation=None,transition=None)
        self.write(self.runtime/'step-01.json',record)
        view=m.load_view(self.runtime)
        self.assertEqual(view['counts']['listedEvaluation'],1);self.assertEqual(view['counts']['prospectiveVerified'],0)
        self.assertEqual(view['records'][1]['status'],'pending_no_record')
        for frame in view['records'][0]['frames'].values():
            self.assertEqual(frame['status'],'verified');self.assertEqual(frame['points'],[])
        record['afterObservation']=dict(annotation=dict(frameId=self.frames['after']['frameId'],pixelSha256=self.frames['after']['pixelSha256']),
            pixelSha256=self.frames['after']['pixelSha256'],perPoint=dict(jawLeft=dict(status='valid',point=[8,12]),jawRight=dict(status='valid',point=[24,12])))
        self.write(self.runtime/'step-01.json',record)
        view=m.load_view(self.runtime)
        self.assertEqual(len(view['records'][0]['frames']['after']['points']),2)
        self.assertTrue(view['executedPlanAnchored']);self.assertEqual(view['executionSourceHashes'],self.plan['sourceHashes'])
        (self.pair/'esp32-before.jpg').write_bytes(b'corrupt')
        record=m.load_view(self.runtime)['records'][0]
        self.assertIn('verificationError',record);self.assertFalse(record['prospectiveVerified'])

    def test_plan_edits_phase_and_missing_denominator_are_top_level_rejections(self):
        record=dict(step=1,phase='preparation',status='preparation_recorded',collection=dict(request=dict(
            approvalPlanSha256=learning.digest(self.plan),command=self.plan['steps'][0]['command'])))
        self.write(self.runtime/'step-01.json',record)
        for plan in (dict(self.plan,steps=self.plan['steps'][:1]),dict(self.plan,steps=[])):
            self.write(self.runtime/'plan.json',plan)
            with self.assertRaises(ValueError):m.load_view(self.runtime)
        self.write(self.runtime/'plan.json',self.plan)
        record['phase']='evaluation';self.write(self.runtime/'step-01.json',record)
        with self.assertRaisesRegex(ValueError,'phase_changed'):m.load_view(self.runtime)
        record=dict(step=1,phase='preparation',status='skipped');self.write(self.runtime/'step-01.json',record)
        with self.assertRaisesRegex(ValueError,'invalid_skipped'):m.load_view(self.runtime)

    def test_only_private_new_output_and_explicit_local_rerun_allowed(self):
        for output,connect in [(self.root/'public.rrd',None),(self.local/'a.rrd','rerun+http://remote.example:9876/proxy')]:
            with self.assertRaises(ValueError):m.export(self.runtime,output,connect=connect)
        output=self.local/'exists.rrd';output.write_bytes(b'keep')
        with self.assertRaisesRegex(ValueError,'new_rrd'):m.export(self.runtime,output)


if __name__ == '__main__': unittest.main()
