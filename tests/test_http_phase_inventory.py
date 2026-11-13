"""최초 HTTP 진단 보조 파일을 감사하되 재시도·변조를 숨기지 않는 회귀 검사."""
import copy
import json
from pathlib import Path
import tempfile
import types
import unittest

from learning.http_phase_trace import HttpPhaseTrace, http_inventory_matches, validate_initial_phase_report
from learning.manual_collection_dataset import Evidence, audit_http
from learning import arm_session_reference, batch_m4
from tests import test_pipeline_evidence as pipeline_fixture
from tests import test_manual_collection_dataset as dataset_fixture
from tests import test_arm_session_reference as arm_fixture


def phase_report(intent, response):
    # 실제 수집기와 같은 constructor/이벤트 직렬화를 사용하고 네트워크는 열지 않는다.
    clock=types.SimpleNamespace(monotonic=lambda:intent['startedAt'])
    trace=HttpPhaseTrace(clock=clock)
    for phase,event in [('send','begin'),('connect','begin'),('connect','end'),('send','end'),
                        ('headers','begin'),('headers','end'),('body','begin'),('body','end')]:
        trace._mark(phase,event)
    return dict(phase=trace.phase,events=trace.events,
                elapsedSeconds=response['receivedAt']-intent['startedAt'])


def add_phase(root):
    intent=json.loads((root/'http-000001-intent.json').read_text())
    response=json.loads((root/'http-000001-response.json').read_text())
    report=phase_report(intent,response)
    (root/'http-000001-phase.json').write_text(json.dumps(report))
    return report


class PhaseInventoryTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.intent=dict(method='GET',path='/actions',payload=None,startedAt=100.,timeoutSeconds=2.)
        self.response=dict(httpStatus=200,receivedAt=100.1,rawUtf8='{}',parsed={})
        self.save('http-000001-intent.json',self.intent)
        self.save('http-000001-response.json',self.response)
        self.report=add_phase(self.root)
        self.expected={'http-000001-intent.json','http-000001-response.json'}
    def save(self,name,value): (self.root/name).write_text(json.dumps(value))
    def test_valid_constructor_report_is_included_and_hashed_without_changing_HTTP_pair(self):
        evidence=Evidence(self.root)
        before={name:(self.root/name).read_bytes() for name in self.expected}
        self.assertTrue(http_inventory_matches(evidence,self.expected))
        self.assertIn('http-000001-phase.json',evidence.hashes)
        self.assertTrue(all((self.root/name).read_bytes()==raw for name,raw in before.items()))
    def test_unknown_and_attempt_two_artifacts_are_not_hidden_by_valid_auxiliary(self):
        for name in ('http-000002-phase.json','http-000001-attempt-2-intent.json','http-000001-error.json',
                     'http-000001-extra.json'):
            self.save(name,{})
            with self.subTest(name=name):self.assertFalse(http_inventory_matches(Evidence(self.root),self.expected))
            (self.root/name).unlink()
    def test_malformed_failure_noncausal_or_sensitive_phase_is_rejected(self):
        operations=[lambda d:d.update(phase='headers'),lambda d:d.update(rawUtf8='private-body'),
            lambda d:d.update(events=d['events'][:-1]),
            lambda d:d['events'][2].update(event='error',errorType='TimeoutError'),
            lambda d:d['events'][3].update(at=103.),lambda d:d.update(elapsedSeconds=0.),
            lambda d:d['events'][2].update(socket=dict(family=2,type=1,timeoutSeconds=3.)),
            lambda d:d['events'][2].update(socket=dict(family=2,type=1,timeoutSeconds=1.,headers='private'))]
        for operation in operations:
            report=copy.deepcopy(self.report);operation(report);self.save('http-000001-phase.json',report)
            with self.subTest(operation=operation),self.assertRaises(ValueError):
                http_inventory_matches(Evidence(self.root),self.expected)
    def test_phase_only_matches_successful_first_actions_GET(self):
        for change in (dict(method='POST'),dict(path='/status'),dict(payload={'cmd':'arm'})):
            self.save('http-000001-intent.json',dict(self.intent,**change))
            with self.subTest(change=change),self.assertRaisesRegex(ValueError,'request_binding'):
                http_inventory_matches(Evidence(self.root),self.expected)
        self.save('http-000001-intent.json',self.intent)
        self.save('http-000001-response.json',dict(self.response,httpStatus=500))
        with self.assertRaisesRegex(ValueError,'request_binding'):
            http_inventory_matches(Evidence(self.root),self.expected)
        with self.assertRaisesRegex(ValueError,'success_pair_required'):
            http_inventory_matches(Evidence(self.root),{'http-000001-intent.json','http-000001-error.json'})


class InventoryPathIntegrationTests(unittest.TestCase):
    def fixture(self, cls):
        fixture=cls();fixture.setUp();self.addCleanup(fixture.doCleanups)
        return fixture
    def test_manual_execution_audit_accepts_only_valid_initial_auxiliary(self):
        fixture=self.fixture(dataset_fixture.DatasetTests)
        root,session,command,receipt=fixture.http_fixture()
        before=audit_http(Evidence(root),session,command,receipt)
        add_phase(root)
        self.assertEqual(audit_http(Evidence(root),session,command,receipt),before)
        (root/'http-000001-attempt-2-intent.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'http_evidence_count_or_retry'):
            audit_http(Evidence(root),session,command,receipt)
    def test_pipeline_execution_and_no_run_paths_keep_original_outcomes(self):
        for no_run in (False,True):
            fixture=self.fixture(pipeline_fixture.PipelineEvidenceTests)
            session,frames,digest=fixture.fixture(failed=no_run,no_run=no_run)
            raw=(fixture.root/'session.json').read_bytes()
            add_phase(fixture.root)
            result=fixture.ingest(digest,frames)
            self.assertEqual(result['execution']['status'],'no_run' if no_run else 'verified',result)
            self.assertEqual((fixture.root/'session.json').read_bytes(),raw)
            self.assertIn('http-000001-phase.json',result['sourceHashes'])
            proof=json.loads((fixture.root/'http-000001-phase.json').read_text());proof['events'].pop()
            (fixture.root/'http-000001-phase.json').write_text(json.dumps(proof))
            result=fixture.ingest(digest,frames)
            self.assertEqual(result['execution']['status'],'unknown')
            self.assertFalse(result['execution']['offVerified'])
    def test_arm_session_reference_audits_auxiliary_and_still_rejects_unknown_HTTP(self):
        fixture=self.fixture(arm_fixture.SessionReferenceTests)
        report,plan=fixture.run_collection()
        add_phase(fixture.store.root)
        evidence=arm_session_reference.PinnedEvidence(fixture.store.root)
        arm_session_reference.audit_trial_http(evidence,report,plan)
        self.assertIn('http-000001-phase.json',evidence.hashes)
        (fixture.store.root/'http-000001-extra.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'http_count_error_or_retry'):
            arm_session_reference.audit_trial_http(arm_session_reference.PinnedEvidence(fixture.store.root),report,plan)
    def test_m4_no_run_audit_accepts_auxiliary_but_not_attempt_two(self):
        fixture=self.fixture(pipeline_fixture.PipelineEvidenceTests)
        fixture.fixture(failed=True,no_run=True)
        add_phase(fixture.root)
        scope=dict(controllerBootId='a'*32,pulse=dict(percent=25,durationMs=350))
        evidence=Evidence(fixture.root)
        batch_m4._audit_no_run(evidence,scope)
        self.assertIn('http-000001-phase.json',evidence.hashes)
        (fixture.root/'http-000001-attempt-2-intent.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'m4_no_run_http_incomplete'):
            batch_m4._audit_no_run(Evidence(fixture.root),scope)


if __name__=='__main__':unittest.main()
