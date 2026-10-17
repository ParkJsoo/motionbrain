"""Combined producer lifecycle tests using fake USB, HTTP and camera only."""
import base64
import copy
import json
from pathlib import Path
import shutil
import subprocess
import sys
import unittest
from unittest.mock import Mock,patch

from learning.artifacts import StorageError,describe_artifact
from learning.usb_camera_probe import collect,audit
from tests import test_mac_controller_probe as fixture
from tests.test_mac_clock_bridge import sample as clock_sample
from tests.test_m4_stage_trace import snapshot,stage_record,samples_from_writes,diagnostic_receipt
from tests.test_compact_m4_probe import CompactSerial


class CombinedTest(unittest.TestCase):
    def setUp(self):
        self.f=fixture.ProbeTest();self.f.setUp();self.addCleanup(self.f.doCleanups)
        self.serial=CompactSerial(self.f.clock);self.stop=Mock();self.diagnostic_calls=0
        self.http_mutate=lambda endpoint,value:None
        outer=self
        class Probe:
            def sample(self,host_id):
                value=clock_sample(outer.f.clock.monotonic(),host_id);outer.f.clock.sleep(.008);return value
        self.probe=Probe();self.launch=Mock(side_effect=self.camera)
        self.f.process.wait.side_effect=lambda **kw:self.f.clock.sleep(5)
    def camera(self,helper,root,device,view,session):
        f=self.f.fixture;shift=self.f.clock.monotonic()-100+.1
        for value in (f.plan,f.session,f.terminal):value['sessionId']=session
        for i,frame in enumerate(f.frames):
            frame.update(sessionId=session,sourceFrameId=f'{session}:{i}')
            for key in ('convertedHostPtsSeconds','callbackHostSeconds','processingCompleteHostSeconds','storageStartedHostSeconds'):
                frame[key]+=shift
            frame['pts']['value']+=round(shift*1000)
            f.terminal['frames'][i]['storageCompleteHostSeconds']+=shift;f.save_frame(i)
        for name,value in [('plan',f.plan),('session',f.session),('terminal',f.terminal)]:
            (f.root/f'{name}.json').write_text(json.dumps(value))
        shutil.copytree(f.root,root/'camera')
        for name in ('helper.stdout','helper.stderr'):(root/'camera'/name).write_bytes(b'')
        (root/'camera/evaluation.json').write_text('{}')
        for name in ('camera-wrapper.stdout','camera-wrapper.stderr'):(root/name).write_bytes(b'')
        self.f.clock.sleep(.2)
        return self.f.process
    def http(self,url,endpoint,*,clock):
        if endpoint!='m4_diagnostics':
            self.f.mutate=self.http_mutate
            return self.f.read(url,endpoint,clock=clock)
        self.diagnostic_calls+=1
        records=[stage_record(s,i) for i,s in enumerate(samples_from_writes(self.serial.writes),1)]
        value=snapshot(records);self.http_mutate(endpoint,value)
        return diagnostic_receipt(value,clock)
    def run_probe(self,**changes):
        options=dict(output=self.f.root,controller_url='http://192.0.2.1',port='fake',usb_serial='fake',
                     helper=sys.executable,device_id='synthetic-device',view_id='synthetic-new-desk',
                     clock=self.f.clock,clock_probe=self.probe,http=self.http,open_port=lambda *_:self.serial,
                     launch=self.launch,stop=self.stop)
        options.update(changes);return collect(**options)
    def reseal(self,relative,change):
        p=self.f.root/relative;v=json.loads(p.read_text());change(v);p.write_text(json.dumps(v))
        p=self.f.root/'result.json';r=json.loads(p.read_text())
        r['artifacts']=[describe_artifact(self.f.root,x['artifactRef']) for x in r['artifacts']]
        if 'clockLink' in r:r['clockLink']=describe_artifact(self.f.root,'clock/link.json')
        p.write_text(json.dumps(r));p=self.f.root/'events/00000001.json';v=json.loads(p.read_text())
        v['payload']['summary']=describe_artifact(self.f.root,'result.json');p.write_text(json.dumps(v))
    def test_single_session_identity_and_full_audit(self):
        r=self.run_probe();self.assertEqual(r['status'],'recorded',r)
        self.assertEqual((r['serialCount'],r['httpCount'],r['diagnosticCount']),(6,8,2))
        self.assertTrue(self.serial.closed);self.stop.assert_called_once_with(self.f.process)
        self.assertTrue(audit(self.f.root)['ok'],audit(self.f.root))
        link=json.loads((self.f.root/'clock/link.json').read_text());self.assertEqual(len(link['frames']),21)
        self.assertFalse(link['physicalSampleAlignmentVerified']);self.assertFalse(r['trainingEligible'])
    def initialization_opener(self, mutate_history=lambda value:None):
        # The synthetic USB/stage fixtures use boot a. Start from a different
        # old boot so only opening the port establishes the new boot baseline.
        self.f.values['actions']['bootId']='b'*32
        self.f.values['action_outputs']['bootId']='b'*32
        def opened(*args):
            self.assertEqual(self.f.calls,['status','sequence','actions','action_outputs'])
            self.assertEqual(self.serial.writes,[]);self.launch.assert_not_called()
            self.f.values['actions']['bootId']='a'*32
            history=dict(bootId='a'*32,dropped=0,entries=[
                dict(sequence=i,motor=i,pwm=0,actionOrdinal=0) for i in range(1,6)])
            mutate_history(history)
            self.f.values['action_outputs']=history
            self.serial.pending.extend(b'=== MotionBrain System Boot ===\n')
            return self.serial
        return Mock(side_effect=opened)
    def test_initialization_drains_before_new_baseline_and_clock_brackets(self):
        opener=self.initialization_opener()
        r=self.run_probe(initialize_port=True,open_port=opener)
        self.assertEqual(r['status'],'recorded',r)
        self.assertEqual((r['httpCount'],r['diagnosticCount'],r['serialCount']),(12,2,6))
        self.assertTrue(r['controllerBootChangedDuringInitialization'])
        self.assertTrue(r['outputHistoryUnchanged']);self.assertFalse(r['trainingEligible'])
        opener.assert_called_once_with('fake','fake');self.assertTrue(self.serial.closed)
        rows=[json.loads((self.f.root/f'http/{i:02d}.json').read_text()) for i in range(1,13)]
        self.assertEqual([row['phase'] for row in rows],['pre_open']*4+['baseline']*4+['final']*4)
        self.assertEqual(rows[2]['response']['bootId'],'b'*32)
        self.assertEqual(rows[6]['response']['bootId'],'a'*32)
        self.assertEqual(rows[7]['response'],rows[11]['response'])
        startup=json.loads((self.f.root/'serial/startup.json').read_text())
        self.assertEqual(base64.b64decode(startup['rawBase64']),b'=== MotionBrain System Boot ===\n')
        self.assertGreaterEqual(startup['endedMonotonic']-startup['startedMonotonic'],20)
        self.assertLessEqual(startup['endedMonotonic']-startup['startedMonotonic'],20.1)
        self.assertLessEqual(rows[3]['receivedMonotonic'],startup['startedMonotonic'])
        self.assertLessEqual(startup['endedMonotonic'],rows[4]['requestStartedMonotonic'])
        clock=json.loads((self.f.root/'clock/before.json').read_text())
        self.assertLessEqual(rows[7]['receivedMonotonic'],clock['pairs'][0]['before'])
        self.assertLessEqual(clock['pairs'][-1]['after'],r['cameraProcessStartedMonotonic'])
        self.stop.assert_called_once_with(self.f.process)
        self.assertTrue(audit(self.f.root)['ok'],audit(self.f.root))
    def test_initialization_empty_new_boot_history_blocks_before_camera_and_s3(self):
        opener=self.initialization_opener(lambda history:history.update(entries=[]))
        r=self.run_probe(initialize_port=True,open_port=opener)
        self.assertEqual(r['failure'],'new_boot_output_history_not_complete_off')
        self.assertEqual((r['httpCount'],r['diagnosticCount'],r['serialCount']),(8,0,0))
        opener.assert_called_once();self.launch.assert_not_called();self.stop.assert_not_called()
        self.assertEqual(self.serial.writes,[]);self.assertTrue(self.serial.closed)
        self.assertTrue(audit(self.f.root)['ok'])
    def test_initialization_nonzero_new_boot_history_cannot_be_hidden_by_current_off(self):
        opener=self.initialization_opener(lambda history:history['entries'][0].update(pwm=40))
        r=self.run_probe(initialize_port=True,open_port=opener)
        self.assertEqual(r['failure'],'new_boot_output_history_not_complete_off')
        self.assertEqual((r['httpCount'],r['diagnosticCount'],r['serialCount']),(8,0,0))
        self.launch.assert_not_called();self.assertEqual(self.serial.writes,[])
        self.assertTrue(self.serial.closed);self.assertTrue(audit(self.f.root)['ok'])
    def test_initialization_does_not_allow_later_controller_boot_change(self):
        opener=self.initialization_opener()
        def change(endpoint,value):
            if endpoint=='actions' and len(self.f.calls)>8:value['bootId']='c'*32
        self.http_mutate=change
        r=self.run_probe(initialize_port=True,open_port=opener)
        self.assertTrue(r['controllerBootChangedDuringInitialization'])
        self.assertEqual(r['failure'],'controller_boot_or_outputs_changed')
        self.assertEqual((r['httpCount'],r['diagnosticCount'],r['serialCount']),(12,2,6))
        opener.assert_called_once();self.assertTrue(self.serial.closed);self.stop.assert_called_once()
        self.assertTrue(audit(self.f.root)['ok'])
    def test_unsafe_baseline_never_opens_camera_or_serial(self):
        def mutate(endpoint,value):
            if endpoint=='actions':value['enabled']=True
        self.http_mutate=mutate;opener=Mock()
        r=self.run_probe(open_port=opener);self.assertEqual(r['status'],'rejected')
        opener.assert_not_called();self.launch.assert_not_called();self.assertTrue(audit(self.f.root)['ok'])
    def test_camera_early_exit_closes_usb_and_keeps_partial_files(self):
        def early(*args):
            p=self.camera(*args);(args[1]/'camera/frame-0000.json').unlink();p.poll.return_value=1;return p
        self.launch.side_effect=early;r=self.run_probe()
        self.assertEqual(r['failure'],'camera_exited_before_first_frame');self.assertEqual(r['serialCount'],0)
        self.assertTrue(self.serial.closed);self.stop.assert_called_once();self.assertTrue(audit(self.f.root)['ok'])
    def test_cancel_during_serial_keeps_partial_attempt_and_stops_both(self):
        original=self.serial.read
        def read(n):
            if self.serial.writes and len(self.serial.pending)==49:raise KeyboardInterrupt()
            return original(n)
        self.serial.read=read;r=self.run_probe()
        self.assertTrue(r['cancelled']);self.assertEqual(r['serialCount'],1)
        self.assertTrue(self.serial.closed);self.stop.assert_called_once();self.assertTrue(audit(self.f.root)['ok'])
    def test_camera_wait_timeout_seals_sensor_and_camera_evidence(self):
        self.f.process.wait.side_effect=subprocess.TimeoutExpired('camera',47)
        r=self.run_probe();self.assertEqual(r['status'],'rejected');self.assertEqual(r['serialCount'],6)
        self.assertTrue(self.serial.closed);self.stop.assert_called_once();self.assertTrue(audit(self.f.root)['ok'])
    def test_each_cleanup_runs_even_if_usb_close_fails(self):
        self.serial.close=Mock(side_effect=OSError('close_failed'))
        r=self.run_probe();self.assertEqual(r['status'],'rejected');self.stop.assert_called_once()
        self.assertTrue(r['cleanupErrors']);self.assertTrue(audit(self.f.root)['ok'])
    def test_storage_failure_cleans_resources_and_seals_when_storage_recovers(self):
        from learning.artifacts import ArtifactStore
        original=ArtifactStore.write_json
        def write(store,path,value):
            if path=='serial/01.json':raise StorageError('injected_storage_failure')
            return original(store,path,value)
        with patch.object(ArtifactStore,'write_json',write):r=self.run_probe()
        self.assertEqual(r['status'],'rejected');self.assertTrue(self.serial.closed);self.stop.assert_called_once()
        self.assertTrue((self.f.root/'result.json').exists())
        # Missing original receipt cannot pass an integrity audit.
        self.assertFalse(audit(self.f.root)['ok'])
    def test_output_change_rejected_after_collection(self):
        def mutate(endpoint,value):
            if endpoint=='action_outputs' and len(self.f.calls)>4:value['dropped']+=1
        self.http_mutate=mutate;r=self.run_probe()
        self.assertEqual(r['failure'],'controller_boot_or_outputs_changed');self.assertEqual(r['serialCount'],6)
    def test_mac_reboot_rejected_by_saved_clock_link(self):
        original=self.probe.sample;calls=[]
        def sample(host):
            v=original(host);calls.append(1)
            if len(calls)==2:v['hostBootId']='different'
            return v
        self.probe.sample=sample;r=self.run_probe();self.assertEqual(r['status'],'rejected')
        self.assertIn('clock_identity_changed',r['failure']);self.assertTrue(self.serial.closed)
    def test_resealed_link_summary_cannot_override_originals(self):
        self.assertEqual(self.run_probe()['status'],'recorded')
        self.reseal('clock/link.json',lambda v:v['frames'][0].update(within20ms=not v['frames'][0]['within20ms']))
        self.assertFalse(audit(self.f.root)['ok'])
    def test_resealed_wrong_receipt_session_rejected(self):
        self.assertEqual(self.run_probe()['status'],'recorded')
        self.reseal('serial/01.json',lambda v:v.update(collectionSessionId='other'))
        self.assertFalse(audit(self.f.root)['ok'])
    def test_resealed_wrong_camera_view_rejected(self):
        self.assertEqual(self.run_probe()['status'],'recorded')
        self.reseal('plan.json',lambda v:v.update(viewId='old-view'))
        self.assertFalse(audit(self.f.root)['ok'])
    def test_new_directory_required_no_restart(self):
        self.run_probe();count=len(self.serial.writes)
        with self.assertRaises(StorageError):self.run_probe()
        self.assertEqual(len(self.serial.writes),count)
    def test_invalid_helper_fails_before_devices(self):
        with self.assertRaises(FileNotFoundError):self.run_probe(helper='/nonexistent/motionbrain/helper')
        self.assertFalse(self.f.calls);self.launch.assert_not_called()

    def test_cancelled_http_attempt_keeps_prefix_and_stops(self):
        original=self.http
        def http(url,endpoint,*,clock):
            row=original(url,endpoint,clock=clock)
            if endpoint=='status':row.update(cancelled=True,transportCompleted=False,error='cancelled')
            return row
        r=self.run_probe(http=http);self.assertTrue(r['cancelled']);self.assertEqual(r['httpCount'],1)
        self.launch.assert_not_called();self.assertTrue(audit(self.f.root)['ok'])
    def test_cancelled_post_camera_diagnostic_keeps_all_attempts(self):
        original=self.http
        def http(url,endpoint,*,clock):
            row=original(url,endpoint,clock=clock)
            if endpoint=='m4_diagnostics' and self.diagnostic_calls==2:
                row.update(cancelled=True,transportCompleted=False,error='cancelled')
            return row
        r=self.run_probe(http=http);self.assertTrue(r['cancelled'])
        self.assertEqual((r['serialCount'],r['diagnosticCount']),(6,2));self.assertTrue(self.serial.closed)
        self.stop.assert_called_once();self.assertTrue(audit(self.f.root)['ok'])


if __name__=='__main__':unittest.main()
