"""Collector stop/preservation checks with fake GETs and virtual host time."""
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'tests'))
from test_fresh_source_age import fresh_receipt
import collect


class CollectorTest(unittest.TestCase):
    def run_fixture(self, failure):
        temp=tempfile.TemporaryDirectory(); self.addCleanup(temp.cleanup)
        root=Path(temp.name)
        files=['learning/device_adapter.py','learning/http_timing.py','learning/fresh_capture.py','learning/artifacts.py',
               'ros2_ws/src/motionbrain_ros_bridge/motionbrain_ros_bridge/observation.py','collect.py','clock_analysis.py']
        for f in files:
            dest=root/f; dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy((HERE if '/' not in f else ROOT)/f,dest)
        class Clock:
            now=100
            def monotonic(self): return self.now
            def sleep(self,seconds): self.now+=seconds
        clock=Clock(); calls=[]
        body=b'\xff\xd8fixture\xff\xd9'
        original=fresh_receipt()
        state={'camera':{'cameraBootId':original['headers']['x-motionbrain-camera-boot-id']}}
        class HTTP:
            host_clock_id=original['hostClockId']
            def get(self,device,path,query=None):
                calls.append(path)
                r=copy.deepcopy(original); r['query']=query or {}
                r['sha256']=hashlib.sha256(body).hexdigest()
                if failure=='invalid_trace': r['connectionTiming']['sendStartSemantics']='unknown'
                if failure=='http_failure': r['status']=503
                if failure=='reboot' and len(calls)==2:
                    r['headers']['x-motionbrain-camera-boot-id']='b'*32
                    r['headers']['x-motionbrain-frame-id']='b'*32+':5000000'
                return r,body
        with patch.object(collect,'ReadOnlyHTTP',return_value=HTTP()), patch.object(collect,'snapshot',return_value=state), \
             patch.object(collect,'time',clock):
            self.assertFalse(collect.collect(root))
        run=root/'clock-run'
        report=json.loads((run/'run.json').read_text())
        self.assertEqual(report['status'],'failed'); self.assertEqual(report['motorCommandsSent'],0)
        return run,report,calls

    def test_invalid_trace_keeps_receipt_and_original_then_stops(self):
        run,r,calls=self.run_fixture('invalid_trace')
        self.assertEqual(calls,['/capture']); self.assertEqual(r['completedSamples'],0)
        self.assertTrue((run/'receipts/0000.json').exists())
        self.assertTrue((run/'images/0/source.jpg').exists())

    def test_http_error_keeps_receipt_without_retry(self):
        run,r,calls=self.run_fixture('http_failure')
        self.assertEqual(calls,['/capture']); self.assertEqual(r['completedSamples'],0)
        self.assertTrue((run/'receipts/0000.json').exists())
        self.assertFalse((run/'images/0/source.jpg').exists())

    def test_camera_reboot_preserves_prefix_and_does_not_restart(self):
        run,r,calls=self.run_fixture('reboot')
        self.assertEqual(len(calls),2); self.assertEqual(r['completedSamples'],1)
        self.assertTrue((run/'receipts/0001.json').exists())
        self.assertTrue((run/'samples/0000.json').exists())


if __name__=='__main__': unittest.main()
