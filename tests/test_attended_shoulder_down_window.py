"""Explicit finite shoulder DOWN experiment: defaults and other guards survive."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.attended_collection_profile import (ATTENDED_SHOULDER_DOWN_12, ATTENDED_60_1000,
    collection_profile, shoulder_down_window, validate_command, audit_profile, audit_shoulder_down_observations)
from learning.manual_collection import ManualCollection
from learning.supervised_transport import SequencePort
from tests import test_supervised_transport as transport
from tests import test_manual_collection as collection


class AttendedShoulderDownWindowTests(unittest.TestCase):
    def setup_port(self, *, enabled=True, delta=-9., start=220.):
        clock=transport.Clock();device=transport.Controller(clock);device.angle=start
        store=transport.Store()
        port=SequencePort('http://controller.invalid','secret','boot',store,clock=clock,
                          attended_shoulder_down_window=enabled)
        port.opener=device
        device.after_run=lambda d:setattr(d,'angle',start+delta)
        return port,device,store

    def test_only_new_DOWN_policy_allows_larger_decrease_and_records_it(self):
        command=transport.pulse('shoulder','down',70,1000)
        for enabled in (False,True):
            with self.subTest(enabled=enabled):
                port,device,_=self.setup_port(enabled=enabled)
                if enabled:
                    receipt=port.execute(command,lambda:None)
                    self.assertTrue(receipt['offVerified'])
                    self.assertEqual(receipt['attendedShoulderDownWindow'],shoulder_down_window())
                else:
                    with self.assertRaisesRegex(ValueError,'m4_observed_change_exceeded'):
                        port.execute(command,lambda:None)
                self.assertIsNone(port._attended_shoulder_down_window_policy)
                self.assertEqual(sum(payload=={'action':'run'} for _,payload in device.requests),1)
                self.assertEqual(sum(payload=={'cmd':'stop'} for _,payload in device.requests),1)

    def test_decrease_above_twelve_reverse_above_three_or_mechanical_boundary_stops(self):
        for delta,start,reason in [(-12.01,220.,'m4_observed_change_exceeded'),
                                  (3.01,220.,'m4_observed_change_exceeded'),
                                  (-9.,135.,'m4_feedback_not_ready')]:
            with self.subTest(delta=delta,start=start):
                port,device,_=self.setup_port(delta=delta,start=start)
                with self.assertRaisesRegex(ValueError,reason):
                    port.execute(transport.pulse('shoulder','down',70,1000),lambda:None)
                self.assertFalse(any(device.speeds.values()))
                self.assertIsNone(port._attended_shoulder_down_window_policy)

    def test_UP_other_joint_low_output_or_short_duration_rejected_before_IO(self):
        for pulse in [transport.pulse('shoulder','up',70,1000),
                      transport.pulse('elbow','up',70,1000),
                      transport.pulse('shoulder','down',59,1000),
                      transport.pulse('shoulder','down',70,999)]:
            with self.subTest(pulse=pulse):
                port,device,_=self.setup_port()
                with self.assertRaises(ValueError):port.execute(pulse,lambda:None)
                self.assertEqual(device.requests,[])

    def test_pre_RUN_drift_still_three_degrees_and_sensor_freshness_required(self):
        for mode in ('drift','stale'):
            with self.subTest(mode=mode):
                port,device,_=self.setup_port();original=device.open
                def opened(request,timeout):
                    response=original(request,timeout)
                    if request.data is not None and b'cmd=arm' in request.data:
                        if mode=='drift':device.angle=224.
                        else:device.sensor_age=200
                    return response
                device.open=opened
                with self.assertRaisesRegex(ValueError,'m4_observed_change_before_run|controller_safety_not_ready'):
                    port.execute(transport.pulse('shoulder','down',70,1000),lambda:None)
                self.assertFalse(any(payload=={'action':'run'} for _,payload in device.requests))
                self.assertIsNone(port._attended_shoulder_down_window_policy)

    def test_profile_provenance_requires_matching_plan_session_receipt(self):
        profile=collection_profile(ATTENDED_SHOULDER_DOWN_12);window=shoulder_down_window()
        pulse=validate_command(ATTENDED_SHOULDER_DOWN_12,
            dict(joint='shoulder',direction='down',percent=70,durationMs=1000))
        plan=dict(collectionProfile=profile,attendedShoulderDownWindow=window)
        session=copy.deepcopy(plan);receipt=dict(attendedShoulderDownWindow=copy.deepcopy(window))
        self.assertEqual(audit_profile(plan,session,pulse,receipt),profile)
        for location in ('plan','session','receipt'):
            values=copy.deepcopy(dict(plan=plan,session=session,receipt=receipt))
            values[location]['attendedShoulderDownWindow']['maximumDecreaseDeg']=13.
            with self.subTest(location=location),self.assertRaisesRegex(ValueError,'window_provenance'):
                audit_profile(values['plan'],values['session'],pulse,values['receipt'])
        legacy=dict(collectionProfile=collection_profile(ATTENDED_60_1000))
        with self.assertRaisesRegex(ValueError,'window_unplanned'):
            audit_profile(legacy,legacy,pulse,receipt)

    def test_manager_optin_binds_profile_plan_session_and_port(self):
        class Port(collection.Port):
            def __init__(self,*args,attended_shoulder_down_window=False,**kwargs):
                super().__init__(*args,**kwargs)
                self.attended_shoulder_down_window=attended_shoulder_down_window
            def execute(self,pulse,tick):
                return dict(super().execute(pulse,tick),attendedShoulderDownWindow=shoulder_down_window())
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);helper=root/'helper';helper.write_text('test')
            manager=ManualCollection(root/'sessions',helper,'http://controller.invalid','secret','camera',
                journal_path=root/'journal.sqlite',clock=collection.Clock(),camera_factory=collection.Camera,
                port_factory=Port,collection_profile=ATTENDED_60_1000,attended_shoulder_down_window=True)
            try:
                with patch('learning.manual_collection.shutil.disk_usage',return_value=type('Disk',(),{'free':20_000_000_000})()):
                    manager.start(dict(joint='shoulder',poseContext='current-L-pose',onsiteConfirmed=True))
                plan=json.loads((manager.store.root/'plan.json').read_text())
                self.assertEqual(plan['collectionProfile'],collection_profile(ATTENDED_SHOULDER_DOWN_12))
                self.assertEqual(plan['attendedShoulderDownWindow'],manager.current['attendedShoulderDownWindow'])
                self.assertTrue(manager.port.attended_shoulder_down_window)
                manager.pulse(dict(joint='shoulder',direction='down',percent=70,durationMs=1000))
                manager.finish()
                session=json.loads((manager.store.root/'session.json').read_text())
                receipt=[e['receipt'] for e in session['events'] if e['kind']=='result'][0]
                audit_profile(plan,session,validate_command(ATTENDED_SHOULDER_DOWN_12,
                    dict(joint='shoulder',direction='down',percent=70,durationMs=1000)),receipt)
            finally:manager.close()

    def test_offline_observed_angles_reapply_asymmetric_and_pre_RUN_limits(self):
        port,device,_=self.setup_port();pulse=transport.pulse('shoulder','down',70,1000)
        def event(at,angle,state):
            device.angle=angle;device.state=state
            return dict(requestStartedAt=at,responseReceivedAt=at+.01,response=copy.deepcopy(device.status()))
        rows=[event(1.,220.,'IDLE'),event(2.,220.,'ARMED'),event(4.,211.,'ARMED')]
        receipt=dict(dispatchBefore=3.,dispatchAfter=3.1,offAt=5.,attendedShoulderDownWindow=shoulder_down_window())
        audit_shoulder_down_observations(rows,pulse,receipt)
        for index,angle in [(1,224.),(2,207.9),(2,223.1)]:
            changed=copy.deepcopy(rows);changed[index]['response']['shoulderAngle']['angleDeg']=angle
            with self.subTest(index=index,angle=angle),self.assertRaises(ValueError):
                audit_shoulder_down_observations(changed,pulse,receipt)
