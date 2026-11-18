"""명시 접촉 미세 수집 예외의 두 명령·감시·자료 출처 검증."""
import copy
import hashlib
import json
import unittest

from learning import attended_collection_profile as profile
from learning.manual_collection import ManualCollection
from learning.supervised_transport import (ELBOW_CONTACT_200_POLICY,ELBOW_CONTACT_250_POLICY,
    ELBOW_DEADLINE_POLICY,WRIST_READ_BUDGET_REVISION,validate_monitor_policy,validate_pulse)
from tests import test_supervised_transport as transport_fixture


class ElbowContactFineTests(unittest.TestCase):
    def command(self,duration=200):return dict(joint='elbow',direction='down',percent=60,durationMs=duration)
    def receipt(self,duration=200):
        return dict(monitorTimingPolicy=ELBOW_CONTACT_200_POLICY if duration==200 else ELBOW_CONTACT_250_POLICY,
            monitorStatusVerified=True,monitorReadBudgetRevision=WRIST_READ_BUDGET_REVISION,offVerified=True)
    def manager(self):
        manager=ManualCollection('/unused','/unused','http://127.0.0.1','','camera',collection_profile=profile.ATTENDED_ELBOW_CONTACT_FINE)
        manager.current=dict(joint='elbow',collectionProfile=profile.collection_profile(profile.ATTENDED_ELBOW_CONTACT_FINE));manager.command_count=0
        return manager
    def test_existing_profile_hashes_and_1000ms_minimum_remain_unchanged(self):
        expected={profile.ATTENDED_60_1000:'0d309fbaf6025af96ea91eeb01badcc78fbe881d693761d85a57abe592ddc036',
            profile.ATTENDED_SHOULDER_DOWN_12:'3c22f86f8f1d95c7c302f1d4ab61911bba811e4ae085afe07cd5dc7ca08607a6'}
        for name,sha in expected.items():
            value=profile.collection_profile(name);actual=hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest();self.assertEqual(actual,sha)
        for duration in (200,250):
            with self.assertRaisesRegex(ValueError,'output_limit'):profile.validate_command(profile.ATTENDED_60_1000,self.command(duration))
        manager=ManualCollection('/unused','/unused','http://127.0.0.1','','camera');self.assertIsNone(manager.collection_profile)
    def test_exact_two_commands_have_same_153_motor_mapping(self):
        metadata=profile.collection_profile(profile.ATTENDED_ELBOW_CONTACT_FINE)
        self.assertEqual(metadata['directions'],{'elbow':['down']});self.assertEqual(metadata['maximumCommands'],1);self.assertEqual(metadata['allowedDurationMs'],[200,250]);self.assertFalse(metadata['automaticRetry']);self.assertFalse(metadata['physicalDurationHardGuarantee'])
        for duration in (200,250):
            pulse=profile.validate_command(profile.ATTENDED_ELBOW_CONTACT_FINE,self.command(duration));self.assertEqual(pulse,dict(self.command(duration),motor=3,expectedPwm=153));self.assertEqual(validate_pulse(pulse),pulse)
        metadata['allowedDurationMs'].append(225);self.assertEqual(profile.collection_profile(profile.ATTENDED_ELBOW_CONTACT_FINE)['allowedDurationMs'],[200,250])
    def test_extra_axis_up_percent_intermediate_duration_or_bool_rejected(self):
        changes=[dict(joint='gripper',direction='open'),dict(joint='wrist'),dict(joint='shoulder'),dict(direction='up'),dict(percent=59),dict(percent=61),dict(percent=True),dict(durationMs=199),dict(durationMs=225),dict(durationMs=251),dict(durationMs=200.),dict(durationMs=True),dict(extra='not_allowed')]
        for change in changes:
            with self.subTest(change=change),self.assertRaises(ValueError):profile.validate_command(profile.ATTENDED_ELBOW_CONTACT_FINE,dict(self.command(),**change))
    def test_exact_monitor_policy_rejects_wrong_pair_mapping_direction_and_types(self):
        for duration,policy in ((200,ELBOW_CONTACT_200_POLICY),(250,ELBOW_CONTACT_250_POLICY)):
            pulse=profile.validate_command(profile.ATTENDED_ELBOW_CONTACT_FINE,self.command(duration));validate_monitor_policy(pulse,policy)
            for change in (dict(durationMs=250 if duration==200 else 200),dict(durationMs=225),dict(joint='wrist',motor=2),dict(direction='up',expectedPwm=-153),dict(motor=1),dict(expectedPwm=152),dict(expectedPwm=153.),dict(percent=True)):
                with self.subTest(duration=duration,change=change),self.assertRaisesRegex(ValueError,'unsupported_monitor'):validate_monitor_policy(dict(pulse,**change),policy)
            with self.assertRaisesRegex(ValueError,'unsupported_monitor'):validate_monitor_policy(pulse,ELBOW_DEADLINE_POLICY)
        old=validate_pulse(dict(joint='elbow',direction='up',percent=60,durationMs=350,motor=3,expectedPwm=-153));validate_monitor_policy(old,ELBOW_DEADLINE_POLICY)
    def test_real_manual_validation_then_monitor_and_one_command_session_budget(self):
        for duration,policy in ((200,ELBOW_CONTACT_200_POLICY),(250,ELBOW_CONTACT_250_POLICY)):
            manager=self.manager();pulse=manager._validate_pulse(self.command(duration));validate_monitor_policy(pulse,policy)
            manager.command_count=1
            with self.assertRaisesRegex(ValueError,'session_command_budget_reached'):manager._validate_pulse(self.command(duration))
        manager=self.manager();manager.current['collectionProfile']['allowedDurationMs'].append(225)
        with self.assertRaisesRegex(ValueError,'profile_changed'):manager._validate_pulse(self.command())
    def test_fine_audit_requires_exact_profile_command_and_safe_monitor_receipt(self):
        metadata=profile.collection_profile(profile.ATTENDED_ELBOW_CONTACT_FINE);plan=dict(collectionProfile=copy.deepcopy(metadata));session=copy.deepcopy(plan)
        for duration in (200,250):
            pulse=profile.validate_command(profile.ATTENDED_ELBOW_CONTACT_FINE,self.command(duration));receipt=self.receipt(duration)
            self.assertEqual(profile.audit_profile(plan,session,pulse,receipt),metadata)
            for change in (dict(monitorTimingPolicy=ELBOW_DEADLINE_POLICY),dict(monitorTimingPolicy=ELBOW_CONTACT_250_POLICY if duration==200 else ELBOW_CONTACT_200_POLICY),dict(monitorStatusVerified=False),dict(offVerified=False),dict(monitorReadBudgetRevision='per-read-200ms-v1')):
                with self.subTest(duration=duration,change=change),self.assertRaisesRegex(ValueError,'monitor_provenance'):profile.audit_profile(plan,session,pulse,dict(receipt,**change))
            with self.assertRaisesRegex(ValueError,'monitor_provenance'):profile.audit_profile(plan,session,pulse)
        changed=copy.deepcopy(plan);changed['collectionProfile']['allowedDurationMs'].append(225)
        with self.assertRaisesRegex(ValueError,'profile_provenance'):profile.audit_profile(changed,copy.deepcopy(changed),pulse,self.receipt(250))
        with self.assertRaises(ValueError):profile.audit_profile(plan,session,dict(pulse,durationMs=225),self.receipt(250))
    def test_real_shared_deadline_transport_requires_first_safe_status_and_one_RUN(self):
        for duration,policy in ((200,ELBOW_CONTACT_200_POLICY),(250,ELBOW_CONTACT_250_POLICY)):
            manager=self.manager();pulse=manager._validate_pulse(self.command(duration));port,device,_=transport_fixture.CompactTransportTests().setup_port()
            port.preflight();result=port.execute(pulse,lambda:None,monitor_policy=policy)
            self.assertTrue(result['offVerified']);self.assertTrue(result['monitorStatusVerified']);self.assertEqual(result['monitorTimingPolicy'],policy);self.assertEqual(result['monitorReadBudgetRevision'],WRIST_READ_BUDGET_REVISION)
            run=next(i for i,(_,body) in enumerate(device.requests) if body=={'action':'run'})
            self.assertTrue(any(path=='/collection_status' for path,_ in device.requests[run+1:]));self.assertEqual(sum(body=={'action':'run'} for _,body in device.requests),1)
            self.assertTrue(any(e['motor']==3 and e['pwm']==153 for e in result['outputTrace']));self.assertFalse(any(device.speeds.values()))
    def test_deadline_without_first_safety_status_fails_closed(self):
        pulse=profile.validate_command(profile.ATTENDED_ELBOW_CONTACT_FINE,self.command());port,device,_=transport_fixture.CompactTransportTests().setup_port();port.preflight();original=port._outputs
        def drain(*args,**kw):
            result=original(*args,**kw)
            if port.may_have_run and kw.get('deadline') is not None:device.clock.now+=.21
            return result
        port._outputs=drain
        with self.assertRaisesRegex(ValueError,'active_safety_status_not_verified'):port.execute(pulse,lambda:None,monitor_policy=ELBOW_CONTACT_200_POLICY)
        self.assertEqual(sum(body=={'action':'run'} for _,body in device.requests),1);self.assertFalse(any(device.speeds.values()))

if __name__=='__main__':unittest.main()
