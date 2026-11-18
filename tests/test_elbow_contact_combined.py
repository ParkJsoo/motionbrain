"""짧은 하강의 단일 응답 감사, 동일 마감과 실패 정지를 검증한다."""
import copy
import unittest
import urllib.parse
from learning import supervised_transport as transport, attended_collection_profile as profile
from tests import test_supervised_transport as fixture


class CombinedContactTests(unittest.TestCase):
    def setup_port(self, mutate=None, active_delay=.11):
        port, device, store = fixture.CompactTransportTests().setup_port()
        original = port.opener.open
        def combined(request, timeout):
            address = urllib.parse.urlsplit(request.full_url)
            if address.path != '/collection_observation':
                return original(request, timeout)
            device.requests.append((address.path, None))
            device.clock.now += active_delay if device.running else .01
            cursor = int(urllib.parse.parse_qs(address.query)['after'][0])
            retained = device.entries[-128:];latest = retained[-1]['sequence']
            entries = [e for e in retained if e['sequence'] > cursor][:8]
            proof = dict(schemaVersion='driver_cursor.v1', bootId=device.boot,
                         sequenceBefore=latest, sequenceAfter=latest, stable=True)
            outputs = dict(bootId=device.boot, evidence='driver_write', clockDomain='controller_boot_millis',
                           oldestSequence=retained[0]['sequence'], latestSequence=latest,
                           gap=cursor < retained[0]['sequence']-1, hasMore=bool(entries and entries[-1]['sequence'] < latest), entries=copy.deepcopy(entries))
            value = dict(schemaVersion='collection_observation.v1',
                         status=dict(device.status(), schemaVersion='collection.v1', outputProof=proof), outputs=outputs)
            if mutate and device.running:mutate(value, device)
            return fixture.Response(value)
        port.opener.open = combined
        return port, device, store

    def pulse(self, duration=200):
        return profile.validate_command(profile.ATTENDED_ELBOW_CONTACT_FINE,
            dict(joint='elbow', direction='down', percent=60, durationMs=duration))

    def test_one_active_request_checks_status_and_history_without_lengthening_period(self):
        for duration, policy in [(200,transport.ELBOW_CONTACT_COMBINED_200_POLICY),
                                 (250,transport.ELBOW_CONTACT_COMBINED_250_POLICY)]:
            port, device, _ = self.setup_port();port.preflight()
            receipt = port.execute(self.pulse(duration), lambda:None, monitor_policy=policy)
            self.assertTrue(receipt['offVerified']);self.assertTrue(receipt['monitorStatusVerified'])
            self.assertEqual(receipt['monitorReadBudgetRevision'],transport.ELBOW_CONTACT_READ_BUDGET_REVISION)
            run = next(i for i, (_,p) in enumerate(device.requests) if p=={'action':'run'})
            stop = next(i for i, (_,p) in enumerate(device.requests[run+1:],run+1) if p=={'cmd':'stop'})
            self.assertEqual(device.requests[run+1:stop],[('/collection_observation',None)])
            stop_time = next(v['startedAt'] for k,v in port.store.rows.items()
                             if k.endswith('-intent.json') and v.get('payload')=={'cmd':'stop'})
            self.assertAlmostEqual(stop_time-receipt['dispatchAfter'],duration/1000,places=6)
            self.assertEqual(sum(p=={'action':'run'} for _,p in device.requests),1)
            profile.audit_profile({'collectionProfile':profile.collection_profile(profile.ATTENDED_ELBOW_CONTACT_FINE)},
                                  {'collectionProfile':profile.collection_profile(profile.ATTENDED_ELBOW_CONTACT_FINE)},self.pulse(duration),receipt)

    def test_bad_complete_response_stops_without_retry(self):
        mutations = [lambda v,d:v['status']['sensor'].update(blocked=True),
            lambda v,d:v['status']['outputProof'].update(stable=False),
            lambda v,d:v['status']['outputProof'].update(sequenceBefore=1),
            lambda v,d:v['outputs'].update(bootId='other'),
            lambda v,d:v['outputs'].update(gap=True),
            lambda v,d:v['outputs'].update(entries=[]),
            lambda v,d:v['outputs']['entries'][0].update(motor=2),
            lambda v,d:v['outputs']['entries'][0].update(pwm=154),
            lambda v,d:v['status']['motors']['M3'].update(speed=0),
            lambda v,d:v.update(schemaVersion='other')]
        for mutate in mutations:
            port,device,_=self.setup_port(mutate);port.preflight()
            with self.assertRaises(ValueError):
                port.execute(self.pulse(),lambda:None,monitor_policy=transport.ELBOW_CONTACT_COMBINED_200_POLICY)
            self.assertFalse(any(device.speeds.values()))
            self.assertEqual(sum(p=={'action':'run'} for _,p in device.requests),1)

    def test_single_response_exceeding_period_is_failure_not_success(self):
        port,device,_=self.setup_port(active_delay=.21);port.preflight()
        with self.assertRaises(TimeoutError):
            port.execute(self.pulse(),lambda:None,monitor_policy=transport.ELBOW_CONTACT_COMBINED_200_POLICY)
        self.assertFalse(any(device.speeds.values()))
        self.assertEqual(sum(p=={'action':'run'} for _,p in device.requests),1)

    def test_missing_endpoint_blocks_before_any_write_or_run(self):
        port,device,_=fixture.CompactTransportTests().setup_port();port.preflight();start=len(device.requests)
        with self.assertRaises(AssertionError):
            port.execute(self.pulse(),lambda:None,monitor_policy=transport.ELBOW_CONTACT_COMBINED_200_POLICY)
        self.assertFalse(any(body is not None for _,body in device.requests[start:]))

    def test_active_storage_deadline_failure_still_stops_and_flushes_evidence(self):
        port,device,store=self.setup_port();port.preflight();save=port._save
        def slow(name,value,emergency=False):
            if device.running and value.get('parsed',{}).get('schemaVersion')=='collection_observation.v1':device.clock.now+=.25
            return save(name,value,emergency)
        port._save=slow
        with self.assertRaises(TimeoutError):
            port.execute(self.pulse(),lambda:None,monitor_policy=transport.ELBOW_CONTACT_COMBINED_200_POLICY)
        self.assertFalse(any(device.speeds.values()))
        self.assertTrue(any(k.endswith('-late-response.json') for k in store.rows))


if __name__=='__main__':unittest.main()
