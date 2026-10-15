"""Scoped fault injection for GET-only shadow probes; never an actuator path.

Run from an isolated checkout: python fault_probe.py CHECKOUT.
Each fault is a separate planned run, not a retry of an interrupted run.
"""
import copy
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

# Also allow importing the harness for offline fixture tests.
REPO = next(p for p in Path(__file__).resolve().parents if (p/'learning').is_dir())
sys.path.insert(0, str(REPO))
from learning.artifacts import ArtifactStore, read_events, read_json_artifact, verify_artifacts
from learning.device_adapter import ReadOnlyHTTP
from learning.fast_probe import run_fast_probe

MODES = ('image_delay', 'image_failure', 'receipt_failure')


class AuditedTransport:
    """Record attempted GETs in memory independently of injected receipt writes."""
    def __init__(self, transport):
        self.transport = transport
        self.calls = []

    def __getattr__(self, name):
        return getattr(self.transport, name)

    @property
    def recorder(self):
        return self.transport.recorder

    @recorder.setter
    def recorder(self, value):
        self.transport.recorder = value

    def get(self, device, path, query=None):
        call = dict(device=device, path=path, query=copy.deepcopy(query),
                    attemptedMonotonic=self.clock.monotonic())
        self.calls.append(call)
        try:
            receipt, body = self.transport.get(device, path, query)
            call['receipt'] = copy.deepcopy(receipt)
            return receipt, body
        finally:
            call['returnedMonotonic'] = self.clock.monotonic()


class FaultStore(ArtifactStore):
    """Fault affects this instance's first captured frame only, never anchor/terminal."""
    def __init__(self, path, mode, *, clock=time, sleep=time.sleep):
        if mode not in MODES:
            raise ValueError('unknown fault mode')
        super().__init__(path)
        self.mode, self.clock, self.sleep = mode, clock, sleep
        self.injections = []

    def inject(self):
        if self.injections:
            raise RuntimeError('unexpected repeated injection')
        row = dict(mode=self.mode, startedMonotonic=self.clock.monotonic())
        self.injections.append(row)
        if self.mode == 'image_delay':
            self.sleep(.300)
            row['returnedMonotonic'] = self.clock.monotonic()
        else:
            row['raisedMonotonic'] = self.clock.monotonic()
            raise OSError('injected_'+self.mode)

    def save_image(self, transition_id, phase, frame, expected_sha256):
        if transition_id == '0' and self.mode in ('image_delay', 'image_failure'):
            self.inject()
        return super().save_image(transition_id, phase, frame, expected_sha256)

    def write_json(self, relative, data):
        if (self.mode == 'receipt_failure' and relative.startswith('raw/')
                and data.get('device') == 'camera' and data.get('path') == '/capture'):
            self.inject()
        return super().write_json(relative, data)


def verify_case(root, report, audit):
    """Verify partial-run evidence; missing image/receipt is intentional, not success."""
    mode = audit['mode']
    assert mode in MODES and len(audit['injections']) == 1
    assert report['status'] == 'failed' and report['consumptionHalted']
    assert report['outputLogUnchanged'] and report['motorCommandsSent'] == 0
    assert not report['controlEligible'] and not report['trainingEligible']
    assert not report['integrityErrors'] and not verify_artifacts(root, report['artifacts'])
    events, errors = read_events(root)
    assert not errors and events[-1]['kind'] == 'finished'
    assert read_json_artifact(root, events[-1]['payload']['report']) == report
    assert report['requestedSamples'] == 3
    session = json.loads((root/'session.json').read_text())
    assert session['policy']['maxSourceAgeMs'] == 100 and session['maxPersistenceMs'] == 250
    assert session['shadowConsumptionEnabled'] and session['freshCaptureRequested']
    assert audit['hostClockId'] == session['hostClockId']
    assert sum(e['kind'] == 'capture_requested' for e in events) == 1
    captures = [c for c in audit['calls'] if c['device']=='camera' and c['path']=='/capture']
    assert len(captures) == 1 and captures[0]['query'] == {'fresh': '1'}
    capture = captures[0]
    injection = audit['injections'][0]
    assert capture['returnedMonotonic'] <= injection['startedMonotonic'] <= audit['probeReturnedMonotonic']
    assert not any(c['attemptedMonotonic'] >= injection['startedMonotonic'] for c in captures)
    assert len(report['consumptions']) == 1
    row = report['consumptions'][0]
    assert capture['returnedMonotonic'] <= row['consumedMonotonic'] <= injection['startedMonotonic']
    assert row['imageSha256'] == capture['receipt']['sha256']
    assert row['hostClockId'] == audit['hostClockId']
    assert row['consumed'] == (report['consumedObservations'] == 1)
    assert report['consumedObservations'] in (0, 1)
    assert not row['controlEligible'] and not row['trainingEligible']
    if row['consumed']:
        assert row['sourceAgeAtConsumptionMs'] <= 100 and row['anchorAgeAtConsumptionUpperMs'] <= 15000
    image_path = root/'images/0/source.jpg'
    receipt_files = [p for p in (root/'raw').glob('*.json') if json.loads(p.read_text()).get('path') == '/capture']
    image_present = image_path.exists()
    assert image_present == (mode != 'image_failure')
    if image_present:
        assert hashlib.sha256(image_path.read_bytes()).hexdigest() == row['imageSha256']
        assert any(a['artifactRef']=='images/0/source.jpg' for a in report['artifacts'])
    assert len(receipt_files) == (0 if mode == 'receipt_failure' else 1)
    if mode == 'image_delay':
        assert report['completedSamples'] == 1 and row['storageStatus'] == 'late'
        assert row['persistenceElapsedMs'] > 250
        assert injection['returnedMonotonic']-injection['startedMonotonic'] >= .300-1e-9
        assert row['frameEvidenceCompletedMonotonic'] <= audit['probeReturnedMonotonic']
        assert 'persistence_deadline_or_consumption_evidence' in report['error']
    else:
        assert report['completedSamples'] == 0 and row['storageStatus'] == 'unconfirmed'
        assert 'injected_'+mode in report['error']
        assert 'frameEvidenceCompletedMonotonic' not in row
    assert audit['imageWorkersAlive'] == []
    assert audit['settledMonotonic'] >= audit['probeReturnedMonotonic']
    return dict(mode=mode, runId=root.name, consumed=row['consumed'],
                consumptionAgeMs=row.get('sourceAgeAtConsumptionMs'), storageStatus=row['storageStatus'],
                persistenceElapsedMs=row.get('persistenceElapsedMs'),
                injectionDelayMs=((injection['returnedMonotonic']-injection['startedMonotonic'])*1000
                                  if mode=='image_delay' else None),
                captureAttempts=1, captureAttemptsAfterInjection=0, remainingRequestedCapturesSkipped=2,
                imagePreserved=image_present, receiptPreserved=bool(receipt_files), terminalVerified=True,
                artifactCount=len(report['artifacts']), eventCount=len(events), motorCommandsSent=0)


def run_case(path, mode, transport, *, sleep=time.sleep):
    audited = AuditedTransport(transport)
    store = FaultStore(path, mode, clock=transport.clock, sleep=sleep)
    report = run_fast_probe(store, audited, samples=3, interval=.1, sleep=sleep,
                            fresh_capture=True, shadow_consumption=True, max_persistence_ms=250,
                            capture_timing=True)
    returned = transport.clock.monotonic()
    sleep(.5)  # No callbacks/retries; check for a stray worker before accepting evidence.
    audit = dict(mode=mode, hostClockId=transport.host_clock_id, calls=audited.calls,
                 injections=store.injections, probeReturnedMonotonic=returned,
                 settledMonotonic=transport.clock.monotonic(),
                 imageWorkersAlive=[t.name for t in threading.enumerate() if t.name.startswith('capture-image')])
    return report, audit


def snapshot(http):
    def get(device, path):
        receipt, body = http.get(device, path)
        assert receipt['status'] == 200
        return json.loads(body)
    c, cam, outputs = get('controller', '/status'), get('camera', '/status'), get('controller', '/action_outputs')
    assert c['state']=='IDLE' and c['motorEnabled'] is False
    assert all(m['speed']==0 and not m['enabled'] for m in c['motors'].values())
    assert cam['freshCaptureSupported'] is True
    return dict(controller={k: c[k] for k in ('state','motorEnabled','motors')},
                lastCommand={k:c['lastCommand'][k] for k in ('id','type','source','executedAtMs')},
                camera={k:cam[k] for k in ('cameraBootId','captureFailures','cameraRecoveries','freshCaptureSupported')},
                outputs=outputs)


def main(root):
    evidence = ArtifactStore(root/'fault-evidence')  # Refuse overwrite/re-run of the experiment.
    before = snapshot(make_http())
    evidence.write_json('before.json', before)
    evidence.write_json('plan.json', dict(modes=list(MODES), requestedSamplesPerCase=3,
        sourceAgeLimitMs=100, maxPersistenceMs=250, injectionDelayMs=300,
        harnessSha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()))
    summaries = []
    for mode in MODES:
        path = root/'runs'/mode
        try:
            report, audit = run_case(path, mode, make_http())
            evidence.write_json(mode+'-audit.json', audit)
        finally:
            after = snapshot(make_http())
            evidence.write_json(mode+'-after.json', after)
        assert after == before, 'unexpected device/output change; stop whole experiment'
        summary = verify_case(path, report, audit)  # Unexpected outcome never starts next case.
        summaries.append(summary)
        evidence.write_json(mode+'-verified.json', summary)
        print(json.dumps(summary), flush=True)
    evidence.write_json('summary.json', dict(cases=summaries, motorCommandsSent=0))


def make_http():
    return ReadOnlyHTTP(controller_url='http://192.168.219.113', camera_url='http://192.168.219.116',
                        perception_url='http://127.0.0.1:8766', connection_timing=True)


if __name__ == '__main__':
    root = Path(sys.argv[1]).resolve()
    assert root == REPO, 'run only in the explicitly staged isolated checkout'
    main(root)
