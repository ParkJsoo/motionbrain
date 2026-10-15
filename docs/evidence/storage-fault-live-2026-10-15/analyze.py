"""Verify the copied Pi fault experiment offline. No device requests."""
import hashlib
import json
from pathlib import Path
import sys

from fault_probe import MODES, verify_case
from learning.source_age import source_age_basis, source_age_upper_ms
from learning.fresh_capture import validate_fresh_capture
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def analyze(repo, experiment):
    def load(p):
        return json.loads(p.read_text())
    evidence = experiment/'fault-evidence'
    plan, before = load(evidence/'plan.json'), load(evidence/'before.json')
    assert plan['modes'] == list(MODES) and plan['requestedSamplesPerCase'] == 3
    assert plan['sourceAgeLimitMs'] == 100 and plan['maxPersistenceMs'] == 250 and plan['injectionDelayMs'] == 300
    assert plan['harnessSha256'] == hashlib.sha256(Path(__file__).with_name('fault_probe.py').read_bytes()).hexdigest()
    summaries = []
    runs = []
    for mode in MODES:
        run = experiment/'runs'/mode
        report, session = load(run/'run.json'), load(run/'session.json')
        for name, digest in session['sourceSha256'].items():
            assert hashlib.sha256((repo/name).read_bytes()).hexdigest() == digest, name
        audit = load(evidence/(mode+'-audit.json'))
        summary = verify_case(run, report, audit)
        capture = next(c for c in audit['calls'] if c['path']=='/capture')
        receipt = capture['receipt']
        source = {**parse_capture_headers(receipt['headers']), 'imageSha256': receipt['sha256']}
        assert source['sourceMetadataValid'] and validate_fresh_capture(receipt, source)
        row = report['consumptions'][0]
        assert source['sourceFrameId'] == row['sourceFrameId']
        assert source['cameraBootId'] == before['camera']['cameraBootId']
        basis = source_age_basis(receipt, source, request_started=receipt['requestStartedMonotonic'],
                                 received=receipt['receivedMonotonic'], host_clock_id=audit['hostClockId'])
        age = source_age_upper_ms(basis, source['reportedSourceAgeMs'], row['consumedMonotonic'])
        assert abs(age-row['sourceAgeAtConsumptionMs']) < 1e-6
        assert all(c['receipt']['method']=='GET' for c in audit['calls'])
        assert summary == load(evidence/(mode+'-verified.json'))
        after = load(evidence/(mode+'-after.json'))
        assert after == before
        summaries.append(summary)
        runs.append(dict(mode=mode, status=report['status'], error=report['error'],
            consumedObservations=report['consumedObservations'], completedSamples=report['completedSamples'],
            consumption=report['consumptions'][0],
            captureReceiptSha256=next(c['receipt']['sha256'] for c in audit['calls'] if c['path']=='/capture'),
            jpegCount=len(list((run/'images').rglob('*.jpg')))))
    assert load(evidence/'summary.json') == dict(cases=summaries, motorCommandsSent=0)
    return dict(sourceAgeLimitMs=100, maxPersistenceMs=250, motorCommandsSent=0,
                cases=summaries, runs=runs, clockPhysicallyCalibrated=False,
                injectedFaults=True, actualDiskHardwareFailure=False)


if __name__ == '__main__':
    print(json.dumps(analyze(*map(Path, sys.argv[1:3])), indent=2))
