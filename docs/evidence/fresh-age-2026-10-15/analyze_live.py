"""Validate saved comparison artifacts and summarize without touching devices."""
import collections
import hashlib
import json
from pathlib import Path
import statistics
import sys

repo, experiment = map(Path, sys.argv[1:3])
sys.path.insert(0, str(repo))
from learning.artifacts import verify_artifacts, read_events, read_json_artifact
from learning.fresh_capture import validate_fresh_capture
from learning.source_age import source_age_basis, source_age_upper_ms
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers

def load(path):
    return json.loads(path.read_text())

def stats(values):
    return {'n': len(values), 'min': min(values), 'median': statistics.median(values), 'max': max(values)} if values else None

entries = load(experiment/'comparison-runs.json')
assert [e['fresh'] for e in entries] == [False, True, True, False]
before = load(experiment/'comparison-before.json')
for index in range(4):
    after = load(experiment/f'after-{index}.json')
    assert after['outputs'] == before['outputs']
    assert after['camera']['cameraBootId'] == before['camera']['cameraBootId']
    assert after['camera']['captureFailures'] == before['camera']['captureFailures']
    assert after['controller']['state'] == 'IDLE' and not after['controller']['motorEnabled']
    assert all(m['speed'] == 0 for m in after['controller']['motors'].values())
out = {'sourceAgeLimitMs': 100, 'motorCommandsSent': 0, 'runs': [], 'modes': {}}
groups = collections.defaultdict(list)
for entry in entries:
    run = experiment/'runs'/Path(entry['runDir']).name
    report, session = load(run/'run.json'), load(run/'session.json')
    assert report['status'] == 'completed' and report['completedSamples'] == 12
    assert report['outputLogUnchanged'] and report['motorCommandsSent'] == 0
    assert not report['controlEligible'] and not report['trainingEligible']
    assert not verify_artifacts(run, report['artifacts'])
    events, errors = read_events(run)
    assert not errors and events[-1]['kind'] == 'finished'
    assert read_json_artifact(run, events[-1]['payload']['report']) == report
    for path, digest in session['sourceSha256'].items():
        assert hashlib.sha256((repo/path).read_bytes()).hexdigest() == digest, path
    assert session['policy']['maxSourceAgeMs'] == 100
    assert session['freshCaptureRequested'] == entry['fresh']
    receipts = [load(p) for p in sorted((run/'raw').glob('*.json'))]
    captures = [r for r in receipts if r.get('device') == 'camera' and r.get('path') == '/capture']
    assert len(captures) == 12
    rows = []
    for i, (measurement, receipt) in enumerate(zip(report['results'], captures)):
        persistence, timing = load(run/f'persistence/{i:04d}.json'), load(run/f'timing/{i:04d}.json')
        source = parse_capture_headers(receipt['headers'])
        assert receipt['status'] == 200 and source['sourceMetadataValid'] and timing['valid']
        assert source['sourceFrameId'] == measurement['sourceFrameId'] == timing['sourceFrameId']
        assert receipt['sha256'] == measurement['imageSha256'] == measurement['image']['sha256']
        proof = validate_fresh_capture(receipt, source)
        assert bool(proof) == entry['fresh']
        b = source_age_basis(receipt, {**source, 'imageSha256': receipt['sha256']},
            request_started=receipt['requestStartedMonotonic'], received=receipt['receivedMonotonic'],
            host_clock_id=receipt['hostClockId'])
        valid_bound = 'sourceAgeBoundStartedMonotonic' in measurement
        old_age = old_persistence = None
        if valid_bound:
            assert abs(source_age_upper_ms(b, source['reportedSourceAgeMs'], measurement['measurementCompletedMonotonic'])
                       - measurement['reportedAgePlusElapsedMs']) < 1e-6
            assert abs(source_age_upper_ms(b, source['reportedSourceAgeMs'], persistence['persistedMonotonic'])
                       - persistence['reportedAgeAtPersistenceMs']) < 1e-6
            old_age = source['reportedSourceAgeMs'] + (measurement['measurementCompletedMonotonic']-b['sourceAgeBoundStartedMonotonic'])*1000
            old_persistence = source['reportedSourceAgeMs'] + (persistence['persistedMonotonic']-b['sourceAgeBoundStartedMonotonic'])*1000
        if proof and valid_bound:
            assert measurement['sourceAgeBoundBasis'] == 'fresh_capture_lower_bound'
            assert not measurement['freshClockErrorCalibrated']
        if measurement['accepted']:
            assert measurement['reportedAgePlusElapsedMs'] <= 100
        row = {'measurementAgeMs': measurement['reportedAgePlusElapsedMs'] if valid_bound else None,
               'oldSendStartAgeMs': old_age, 'oldSendStartPersistenceAgeMs': old_persistence,
               'oldSendStartAccepted': old_age is not None and old_age <= 100 and measurement['accepted'],
               'persistenceAgeMs': persistence['reportedAgeAtPersistenceMs'],
               'reportedSourceAgeMs': source['reportedSourceAgeMs'], 'httpMs': timing['requestTotalMs'],
               'processingMs': measurement['processingMs'], 'accepted': measurement['accepted'],
               'persistenceWithin100ms': persistence['withinAgePolicyAtPersistence'],
               'reason': measurement.get('reason'), 'serverAssociated': timing['serverStatsAssociated'],
               'serverSelectionMs': timing.get('serverFreshFrameSelectionMs', timing.get('serverFrameAcquireWaitMs')),
               'discarded': proof['discardedFrames'] if proof else None}
        rows.append(row)
    mode = 'fresh' if entry['fresh'] else 'legacy'
    groups[mode].extend(rows)
    out['runs'].append({'runId': run.name, 'mode': mode, 'order': entry['order'],
                        'artifactCount': len(report['artifacts']), 'eventCount': len(events),
                        'jpegCount': len(list((run/'images').rglob('*.jpg'))), 'samples': rows})
for mode, rows in groups.items():
    out['modes'][mode] = {'samples': len(rows), 'accepted': sum(r['accepted'] for r in rows),
        'oldSendStartAccepted': sum(r['oldSendStartAccepted'] for r in rows),
        'persistenceWithin100ms': sum(r['persistenceWithin100ms'] for r in rows),
        'reasons': dict(collections.Counter(r['reason'] for r in rows)),
        'statistics': {key: stats([r[key] for r in rows if r[key] is not None]) for key in (
            'measurementAgeMs', 'persistenceAgeMs', 'reportedSourceAgeMs', 'httpMs',
            'processingMs', 'serverSelectionMs', 'discarded', 'oldSendStartAgeMs', 'oldSendStartPersistenceAgeMs')}}
print(json.dumps(out, indent=2))
