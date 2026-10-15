"""Replay saved images using recorded completion times, not new Mac timings."""
import collections
import hashlib
import json
from pathlib import Path
import statistics
import sys
from unittest.mock import Mock

repo = Path(sys.argv[1])
sys.path.insert(0, str(repo))
from learning.artifacts import read_events, read_json_artifact, verify_artifacts
from learning.fast_observation import AnchoredRoiMeasurement
from learning.source_age import source_age_upper_ms

RUNS = [('legacy', '1bdd4cb50cf5474ab1a123e822a4fe63'), ('fresh', '675bb1c66e724f638aa6657133b691b4'),
        ('fresh', 'cc0f7996d2f8476cbbe1c6bf70eb76e5'), ('legacy', '8db28e2255d543108735b768eee9afb2')]

def load(path): return json.loads(path.read_text())

rows = []
counts = collections.Counter()
for mode, run_id in RUNS:
    root = repo/'datasets/fast_probes'/run_id
    report, anchor = load(root/'run.json'), load(root/'anchor.json')
    assert report['status'] == 'completed' and report['completedSamples'] == 12
    assert not verify_artifacts(root, report['artifacts'])
    events, errors = read_events(root)
    assert not errors and events[-1]['kind'] == 'finished'
    assert read_json_artifact(root, events[-1]['payload']['report']) == report
    counts.update(artifacts=len(report['artifacts']), events=len(events), jpeg=len(list((root/'images').rglob('*.jpg'))))
    receipts = [load(p) for p in sorted((root/'raw').glob('*.json'))]
    captures = [r for r in receipts if r.get('device') == 'camera' and r.get('path') == '/capture']
    assert len(captures) == 12
    clock = Mock()
    tracker = AnchoredRoiMeasurement(anchor['observation'], (root/anchor['image']['artifactRef']).read_bytes(), clock=clock)
    for i, (old, receipt) in enumerate(zip(report['results'], captures)):
        body = (root/old['image']['artifactRef']).read_bytes()
        assert hashlib.sha256(body).hexdigest() == receipt['sha256'] == old['imageSha256']
        finished = old['measurementCompletedMonotonic']
        started = old.get('measurementStartedMonotonic', finished-old['processingMs']/1000)
        clock.monotonic.side_effect = [started, finished, finished]
        new = tracker.measure(body, old['source'], request_started=receipt['requestStartedMonotonic'],
                              received=receipt['receivedMonotonic'], host_clock_id=receipt['hostClockId'],
                              capture_receipt=receipt)
        for key in ('score', 'peakMargin', 'bestScaleProbe'):
            if key in old:
                assert abs(old[key]-new[key]) < 1e-5, (run_id, i, key, old[key], new[key])
        if old.get('reason') not in (None, 'source_age_exceeds_policy'):
            assert new.get('reason') == old['reason'] and not new['accepted']
        p = load(root/f'persistence/{i:04d}.json')
        new_persistence = (source_age_upper_ms(new, old['source']['reportedSourceAgeMs'], p['persistedMonotonic'])
                           if 'sourceAgeBoundStartedMonotonic' in new else None)
        if mode == 'legacy':
            assert new['accepted'] == old['accepted']
            assert abs(new['reportedAgePlusElapsedMs']-old['reportedAgePlusElapsedMs']) < 1e-6
        rows.append({'mode': mode, 'runId': run_id, 'index': i, 'sourceFrameId': old['sourceFrameId'],
                     'imageSha256': old['imageSha256'], 'oldAccepted': old['accepted'], 'newAccepted': new['accepted'],
                     'oldAgeMs': old['reportedAgePlusElapsedMs'] if 'sourceAgeBoundStartedMonotonic' in new else None,
                     'newAgeMs': new['reportedAgePlusElapsedMs'] if 'sourceAgeBoundStartedMonotonic' in new else None,
                     'oldPersistenceAgeMs': p['reportedAgeAtPersistenceMs'], 'newPersistenceAgeMs': new_persistence,
                     'oldPersistenceWithin100ms': p['withinAgePolicyAtPersistence'],
                     'newPersistenceWithin100ms': new_persistence is not None and 0 <= new_persistence <= 100,
                     'reason': new.get('reason'), 'clockRateErrorPpm': new.get('freshClockRateErrorPpm'),
                     'timestampErrorMs': new.get('freshTimestampErrorMs')})

summary = {}
for mode in ('legacy', 'fresh'):
    group = [r for r in rows if r['mode'] == mode]
    summary[mode] = {'samples': len(group), 'validAgeSamples': sum(r['newAgeMs'] is not None for r in group),
        **{k: sum(r[k] for r in group) for k in (
        'oldAccepted', 'newAccepted', 'oldPersistenceWithin100ms', 'newPersistenceWithin100ms')},
        'mediansMs': {k: statistics.median([r[k] for r in group if r[k] is not None]) for k in (
            'oldAgeMs', 'newAgeMs', 'oldPersistenceAgeMs', 'newPersistenceAgeMs')}}
print(json.dumps({'mode': 'offline_replay_recorded_times_not_a_speed_benchmark', 'counts': dict(counts),
                  'sourceAgeLimitMs': 100, 'motorCommandsSent': 0, 'summary': summary, 'rows': rows}, indent=2))
