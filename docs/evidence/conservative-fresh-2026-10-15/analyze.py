"""Offline verification of original GET receipts, images and shadow barriers.

Usage: python analyze.py REPO COPIED_EXPERIMENT
No device requests. Raw images remain outside Git.
"""
import collections
import hashlib
import json
from pathlib import Path
import statistics
import sys


def analyze(repo, experiment):
    sys.path.insert(0, str(repo))
    from learning.artifacts import verify_artifacts, read_events, read_json_artifact
    from learning.fresh_capture import validate_fresh_capture
    from learning.observation_consumer import ShadowObservationConsumer, ConsumptionStopped
    from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers

    def load(path):
        return json.loads(path.read_text())

    def stats(values):
        return dict(n=len(values), min=min(values), median=statistics.median(values), max=max(values)) if values else None

    plan = load(experiment/'comparison-plan.json')
    entries = load(experiment/'comparison-runs.json')
    assert plan == dict(freshAgeModeOrder=["clock_margin", "send_start", "send_start", "clock_margin"], samplesPerRun=12,
                        sourceAgeLimitMs=100, maxPersistenceMs=250, retryFailedRuns=False)
    assert 1 <= len(entries) <= 4
    before = load(experiment/'comparison-before.json')
    groups = collections.defaultdict(list)
    output = dict(sourceAgeLimitMs=100, maxPersistenceMs=250, motorCommandsSent=0,
                  clockPhysicallyCalibrated=False, plannedRuns=4, completedRuns=0, runs=[], modes={})
    all_frames = set()
    for index, entry in enumerate(entries):
        assert entry['order'] == index and entry['fresh'] is True and entry['freshAgeMode'] == plan['freshAgeModeOrder'][index]
        after = load(experiment/f'after-{index}.json')
        assert after['camera'] == before['camera'] and after['outputs'] == before['outputs']
        assert after['controller']['state'] == 'IDLE' and after['controller']['motorEnabled'] is False
        assert all(m['speed'] == 0 and not m['enabled'] for m in after['controller']['motors'].values())
        for key in ('id', 'type', 'source', 'executedAtMs'):
            assert after['controller']['lastCommand'][key] == before['controller']['lastCommand'][key]
        run = experiment/'runs'/Path(entry['runDir']).name
        report, session = load(run/'run.json'), load(run/'session.json')
        assert report['outputLogUnchanged'] and report['motorCommandsSent'] == 0
        assert not report['controlEligible'] and not report['trainingEligible']
        assert not report['integrityErrors'] and not verify_artifacts(run, report['artifacts'])
        events, errors = read_events(run)
        assert not errors and events[-1]['kind'] == 'finished'
        assert read_json_artifact(run, events[-1]['payload']['report']) == report
        for path, digest in session['sourceSha256'].items():
            assert hashlib.sha256((repo/path).read_bytes()).hexdigest() == digest, path
        assert session['freshAgeMode'] == entry['freshAgeMode']
        assert session['shadowConsumptionEnabled'] and session['maxPersistenceMs'] == 250
        assert session['policy']['maxSourceAgeMs'] == 100 and session['freshCaptureRequested'] == entry['fresh']
        for key in ('status', 'error', 'completedSamples', 'acceptedMeasurements', 'consumedObservations', 'consumptionHalted'):
            assert entry[key] == report[key]
        receipts = [load(p) for p in sorted((run/'raw').glob('*.json'))]
        captures = [r for r in receipts if r.get('device') == 'camera' and r.get('path') == '/capture']
        rows = report['consumptions']
        # This verifier accepts complete frame barriers and a final late barrier.
        # Unconfirmed/incomplete evidence is intentionally not summarized as success.
        assert len(captures) == len(rows) == len(report['results']) == report['completedSamples']
        assert sum(e['kind'] == 'capture_requested' for e in events) == len(rows)
        stats_rows = []
        for i, (measurement, receipt, row) in enumerate(zip(report['results'], captures, rows)):
            source = {**parse_capture_headers(receipt['headers']), 'imageSha256': receipt['sha256']}
            assert source['sourceMetadataValid'] and receipt['status'] == 200
            assert source['cameraBootId'] == before['camera']['cameraBootId']
            assert source['sourceFrameId'] not in all_frames
            all_frames.add(source['sourceFrameId'])
            assert bool(validate_fresh_capture(receipt, source)) == entry['fresh']
            assert measurement['source'] == source
            image = (run/measurement['image']['artifactRef']).read_bytes()
            assert hashlib.sha256(image).hexdigest() == row['imageSha256'] == receipt['sha256']
            assert source['sourceFrameId'] == row['sourceFrameId'] == measurement['sourceFrameId']
            now = row['consumedMonotonic']
            class RecordedClock:
                def monotonic(self):
                    return now
            consumer = ShadowObservationConsumer(RecordedClock(), session['hostClockId'],
                fresh_age_mode=entry['freshAgeMode'])
            pending = consumer.consume(measurement, source, receipt, image)
            assert pending == load(run/f'consumption/{i:04d}.json')
            assert measurement['freshAgeMode'] == row['freshAgeMode'] == entry['freshAgeMode']
            now = row['frameEvidenceCompletedMonotonic']
            try:
                consumer.finish_persistence(row['sourceFrameId'], session['hostClockId'])
            except ConsumptionStopped:
                assert row['storageStatus'] == 'late' and i == len(rows)-1
            assert consumer.snapshot() == [row]
            assert row['storageStatus'] in ('completed', 'late')
            assert row['storageStatus'] == ('completed' if row['persistenceElapsedMs'] <= 250 else 'late')
            persistence = load(run/f'persistence/{i:04d}.json')
            assert receipt['receivedMonotonic'] <= measurement['measurementCompletedMonotonic'] <= row['consumedMonotonic']
            assert row['consumedMonotonic'] <= persistence['persistedMonotonic'] <= row['frameEvidenceCompletedMonotonic']
            if i+1 < len(rows):
                assert row['storageStatus'] == 'completed'
                assert captures[i+1]['requestStartedMonotonic'] >= row['frameEvidenceCompletedMonotonic']
            if row['consumed']:
                assert measurement['accepted'] and row['sourceAgeAtConsumptionMs'] <= 100
                assert row['anchorAgeAtConsumptionUpperMs'] <= 15000
            # Independently re-evaluate recorded time differences, beyond replay.
            if entry['freshAgeMode'] == 'send_start':
                assert row['sourceCaptureLowerBoundMonotonic'] == receipt['connectionTiming']['requestSendStartedMonotonic']
                assert row['sourceAgeBoundBasis'] == 'fresh_send_start'
                assert row['sourceAgeClockRateAssumptionUsed'] is False
                assert row['sourceAgeTimestampMarginUsed'] is False
            assert persistence['freshAgeMode'] == entry['freshAgeMode']
            if entry['fresh']:
                age = (row['consumedMonotonic']-row['sourceCaptureLowerBoundMonotonic'])*1000
            else:
                age = source['reportedSourceAgeMs']/.999+.002+(row['consumedMonotonic']-row['sourceAgeBoundStartedMonotonic'])*1000
            assert abs(age-row['sourceAgeAtConsumptionMs']) < 1e-6
            assert abs((row['frameEvidenceCompletedMonotonic']-row['consumedMonotonic'])*1000-row['persistenceElapsedMs']) < 1e-6
            stats_rows.append(dict(frameId=row['sourceFrameId'], measurementAccepted=measurement['accepted'],
                consumed=row['consumed'], consumptionReason=row['reason'], measurementReason=measurement.get('reason'),
                measurementAgeMs=measurement.get('reportedAgePlusElapsedMs'), consumptionAgeMs=age,
                consumeAfterMeasurementMs=(row['consumedMonotonic']-measurement['measurementCompletedMonotonic'])*1000,
                persistenceElapsedMs=row['persistenceElapsedMs'], storageStatus=row['storageStatus'],
                imageMeasurementPersistenceAgeMs=persistence['reportedAgeAtPersistenceMs'],
                imageMeasurementPersistenceWithin100ms=persistence['withinAgePolicyAtPersistence']))
        assert report['consumedObservations'] == sum(r['consumed'] for r in rows)
        if report['status'] == 'completed':
            assert entry['exitCode'] == 0 and len(rows) == 12 and not report['consumptionHalted']
            assert all(r['storageStatus'] == 'completed' for r in rows)
            output['completedRuns'] += 1
        else:
            assert entry['exitCode'] != 0 and report['consumptionHalted'] and index == len(entries)-1
            assert report['status'] == 'failed' and rows[-1]['storageStatus'] == 'late'
            stop = load(experiment/'comparison-stop.json')
            assert stop['order'] == index and stop['remainingRunsSkipped'] == 3-index
        mode = entry['freshAgeMode']
        groups[mode].extend(stats_rows)
        output['runs'].append(dict(runId=run.name, mode=mode, status=report['status'], error=report['error'],
            completedSamples=len(rows), consumed=report['consumedObservations'],
            artifactCount=len(report['artifacts']), eventCount=len(events),
            jpegCount=len(list((run/'images').rglob('*.jpg'))), samples=stats_rows))
    if len(entries) < 4:
        assert output['runs'][-1]['status'] == 'failed'
    output['allPlannedRunsCompleted'] = output['completedRuns'] == 4
    for mode, rows in groups.items():
        output['modes'][mode] = dict(samples=len(rows), measurementAccepted=sum(r['measurementAccepted'] for r in rows),
            consumed=sum(r['consumed'] for r in rows),
            consumptionReasons=dict(collections.Counter(r['consumptionReason'] for r in rows)),
            storageStatuses=dict(collections.Counter(r['storageStatus'] for r in rows)),
            imageMeasurementPersistenceWithin100ms=sum(r['imageMeasurementPersistenceWithin100ms'] for r in rows),
            statistics={key: stats([r[key] for r in rows if r[key] is not None]) for key in (
                'measurementAgeMs', 'consumptionAgeMs', 'consumeAfterMeasurementMs',
                'persistenceElapsedMs', 'imageMeasurementPersistenceAgeMs')})
    return output


if __name__ == '__main__':
    print(json.dumps(analyze(*map(Path, sys.argv[1:3])), indent=2))
