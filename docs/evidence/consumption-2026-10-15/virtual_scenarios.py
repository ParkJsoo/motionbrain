"""Device-free evidence from the same synthetic fixture and production probe."""
import json
from pathlib import Path
import sys
from unittest.mock import patch

repo = Path(sys.argv[1])
sys.path[:0] = [str(repo), str(repo/'tests')]
from learning.artifacts import ArtifactStore, read_events, verify_artifacts
from learning.fast_observation import AnchoredRoiMeasurement
from test_fast_observation import FastProbeTest

output = []
for scenario in ('normal', 'consume_delayed', 'storage_120ms', 'storage_251ms', 'image_failure', 'receipt_interrupt'):
    fixture = FastProbeTest('test_originals_hashes_results_and_dataset_exclusion')
    fixture.setUp()
    image, write, measure = ArtifactStore.save_image, ArtifactStore.write_json, AnchoredRoiMeasurement.measure
    def store_image(store, index, phase, body, digest):
        if index != 'anchor':
            if scenario == 'image_failure': raise OSError('virtual image failure')
            if scenario.startswith('storage_'):
                fixture.transport.clock.advance(.120 if scenario == 'storage_120ms' else .251)
        return image(store, index, phase, body, digest)
    def store_json(store, path, value):
        if scenario == 'receipt_interrupt' and value.get('path') == '/capture': raise KeyboardInterrupt()
        return write(store, path, value)
    def measurement(*args, **kwargs):
        value = measure(*args, **kwargs)
        if scenario == 'consume_delayed': fixture.transport.clock.advance(.100)
        return value
    try:
        with patch.object(ArtifactStore, 'save_image', store_image), patch.object(ArtifactStore, 'write_json', store_json), patch.object(AnchoredRoiMeasurement, 'measure', measurement):
            report = fixture.run_probe(shadow_consumption=True)
        assert report['motorCommandsSent'] == 0 and report['outputLogUnchanged']
        assert not verify_artifacts(fixture.root, report['artifacts'])
        events, errors = read_events(fixture.root)
        assert not errors and events[-1]['kind'] == 'finished'
        output.append({'scenario': scenario, 'status': report['status'], 'error': report['error'],
            'captureCount': fixture.capture_count, 'acceptedMeasurements': report['acceptedMeasurements'],
            'consumedObservations': report['consumedObservations'], 'halted': report['consumptionHalted'],
            'motorCommandsSent': 0, 'frames': [{k: row.get(k) for k in ('consumed', 'reason',
                'sourceAgeAtConsumptionMs', 'storageStatus', 'persistenceElapsedMs', 'storageError')}
                for row in report['consumptions']]})
    finally:
        fixture.doCleanups()
print(json.dumps({'mode': 'synthetic_virtual_time', 'physicalValidation': False,
                  'sourceAgeLimitMs': 100, 'storageBudgetMs': 250, 'scenarios': output}, indent=2))
