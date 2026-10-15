"""GET-only live comparison in an isolated Pi checkout. Stop on first failed run.

Usage: python compare.py ISOLATED_CHECKOUT
Never changes services, camera settings, firmware or actuator state.
"""
import json
from pathlib import Path
import subprocess
import sys
import time

PLAN = (True, False, False, True)


def snapshot(http):
    def get(device, path):
        receipt, body = http.get(device, path)
        assert receipt['status'] == 200
        return json.loads(body)
    controller = get('controller', '/status')
    assert controller['state'] == 'IDLE' and controller['motorEnabled'] is False
    assert all(m['speed'] == 0 and not m['enabled'] for m in controller['motors'].values())
    camera = get('camera', '/status')
    perception = get('perception', '/health')
    assert perception['ok'] and perception['fresh']
    return {'camera': {k: camera[k] for k in (
                'cameraBootId', 'freshCaptureSupported', 'captureFailures', 'cameraRecoveries')},
            'outputs': get('controller', '/action_outputs'),
            'controller': {k: controller[k] for k in ('state', 'motorEnabled', 'motors', 'lastCommand')},
            'perception': {k: perception[k] for k in ('ok', 'fresh', 'consecutiveErrors', 'lastError')}}


def compare(root, http):
    def save(name, obj):
        (root/name).write_text(json.dumps(obj, indent=2)+'\n')
    before = snapshot(http)
    save('comparison-before.json', before)
    assert before['camera']['freshCaptureSupported'] is True
    runs = []
    save('comparison-plan.json', {'freshOrder': list(PLAN), 'samplesPerRun': 12,
         'sourceAgeLimitMs': 100, 'maxPersistenceMs': 250, 'retryFailedRuns': False})
    for index, fresh in enumerate(PLAN):
        cmd = [sys.executable, str(root/'tools/probe_fast_observation.py'),
               '--controller-url', 'http://192.168.219.113', '--camera-url', 'http://192.168.219.116',
               '--output-dir', str(root/'runs'), '--samples', '12', '--capture-timing',
               '--connection-timing', '--shadow-consumption', '--max-persistence-ms', '250']
        if fresh:
            cmd.append('--fresh-capture')
        try:
            # A timeout is an unknown outcome: never retry/continue the sequence.
            completed = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
            (root/f'probe-{index}.json').write_text(completed.stdout)
            (root/f'probe-{index}.stderr').write_text(completed.stderr)
            report = json.loads(completed.stdout)
            item = {'order': index, 'fresh': fresh, 'exitCode': completed.returncode,
                    **{k: report[k] for k in ('runDir', 'status', 'error', 'completedSamples',
                        'acceptedMeasurements', 'consumedObservations', 'consumptionHalted',
                        'outputLogUnchanged', 'motorCommandsSent')}}
            runs.append(item)
            save('comparison-runs.json', runs)
            print(json.dumps(item), flush=True)
        finally:
            # Status GETs only; no new capture after failure, even outside probe.
            after = snapshot(http)
            save(f'after-{index}.json', after)
            save('comparison-after.json', after)
        assert after['outputs'] == before['outputs'], 'output log changed'
        assert after['camera'] == before['camera'], 'camera reboot/failure/recovery/capability change'
        for key in ('id', 'type', 'source', 'executedAtMs'):
            assert after['controller']['lastCommand'][key] == before['controller']['lastCommand'][key]
        assert report['outputLogUnchanged'] and report['motorCommandsSent'] == 0
        if completed.returncode != 0 or report['status'] != 'completed' or report['consumptionHalted']:
            save('comparison-stop.json', {'order': index, 'reason': report['error'],
                 'remainingRunsSkipped': len(PLAN)-index-1})
            return False
        assert report['completedSamples'] == 12
        time.sleep(1)
    return True


if __name__ == '__main__':
    root = Path(sys.argv[1]).resolve()
    sys.path.insert(0, str(root))
    from learning.device_adapter import ReadOnlyHTTP
    http = ReadOnlyHTTP(controller_url='http://192.168.219.113', camera_url='http://192.168.219.116',
                        perception_url='http://127.0.0.1:8766')
    raise SystemExit(0 if compare(root, http) else 2)
