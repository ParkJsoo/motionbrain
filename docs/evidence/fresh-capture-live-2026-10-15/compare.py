"""CAM GET-only comparison; run in isolated Pi checkout. Never sends commands."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import urllib.request

root = Path('/tmp/motionbrain-fresh-live-20261015')
sys.path.insert(0, str(root))
from learning.device_adapter import ReadOnlyHTTP
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers

http = ReadOnlyHTTP(controller_url='http://192.168.219.113', camera_url='http://192.168.219.116',
                    perception_url='http://127.0.0.1:8766')
def get(device, path):
    receipt, body = http.get(device, path)
    assert receipt['status'] == 200, receipt
    return json.loads(body)

def snapshot():
    c = get('controller', '/status')
    assert c['state'] == 'IDLE' and not c['motorEnabled']
    assert all(m['speed'] == 0 for m in c['motors'].values())
    return {'camera': get('camera', '/status'), 'outputs': get('controller', '/action_outputs'),
            'controller': {k: c[k] for k in ('state', 'motorEnabled', 'motors', 'lastCommand')},
            'perception': get('perception', '/health')}

before = snapshot()
(root/'comparison-before.json').write_text(json.dumps(before, indent=2))
assert before['camera']['freshCaptureSupported'] is True
boot = before['camera']['cameraBootId']
# Check both default UI URLs without changing camera settings. Images remain private.
checks = []
for i, path in enumerate(('/', '/capture', '/capture?t=commissioning')):
    with urllib.request.urlopen('http://192.168.219.116'+path, timeout=5) as response:
        raw = response.read(2*1024*1024+1)
        assert response.status == 200 and len(raw) <= 2*1024*1024
        headers = dict(response.headers)
    item = {'path': path, 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}
    if i:
        source = parse_capture_headers(headers)
        assert source['sourceMetadataValid'] and source['cameraBootId'] == boot, source
        (root/f'basic-{i}.jpg').write_bytes(raw)
        item['source'] = source
    checks.append(item)
(root/'basic-checks.json').write_text(json.dumps(checks, indent=2))

runs = []
for index, fresh in enumerate((False, True, True, False)):
    cmd = [sys.executable, str(root/'tools/probe_fast_observation.py'),
           '--controller-url', 'http://192.168.219.113', '--camera-url', 'http://192.168.219.116',
           '--output-dir', str(root/'runs'), '--samples', '12', '--capture-timing', '--connection-timing']
    if fresh:
        cmd.append('--fresh-capture')
    completed = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
    (root/f'probe-{index}.json').write_text(completed.stdout)
    (root/f'probe-{index}.stderr').write_text(completed.stderr)
    report = json.loads(completed.stdout)
    item = {'order': index, 'fresh': fresh, 'exitCode': completed.returncode,
            **{k: report[k] for k in ('runDir', 'status', 'error', 'completedSamples',
                                     'acceptedMeasurements', 'outputLogUnchanged', 'motorCommandsSent')}}
    runs.append(item)
    (root/'comparison-runs.json').write_text(json.dumps(runs, indent=2))
    print(json.dumps(item), flush=True)
    after = snapshot()
    (root/f'after-{index}.json').write_text(json.dumps(after, indent=2))
    assert after['outputs'] == before['outputs'], 'output log changed'
    assert after['camera']['cameraBootId'] == boot, 'camera reboot'
    assert completed.returncode == 0 and report['status'] == 'completed', 'stop after failed run'
    assert after['camera']['captureFailures'] == before['camera']['captureFailures'], 'capture failure'
    time.sleep(1)
(root/'comparison-after.json').write_text(json.dumps(after, indent=2))
