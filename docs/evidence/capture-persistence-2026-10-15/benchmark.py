"""Offline same-original serial/parallel durable storage comparison; no devices."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from learning.artifacts import ArtifactStore, verify_artifacts
from learning.fast_probe import _persist_capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    src = args.source_dir
    report = json.loads((src/'run.json').read_text())
    assert not verify_artifacts(src, report['artifacts'])
    receipts = {}
    for p in (src/'raw').glob('*.json'):
        r = json.loads(p.read_text())
        if r.get('path') == '/capture': receipts[r['headers']['x-motionbrain-frame-id']] = r
    base = args.output_dir/uuid.uuid4().hex
    rows = []
    for index, measurement in enumerate(report['results']):
        body = (src/measurement['image']['artifactRef']).read_bytes()
        receipt = receipts[measurement['sourceFrameId']]
        assert hashlib.sha256(body).hexdigest() == receipt['sha256'] == measurement['imageSha256']
        for mode in (('serial','parallel') if index%2==0 else ('parallel','serial')):
            store = ArtifactStore(base/f'{index}-{mode}')
            # Same steady-state directories as the live probe after anchor/first measurement.
            store.write_json('raw/preflight.json', {})
            store.save_image('anchor', 'source', body, receipt['sha256'])
            store.write_json('measurements/warm.json', {})
            prep = time.monotonic()
            if mode == 'parallel':
                store.prepare_parent('images/0/source.jpg')
                store.prepare_parent('measurements/0000.json')
            prep_ms = (time.monotonic()-prep)*1000
            refs = []
            def record(r): refs.append(store.write_json('raw/capture.json', r))
            start = time.monotonic()
            image, timing = _persist_capture(store, 0, receipt, body, [receipt], record, refs,
                                             clock=time, parallel=mode=='parallel')
            begun = time.monotonic()
            refs.append(store.write_json('measurements/0000.json', {**measurement, 'image':image}))
            end = time.monotonic()
            assert not verify_artifacts(store.root, refs)
            rows.append(dict(sourceFrameId=measurement['sourceFrameId'], imageSha256=receipt['sha256'],
                mode=mode, prepareMs=prep_ms, storageMs=(end-start)*1000,
                measurementWriteMs=(end-begun)*1000, **timing))
    print(json.dumps(dict(runDir=str(base), rows=rows, sourceSha256={
        p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (
        'learning/artifacts.py','learning/fast_probe.py',
        'docs/evidence/capture-persistence-2026-10-15/benchmark.py')}), indent=2))

if __name__ == '__main__': main()
