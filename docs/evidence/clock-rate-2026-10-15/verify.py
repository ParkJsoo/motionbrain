"""Offline verification and clock report from the original receipts/JPEGs."""
import hashlib
import json
from pathlib import Path
import statistics
import sys

from clock_analysis import analyze, sample_from_receipt
from learning.artifacts import verify_artifacts, read_events, read_json_artifact


def verify(repo, experiment):
    def load(p): return json.loads(p.read_text())
    root=experiment/'clock-run'
    report,session=load(root/'run.json'),load(root/'session.json')
    assert report['status']=='completed' and report['completedSamples']==session['requestedSamples']==91
    assert report['outputLogUnchanged'] and report['motorCommandsSent']==0
    assert not report['controlEligible'] and not report['trainingEligible']
    assert not report['integrityErrors'] and not verify_artifacts(root,report['artifacts'])
    events,errors=read_events(root)
    assert not errors and events[-1]['kind']=='finished'
    assert read_json_artifact(root,events[-1]['payload']['report'])==report
    for name,digest in session['sourceSha256'].items():
        local=(Path(__file__).parent if '/' not in name else repo)/name
        assert hashlib.sha256(local.read_bytes()).hexdigest()==digest,name
    before=load(root/'before.json')
    assert load(root/'after.json')==before
    for path in (root/'status').glob('*.json'): assert load(path)==before
    samples=[]
    for i in range(91):
        receipt=load(root/f'receipts/{i:04d}.json')
        body=(root/f'images/{i}/source.jpg').read_bytes()
        assert hashlib.sha256(body).hexdigest()==receipt['sha256']
        assert body.startswith(b'\xff\xd8') and body.endswith(b'\xff\xd9')
        sample=sample_from_receipt(receipt)
        assert sample==load(root/f'samples/{i:04d}.json')==report['samples'][i]
        assert sample['cameraBootId']==before['camera']['cameraBootId']
        assert sample['hostClockId']==session['hostClockId']
        assert sample['fresh']==bool(i%2)
        samples.append(sample)
    def stats(vals): return dict(min=min(vals),median=statistics.median(vals),max=max(vals))
    return dict(allSamples=analyze(samples),zeroTimestampMarginSensitivity=analyze(samples,delta_error_us=0),
        modes={name:analyze([s for s in samples if s['fresh']==fresh])
               for name,fresh in [('legacy',False),('fresh',True)]},
        bracketWidthMs={name:stats([s['bracketWidthMs'] for s in samples if s['fresh']==fresh])
                        for name,fresh in [('legacy',False),('fresh',True)]},
        artifactCount=len(report['artifacts']),eventCount=len(events),jpegCount=91,
        motorCommandsSent=0,sourceAgeLimitMs=100,selectionUsedRateAssumption=False)


if __name__=='__main__': print(json.dumps(verify(*map(Path,sys.argv[1:3])),indent=2))
