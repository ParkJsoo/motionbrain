"""Bounded GET-only latency comparison. No camera or control entry point.

The direct transport uses a process-wide real-time alarm in the main thread;
refuse to replace an existing alarm. Socket timeouts alone cannot bound a drip
response. Each call closes its connection and never redirects or retries.
"""
import contextlib
import base64
import hashlib
import http.client
import signal
import subprocess
import threading
import time
import uuid
from pathlib import Path
from urllib.parse import urlsplit

from learning.artifacts import ArtifactStore, StorageError, _decode_json
from learning.mac_controller_probe import ENDPOINTS, ProbeRejected, _check, read_controller
from tools.diagnose_controller_http import validate_target

POLICY = dict(maxRequests=14, samplesPerTransport=3, requestSeconds=.8,
              sessionSeconds=12, intervalSeconds=.025, motorRequests=0, retries=0)


@contextlib.contextmanager
def deadline(seconds):
    if threading.current_thread() is not threading.main_thread() or signal.getitimer(signal.ITIMER_REAL) != (0., 0.):
        raise ValueError('main_thread_without_existing_alarm_required')
    previous=signal.getsignal(signal.SIGALRM)
    def expired(signum, frame):
        raise TimeoutError('request_deadline')
    signal.signal(signal.SIGALRM,expired)
    signal.setitimer(signal.ITIMER_REAL,seconds)
    try:
        yield
    finally:
        signal.setitimer(signal.ITIMER_REAL,0)
        signal.signal(signal.SIGALRM,previous)


def direct_get(url, endpoint, *, clock=time):
    url=validate_target(url)
    if endpoint not in ENDPOINTS:raise ValueError('read_only_endpoint_required')
    address=urlsplit(url)
    start=clock.monotonic()
    row=dict(transport='python_http_client',transportCompleted=False,method='GET',endpoint=endpoint,
             requestStartedMonotonic=start,clockDomain='python_monotonic',timing={})
    connection=http.client.HTTPConnection(address.hostname,address.port or 80,timeout=.8)
    body=bytearray()
    try:
        with deadline(.8):
            connection.connect()
            row['timing']['connectedMonotonic']=clock.monotonic()
            # No preconnected idle socket is held across requests.
            row['timing']['sendStartedMonotonic']=clock.monotonic()
            connection.request('GET','/'+endpoint,headers={'Connection':'close','Accept':'application/json'})
            row['timing']['sentMonotonic']=clock.monotonic()
            response=connection.getresponse()
            row['timing']['headersMonotonic']=clock.monotonic()
            row['httpStatus']=str(response.status)
            while len(body)<=65536:
                chunk=response.read1(min(4096,65537-len(body)))
                if not chunk:break
                body.extend(chunk)
            if len(body)>65536:raise ValueError('response_too_large')
            if response.length not in (None,0) or not response.isclosed():
                raise ValueError('incomplete_http_body')
            row.update(response=_decode_json(bytes(body)),transportCompleted=True)
    except (OSError, ValueError, http.client.HTTPException) as exc:
        row['error']=type(exc).__name__
        if isinstance(exc,http.client.IncompleteRead):body.extend(exc.partial[:max(0,65537-len(body))])
    except RecursionError:
        row['error']='invalid_json_depth'
    finally:
        connection.close()
        row.update(responseBodyBase64=base64.b64encode(body).decode('ascii'),
                   bodySha256=hashlib.sha256(body).hexdigest())
        try:row['responseBody']=body.decode('utf-8')
        except UnicodeError:row['responseBody']=None
        row['receivedMonotonic']=clock.monotonic()
        row['elapsedMs']=(row['receivedMonotonic']-start)*1000
    return row


def collect(output, url, *, direct=direct_get, curl=read_controller, clock=time):
    url=validate_target(url)
    store=ArtifactStore(Path(output));refs=[];rows=[];start=clock.monotonic()
    host_id=uuid.uuid4().hex
    def save(path,value):
        begun=clock.monotonic();ref=store.write_json(path,value);refs.append(ref)
        if clock.monotonic()-begun>.25:raise StorageError('persistence_exceeds_250ms')
        return ref
    def get(endpoint,phase,transport):
        if len(rows)>=14 or clock.monotonic()-start>=12:raise ProbeRejected('observation_budget')
        started=clock.monotonic()
        try:
            row=(direct if transport=='direct' else curl)(url,endpoint,clock=clock)
        except (OSError,subprocess.TimeoutExpired) as exc:
            ended=clock.monotonic()
            row=dict(endpoint=endpoint,method='GET',transport='python_http_client' if transport=='direct' else 'curl',
                     error=type(exc).__name__,requestStartedMonotonic=started,receivedMonotonic=ended,
                     elapsedMs=(ended-started)*1000,clockDomain='python_monotonic')
            for name in ('output','stderr'):
                partial=getattr(exc,name,None)
                if isinstance(partial,bytes):row[name+'Base64']=base64.b64encode(partial).decode('ascii')
        row.update(phase=phase,hostClockId=host_id)
        row.setdefault('transport','curl')
        rows.append(row);save(f'http/{len(rows):02d}.json',row)
        return _check(row,endpoint)
    save('plan.json',dict(policy=POLICY,url=url,hostClockId=host_id,mode='read_only_latency_comparison'))
    result=dict(status='rejected',failure=None,trainingEligible=False,controlEligible=False,motorRequests=0,
                clockDomainsMerged=False,startedMonotonic=start,hostClockId=host_id)
    try:
        baseline={e:get(e,'baseline','direct') for e in ENDPOINTS}
        if baseline['actions']['bootId']!=baseline['action_outputs']['bootId']:raise ProbeRejected('boot_mismatch')
        for transport in ('curl','direct')*3:
            get('status','comparison',transport)
            clock.sleep(.025)
        final={e:get(e,'final','direct') for e in ENDPOINTS}
        if final['actions']['bootId']!=baseline['actions']['bootId'] or final['action_outputs']!=baseline['action_outputs']:
            raise ProbeRejected('controller_boot_or_output_changed')
        if clock.monotonic()-start>12:raise ProbeRejected('observation_budget')
        result.update(status='recorded',outputHistoryUnchanged=True)
    except (ProbeRejected,OSError,ValueError,subprocess.TimeoutExpired) as exc:
        result['failure']=str(exc) if isinstance(exc,ProbeRejected) else type(exc).__name__
    result.update(endedMonotonic=clock.monotonic(),requestCount=len(rows),rows=rows)
    result['artifacts']=list(refs)
    summary=save('result.json',result);store.append('latency_sealed','observation',{'summary':summary})
    return result


def summarize(result):
    """Diagnostics only; reported integer sensor ages retain 1ms quantization."""
    import statistics
    out={}
    for transport in ('curl','python_http_client'):
        rows=[r for r in result['rows'] if r['phase']=='comparison' and r['transport']==transport]
        values=[]
        valid=[]
        for row in rows:
            try:_check(row,'status')
            except ProbeRejected:continue
            valid.append(row)
            values.append(row['elapsedMs']+row['response']['shoulderAngle']['ageMs']+1)
        out[transport]=dict(samples=len(rows),validSamples=len(valid),rttMs=[r['elapsedMs'] for r in valid],
                            ageUpperDiagnosticMs=values,within20ms=sum(v<=20 for v in values),
                            medianRttMs=statistics.median([r['elapsedMs'] for r in valid]) if valid else None)
    return dict(status=result['status'],transports=out,trainingEligible=False,controlEligible=False,
                sensorPhysicalSampleTimeVerified=False)
