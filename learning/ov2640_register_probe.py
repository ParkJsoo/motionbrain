"""Finite register diagnostics only; never qualify exposure or learning data."""
import base64
import contextlib
import hashlib
import http.client
import math
import re
import signal
import time
import uuid
from pathlib import Path
from urllib.parse import urlsplit

from learning.artifacts import (ArtifactStore, StorageError, _decode_json,
                                read_json_artifact)
from learning.controller_latency import deadline
from tools.diagnose_controller_http import validate_target

POLICY = dict(maxRequests=3, sessionSeconds=30, requestSeconds=2,
              intervalSeconds=1, retries=0, maxResponseBytes=32768,
              imageExports=0, motorRequests=0)
SCHEMA = 'motionbrain.ov2640_register_snapshot.v1'
_HEX16 = re.compile(r'[0-9a-f]{16}\Z')
_HEX32 = re.compile(r'[0-9a-f]{32}\Z')
_HEX64 = re.compile(r'[0-9a-f]{64}\Z')
SENSOR_REGISTERS = (0x0a, 0x0b, 0x1c, 0x1d, 0x03, 0x0c, 0x11, 0x12, 0x13, 0x04, 0x10, 0x45,
                    0x2a, 0x2b, 0x2d, 0x2e, 0x46, 0x47, 0x17, 0x18, 0x19, 0x1a, 0x32,
                    0x48, 0x49, 0x4e, 0x4f, 0x50)
DSP_REGISTERS = (0x05, 0xc2, 0xd3, 0xda, 0xe0, 0x86)
AEC_REGISTERS = (0x04, 0x10, 0x45)
STATIC_FIELDS = ('cameraBootId', 'appElfSha256', 'xclkRequestedHz', 'xclkConfiguredHz',
                 'sensorPidReported', 'sensorAddress', 'frameSize', 'frameWidth',
                 'frameHeight', 'jpegQuality', 'internalAcquisition', 'imageExports', 'imageRequests')


def _hex(pattern, value):
    return isinstance(value, str) and pattern.fullmatch(value) is not None


@contextlib.contextmanager
def _defer_cancellation():
    """Finish one durable evidence/reference pair before honoring a stop signal."""
    pending = []
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    def remember(signum, frame):
        pending.append(signum)
    try:
        for sig in previous:
            signal.signal(sig, remember)
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if pending:
        raise KeyboardInterrupt('cancelled_during_evidence_storage')


def direct_get(url, nonce, *, clock=time, timeout=2):
    """One connection, no redirect/retry; keep even incomplete response bytes."""
    url = validate_target(url)
    if not _hex(_HEX16, nonce) or not 0 < timeout <= 2:
        raise ValueError('invalid_request')
    address = urlsplit(url)
    route = '/diagnostics/ov2640?nonce=' + nonce
    start = clock.monotonic()
    row = dict(method='GET', route=route, nonce=nonce, transportCompleted=False,
               requestStartedMonotonic=start, clockDomain='python_monotonic')
    body = bytearray()
    connection = http.client.HTTPConnection(address.hostname, address.port or 80, timeout=timeout)
    try:
        with deadline(timeout):
            connection.request('GET', route, headers={'Connection': 'close', 'Accept': 'application/json'})
            response = connection.getresponse()
            row['httpStatus'] = response.status
            while len(body) <= POLICY['maxResponseBytes']:
                chunk = response.read1(min(4096, POLICY['maxResponseBytes'] + 1 - len(body)))
                if not chunk:
                    break
                body.extend(chunk)
            if len(body) > POLICY['maxResponseBytes']:
                raise ValueError('response_too_large')
            if response.length not in (None, 0) or not response.isclosed():
                raise ValueError('incomplete_http_body')
            row['transportCompleted'] = True
    except KeyboardInterrupt:
        row.update(error='cancelled', cancelled=True)
    except (OSError, ValueError, http.client.HTTPException) as exc:
        row['error'] = str(exc) or type(exc).__name__
        if isinstance(exc, http.client.IncompleteRead):
            body.extend(exc.partial[:max(0, POLICY['maxResponseBytes'] + 1 - len(body))])
    finally:
        connection.close()
        row.update(responseBodyBase64=base64.b64encode(body).decode('ascii'),
                   bodySha256=hashlib.sha256(body).hexdigest(), receivedMonotonic=clock.monotonic())
    return row


def validate_snapshot(row, nonce, expected_hash, ordinal, baseline=None):
    """Validate raw bytes, not a collector-provided decoded response."""
    raw = base64.b64decode(row['responseBodyBase64'], validate=True)
    if len(raw) > POLICY['maxResponseBytes'] + 1 or hashlib.sha256(raw).hexdigest() != row.get('bodySha256'):
        raise ValueError('raw_body_integrity')
    if row.get('cancelled') is True:
        raise ValueError('cancelled')
    if (row.get('method') != 'GET' or row.get('route') != '/diagnostics/ov2640?nonce=' + nonce
            or row.get('nonce') != nonce or row.get('transportCompleted') is not True
            or row.get('httpStatus') != 200 or row.get('error') or len(raw) > POLICY['maxResponseBytes']):
        raise ValueError('http_failure')
    stamps = [row.get('requestStartedMonotonic'), row.get('receivedMonotonic')]
    if (any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in stamps)
            or not 0 <= stamps[1] - stamps[0] <= POLICY['requestSeconds']):
        raise ValueError('request_timing')
    snapshot = _decode_json(raw)
    if not isinstance(snapshot, dict) or snapshot.get('schemaVersion') != SCHEMA:
        raise ValueError('snapshot_schema')
    if (snapshot.get('nonce') != nonce or snapshot.get('appElfSha256') != expected_hash
            or not _hex(_HEX32, snapshot.get('cameraBootId'))
            or type(snapshot.get('attempt')) is not int or snapshot['attempt'] != ordinal):
        raise ValueError('snapshot_identity')
    if snapshot.get('ok') is not True or snapshot.get('failure') != '':
        raise ValueError('snapshot_failed')
    if (snapshot.get('exposureUpperBoundUs') is not None
            or any(snapshot.get(k) is not False for k in ('opticalExposureVerified', 'exposureUpperBoundVerified', 'atomicSnapshot', 'trainingEligible', 'controlEligible'))
            or snapshot.get('bankRestored') is not True):
        raise ValueError('unsupported_qualification')
    expected = dict(xclkRequestedHz=20000000, sensorPidReported=38, sensorAddress=48,
                    frameSize='qvga', frameWidth=320, frameHeight=240, jpegQuality=15,
                    internalAcquisition=True, imageExports=0, imageRequests=0)
    if any(type(snapshot.get(k)) is not type(v) or snapshot[k] != v for k, v in expected.items()):
        raise ValueError('unexpected_configuration')
    if type(snapshot.get('xclkConfiguredHz')) is not int or snapshot['xclkConfiguredHz'] != 20000000:
        raise ValueError('missing_xclk_configuration')
    if any(type(snapshot.get(k)) is not int or snapshot[k] < 0 for k in ('startedUs', 'endedUs')) or not 0 <= snapshot['endedUs'] - snapshot['startedUs'] <= 750000:
        raise ValueError('snapshot_timing')
    if (snapshot.get('trialClosed') is not (ordinal == 3)
            or (ordinal < 3 and snapshot.get('cameraDeinitStatus') is not None)
            or (ordinal == 3 and (type(snapshot.get('cameraDeinitStatus')) is not int
                                  or snapshot['cameraDeinitStatus'] != 0))):
        raise ValueError('trial_cleanup_unverified')
    transactions = snapshot.get('transactions')
    if not isinstance(transactions, list) or not transactions:
        raise ValueError('missing_transactions')
    previous = snapshot['startedUs']
    for tx in transactions:
        if (not isinstance(tx, dict) or tx.get('op') not in ('read', 'write_bank')
                or type(tx.get('bank')) is not int or tx['bank'] not in (-1, 0, 1)
                or type(tx.get('register')) is not int or not 0 <= tx['register'] <= 255
                or type(tx.get('value')) is not int or not 0 <= tx['value'] <= 255
                or type(tx.get('status')) is not int or tx['status'] != 0
                or (tx.get('op') == 'read' and (type(tx.get('addressStatus')) is not int or tx['addressStatus'] != 0))
                or (tx.get('op') == 'write_bank' and tx.get('addressStatus') is not None)
                or any(type(tx.get(k)) is not int for k in ('startedUs', 'endedUs'))
                or not previous <= tx['startedUs'] <= tx['endedUs'] <= snapshot['endedUs']):
            raise ValueError('invalid_transaction')
        previous = tx['endedUs']
    original_bank = transactions[0]['value']
    expected_ops = [('read', -1, 255), ('write_bank', 1, 255), ('read', 1, 255)]
    expected_ops += [('read', 1, r) for r in SENSOR_REGISTERS]
    expected_ops += [('write_bank', 0, 255), ('read', 0, 255)]
    expected_ops += [('read', 0, r) for r in DSP_REGISTERS]
    expected_ops += [('write_bank', original_bank, 255), ('read', original_bank, 255)]
    if original_bank not in (0, 1) or [(t['op'], t['bank'], t['register']) for t in transactions] != expected_ops:
        raise ValueError('transaction_sequence')
    if any(t['register'] == 255 and t['bank'] != -1 and t['value'] != t['bank'] for t in transactions):
        raise ValueError('bank_selection_or_restore')
    if transactions[3]['value'] != 38:
        raise ValueError('raw_sensor_pid')
    if baseline is not None:
        def static_registers(s):
            return [(t['bank'], t['register'], t['value'] &
                     ({0x04: 0xfc, 0x45: 0xc0}.get(t['register'], 0xff) if t['bank'] == 1 else 0xff))
                    for t in s['transactions']
                    if t['op'] == 'read' and t['register'] != 255
                    and not (t['bank'] == 1 and t['register'] == 0x10)]
        if (any(snapshot[k] != baseline[k] for k in STATIC_FIELDS)
                or static_registers(snapshot) != static_registers(baseline)):
            raise ValueError('boot_firmware_or_configuration_changed')
    return snapshot


def collect(output_dir, url, expected_app_elf_sha256, *, request=direct_get, clock=time):
    url = validate_target(url)
    if not _hex(_HEX64, expected_app_elf_sha256):
        raise ValueError('expected_app_elf_sha256_required')
    store = ArtifactStore(Path(output_dir))
    start = clock.monotonic()
    nonces = [uuid.uuid4().hex[:16] for _ in range(3)]
    plan = dict(schemaVersion='motionbrain.ov2640_register_plan.v1', policy=POLICY,
                url=url, expectedAppElfSha256=expected_app_elf_sha256, nonces=nonces,
                startedMonotonic=start, hostClockId=uuid.uuid4().hex)
    plan_ref = store.write_json('plan.json', plan)
    attempts = []
    failure = None
    baseline = None
    try:
        for ordinal, nonce in enumerate(nonces, 1):
            if ordinal > 1:
                clock.sleep(POLICY['intervalSeconds'])
            if clock.monotonic() - start >= POLICY['sessionSeconds'] - POLICY['requestSeconds']:
                raise ValueError('session_budget_exhausted')
            intent = dict(ordinal=ordinal, nonce=nonce, preparedMonotonic=clock.monotonic())
            with _defer_cancellation():
                intent_ref = store.write_json(f'attempts/{ordinal:02d}-intent.json', intent)
                attempts.append(dict(intent=intent_ref, raw=None))
            if clock.monotonic() - start >= POLICY['sessionSeconds'] - POLICY['requestSeconds']:
                raise ValueError('session_budget_exhausted')
            row = request(url, nonce, clock=clock, timeout=POLICY['requestSeconds'])
            with _defer_cancellation():
                attempts[-1]['raw'] = store.write_json(f'attempts/{ordinal:02d}-raw.json', row)
            snapshot = validate_snapshot(row, nonce, expected_app_elf_sha256, ordinal, baseline)
            baseline = baseline or snapshot
            if clock.monotonic() - start > POLICY['sessionSeconds']:
                raise ValueError('session_budget_exhausted')
    except KeyboardInterrupt:
        failure = 'cancelled'
    except (OSError, ValueError, StorageError, KeyError, TypeError, RecursionError) as exc:
        failure = str(exc) or type(exc).__name__
    result = dict(schemaVersion='motionbrain.ov2640_register_run.v1', plan=plan_ref,
                  attempts=attempts, plannedCount=3, intentCount=len(attempts),
                  responseCount=sum(a['raw'] is not None for a in attempts),
                  successfulRun=failure is None and len(attempts) == 3, failure=failure,
                  endedMonotonic=clock.monotonic(), exposureUpperBoundUs=None,
                  opticalExposureVerified=False, trainingEligible=False, controlEligible=False)
    ref = store.write_json('result.json', result)
    store.append('register_probe_sealed', 'observation', {'result': ref})
    return result


def audit(root):
    """Recompute all successful response checks; rejected attempts stay visible."""
    root = Path(root)
    errors = []
    validated = 0
    derived_failure = None
    complete = False
    try:
        from learning.artifacts import read_events
        events, event_errors = read_events(root)
        if event_errors or len(events) != 1 or events[0]['kind'] != 'register_probe_sealed':
            raise ValueError('missing_or_invalid_terminal_record')
        result = read_json_artifact(root, events[0]['payload']['result'])
        plan = read_json_artifact(root, result['plan'])
        if plan['policy'] != POLICY or len(plan['nonces']) != 3 or len(set(plan['nonces'])) != 3:
            raise ValueError('invalid_plan')
        if any(not _hex(_HEX16, n) for n in plan['nonces']) or not _hex(_HEX64, plan['expectedAppElfSha256']):
            raise ValueError('invalid_plan_identity')
        validate_target(plan['url'])
        attempts = result['attempts']
        if not isinstance(attempts, list) or len(attempts) > 3 or result['plannedCount'] != 3 or result['intentCount'] != len(attempts):
            raise ValueError('attempt_denominator')
        expected_files = {f'{n:02d}-intent.json' for n in range(1, len(attempts) + 1)}
        expected_files |= {f'{n:02d}-raw.json' for n, a in enumerate(attempts, 1) if a['raw'] is not None}
        attempt_dir = root / 'attempts'
        actual_files = {p.name for p in attempt_dir.iterdir()} if attempt_dir.exists() else set()
        if actual_files != expected_files:
            raise ValueError('unreported_or_missing_attempt_artifacts')
        baseline = None
        previous_received = None
        for ordinal, attempt in enumerate(attempts, 1):
            if derived_failure is not None:
                errors.append('request_after_failure')
            nonce = plan['nonces'][ordinal - 1]
            intent = read_json_artifact(root, attempt['intent'])
            if intent['ordinal'] != ordinal or intent['nonce'] != nonce:
                raise ValueError('intent_identity')
            if attempt['intent']['artifactRef'] != f'attempts/{ordinal:02d}-intent.json':
                raise ValueError('intent_reference')
            if attempt['raw'] is None:
                derived_failure = 'missing_response'
                continue
            if attempt['raw']['artifactRef'] != f'attempts/{ordinal:02d}-raw.json':
                raise ValueError('raw_reference')
            row = read_json_artifact(root, attempt['raw'])
            stamps = [plan['startedMonotonic'], intent['preparedMonotonic'],
                      row['requestStartedMonotonic'], row['receivedMonotonic'], result['endedMonotonic']]
            if (any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in stamps)
                    or stamps != sorted(stamps)):
                raise ValueError('request_timing')
            if (row['receivedMonotonic'] - row['requestStartedMonotonic'] > POLICY['requestSeconds']
                    and row.get('transportCompleted') is True):
                errors.append('request_budget_exceeded')
            if not intent['preparedMonotonic'] <= row['requestStartedMonotonic'] <= row['receivedMonotonic']:
                raise ValueError('request_timing')
            if previous_received is not None and row['requestStartedMonotonic'] - previous_received < POLICY['intervalSeconds']:
                errors.append('request_interval_too_short')
            previous_received = row['receivedMonotonic']
            try:
                snap = validate_snapshot(row, nonce, plan['expectedAppElfSha256'], ordinal, baseline)
                baseline = baseline or snap
                validated += 1
            except (ValueError, KeyError, TypeError, RecursionError) as exc:
                derived_failure = str(exc) or type(exc).__name__
        if derived_failure is not None and result['failure'] is None:
            errors.append('failure_omitted_from_result')
        if result['failure'] is None and len(attempts) != 3:
            errors.append('unexplained_missing_attempts')
        complete = validated == 3 and derived_failure is None and result['failure'] is None
        if result['responseCount'] != sum(a['raw'] is not None for a in attempts) or result['successfulRun'] is not complete:
            errors.append('result_mismatch')
        if complete and result['endedMonotonic'] - plan['startedMonotonic'] > POLICY['sessionSeconds']:
            errors.append('session_budget_exceeded')
        if any(result.get(k) is not False for k in ('opticalExposureVerified', 'trainingEligible', 'controlEligible')) or result.get('exposureUpperBoundUs') is not None:
            errors.append('unsupported_qualification')
    except (OSError, ValueError, StorageError, KeyError, TypeError, RecursionError) as exc:
        errors.append(str(exc) or type(exc).__name__)
    return dict(ok=not errors, evidenceIntegrityOk=not errors, errors=errors,
                validatedResponses=validated, successfulRun=not errors and complete,
                derivedFailure=derived_failure, exposureUpperBoundUs=None,
                opticalExposureVerified=False, trainingEligible=False, controlEligible=False)
