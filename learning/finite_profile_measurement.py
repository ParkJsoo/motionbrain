"""Offline finite-profile endpoint measurement; never dispatches or trains.

Host anchors stay in Python monotonic time. Device millis are used only for
output evidence consistency, never as a camera clock conversion.

Public file layout: capture is the native-camera directory. Its parent must
contain before-receipt.json (persisted before execute) and
closed-actual-evidence.json (the final device response). measure() reads these
fixed paths, compares their content with the report and records byte hashes.
preflight() does not write files; callers must persist its exact receipt.
"""
import json
import statistics
from pathlib import Path
from . import supervised_measurement as shared

SCHEMA = 'motionbrain.finite_profile_capture.v1'
RESULT_SCHEMA = 'motionbrain.finite_profile_endpoint_measurement.v1'
CONTRACT = 'finite_settled_fixed_before_host_off_receipt_v1'
MAX_BEFORE_ANCHOR_AGE_SECONDS = 2.0


def digest(value):
    return shared.sha(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def execution_identity(report):
    boot, epoch, command = report.get('expectedBootId'), report.get('expectedEpoch'), report.get('trial', {}).get('id')
    if (not isinstance(boot, str) or not boot or type(epoch) is not int or not 1 <= epoch < 2**32
            or not isinstance(command, str) or not command):
        raise ValueError('expected_execution_identity_required')
    return dict(bootId=boot, commandId=command, requestEpoch=epoch)


def context(report, config, capture):
    shared.validate_config(config)
    execution_identity(report)
    if (report.get('schemaVersion') != SCHEMA or report.get('actionContract') != 'finite_profile.v1'
            or report.get('split') != 'development' or not isinstance(report.get('captureGroup'), str)
            or not report['captureGroup']):
        raise ValueError('finite_capture_contract')
    if config['representation']['kind'] != 'relative_angle' or config['stabilityThreshold'] != .5:
        raise ValueError('finite_relative_angle_policy')
    if not isinstance(report.get('requestedProfile'), dict):
        raise ValueError('requested_profile_required')
    camera = report['camera']
    if any(not isinstance(camera.get(k), str) or not camera[k] for k in ('sessionId', 'viewId', 'deviceId', 'hostClockId')):
        raise ValueError('camera_identity')
    if camera.get('width') != config['width'] or camera.get('height') != config['height']:
        raise ValueError('camera_dimensions')
    hashes = {}
    clock = shared.clock_evidence(capture, hashes)
    if camera['hostClockId'] != clock['hostClockId']:
        raise ValueError('camera_clock_identity')
    ids, previous = set(), None
    for frame in report['frames']:
        lo, hi = frame.get('sourceEarliest'), frame.get('sourceLatest')
        index = frame.get('frameIndex')
        if (type(index) is not int or index in ids or not shared.finite(lo) or not shared.finite(hi)
                or lo > hi or (previous is not None and lo <= previous)):
            raise ValueError('frame_order_or_timing')
        ids.add(index)
        previous = lo
    return clock, hashes


def window(report, config, capture, clock, hashes, start, end, fixed_ids=None, available_at=None):
    frames = report['frames']
    if fixed_ids is not None:
        frames = [f for f in frames if f['frameIndex'] in fixed_ids]
        if [f['frameIndex'] for f in frames] != fixed_ids:
            raise ValueError('before_frame_identity')
    chosen = [f for f in frames if start <= f['sourceEarliest'] and f['sourceLatest'] <= end]
    if fixed_ids is not None and len(chosen) != len(fixed_ids):
        raise ValueError('before_frame_window')
    if (len(chosen) < 3 or chosen[0]['sourceLatest'] - start > .3 + 1e-9
            or end - chosen[-1]['sourceEarliest'] > .3 + 1e-9
            or any(b['sourceLatest'] - a['sourceEarliest'] > .3 + 1e-9 for a, b in zip(chosen, chosen[1:]))):
        raise ValueError('measurement_window_missing_or_gapped')
    values, identities = [], set()
    for frame in chosen:
        if available_at is not None and (not shared.finite(frame.get('stdoutReadAt'))
                or frame['stdoutReadAt'] > available_at or frame['stdoutReadAt'] < frame['sourceEarliest']):
            raise ValueError('before_frame_not_available')
        value, identity = shared.frame_value(capture, frame, config, clock, hashes, report['camera'])
        values.append(value)
        identities.add(identity)
    if len(identities) != 1:
        raise ValueError('measurement_configuration_changed')
    values = [values[0] + shared.wrap(x - values[0]) for x in values]
    spread = max(values) - min(values)
    if spread > config['stabilityThreshold']:
        raise ValueError('measurement_window_unsettled')
    times = [(f['sourceEarliest'] + f['sourceLatest']) / 2 for f in chosen]
    mean_t, mean_v = statistics.mean(times), statistics.mean(values)
    denominator = sum((t - mean_t) ** 2 for t in times)
    if denominator <= 0:
        raise ValueError('measurement_velocity_time_span')
    velocity = sum((t - mean_t) * (v - mean_v) for t, v in zip(times, values)) / denominator
    return dict(value=shared.wrap(statistics.median(values)), spread=spread,
                velocityPerSecond=velocity, frameIds=[f['frameIndex'] for f in chosen],
                window=[start, end]), list(identities.pop())


def preflight(report, config, capture, anchor):
    """Validate a closed, already available before window; caller persists receipt before execute."""
    result = dict(status='rejected', reason=None, contract=CONTRACT,
                  anchorHostMonotonic=anchor, measurementConfigSha256=digest(config))
    try:
        if not shared.finite(anchor):
            raise ValueError('before_anchor_invalid')
        clock, hashes = context(report, config, capture)
        before, identity = window(report, config, capture, clock, hashes, anchor - .75, anchor - .10,
                                  available_at=anchor)
        result.update(status='eligible', before=before, cameraIdentity=identity, sourceHashes=hashes,
                      requestedProfileSha256=digest(report['requestedProfile']), executionIdentity=execution_identity(report),
                      maxBeforeAnchorAgeSeconds=MAX_BEFORE_ANCHOR_AGE_SECONDS)
    except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
        result['reason'] = str(exc)
    result['receiptSha256'] = digest(result)
    return result


def elapsed(later, earlier):
    if type(later) is not int or type(earlier) is not int or not 0 <= later < 2**32 or not 0 <= earlier < 2**32:
        raise ValueError('device_timestamp')
    delta = (later - earlier) % 2**32
    if delta > 60000:
        raise ValueError('device_timestamp_order')
    return delta


def validate_output(trial, requested_profile, expected_identity):
    evidence = trial['closedEvidence']
    record, output = evidence['record'], evidence['actualOutput']
    if any(record.get(k) != value or output.get(k) != value for k, value in expected_identity.items()):
        raise ValueError('finite_execution_identity')
    if (record.get('commandId') != trial['id'] or record.get('state') != 'Completed'
            or record.get('reason') != 'NominalEnd'
            or any(record.get(k) is not True for k in ('executed', 'anyNonzeroWrite', 'softwareOffConfirmed',
                    'offWriteRecorded', 'offConfirmationRecorded', 'timingConformant', 'traceComplete'))
            or record.get('quietHeld') is not False or record.get('offFailureLatched') is not False):
        raise ValueError('finite_execution_not_complete')
    if (output.get('evidence') != 'driver_write' or output.get('clockDomain') != 'controller_boot_millis'
            or output.get('closed') is not True or output.get('captureComplete') is not True
            or output.get('dropped') != 0 or any(output.get(k) != record.get(k) for k in ('bootId', 'commandId', 'requestEpoch'))
            or not record.get('bootId') or type(output.get('lease')) is not int or output['lease'] <= 0
            or type(output.get('ordinal')) is not int or output['ordinal'] <= 0):
        raise ValueError('finite_output_identity_or_completeness')
    profile, receipts, entries = record['requestedProfile'], record['plannedEventReceipts'], output['entries']
    if profile != requested_profile:
        raise ValueError('requested_profile_changed')
    points = profile['points']
    if (profile.get('motor') != 2 or profile.get('direction') not in (-1, 1)
            or not 2 <= len(points) <= 32 or points[0]['offsetMs'] != 0 or points[-1]['pwm'] != 0
            or len(receipts) != len(points) or record.get('nextEventIndex') != len(points)
            or output.get('totalWrites') != len(entries) or len(entries) < len(points) + 5):
        raise ValueError('finite_output_shape')
    for i, entry in enumerate(entries):
        if (type(entry.get('sequence')) is not int or entry['sequence'] != i + 1
                or type(entry.get('motor')) is not int or not 1 <= entry['motor'] <= 5
                or type(entry.get('pwm')) is not int or abs(entry['pwm']) > 255):
            raise ValueError('finite_output_sequence')
        elapsed(entry['atMs'], entries[i - 1]['atMs'] if i else output['openedAt'])
        if i >= len(points) and (entry['pwm'] != 0 or entry.get('reason') != 'forced_off'):
            raise ValueError('finite_quiet_output')
    lateness = []
    for i, (point, receipt, entry) in enumerate(zip(points, receipts, entries)):
        offset, pwm = point['offsetMs'], point['pwm']
        if (type(offset) is not int or offset < 0 or type(pwm) is not int
                or (i < len(points) - 1 and (pwm * profile['direction'] <= 0 or abs(pwm) > 255))
                or (i and offset - points[i - 1]['offsetMs'] <= profile['maxLatenessMs'])
                or receipt.get('index') != i or receipt.get('present') is not True
                or receipt.get('pwm') != pwm or receipt.get('atMs') != entry['atMs']
                or entry['motor'] != 2 or entry['pwm'] != pwm or entry.get('reason') != 'driver_write'):
            raise ValueError('finite_event_receipt')
        late = elapsed(receipt['atMs'], (record['plannedStartAt'] + offset) % 2**32)
        if late > profile['maxLatenessMs']:
            raise ValueError('finite_event_late')
        lateness.append(late)
    if (elapsed(record['plannedStartAt'], record['executeAcceptedAt']) != profile['startLeadMs']
            or elapsed(record['plannedOffAt'], record['plannedStartAt']) != points[-1]['offsetMs']
            or record['firstNonzeroWriteAt'] != entries[0]['atMs']
            or record['offWriteAt'] != receipts[-1]['atMs']
            or {e['motor'] for e in entries[len(points):]} != {1, 2, 3, 4, 5}):
        raise ValueError('finite_output_anchors')
    elapsed(record['offConfirmedAt'], entries[len(points) + 4]['atMs'])
    elapsed(output['closedAt'], entries[-1]['atMs'])
    selected = [entry for entry in entries if entry['motor'] == 2]
    integral = sum(a['pwm'] * elapsed(b['atMs'], a['atMs']) for a, b in zip(selected, selected[1:]))
    active = sum(elapsed(b['atMs'], a['atMs']) for a, b in zip(selected, selected[1:]) if a['pwm'])
    planned_integral = sum(a['pwm'] * (b['offsetMs'] - a['offsetMs']) for a, b in zip(points, points[1:]))
    return dict(activeMs=active, signedPwmIntegralCountMs=integral,
                plannedActiveMs=points[-1]['offsetMs'], plannedSignedPwmIntegralCountMs=planned_integral,
                peakAbsolutePwm=max(abs(e['pwm']) for e in selected), maximumLatenessMs=max(lateness),
                evidence='driver_write', closedEvidenceSha256=digest(evidence))


def measure(report, config, capture):
    result = dict(schemaVersion=RESULT_SCHEMA, contract=CONTRACT, actionContract='finite_profile.v1',
                  captureGroup=report.get('captureGroup'), split='development', status='rejected', reason=None,
                  units='projected_degrees', velocityUnits='projected_degrees_per_second',
                  physicalJointAngleCalibrated=False, trainingPerformed=False, predictionPerformed=False,
                  measurementConfigSha256=digest(config))
    try:
        clock, hashes = context(report, config, capture)
        trial = report['trial']
        receipt = trial['beforeReceipt']
        expected = dict(receipt)
        receipt_hash = expected.pop('receiptSha256')
        if (digest(expected) != receipt_hash or receipt.get('status') != 'eligible'
                or receipt.get('contract') != CONTRACT or receipt.get('measurementConfigSha256') != digest(config)
                or receipt.get('maxBeforeAnchorAgeSeconds') != MAX_BEFORE_ANCHOR_AGE_SECONDS
                or receipt.get('requestedProfileSha256') != digest(report['requestedProfile'])
                or receipt.get('executionIdentity') != execution_identity(report)):
            raise ValueError('before_receipt_invalid')
        for filename, value in (('before-receipt.json', receipt), ('closed-actual-evidence.json', trial['closedEvidence'])):
            raw = shared.read_bound(Path(capture).parent, filename)
            hashes['../' + filename] = shared.sha(raw)
            if json.loads(raw) != value:
                raise ValueError('finite_source_receipt_mismatch:' + filename)
        anchor = receipt['anchorHostMonotonic']
        dispatch, dispatched, off, quiet, post, release = (trial[k] for k in
            ('dispatchBefore', 'dispatchAfter', 'hostOffReceiptAt', 'quietVerifiedUntil', 'postEnd', 'releaseStartedAt'))
        if (not all(shared.finite(x) for x in (anchor, dispatch, dispatched, off, quiet, post, release))
                or not anchor <= dispatch <= dispatched <= off or dispatch - anchor > MAX_BEFORE_ANCHOR_AGE_SECONDS):
            raise ValueError('finite_host_timing_or_stale_before')
        if min(quiet, post, release) < off + 3. or release < quiet:
            raise ValueError('finite_quiet_window_incomplete')
        output_summary = validate_output(trial, report['requestedProfile'], receipt['executionIdentity'])
        before, identity = window(report, config, capture, clock, hashes, anchor - .75, anchor - .10,
                                  fixed_ids=receipt['before']['frameIds'], available_at=anchor)
        if before != receipt['before'] or identity != receipt['cameraIdentity']:
            raise ValueError('before_receipt_measurement_changed')
        for path, value in receipt['sourceHashes'].items():
            if shared.sha(shared.read_bound(capture, path)) != value:
                raise ValueError('before_receipt_source_changed')
        after, last_identity = window(report, config, capture, clock, hashes, off + 2.25, off + 3.)
        if identity != last_identity:
            raise ValueError('measurement_configuration_changed')
        result.update(status='eligible', before=before, after=after, delta=shared.wrap(after['value'] - before['value']),
                      beforeReceiptSha256=receipt_hash, beforeAnchorAgeSeconds=dispatch - anchor,
                      actualOutputSummary=output_summary, clockEvidence=clock, sourceHashes=hashes,
                      captureReportSha256=digest(report))
    except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
        result['reason'] = str(exc)
    result['summary'] = dict(attempts=1, eligible=int(result['status'] == 'eligible'), rejected=int(result['status'] != 'eligible'))
    return result
