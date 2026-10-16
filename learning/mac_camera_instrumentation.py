"""Audit optional v1 AVFoundation diagnostics without changing freshness criteria."""
from __future__ import annotations

import hashlib
import json
import math


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def stamp(value):
    if not number(value) or value < 0:
        raise ValueError('instrumentation_invalid_host_time')
    return value


def raw_cmtime(value):
    if (not isinstance(value, dict)
            or any(type(value.get(k)) is not int for k in ('value', 'timescale', 'flags', 'epoch'))):
        raise ValueError('instrumentation_invalid_cmtime_record')
    if (not -(2**63) <= value['value'] < 2**63 or not -(2**63) <= value['epoch'] < 2**63
            or not -(2**31) <= value['timescale'] < 2**31 or not 0 <= value['flags'] < 2**32):
        raise ValueError('instrumentation_cmtime_out_of_range')
    return value


def rational(value):
    raw_cmtime(value)
    if value['timescale'] <= 0 or value['flags'] & ~3 or not value['flags'] & 1:
        raise ValueError('instrumentation_invalid_cmtime')
    return value['value'] / value['timescale']


def configuration(value, *, frame=None):
    if not isinstance(value, dict) or not isinstance(value.get('sessionPreset'), str) or not value['sessionPreset']:
        raise ValueError('instrumentation_invalid_configuration')
    settings = value.get('outputVideoSettings')
    active = value.get('activeFormat')
    if not isinstance(settings, dict) or not settings or not isinstance(active, dict):
        raise ValueError('instrumentation_missing_format')
    for key in ('width', 'height', 'mediaSubtype'):
        if type(active.get(key)) is not int or active[key] <= 0:
            raise ValueError('instrumentation_invalid_active_format')
    # Device-reported duration can be unset; preserve the raw sentinel, never
    # turn it into an observed frame rate. Delivered PTS is audited separately.
    for key in ('videoMinFrameDuration', 'videoMaxFrameDuration'):
        raw_cmtime(active.get(key))
    if frame is not None:
        sample = value.get('sampleFormat')
        delivered = value.get('deliveredPixelBuffer')
        if (not isinstance(sample, dict) or not isinstance(delivered, dict)
                or any(type(sample.get(k)) is not int or type(delivered.get(k)) is not int
                       or sample[k] != frame[k] or delivered[k] != frame[k] for k in ('width', 'height'))
                or type(sample.get('mediaSubtype')) is not int or sample['mediaSubtype'] != 1111970369
                or type(delivered.get('pixelFormat')) is not int or delivered['pixelFormat'] != 1111970369
                or value.get('videoMirrored') is not frame['videoMirrored']
                or value.get('videoRotationAngle') != frame['videoRotationAngle']):
            raise ValueError('instrumentation_sample_format_mismatch')
    return value


def begin(plan, session, terminal):
    required = plan.get('requiredInstrumentationVersion')
    versions = [item.get('instrumentationVersion') for item in (session, terminal)]
    if required is None and versions == [None, None]:
        return None  # Immutable legacy records remain auditable by their original contract.
    if (required not in (None, 1) or any(type(v) is not int or v != 1 for v in versions)
            or (required is not None and type(required) is not int)):
        raise ValueError('instrumentation_version_missing_or_invalid')
    configuration(session.get('actualConfigurationBeforeStart'))
    lifecycle = terminal.get('lifecycle')
    keys = ('startRunningBeginHostSeconds', 'startRunningEndHostSeconds',
            'stopRunningBeginHostSeconds', 'stopRunningEndHostSeconds', 'callbackQueueDrainedHostSeconds')
    if not isinstance(lifecycle, dict):
        raise ValueError('instrumentation_lifecycle_missing')
    times = [stamp(lifecycle.get(k)) for k in keys]
    if times != sorted(times):
        raise ValueError('instrumentation_lifecycle_order')
    if stamp(session.get('launchHostSeconds')) > times[0]:
        raise ValueError('instrumentation_launch_order')
    warmups = terminal.get('warmupTrace')
    drops = terminal.get('droppedTrace')
    trunc = terminal.get('droppedTraceTruncatedCount')
    if (not isinstance(warmups, list) or len(warmups) != terminal['warmupFrames']
            or not isinstance(drops, list) or len(drops) > 64
            or type(trunc) is not int or trunc < 0
            or len(drops) + trunc != terminal['droppedFrames']):
        raise ValueError('instrumentation_trace_count')
    previous_return = times[0]
    for i, item in enumerate(warmups):
        if not isinstance(item, dict) or item.get('callbackIndex') != i or type(item.get('callbackIndex')) is not int:
            raise ValueError('instrumentation_warmup_index')
        arrival, returned = stamp(item.get('callbackHostSeconds')), stamp(item.get('callbackReturnPreparedHostSeconds'))
        if not previous_return <= arrival <= returned <= times[2]:
            raise ValueError('instrumentation_warmup_order')
        raw_cmtime(item.get('pts'))
        converted = item.get('convertedHostPtsSeconds')
        if converted is not None and not 0 <= stamp(converted) <= arrival:
            raise ValueError('instrumentation_warmup_pts')
        previous_return = returned
    for i, item in enumerate(drops):
        if (not isinstance(item, dict) or type(item.get('dropIndex')) is not int or item['dropIndex'] != i
                or (item.get('reason') is not None and not isinstance(item['reason'], str))):
            raise ValueError('instrumentation_dropped_trace_invalid')
        arrival, returned = stamp(item.get('callbackHostSeconds')), stamp(item.get('callbackReturnPreparedHostSeconds'))
        if not times[0] <= arrival <= returned <= times[4]:
            raise ValueError('instrumentation_drop_time_order')
        # A dropped sample can have invalid timing; retain the raw diagnostic instead of inventing it.
        raw_cmtime(item.get('pts'))
    return {'lifecycle': lifecycle, 'lastReturn': previous_return, 'configuration': None,
            'warmupTrace': warmups, 'droppedTrace': drops, 'rows': []}


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('instrumentation_duplicate_configuration_key')
        result[key] = value
    return result


def invalid_constant(value):
    raise ValueError('instrumentation_nonfinite_configuration')


def check_frame(state, frame, receipt):
    if state is None:
        if frame.get('instrumentationVersion') is not None:
            raise ValueError('instrumentation_unbound_frame')
        return
    if type(frame.get('instrumentationVersion')) is not int or frame['instrumentationVersion'] != 1:
        raise ValueError('instrumentation_frame_version')
    config = configuration(frame.get('actualConfiguration'), frame=frame)
    serialized = frame.get('actualConfigurationJson')
    if (not isinstance(serialized, str)
            or hashlib.sha256(serialized.encode()).hexdigest() != frame.get('configurationSha256')
            or json.loads(serialized, object_pairs_hook=unique_pairs, parse_constant=invalid_constant) != config):
        raise ValueError('instrumentation_configuration_hash')
    if state['configuration'] is not None and state['configuration'] != serialized:
        raise ValueError('instrumentation_configuration_changed')
    state['configuration'] = serialized
    returned = stamp(receipt.get('callbackReturnPreparedHostSeconds'))
    arrival = frame['callbackHostSeconds']
    if not state['lastReturn'] <= arrival <= receipt['storageCompleteHostSeconds'] <= returned <= state['lifecycle']['stopRunningBeginHostSeconds']:
        raise ValueError('instrumentation_callback_order')
    row = {'frameIndex': frame['frameIndex'],
           'ptsToCallbackDiagnosticMs': (arrival-frame['convertedHostPtsSeconds'])*1000,
           'callbackToReturnPreparedMs': (returned-arrival)*1000,
           'postPersistenceCallbackMs': (returned-receipt['storageCompleteHostSeconds'])*1000,
           'gapAfterPreviousCallbackPreparedMs': (arrival-state['lastReturn'])*1000,
           'callbackReturnPreparedHostSeconds': returned,
           'configurationSha256': frame['configurationSha256']}
    state['lastReturn'] = returned
    state['rows'].append(row)


def summary(state):
    if state is None:
        return {'available': False, 'reason': 'legacy_capture_without_instrumentation'}
    return {'available': True, 'instrumentationVersion': 1, 'lifecycle': state['lifecycle'],
            'warmupTrace': state['warmupTrace'], 'droppedTrace': state['droppedTrace'],
            'frames': state['rows'],
            'returnTimestampSemantics': 'sampled_before_callback_return_not_dispatch_ready_or_actual_return',
            'opticalTimingVerified': False}
