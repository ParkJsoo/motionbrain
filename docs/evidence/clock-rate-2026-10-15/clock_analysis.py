"""Causal CAM/Pi rate intervals; diagnostic only, never control calibration.

No import/use of source_age_basis: testing its rate assumption must not select
samples using that assumption. HTTP delay can be arbitrarily asymmetric.
"""
from fractions import Fraction as F
import math
from pathlib import Path
import re
import sys

REPO = next(p for p in Path(__file__).resolve().parents if (p/'learning').is_dir())
sys.path.insert(0, str(REPO))
from learning.http_timing import TRACE_STAMPS, SEND_START_SEMANTICS
from learning.fresh_capture import validate_fresh_capture
from ros2_ws.src.motionbrain_ros_bridge.motionbrain_ros_bridge.observation import parse_capture_headers


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def sample_from_receipt(r):
    if (r.get('device'), r.get('path'), r.get('method'), r.get('status')) != ('camera','/capture','GET',200) or 'error' in r:
        raise ValueError('invalid_capture')
    h, t = r.get('headers', {}), r.get('connectionTiming', {})
    source = parse_capture_headers(h)
    age = h.get('x-motionbrain-source-age-us')
    if not source['sourceMetadataValid'] or not isinstance(age,str) or not re.fullmatch(r'[0-9]{1,19}',age):
        raise ValueError('invalid_source_clock')
    if r.get('query') not in ({}, {'fresh':'1'}):
        raise ValueError('unexpected_capture_query')
    proof = validate_fresh_capture(r, source)
    stamps = [r.get('requestStartedMonotonic'), *(t.get(k) for k in TRACE_STAMPS),
              r.get('headersAvailableMonotonic'), r.get('bodyReadCompletedMonotonic'), r.get('receivedMonotonic')]
    if (not all(finite(v) for v in stamps) or stamps != sorted(stamps)
            or type(t.get('addressAttempts')) is not int or t['addressAttempts'] < 1
            or t.get('sendStartSemantics') != SEND_START_SEMANTICS):
        raise ValueError('invalid_direct_send_trace')
    if not isinstance(r.get('hostClockId'),str) or not r['hostClockId']:
        raise ValueError('invalid_host_clock')
    low, high = t['requestSendStartedMonotonic'], r['headersAvailableMonotonic']
    if low >= high:
        raise ValueError('empty_causal_interval')
    return dict(sourceFrameId=source['sourceFrameId'], cameraBootId=source['cameraBootId'],
                hostClockId=r['hostClockId'], cameraNowUs=source['capturedUs']+int(age),
                capturedUs=source['capturedUs'], sourceAgeUs=int(age),
                hostLower=low, hostUpper=high, fresh=proof is not None,
                freshRequestedUs=proof['requestedUs'] if proof else None,
                bracketWidthMs=(high-low)*1000)


def checked(samples):
    if not isinstance(samples,list) or len(samples) < 2:
        raise ValueError('at_least_two_samples_required')
    seen = set()
    for i, s in enumerate(samples):
        if (type(s.get('cameraNowUs')) is not int or s['cameraNowUs'] <= 0
                or not finite(s.get('hostLower')) or not finite(s.get('hostUpper'))
                or s['hostLower'] >= s['hostUpper']):
            raise ValueError('invalid_clock_sample')
        if (not s.get('cameraBootId') or not s.get('hostClockId') or not s.get('sourceFrameId')
                or s['cameraBootId'] != samples[0]['cameraBootId']
                or s['hostClockId'] != samples[0]['hostClockId'] or s['sourceFrameId'] in seen):
            raise ValueError('clock_identity_changed_or_repeated')
        seen.add(s['sourceFrameId'])
        if i and (s['cameraNowUs'] <= samples[i-1]['cameraNowUs'] or s['hostLower'] <= samples[i-1]['hostUpper']):
            raise ValueError('clock_regressed_or_intervals_overlap')


def pair_bounds(a, b, delta_error_us):
    # Difference before conversion preserves long camera uptimes exactly.
    delta = b['cameraNowUs']-a['cameraNowUs']
    lo = F(delta-delta_error_us, 1000000)/(F(b['hostUpper'])-F(a['hostLower']))
    hi = F(delta+delta_error_us, 1000000)/(F(b['hostLower'])-F(a['hostUpper']))
    return lo, hi


def present(lo, hi):
    assumption_low, assumption_high = F(999,1000), F(1001,1000)
    feasible = lo <= hi
    classification = ('inconsistent_constant_rate_model' if not feasible else
        'outside_assumed_range' if hi < assumption_low or lo > assumption_high else
        'contained_in_assumed_range' if assumption_low <= lo <= hi <= assumption_high else
        'overlaps_but_not_bounded_by_assumed_range')
    return dict(cameraPerHostRate=[float(lo),float(hi)],
                ppm=[float((lo-1)*1000000),float((hi-1)*1000000)],
                feasible=feasible, classification=classification)


def analyze(samples, *, delta_error_us=2):
    checked(samples)
    if type(delta_error_us) is not int or not 0 <= delta_error_us <= 1000:
        raise ValueError('invalid_timestamp_difference_margin')
    if any(b['cameraNowUs']-a['cameraNowUs'] <= delta_error_us for a,b in zip(samples,samples[1:])):
        raise ValueError('camera_delta_too_small')
    all_pairs = [pair_bounds(a,b,delta_error_us) for i,a in enumerate(samples) for b in samples[i+1:]]
    lo,hi = max(p[0] for p in all_pairs),min(p[1] for p in all_pairs)
    windows = []
    # Index windows are reported with their actual host span, not assumed duration.
    for i in range(0,len(samples)-1,30):
        j = min(i+30,len(samples)-1)
        windows.append(dict(first=i,last=j,hostStartSpanSeconds=samples[j]['hostLower']-samples[i]['hostLower'],
                            **present(*pair_bounds(samples[i],samples[j],delta_error_us))))
    return dict(sampleCount=len(samples), sourceAgeLimitMs=100, assumedClockRatePpm=1000,
        timestampDifferenceMarginUs=delta_error_us, timestampMarginMeasured=False,
        hostStartSpanSeconds=samples[-1]['hostLower']-samples[0]['hostLower'],
        endpointAverage=present(*pair_bounds(samples[0],samples[-1],delta_error_us)),
        constantAffineModel=present(lo,hi), windows=windows,
        physicalCalibration=False, usableForControl=False, trainingEligible=False,
        limitations=['endpoint_and_window_averages_not_instantaneous_rate',
                     'constant_affine_fit_is_a_model_not_uniform_rate_proof',
                     'timestamp_margin_assumed_not_measured', 'first_dma_not_optical_exposure',
                     'no_future_drift_or_other_boot_guarantee'])
