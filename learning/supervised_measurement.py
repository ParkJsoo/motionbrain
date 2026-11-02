"""Independent endpoint measurements from recorded supervised trials; no devices."""
import hashlib
import json
import math
from pathlib import Path
import statistics

SCHEMA = 'motionbrain.endpoint_measurement.v1'
DATA_SCHEMA = 'motionbrain.supervised_endpoint_dataset.v3'


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def read_bound(root, relative, expected=None):
    root = Path(root).resolve()
    path = root / relative
    if not isinstance(relative, str) or Path(relative).is_absolute() or root not in path.resolve().parents:
        raise ValueError('unsafe_evidence_path')
    raw = path.read_bytes()
    if expected is not None and sha(raw) != expected:
        raise ValueError('evidence_hash_mismatch')
    return raw


def validate_config(config):
    if config.get('schemaVersion') != SCHEMA:
        raise ValueError('measurement_schema')
    width, height = config.get('width'), config.get('height')
    if type(width) is not int or type(height) is not int or not 1 <= width <= 1920 or not 1 <= height <= 1080:
        raise ValueError('measurement_dimensions')
    threshold = config.get('stabilityThreshold')
    if not finite(threshold) or threshold <= 0:
        raise ValueError('stability_threshold_required')
    points = config.get('points')
    if not isinstance(points, list) or not 2 <= len(points) <= 12:
        raise ValueError('measurement_points')
    names = set()
    for point in points:
        name = point.get('id')
        if not isinstance(name, str) or not name or name in names:
            raise ValueError('measurement_point_identity')
        names.add(name)
        roi = point.get('roi', [])
        if len(roi) != 4 or any(type(v) is not int for v in roi):
            raise ValueError('measurement_roi')
        x, y, w, h = roi
        if min(x, y) < 0 or min(w, h) <= 0 or x+w > width or y+h > height:
            raise ValueError('measurement_roi')
        lower, upper = point.get('hsvLower', []), point.get('hsvUpper', [])
        if len(lower) != 3 or len(upper) != 3:
            raise ValueError('measurement_hsv')
        if any(type(a) is not int or type(b) is not int or not 0 <= a <= b <= limit
               for a, b, limit in zip(lower, upper, (179, 255, 255))):
            raise ValueError('measurement_hsv')
        low, high = point.get('minAreaPx'), point.get('maxAreaPx')
        if type(low) is not int or type(high) is not int or not 1 <= low <= high <= w*h:
            raise ValueError('measurement_blob_area')
    rep = config.get('representation', {})
    kind = rep.get('kind')
    if kind == 'relative_angle':
        parent, child = rep.get('parent', []), rep.get('child', [])
        if len(parent) != 2 or len(child) != 2 or len(set(parent+child)) != 4 or not set(parent+child) <= names:
            raise ValueError('measurement_relative_angle_points')
    elif kind == 'distance':
        ids = rep.get('points', [])
        if len(ids) != 2 or len(set(ids)) != 2 or not set(ids) <= names:
            raise ValueError('measurement_distance_points')
    elif kind == 'projected_position':
        ids = [rep.get('moving'), rep.get('reference')]
        if len(set(ids)) != 2 or not set(ids) <= names or rep.get('axis') not in ('x', 'y'):
            raise ValueError('measurement_position_points')
    else:
        raise ValueError('measurement_representation')
    minimum = config.get('minLinkLengthPx', 5.)
    if not finite(minimum) or minimum <= 0:
        raise ValueError('measurement_minimum_link')
    return config


def detect_points(pixels, config):
    """Independent frame detection; no tracker history or later frame input."""
    import cv2
    import numpy as np
    image = np.frombuffer(pixels, dtype=np.uint8).reshape(config['height'], config['width'], 4)
    hsv = cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2HSV)
    points = {}
    for point in config['points']:
        x, y, w, h = point['roi']
        mask = cv2.inRange(hsv[y:y+h, x:x+w], tuple(point['hsvLower']), tuple(point['hsvUpper']))
        count, labels, stats, centres = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates = [i for i in range(1, count) if point['minAreaPx'] <= stats[i, cv2.CC_STAT_AREA] <= point['maxAreaPx']]
        if len(candidates) != 1:
            raise ValueError('marker_missing_or_ambiguous:'+point['id'])
        i = candidates[0]
        bx, by, bw, bh, _ = stats[i]
        if bx == 0 or by == 0 or bx+bw == w or by+bh == h:
            raise ValueError('marker_touches_roi:'+point['id'])
        points[point['id']] = (float(centres[i][0]+x), float(centres[i][1]+y))
    if len(set(points.values())) != len(points):
        raise ValueError('marker_identity_overlap')
    return points


def wrap(angle):
    return (angle+180.) % 360. - 180.


def representation(points, config):
    rep = config['representation']
    def vector(ids):
        a, b = (points[i] for i in ids)
        return b[0]-a[0], b[1]-a[1]
    if rep['kind'] == 'relative_angle':
        parent, child = vector(rep['parent']), vector(rep['child'])
        if min(math.hypot(*parent), math.hypot(*child)) < config.get('minLinkLengthPx', 5.):
            raise ValueError('projected_link_too_short')
        return wrap(math.degrees(math.atan2(child[1], child[0])-math.atan2(parent[1], parent[0])))
    if rep['kind'] == 'distance':
        return math.hypot(*vector(rep['points']))
    index = 0 if rep['axis'] == 'x' else 1
    return points[rep['moving']][index]-points[rep['reference']][index]


def clock_evidence(capture, evidence):
    from learning.mac_clock_bridge import validate_sample
    from learning.action_recorder import recording_bridge
    path = 'host-ingress/clock-before.json'
    raw = read_bound(capture, path); evidence[path] = sha(raw)
    before = json.loads(raw); initial = validate_sample(before)
    start = before['pairs'][-1]['after']
    result = {'basis':'before_only_bounded_same_mach_clock', 'initialOffsets':list(initial),
              'allowedOffsets':[list(initial)], 'validPythonBounds':[start,start+120.],
              'hostClockId':before['hostClockId'], 'opticalExposureVerified':False}
    path = 'host-ingress/clock-after.json'
    if (Path(capture)/path).exists():
        raw = read_bound(capture,path); evidence[path]=sha(raw)
        bridge = recording_bridge(before,json.loads(raw))
        result.update(basis='before_after_same_mach_clock',validPythonBounds=bridge['validPythonBounds'])
        result['allowedOffsets'].append(bridge['offsetBoundsSeconds'])
    return result


def frame_value(capture, frame, config, clock, evidence, camera):
    from learning.lossless_frame import decode, ENCODING
    raw = read_bound(capture, frame['metadataFile'], frame['metadataSha256'])
    meta = json.loads(raw)
    if (meta.get('schemaVersion') != 'motionbrain.mac_camera_frame.v2' or meta.get('storageEncoding') != ENCODING
            or meta.get('frameIndex') != frame['frameIndex'] or meta.get('imageFile') != frame['imageFile']
            or meta.get('width') != config['width'] or meta.get('height') != config['height']
            or meta.get('rowBytes') != config['width']*4 or meta.get('pixelFormat') != 'BGRA8'
            or meta.get('rawByteLength') != config['width']*config['height']*4
            or meta.get('imageSha256') != frame['imageSha256']
            or meta.get('clockDomain') != 'coremedia_host_clock'
            or meta.get('timestampSemantics') != 'cmsamplebuffer_presentation_timestamp'):
        raise ValueError('frame_metadata_mismatch')
    offsets = frame.get('clockOffsetBoundsSeconds', [])
    if len(offsets) != 2 or not all(finite(x) for x in offsets) or offsets[0] > offsets[1]:
        raise ValueError('frame_clock_bounds_required')
    if not any(all(abs(a-b) <= 1e-12 for a,b in zip(offsets,allowed)) for allowed in clock['allowedOffsets']):
        raise ValueError('frame_clock_offsets_unproven')
    if not clock['validPythonBounds'][0] <= frame['sourceEarliest'] <= frame['sourceLatest'] <= clock['validPythonBounds'][1]:
        raise ValueError('frame_outside_clock_evidence')
    if any(meta.get(k) != camera[v] for k,v in (('sessionId','sessionId'),('viewConfigurationId','viewId'),('deviceUniqueId','deviceId'))):
        raise ValueError('frame_plan_identity_mismatch')
    native = meta.get('convertedHostPtsSeconds')
    if not finite(native) or any(abs(frame[key]-(native+offset)) > 1e-6
                               for key, offset in zip(('sourceEarliest', 'sourceLatest'), offsets)):
        raise ValueError('frame_clock_bounds_mismatch')
    encoded_config = meta.get('actualConfigurationJson')
    if not isinstance(encoded_config, str) or sha(encoded_config.encode()) != meta.get('configurationSha256'):
        raise ValueError('frame_configuration_hash')
    stored = read_bound(capture, frame['imageFile'], frame['imageSha256'])
    if len(stored) != meta.get('storedByteLength'):
        raise ValueError('frame_stored_length')
    pixels = decode(stored, meta['rawByteLength'], meta['pixelSha256'])
    evidence[frame['metadataFile']] = sha(raw)
    evidence[frame['imageFile']] = sha(stored)
    identity = (meta.get('sessionId'), meta.get('viewConfigurationId'), meta.get('deviceUniqueId'),
                meta['configurationSha256'], meta.get('videoMirrored'), meta.get('videoRotationAngle'))
    if any(value is None for value in identity):
        raise ValueError('frame_identity_missing')
    return representation(detect_points(pixels, config), config), identity


def validate_execution(trial, planned):
    from learning.supervised_transport import validate_pulse
    pulse = validate_pulse(trial['pulse'])
    if pulse != planned or trial.get('requestedPulse') != pulse or trial.get('motor') != pulse['motor']:
        raise ValueError('execution_pulse_mismatch')
    trace = trial.get('outputTrace')
    if not isinstance(trace,list) or not trace:
        raise ValueError('execution_output_trace_missing')
    previous = None; observed = False; selected = []
    for entry in trace:
        if any(type(entry.get(k)) is not int for k in ('sequence','atMs','motor','pwm')):
            raise ValueError('execution_output_trace_invalid')
        if not 0 <= entry['atMs'] < 2**32 or not 1 <= entry['motor'] <= 5:
            raise ValueError('execution_output_trace_invalid')
        if previous is not None and (entry['sequence'] != previous['sequence']+1
                or (entry['atMs']-previous['atMs']) % 2**32 > 60000):
            raise ValueError('execution_output_trace_gap')
        previous=entry
        pwm = entry['pwm']
        if pwm and (entry['motor'] != pulse['motor'] or pwm*pulse['expectedPwm'] <= 0 or abs(pwm)>abs(pulse['expectedPwm'])):
            raise ValueError('execution_unexpected_output')
        if entry['motor'] == pulse['motor']:
            selected.append(pwm);observed = observed or pwm != 0
    if not observed or not selected or selected[-1] != 0:
        raise ValueError('execution_pulse_or_off_missing')


def measure(report, config, capture):
    validate_config(config)
    if report.get('schemaVersion') != 'motionbrain.supervised_collection.v1':
        raise ValueError('collection_schema')
    plan = report['plan']
    if not isinstance(plan.get('captureGroup'), str) or not plan['captureGroup'] or plan.get('split') not in ('development', 'train', 'validation', 'test'):
        raise ValueError('capture_group_split_required')
    camera = plan.get('camera', {})
    if any(not isinstance(camera.get(k),str) or not camera[k] for k in ('sessionId','viewId','deviceId')):
        raise ValueError('plan_camera_identity_required')
    if camera.get('width') != config['width'] or camera.get('height') != config['height']:
        raise ValueError('plan_camera_dimensions_mismatch')
    frames = report['frames']
    ids = set(); previous = None
    for frame in frames:
        a, b = frame.get('sourceEarliest'), frame.get('sourceLatest')
        if (type(frame.get('frameIndex')) is not int or frame['frameIndex'] in ids
                or not finite(a) or not finite(b) or a > b
                or (previous is not None and a <= previous)):
            raise ValueError('frame_order_or_timing')
        ids.add(frame['frameIndex']); previous = a
    trials = report['trials']
    if len({t['id'] for t in trials}) != len(trials):
        raise ValueError('duplicate_trial_id')
    cache, evidence, rows = {}, {}, []
    clock = clock_evidence(capture,evidence)
    if clock['hostClockId'] != camera.get('hostClockId',camera['sessionId']):
        raise ValueError('plan_clock_identity_mismatch')
    def selected(start, end):
        chosen = [f for f in frames if start <= f['sourceEarliest'] and f['sourceLatest'] <= end]
        if (len(chosen) < 3 or chosen[0]['sourceLatest']-start > .3+1e-9 or end-chosen[-1]['sourceEarliest'] > .3+1e-9
                or any(b['sourceLatest']-a['sourceEarliest'] > .3+1e-9 for a, b in zip(chosen, chosen[1:]))):
            raise ValueError('measurement_window_missing_or_gapped')
        values, identities = [], set()
        for frame in chosen:
            index = frame['frameIndex']
            if index not in cache:
                try: cache[index] = frame_value(capture, frame, config, clock, evidence, camera)
                except (ValueError, KeyError, OSError, TypeError) as exc: cache[index] = exc
            value = cache[index]
            if isinstance(value, Exception): raise ValueError('frame_invalid:'+str(value))
            scalar, identity = value; values.append(scalar); identities.add(identity)
        if len(identities) != 1: raise ValueError('measurement_configuration_changed')
        angular = config['representation']['kind'] == 'relative_angle'
        if angular: values = [values[0]+wrap(x-values[0]) for x in values]
        spread = max(values)-min(values)
        if spread > config['stabilityThreshold']: raise ValueError('measurement_window_unsettled')
        value = statistics.median(values)
        times=[(f['sourceEarliest']+f['sourceLatest'])/2 for f in chosen]
        mean_t,mean_v=statistics.mean(times),statistics.mean(values)
        denominator=sum((t-mean_t)**2 for t in times)
        if denominator<=0: raise ValueError('measurement_velocity_time_span')
        velocity=sum((t-mean_t)*(v-mean_v) for t,v in zip(times,values))/denominator
        return {'velocityPerSecond':velocity,'value':wrap(value) if angular else value, 'spread':spread,
                'frameIds':[f['frameIndex'] for f in chosen], 'window':[start,end]}, identities.pop()
    for ordinal,trial in enumerate(trials):
        row = {'id':trial['id'], 'captureGroup':plan['captureGroup'], 'split':plan['split'],
               'pulse':trial.get('pulse'), 'executionStatus':trial['status'], 'status':'rejected',
               'reason':None, 'outputTrace':trial.get('outputTrace', [])}
        try:
            if trial['status'] != 'executed' or trial.get('offVerified') is not True:
                raise ValueError('execution_not_confirmed_complete')
            validate_execution(trial,plan['pulses'][ordinal])
            receipt_path=f'trial-{ordinal+1:03d}-result.json'
            receipt_raw=read_bound(Path(capture).parent,receipt_path)
            if json.loads(receipt_raw) != trial: raise ValueError('execution_receipt_report_mismatch')
            evidence['../'+receipt_path]=sha(receipt_raw)
            before, dispatched, off, post = (trial.get(k) for k in ('dispatchBefore','dispatchAfter','offAt','postEnd'))
            if not all(finite(x) for x in (before,dispatched,off)) or not before <= dispatched <= off:
                raise ValueError('trial_times_invalid')
            quiet = trial.get('quietVerifiedUntil')
            if not finite(post) or not finite(quiet) or min(post,quiet) < off+3.:
                raise ValueError('post_window_quiet_not_verified')
            windows = ((before-.75,before-.10),(off+2.25,off+3.))
            for other in trials:
                if other is trial or other['status'] not in ('executed','outcome_unknown'): continue
                start = other.get('dispatchBefore')
                if not finite(start): raise ValueError('other_execution_time_unknown')
                stop = other.get('offAt') if other.get('offVerified') is True else None
                stop = stop if finite(stop) else math.inf
                if any(start <= end and stop >= begin for begin,end in windows):
                    raise ValueError('measurement_window_intersects_other_command')
            first, identity = selected(*windows[0]); last, last_identity = selected(*windows[1])
            if identity != last_identity: raise ValueError('measurement_configuration_changed')
            delta = last['value']-first['value']
            if config['representation']['kind'] == 'relative_angle': delta = wrap(delta)
            row.update(status='eligible',before=first,after=last,delta=delta,reason=None)
        except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
            row['reason'] = str(exc)
        rows.append(row)
    return {'schemaVersion':DATA_SCHEMA,'contract':'settled_endpoints_after_confirmed_off_v1',
            'units':'projected_degrees' if config['representation']['kind']=='relative_angle' else 'pixels',
            'velocityUnits':'projected_degrees_per_second' if config['representation']['kind']=='relative_angle' else 'pixels_per_second',
            'physicalJointAngleCalibrated':False,'trainingPerformed':False,
            'collectionReportSha256':sha(json.dumps(report,sort_keys=True,separators=(',',':')).encode()),
            'measurementConfig':config,'measurementConfigSha256':sha(json.dumps(config,sort_keys=True,separators=(',',':')).encode()),
            'captureGroup':plan['captureGroup'],'split':plan['split'],'clockEvidence':clock,'rows':rows,
            'summary':{'attempts':len(rows),'eligible':sum(r['status']=='eligible' for r in rows),
                       'rejected':sum(r['status']=='rejected' for r in rows)},'sourceHashes':evidence}
