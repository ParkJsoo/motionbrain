"""Offline stationary-camera repeatability diagnostics; no acquisition or training.

All planned windows remain in the denominator. Statistics on eligible windows
are explicitly conditional and cannot establish improvement on rejected ones.
"""
import argparse
import json
import math
from pathlib import Path
import statistics

from . import supervised_measurement as shared

SCHEMA = 'motionbrain.stationary_repeatability.v1'
METHODS = {
    'A': ('beforeWindow', 'afterWindow', 'stationary_existing_short_window_v1'),
    'B': ('candidateBeforeWindow', 'candidateAfterWindow', 'stationary_candidate_two_second_median_v1'),
}


def angular_stats(values):
    if not values:
        return dict(count=0, median=None, std=None, range=None)
    unwrapped = [values[0] + shared.wrap(value - values[0]) for value in values]
    return dict(count=len(values), median=shared.wrap(statistics.median(unwrapped)),
                std=statistics.pstdev(unwrapped), range=max(unwrapped)-min(unwrapped))


def summarize_pairs(pairs):
    values = [p['delta'] for p in pairs if p['status'] == 'eligible']
    absolute = [abs(value) for value in values]
    return dict(attempts=len(pairs), eligible=len(values), rejected=len(pairs)-len(values),
                statisticsPopulation='eligible_pairs_only', medianAbsDelta=statistics.median(absolute) if values else None,
                maxAbsDelta=max(absolute) if values else None,
                rmsDelta=math.sqrt(statistics.mean([value*value for value in values])) if values else None)


def validate_plan(plan):
    anchors = plan.get('anchorsSeconds')
    if (not isinstance(anchors, list) or not anchors or
            any(not shared.finite(x) or x <= 0 for x in anchors) or anchors != sorted(set(anchors))):
        raise ValueError('plan_anchors')
    for before, after, _ in METHODS.values():
        for key in (before, after):
            bounds = plan.get(key)
            if (not isinstance(bounds, list) or len(bounds) != 2 or
                    not all(shared.finite(x) for x in bounds) or bounds[0] >= bounds[1]):
                raise ValueError('plan_window:' + key)
        if plan[before][1] >= 0 or plan[after][0] <= 0:
            raise ValueError('plan_window_order')
    if (plan['candidateBeforeWindow'][1] != plan['beforeWindow'][1] or
            plan['candidateAfterWindow'][1] != plan['afterWindow'][1] or
            abs(plan['candidateBeforeWindow'][1]-plan['candidateBeforeWindow'][0]-2.) > 1e-9 or
            abs(plan['candidateAfterWindow'][1]-plan['candidateAfterWindow'][0]-2.) > 1e-9):
        raise ValueError('candidate_endpoints_or_duration_changed')
    if plan.get('stabilityThreshold') != .5 or plan.get('maxGapSeconds') != .3:
        raise ValueError('fixed_quality_policy')
    clips = plan.get('clips', ['clip-01', 'clip-02', 'clip-03'])
    if (not isinstance(clips, list) or not clips or len(set(clips)) != len(clips) or
            any(not isinstance(x, str) or Path(x).name != x or x in ('.', '..') for x in clips)):
        raise ValueError('plan_clips')
    return clips


def pixel_diagnostics(pixels, points, config):
    """Optional diagnostics after the unchanged shared frame validation."""
    import cv2
    import numpy as np
    image = np.frombuffer(pixels, dtype=np.uint8).reshape(config['height'], config['width'], 4)[:, :, :3]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    markers = {}
    for point in config['points']:
        x, y, w, h = point['roi']
        roi = hsv[y:y+h, x:x+w]
        mask = cv2.inRange(roi, tuple(point['hsvLower']), tuple(point['hsvUpper']))
        _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        candidates = [i for i in range(1, len(stats)) if point['minAreaPx'] <= stats[i, cv2.CC_STAT_AREA] <= point['maxAreaPx']]
        if len(candidates) != 1:
            raise ValueError('diagnostic_marker_disagreement')
        selected = labels == candidates[0]
        markers[point['id']] = dict(x=points[point['id']][0], y=points[point['id']][1],
            areaPx=int(stats[candidates[0], cv2.CC_STAT_AREA]), brightnessMean=float(roi[:, :, 2][selected].mean()))
    links = {}
    for name in ('parent', 'child'):
        a, b = (points[key] for key in config['representation'][name])
        dx, dy = b[0]-a[0], b[1]-a[1]
        links[name] = dict(angleDeg=math.degrees(math.atan2(dy, dx)), lengthPx=math.hypot(dx, dy))
    return dict(points=markers, links=links, frameBrightnessMean=float(hsv[:, :, 2].mean()))


def evaluate_window(rows, start, end, plan):
    chosen = [row for row in rows if start <= row['sourceEarliest'] and row['sourceLatest'] <= end]
    result = dict(status='rejected', reasons=[], window=[start, end], count=len(chosen),
                  validCount=sum(row['status'] == 'valid' for row in chosen),
                  frameIds=[row['frameIndex'] for row in chosen], value=None, spread=None, gaps=[])
    gaps = ([chosen[0]['sourceLatest']-start] +
            [b['sourceLatest']-a['sourceEarliest'] for a, b in zip(chosen, chosen[1:])] +
            [end-chosen[-1]['sourceEarliest']]) if chosen else [end-start]
    result.update(gaps=gaps, maxGap=max(gaps), failedFrameIds=[r['frameIndex'] for r in chosen if r['status'] != 'valid'])
    if len(chosen) < 3 or max(gaps) > plan['maxGapSeconds'] + 1e-9:
        result['reasons'].append('window_missing_or_gapped')
    if result['failedFrameIds']:
        result['reasons'].append('window_contains_invalid_frame')
    identities = {tuple(row['identity']) for row in chosen if row['status'] == 'valid'}
    if len(identities) != 1:
        result['reasons'].append('window_configuration_missing_or_changed')
    if result['validCount'] == len(chosen) and chosen:
        stats = angular_stats([row['angleDeg'] for row in chosen])
        result.update(value=stats['median'], spread=stats['range'], std=stats['std'])
        if stats['range'] > plan['stabilityThreshold']:
            result['reasons'].append('window_unsettled')
    if not result['reasons']:
        result['status'] = 'eligible'
        result['identity'] = list(next(iter(identities)))
    return result


def make_pairs(rows, first_source, plan, clip_reasons=()):
    pairs = []
    for offset in plan['anchorsSeconds']:
        anchor = first_source + offset if first_source is not None else None
        for method, (before_key, after_key, contract) in METHODS.items():
            pair = dict(method=method, contract=contract, anchorOffsetSeconds=offset, anchor=anchor,
                        status='rejected', reasons=list(clip_reasons), delta=None)
            if anchor is not None:
                for name, key in (('before', before_key), ('after', after_key)):
                    pair[name] = evaluate_window(rows, anchor+plan[key][0], anchor+plan[key][1], plan)
                    pair['reasons'].extend(name + ':' + reason for reason in pair[name]['reasons'])
                if pair['before']['status'] == pair['after']['status'] == 'eligible':
                    if pair['before']['identity'] != pair['after']['identity']:
                        pair['reasons'].append('pair_configuration_changed')
                    elif not pair['reasons']:
                        pair.update(status='eligible', delta=shared.wrap(pair['after']['value']-pair['before']['value']))
            else:
                pair['reasons'].append('clip_has_no_usable_anchor')
            pairs.append(pair)
    return pairs


def raw_summary(rows):
    valid = [row for row in rows if row['status'] == 'valid']
    result = dict(frames=len(rows), validFrames=len(valid), failedFrames=len(rows)-len(valid),
                  angle=angular_stats([row['angleDeg'] for row in valid]), points={}, links={})
    if not valid:
        return result
    for name in valid[0]['diagnostics']['points']:
        points = [row['diagnostics']['points'][name] for row in valid]
        xs, ys = [p['x'] for p in points], [p['y'] for p in points]
        mx, my = statistics.mean(xs), statistics.mean(ys)
        result['points'][name] = dict(meanX=mx, meanY=my, stdX=statistics.pstdev(xs), stdY=statistics.pstdev(ys),
            rangeX=max(xs)-min(xs), rangeY=max(ys)-min(ys),
            rmsDisplacementPx=math.sqrt(statistics.mean([(x-mx)**2+(y-my)**2 for x,y in zip(xs,ys)])),
            areaRangePx=[min(p['areaPx'] for p in points),max(p['areaPx'] for p in points)],
            brightnessRange=[min(p['brightnessMean'] for p in points),max(p['brightnessMean'] for p in points)])
    for name in ('parent', 'child'):
        links = [row['diagnostics']['links'][name] for row in valid]
        result['links'][name] = dict(angle=angular_stats([link['angleDeg'] for link in links]),
            lengthRangePx=[min(link['lengthPx'] for link in links),max(link['lengthPx'] for link in links)])
    return result


def validate_recording(capture, report, hashes):
    records = []
    for path in ('terminal.json', 'host-ingress/recording-summary.json'):
        raw = shared.read_bound(capture, path)
        hashes[path] = shared.sha(raw)
        records.append(json.loads(raw))
    terminal, summary = records
    camera, frames = report['camera'], report['frames']
    if (terminal.get('schemaVersion') != 'motionbrain.mac_camera_stream_terminal.v1' or
            terminal.get('sessionId') != camera['sessionId'] or terminal.get('viewConfigurationId') != camera['viewId'] or
            terminal.get('deviceUniqueId') != camera['deviceId'] or terminal.get('status') != 'completed' or
            terminal.get('reason') != 'duration_complete' or summary.get('status') != 'completed' or
            summary.get('terminalStatus') != 'completed' or summary.get('reason') is not None):
        raise ValueError('recording_terminal_not_completed_or_identity')
    if (terminal.get('recordedFrames') != len(frames) or summary.get('recordedFrames') != len(frames) or
            summary.get('frames') != frames or
            {k:v for k,v in summary.items() if k!='frames'} != report.get('recording') or
            len(terminal.get('frames', [])) != len(frames)):
        raise ValueError('recording_frame_count_or_summary_mismatch')
    for receipt, frame in zip(terminal['frames'], frames):
        if (receipt.get('durable') is not True or any(receipt.get(key) != frame.get(key) for key in
                ('frameIndex','metadataFile','metadataSha256','imageFile','imageSha256'))):
            raise ValueError('recording_terminal_receipt_mismatch')


def analyze_clip(root, name, config, plan):
    clip = dict(id=name, status='rejected', reasons=[], frames=[], sourceHashes={})
    capture = Path(root)/name/'native-camera'
    first_source = None
    try:
        raw = shared.read_bound(root, name+'/report.json')
        clip['sourceHashes']['report.json'] = shared.sha(raw)
        report = json.loads(raw)
        if (report.get('schemaVersion') != 'motionbrain.stationary_capture.v1' or
                type(report.get('motorCommands')) is not int or report['motorCommands'] != 0):
            raise ValueError('stationary_capture_contract')
        camera = report['camera']
        if (any(not isinstance(camera.get(k), str) or not camera[k] for k in ('sessionId', 'viewId', 'deviceId', 'hostClockId'))
                or camera.get('width') != config['width'] or camera.get('height') != config['height']):
            raise ValueError('camera_identity_or_dimensions')
        clock = shared.clock_evidence(capture, clip['sourceHashes'])
        if camera['hostClockId'] != clock['hostClockId']:
            raise ValueError('camera_clock_identity')
        frames = report['frames']
        try:
            validate_recording(capture, report, clip['sourceHashes'])
        except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
            clip['reasons'].append(str(exc))
        ids, previous = set(), None
        if frames:
            first_source = frames[0]['sourceEarliest']
            if not shared.finite(first_source):
                first_source = None
                raise ValueError('first_frame_time')
        if report.get('recording', {}).get('status') != 'completed':
            clip['reasons'].append('recording_not_completed')
        for frame in frames:
            row = {k: frame.get(k) for k in ('frameIndex', 'sourceEarliest', 'sourceLatest')}
            row.update(status='invalid', reason=None)
            try:
                lo, hi, index = row['sourceEarliest'], row['sourceLatest'], row['frameIndex']
                if (type(index) is not int or index in ids or not shared.finite(lo) or not shared.finite(hi)
                        or lo > hi or (previous is not None and lo <= previous)):
                    raise ValueError('frame_order_or_timing')
                ids.add(index); previous = lo
                angle, identity = shared.frame_value(capture, frame, config, clock, clip['sourceHashes'], camera)
                # Keep the established validator unchanged. Re-read with the same
                # hashes so mutation between validation and diagnostics fails.
                from .lossless_frame import decode
                meta = json.loads(shared.read_bound(capture, frame['metadataFile'], frame['metadataSha256']))
                pixels = decode(shared.read_bound(capture, frame['imageFile'], frame['imageSha256']),
                                meta['rawByteLength'], meta['pixelSha256'])
                diagnostics = pixel_diagnostics(pixels, shared.detect_points(pixels, config), config)
                row.update(status='valid', angleDeg=angle, identity=list(identity), diagnostics=diagnostics)
            except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
                row['reason'] = str(exc)
                if row['reason'] == 'frame_order_or_timing':
                    clip['reasons'].append('frame_order_or_timing')
            clip['frames'].append(row)
        if clip['frames'] and clip['frames'][0]['status'] != 'valid':
            clip['reasons'].append('first_anchor_frame_invalid')
        if len({tuple(row['identity']) for row in clip['frames'] if row['status']=='valid'}) > 1:
            clip['reasons'].append('clip_configuration_changed')
        clip['clockEvidence'] = clock
        if not clip['reasons']:
            clip['status'] = 'analyzed'
    except (ValueError, KeyError, TypeError, IndexError, OSError) as exc:
        clip['reasons'].append(str(exc))
    usable = [row for row in clip['frames'] if shared.finite(row['sourceEarliest']) and shared.finite(row['sourceLatest'])]
    clip['pairs'] = make_pairs(usable, first_source, plan, clip['reasons'])
    clip['rawSummary'] = raw_summary(clip['frames'])
    clip['methodSummary'] = {method: summarize_pairs([p for p in clip['pairs'] if p['method'] == method]) for method in METHODS}
    return clip


def analyze(root, config, plan):
    names = validate_plan(plan)
    shared.validate_config(config)
    if config['representation']['kind'] != 'relative_angle' or config['stabilityThreshold'] != plan['stabilityThreshold']:
        raise ValueError('fixed_relative_angle_policy')
    clips = [analyze_clip(root, name, config, plan) for name in names]
    pairs = [pair for clip in clips for pair in clip['pairs']]
    common = {method: [] for method in METHODS}
    for clip in clips:
        for anchor in plan['anchorsSeconds']:
            selected = [p for p in clip['pairs'] if p['anchorOffsetSeconds'] == anchor]
            if all(p['status'] == 'eligible' for p in selected):
                for pair in selected:
                    common[pair['method']].append(pair)
    result = dict(schemaVersion=SCHEMA, split='development', independentTestSetAvailable=False,
        trainingPerformed=False, existingLabelsChanged=False, physicalJointAngleCalibrated=False,
        units='projected_degrees', correlatedWithinSession=True, clips=clips,
        plannedPhysicalPairs=len(names)*len(plan['anchorsSeconds']),
        methodSummary={method:summarize_pairs([p for p in pairs if p['method']==method]) for method in METHODS},
        commonEligiblePairSummary={method:summarize_pairs(common[method]) for method in METHODS},
        firstClipMethodSummary={method:summarize_pairs([p for p in clips[0]['pairs'] if p['method']==method]) for method in METHODS},
        laterClipsMethodSummary={method:summarize_pairs([p for c in clips[1:] for p in c['pairs'] if p['method']==method]) for method in METHODS},
        limitation='Same-session stationary diagnostic; eligible-only statistics do not remove failures or prove independent-test improvement.',
        configSha256=shared.sha(json.dumps(config,sort_keys=True,separators=(',',':'),allow_nan=False).encode()),
        plan=plan)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for flag in ('root', 'config', 'plan', 'output'):
        parser.add_argument('--'+flag, required=True, type=Path)
    args = parser.parse_args(argv)
    raw_config, raw_plan = args.config.read_bytes(), args.plan.read_bytes()
    result = analyze(args.root, json.loads(raw_config), json.loads(raw_plan))
    result['inputFileHashes'] = dict(config=shared.sha(raw_config), plan=shared.sha(raw_plan))
    with args.output.open('x') as output:
        json.dump(result, output, indent=2, allow_nan=False)
        output.write('\n')
    print(json.dumps(dict(clips=len(result['clips']), plannedPhysicalPairs=result['plannedPhysicalPairs'],
                          methods=result['methodSummary']), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
