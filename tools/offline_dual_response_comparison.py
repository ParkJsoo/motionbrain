"""Compare pre-command dual-view response estimates on archived open-arm approach runs.

Run from the repository root with a Python environment providing OpenCV and NumPy.
The input is the near-grasp directory holding relative-approach-replay and its raw trials.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.lossless_frame import decode
from learning.near_part_overlay import render_near
from learning.open_side_approach import LEGACY_SIDE_VIEW, observe_side_tip
from learning.relative_approach import NEIGHBORHOOD, relative_state


def read(path):
    return json.loads(path.read_bytes())


def checked_bytes(path, expected):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != expected:
        raise ValueError(f'frame digest mismatch: {path.name}')
    return data


def images(root, pair):
    meta = read(root / pair['externalIngress'])
    raw = checked_bytes(root / 'external-stream' / meta['imageFile'], meta['imageSha256'])
    bgra = decode(raw, meta['rawByteLength'], pair['externalPixelSha256'])
    side = np.frombuffer(bgra, np.uint8).reshape(meta['height'], meta['width'], 4)
    jpeg = checked_bytes(root / pair['nearJpeg'], pair['nearSha256'])
    near = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    return cv2.cvtColor(side, cv2.COLOR_BGRA2RGB), cv2.cvtColor(near, cv2.COLOR_BGR2RGB)


def describe(side, near, pre):
    # Equal weights per camera, before any supervised features are added.
    pixels = np.concatenate([cv2.resize(view, (24, 16), interpolation=cv2.INTER_AREA).ravel()
                             for view in (side, near)]).astype(float) / 255
    try:
        _, tracking = observe_side_tip(side, view_id=LEGACY_SIDE_VIEW)
    except ValueError:
        tracking = 'unresolved'
    _, near_parts = render_near(near)
    flags = [tracking == 'paired_markers', tracking == 'single_marker_wrist_contour']
    flags.extend(near_parts['parts'][name]['visible'] for name in ('leftJaw', 'rightJaw', 'target'))
    geometry = np.array(relative_state(pre)) / NEIGHBORHOOD
    return pixels, np.r_[geometry, flags], flags


def tip(o):
    x, y, w, _ = o['box']
    return np.asarray(o['state'][:2]) + [x+w/2, y]


def load_run(base, run, known):
    root = base / run
    trace = [json.loads(line) for line in (root / 'trace.jsonl').read_text().splitlines()]
    plans = [e for e in trace if e['kind'] == 'prediction']
    terminal = read(root/'result.json')
    if not terminal['finalIdleOffVerified']:
        raise ValueError('missing terminal output-OFF verification')
    pairs = {p['frameId']: p for p in map(read, (root/'source/camera-observations').glob('*.json'))}
    writes = {}
    adds = []
    for file in sorted((root/'http').glob('*-intent.json')):
        request = read(file)
        payload = request.get('payload') or {}
        if request['path'] == '/sequence' and payload.get('action') == 'add':
            adds.append(request)
    for response in map(read, (root/'http').glob('*-response.json')):
        for w in (response.get('result') or {}).get('outputs', {}).get('entries', []):
            if w['sequence'] in writes and w != writes[w['sequence']]:
                raise ValueError('output sequence conflict in trial')
            writes[w['sequence']] = w
    data = []
    template = next(iter(known.values()))
    for i, event in enumerate(plans):
        plan = event['value']
        step = plan['step']
        last = plans[i+1]['t'] if i+1 < len(plans) else float('inf')
        in_step = [a for a in adds if event['t'] <= a['startedAt'] < last]
        joint, direction = plan.get('action', plan.get('phase')).split('/')
        pre = plan['input']
        entry = dict(episode=run, setup=template['setup'], step=step, view=LEGACY_SIDE_VIEW,
                     startShoulderDeg=pre['angle'], preFrame=pre['frame'],
                     command=[joint, direction, None, plan['targetMs'], template['command'][-1]])
        data.append(entry)
        if in_step:
            if len(in_step) != 1:
                raise ValueError('expected one queued action per attempted step')
            queued = in_step[0]['payload']
            if [queued['joint'], queued['direction'], queued['duration']] != [joint, direction, plan['targetMs']]:
                raise ValueError('plan and queued action differ')
            entry['command'][2] = queued['speed']
        if step not in known:
            entry['hold'] = 'no_completed_settled_transition'
            entry['terminalReason'] = terminal['reason']
            continue
        rec = known[step]
        if entry['command'] != rec['command']:
            raise ValueError('command profile disagrees with verified archive')
        receipt, = [e for e in trace if e['kind']=='receipt' and event['t'] < e['t'] < last]
        end = receipt['value']
        motor = {'shoulder':4, 'elbow':3, 'wrist':2}[joint]
        stop = writes[end['zero']]
        drove = [w for w in writes.values() if w['motor']==motor and
                 w['atMs'] == stop['atMs']-end['driverMs'] and w['sequence'] < stop['sequence']]
        if len(drove) != 1 or not drove[0]['pwm'] or stop['pwm'] or stop['motor'] != motor:
            raise ValueError('recorded drive interval or zero mismatch')
        burst = [writes[seq] for seq in range(drove[0]['sequence'], stop['sequence']+1)]
        drive = [w['pwm'] for w in burst if w['pwm']]
        if any(w['pwm'] and w['motor']!=motor for w in burst) or len(set(np.sign(drive))) != 1:
            raise ValueError('mixed drive during a single-joint transition')
        expected_sign = {('shoulder','down'):-1, ('elbow','down'):1, ('wrist','down'):-1}
        # The queue takes a speed percentage, while the driver records signed PWM.
        if np.sign(drive[0]) != expected_sign[joint,direction]:
            raise ValueError('observed direction differs from the queued command')
        entry['output'] = dict(firstSequence=drove[0]['sequence'], zeroSequence=end['zero'],
                               driverMs=end['driverMs'], peakPwm=max(drive, key=abs))
        pair = pairs[pre['frame']]
        posts = rec['after']
        after_pairs = [pairs[o['frame']] for o in posts]
        # Host intervals bound acquisition, they do not certify optical synchronization.
        if any(pair[c]['availableAt'] > event['t'] for c in ('extClock','nearClock')):
            raise ValueError('plan precedes image availability')
        for obs, after in zip(posts, after_pairs):
            if any(after[c]['sourceEarliest'] <= receipt['t'] for c in ('extClock','nearClock')):
                raise ValueError('post-frame is not confirmed after OFF')
            if any(after[c]['availableAt'] >= last for c in ('extClock','nearClock')):
                raise ValueError('post-frame overlaps a subsequent action plan')
            if any(obs[k] != after[k] for k in ('nearSha256','externalPixelSha256')):
                raise ValueError('post label/image mismatch')
            # Verify both source files without feeding post-action pixels into a predictor.
            images(root/'source', after)
        for field in ('frameId','nearFrameId'):
            if len({p[field] for p in [pair,*after_pairs]}) != 3:
                raise ValueError('repeated raw frame')
        if any(p['configuration'] != pair['configuration'] for p in after_pairs):
            raise ValueError('configuration changed within transition')
        entry['configuration'] = pair['configuration']['configurationSha256']
        if pair['configuration']['viewConfigurationId'] != LEGACY_SIDE_VIEW:
            raise ValueError('unexpected camera view')
        if pre.get('nearFrame', pair['nearFrameId']) != pair['nearFrameId']:
            raise ValueError('command and near exposure ID differ')
        pix, feat, flags = describe(*images(root/'source', pair), pre)
        post_state = np.median([p['state'] for p in posts], axis=0)
        entry.update(pixels=pix, geometry=feat, flags=flags, preTip=tip(pre),
                     angleDelta=float(np.median([p['angle'] for p in posts])-pre['angle']),
                     tipDelta=np.median([tip(p) for p in posts], axis=0)-tip(pre),
                     truth=post_state-np.array(pre['state']), postFrames=[p['frame'] for p in posts],
                     imageDigests=[(p['externalPixelSha256'],p['nearSha256']) for p in [pair,*after_pairs]])
    return data


def rotate_matrix(degrees):
    a = np.radians(degrees)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def predictions(query, train):
    ys = np.stack([r['truth'] for r in train])
    image_distance = np.array([np.mean((r['pixels']-query['pixels'])**2) for r in train])
    # Unit-normalize the image distance by training-to-training variation alone.
    scale = np.mean([np.mean((a['pixels']-b['pixels'])**2) for a in train for b in train])
    d = image_distance / max(scale, 1e-10)
    def interpolate(distances, values):
        indices = np.argsort(distances, kind='stable')[:3]
        return np.average(values[indices], axis=0, weights=1/(0.1+distances[indices]))
    extra_distance = np.array([np.mean((query['geometry']-r['geometry'])**2) for r in train])
    pred = interpolate(d + extra_distance, ys)
    if query['command'][0] == 'shoulder':
        # Same in-plane rigid rotation used by the existing projected approach model.
        # Fit pivot and pulse response within the training fold to avoid held-out calibration.
        rotations = [rotate_matrix(r['angleDelta']) - np.eye(2) for r in train]
        rhs = [m @ r['preTip']-r['tipDelta'] for m, r in zip(rotations, train)]
        pivot = np.linalg.lstsq(np.vstack(rotations), np.hstack(rhs), rcond=None)[0]
        angle = interpolate(d + extra_distance, np.array([r['angleDelta'] for r in train]))
        fk = (rotate_matrix(angle)-np.eye(2)) @ (query['preTip']-pivot)
        # The object is stationary during these open-arm pulses. Offset targets
        # retain residual object motion and projection errors from training only.
        residuals = [y[:2]-m@(r['preTip']-pivot) for y,m,r in zip(ys,rotations,train)]
        pred[:2] = fk + interpolate(d + extra_distance, np.array(residuals))
    return dict(unchanged=np.zeros(4), commandMean=ys.mean(axis=0),
                twoViews=interpolate(d, ys), twoViewsPartsGeometryFK=pred)


def score(scored):
    metrics = {}
    for method in (scored[0]['prediction'] if scored else []):
        metrics[method] = {}
        for view, sl in [('side',slice(0,2)),('near',slice(2,4))]:
            p = np.stack([r['prediction'][method][sl] for r in scored])
            y = np.stack([r['truth'][sl] for r in scored])
            def direction(v):
                return np.where(np.abs(v)<=1, 0, np.sign(v))
            metrics[method][view] = dict(n=len(y), xDirectionHits=int(sum(direction(p[:,0])==direction(y[:,0]))),
                yDirectionHits=int(sum(direction(p[:,1])==direction(y[:,1]))),
                yMaePx=float(np.abs(y[:,1]-p[:,1]).mean()), vectorErrorPx=float(np.linalg.norm(y-p,axis=1).mean()),
                lengthMaePx=float(np.abs(np.linalg.norm(y,axis=1)-np.linalg.norm(p,axis=1)).mean()))
    return metrics


def evaluate(base):
    archive = base/'relative-approach-replay'
    audit = read(archive/'replay.json')
    verified = read(archive/'transitions.json') + audit['observedButUnconfirmed']
    groups = {}
    for r in verified:
        groups.setdefault(r['source'], {})[int(r['phase'].split('-')[-1])] = r
    rows = [r for run, known in groups.items() for r in load_run(base,run,known)]
    scored = []
    for r in rows:
        if 'hold' in r:
            continue
        train = [s for s in rows if 'truth' in s and s['setup'] != r['setup'] and
                 s['command'] == r['command'] and s['configuration'] == r['configuration']]
        if not train:
            r['hold'] = 'no_other_setup_same_command_profile'
            continue
        r['fitEpisodes'] = sorted({s['episode'] for s in train})
        if any({a[v] for a in r['imageDigests']} & {a[v] for s in train for a in s['imageDigests']}
               for v in (0,1)):
            raise ValueError('training and test share a source image')
        r['prediction'] = {k:v.tolist() for k,v in predictions(r, train).items()}
        scored.append(r)
    metrics = score(scored)
    for r in rows:
        for field in ('pixels','geometry','tipDelta','preTip','angleDelta','imageDigests'):
            r.pop(field, None)
        if 'truth' in r:
            r['truth'] = r['truth'].tolist()
    return dict(schema='motionbrain.offline_open_arm_response.v1', attempts=len(rows), comparable=len(scored),
                statusCounts=dict(Counter(r.get('hold','compared') for r in rows)),
                labels='Median of two post-OFF relative xy coordinates minus pre-plan relative xy, in native px. '
                       'Side tool minus object (center X, top Y); near object minus jaw midpoint. Positive Y down.',
                direction='X and Y directions each use a fixed +/-1 px stationary band in each camera.',
                split='Exclude whole held-out physical setup and require matching full command, stop profile and side configuration.',
                measurements='Landmark/pixel proxy outcomes; 24x16 RGB per view. Host receipt bounds, not exposure sync.',
                flags=['paired side markers','single side marker','near left jaw','near right jaw','near target'],
                metrics=metrics, rows=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('near_grasp', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = evaluate(args.near_grasp)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('attempts','comparable','statusCounts','metrics')}, indent=2))
