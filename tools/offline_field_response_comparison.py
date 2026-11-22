"""Replay matched field pulses with entire acquisition campaigns held out."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from offline_dual_response_comparison import describe, predictions, read, score
from learning.dual_view_capture import gripper_image, side_image
from learning.open_side_approach import LEGACY_SIDE_VIEW


def state(g):
    x,y,w,_ = g['sideObjectBox']
    return np.r_[np.array(g['sideToolPx'])-[x+w/2,y], np.array(g['nearObjectCenterPx'])-g['nearJawMidPx']]


def complete(g):
    return all(g.get(k) is not None for k in ('sideToolPx','sideObjectBox','nearObjectCenterPx','nearJawMidPx'))


def campaign(folder, *, expected_view=LEGACY_SIDE_VIEW):
    for path in sorted(folder.glob('metrics-trial-*.json')):
        m = read(path)
        trial = read(path.with_name(path.name.removeprefix('metrics-')))
        root = Path(trial['root'])
        p = trial['planned']
        a = trial['result']['action']
        if (p != a['request'] or not a['record']['outputOff'] or trial['captureExit'] or
                trial['result']['executionState']!='known_terminal' or a['outputLog']['dropped'] or
                a['outputLog']['bootId'] != p['bootId'] or a['record']['commandId'] != p['commandId']):
            raise ValueError('unverified trial or requested command')
        drive = [w for w in a['outputLog']['entries'] if w['pwm']]
        if not drive or any(w['motor'] != p['motor'] for w in drive):
            raise ValueError('expected single driven motor')
        if p['kind']=='pulse' and any(not 0 < w['pwm']*p['pwm'] <= p['pwm']**2 for w in drive):
            raise ValueError('output sign or power mismatch')
        q = trial['startPair']
        if max(q[k]['availableAt'] for k in ('extClock','nearClock')) > a['startHostBounds']['earliest']:
            raise ValueError('camera pair unavailable at command')
        pre = m['geometry']['startPair']
        if (pre['sideFrame'],pre['nearFrame']) != (q['frameId'],q['nearFrameId']):
            raise ValueError('pre label not bound to the chosen pair')
        plan = read(root/'plan.json')
        contract = {k: plan.get(k) for k in ('external','maxAge','maxSkew','maxViewRmse','scene','examples','responsePolicy','profiles')}
        contract_hash = hashlib.sha256(json.dumps(contract,sort_keys=True).encode()).hexdigest()
        profile = [trial['label'],p['kind'],p['pwm'],p['durationMs'],p['targetDegrees'],p['percent']]
        row = dict(campaign=folder.name, trial=path.stem[8:], episode=q['frameId'].split(':')[0],
                   command=profile, configuration=contract_hash, startShoulderDeg=trial['initial']['shoulderAngle']['angleDeg'],
                   peakPwm=max((w['pwm'] for w in drive),key=abs), driverMs=m['driverWindowDurationMs'],
                   before=[q['frameId'],q['nearFrameId']], view=plan['external']['viewConfigurationId'])
        if row['view'] != expected_view:
            raise ValueError('unexpected field camera')
        afters = m['geometryBursts']['lateOff']
        row['completePostPairs'] = sum(complete(o) for o in afters)
        if not complete(pre) or row['completePostPairs']<2:
            row['hold'] = 'missing_pre_or_two_post_pair_labels'
        else:
            post = [o for o in afters if complete(o)]
            ids = [(o['sideFrame'],o['nearFrame']) for o in post]
            clocks = {(r['frameId'],r['nearFrameId']):r for r in map(json.loads,(root/'decisions.jsonl').read_bytes().splitlines())}
            if (any(len({i[k] for i in [tuple(row['before']),*ids]}) != len(ids)+1 for k in (0,1))
                or any(clocks[i][k]['sourceEarliest'] <= a['outputOffHostBounds']['latest']
                       for i in ids for k in ('extClock','nearClock'))):
                raise ValueError('post pairs repeated or not strictly after terminal OFF')
            row['post'] = ids
            row['truth'] = (np.median([state(o) for o in post],axis=0)-state(pre)).tolist()
            row['_pre'] = pre
            row['_root'] = root
        yield row


def visual_input(row):
    root = row['_root']
    side_id, near_id = row['before']
    refs = {r['frameId']:Path(r['receipt']) for r in map(json.loads,(root/'display.jsonl').read_bytes().splitlines())}
    side_meta = read(refs[side_id])
    binding = read(refs[near_id])
    if side_meta['frameId']!=side_id or binding['record']['frame']['frameId']!=near_id:
        raise ValueError('display and source receipt differ')
    side = side_image(root/'external-stream',side_meta,read(root/'plan.json')['external'])
    near = gripper_image(refs[near_id].parent,binding)
    g = row['_pre']
    pre = dict(state=state(g).tolist(), box=g['sideObjectBox'], gap=g['nearJawGapPx'],
               upright=True, angle=row['startShoulderDeg'])
    row['pixels'],row['geometry'],row['flags'] = describe(np.array(side.convert('RGB')),np.array(near),pre)


def evaluate(folders):
    rows = [r for folder in folders for r in campaign(folder)]
    scored = []
    for r in rows:
        if 'truth' not in r:
            continue
        train = [s for s in rows if s['campaign']!=r['campaign'] and s['command']==r['command'] and
                 s['configuration']==r['configuration'] and 'truth' in s]
        if not train:
            r['hold']='no_other_campaign_same_profile_with_complete_labels'
            continue
        if any(s['episode']==r['episode'] for s in train):
            raise ValueError('campaigns share episode')
        for s in [*train,r]:
            if 'pixels' not in s:
                visual_input(s)
        r['prediction']={k:v.tolist() for k,v in predictions(r,train).items()}
        r['fitEpisodes']=[s['episode'] for s in train]
        scored.append(r)
    metrics=score(scored)
    for r in rows:
        for k in ('_root','_pre','pixels','geometry'):
            r.pop(k,None)
    return dict(attempts=len(rows),comparable=len(scored),statusCounts=dict(Counter(r.get('hold','compared') for r in rows)),
                split='Entire field campaign held out. Match planned profile and camera/feedback contract. '
                      'Median of at least two distinct late-OFF labels; other campaigns only.',
                metrics=metrics, rows=rows)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folders', type=Path,nargs='+')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=evaluate(args.folders)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
