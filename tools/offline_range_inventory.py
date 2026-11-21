"""Inventory human range gestures and replay endpoint landmarks without device access."""

import argparse
import json
import sys
from collections import defaultdict
from functools import lru_cache
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.dual_view_capture import read, side_image
from learning.side_part_overlay import render_side
from learning.supervised_transport import JOINTS


def lines(path):
    with path.open() as source:
        return [json.loads(line) for line in source]


def intent_rows(folder):
    events = {}
    bounds = {}
    results = {}
    for batch in lines(folder / 'ui-events/ui-events.jsonl'):
        for b in batch.get('eventHostBounds', []):
            bounds[b['pageId'], b['sequence']] = b
        for e in batch.get('events', []):
            events[e['pageId'], e['sequence']] = e
            if e['kind'] == 'command_result':
                results[e['pageId'], e['requestSequence']] = e['success']
    starts = []
    for key, event in events.items():
        q = event.get('query', {})
        if event['kind'] != 'command_intent' or q.get('lease') != 'start':
            continue
        press = bounds[event['pageId'], event['domEventSequence']]
        starts.append(dict(episode=folder.name, gesture=event['gestureId'], joint=q['joint'],
                           direction=q['action'], requestedPercent=int(q['percent']),
                           press=press['sourceEarliest'], accepted=results.get(key)))
    return sorted(starts, key=lambda row: row['press'])


def output_bouts(records, statuses, boot):
    # Anchor each MCU write to the nearest status round trip. Use its upper bound
    # for post-OFF frames. Discard the historical ring snapshot on the first read.
    reads = [r for r in records if r['path'].split('?')[0] == '/action_outputs' and 'response' in r]
    if any(r['response']['bootId'] != boot for r in reads):
        raise ValueError('controller reboot')
    first = max((e['sequence'] for e in reads[0]['response']['entries']), default=0)
    entries = {e['sequence']: e for r in reads for e in r['response']['entries'] if e['sequence'] > first}
    active = {}
    bouts = []
    for seq, e in sorted(entries.items()):
        if seq > first + 1 and seq - 1 not in entries:
            raise ValueError('driver history gap')
        anchor = min(statuses, key=lambda r: abs(r['response']['uptimeMs'] - e['atMs']))
        offset = (e['atMs'] - anchor['response']['uptimeMs']) / 1000
        lo, hi = anchor['startedAt'] + offset - .002, anchor['receivedAt'] + offset + .002
        motor, pwm = e['motor'], e['pwm']
        if pwm:
            if motor not in active:
                bout = dict(motor=motor, startLower=lo, startUpper=hi, startMs=e['atMs'], peak=pwm)
                active[motor] = bout
                bouts.append(bout)
            elif pwm * active[motor]['peak'] <= 0:
                raise ValueError('sign reversal without OFF')
            elif abs(pwm) > abs(active[motor]['peak']):
                active[motor]['peak'] = pwm
        elif motor in active:
            b = active.pop(motor)
            b.update(offUpper=hi, durationMs=e['atMs'] - b['startMs'], stopReason=e['reason'])
    return bouts


def episode(folder, archive):
    rows = intent_rows(folder)
    if not rows and not (folder / 'controller.jsonl').is_file():
        return [], None
    plan = read(folder / 'plan.json')
    records = lines(folder / 'controller.jsonl')
    statuses = [r for r in records if r['path'] == '/status' and 'response' in r]
    bouts = output_bouts(records, statuses, plan['controllerBootId'])
    if not rows:
        return [], bouts
    config = plan['external']
    media = Path(read(folder / 'live-status.json')['root'])
    if not media.is_dir():
        media = archive / media.name
    pairs = lines(media / 'decisions.jsonl')
    near = {r['frameId']: r['observation']['parts'] for r in lines(media / 'display.jsonl')
            if r.get('camera') == 'gripper' and r.get('kind') == 'overlay'}
    @lru_cache(None)
    def labels(frame, near_id):
        stream = media / 'external-stream'
        i = int(frame.rsplit(':', 1)[1])
        receipt = read(stream / 'host-ingress' / f'{i:04}.json')
        image = side_image(stream, receipt, config).convert('RGB')
        _, side = render_side(np.asarray(image), view_id=config['viewConfigurationId'])
        parts = near.get(near_id, {})
        left, right = [parts.get(k, {}).get('tip') for k in ('leftJaw', 'rightJaw')]
        gap = right[0] - left[0] if left and right else None
        return dict(sideFrame=frame, nearFrame=near_id, sideParts=side['parts'],
                    sideLandmarks=side['landmarks'], nearJawGapPx=gap)

    for i, row in enumerate(rows):
        start = row['press']
        end = rows[i + 1]['press'] if i + 1 < len(rows) else float('inf')
        before = [r for r in pairs if max(r[k]['availableAt'] for k in ('extClock', 'nearClock')) < start
                  and min(r[k]['sourceEarliest'] for k in ('extClock', 'nearClock')) >= start - 1.2]
        prior_status = [r for r in statuses if start - 2 <= r['receivedAt'] < start]
        row.update(view=config['viewConfigurationId'], plannedDurationMs=None,
                   shoulderBeforeDeg=prior_status[-1]['response']['shoulderAngle']['angleDeg'] if prior_status else None,
                   pre=labels(before[-1]['frameId'], before[-1]['nearFrameId']) if before else None,
                   post=None, driver=None)
        if row['accepted'] is not True:
            continue
        motor, signs = JOINTS[row['joint']]
        matches = [b for b in bouts if b['motor'] == motor and b['peak'] * signs[row['direction']] > 0
                   and start <= b['startUpper'] and b['startLower'] < end and b['startUpper'] < start + 2]
        if len(matches) != 1:
            continue
        b = matches[0]
        row['driver'] = b
        if 'offUpper' not in b:
            continue
        off = b['offUpper']
        after = [r for r in pairs if min(r[k]['sourceEarliest'] for k in ('extClock', 'nearClock')) >= off + .3
                 and max(r[k]['availableAt'] for k in ('extClock', 'nearClock')) < min(off + 2.5, end)]
        if after:
            row['post'] = labels(after[0]['frameId'], after[0]['nearFrameId'])
    matched = [r['driver'] for r in rows if r['driver']]
    if len({id(b) for b in matched}) != len(matched):
        raise ValueError('ambiguous driver match')
    return rows, [b for b in bouts if b not in matched]



def fit_range_means(rows):
    """Describe training support for each manual hold profile, including missing targets."""
    profiles = defaultdict(list)
    for row in rows:
        key = (row['view'], row['joint'], row['direction'], row['requestedPercent'])
        profiles[key].append(row)
    result = []
    for (view, joint, direction, percent), group in profiles.items():
        side, near = [], []
        measured = [r for r in group if r['driver'] is not None]
        for row in measured:
            if not (row['pre'] and row['post']):
                continue
            a, b = row['pre'], row['post']
            tip_a, tip_b = (o['sideLandmarks'].get('visibleJawTip') for o in (a, b))
            if tip_a is not None and tip_b is not None:
                side.append(np.subtract(tip_b, tip_a))
            if a['nearJawGapPx'] is not None and b['nearJawGapPx'] is not None:
                near.append(b['nearJawGapPx'] - a['nearJawGapPx'])
        result.append(dict(view=view, joint=joint, direction=direction, requestedPercent=percent,
                           episodes=sorted({r['episode'] for r in measured}), starts=len(group),
                           driverMatched=len(measured), sideLabelled=len(side), nearLabelled=len(near),
                           meanSideJawTipDeltaPx=np.mean(side, axis=0).tolist() if side else None,
                           meanNearJawGapDeltaPx=float(np.mean(near)) if near else None))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('range_root', type=Path)
    parser.add_argument('--archive', type=Path, required=True)
    args = parser.parse_args()
    rows = []
    unmatched = {}
    sessions = {}
    for ui in sorted(args.range_root.glob('*/ui-events/ui-events.jsonl')):
        folder = ui.parents[1]
        gestures, unmatched[folder.name] = episode(folder, args.archive)
        sessions[folder.name] = len(gestures)
        rows.extend(gestures)
    for row in rows:
        row['otherEpisodesSameProfile'] = sorted({r['episode'] for r in rows
            if r['episode'] != row['episode'] and r['driver'] and all(
                row[k] == r[k] for k in ('view', 'joint', 'direction', 'requestedPercent'))})
    print(json.dumps(dict(gestures=rows, sessionStarts=sessions, unmatchedOutputs=unmatched,
                          developmentProfileMeans=fit_range_means(rows)),
                     indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
