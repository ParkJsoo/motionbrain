"""관측 전용 접근 모델을 실제 개발 명령의 원본 측면 영상에서 만든다."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from offline_field_response_comparison import campaign, read
from learning.action_execution import checked_record, finite, selected_outputs
from learning.dual_view_capture import side_image
from learning.open_side_approach import WIDE_SIDE_VIEW, observe_open_side
from learning.timed_pick_evidence import APPROACH_LABELS, SIDE_RESPONSE_SCHEMA


def native_geometry(root, external, frame_id, clock):
    session, number = frame_id.rsplit(':',1)
    receipt = read(root/'external-stream'/'host-ingress'/f'{int(number):04}.json')
    if receipt['frameId'] != frame_id or session != root.name:
        raise ValueError('native_frame_session_mismatch')
    raw_clock = dict(sourceEarliest=receipt['sourceEarliest'],sourceLatest=receipt['sourceLatest'],
                     availableAt=receipt['stdoutReadAt'])
    if (receipt['qualityFlags'] != [] or clock != raw_clock or not all(map(finite,raw_clock.values()))
            or not raw_clock['sourceEarliest'] <= raw_clock['sourceLatest'] <= raw_clock['availableAt']):
        raise ValueError('native_source_clock_or_quality_mismatch')
    image = side_image(root/'external-stream',receipt,external)
    view = external['viewConfigurationId']
    return observe_open_side(np.asarray(image.convert('RGB')), view_id=view,
                             diagnose_green_core=view==WIDE_SIDE_VIEW)


def build(plan, folders):
    samples = []
    skipped = []
    for folder in folders:
        for item in campaign(folder,expected_view=plan['external']['viewConfigurationId']):
            label = item['command'][0]
            if label not in APPROACH_LABELS or label not in plan['profiles'] or 'truth' not in item:
                continue
            trial = read(folder/(item['trial']+'.json'))
            action = trial['result']['action']
            request = action['request']
            command = {k:v for k,v in request.items() if k not in ('bootId','commandId')}
            if command != plan['profiles'][label]['command']:
                continue
            root = item['_root']
            source = read(root/'plan.json')
            if any(source[k]!=plan[k] for k in ('external','scene','gripperBootId')):
                raise ValueError('mixed_native_response_contracts')
            record = checked_record(action['record'], request)
            bounds = action['outputSequenceBounds']
            # The archived executor retained the selected log and its sequence range.
            previous = dict(bootId=request['bootId'],dropped=0,entries=[dict(sequence=bounds['first']-1)])
            entries = selected_outputs(action['outputLog'],record,previous)['entries']
            if entries[-1]['sequence'] != bounds['last']:
                raise ValueError('native_terminal_output_range_changed')
            motor = [v for v in entries if v['motor']==request['motor']]
            if (record['state'] not in ('completed','timed_out') or record['bootId']!=request['bootId']
                    or any(e['actionOrdinal']!=record['ordinal'] for e in entries)
                    or not motor or motor[-1]['pwm']!=0):
                raise ValueError('unbound_native_driver_response')
            # Restrict ingestion to the archived energized development protocols.
            if (trial['role'] not in ('agent_selected_response_probe','current_pose_command_response',
                                      'agent_directed_rear_correction_development')
                    or trial['humanDemonstration'] is not False):
                raise ValueError('unclassified_collection_protocol')
            clocks = {}
            for pair in map(json.loads,(root/'decisions.jsonl').read_bytes().splitlines()):
                key = (pair['frameId'],pair['nearFrameId'])
                if key in clocks and clocks[key] != pair['extClock']:
                    raise ValueError('conflicting_native_frame_clocks')
                clocks[key] = pair['extClock']
            if (clocks[tuple(item['before'])] != trial['startPair']['extClock']
                    or max(trial['startPair'][k]['availableAt'] for k in ('extClock','nearClock'))
                        >= action['startHostBounds']['earliest']):
                raise ValueError('native_start_pair_clock_changed')
            try:
                pre = native_geometry(root,plan['external'],item['before'][0],clocks[tuple(item['before'])])
                post = [native_geometry(root,plan['external'],pair[0],clocks[tuple(pair)]) for pair in item['post']]
            except ValueError as exc:
                skipped.append(dict(episode=item['episode'],reason=str(exc)))
                continue
            if any(v['trackingMode'] != pre['trackingMode'] for v in post):
                skipped.append(dict(episode=item['episode'],reason='post_tracking_mode_changed'))
                continue
            samples.append(dict(label=label, command=command, episode=item['episode'],
                sourceRole=trial['role'],terminal=record['state'], shoulderDeg=item['startShoulderDeg'],
                commandId=request['commandId'],peakPwm=item['peakPwm'],driverMs=item['driverMs'],
                beforePair=item['before'], afterPairs=item['post'],
                beforeOffset=pre['sideOffsetPx'], afterOffsets=[v['sideOffsetPx'] for v in post],
                trackingMode=pre['trackingMode'], afterTrackingModes=[v['trackingMode'] for v in post]))
    if len({r['episode'] for r in samples})!=len(samples):
        raise ValueError('duplicate_training_episode')
    return dict(schema=SIDE_RESPONSE_SCHEMA,controlEligible=False,external=plan['external'],
                scene=plan['scene'],gripperBootId=plan['gripperBootId'],samples=samples, skipped=skipped)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('folders',nargs='+',type=Path)
    args=parser.parse_args()
    model=build(read(args.plan),args.folders)
    args.output.write_text(json.dumps(model,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(samples=len(model['samples']),sha256=hashlib.sha256(args.output.read_bytes()).hexdigest())))
