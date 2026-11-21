"""Score manual press choices on disjoint episodes using saved frames and operator intent."""
import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
from PIL import Image

from learning.dual_view_capture import policy_example, read, verified
from learning.dual_view_policy import ACTIONS, DualViewPolicy

PICK = {'approach', 'pregrasp_open', 'close', 'held_lift'}
PLACE = {'lower', 'release'}
# The two later recordings have audited UI press times but no assembled IL manifest.
# These retrospective phases describe the five presses up through release in both.
LATER_PHASES = [('approach', 'shoulder/down'), ('close', 'gripper/close'),
                ('held_lift', 'shoulder/up'), ('lower', 'shoulder/down'),
                ('release', 'gripper/open')]


def later_episode(folder):
    name = folder.name
    views = {'external': [], 'gripper': []}
    for path in sorted((folder / 'external-stream/host-ingress').glob('[0-9][0-9][0-9][0-9].json')):
        f = read(path)
        views['external'].append(dict(id=f['frameId'], file='external-stream/' + f['imageFile'],
                                      sha256=f['imageSha256'], availableHostTime=f['stdoutReadAt'],
                                      sourceHostBounds=[f['sourceEarliest'], f['sourceLatest']]))
    clocks = {}
    for line in (folder / 'gripper-clock.jsonl').read_text().splitlines():
        clock = json.loads(line)
        clocks[clock['frame']['frameId']] = [clock['firstDmaLower'], clock['firstDmaUpper']]
    for path in sorted(folder.glob('gripper*/capture-*/source-binding.json')):
        binding = read(path)
        record = binding['record']
        frame_id = record['frame']['frameId']
        if frame_id in clocks:
            views['gripper'].append(dict(id=frame_id, file=str((path.parent/'esp32-frame.jpg').relative_to(folder)),
                                         sha256=binding['imageSha256'], availableHostTime=record['persistedAt'],
                                         sourceHostBounds=clocks[frame_id]))
    gestures = read(folder/'shadow-and-retreat-result.json')['gestures']
    actions = []
    for gesture, (phase, expected) in zip(gestures, LATER_PHASES):
        query = gesture['query']
        joint, direction = query['joint'][0], query['action'][0]
        if f'{joint}/{direction}' != expected or gesture['action'] != len(actions) + 1:
            raise ValueError(f'{name}: historical phase annotation does not match gesture')
        prior = {}
        for camera, frames in views.items():
            candidates = [f for f in frames if max(f['sourceHostBounds'][1], f['availableHostTime']) < gesture['press'][0]]
            if candidates:
                prior[camera] = max(candidates, key=lambda f: f['sourceHostBounds'][1])['id']
        actions.append(dict(index=gesture['action'], phase=phase, motor=ACTIONS[expected][0],
                            command=dict(joint=joint, direction=direction),
                            press=dict(hostBounds=gesture['press'], availablePriorObservation=prior),
                            release=dict(hostBounds=gesture['release'])))
    if len(actions) != len(LATER_PHASES):
        raise ValueError(f'{name}: incomplete gesture sequence')
    return dict(schemaVersion='motionbrain.manual_il_episode.v1', source='human_teleoperation',
                episode=name, frames=views, actions=actions), len(gestures) - len(actions)


def samples(folder, episode, plan):
    previous, released = 'START', float('-inf')
    rows = []
    frames = {camera: {frame['id']: frame for frame in records}
              for camera, records in episode['frames'].items()}
    for action in episode['actions']:
        phase, idx = action['phase'], action['index']
        label = action['command']['joint'] + '/' + action['command']['direction']
        if not released < action['press']['hostBounds'][0]:
            raise ValueError('prior gesture not released before current press')
        if phase in PICK | PLACE:
            pair = action['press']['availablePriorObservation']
            row = dict(episode=episode['episode'], index=idx, phase=phase,
                       goal='pick' if phase in PICK else 'place', label=label, previous=previous,
                       pair=pair, example=None)
            if set(pair) == {'external', 'gripper'} and all(pair.values()):
                at = action['press']['hostBounds'][1]
                row['inputAgeUpperSeconds'] = {camera: at - frames[camera][frame_id]['sourceHostBounds'][0]
                                               for camera, frame_id in pair.items()}
                row['example'] = policy_example(folder/'whole-arm-il-episode.json', episode, idx, plan)
            rows.append(row)
        previous, released = label, action['release']['hostBounds'][1]
    return rows


def score_fold(training, test, limits):
    # Both methods know the task segment. Neither receives the annotated fine phase.
    transition_counts = defaultdict(Counter)
    for row in training:
        transition_counts[row['previous']][row['label']] += 1
    examples = [r['example'] for r in training if r['example'] is not None]
    policy = DualViewPolicy(examples, limits)
    # Earlier full-image imitation benchmark: equal-camera mean squared error.
    joint_bank = np.stack([np.stack(example[1:]) for example in examples])
    scored = []
    for row in test:
        counts = transition_counts[row['previous']]
        baseline = min(counts, key=lambda label: (-counts[label], label)) if counts else 'hold'
        result = {k: v for k, v in row.items() if k != 'example'}
        result.update(previousCommand=baseline, repeatPrevious=row['previous'] if row['previous'] != 'START' else 'hold',
                      binocular='hold', binocularNearest='hold', missingPair=row['example'] is None)
        if row['example'] is not None:
            # policy_example already returns 8-bit BOX thumbnails normalized to [0,1].
            images = [Image.fromarray(np.rint(view*255).astype(np.uint8)) for view in row['example'][1:]]
            error = np.mean((joint_bank - np.stack(row['example'][1:]))**2, axis=(1, 2, 3, 4))
            result['binocularNearest'] = examples[int(np.argmin(error))][0]
            vote = policy.predict(*images)
            result.update(binocular=vote['action'], viewVotes=vote['viewVotes'], viewRmse=vote['viewRmse'])
        scored.append(result)
    return scored


def summarize(rows):
    return dict(total=len(rows), binocular=sum(r['binocular'] == r['label'] for r in rows),
                binocularNearest=sum(r['binocularNearest'] == r['label'] for r in rows),
                repeatPrevious=sum(r['repeatPrevious'] == r['label'] for r in rows),
                repeatPreviousHold=sum(r['repeatPrevious'] == 'hold' for r in rows),
                previousCommand=sum(r['previousCommand'] == r['label'] for r in rows),
                binocularHold=sum(r['binocular'] == 'hold' for r in rows),
                previousCommandHold=sum(r['previousCommand'] == 'hold' for r in rows),
                missingPair=sum(r['missingPair'] for r in rows))


def evaluate(root, plan):
    episodes = {}
    refs = defaultdict(list)
    for ref in plan['examples']:
        path = Path(ref['episode'])
        refs[path.parent.name].append(ref)
    excluded = {}
    for name, items in sorted(refs.items()):
        folder = root/name
        digests = {r['episodeSha256'] for r in items}
        if len(digests) != 1:
            raise ValueError('inconsistent demonstration digest')
        episode = json.loads(verified(folder/'whole-arm-il-episode.json', digests.pop()))
        if episode['episode'] != name or episode['splitGroup'] != name:
            raise ValueError('unexpected recording group')
        included = [r['actionIndex'] for r in items]
        if sorted(included) != list(range(1, len(episode['actions'])+1)):
            raise ValueError('expected full historical gesture sequence')
        episodes[name] = samples(folder, episode, plan)
        excluded[name] = len(episode['actions']) - len(episodes[name])
    bank_ids = sorted(episodes)
    for name in ('whole-arm-manual-user-08', 'whole-arm-manual-user-09'):
        if name in episodes:
            raise ValueError('later replay overlaps reference bank')
        episode, excluded[name] = later_episode(root/name)
        episodes[name] = samples(root/name, episode, plan)
    evaluations = []
    for name, tests in episodes.items():
        train_ids = [key for key in bank_ids if key != name]
        split = 'leaveOneEpisodeOut' if name in bank_ids else 'laterEpisodes'
        for goal in ('pick', 'place'):
            training = [r for key in train_ids for r in episodes[key] if r['goal'] == goal]
            test = [r for r in tests if r['goal'] == goal]
            rows = score_fold(training, test, plan['maxViewRmse'])
            evaluations.append(dict(split=split, goal=goal, episode=name, trainingEpisodes=train_ids,
                                    rows=rows, summary=summarize(rows)))
    totals = {}
    for split in ('leaveOneEpisodeOut', 'laterEpisodes'):
        totals[split] = {goal: summarize([row for e in evaluations if e['split'] == split and e['goal'] == goal
                                          for row in e['rows']]) for goal in ('pick', 'place')}
    return dict(view=plan['external']['viewConfigurationId'], maxViewRmse=plan['maxViewRmse'],
                evaluation='Retrospective, conditional on known pick/place segment and human press onset. '
                           'Start/previous completed human gesture predicts train-only modal next action; '
                           'lexical tie break and abstention for unseen history. Camera disagreement counts as a miss.',
                excludedAfterRelease=excluded, totals=totals, folds=evaluations)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--episodes', required=True, type=Path)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = evaluate(args.episodes, read(args.plan))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['totals'], indent=2))


if __name__ == '__main__':
    main()
