#!/usr/bin/env python3
"""저장 영상의 상대 상태 전이를 학습하고 후보를 오프라인 비교한다."""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from learning.artifacts import ArtifactStore
from learning.manual_collection_dataset import digest
from learning.relative_action_learning import load_manifest, fit, evaluate, select


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    local = Path(__file__).resolve().parents[1]/'.local'
    if not args.output_dir.resolve().is_relative_to(local):
        parser.error('출력은 저장소 .local 아래여야 합니다')
    raw = args.manifest.read_bytes(); manifest = json.loads(raw)
    rows, rejected = load_manifest(manifest, Path(__file__).resolve().parents[1])
    groups = manifest.get('evaluationGroups', [])
    if not isinstance(groups, list) or not all(isinstance(g, str) and g for g in groups):
        raise ValueError('evaluation_groups_required')
    training = [r for r in rows if r['captureGroup'] not in groups]
    evaluation = [r for r in rows if r['captureGroup'] in groups]
    model = fit(training) if training else None
    report = evaluate(model, evaluation) if model else dict(
        status='unavailable', reason='no_eligible_training_transitions', evaluated=0,
        rejected=[], metrics={}, controlEligible=False, independentTest=False)
    report.update(sourceRejected=rejected, manifestSha256=digest(raw),
                  codeSha256=digest((Path(__file__).resolve().parents[1]/'learning/relative_action_learning.py').read_bytes()))
    query = manifest.get('query')
    if query and model:
        report['selection'] = select(model, query['state'], query['target'], query['commands'], query['context'])
    store = ArtifactStore(args.output_dir)
    store.write_json('model.json', model); store.write_json('report.json', report)
    print(json.dumps(dict(evaluated=report['evaluated'], rejected=len(report['rejected']),
                          sourceRejected=len(rejected), metrics=report['metrics'], controlEligible=False)))


if __name__ == '__main__':
    main()
