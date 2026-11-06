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
from learning.relative_model_bundle import load_frozen_model

ROOT = Path(__file__).resolve().parents[1]


def run(manifest_path, output_dir, *, frozen_model=None, model_sha256=None, root=ROOT):
    output_dir = Path(output_dir)
    if not output_dir.resolve().is_relative_to((Path(root)/'.local').resolve()):
        raise ValueError('private_output_required')
    if (frozen_model is None) != (model_sha256 is None):
        raise ValueError('frozen_model_and_hash_required_together')
    raw = Path(manifest_path).read_bytes(); manifest = json.loads(raw)
    groups = manifest.get('evaluationGroups', [])
    if not isinstance(groups, list) or not all(isinstance(g, str) and g for g in groups):
        raise ValueError('evaluation_groups_required')
    # Check the declared groups too: a rejected source must not conceal a
    # training record in a supposedly frozen evaluation manifest.
    if frozen_model is not None and any(
            item['annotation']['captureGroup'] not in groups for item in manifest['records']):
        raise ValueError('frozen_evaluation_manifest_must_only_list_evaluation_groups')
    model = load_frozen_model(frozen_model, model_sha256) if frozen_model is not None else None
    rows, rejected = load_manifest(manifest, root)
    training = [r for r in rows if r['captureGroup'] not in groups]
    evaluation = [r for r in rows if r['captureGroup'] in groups]
    if frozen_model is None:
        model = fit(training) if training else None
    report = evaluate(model, evaluation) if model else dict(
        status='unavailable', reason='no_eligible_training_transitions', evaluated=0,
        rejected=[], metrics={}, controlEligible=False, independentTest=False)
    report.update(sourceRejected=rejected, manifestSha256=digest(raw),
                  codeSha256=digest((ROOT/'learning/relative_action_learning.py').read_bytes()),
                  trainingPerformed=frozen_model is None and model is not None,
                  modelRefit=frozen_model is None and model is not None)
    query = manifest.get('query')
    if query and model:
        report['selection'] = select(model, query['state'], query['target'], query['commands'], query['context'])
    store = ArtifactStore(output_dir)
    if frozen_model is None:
        ref = store.write_json('model.json', model)
        report['modelSha256'] = ref['sha256'] if model else None
    else:
        report['modelSha256'] = model_sha256
    store.write_json('report.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--frozen-model', type=Path)
    parser.add_argument('--model-sha256')
    args = parser.parse_args(argv)
    try:
        report = run(args.manifest, args.output_dir, frozen_model=args.frozen_model,
                     model_sha256=args.model_sha256)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        parser.error(str(exc))
    print(json.dumps(dict(evaluated=report['evaluated'], rejected=len(report['rejected']),
                          sourceRejected=len(report['sourceRejected']), metrics=report['metrics'],
                          modelRefit=report['modelRefit'], controlEligible=False)))


if __name__ == '__main__':
    main()
