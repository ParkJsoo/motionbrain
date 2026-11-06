#!/usr/bin/env python3
"""고정 배경·전완 표식의 투영 상태를 오프라인 학습하거나 고정 모델로 평가한다."""
import argparse
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.artifacts import ArtifactStore
from learning.manual_collection_dataset import digest, require
from learning import projected_arm_learning as learning


def run(manifest_path, output_dir, *, frozen_model=None, model_sha256=None, root=ROOT):
    output_dir = Path(output_dir)
    require(output_dir.resolve().is_relative_to((Path(root)/'.local').resolve()), 'private_output_required')
    require(not output_dir.exists(), 'output_already_exists')
    require((frozen_model is None) == (model_sha256 is None), 'frozen_model_and_hash_required_together')
    if model_sha256 is not None:
        require(isinstance(model_sha256, str) and re.fullmatch('[0-9a-f]{64}', model_sha256) is not None,
                'model_sha256_required')
    raw = Path(manifest_path).read_bytes()
    manifest = json.loads(raw)
    rows, rejected = learning.load_manifest(manifest, root)
    groups = manifest.get('evaluationGroups', [])
    training = [r for r in rows if r['captureGroup'] not in groups]
    evaluation = [r for r in rows if r['captureGroup'] in groups]
    if frozen_model is not None:
        model_raw = Path(frozen_model).read_bytes()
        require(digest(model_raw) == model_sha256, 'frozen_model_hash_changed')
        require(not training, 'frozen_evaluation_manifest_must_only_list_evaluation_groups')
        model = json.loads(model_raw)
        require(model.get('codeSha256') == digest((ROOT/'learning/projected_arm_learning.py').read_bytes())
                and model.get('commandContractCodeSha256') == digest(
                    (ROOT/'learning/relative_action_learning.py').read_bytes()), 'frozen_model_code_changed')
        report = learning.evaluate(model, evaluation)
        model_refit = False
    else:
        require(bool(training), 'eligible_training_transitions_required')
        model = learning.fit(training)
        report = learning.evaluate(model, evaluation)
        model_refit = True
    report.update(sourceRejected=rejected, manifestSha256=digest(raw),
        codeSha256=digest((ROOT/'learning/projected_arm_learning.py').read_bytes()),
        cliCodeSha256=digest(Path(__file__).read_bytes()), trainingPerformed=model_refit,
        modelRefit=model_refit, completedMonotonic=time.monotonic(),
        semantics='4_pixel_marker_state_not_calibrated_tool_pose_or_joint_angle')
    store = ArtifactStore(output_dir)
    if model_refit:
        model['sourceManifestSha256'] = digest(raw)
        ref = store.write_json('model.json', model)
        report['modelSha256'] = ref['sha256']
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
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
