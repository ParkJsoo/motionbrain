#!/usr/bin/env python3
"""같은 촬영 첫 기준 영상과 supervised 출력 후 영상을 오프라인으로 대조한다."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.arm_session_reference import compare_run, load_template
from learning.artifacts import ArtifactStore, _read


def private_path(value):
    requested = Path(value).absolute()
    resolved = requested.resolve()
    if requested != resolved or not resolved.is_relative_to((ROOT / '.local').resolve()):
        raise ValueError('canonical_private_path_required')
    return resolved


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--arm-reference-template', type=Path, required=True)
    parser.add_argument('--expected-plan-sha256', required=True)
    parser.add_argument('--expected-template-sha256', required=True)
    parser.add_argument('--allow-offline-registration', action='store_true',
                        help='explicit historical reconstruction; never claims pre-action registration')
    args = parser.parse_args(argv)
    run, output = private_path(args.run), private_path(args.output)
    if output.exists() or output == run or run in output.parents:
        raise ValueError('new_separate_output_required')
    template = _read(args.arm_reference_template.parent, args.arm_reference_template.name)
    load_template(template, args.expected_template_sha256)
    result, images = compare_run(run, template, args.expected_plan_sha256,
                                allow_offline_registration=args.allow_offline_registration)
    store = ArtifactStore(output)
    result['images'] = {}
    import cv2
    for label, image in zip(('before', 'after'), images):
        ok, encoded = cv2.imencode('.png', image)
        if not ok:raise ValueError('comparison_png_encode_failed')
        result['images'][label] = store._write(label + '.png', encoded.tobytes())
    store.write_json('comparison.json', result)
    print(json.dumps(dict(summary=result['summary'], preActionReferenceVerified=result['preActionReferenceVerified'],
                         sourceSchema=result['sourceSchema'], newPhysicalCommands=0), ensure_ascii=False))
    return 0 if result['summary']['rejected'] == 0 else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(json.dumps(dict(error=type(exc).__name__, reason=str(exc) if isinstance(exc, ValueError)
                              else 'failed_preserve_source', newPhysicalCommands=0)), file=sys.stderr)
        raise SystemExit(2)
