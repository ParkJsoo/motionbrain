#!/usr/bin/env python3
"""평면 팔 치수의 파생물 검사 또는 별도 로컬 후보 생성. 실물 출력 없음."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/motionbrain_ros_bridge'))
sys.path.insert(0, str(ROOT))

from motionbrain_ros_bridge.arm_geometry import load_geometry_profile, profile_digest
from motionbrain_ros_bridge.arm_geometry_urdf import URDF_PATHS, render_urdf, geometry_mismatches


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, help='생략하면 패키지 기본 치수 사용')
    parser.add_argument('--evidence', type=Path, action='append', default=[],
                        help='measured 항목의 SHA-256과 일치하는 비공개 원본 파일. 반복 지정 가능')
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true', help='체크인된 5개 URDF와 비교')
    mode.add_argument('--output', type=Path, help='.local 아래 새 후보 폴더')
    args = parser.parse_args(argv)
    profile = load_geometry_profile(args.profile)
    required_evidence = {v['evidence_sha256'] for v in profile['parameters'].values() if v['status'] == 'measured'}
    supplied_evidence = {hashlib.sha256(p.read_bytes()).hexdigest() for p in args.evidence}
    # A distributable default profile can refer to private measurement records.
    # Reproducing its geometry does not require publishing those raw records.
    # Importing a new measured profile does require the actual source files.
    verify_evidence = args.profile is not None or bool(args.evidence)
    if verify_evidence and required_evidence != supplied_evidence:
        parser.error('measured 항목의 근거 해시와 --evidence 파일 집합이 일치해야 합니다')
    derivatives = {}
    failures = {}
    sources = {}
    for relative in URDF_PATHS:
        path = ROOT / 'ros2_ws/src' / relative
        text = path.read_text()
        derivatives[path.name] = render_urdf(text, profile)
        sources[relative] = hashlib.sha256(path.read_bytes()).hexdigest()
        mismatches = geometry_mismatches(text, profile)
        if mismatches:
            failures[relative] = mismatches
    if args.check:
        print(json.dumps(dict(profileSha256=profile_digest(profile), mismatches=failures,
                              physicallyCalibrated=False), ensure_ascii=False))
        return int(bool(failures))
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / '.local'):
        parser.error('후보는 .local 아래 새 폴더에만 생성합니다')
    output.mkdir(parents=True, exist_ok=False)
    for name, text in derivatives.items():
        (output / name).write_text(text)
    (output / 'arm_geometry.json').write_text(json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    from learning.robot_visualization import generate_models
    generate_models(output / 'motionbrain.urdf', output)
    files = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.is_file()}
    manifest = dict(schema='motionbrain.arm_geometry.bundle.v1', profileSha256=profile_digest(profile),
                    sourceUrdfSha256=sources, generatedSha256=files,
                    geometryStatus=profile['status'], physicallyCalibrated=False,
                    measurementEvidenceFilesVerified=verify_evidence and bool(required_evidence),
                    feedbackCalibrated=False, dynamicsCalibrated=False, contactCalibrated=False,
                    controlEligible=False, dimensionsOnly=True)
    (output / 'geometry-manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(output), geometryStatus=profile['status'],
                          profileSha256=manifest['profileSha256'], controlEligible=False)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
