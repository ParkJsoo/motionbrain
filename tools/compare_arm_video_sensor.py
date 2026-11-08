#!/usr/bin/env python3
"""기존 M4 영상·센서 기록을 .local의 새 폴더에 사후 대조한다. 장치 출력 없음."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from learning.arm_motion_comparison import compare_manifest, sha
from learning.artifacts import _decode_json


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--series-stride', type=int, default=0, help='0은 전후만, 양수는 고정 기준 프레임에 대한 원본 영상 표본 간격')
    a = p.parse_args(argv)
    output = a.output.resolve()
    if not output.is_relative_to(ROOT / '.local') or output.exists():
        p.error('.local 아래 새 출력 폴더가 필요합니다')
    raw = a.manifest.read_bytes()
    result = compare_manifest(_decode_json(raw), ROOT, series_stride=a.series_stride)
    result['inputManifestSha256'] = sha(raw)
    result['analysisCodeSha256'] = {name:sha((ROOT/name).read_bytes()) for name in
                                   ['learning/arm_motion_observation.py', 'learning/arm_motion_comparison.py']}
    result['cliCodeSha256'] = sha(Path(__file__).read_bytes())
    output.mkdir(parents=True, exist_ok=False)
    (output / 'comparison.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result['summary'], ensure_ascii=False))
    return int(bool(result['summary']['rejected']))


if __name__ == '__main__':
    raise SystemExit(main())
