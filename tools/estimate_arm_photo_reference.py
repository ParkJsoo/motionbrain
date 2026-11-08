#!/usr/bin/env python3
"""사진 기반 근사 자세를 .local의 새 폴더에 저장한다. 장치 출력은 없다."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_ws/src/motionbrain_ros_bridge"))
from learning.arm_photo_reference import infer_reference  # noqa: E402


def _unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate_reference_key:" + key)
        result[key] = value
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / ".local"):
        parser.error("출력은 .local 아래 새 폴더여야 합니다")
    if output.exists():
        parser.error("기존 출력 폴더를 덮어쓰지 않습니다")
    raw = args.input.read_bytes()
    manifest = json.loads(raw, object_pairs_hook=_unique)
    result = infer_reference(manifest, ROOT)
    result["evidence"]["inputFileSha256"] = hashlib.sha256(raw).hexdigest()
    rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    output.mkdir(parents=True, exist_ok=False)
    (output / "reference.json").write_text(rendered, encoding="utf-8")
    print(json.dumps({"output": str(output), "status": result["status"], "controlEligible": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
