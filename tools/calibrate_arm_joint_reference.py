#!/usr/bin/env python3
"""높이와 저장된 정지 관측으로 근사 M4 기준점을 만든다. 장치에 적용하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_ws/src/motionbrain_ros_bridge"))
from learning.arm_joint_reference import infer_joint_reference
from learning.artifacts import _decode_json


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / ".local") or output.exists():
        parser.error("출력은 .local 아래 존재하지 않는 새 폴더여야 합니다")
    raw = args.input.read_bytes()
    result = infer_joint_reference(_decode_json(raw), ROOT)
    result["evidence"]["inputFileSha256"] = hashlib.sha256(raw).hexdigest()
    rendered = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    output.mkdir(parents=True, exist_ok=False)
    (output / "joint-reference.json").write_text(rendered, encoding="utf-8")
    print(json.dumps({"output": str(output), "status": result["status"], "controlEligible": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
