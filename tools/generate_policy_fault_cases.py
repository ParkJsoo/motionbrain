#!/usr/bin/env python3

from __future__ import annotations

import argparse
import copy
import json
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any


SCHEMA_VERSION = "motionbrain.policy_fault_cases.v1"


def generate_fault_cases(source: Path, output: Path, mode: str) -> Path:
    if mode not in {"stale", "held"}:
        raise ValueError("mode must be stale or held")
    entries = [
        json.loads(line)
        for line in (source / "episodes.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    output.mkdir(parents=True, exist_ok=False)
    generated: list[dict[str, Any]] = []
    for entry in entries:
        if not entry.get("ok", True):
            continue
        item = copy.deepcopy(entry)
        item["operatorAction"] = "hold"
        item["faultInjection"] = {
            "mode": mode,
            "provenance": "offline_fault_injection",
            "sourceDataset": str(source),
            "sourceIndex": entry.get("index"),
        }
        detection = item.setdefault("detection", {})
        guard = item.setdefault("controlGuard", {})
        if mode == "stale":
            detection["fresh"] = False
            detection["held"] = False
            guard["detectionFresh"] = False
        else:
            detection["fresh"] = True
            detection["held"] = True
            guard["detectionFresh"] = True
        item.pop("policyProposal", None)
        generated.append(item)

    manifest = {
        "schemaVersion": SCHEMA_VERSION,
        "createdAt": datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "sourceDataset": str(source),
        "mode": mode,
        "provenance": "offline_fault_injection",
        "samples": len(generated),
        "physicalExecution": False,
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "episodes.jsonl").write_text(
        "".join(json.dumps(item, separators=(",", ":"), sort_keys=True) + "\n" for item in generated),
        encoding="utf-8",
    )
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate provenance-marked offline policy fault cases.")
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--mode", choices=("stale", "held"), required=True)
    args = parser.parse_args()
    output = generate_fault_cases(Path(args.source), Path(args.output), args.mode)
    print(f"generated policy fault cases: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
