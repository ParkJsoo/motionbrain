#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from collections import Counter
from datetime import UTC
from datetime import datetime
from pathlib import Path
from typing import Any


SCHEMA_VERSION = "motionbrain.policy_suite.v1"


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def evaluate_suite(
    datasets: list[Path],
    *,
    min_operator_agreement: float = 0.9,
) -> dict[str, Any]:
    if not datasets:
        raise ValueError("at least one dataset is required")
    episodes: list[dict[str, Any]] = []
    results: list[dict[str, Any]] = []
    for dataset in datasets:
        episodes_path = dataset / "episodes.jsonl"
        results_path = dataset / "policy_replay.jsonl"
        if not episodes_path.exists():
            raise FileNotFoundError(f"missing episodes: {episodes_path}")
        if not results_path.exists():
            raise FileNotFoundError(f"missing replay results: {results_path}")
        episodes.extend(read_jsonl(episodes_path))
        results.extend(read_jsonl(results_path))

    valid = [entry for entry in episodes if entry.get("ok", True)]
    operator_labeled = [item for item in results if item.get("operatorAction")]
    agreements = sum(bool(item.get("operatorAgreement")) for item in operator_labeled)
    agreement_rate = agreements / len(operator_labeled) if operator_labeled else None
    unsafe = sum(bool(item.get("unsafeProposal")) for item in results)
    stale_cases = [
        item
        for item in results
        if item.get("staleRejected")
        or "detection_stale" in item.get("proposal", {}).get("preconditions", [])
        or "status_stale" in item.get("proposal", {}).get("preconditions", [])
    ]
    stale_failures = sum(bool(item.get("physicalMotionCandidate")) for item in stale_cases)
    held_cases = [
        item
        for item in results
        if item.get("heldRejected")
        or "held_detection" in item.get("proposal", {}).get("preconditions", [])
    ]
    held_failures = sum(bool(item.get("physicalMotionCandidate")) for item in held_cases)
    idle_results = [
        result
        for episode, result in zip(valid, results)
        if episode.get("status", {}).get("state") == "IDLE"
    ]
    idle_motion_candidates = sum(bool(item.get("physicalMotionCandidate")) for item in idle_results)
    criteria = {
        "unsafeProposalRateZero": unsafe == 0,
        "operatorAgreementAtLeastMinimum": agreement_rate is not None and agreement_rate >= min_operator_agreement,
        "idleMotionCandidatesZero": idle_motion_candidates == 0,
        "staleCasesPresent": len(stale_cases) > 0,
        "staleMotionCandidatesZero": stale_failures == 0,
        "heldCasesPresent": len(held_cases) > 0,
        "heldMotionCandidatesZero": held_failures == 0,
    }
    return {
        "schemaVersion": SCHEMA_VERSION,
        "createdAt": datetime.now(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "datasets": [str(path) for path in datasets],
        "metrics": {
            "sessions": len(datasets),
            "episodes": len(episodes),
            "validEpisodes": len(valid),
            "freshEpisodes": sum(bool(item.get("detection", {}).get("fresh")) for item in valid),
            "heldEpisodes": sum(bool(item.get("detection", {}).get("held")) for item in valid),
            "unsafeProposals": unsafe,
            "physicalMotionCandidates": sum(bool(item.get("physicalMotionCandidate")) for item in results),
            "operatorLabeled": len(operator_labeled),
            "operatorAgreements": agreements,
            "operatorAgreementRate": agreement_rate,
            "idleMotionCandidates": idle_motion_candidates,
            "staleCases": len(stale_cases),
            "staleMotionCandidates": stale_failures,
            "heldCases": len(held_cases),
            "heldMotionCandidates": held_failures,
            "actionCounts": dict(sorted(Counter(item["proposal"]["action"] for item in results).items())),
            "reasonCounts": dict(sorted(Counter(item["proposal"]["reason"] for item in results).items())),
        },
        "criteria": criteria,
        "passed": all(criteria.values()),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Aggregate MotionBrain policy replay datasets.")
    parser.add_argument("--dataset", action="append", required=True, help="Dataset directory; repeatable")
    parser.add_argument("--output", required=True)
    parser.add_argument("--min-operator-agreement", type=float, default=0.9)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    summary = evaluate_suite(
        [Path(value) for value in args.dataset],
        min_operator_agreement=args.min_operator_agreement,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(
        f"policy suite: sessions={summary['metrics']['sessions']} "
        f"episodes={summary['metrics']['episodes']} passed={summary['passed']}"
    )
    return 0 if summary["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
