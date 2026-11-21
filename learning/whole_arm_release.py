"""Two-camera release evidence for deciding when to stop OPEN and recheck."""

import math
import re

from learning.limited_scene_cycle_assessment import _table, released
from learning.near_gripper_tip_observer import validate_observation

SCHEMA = "motionbrain.prism_obs3_release_pair.v1"


def validate_geometry(external):
    if re.fullmatch(r"[0-9a-f]{64}", external["imageSha256"]) is None:
        raise ValueError("invalid_external_sha")
    o = external["object"]
    x, y, w, h = o["bbox"]
    if not all(type(v) in (int, float) and math.isfinite(v) for v in (x, y, w, h)):
        raise ValueError("invalid_external_bbox")
    if not (
        0 <= x < x + w <= 1920
        and 0 <= y < y + h <= 1080
        and o["bottom"] == y + h
        and o["center"] == [x + w / 2, y + h / 2]
        and o["upright"] is (h >= 2 * w)
    ):
        raise ValueError("inconsistent_external_geometry")


def released_for_cycle(before, after, baseline, *, decision_at):
    records = [baseline, before, after]
    try:
        if type(decision_at) not in (int, float) or not math.isfinite(decision_at):
            raise ValueError("invalid_observation_cutoff")
        prev = -float("inf")
        for r in records:
            clocks = [r[k] for k in ("extClock", "nearClock")]
            if not all(
                all(
                    type(c[k]) in (int, float) and math.isfinite(c[k])
                    for k in ("sourceEarliest", "sourceLatest", "availableAt")
                )
                and c["sourceEarliest"] <= c["sourceLatest"] <= c["availableAt"] <= decision_at
                for c in clocks
            ):
                raise ValueError("invalid_release_clocks")
            start = min(c["sourceEarliest"] for c in clocks)
            end = max(c["availableAt"] for c in clocks)
            if start <= prev or end - start > 3:
                raise ValueError("stale_or_overlapping_release_pair")
            prev = end
        if all(isinstance(r, dict) and r.get("schema") != SCHEMA for r in records):
            return released(before, after, baseline)
        if not all(isinstance(r, dict) and r.get("schema") == SCHEMA for r in records):
            raise ValueError("release_pair_contract_mismatch")
        if (
            len(
                {
                    (r["viewId"], r["referenceHeight"], r["near"]["observationContractSha256"])
                    for r in records
                }
            )
            != 1
        ):
            raise ValueError("release_observation_view_changed")
        for r in records:
            validate_geometry(r["external"])
        if not baseline["external"]["object"]["upright"]:
            raise ValueError("baseline_not_upright")
        _table(baseline["external"]["object"], after["external"]["object"])
        for r in records:
            validate_observation(r["near"])
        delta = after["near"]["state"][2] - before["near"]["state"][2]
        x, _, w, _ = after["near"]["object"]["bbox"]
        left = x - after["near"]["points"]["left"][0]
        right = after["near"]["points"]["right"][0] - (x + w)
        if delta < 20:
            raise ValueError("release_gap_did_not_increase")
        if min(left, right) < 3:
            raise ValueError("released_jaws_not_both_clear")
    except (KeyError, TypeError, ValueError, IndexError) as err:
        return dict(accepted=False, evidence=dict(reason=str(err)))
    return dict(
        accepted=True,
        evidence=dict(
            schema=SCHEMA,
            gapIncreasePx=delta,
            leftJawMarginPx=left,
            rightJawMarginPx=right,
            savedTableObject=baseline["external"]["object"],
            afterExternalObject=after["external"]["object"],
            sourceImages=[[r["external"]["imageSha256"], r["near"]["imageSha256"]] for r in records],
        ),
    )


def fresh_pair_after(pair, boundary, decision_at):
    """Both source intervals must begin after the proven output boundary."""
    def valid_time(value):
        return type(value) in (int, float) and math.isfinite(value) and value >= 0

    if not isinstance(pair, dict) or not all(map(valid_time, (boundary, decision_at))):
        return False
    for key in ("extClock", "nearClock"):
        clock = pair.get(key)
        if not isinstance(clock, dict):
            return False
        times = [clock.get(k) for k in ("sourceEarliest", "sourceLatest", "availableAt")]
        if not all(map(valid_time, times)):
            return False
        start, end, available = times
        if not (boundary < start <= end <= available <= decision_at and decision_at - start <= 3):
            return False
    return True


def visual_open_stop(before, current, baseline, output_proven_at, decision_at):
    if not fresh_pair_after(current, output_proven_at, decision_at):
        return None
    result = released_for_cycle(before, current, baseline, decision_at=decision_at)
    return result["evidence"] if result["accepted"] else None
