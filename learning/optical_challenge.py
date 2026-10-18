"""Synthetic optical challenge experiments, never physical calibration or admission.

No capture, display, device, clock sampling, or training adapter is provided.
Integer nanoseconds avoid rounding a failed boundary into a passing result.
"""
from __future__ import annotations

import re
import zlib

from learning.collection_readiness import POLICY

SCHEMA = "motionbrain.synthetic_optical_challenge.v1"
NONCE = re.compile(r"[0-9a-f]{16}\Z")
MAX_ROWS = 512
MAX_WIDTH = 2048


def marker_cells(nonce):
    """64 nonce bits + CRC32, Manchester coded, with eight cells at each end.

    CRC detects corruption only; it is neither authentication nor a freshness proof.
    """
    if not isinstance(nonce, str) or not NONCE.fullmatch(nonce):
        raise ValueError("nonce must be 16 lowercase hexadecimal characters")
    raw = bytes.fromhex(nonce)
    payload = raw + zlib.crc32(raw).to_bytes(4, "big")
    bits = [(byte >> bit) & 1 for byte in payload for bit in range(7, -1, -1)]
    return [1, 1, 1, 0, 0, 0, 1, 0] + [v for b in bits for v in (b, 1-b)] + [0, 1, 0, 0, 0, 1, 1, 1]


def render_marker(nonce, *, rows=16, cell_width=2):
    """Return a registered grayscale raster; never show it on a physical screen."""
    if type(rows) is not int or not 1 <= rows <= MAX_ROWS:
        raise ValueError("invalid row count")
    if type(cell_width) is not int or not 1 <= cell_width <= 8:
        raise ValueError("invalid cell width")
    row = [255*b for b in marker_cells(nonce) for _ in range(cell_width)]
    return [list(row) for _ in range(rows)]


def inspect_marker(pixels, nonce, cell_width=2):
    """Check a known, registered strip. No geometric recovery or source-row claim.

    Every pixel of each cell must be confidently dark (<=64) or light (>=192).
    Unreadable and mismatched rows remain in the denominator.
    """
    expected = marker_cells(nonce)
    if type(cell_width) is not int or not 1 <= cell_width <= 8:
        raise ValueError("invalid cell width")
    width = len(expected)*cell_width
    if not isinstance(pixels, list) or not 1 <= len(pixels) <= MAX_ROWS:
        raise ValueError("invalid raster height")
    matches, rejected = [], []
    for index, row in enumerate(pixels):
        if not isinstance(row, list) or len(row) != width or len(row) > MAX_WIDTH:
            raise ValueError("registered raster width mismatch")
        if any(type(p) is not int or not 0 <= p <= 255 for p in row):
            raise ValueError("pixels must be integer grayscale bytes")
        good = all(all(p >= 192 if bit else p <= 64
                       for p in row[i*cell_width:(i+1)*cell_width])
                   for i, bit in enumerate(expected))
        (matches if good else rejected).append(index)
    return dict(totalRows=len(pixels), matchedRows=matches, rejectedRows=rejected,
                outputPixelMatchOnly=True, sourceRowCoverageVerified=False)


def evaluate_case(case):
    """Evaluate hypothetical conditions; even a consistent case stays ineligible.

    If a post-challenge photon belongs to a full exposure span of width <= W, its start is
    no earlier than challenge-W. Complete receipt bounds its end. W must cover
    the entire frame, or each source row must independently carry the challenge.
    This span includes dark integration periods and gaps between sub-exposures;
    bounding only photon arrival times or summing integration lengths is invalid.
    Caller-supplied assumptions are deliberately NOT treated as measured facts.
    """
    report = dict(schemaVersion="motionbrain.synthetic_optical_report.v1",
                  caseOrigin="synthetic", modelConsistent=False, blockers=[],
                  modeledBoundsNs=None, opticalExposureVerified=False,
                  trainingEligible=False, controlEligible=False, executionAllowed=False,
                  missingPhysicalEvidence=["measured_optical_support_bound",
                    "single_frame_processing_provenance", "physical_nonce_photons",
                    "current_view_and_session_binding"],
                  outsideScope=["M4_20ms", "anchor_15s", "output_and_OFF_history",
                                "whole_arm_settling", "physical_persistence_receipts"])
    errors = report["blockers"]
    if not isinstance(case, dict) or case.get("schemaVersion") != SCHEMA or case.get("caseOrigin") != "synthetic":
        errors.append("synthetic_case_required")
        return report
    try:
        report["marker"] = inspect_marker(case.get("pixels"), case.get("nonce"), case.get("cellWidth"))
    except ValueError as exc:
        errors.append("marker_invalid:" + str(exc))
        return report
    assumptions = case.get("assumptions")
    if not isinstance(assumptions, dict):
        assumptions = {}
    for field in ("singleFrame", "freshAfterChallenge", "notReused"):
        if assumptions.get(field) is not True:
            errors.append("assumption_required:" + field)
    if assumptions.get("supportMeaning") != "full_exposure_start_to_end_span":
        errors.append("full_optical_support_span_required")
    scope = assumptions.get("supportScope")
    marker = report["marker"]
    if scope == "each_source_row":
        if case.get("imageDomain") != "synthetic_source_rows" or assumptions.get("sourceRowCorrespondence") is not True:
            errors.append("output_rows_do_not_prove_source_rows")
        if marker["rejectedRows"]:
            errors.append("every_source_row_must_match")
    elif scope == "whole_frame":
        if case.get("imageDomain") not in ("synthetic_source_rows", "synthetic_processed_rows"):
            errors.append("synthetic_image_domain_required")
        if not marker["matchedRows"]:
            errors.append("no_matching_optical_marker")
    else:
        errors.append("support_scope_required")

    def ns(value, label, positive=False):
        if type(value) is not int or value < (1 if positive else 0) or value > 2**63-1:
            errors.append("invalid_nanoseconds:" + label)
            return None
        return value

    width = ns(assumptions.get("supportUpperNs"), "supportUpperNs", True)
    uncertainty = ns(assumptions.get("uncertaintyNs"), "uncertaintyNs")
    usage = case.get("usage")
    if usage not in ("observation", "before", "after"):
        errors.append("usage_required")
    fields = ["challenge", "nonceGenerated", "received", "persistenceStarted", "persistenceCompleted", "consumed"]
    if usage == "before":
        fields += ["dispatchEarliest", "dispatchLatest"]
    if usage == "after":
        fields += ["offLatest"]
    clock = case.get("clockId")
    if not isinstance(clock, str) or not clock.strip():
        errors.append("clock_id_required")
    times = case.get("times")
    times = times if isinstance(times, dict) else {}
    values = {}
    for field in fields:
        stamp = times.get(field)
        stamp = stamp if isinstance(stamp, dict) else {}
        if stamp.get("clockId") != clock:
            errors.append("clock_mismatch:" + field)
        values[field] = ns(stamp.get("ns"), field)
    if errors:
        return report
    ordered = fields[:-1] if usage == "after" else fields
    if any(values[a] > values[b] for a, b in zip(ordered, ordered[1:])):
        errors.append("event_order_invalid")
    earliest = values["challenge"] - width - uncertainty
    latest = values["received"] + uncertainty
    if earliest < 0:
        errors.append("negative_modeled_bound")
    report["modeledBoundsNs"] = dict(clockId=clock, earliest=earliest, latest=latest,
                                      hypothetical=True)
    age = values["consumed"] - earliest
    report["modeledSourceAgeUpperNs"] = age
    if age > POLICY["maxSourceAgeMs"]*1_000_000:
        errors.append("source_age_over_100ms")
    if values["persistenceCompleted"]-values["persistenceStarted"] > POLICY["maxPersistenceMs"]*1_000_000:
        errors.append("persistence_over_250ms")
    if values["persistenceStarted"] < latest:
        errors.append("persistence_before_exposure_upper_bound")
    if latest > values["consumed"]:
        errors.append("consumption_before_exposure_upper_bound")
    if usage == "before":
        if values["dispatchLatest"]-earliest > POLICY["maxSourceAgeMs"]*1_000_000:
            errors.append("dispatch_age_over_100ms")
        if latest >= values["dispatchEarliest"]:
            errors.append("exposure_not_strictly_before_dispatch")
    if usage == "after" and earliest <= values["offLatest"] + POLICY["minimumSettlingMs"]*1_000_000:
        errors.append("exposure_not_after_OFF_plus_50ms")
    report["modelConsistent"] = not errors
    return report


def synthetic_case():
    """A reproducible hypothetical example; all numbers and pixels are invented."""
    nonce = "0123456789abcdef"
    times = dict(challenge=1_000_000_000, nonceGenerated=1_001_000_000,
                 received=1_040_000_000, persistenceStarted=1_042_000_000,
                 persistenceCompleted=1_045_000_000, consumed=1_050_000_000,
                 dispatchEarliest=1_055_000_000, dispatchLatest=1_060_000_000)
    return dict(schemaVersion=SCHEMA, caseOrigin="synthetic", nonce=nonce,
                cellWidth=2, pixels=render_marker(nonce), clockId="synthetic-host",
                imageDomain="synthetic_source_rows", usage="before",
                times={key:dict(clockId="synthetic-host", ns=value) for key,value in times.items()},
                assumptions=dict(singleFrame=True, freshAfterChallenge=True, notReused=True,
                    supportMeaning="full_exposure_start_to_end_span", supportScope="each_source_row",
                    sourceRowCorrespondence=True, supportUpperNs=20_000_000, uncertaintyNs=1_000_000))
