"""Reproducible dataset inventories with independent integrity/provenance gates.

No training, image rewriting, device access, or promotion of synthetic evidence.
Physical declarations are checked for consistency, not authenticated hardware truth.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

from learning.artifacts import StorageError, read_events, read_json_artifact, verify_artifacts
from learning.provenance import KNOWN_SYNTHETIC_IMAGE_HASHES, physical_provenance_reasons, synthetic_reasons
from learning.schema import validate_transition
from learning.splits import LABELS, assign_splits, validate_splits

MANIFEST_VERSION = "motionbrain.dataset_manifest.v1"
SPLIT_POLICY = {"algorithm": "sha256-connected-groups-v1", "groupBy": "session",
                "episodeKey": ["sessionId", "episodeId"], "ratios": [80, 10, 10],
                "sharedEvidence": ["imageSha256", "sourceFrameId", "bootId+commandId", "splitGroupId"]}


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=True,
                                    allow_nan=False, separators=(",", ":")).encode()).hexdigest()


def _obj(value):
    return value if isinstance(value, dict) else {}


def _text(value):
    return value if isinstance(value, str) and value.strip() else None


def _inspect_run(path):
    root = Path(path).absolute()
    issues, rows = [], []
    if root.is_symlink():
        return {"root": str(root), "sessionId": None, "journalSha256": None,
                "issues": ["run_root_symlink"]}, rows
    events, read_issues = read_events(root)
    issues.extend(read_issues)

    def load(reference, label, local_issues=issues):
        try:
            value = read_json_artifact(root, reference)
            if not isinstance(value, dict):
                raise StorageError("record must be an object")
            return value
        except StorageError as exc:
            local_issues.append(f"{label}:{exc}")
            return {}

    starts = [event for event in events if event.get("kind") == "session_started"]
    finishes = [event for event in events if event.get("kind") == "session_finished"]
    if (len(starts) != 1 or len(finishes) != 1 or not events
            or events[0].get("kind") != "session_started" or events[-1].get("kind") != "session_finished"):
        issues.append("session_boundaries_invalid")
    start = starts[0] if starts else {}
    finish = finishes[0] if finishes else {}
    session_ref = _obj(start.get("payload")).get("session")
    run_ref = _obj(finish.get("payload")).get("summary")
    session = load(session_ref, "session") if session_ref else {}
    run = load(run_ref, "run") if run_ref else {}
    sid = _text(session.get("sessionId"))
    if not sid or any(value != sid for value in (start.get("transitionId"), finish.get("transitionId"), run.get("sessionId"))):
        issues.append("session_identity_mismatch")
    if session.get("schemaVersion") != "motionbrain.capture_session.v1":
        issues.append("session_schema_unknown")
    if run.get("status") not in ("completed", "stopped"):
        issues.append("run_status_unknown")

    def references(values, label):
        result = {}
        if not isinstance(values, list):
            issues.append(f"{label}:list_required")
            return result
        for reference in values:
            name = _text(_obj(reference).get("artifactRef"))
            if not name or name in result:
                issues.append(f"{label}:duplicate_or_invalid_reference")
                continue
            result[name] = reference
        issues.extend(f"{label}:{issue}" for issue in verify_artifacts(root, values))
        return result

    snapshots = references(session.get("snapshots"), "snapshots")
    config_ref, calibration_ref = (snapshots.get(f"snapshots/{name}.json") for name in ("config", "calibration"))
    config, calibration = load(config_ref, "config"), load(calibration_ref, "calibration")
    if "qualityPolicy" in config or config.get("mode") == "physical":
        if not isinstance(config.get("qualityPolicy"), dict) or config["qualityPolicy"] != session.get("qualityPolicy"):
            issues.append("snapshot_quality_policy_mismatch_or_missing")
    for key, reference in (("configHash", config_ref), ("calibrationHash", calibration_ref)):
        if _obj(session.get("metadata")).get(key) != _obj(reference).get("sha256") or reference is None:
            issues.append(f"snapshot_metadata_mismatch:{key}")
    run_summaries = references(run.get("attempts"), "run_attempts")
    journals = {}
    for event in events:
        if event["kind"] not in ("session_started", "session_finished"):
            journals.setdefault(event["transitionId"], []).append(event)
    if run.get("attemptIds") != sorted(journals):
        issues.append("run_attempt_index_mismatch")
    observed_summaries = {}
    for tid, journal in sorted(journals.items()):
        errors, quality_issues = [], []
        start_events = [event for event in journal if event["kind"] == "attempt_started"]
        end_events = [event for event in journal if event["kind"] == "attempt_finished"]
        if len(start_events) != 1 or len(end_events) != 1 or journal[0]["kind"] != "attempt_started" or journal[-1]["kind"] != "attempt_finished":
            errors.append("attempt_boundaries_invalid")
        started = _obj(start_events[0].get("payload")) if start_events else {}
        end_payload = _obj(end_events[0].get("payload")) if end_events else {}
        summary_ref = end_payload.get("summary")
        summary = load(summary_ref, "summary", errors) if summary_ref else {}
        if summary_ref:
            summary_name = _text(_obj(summary_ref).get("artifactRef"))
            if summary_name:
                observed_summaries[summary_name] = summary_ref
            else:
                errors.append("summary_reference_invalid")
        transition = _obj(summary.get("transition"))
        episode = _text(transition.get("episodeId")) or _text(started.get("episodeId"))
        if (summary.get("schemaVersion") != "motionbrain.capture_attempt.v1"
                or summary.get("transitionId") != tid or transition.get("transitionId") != tid
                or transition.get("sessionId") != sid or not episode or episode != started.get("episodeId")):
            errors.append("attempt_identity_or_schema_mismatch")
        if end_payload.get("status") != summary.get("status"):
            errors.append("attempt_status_mismatch")
        if transition.get("metadata") != session.get("metadata") or transition.get("qualityPolicy") != session.get("qualityPolicy"):
            errors.append("session_metadata_or_policy_mismatch")
        artifact_values = summary.get("artifacts")
        if not isinstance(artifact_values, list):
            artifact_values = []
            errors.append("attempt_artifacts_missing")
        artifact_map = {}
        for reference in artifact_values:
            name = _text(_obj(reference).get("artifactRef"))
            if not name or name in artifact_map:
                errors.append("attempt_artifact_duplicate_or_invalid")
            else:
                artifact_map[name] = reference
        errors.extend(verify_artifacts(root, artifact_values))
        for reference in (session_ref, config_ref, calibration_ref):
            if not reference or artifact_map.get(_text(_obj(reference).get("artifactRef"))) != reference:
                errors.append("snapshot_or_session_reference_missing")

        raw_observations, image_refs, raw_action, intents = {}, {}, None, []
        raw_documents, stages = [], []
        for event in journal:
            payload = _obj(event.get("payload"))
            kind, phase = event["kind"], payload.get("phase")
            if kind in ("observation_received", "image_saved"):
                if phase not in ("before", "after"):
                    errors.append("observation_phase_invalid")
                    continue
                stages.append((0 if phase == "before" else 4) + (kind == "image_saved"))
                destination = raw_observations if kind == "observation_received" else image_refs
                if phase in destination:
                    errors.append("duplicate_observation_or_image_event")
                reference = payload.get("raw" if kind == "observation_received" else "image")
                if artifact_map.get(_text(_obj(reference).get("artifactRef"))) != reference or reference is None:
                    errors.append("event_artifact_not_in_summary")
                if kind == "observation_received":
                    destination[phase] = load(reference, f"raw_{phase}", errors)
                    raw_documents.append(destination[phase])
                else:
                    destination[phase] = reference
                    errors.extend(verify_artifacts(root, [reference]))
            elif kind == "action_intent":
                stages.append(2)
                intents.append(payload.get("request"))
            elif kind == "action_received":
                stages.append(3)
                if raw_action is not None:
                    errors.append("duplicate_action_result")
                reference = payload.get("raw")
                if artifact_map.get(_text(_obj(reference).get("artifactRef"))) != reference or reference is None:
                    errors.append("action_artifact_not_in_summary")
                raw_action = load(reference, "raw_action", errors)
                raw_documents.append(raw_action)
            elif kind not in ("attempt_started", "attempt_finished"):
                errors.append("unknown_attempt_event")
        if stages != sorted(set(stages)):
            errors.append("attempt_event_order_invalid")
        if 2 in stages and not all(stage in stages for stage in (0, 1)):
            errors.append("action_intent_without_before_image")
        if any(stage in stages for stage in (4, 5)) and 3 not in stages:
            errors.append("after_without_action_result")
        images = []
        for phase in ("before", "after"):
            observation = transition.get(phase)
            raw, image = raw_observations.get(phase), image_refs.get(phase)
            if raw is not None:
                expected = dict(raw)
                if image is not None:
                    expected["image"] = {"artifactRef": _obj(image).get("artifactRef"), "sha256": _obj(image).get("sha256")}
                if expected != observation:
                    errors.append(f"{phase}:raw_summary_mismatch")
            elif observation is not None:
                errors.append(f"{phase}:raw_observation_missing")
            if image is not None:
                images.append({**_obj(image), "phase": phase, "sourceFrameId": _obj(raw).get("sourceFrameId")})
                if (_obj(image).get("sha256") != _obj(raw).get("imageSha256")
                        or _obj(image).get("expectedSha256") != _obj(raw).get("imageSha256")
                        or _obj(image).get("hashMatches") is not True):
                    quality_issues.append(f"{phase}:image_hash_disagreement")
        action = _obj(transition.get("action"))
        if raw_action is not None and raw_action != action:
            errors.append("action:raw_summary_mismatch")
        if action.get("request") != started.get("request"):
            errors.append("action:started_request_mismatch")
        if (raw_action is not None and (len(intents) != 1 or intents[0] != started.get("request"))
                or len(intents) > 1):
            errors.append("action:intent_identity_mismatch")
        if raw_action is None and "record" in action:
            errors.append("action:raw_result_missing")
        provenance = physical_provenance_reasons(session, run, summary, transition, config, calibration)
        markers = synthetic_reasons(session, run, summary, transition, config, calibration, *raw_documents, journal, start, finish)
        validation = validate_transition(transition)
        excluded = list(provenance) + list(markers) + quality_issues + list(validation["exclusionReasons"])
        if not validation["valid"] or not validation["trainingEligible"]:
            excluded.append("transition_not_training_eligible")
        if summary.get("status") != "completed" or summary.get("executionState") != "known_terminal":
            excluded.append("collection_not_complete_known_terminal")
        if summary.get("errors") != [] or summary.get("missing") != []:
            excluded.append("collection_errors_or_missing")
        if len(images) != 2 or any(_obj(image).get("hashMatches") is not True for image in images):
            excluded.append("image_evidence_incomplete_or_mismatched")
        if errors:
            excluded.append("artifact_or_record_integrity_invalid")
            issues.extend(f"attempt:{tid}:{error}" for error in errors)
        row = {"sampleId": _digest([sid, tid]), "sessionId": sid, "episodeId": episode, "transitionId": tid,
               "sourceRoot": str(root), "summary": summary_ref, "images": images,
               "actionIdentity": {key: _obj(action.get("request")).get(key) for key in ("bootId", "commandId")},
               "status": summary.get("status", "incomplete"), "taskSuccess": _obj(transition.get("outcome")).get("taskSuccess"),
               "provenance": "synthetic" if markers else "physical" if not provenance else "unknown",
               "validation": validation, "integrityIssues": sorted(set(errors)),
               "eligible": not excluded, "exclusionReasons": sorted(set(excluded))}
        split_group = _obj(session.get("metadata")).get("splitGroupId")
        if split_group is not None:
            row["splitGroupId"] = split_group
        rows.append(row)
    if observed_summaries != run_summaries:
        issues.append("run_summary_index_mismatch")
    source = {"root": str(root), "sessionId": sid, "journalSha256": _digest(events),
              "session": session_ref, "run": run_ref, "issues": sorted(set(issues))}
    if issues:
        for row in rows:
            row["eligible"] = False
            row["exclusionReasons"] = sorted(set(row["exclusionReasons"] + ["source_run_integrity_invalid"]))
    return source, rows


def build_manifest(run_dirs, *, seed="motionbrain-v1"):
    """Keep all attempts in an inventory; partition only verified physical rows."""
    if not isinstance(run_dirs, (list, tuple)) or not run_dirs:
        raise ValueError("at least one explicit run directory is required")
    if not isinstance(seed, str) or not seed.strip():
        raise ValueError("seed must be a nonempty string")
    sources, rows, input_issues = [], [], []
    roots = [str(Path(path).absolute()) for path in run_dirs]
    if len(set(roots)) != len(roots):
        input_issues.append("duplicate_source_root")
    for root in sorted(roots):
        source, entries = _inspect_run(root)
        sources.append(source)
        rows.extend(entries)
        input_issues.extend(f"{root}:{issue}" for issue in source["issues"])
    sessions, samples, actions = {}, {}, {}
    for source in sources:
        if source.get("sessionId"):
            sessions.setdefault(source["sessionId"], []).append(source["root"])
    for sid, locations in sessions.items():
        if len(locations) > 1:
            input_issues.append(f"duplicate_session_source:{sid}")
    synthetic_hashes = set(KNOWN_SYNTHETIC_IMAGE_HASHES)
    for row in rows:
        samples.setdefault(row["sampleId"], []).append(row)
        identity = row["actionIdentity"]
        if all(_text(identity.get(key)) for key in ("bootId", "commandId")):
            actions.setdefault((identity["bootId"], identity["commandId"]), []).append(row)
        if row["provenance"] == "synthetic":
            synthetic_hashes.update(image["sha256"] for image in row["images"] if _text(image.get("sha256")))
    for table, reason in ((samples, "duplicate_sample_identity"), (actions, "duplicate_action_identity")):
        for group in table.values():
            if len(group) > 1:
                input_issues.append(reason)
                for row in group:
                    row["eligible"] = False
                    row["exclusionReasons"].append(reason)
    for row in rows:
        if any(_text(image.get("sha256")) in synthetic_hashes for image in row["images"]):
            row["eligible"] = False
            row["exclusionReasons"].append("image_seen_in_synthetic_source")
        row["exclusionReasons"] = sorted(set(row["exclusionReasons"]))
    rows.sort(key=lambda row: (row["sampleId"], row["sourceRoot"]))
    eligible = [row for row in rows if row["eligible"]]
    try:
        assignments = assign_splits(eligible, seed)
        input_issues.extend(validate_splits(eligible, assignments))
    except ValueError as exc:
        assignments = {}
        input_issues.append(f"split_input_invalid:{exc}")
    counts = {label: sum(value == label for value in assignments.values()) for label in LABELS}
    return {"schemaVersion": MANIFEST_VERSION, "purpose": "physical_training_inventory",
            "seed": seed, "splitPolicy": copy.deepcopy(SPLIT_POLICY), "sources": sources, "rows": rows,
            "assignments": assignments, "splitCounts": counts, "eligibleCount": len(eligible),
            "excludedCount": len(rows) - len(eligible), "inputIssues": sorted(set(input_issues)),
            "readyForTraining": not input_issues and all(counts.values())}


def validate_manifest(manifest):
    """Reload source bytes and rederive eligibility; cached flags are never trusted."""
    if not isinstance(manifest, dict):
        return {"valid": False, "readyForTraining": False, "errors": ["manifest_object_required"]}
    errors = []
    try:
        sources = manifest.get("sources")
        if not isinstance(sources, list) or not sources or any(not isinstance(source, dict) or not isinstance(source.get("root"), str) for source in sources):
            raise ValueError("manifest_source_roots_required")
        rebuilt = build_manifest([source["root"] for source in sources], seed=manifest.get("seed"))
    except (ValueError, TypeError, OSError, StorageError) as exc:
        return {"valid": False, "readyForTraining": False, "errors": [f"source_validation_failed:{exc}"]}
    errors.extend(rebuilt["inputIssues"])
    for field in ("schemaVersion", "purpose", "splitPolicy", "sources", "rows", "splitCounts", "eligibleCount", "excludedCount", "inputIssues", "readyForTraining"):
        if manifest.get(field) != rebuilt[field]:
            errors.append(f"manifest_source_or_policy_mismatch:{field}")
    eligible = [row for row in rebuilt["rows"] if row["eligible"]]
    errors.extend(validate_splits(eligible, manifest.get("assignments")))
    if manifest.get("assignments") != rebuilt["assignments"]:
        errors.append("assignments_not_reproducible")
    return {"valid": not errors, "readyForTraining": not errors and rebuilt["readyForTraining"],
            "errors": sorted(set(errors)), "splitCounts": rebuilt["splitCounts"],
            "eligibleCount": rebuilt["eligibleCount"], "excludedCount": rebuilt["excludedCount"]}
