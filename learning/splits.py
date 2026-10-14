"""Deterministic leakage groups for already eligible dataset rows.

This is a pure partitioning helper, not an eligibility or physical-provenance
gate. Related sessions, images and commands stay together; small datasets may
therefore have empty partitions. Paths never participate in identity.
"""
from __future__ import annotations

import hashlib
import json
import re

LABELS = ("train", "validation", "test")
_SHA256 = re.compile(r"[0-9a-f]{64}\Z")


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name}: nonempty string required")
    return value


def _rows(rows):
    if not isinstance(rows, (list, tuple)):
        raise ValueError("rows: list or tuple required")
    normalized = []
    for index, row in enumerate(rows):
        path = f"rows[{index}]"
        if not isinstance(row, dict):
            raise ValueError(f"{path}: object required")
        entry = {key: _text(row.get(key), f"{path}.{key}")
                 for key in ("sampleId", "sessionId", "episodeId", "transitionId")}
        if "splitGroupId" in row:
            entry["splitGroupId"] = _text(row["splitGroupId"], f"{path}.splitGroupId")
        action = row.get("actionIdentity")
        if not isinstance(action, dict):
            raise ValueError(f"{path}.actionIdentity: object required")
        entry["actionIdentity"] = tuple(_text(action.get(key), f"{path}.actionIdentity.{key}")
                                         for key in ("bootId", "commandId"))
        images = row.get("images")
        if not isinstance(images, (list, tuple)) or not images:
            raise ValueError(f"{path}.images: nonempty list or tuple required")
        entry["images"] = []
        for image in images:
            if not isinstance(image, dict):
                raise ValueError(f"{path}.images: image object required")
            frame = _text(image.get("sourceFrameId"), f"{path}.images.sourceFrameId")
            digest = image.get("sha256")
            if not isinstance(digest, str) or not _SHA256.fullmatch(digest):
                raise ValueError(f"{path}.images.sha256: lowercase SHA-256 required")
            entry["images"].append((frame, digest))
        normalized.append(entry)
    return normalized


def _group_keys(row):
    # Namespaced tuples avoid ambiguous concatenations and cross-kind collisions.
    yield ("session", row["sessionId"])
    yield ("episode", row["sessionId"], row["episodeId"])
    if "splitGroupId" in row:
        yield ("split_group", row["splitGroupId"])
    yield ("action", *row["actionIdentity"])
    for frame, digest in row["images"]:
        yield ("source_frame", frame)
        yield ("image_sha256", digest)


def assign_splits(rows, seed="motionbrain-v1"):
    """Assign connected components by SHA-256 to 80/10/10 hash intervals.

    The proportions are targets, not guarantees. Never split a component merely
    to fill an empty validation/test set. The same row set and seed produce the
    same assignments regardless of row order or artifact paths.
    """
    seed = _text(seed, "seed")
    rows = _rows(rows)
    sample_ids = [row["sampleId"] for row in rows]
    if len(set(sample_ids)) != len(sample_ids):
        raise ValueError("duplicate_sample_id")
    parents = list(range(len(rows)))

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    seen = {}
    for index, row in enumerate(rows):
        for key in _group_keys(row):
            if key in seen:
                parents[find(index)] = find(seen[key])
            else:
                seen[key] = index
    groups = {}
    for index, sample_id in enumerate(sample_ids):
        groups.setdefault(find(index), []).append(sample_id)
    result = {}
    for component in groups.values():
        component.sort()
        payload = json.dumps([seed, component], ensure_ascii=True, separators=(",", ":"))
        bucket = int.from_bytes(hashlib.sha256(payload.encode("utf-8")).digest(), "big") % 100
        label = "train" if bucket < 80 else "validation" if bucket < 90 else "test"
        result.update((sample_id, label) for sample_id in component)
    return dict(sorted(result.items()))


def validate_splits(rows, assignments):
    """Independently audit identities and leakage; malformed input is an error.

    Valid grouping cannot legitimize duplicate samples, transitions or executed
    commands. Images may recur within a partition, including before/after reuse.
    """
    try:
        rows = _rows(rows)
    except ValueError as exc:
        return [str(exc)]
    if not isinstance(assignments, dict):
        return ["assignments: object required"]
    issues = []
    identities = {"sample": set(), "session_transition": set(), "action": set()}
    expected = {row["sampleId"] for row in rows}
    for key in assignments:
        if not isinstance(key, str) or key not in expected:
            issues.append(f"unexpected_assignment:{key!r}")
    for key in sorted(expected):
        if key not in assignments:
            issues.append(f"missing_assignment:{key}")
        elif not isinstance(assignments[key], str) or assignments[key] not in LABELS:
            issues.append(f"invalid_split:{key}")
    groups = {}
    for row in rows:
        for kind, identity in (("sample", row["sampleId"]),
                               ("session_transition", (row["sessionId"], row["transitionId"])),
                               ("action", row["actionIdentity"])):
            if identity in identities[kind]:
                issues.append(f"duplicate_{kind}_identity:{identity!r}")
            identities[kind].add(identity)
        label = assignments.get(row["sampleId"])
        if not isinstance(label, str) or label not in LABELS:
            continue
        for key in _group_keys(row):
            groups.setdefault(key, set()).add(label)
    for key, labels in sorted(groups.items()):
        if len(labels) > 1:
            issues.append(f"{key[0]}_leakage:{key[1:]!r}")
    return sorted(set(issues))
