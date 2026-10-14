"""Conservative dataset provenance checks; declarations are not physical proof.

Unknown provenance is never promoted to physical data. These checks detect
explicit synthetic markers and require the future physical collector's positive
contract. They do not authenticate producers or prevent coordinated relabeling.
"""
from __future__ import annotations

import re


# Exact bytes of learning.dry_run.JPEG_BYTES. This catches accidental reuse of
# this repository's fixture only; an unknown image hash is not physical evidence.
KNOWN_SYNTHETIC_IMAGE_HASHES = frozenset({
    "c944873d37463dd4fedec83fb0f81b630359cb08814c254427a03589ceea56be",
})

_SYNTHETIC_PREFIX = re.compile(
    r"^(?:synthetic(?:$|[^a-z]|adapter)|dry[_ -]?run(?:$|[^a-z])|simulation(?:$|[^a-z]))",
    re.IGNORECASE,
)
_SEMANTIC_KEYS = {
    "mode", "evidence", "provenance", "adapter", "robotid", "clockid",
    "hostclockid", "observationid", "label", "method", "basis",
}


def _semantic_key(key):
    normalized = key.lower().replace("_", "").replace("-", "")
    return (normalized in _SEMANTIC_KEYS
            or normalized.endswith(("ref", "refs", "version", "provenance", "basis")))


def synthetic_reasons(*documents):
    """Find explicit synthetic provenance anywhere in JSON documents.

    Inspect semantic fields, including nested sensor/output records and evidence
    references. Ordinary free-text notes are not interpreted as provenance.
    Returned paths are stable diagnostics; inputs are never changed.
    """
    reasons = []

    def visit(value, path, key=""):
        if isinstance(value, dict):
            for child_key, child in value.items():
                child_path = f"{path}.{child_key}"
                if isinstance(child_key, str) and child_key.lower() == "synthetic" and child is True:
                    reasons.append(f"synthetic_marker:{child_path}")
                visit(child, child_path, child_key if isinstance(child_key, str) else "")
        elif isinstance(value, list):
            for index, child in enumerate(value):
                visit(child, f"{path}[{index}]", key)
        elif isinstance(value, str) and _semantic_key(key) and _SYNTHETIC_PREFIX.match(value.strip()):
            reasons.append(f"synthetic_marker:{path}")

    for index, document in enumerate(documents):
        visit(document, f"document[{index}]")
    return reasons


def physical_provenance_reasons(session, run, summary, transition, config, calibration):
    """Return every reason a record lacks the explicit physical-data contract.

    A passing result confirms consistent recorded declarations, not hardware
    verification. Image integrity, collection completion, clock evidence, and
    transition validation remain independent mandatory dataset gates.
    """
    documents = (session, run, summary, transition, config, calibration)
    reasons = synthetic_reasons(*documents)

    def require(record, key, expected, path):
        value = record.get(key) if isinstance(record, dict) else None
        matches = value is True if expected is True else value == expected
        if not matches:
            reasons.append(f"physical_provenance_required:{path}.{key}")

    for name, record in (("session", session), ("run", run), ("summary", summary)):
        require(record, "mode", "physical", name)
        require(record, "evidence", "physical", name)
    collection = transition.get("collection") if isinstance(transition, dict) else None
    require(collection, "mode", "physical", "transition.collection")
    require(collection, "evidence", "physical", "transition.collection")
    require(config, "mode", "physical", "config")
    require(calibration, "physicalCalibration", True, "calibration")
    return reasons
