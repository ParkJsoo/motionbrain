"""Bind one locally witnessed elbow-down run to the next idle controller epoch."""

import hashlib
import hmac
import json
import re

PHASE = "elbow_down_once"
PROFILE = "m3-down-50-350ms-v1"
SCHEMA = "motionbrain.elbow_down_handoff.v1"


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _uint(value):
    return type(value) is int and 0 <= value < 2**32


def _signature(fields, key):
    _require(isinstance(key, bytes) and len(key) >= 32, "handoff_signing_key_required")
    raw = json.dumps(fields, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    return hmac.digest(key, SCHEMA.encode() + b"\0" + raw, hashlib.sha256).hex()


def _completed(owner, boot, trial, generation):
    _require(isinstance(boot, str) and re.fullmatch("[0-9a-f]{32}", boot)
             and isinstance(trial, str) and re.fullmatch("[0-9a-f]{32}", trial)
             and owner.get("bootId") == boot and owner.get("trialId") == trial
             and owner.get("phase") == PHASE and owner.get("elbowDownOnceProfile") == PROFILE
             and type(owner.get("seq")) is int and owner["seq"] == 1
             and _uint(generation) and _uint(owner.get("controlGeneration"))
             and owner["controlGeneration"] == generation and owner.get("reason") == "completed"
             and owner.get("active") is False and owner.get("failed") is False,
             "elbow_owner_not_completed")
    first, zero = owner.get("elbowFirstWriteOffsetUs"), owner.get("elbowZeroWriteOffsetUs")
    _require(_uint(first) and _uint(zero) and first < zero and 349000 <= zero < 365000,
             "elbow_write_timing_invalid")
    return first, zero


def _off(status, boot, cursor):
    proof = status.get("outputProof", {})
    motors = status.get("motors", {})
    _require(status.get("schemaVersion") == "collection.v1" and status.get("state") == "IDLE"
             and proof.get("schemaVersion") == "driver_cursor.v1" and proof.get("bootId") == boot
             and proof.get("stable") is True and _uint(cursor)
             and all(_uint(proof.get(k)) and proof[k] == cursor
                     for k in ("sequenceBefore", "sequenceAfter"))
             and set(motors) == {f"M{i}" for i in range(1, 6)}
             and all(m.get("enabled") is False and type(m.get("speed")) is int and m["speed"] == 0
                     for m in motors.values()), "elbow_idle_off_cursor_changed")


def make_elbow_receipt(key, *, boot, trial, generation, start_cursor, owner, status, entries):
    """Sign a complete post-disarm journal already read with the boot-bound paging API.

    generation is the expected terminal generation including the caller's own STOP.
    start_cursor is the witnessed cursor immediately before the one-shot START.
    """
    first, zero = _completed(owner, boot, trial, generation)
    _require(_uint(start_cursor) and len(entries) >= 6, "elbow_journal_incomplete")
    end_cursor = start_cursor + len(entries)
    _off(status, boot, end_cursor)
    for i, e in enumerate(entries, 1):
        _require(_uint(e.get("sequence")) and e["sequence"] == start_cursor + i
                 and type(e.get("motor")) is int and type(e.get("pwm")) is int
                 and _uint(e.get("atMs")), "elbow_journal_gap")
    motion = entries[:6]
    _require([e["pwm"] for e in motion] == [10, 20, 30, 40, 50, 0]
             and all(e["motor"] == 3 and e.get("reason") == "driver_write" for e in motion),
             "elbow_normal_zero_missing")
    elapsed = [(e["atMs"] - motion[0]["atMs"]) % 2**32 for e in motion]
    _require(all(a < b for a, b in zip(elapsed, elapsed[1:]))
             and abs(1000 * elapsed[-1] - (zero-first)) <= 2000, "elbow_clock_mismatch")
    _require(all(e.get("reason") == "forced_off" and e["pwm"] == 0
                 and e["motor"] == (i % 5) + 1 for i, e in enumerate(entries[6:]))
             and len(entries[6:]) % 5 == 0, "outputs_after_elbow_zero")
    after_zero = [(e["atMs"] - motion[-1]["atMs"]) % 2**32 for e in entries[6:]]
    _require(all(t < 2**31 for t in after_zero) and after_zero == sorted(after_zero),
             "out_of_order_disarm_writes")
    fields = dict(schema=SCHEMA, phase=PHASE, profile=PROFILE, bootId=boot, trialId=trial,
                  controlGeneration=generation, seq=1, firstUs=first, zeroUs=zero,
                  zeroCursor=motion[-1]["sequence"], idleCursor=end_cursor)
    return dict(fields=fields, hmacSha256=_signature(fields, key))


def verify_elbow_handoff(receipt, key, *, boot, owner, status, entries):
    """Accept only the exact signed idle cursor before the next command."""
    fields = receipt["fields"]
    tag = receipt.get("hmacSha256")
    _require(isinstance(tag, str) and re.fullmatch("[0-9a-f]{64}", tag)
             and hmac.compare_digest(tag, _signature(fields, key)), "elbow_handoff_signature")
    _require(fields["schema"] == SCHEMA and fields["phase"] == PHASE and fields["profile"] == PROFILE
             and boot == fields["bootId"] and fields["seq"] == 1
             and _uint(fields["zeroCursor"]) and _uint(fields["idleCursor"])
             and fields["zeroCursor"] <= fields["idleCursor"] and entries == [],
             "elbow_handoff_history_changed")
    times = _completed(owner, boot, fields["trialId"], fields["controlGeneration"])
    _require(times == (fields["firstUs"], fields["zeroUs"]), "elbow_handoff_write_times_changed")
    _off(status, boot, fields["idleCursor"])
    return fields["idleCursor"]
