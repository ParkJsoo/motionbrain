"""Single-frame, in-memory shadow consumer. No actuator or callback interface.

Consumption is not a durable acknowledgement. Only the terminal probe report
can confirm the later per-frame evidence barrier; a missing terminal is unknown.
"""
import copy
import hashlib
import math

from learning.source_age import source_age_basis, source_age_upper_ms


def _number(value):
    return type(value) in (int, float) and math.isfinite(value)


class ConsumptionStopped(RuntimeError):
    pass


class ShadowObservationConsumer:
    MAX_SOURCE_AGE_MS = 100.0
    MAX_ANCHOR_AGE_MS = 15000.0
    # Additional shadow assumption: camera clock is no slower than 0.999 of
    # host clock. Converting an anchor duration upper bound needs the LOWER
    # rate, unlike the fresh capture lower bound. Neither is calibrated.
    ANCHOR_CLOCK_MIN_RATE = 0.999
    ANCHOR_TIMESTAMP_ERROR_MS = 0.002

    def __init__(self, clock, host_clock_id, *, max_persistence_ms=250):
        if not _number(max_persistence_ms) or not 0 < max_persistence_ms <= 30000:
            raise ValueError("max_persistence_ms must be in (0, 30000]")
        self.clock, self.host_clock_id = clock, host_clock_id
        self.max_persistence_ms = max_persistence_ms
        self.records, self.seen = [], set()
        self.pending = None
        self.halted = False

    def check_ready(self):
        if self.halted or self.pending is not None:
            raise ConsumptionStopped("consumption_not_ready")

    def consume(self, measurement, source, receipt, body):
        self.check_ready()
        frame_id = source.get("sourceFrameId")
        if frame_id in self.seen:
            self.halted = True
            raise ConsumptionStopped("consumption_frame_repeated")
        self.seen.add(frame_id)
        row = {"schemaVersion": "motionbrain.shadow_consumption.v1",
               "sourceFrameId": frame_id, "imageSha256": source.get("imageSha256"),
               "hostClockId": self.host_clock_id, "consumed": False, "reason": None,
               "storageStatus": "pending", "controlEligible": False, "trainingEligible": False}
        self.pending = row
        self.records.append(row)
        fatal = False
        try:
            if (measurement.get("sourceFrameId") != frame_id
                    or measurement.get("imageSha256") != source.get("imageSha256")
                    or hashlib.sha256(body).hexdigest() != source.get("imageSha256")
                    or measurement.get("hostClockId") != self.host_clock_id):
                raise ValueError("consumption_measurement_mismatch")
            basis = source_age_basis(receipt, source,
                request_started=receipt["requestStartedMonotonic"], received=receipt["receivedMonotonic"],
                host_clock_id=self.host_clock_id)
            completed = measurement.get("measurementCompletedMonotonic")
            anchor_age = measurement.get("anchorAgeMs")
            decision = None
            if measurement.get("accepted") is True:
                box = measurement.get("targetBox")
                if (not isinstance(box, dict) or any(not _number(box.get(k)) for k in ("x", "y", "width", "height"))
                        or box["width"] <= 0 or box["height"] <= 0):
                    raise ValueError("invalid_shadow_target")
                decision = {"targetBox": copy.deepcopy(box)}
            # Copy evidence before sampling consumption time; there is no user
            # callback, queued task or I/O between this gate and the shadow use.
            row.update(basis)
            now = self.clock.monotonic()
            row["consumedMonotonic"] = now if _number(now) else None
            if (not _number(completed) or not _number(now)
                    or not receipt["receivedMonotonic"] <= completed <= now):
                raise ValueError("invalid_consumption_clock")
            reported_age = source["reportedSourceAgeMs"]
            legacy_age = (reported_age / self.ANCHOR_CLOCK_MIN_RATE + self.ANCHOR_TIMESTAMP_ERROR_MS
                          if "sourceCaptureLowerBoundMonotonic" not in basis else None)
            age = source_age_upper_ms(basis, legacy_age if legacy_age is not None else reported_age, now)
            row.update(reportedSourceAgeMs=reported_age, legacyReportedAgeUpperMs=legacy_age)
            row["sourceAgeAtConsumptionMs"] = age
            if not _number(age) or age < 0:
                raise ValueError("invalid_consumption_clock")
            anchor_upper = (anchor_age / self.ANCHOR_CLOCK_MIN_RATE + age + self.ANCHOR_TIMESTAMP_ERROR_MS
                            if _number(anchor_age) and anchor_age >= 0 else None)
            row.update(anchorAgeAtConsumptionUpperMs=anchor_upper,
                       anchorClockMinRate=self.ANCHOR_CLOCK_MIN_RATE,
                       anchorTimestampErrorMs=self.ANCHOR_TIMESTAMP_ERROR_MS,
                       anchorClockErrorCalibrated=False)
            if measurement.get("accepted") is not True:
                row["reason"] = "measurement_rejected"
            elif age > self.MAX_SOURCE_AGE_MS:
                row["reason"] = "source_age_at_consumption_exceeds_policy"
            elif anchor_upper is None or anchor_upper > self.MAX_ANCHOR_AGE_MS:
                row["reason"] = "anchor_expired_at_consumption"
            else:
                row["consumed"] = True  # Shadow use only, never an actuator command.
                row["shadowDecision"] = decision
        except (ValueError, KeyError, TypeError) as exc:
            row["reason"] = str(exc)
            fatal = True
        if "consumedMonotonic" not in row:
            now = self.clock.monotonic()
            row["consumedMonotonic"] = now if _number(now) else None
        row["haltAfterPersistence"] = fatal
        return copy.deepcopy(row)

    def finish_persistence(self, frame_id, host_clock_id):
        """Call after all current-frame core evidence writes return successfully.

        A late write is joined/preserved, never cancelled or retried. This is
        a completion-time limit, not a cancellable fsync wall-clock deadline.
        """
        if self.pending is None:
            raise ConsumptionStopped("no_pending_consumption")
        row = self.pending
        if frame_id != row["sourceFrameId"] or host_clock_id != self.host_clock_id:
            self.stop("persistence_identity_mismatch")
            raise ConsumptionStopped("persistence_identity_mismatch")
        now, started = self.clock.monotonic(), row.get("consumedMonotonic")
        if not _number(now) or not _number(started) or now < started:
            row.update(storageStatus="unconfirmed", storageError="invalid_persistence_clock")
            self.halted = True
        else:
            elapsed = (now-started)*1000
            row.update(frameEvidenceCompletedMonotonic=now, persistenceElapsedMs=elapsed,
                       storageStatus="completed" if elapsed <= self.max_persistence_ms else "late")
            self.halted = elapsed > self.max_persistence_ms or row["haltAfterPersistence"]
        self.pending = None
        if self.halted:
            raise ConsumptionStopped(row.get("storageError", "persistence_deadline_or_consumption_evidence"))

    def stop(self, reason):
        self.halted = True
        if self.pending is not None:
            # Some writes may already exist, but the full barrier is unproven.
            self.pending.update(storageStatus="unconfirmed", storageError=reason)
            self.pending = None

    def snapshot(self):
        return copy.deepcopy(self.records)
