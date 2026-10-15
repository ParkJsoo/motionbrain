"""Fixed, finite shadow request policy. No reacquisition or actuator interface."""
import math

POLICY = {"maxCaptures": 5, "operationBudgetMs": 3000,
          "evaluationAttempts": 20, "minimumSuccesses": 16,
          "maxSourceAgeMs": 100, "maxAnchorAgeMs": 15000, "maxPersistenceMs": 250}


class AttemptStopped(RuntimeError):
    pass


class ObservationAttempt:
    def __init__(self, request_id, clock):
        if not isinstance(request_id, str) or not request_id.strip():
            raise ValueError("request_id required")
        self.clock, self.request_id = clock, request_id
        self.started = self.last = self._now_initial()
        self.ready = self.deadline = self.ended = self.consumed = None
        self.captures, self.reason = 0, None
        self.status = "acquiring"

    def _now_initial(self):
        now = self.clock.monotonic()
        if type(now) not in (int, float) or not math.isfinite(now):
            raise AttemptStopped("invalid_attempt_clock")
        return now

    def now(self):
        now = self._now_initial()
        if now < self.last:
            raise AttemptStopped("attempt_clock_regressed")
        self.last = now
        return now

    def anchor_ready(self):
        self.ready = self.now()
        self.deadline = self.ready + POLICY["operationBudgetMs"] / 1000
        self.status = "running"

    def check(self):
        if self.status != "running":
            raise AttemptStopped("attempt_already_terminal")
        now = self.now()
        if now > self.deadline:
            raise AttemptStopped("operation_budget_exceeded")
        return now

    def capture(self):
        self.check()
        if self.captures >= POLICY["maxCaptures"]:
            raise AttemptStopped("capture_budget_exhausted")
        self.captures += 1

    def finish_frame(self, measurement, row):
        now = self.check()
        if row["storageStatus"] != "completed":
            raise AttemptStopped("persistence_not_completed")
        if row["consumed"]:
            self.status, self.ended = "success", now
            self.consumed = row["consumedMonotonic"]
            return True
        reason = row.get("reason")
        if reason == "measurement_rejected":
            reason = measurement.get("reason")
        if reason not in ("source_age_exceeds_policy", "source_age_at_consumption_exceeds_policy"):
            raise AttemptStopped(reason or "observation_rejected")
        if self.captures == POLICY["maxCaptures"]:
            raise AttemptStopped("capture_budget_exhausted")
        return False

    def fail(self, reason):
        # Preserve late writes and an honest invalid-clock result, never a negative duration.
        if self.status in ("failed", "acquisition_failed"):
            return
        self.status = "acquisition_failed" if self.ready is None else "failed"
        self.reason = reason
        try:
            self.ended = self.now()
        except AttemptStopped as exc:
            self.ended = None
            self.reason = str(exc)

    def snapshot(self):
        return {"requestId": self.request_id, "status": self.status, "reason": self.reason,
                "policy": dict(POLICY), "requestStartedMonotonic": self.started,
                "anchorReadyMonotonic": self.ready, "deadlineMonotonic": self.deadline,
                "coreEvidenceEndedMonotonic": self.ended, "currentCaptures": self.captures,
                "anchorAcquisitionMs": (self.ready-self.started)*1000 if self.ready is not None else None,
                "requestToConsumptionMs": (self.consumed-self.started)*1000 if self.consumed is not None else None,
                "requestToCoreEndMs": (self.ended-self.started)*1000 if self.ended is not None else None,
                "operationElapsedMs": (self.ended-self.ready)*1000 if self.ended is not None and self.ready is not None else None,
                "operationBudgetExceeded": self.ended is not None and self.deadline is not None and self.ended > self.deadline}
