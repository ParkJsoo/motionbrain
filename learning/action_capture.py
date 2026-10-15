"""Exercise the journaled command path with the existing synthetic collector.

This adapter deliberately cannot label an HTTP port as dry-run or enable physical
CaptureRunner sessions. Physical observation/settling commissioning remains separate.
"""
from learning.action_execution import canonical_request


class JournaledSyntheticCaptureAdapter:
    mode = "dry_run"

    def __init__(self, observation_adapter, executor, *, dispatch_guard):
        if (getattr(observation_adapter, "mode", None) != "dry_run"
                or getattr(executor.port, "synthetic", None) is not True
                or observation_adapter.clock is not executor.clock
                or observation_adapter.host_clock_id != executor.port.host_clock_id
                or not callable(dispatch_guard)):
            raise ValueError("synthetic_same_clock_adapter_and_guard_required")
        self.observations, self.executor = observation_adapter, executor
        self.dispatch_guard = dispatch_guard
        self.clock, self.host_clock_id = observation_adapter.clock, observation_adapter.host_clock_id

    def request(self, command_id):
        return canonical_request(self.observations.request(command_id))

    def observe(self, phase):
        return self.observations.observe(phase)

    def fetch_frame(self, source_frame_id):
        return self.observations.fetch_frame(source_frame_id)

    def execute(self, request):
        self.executor.propose(request)
        # Automatic confirmation exists only inside this explicitly synthetic
        # harness. Physical callers must obtain and pass an actual confirmation.
        result = self.executor.confirm(request["commandId"], confirmed_request=request,
                                       dispatch_guard=self.dispatch_guard)
        if result["executionState"] != "known_terminal":
            raise RuntimeError("journaled_action_not_terminal; inspect persistent journal")
        return {**result["action"], "synthetic": True}

    def settle(self, milliseconds):
        self.observations.settle(milliseconds)
