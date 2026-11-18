"""Replay one demonstrated M4 correction on a virtual clock with an in-memory motor."""
import argparse
import json
import math
from pathlib import Path


def waveform(episode):
    samples = [a for a in episode["actions"] if a["phase"] == "lower"]
    if len(samples) != 2 or samples[1]["command"] != {"joint": "shoulder", "direction": "down", "percent": 70}:
        raise ValueError("expected the demonstrated second shoulder descent")
    entries = [w["entry"] for w in samples[1]["evidence"]["driverWrites"]]
    if [e["pwm"] for e in entries] != [-10, -20, -30, -40, -50, -40, -30, -20, -10, 0]:
        raise ValueError("unexpected correction waveform")
    steps = []
    for i, e in enumerate(entries):
        if (e["motor"] != 4 or e["reason"] != "driver_write"
                or e["sequence"] != entries[0]["sequence"] + i or type(e["atMs"]) is not int):
            raise ValueError("invalid driver journal")
        offset = e["atMs"] - entries[0]["atMs"]
        if i and offset <= steps[-1][0]:
            raise ValueError("nonmonotonic driver journal")
        steps.append((offset, e["pwm"]))
    return steps


class SimulatedCorrection:
    def __init__(self, steps):
        self.steps = steps
        self.state = "ready"
        self.started = None
        self.off_at = None
        self.cursor = 0
        self.now = 0
        self.motors = [0] * 5
        self.writes = []

    def advance(self, now):
        if not math.isfinite(now) or now < self.now:
            raise ValueError("invalid virtual time")
        self.now = now
        while self.state == "running":
            offset, pwm = self.steps[self.cursor]
            due = self.started + offset
            if due > now:
                return
            self.motors[3] = pwm
            self.writes.append((due, pwm))
            self.cursor += 1
            if pwm == 0:
                self.off_at = due
                self.state = "awaiting_post_off_pair"

    def decide(self, now, proposal):
        self.advance(now)
        if self.state == "ready" and proposal == "demonstrated_correction":
            self.started = now
            self.state = "running"
            self.advance(now)

    def observe(self, now, external_interval, gripper_interval):
        self.advance(now)
        if self.state != "awaiting_post_off_pair":
            return
        if all(self.off_at < start <= end <= now and now - start <= 3000
               for start, end in (external_interval, gripper_interval)):
            self.state = "ready_for_reassessment"


def replay(episode, choices):
    steps = waveform(episode)
    result = {}
    for name, rows in choices["replay"].items():
        sim = SimulatedCorrection(steps)
        # Proposal ordering comes from replay. Timing and post-correction frames
        # are synthetic because the physical correction was never executed.
        for i, row in enumerate(rows):
            sim.decide(i * 500, row["proposal"])
        sim.advance(sim.now + steps[-1][0] + 1)
        before = sim.state
        if sim.off_at is not None:
            off = sim.off_at
            sim.observe(sim.now, (off-1, off+1), (off+1, off+2))
            assert sim.state == before
            capture = sim.now + 1
            sim.observe(capture+1, (capture, capture+1), (capture, capture+1))
            sim.decide(sim.now+1, "demonstrated_correction")
        result[name] = dict(proposals=len(rows), writes=sim.writes, beforeNewPair=before,
                            afterSyntheticPair=sim.state, finalMotorOutputs=sim.motors)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("episode", type=Path)
    parser.add_argument("choices", type=Path)
    args = parser.parse_args()
    print(json.dumps(replay(json.loads(args.episode.read_bytes()), json.loads(args.choices.read_bytes())), indent=2))
