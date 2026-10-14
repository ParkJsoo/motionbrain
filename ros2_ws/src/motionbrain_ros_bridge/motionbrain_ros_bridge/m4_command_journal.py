"""Bounded, persistent command ledger for one POSIX host executor.

The file lock deliberately rejects a second live executor using the same journal.
Rows are never evicted: losing a tombstone could allow an old command to run again.
"""
from __future__ import annotations

import fcntl
import json
import os
from pathlib import Path
import sqlite3
import threading
from typing import Any

from motionbrain_ros_bridge.m4_write_contract import M4ContractError


class M4CommandJournal:
    def __init__(self, path: str, max_records: int) -> None:
        if isinstance(max_records, bool) or not isinstance(max_records, int) or max_records < 1:
            raise ValueError("max_records must be a positive integer")
        self.max_records = max_records
        self.lock = threading.RLock()
        self._file_lock = None
        self.connection = None
        try:
            if path != ":memory:":
                journal_path = Path(path).expanduser().resolve()
                journal_path.parent.mkdir(parents=True, exist_ok=True)
                self._file_lock = open(str(journal_path) + ".lock", "a+b")
                try:
                    fcntl.flock(self._file_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise M4ContractError("journal_in_use") from None
                path = str(journal_path)
            self.connection = sqlite3.connect(path, check_same_thread=False)
            self.connection.execute("PRAGMA synchronous=FULL")
            self.connection.execute("""CREATE TABLE IF NOT EXISTS commands (
                command_id TEXT PRIMARY KEY,
                fingerprint TEXT NOT NULL,
                request_json TEXT NOT NULL,
                expires_at REAL NOT NULL,
                phase TEXT NOT NULL,
                result_json TEXT NOT NULL
            )""")
            self.connection.commit()
        except Exception:
            self.close()
            raise

    @staticmethod
    def _decode(row: tuple | None) -> dict[str, Any] | None:
        if row is None:
            return None
        return dict(commandId=row[0], fingerprint=row[1], request=json.loads(row[2]),
                    expiresAt=row[3], phase=row[4], result=json.loads(row[5]))

    def get(self, command_id: str) -> dict[str, Any] | None:
        with self.lock:
            return self._decode(self.connection.execute(
                "SELECT * FROM commands WHERE command_id = ?", (command_id,)
            ).fetchone())

    def insert(self, command_id: str, fingerprint: str, request: dict,
               expires_at: float, result: dict) -> dict:
        with self.lock, self.connection:
            existing = self.get(command_id)
            if existing is not None:
                if existing["fingerprint"] != fingerprint:
                    raise M4ContractError("command_id_conflict")
                return existing
            count = self.connection.execute("SELECT COUNT(*) FROM commands").fetchone()[0]
            if count >= self.max_records:
                raise M4ContractError("journal_capacity_reached")
            self.connection.execute("INSERT INTO commands VALUES (?, ?, ?, ?, ?, ?)", (
                command_id, fingerprint, json.dumps(request, sort_keys=True, allow_nan=False),
                expires_at, "pending", json.dumps(result, sort_keys=True, allow_nan=False)))
        return self.get(command_id)

    def reserve(self, command_id: str) -> tuple[dict | None, str | None]:
        with self.lock, self.connection:
            record = self.get(command_id)
            if record is None:
                return None, "proposal_not_found"
            if record["phase"] != "pending":
                return record, "proposal_already_consumed"
            if self.connection.execute(
                "SELECT 1 FROM commands WHERE phase = 'outcome_unknown' LIMIT 1"
            ).fetchone():
                return record, "unresolved_outcome_unknown"
            if self.connection.execute(
                "SELECT 1 FROM commands WHERE phase IN ('validating', 'dispatching') LIMIT 1"
            ).fetchone():
                return record, "executor_busy"
            self.connection.execute(
                "UPDATE commands SET phase = 'validating' WHERE command_id = ? AND phase = 'pending'",
                (command_id,))
        return record, None

    def save(self, command_id: str, phase: str, result: dict) -> None:
        with self.lock, self.connection:
            self.connection.execute(
                "UPDATE commands SET phase = ?, result_json = ? WHERE command_id = ?",
                (phase, json.dumps(result, sort_keys=True, allow_nan=False), command_id))

    def interrupted(self) -> list[dict]:
        with self.lock:
            rows = self.connection.execute(
                "SELECT * FROM commands WHERE phase IN ('pending', 'validating', 'dispatching')"
            ).fetchall()
            return [self._decode(row) for row in rows]

    def close(self) -> None:
        with self.lock:
            if self.connection is not None:
                self.connection.close()
                self.connection = None
            if self._file_lock is not None:
                self._file_lock.close()
                self._file_lock = None
