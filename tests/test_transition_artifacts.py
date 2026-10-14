"""Real temporary-disk evidence persistence; no camera, MCU or network."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore, StorageError, read_events, verify_artifacts


class ArtifactStorageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        self.store = ArtifactStore(self.base / "run")

    def test_original_bytes_and_readback(self):
        raw = b"\xff\xd8original\x00JPEG\xff\xd9"
        digest = hashlib.sha256(raw).hexdigest()
        item = self.store.save_image("transition-1", "before", raw, digest)
        self.assertEqual((self.store.root / item["artifactRef"]).read_bytes(), raw)
        self.assertTrue(item["hashMatches"])
        self.assertEqual(item["expectedSha256"], digest)
        self.assertEqual(verify_artifacts(self.store.root, [item]), [])

    def test_hash_mismatch_keeps_original_for_quarantine(self):
        item = self.store.save_image("t1", "after", b"raw", "0" * 64)
        self.assertFalse(item["hashMatches"])
        self.assertEqual((self.store.root / item["artifactRef"]).read_bytes(), b"raw")
        self.assertEqual(verify_artifacts(self.store.root, [item]), [])

    def test_immutable_no_run_or_artifact_overwrite(self):
        with self.assertRaises(StorageError):
            ArtifactStore(self.store.root)
        self.store.write_json("records/one.json", {"a": 1})
        with self.assertRaises(StorageError):
            self.store.write_json("records/one.json", {"a": 2})
        self.assertEqual(json.loads((self.store.root / "records/one.json").read_text()), {"a": 1})
        self.assertEqual(list((self.store.root / "records").glob(".pending-*")), [])

    def test_corruption_deletion_and_invalid_descriptor(self):
        item = self.store.write_json("a.json", {"a": 1})
        (self.store.root / "a.json").write_bytes(b"bad")
        errors = verify_artifacts(self.store.root, [item])
        self.assertTrue(any("sha256_mismatch" in e for e in errors))
        self.assertTrue(any("byte_count_mismatch" in e for e in errors))
        (self.store.root / "a.json").unlink()
        self.assertTrue(verify_artifacts(self.store.root, [item]))
        for item in (None, {}, {"artifactRef": "a", "sha256": "bad", "bytes": 1},
                     {"artifactRef": "a", "sha256": "0" * 64, "bytes": True}):
            self.assertTrue(verify_artifacts(self.store.root, [item]))

    def test_path_traversal_and_symlink_rejected(self):
        for path in ("../escape.json", "/tmp/escape.json", "a//b", "./x", "a/../b", "a\\b"):
            with self.subTest(path=path), self.assertRaises(StorageError):
                self.store.write_json(path, {})
        for transition, phase in (("../bad", "before"), ("ok", "../after")):
            with self.assertRaises(StorageError):
                self.store.save_image(transition, phase, b"x", "0" * 64)
        outside = self.base / "outside"
        outside.mkdir()
        (self.store.root / "linked").symlink_to(outside, target_is_directory=True)
        with self.assertRaises(StorageError):
            self.store.write_json("linked/escape.json", {})
        self.assertFalse((outside / "escape.json").exists())
        item = self.store.write_json("a.json", {"a": 1})
        original = (self.store.root / "a.json").read_bytes()
        (outside / "same.json").write_bytes(original)
        (self.store.root / "a.json").unlink()
        (self.store.root / "a.json").symlink_to(outside / "same.json")
        self.assertTrue(verify_artifacts(self.store.root, [item]))
        root_alias = self.base / "alias"
        root_alias.symlink_to(self.store.root, target_is_directory=True)
        with self.assertRaises(StorageError):
            ArtifactStore(root_alias)
        self.assertTrue(verify_artifacts(root_alias, [item]))

    def test_nonregular_file_rejected_without_blocking(self):
        import os
        os.mkfifo(self.store.root / "pipe")
        descriptor = {"artifactRef": "pipe", "sha256": "0" * 64, "bytes": 0}
        self.assertTrue(verify_artifacts(self.store.root, [descriptor]))

    def test_image_limits_and_invalid_json(self):
        for frame in (b"", b"x" * (2 * 1024 * 1024 + 1), "not bytes"):
            with self.assertRaises(StorageError):
                self.store.save_image("t1", "before", frame, "0" * 64)
        with self.assertRaises(StorageError):
            self.store.save_image("t1", "before", b"x", "unknown")
        for payload in ({"x": float("nan")}, {"x": float("inf")}, {"x": object()}, {"x": "\ud800"}):
            with self.assertRaises(StorageError):
                self.store.write_json("bad.json", payload)
        self.assertFalse((self.store.root / "bad.json").exists())

    def test_event_append_and_missing_middle(self):
        descriptors = [self.store.append("transition_started", "t1", {"dryRun": True}),
                       self.store.append("image_saved", "t1", {}),
                       self.store.append("transition_completed", "t1", {})]
        events, errors = read_events(self.store.root)
        self.assertEqual(errors, [])
        self.assertEqual([e["sequence"] for e in events], [1, 2, 3])
        self.assertEqual(verify_artifacts(self.store.root, descriptors), [])
        (self.store.root / descriptors[1]["artifactRef"]).unlink()
        events, errors = read_events(self.store.root)
        self.assertEqual(len(events), 1)
        self.assertTrue(errors)

    def test_event_malformed_json_and_sequence(self):
        item = self.store.append("started", "t1", {})
        path = self.store.root / item["artifactRef"]
        for raw in (b"{", b'{"sequence":1,"sequence":1}',
                    b'{"sequence":true,"kind":"started","transitionId":"t1","payload":{}}',
                    b'{"sequence":1,"kind":"started","transitionId":"t1","payload":NaN}',
                    b'{"sequence":1,"kind":"started","transitionId":"t1","payload":1e999}'):
            path.write_bytes(raw)
            events, errors = read_events(self.store.root)
            self.assertEqual(events, [])
            self.assertTrue(errors)

    def test_event_missing_tail_is_not_claimed_detectable(self):
        first = self.store.append("started", "t1", {})
        last = self.store.append("completed", "t1", {})
        (self.store.root / last["artifactRef"]).unlink()
        events, errors = read_events(self.store.root)
        self.assertEqual(errors, [])
        self.assertEqual([e["kind"] for e in events], ["started"])
        self.assertEqual(verify_artifacts(self.store.root, [first]), [])
        self.assertTrue(verify_artifacts(self.store.root, [last]))

    def test_events_directory_symlink_rejected(self):
        external = self.base / "external"
        external.mkdir()
        (self.store.root / "events").symlink_to(external, target_is_directory=True)
        self.assertTrue(read_events(self.store.root)[1])
        with self.assertRaises(StorageError):
            self.store.append("started", "t1", {})

    def test_fsync_failure_and_readback_failure_are_errors(self):
        # Precreate parent so fault is the evidence durability call.
        (self.store.root / "events").mkdir()
        with patch("learning.artifacts.os.fsync", side_effect=OSError("disk error")):
            with self.assertRaises(StorageError):
                self.store.append("started", "t1", {})
        self.assertEqual(list((self.store.root / "events").iterdir()), [])
        item = self.store.append("started", "t1", {})
        self.assertEqual(item["artifactRef"], "events/00000001.json")
        with patch("learning.artifacts._read", return_value=b"corrupt readback"):
            with self.assertRaisesRegex(StorageError, "readback mismatch"):
                self.store.write_json("unverified.json", {"x": 1})
        # Ambiguous persisted evidence remains for audit, never silently replaced.
        self.assertTrue((self.store.root / "unverified.json").exists())

    def test_empty_events_and_strict_event_input(self):
        self.assertEqual(read_events(self.store.root), ([], []))
        for kind, identifier in (("../bad", "t1"), ("started", "../bad")):
            with self.assertRaises(StorageError):
                self.store.append(kind, identifier, {})
        with self.assertRaises(StorageError):
            self.store.append("started", "t1", {"bad": float("nan")})
        self.assertEqual(self.store.append("started", "t1", {})["artifactRef"], "events/00000001.json")


if __name__ == "__main__":
    unittest.main()
