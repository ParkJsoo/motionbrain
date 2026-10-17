"""Offline evidence joins; fixtures are never physical measurements or training data."""
import copy
import contextlib
import io
import json
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning.artifacts import ArtifactStore, StorageError, describe_artifact
from learning.collection_readiness import (POLICY, load_transition, review_demo,
                                           review_transition, review_device_capture, scene_template)
from learning.dry_run import JPEG_BYTES, SyntheticAdapter
from learning.hardware_demo import archive_demo
from learning.runner import CaptureRunner
from tests.test_action_transition_schema import make_transition
from tools.review_collection_readiness import main


class CollectionReadinessTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.store = ArtifactStore(Path(self.temp.name)/"evidence")
        self.root = self.store.root
        self.serial = 0
        self.t = make_transition()
        self.t["collection"] = dict(mode="dry_run", evidence="synthetic")
        # Keep fixture provenance. A full scene still cannot become physical data.
        self.t["action"]["startHostBounds"].update(earliest=10.02,latest=10.025)
        for phase, stamp in (("before", 9.98), ("after", 11.0)):
            o = self.t[phase]
            o.update(capturedUs=round(stamp*1e6), sourceFrameId="a"*32+":"+str(round(stamp*1e6)),
                     receivedMonotonic=stamp+.03, inferenceCompletedMonotonic=stamp+.04)
            o["detectionSourceFrameId"] = o["sourceFrameId"]
            o.update(detected=True, liveDetected=True, label="lipbalm", width=2, height=2,
                     targetBox=dict(x=0,y=0,width=1,height=1))
            o["captureHostBounds"].update(earliest=stamp, latest=stamp+.01)
            o["m4"]["sampledHostBounds"].update(earliest=stamp+.025, latest=stamp+.026)
            image = self.store.save_image("fixture", phase, JPEG_BYTES, hashlib.sha256(JPEG_BYTES).hexdigest())
            o["imageSha256"] = image["sha256"]
            o["image"] = {k:image[k] for k in ("artifactRef", "sha256")}
        self.ref = self.store.write_json("transition.json", self.t)
        self.scene = scene_template(self.t, self.ref)
        self.scene.update(viewId="new-view", objectInstanceId="lipbalm-1", m4ToleranceDegrees=.2)
        self.calibration = dict(sessionId=self.scene["sessionId"], viewId="new-view", cameraBootId="a"*32,
                                method="optical_exposure_bounds", opticalExposureVerified=True,
                                reviewer="synthetic-reviewer", measurement=self.put({"synthetic":True}),
                                validHostBounds=self.bounds(0,20))
        self.scene["opticalCalibration"] = self.put(self.calibration)
        self.scene["anchor"] = self.frame("anchor", 9.90)
        self.scene["before"] = self.frame("before", 9.98, self.t["before"])
        self.scene["after"] = self.frame("after", 11, self.t["after"])
        self.scene["settling"] = [self.frame("settle"+str(i),stamp) for i,stamp in enumerate((10.80,10.84,10.88))]
        self.review = dict(transition=self.ref, sessionId=self.scene["sessionId"], viewId="new-view",
                           objectInstanceId="lipbalm-1", reviewer="synthetic-reviewer", method="human_sequence_review",
                           frames=self.frame_refs(), wholeArmStationary=True, objectStationary=True, taskOutcome="unknown")
        self.scene["visualReview"] = self.put(self.review)
        self.tail = copy.deepcopy(self.t["action"]["outputLog"])
        self.tail["readHostBounds"] = self.bounds(11.05,11.06)
        self.scene["finalOutputLog"] = self.put(self.tail)

    def put(self,value):
        self.serial += 1
        return self.store.write_json(f"evidence/{self.serial}.json", value)

    def bounds(self,a,b):
        return dict(clockId="host-a",earliest=a,latest=b)

    def frame(self,name,stamp,o=None):
        if o:
            image=describe_artifact(self.root,o["image"]["artifactRef"])
        else:
            image=self.store.save_image("fixture",name,JPEG_BYTES,hashlib.sha256(JPEG_BYTES).hexdigest())
        fid="a"*32+":"+str(round(stamp*1e6))
        receipt=dict(basis="original_capture_transaction",clockId="host-a",startedMonotonic=stamp+.012,
                     completedMonotonic=stamp+.035,image=image,sourceFrameId=fid,
                     fileFsynced=True,directoryFsynced=True,readbackVerified=True)
        value=dict(sessionId=self.scene["sessionId"],viewId="new-view",objectInstanceId="lipbalm-1",targetLabel="lipbalm",
                   sourceFrameId=fid,cameraBootId="a"*32,capturedUs=round(stamp*1e6),image=image,
                   opticalCalibration=self.scene["opticalCalibration"],exposureHostBounds=self.bounds(stamp-.001,stamp+.011),
                   consumedMonotonic=stamp+.04,persistence={**receipt,"receipt":self.put(receipt)},
                   imageWidth=2,imageHeight=2,box=dict(x=0,y=0,width=1,height=1),
                   annotationMethod="human_semantic_review",reviewer="synthetic-reviewer")
        if name!="anchor":
            m4=o["m4"] if o else dict(motor=4,sensor="AS5600",measured=True,angleDegrees=4.5,
                                       sampledHostBounds=self.bounds(stamp+.025,stamp+.026))
            value["m4"]=self.put(m4)
        return value

    def frame_refs(self):
        return {**{k:self.scene[k]["image"] for k in ("anchor","before","after")},
                **{f"settling[{i}]":f["image"] for i,f in enumerate(self.scene["settling"])}}

    def report(self):
        return review_transition(self.root,"transition.json",self.scene)

    def structural(self):
        return [b for b in self.report()["blockers"] if "synthetic" not in b]

    def test_complete_synthetic_contract_has_only_provenance_exclusions(self):
        self.assertEqual(self.structural(),[])
        r=self.report()
        self.assertFalse(r["trainingEligible"]);self.assertFalse(r["executionAllowed"])
        self.assertFalse(r["recordedEvidenceComplete"])

    def test_template_never_invents_scene_or_timing(self):
        value=scene_template(self.t,self.ref)
        self.assertIsNone(value["viewId"]);self.assertIsNone(value["anchor"])
        self.assertIsNone(value["opticalCalibration"]);self.assertEqual(value["settling"],[])
        self.assertEqual(value["policy"],POLICY)
        self.assertFalse(review_transition(self.root,"transition.json",value)["recordedEvidenceComplete"])

    def test_input_is_unchanged(self):
        old=copy.deepcopy(self.scene)
        self.report();self.assertEqual(old,self.scene)

    def test_new_view_and_object_identity_must_match_every_frame(self):
        for key in ("viewId","sessionId","objectInstanceId","targetLabel"):
            with self.subTest(key=key):
                old=self.scene["before"][key];self.scene["before"][key]="old-or-other"
                self.assertIn(f"before:{key}_mismatch",self.structural());self.scene["before"][key]=old

    def test_no_pts_callback_or_http_calibration(self):
        for method in ("pts","callback","http_receive","first_dma_buffer"):
            self.scene["opticalCalibration"]=self.put({**self.calibration,"method":method})
            self.assertIn("optical:exposure_evidence_required",self.structural())

    def test_before_dispatch_age_is_rechecked(self):
        self.scene["before"]["exposureHostBounds"]["earliest"]=9.8
        self.assertIn("before:expired_at_dispatch",self.structural())

    def test_m4_age_is_rechecked_at_dispatch(self):
        sample=copy.deepcopy(self.t["before"]["m4"])
        sample["sampledHostBounds"].update(earliest=9.99,latest=9.991)
        self.scene["before"]["m4"]=self.put(sample)
        self.assertIn("before:m4_expired_at_dispatch",self.structural())

    def test_anchor_age_is_checked_at_after_consumption(self):
        self.scene["anchor"]["exposureHostBounds"]=self.bounds(0,0.01)
        self.scene["after"]["consumedMonotonic"]=15.001
        self.assertIn("anchor:future_or_expired_at_use",self.structural())

    def test_future_anchor_is_rejected(self):
        self.scene["anchor"]["exposureHostBounds"]=self.bounds(10,10.01)
        self.assertIn("before:anchor_from_future_frame",self.structural())

    def test_persistence_is_original_bounded_and_completed_before_consumption(self):
        p=self.scene["before"]["persistence"]
        for change in ({"basis":"offline_copy"},{"completedMonotonic":10.3},
                       {"fileFsynced":False},{"image":self.scene["after"]["image"]}):
            self.scene["before"]["persistence"]={**p,**change}
            self.assertTrue(any("before:" in b and "persistence" in b for b in self.structural()))
        self.scene["before"]["persistence"]=p

    def test_250ms_boundary_is_not_relaxed(self):
        # Valid structure cannot silently use a 251 ms original transaction.
        p=self.scene["after"]["persistence"]
        p["completedMonotonic"]=p["startedMonotonic"]+.251
        self.assertIn("after:original_persistence_unverified_or_over_250ms",self.structural())

    def test_calibration_expiration_and_clock_mismatch(self):
        self.scene["after"]["exposureHostBounds"]["clockId"]="other-host"
        self.assertIn("after:exposure:clock_or_bounds_invalid",self.structural())
        self.scene["after"]["exposureHostBounds"]=self.bounds(21,21.01)
        self.assertIn("after:calibration_expired",self.structural())

    def test_image_and_transition_bytes_cannot_be_substituted(self):
        self.scene["before"]["image"]=self.scene["after"]["image"]
        self.assertIn("before:transition_image_mismatch",self.structural())
        self.scene["transition"]["sha256"]="0"*64
        self.assertIn("scene:transition_bytes_mismatch",self.structural())

    def test_scene_cannot_relabel_model_target_or_change_coordinates(self):
        self.scene["before"]["box"]["x"]=1
        self.assertIn("before:transition_target_mismatch",self.structural())

    def test_source_timestamp_regression_is_not_hidden_by_host_bounds(self):
        frame=self.scene["settling"][1]
        frame["capturedUs"]=1;frame["sourceFrameId"]="a"*32+":1"
        self.assertIn("frames:source_time_not_advancing",self.structural())

    def test_changed_image_is_not_accepted(self):
        (self.root/self.scene["before"]["image"]["artifactRef"]).write_bytes(b"changed")
        self.assertIn("before:image:artifact_missing_or_changed",self.structural())

    def test_traversal_and_symlink_are_rejected(self):
        self.scene["before"]["image"]["artifactRef"]="../outside.jpg"
        self.assertIn("before:image:artifact_missing_or_changed",self.structural())
        (self.root/"linked.json").symlink_to(self.root/"transition.json")
        with self.assertRaises(StorageError):load_transition(self.root,"linked.json")

    def test_missing_m4_and_motion_are_rejected(self):
        sample=dict(motor=4,sensor="AS5600",measured=True,angleDegrees=5.5,
                    sampledHostBounds=self.bounds(10.905,10.906))
        self.scene["settling"][2]["m4"]=self.put(sample)
        self.assertIn("settling:m4_moved",self.structural())
        self.scene["after"]["m4"]=None
        self.assertIn("after:m4:artifact_missing_or_changed",self.structural())

    def test_visual_settling_and_outcome_are_independent(self):
        self.scene["visualReview"]=self.put({**self.review,"taskOutcome":"success","wholeArmStationary":False})
        self.assertIn("visual:settling_unverified",self.structural())
        self.assertFalse(self.report()["trainingEligible"])

    def test_settling_requires_distinct_frames_and_time_span(self):
        self.scene["settling"][2]=copy.deepcopy(self.scene["settling"][0])
        self.assertIn("frames:reused_or_missing",self.structural())
        self.assertIn("settling:order_invalid",self.structural())

    def test_output_history_covers_after_and_remains_off(self):
        for change,reason in (({"dropped":1},"outputs:scope_or_dropped"),
                              ({"readHostBounds":self.bounds(10.9,10.99)},"outputs:read_precedes_after"),
                              ({"entries":[]},"outputs:terminal_missing")):
            self.scene["finalOutputLog"]=self.put({**self.tail,**change})
            self.assertIn(reason,self.structural())
        tail=copy.deepcopy(self.tail)
        tail["entries"].append(dict(sequence=9,pwm=20))
        self.scene["finalOutputLog"]=self.put(tail)
        self.assertIn("outputs:reactivated_after_off",self.structural())
        tail["entries"][-1]=dict(sequence=10,pwm=0)
        self.scene["finalOutputLog"]=self.put(tail)
        self.assertIn("outputs:sequence_gap",self.structural())

    def test_malformed_nested_values_fail_closed_without_crashing(self):
        original=copy.deepcopy(self.scene)
        for key in original:
            for bad in (None, [], {}, True, "", 1):
                self.scene=copy.deepcopy(original);self.scene[key]=bad
                self.assertFalse(self.report()["recordedEvidenceComplete"])
        self.scene=copy.deepcopy(original)
        for key in self.scene["before"]:
            for bad in (None, [], {}, True):
                self.scene=copy.deepcopy(original);self.scene["before"][key]=bad
                self.assertFalse(self.report()["recordedEvidenceComplete"])
        self.scene=original

    def test_missing_scene_can_be_reviewed_without_network(self):
        with patch("socket.socket",side_effect=AssertionError("no networking")):
            r=review_transition(self.root,"transition.json")
        self.assertIn("scene:missing_or_unsupported",r["blockers"])

    def test_cli_writes_report_and_template_once_without_changing_input(self):
        out=Path(self.temp.name)/"review"
        before=(self.root/"transition.json").read_bytes()
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()), \
                patch("socket.socket",side_effect=AssertionError("no networking")):
            self.assertEqual(main([str(self.root),"--transition","transition.json","--output-dir",str(out)]),2)
            self.assertEqual(main([str(self.root),"--transition","transition.json","--output-dir",str(out)]),2)
        self.assertTrue((out/"scene-template.json").is_file())
        self.assertEqual(before,(self.root/"transition.json").read_bytes())

    def test_actual_synthetic_collector_summary_can_be_reviewed(self):
        store=ArtifactStore(Path(self.temp.name)/"runner")
        runner=CaptureRunner(store,SyntheticAdapter())
        result=runner.capture(transition_id="attempt-1");runner.close()
        summaries=list(store.root.glob("attempts/*.json"))
        self.assertEqual(len(summaries),1)
        report=review_transition(store.root,str(summaries[0].relative_to(store.root)))
        self.assertFalse(report["trainingEligible"])
        self.assertEqual(result["transitionId"],"attempt-1")

    def test_invalid_tail_fields_and_reversed_device_time_are_rejected(self):
        tail=copy.deepcopy(self.tail)
        tail["entries"].append(dict(sequence=9,pwm=0))
        self.scene["finalOutputLog"]=self.put(tail)
        self.assertIn("outputs:invalid_entry",self.structural())
        tail["entries"][-1]=dict(sequence=9,pwm=0,motor=1,actionOrdinal=0,atMs=1000,reason="forced_off")
        self.scene["finalOutputLog"]=self.put(tail)
        self.assertIn("outputs:time_regressed",self.structural())

    def test_demo_inventory_preserves_failures_and_no_camera_denominator(self):
        source=Path(self.temp.name).resolve()/"manual"
        (source/"step-01/execution").mkdir(parents=True)
        (source/"step-01/plan.json").write_text('{}')
        (source/"step-01/execution/result.json").write_text('{"failure":"timeout"}')
        (source/"step-01/after.jpg").write_bytes(JPEG_BYTES)
        (source/"step-02").mkdir()
        dest=Path(self.temp.name).resolve()/"archive"
        archive_demo(source,dest)
        r=review_demo(dest)
        self.assertTrue(r["archiveIntegrityPassed"])
        self.assertEqual(len(r["attempts"]),2)
        self.assertEqual(r["attempts"][0]["reportedFailure"],"timeout")
        self.assertFalse(r["attempts"][0]["camImagePresent"])
        self.assertFalse(r["attempts"][1]["executionRecordPresent"])
        self.assertEqual(r["eligibleTransitions"],0)
        d=json.loads((dest/"demo.json").read_text());d["attempts"]=None
        (dest/"demo.json").write_text(json.dumps(d))
        self.assertIn("invalid_attempts",review_demo(dest)["integrityErrors"])

    def test_nested_hash_and_visual_outcome_malformed_are_reported(self):
        self.scene["before"]["image"]["sha256"]=[]
        self.assertIn("before:synthetic_or_invalid_image_hash",self.report()["blockers"])
        self.scene["visualReview"]=self.put({**self.review,"taskOutcome":[]})
        self.assertIn("visual:outcome_required",self.report()["blockers"])

    def test_missing_commissioning_seal_is_unknown_and_never_replayed(self):
        with patch("socket.socket",side_effect=AssertionError("no networking")):
            r=review_device_capture(self.root)
        self.assertEqual(r["commissioningAudit"]["executionState"],"outcome_unknown")
        self.assertFalse(r["trainingEligible"])


if __name__ == "__main__":
    unittest.main()
