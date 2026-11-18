"""수집 전용 HTTP 응답의 안전 필드와 단일 세그먼트 크기 계약."""
import json
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]

class CollectionHttpContractTest(unittest.TestCase):
    def test_route_and_required_status_guards(self):
        web = (ROOT/'src/network/web_server.cpp').read_text()
        self.assertIn('server_.on("/collection_status", HTTP_GET', web)
        compact = web.split('bool MotionBrainWebServer::buildCollectionStatusJson')[1].split('void MotionBrainWebServer::handleCollectionStatus()')[0]
        for name in ('motorEnabled','motors','speed','enabled','connected','simulated','imuOk','blocked','faultLatched','lastUpdateMs','deadman','controlActive','active'):
            self.assertIn('\\"'+name+'\\"',compact)
        self.assertIn('return json.length() <= 1200',compact)
        self.assertIn('driver_cursor.v1',compact)
        self.assertIn('stableOutputCursor',compact)
        self.assertIn('json.length()+outputProof.length()+1 <= 1200',compact)
        shoulder=(ROOT/'src/control/shoulder_angle_controller.cpp').read_text().split('void ShoulderAngleController::appendCollectionStatusJson')[1].split('void ShoulderAngleController::appendShoulderStatusJson')[0]
        for name in ('available','sensorConnected','sensorFresh','sensorReady','angleDeg','ageMs','manualDownBoundaryDeg','manualUpBoundaryDeg','manualGuardBlocked','manualCalibrationUnlock'):
            self.assertIn('\\"'+name+'\\"',shoulder)
        self.assertIn('sensor_->isReadyForMotion(SENSOR_STALE_MS)', shoulder)

    def test_maximum_output_page_json_under_1200_bytes(self):
        entry=dict(sequence=4294967295,atMs=4294967295,actionOrdinal=4294967295,motor=5,pwm=-255,reason='driver_write')
        page=dict(bootId='f'*32,evidence='driver_write',clockDomain='controller_boot_millis',dropped=4294967295,oldestSequence=4294967295,latestSequence=4294967295,gap=False,hasMore=False,entries=[entry]*8)
        self.assertLessEqual(len(json.dumps(page,separators=(',',':')).encode()),1200)

    def test_optional_cursor_maximum_metadata_and_existing_size_cap(self):
        proof=dict(schemaVersion='driver_cursor.v1',bootId='f'*32,
                   sequenceBefore=4294967295,sequenceAfter=4294967295,stable=False)
        encoded=json.dumps(dict(outputProof=proof),separators=(',',':')).encode()
        self.assertLess(len(encoded),200)
        # Existing fields are retained. Oversized status simply omits this
        # optional extension rather than changing collection.v1's 1200 cap.
        for base in (835,1199):
            appended=base+len(encoded)-1
            output_length=appended if appended<=1200 else base
            self.assertLessEqual(output_length,1200)

    def test_combined_snapshot_reuses_existing_serializers_and_requires_stable_cursor(self):
        web=(ROOT/'src/network/web_server.cpp').read_text()
        self.assertIn('server_.on("/collection_observation", HTTP_GET',web)
        combined=web.split('void MotionBrainWebServer::handleCollectionObservation()')[1].split('void MotionBrainWebServer::handleStatus()')[0]
        for required in ('observationPageArguments(server_,after,limit)','buildCollectionStatusJson(status,true)',
                         'BoundedActions::appendOutputJson(outputs,true,after,limit)','stableCollectionObservation',
                         'sequenceBefore,sequenceAfter,page.latest','bootBefore.c_str(),bootAfter.c_str(),pageBoot.c_str()',
                         'outputs.length()>1200','json.length()>2500','collection_observation_cursor_changed',
                         'sendNoStoreHeaders(server_)','motorControl_!=&motorControl'):
            self.assertIn(required,combined)
        for mutation in ('setSpeed','stopAll','motorForward','motorReverse','append(', 'update(', 'delay(', 'yield('):
            self.assertNotIn(mutation,combined)
        self.assertIn('else if (requireOutputProof) return false;',web)

    def test_combined_maximum_size_remains_bounded(self):
        # 두 기존 본문 한도를 유지하고, 새 envelope만 더한다.
        overhead=len('{"schemaVersion":"collection_observation.v1","status":,"outputs":}')
        self.assertLessEqual(1200+1200+overhead,2500)
