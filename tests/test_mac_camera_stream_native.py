"""실제 카메라 없이 native 저장 경계·압축·종료 계약을 검증한다."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import unittest

from learning.action_recorder import ActionRecorder
from learning.artifacts import ArtifactStore
from learning.lossless_frame import ENCODING, decode


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(sys.platform == "darwin" and shutil.which("xcrun"), "Apple SDK required")
class NativeStreamTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.build.cleanup)
        root = Path(cls.build.name)
        source = root / "native-test.m"
        cls.binary = root / "native-test"
        source.write_text("#define main cameraProgramMain\n#include " + json.dumps(str(ROOT / "tools/mac_camera_capture.m")) + "\n#undef main\n" + r'''
int main(int argc,const char *argv[]){@autoreleasepool{
    if(argc==2&&!strcmp(argv[1],"--describe-capabilities"))return cameraProgramMain(argc,argv);
    if(argc==6&&!strcmp(argv[1],"--budget")){
        unsigned long long frames=0,seconds=0,fps=0,bytes=0;
        const char *arguments[]={argv[2],argv[3],argv[4],argv[5]};
        return streamBudget(arguments,&frames,&seconds,&fps,&bytes)?0:2;
    }
    if(argc==3&&!strcmp(argv[1],"--wait-stop")){
        struct sigaction action={0};action.sa_handler=requestStreamStop;action.sa_flags=SA_RESTART;sigemptyset(&action.sa_mask);
        if(sigaction(SIGTERM,&action,NULL))return 5;
        emit(@{@"ready":@YES});double start=hostNow();
        while(!streamStopRequested&&hostNow()-start<2)usleep(10000);
        if(!streamStopRequested)return 5;
        NSDictionary *terminal=@{@"status":completionStatus(@"operator_stop",YES),@"reason":@"operator_stop",
                                  @"controlEligible":@NO};
        NSString *directory=@(argv[2]);
        BOOL saved=durable([directory stringByAppendingPathComponent:@"terminal.json"],json(terminal))&&syncDirectory(directory);
        emit(terminal);return saved?0:6;
    }
    if(argc==2&&!strcmp(argv[1],"--failure-priority")){
        for(NSString *reason in @[@"camera_dropped_frame",@"storage_budget_exceeded"]){
            Probe *probe=[Probe new];[probe stop:reason];[probe stop:@"operator_stop"];
            if(![probe.reason isEqualToString:reason]||![completionStatus(probe.reason,YES) isEqualToString:@"failed"])return 7;
        }
        if(![completionStatus(@"operator_stop",NO) isEqualToString:@"failed"])return 7;
        return 0;
    }
    if(argc!=5||strcmp(argv[1],"--encode"))return 2;
    NSData *raw=[NSData dataWithContentsOfFile:@(argv[2])],*payload=nil;
    BOOL streamMode=!strcmp(argv[4],"stream");
    NSDictionary *storage=frameStorageRecord(raw,streamMode,0,&payload);
    if(!storage)return 3;
    NSString *directory=@(argv[3]),*metadataFile=@"frame-0000.json";
    NSData *metadata=json(storage);
    BOOL saved=durable([directory stringByAppendingPathComponent:storage[@"imageFile"]],payload)&&
        durable([directory stringByAppendingPathComponent:metadataFile],metadata)&&syncDirectory(directory);
    NSDictionary *receipt=@{@"frameIndex":@0,@"imageFile":storage[@"imageFile"],@"metadataFile":metadataFile,
        @"metadataSha256":digest(metadata),@"imageSha256":storage[@"imageSha256"],@"persistenceMs":@1,@"durable":@(saved)};
    emit(@{@"storage":storage,@"receipt":receipt});return saved?0:6;
}}
''')
        sdk = subprocess.check_output(["xcrun", "--sdk", "macosx", "--show-sdk-path"], text=True).strip()
        result = subprocess.run(["xcrun", "--sdk", "macosx", "clang", "-isysroot", sdk, "-fobjc-arc",
                                 "-O2", "-Wall", "-Wextra", "-Wno-unused-parameter",
                                 "-framework", "Foundation", "-framework", "AVFoundation",
                                 "-framework", "CoreMedia", "-framework", "CoreVideo", str(source), "-o", str(cls.binary)],
                                capture_output=True, text=True, timeout=60)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def encoded(self, raw, mode="stream", directory="output"):
        source = self.root / (directory + "-input.bgra")
        source.write_bytes(raw)
        output = self.root / directory
        output.mkdir(exist_ok=True)
        result = subprocess.run([str(self.binary), "--encode", str(source), str(output), mode],
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stderr)
        return output, json.loads(result.stdout)

    def test_capabilities_are_read_only_and_match_recorder_budget(self):
        result = subprocess.run([str(self.binary), "--describe-capabilities"], capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stderr)
        cap = json.loads(result.stdout)
        self.assertEqual(cap["capture"]["frameSchemaVersion"], "motionbrain.mac_camera_frame.v1")
        self.assertEqual(cap["capture"]["processingMethod"], "bgra_luma_mean_v1")
        stream = cap["stream"]
        self.assertEqual(stream["frameSchemaVersion"], "motionbrain.mac_camera_frame.v2")
        self.assertEqual(stream["storageEncoding"], ENCODING)
        self.assertEqual(stream["processingMethod"], "lossless_record_only_v1")
        self.assertEqual(stream["droppedFramePolicy"], "fail_close")
        self.assertEqual(stream["storageBudgetMs"], 250)
        self.assertEqual(stream["overallDeadlineSeconds"], 170)
        plan = dict(deviceId="synthetic", sessionId="s", viewId="v", maxFrames=stream["maxFrames"],
                    seconds=stream["maxSeconds"], sampleFps=stream["maxSampleFps"], maxBytes=stream["maxBytes"])
        ActionRecorder(self.root / "not-created", self.binary, plan)
        self.assertFalse((self.root / "not-created").exists())
        self.assertFalse(cap["controlEligible"])

    def test_actual_native_codec_matches_existing_lossless_decoder(self):
        for index, raw in enumerate([bytes([0, 30, 80, 255]) * (64 * 48), os.urandom(64 * 48 * 4),
                                     bytes([0, 30, 80, 255]) * (1920 * 1080)]):
            with self.subTest(index=index):
                output, record = self.encoded(raw, directory="roundtrip" + str(index))
                metadata, receipt = record["storage"], record["receipt"]
                stored = (output / metadata["imageFile"]).read_bytes()
                self.assertTrue(metadata["imageFile"].endswith(".bgra.lzbitmap"))
                self.assertEqual(metadata["schemaVersion"], "motionbrain.mac_camera_frame.v2")
                self.assertEqual(metadata["rawByteLength"], len(raw))
                self.assertEqual(metadata["storedByteLength"], len(stored))
                self.assertEqual(metadata["imageSha256"], hashlib.sha256(stored).hexdigest())
                self.assertEqual(metadata["pixelSha256"], hashlib.sha256(raw).hexdigest())
                self.assertEqual(decode(stored, len(raw), metadata["pixelSha256"]), raw)
                self.assertTrue(receipt["durable"])
                self.assertEqual(receipt["metadataSha256"], hashlib.sha256((output / "frame-0000.json").read_bytes()).hexdigest())

    def test_codec_rejects_invalid_raw_size_without_frame_artifacts(self):
        for index, raw in enumerate([b"abc", bytes(1920 * 1080 * 4 + 4)]):
            source = self.root / f"bad-{index}.bgra"
            source.write_bytes(raw)
            output = self.root / f"rejected-{index}"
            output.mkdir()
            result = subprocess.run([str(self.binary), "--encode", str(source), str(output), "stream"],
                                    capture_output=True, timeout=3)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(list(output.iterdir()), [])

    def test_legacy_capture_storage_remains_raw_v1(self):
        raw = bytes(range(256)) * 10
        output, record = self.encoded(raw, mode="capture")
        metadata = record["storage"]
        self.assertEqual(metadata["schemaVersion"], "motionbrain.mac_camera_frame.v1")
        self.assertEqual(metadata["imageFile"], "frame-0000.bgra")
        self.assertEqual((output / metadata["imageFile"]).read_bytes(), raw)
        self.assertNotIn("storageEncoding", metadata)
        self.assertNotIn("pixelSha256", metadata)

    def test_native_v2_storage_is_accepted_by_action_recorder(self):
        raw = bytes([4, 30, 55, 255]) * 64
        output, record = self.encoded(raw)
        metadata = dict(record["storage"], sessionId="s", viewConfigurationId="v", deviceUniqueId="d",
                        frameIndex=0, sourceFrameId="s:0", pixelFormat="BGRA8", clockDomain="coremedia_host_clock",
                        timestampSemantics="cmsamplebuffer_presentation_timestamp", width=8, height=8, rowBytes=32,
                        actualConfigurationJson="{}", configurationSha256=hashlib.sha256(b"{}").hexdigest(),
                        pts=dict(value=99800, timescale=1000, epoch=0, flags=1),
                        convertedHostPtsSeconds=99.8, callbackHostSeconds=99.81,
                        videoMirrored=False, videoRotationAngle=0)
        encoded = json.dumps(metadata).encode()
        (output / "frame-0000.json").write_bytes(encoded)
        receipt = dict(record["receipt"], metadataSha256=hashlib.sha256(encoded).hexdigest())
        recorder = ActionRecorder(output, self.binary, dict(deviceId="d", sessionId="s", viewId="v",
            maxFrames=20, seconds=10, sampleFps=8, maxBytes=1000000, width=8, height=8))
        recorder.started=90.;recorder.offset_low=-.000001;recorder.offset_high=.000001
        recorder.store=ArtifactStore(output / "host-ingress")
        recorder._packet(receipt, 100.)
        self.assertEqual(len(recorder.frames), 1)
        self.assertEqual(recorder.frames[0]["pixelSha256"], hashlib.sha256(raw).hexdigest())

    def test_budget_validation_has_same_public_maxima_and_rejects_overflow(self):
        cases = [([1000, 120, 30, 8400000000], True), ([2, 2, 1, 4], True),
                 ([1001, 120, 30, 8400000000], False), ([1000, 121, 30, 8400000000], False),
                 ([1000, 120, 31, 8400000000], False), ([1000, 120, 30, 8400000001], False),
                 (["18446744073709551616", 2, 1, 4], False), (["2.0", 2, 1, 4], False), ([-2, 2, 1, 4], False)]
        for values, accepted in cases:
            with self.subTest(values=values):
                result = subprocess.run([str(self.binary), "--budget", *map(str, values)], capture_output=True, timeout=3)
                self.assertEqual(result.returncode == 0, accepted)

    def test_operator_stop_has_durable_partial_terminal_without_camera(self):
        process = subprocess.Popen([str(self.binary), "--wait-stop", str(self.root)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(lambda: process.kill() if process.poll() is None else None)
        self.assertTrue(json.loads(process.stdout.readline())["ready"])
        process.send_signal(signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=3)
        self.assertEqual(process.returncode, 0, stderr)
        terminal = json.loads((self.root / "terminal.json").read_text())
        self.assertEqual(terminal["status"], "partial")
        self.assertEqual(terminal["reason"], "operator_stop")
        self.assertEqual(json.loads(stdout), terminal)

    def test_first_drop_or_storage_failure_is_not_relabelled_operator_stop(self):
        result = subprocess.run([str(self.binary), "--failure-priority"], capture_output=True, timeout=3)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_existing_original_is_never_overwritten(self):
        output, _ = self.encoded(bytes([4, 30, 55, 255]) * 64)
        original = {p.name: p.read_bytes() for p in output.iterdir()}
        raw = self.root / "new.bgra"
        raw.write_bytes(bytes(256))
        result = subprocess.run([str(self.binary), "--encode", str(raw), str(output), "stream"], capture_output=True, text=True, timeout=3)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(json.loads(result.stdout)["receipt"]["durable"])
        self.assertEqual(original, {p.name: p.read_bytes() for p in output.iterdir()})
