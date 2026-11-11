"""공개 SDK 읽기 진단 계약. 가짜 장치 getter만 사용하며 카메라 접근 없음."""
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]

@unittest.skipUnless(sys.platform=='darwin' and shutil.which('xcrun'),'Apple SDK required')
class OpticalDiagnosticsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.addClassCleanup(cls.temp.cleanup)
        root=Path(cls.temp.name);source=root/'test.m';cls.binary=root/'test'
        source.write_text('#define main cameraProgramMain\n#include '+json.dumps(str(ROOT/'tools/mac_camera_capture.m'))+'\n#undef main\n'+r'''
#import <objc/runtime.h>
@interface FakeOptics : NSObject
@end
@implementation FakeOptics
- (NSInteger)focusMode{return 2;}
- (BOOL)isAdjustingFocus{return YES;}
- (BOOL)isFocusModeSupported:(NSInteger)mode{return mode==2;}
- (NSInteger)exposureMode{return 1;}
- (BOOL)isAdjustingExposure{return NO;}
- (NSInteger)whiteBalanceMode{return 1;}
- (BOOL)isAdjustingWhiteBalance{return NO;}
- (NSString*)uniqueID{return @"synthetic-only";}
- (BOOL)isCenterStageActive{return NO;}
- (id)activeFormat{return self;}
- (BOOL)isCenterStageSupported{return YES;}
- (BOOL)isPortraitEffectActive{return NO;}
@end
static BOOL falseGetter(id object,SEL selector){return NO;}
static NSInteger modeGetter(id object,SEL selector){return 0;}
int main(int argc,const char *argv[]){@autoreleasepool{
    if(argc>1&&!strcmp(argv[1],"--parse")){
        int count=argc-1;BOOL enabled=opticalDiagnosticsOption(&count,argv+1);
        emit(@{@"enabled":@(enabled),@"argc":@(count)});return 0;
    }
    if(argc>1&&!strcmp(argv[1],"--invalid"))return cameraProgramMain(argc-1,argv+1);
    Class meta=object_getClass([AVCaptureDevice class]);
    class_replaceMethod(meta,@selector(isCenterStageEnabled),(IMP)falseGetter,"B@:");
    class_replaceMethod(meta,@selector(centerStageControlMode),(IMP)modeGetter,"q@:");
    class_replaceMethod(meta,@selector(isPortraitEffectEnabled),(IMP)falseGetter,"B@:");
    emit(opticalDiagnosticsRecord((AVCaptureDevice*)(id)[FakeOptics new],@"synthetic",@"frame",@"synthetic:3",@1.));
    return 0;
}}
''')
        result=subprocess.run(['xcrun','clang','-fobjc-arc','-O2','-Wall','-Wextra','-Wno-unused-parameter',
            '-framework','Foundation','-framework','AVFoundation','-framework','CoreMedia','-framework','CoreVideo',
            str(source),'-o',str(cls.binary)],capture_output=True,text=True,timeout=60)
        if result.returncode:raise RuntimeError(result.stderr)

    def call(self,*args):
        return subprocess.run([str(self.binary),*args],capture_output=True,text=True,timeout=10)

    def test_getters_preserve_unknown_and_do_not_claim_exposure_state(self):
        result=self.call();self.assertEqual(result.returncode,0,result.stderr)
        value=json.loads(result.stdout)
        self.assertEqual(value['sourceFrameId'],'synthetic:3')
        self.assertEqual(value['callbackHostSeconds'],1)
        self.assertLessEqual(value['readStartedHostSeconds'],value['readCompletedHostSeconds'])
        self.assertEqual(value['semantics'],'getter_state_at_read_time_not_exposure_time')
        self.assertFalse(value['settingsChanged']);self.assertFalse(value['controlEligible'])
        state=value['state'];self.assertTrue(state['adjustingFocus']);self.assertFalse(state['focusLockedSupported'])
        for key in ('lensPosition','videoZoomFactor','videoStabilizationMode','videoFieldOfView'):
            self.assertEqual(state[key]['status'],'unknown_api_unavailable_macos')

    def test_flag_is_optional_and_only_consumed_at_end(self):
        for args,expected in [(('--capture','id','dir','session','view'),(False,6)),
            (('--capture','id','dir','session','view','--optical-diagnostics'),(True,6)),
            (('--optical-diagnostics','--capture'),(False,3))]:
            value=json.loads(self.call('--parse',*args).stdout)
            self.assertEqual((value['enabled'],value['argc']),expected)

    def test_invalid_cli_options_reject_before_camera_access(self):
        for args in [('--list','--optical-diagnostics'),('--capture','--optical-diagnostics'),
                     ('--stream','id','dir','session','view','2','2','1','4','--bogus')]:
            self.assertEqual(self.call('--invalid',*args).returncode,2)
