"""카메라 없이 실제 저장·동기화 오류와 지연 분류를 검증한다."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]


@unittest.skipUnless(sys.platform=='darwin' and shutil.which('xcrun'),'Apple SDK required')
class StorageTimingNativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.addClassCleanup(cls.temp.cleanup)
        root=Path(cls.temp.name);source=root/'test.m';cls.binary=root/'test'
        source.write_text(r'''
#import <Foundation/Foundation.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <stdarg.h>
static int mode=0;
static const char *outputPath=NULL;
static int testSync(int fd){
 if(mode==1||mode==6)usleep(270000);
 if(mode==2||mode==6){errno=EIO;return -1;}
 return fsync(fd);
}
static int testClose(int fd){
 if(mode==7)usleep(270000);
 int result=close(fd);
 if(mode==5){int c=open(outputPath,O_WRONLY);if(c>=0){write(c,"!",1);fsync(c);close(c);}}
 return result;
}
static ssize_t testWrite(int fd,const void *bytes,size_t size){
 if(mode==3){errno=ENOSPC;return -1;}
 if(mode==4&&size>33)size=33;
 return write(fd,bytes,size);
}
static int testFcntl(int fd,int command,...){
 va_list args;va_start(args,command);int value=va_arg(args,int);va_end(args);
 if(mode==8){errno=EINVAL;return -1;}
 return fcntl(fd,command,value);
}
#define fsync testSync
#define close testClose
#define write testWrite
#define fcntl testFcntl
#define main cameraProgramMain
#include '''+json.dumps(str(ROOT/'tools/mac_camera_capture.m'))+r'''
#undef main
#undef fsync
#undef close
#undef write
#undef fcntl
int main(int argc,const char *argv[]){@autoreleasepool{
 if(argc!=5)return 2;
 mode=atoi(argv[1]);outputPath=argv[3];NSData *data=[NSData dataWithContentsOfFile:@(argv[2])];
 NSDictionary *timing=nil;double begin=hostNow();BOOL ok=durableTimed(@(argv[3]),data,&timing);double ms=(hostNow()-begin)*1000;
 NSString *failure=storageFailureReason(ok,ms);
 emit(@{@"ok":@(ok),@"timing":timing?:@{},@"totalMs":@(ms),@"failure":failure?:[NSNull null],
  @"boundary250":storageFailureReason(YES,250)?:[NSNull null],@"boundaryOver":storageFailureReason(YES,250.001)?:[NSNull null],
  @"failedPriority":storageFailureReason(NO,270)?:[NSNull null]});
 return 0;
}}
''')
        sdk=subprocess.check_output(['xcrun','--sdk','macosx','--show-sdk-path'],text=True).strip()
        subprocess.run(['xcrun','clang','-isysroot',sdk,'-fobjc-arc','-O2','-Wall','-Wextra','-Wno-unused-parameter',
            '-framework','Foundation','-framework','AVFoundation','-framework','CoreMedia','-framework','CoreVideo',str(source),'-o',str(cls.binary)],check=True,capture_output=True,timeout=60)
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
    def run_case(self,mode,size=8192,existing=None):
        raw=b'abcd'* (size//4);source=self.root/'input';source.write_bytes(raw);out=self.root/'output'
        if existing is not None:out.write_bytes(existing)
        result=subprocess.run([str(self.binary),str(mode),str(source),str(out),'unused'],capture_output=True,text=True,check=True,timeout=5)
        return json.loads(result.stdout),out,raw
    def test_large_frame_disables_cache_but_fsync_and_readback_remain(self):
        r,out,raw=self.run_case(0,2*1024*1024);t=r['timing'];self.assertTrue(r['ok']);self.assertEqual(out.read_bytes(),raw)
        for key in ('cacheDisabled','fsyncSucceeded','closeSucceeded','readbackVerified'):self.assertTrue(t[key],key)
        self.assertEqual(t['bytesWritten'],len(raw));self.assertIsNone(r['boundary250']);self.assertEqual(r['boundaryOver'],'storage_budget_exceeded')
    def test_slow_sync_is_measured_and_still_fails_250_budget(self):
        r,_,_=self.run_case(1);self.assertTrue(r['ok']);self.assertGreaterEqual(r['timing']['fsyncMs'],250);self.assertEqual(r['failure'],'storage_budget_exceeded')
    def test_failed_sync_remains_failure_even_with_matching_readback(self):
        r,_,_=self.run_case(2);self.assertFalse(r['ok']);self.assertFalse(r['timing']['fsyncSucceeded']);self.assertTrue(r['timing']['readbackVerified']);self.assertEqual(r['timing']['fsyncErrno'],5);self.assertEqual(r['failure'],'storage_failed')
    def test_partial_writes_are_accounted_without_losing_bytes(self):
        r,out,raw=self.run_case(4);self.assertTrue(r['ok']);self.assertGreater(r['timing']['writeCalls'],1);self.assertEqual(r['timing']['bytesWritten'],len(raw));self.assertEqual(out.read_bytes(),raw)
    def test_write_failure_keeps_immediate_errno(self):
        r,_,_=self.run_case(3);self.assertFalse(r['ok']);self.assertEqual(r['timing']['bytesWritten'],0);self.assertEqual(r['timing']['writeErrno'],28);self.assertEqual(r['failure'],'storage_failed')
    def test_readback_corruption_is_not_accepted(self):
        r,_,_=self.run_case(5);self.assertFalse(r['ok']);self.assertFalse(r['timing']['readbackVerified']);self.assertEqual(r['failure'],'storage_failed')
    def test_original_is_never_replaced_on_exclusive_create_collision(self):
        r,out,_=self.run_case(0,existing=b'original');self.assertFalse(r['ok']);self.assertEqual(out.read_bytes(),b'original');self.assertFalse(r['timing']['openSucceeded']);self.assertNotIn('fsyncMs',r['timing'])
    def test_slow_failure_and_close_preserve_failure_priority(self):
        r,_,_=self.run_case(6);self.assertEqual(r['failure'],'storage_failed');self.assertEqual(r['failedPriority'],'storage_failed')
    def test_slow_close_is_visible_and_not_exempted_from_budget(self):
        r,_,_=self.run_case(7);self.assertTrue(r['ok']);self.assertGreaterEqual(r['timing']['closeMs'],250);self.assertEqual(r['failure'],'storage_budget_exceeded')
    def test_cache_hint_failure_is_explicit_and_no_write_follows(self):
        r,out,_=self.run_case(8,2*1024*1024);self.assertFalse(r['ok']);self.assertFalse(r['timing']['cacheDisabled']);self.assertEqual(r['timing']['cacheControlErrno'],22);self.assertNotIn('writeMs',r['timing']);self.assertEqual(out.stat().st_size,0)
