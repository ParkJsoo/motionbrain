// macOS-only, finite AVFoundation acquisition. No network or actuator interfaces.
#import <Foundation/Foundation.h>
#import <AVFoundation/AVFoundation.h>
#import <CoreMedia/CoreMedia.h>
#import <CommonCrypto/CommonDigest.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <stdlib.h>
#include <compression.h>
#include <dlfcn.h>
#include <signal.h>

static volatile sig_atomic_t streamStopRequested=0;
static void requestStreamStop(int signalNumber) { streamStopRequested=1; }
static const unsigned long long STREAM_MAX_BYTES=8400000000ULL;
static const double STREAM_OVERALL_SECONDS=170.0;

// The build-only helper remains self-contained: use Apple's SDK algorithm and
// the same libcompression buffer ABI/0x702 as learning.lossless_frame.
typedef size_t (*BufferCodec)(uint8_t *,size_t,const uint8_t *,size_t,void *,compression_algorithm);
static NSData *encodeLosslessFrame(NSData *raw) {
    if(raw.length<4||raw.length>1920*1080*4||raw.length%4)return nil;
    static BufferCodec encode=NULL,decode=NULL;
    static dispatch_once_t once;
    dispatch_once(&once,^{
        void *library=dlopen("/usr/lib/libcompression.dylib",RTLD_NOW|RTLD_LOCAL);
        if(library){encode=(BufferCodec)dlsym(library,"compression_encode_buffer");decode=(BufferCodec)dlsym(library,"compression_decode_buffer");}
    });
    if(!encode||!decode)return nil;
    NSMutableData *stored=[NSMutableData dataWithLength:raw.length+65536];
    size_t count=encode(stored.mutableBytes,stored.length,raw.bytes,raw.length,NULL,COMPRESSION_LZBITMAP);
    if(!count||count>raw.length+65536)return nil;
    stored.length=count;
    NSMutableData *restored=[NSMutableData dataWithLength:raw.length+1];
    size_t restoredCount=decode(restored.mutableBytes,restored.length,stored.bytes,stored.length,NULL,COMPRESSION_LZBITMAP);
    if(restoredCount!=raw.length)return nil;
    restored.length=restoredCount;
    return [restored isEqualToData:raw]?stored:nil;
}

static BOOL streamBudget(const char *arguments[4],unsigned long long *frames,unsigned long long *seconds,
                         unsigned long long *fps,unsigned long long *bytes) {
    unsigned long long *values[]={frames,seconds,fps,bytes};
    for(int i=0;i<4;i++){
        const char *arg=arguments[i];if(!*arg)return NO;
        for(const char *c=arg;*c;c++)if(*c<'0'||*c>'9')return NO;
        errno=0;char *end=NULL;*values[i]=strtoull(arg,&end,10);if(errno||*end)return NO;
    }
    return *frames>=2&&*frames<=1000&&*seconds>=2&&*seconds<=120&&*fps>=1&&*fps<=30&&*bytes>=4&&*bytes<=STREAM_MAX_BYTES;
}
static NSString *completionStatus(NSString *reason,BOOL streamMode) {
    if([reason isEqualToString:streamMode?@"duration_complete":@"sample_budget_complete"])return @"completed";
    if(streamMode&&[reason isEqualToString:@"operator_stop"])return @"partial";
    return @"failed";
}

static double hostNow(void) { return CMTimeGetSeconds(CMClockGetTime(CMClockGetHostTimeClock())); }
static NSString *digest(NSData *data) {
    unsigned char hash[CC_SHA256_DIGEST_LENGTH];
    CC_SHA256(data.bytes, (CC_LONG)data.length, hash);
    NSMutableString *s = [NSMutableString string];
    for (int i=0;i<CC_SHA256_DIGEST_LENGTH;i++) [s appendFormat:@"%02x",hash[i]];
    return s;
}
// Only storage fields vary between raw --capture and lossless --stream.
// This pure boundary is also tested without opening or discovering a camera.
static NSDictionary *frameStorageRecord(NSData *raw,BOOL streamMode,NSUInteger index,NSData **payload) {
    *payload=streamMode?encodeLosslessFrame(raw):raw;
    if(!*payload)return nil;
    NSMutableDictionary *record=[@{
        @"schemaVersion":streamMode?@"motionbrain.mac_camera_frame.v2":@"motionbrain.mac_camera_frame.v1",
        @"imageFile":[NSString stringWithFormat:streamMode?@"frame-%04lu.bgra.lzbitmap":@"frame-%04lu.bgra",(unsigned long)index],
        @"imageSha256":digest(*payload) } mutableCopy];
    if(streamMode)[record addEntriesFromDictionary:@{ @"storageEncoding":@"apple-lzbitmap-bgra8-v1",
        @"pixelSha256":digest(raw),@"rawByteLength":@(raw.length),@"storedByteLength":@((*payload).length) }];
    return record;
}
static NSData *json(id obj) { return [NSJSONSerialization dataWithJSONObject:obj options:NSJSONWritingSortedKeys error:nil]; }
static NSDictionary *timeRecord(CMTime t) {
    return @{ @"value":@(t.value),@"timescale":@(t.timescale),@"flags":@(t.flags),@"epoch":@(t.epoch) };
}
static id hostPTSRecord(CMSampleBufferRef sample, AVCaptureSession *session) {
    CMTime pts=CMSampleBufferGetPresentationTimeStamp(sample);
    CMClockRef clock=session.synchronizationClock;
    if(!CMTIME_IS_NUMERIC(pts)||pts.timescale<=0||!clock)return [NSNull null];
    CMTime converted=CMSyncConvertTime(pts,clock,CMClockGetHostTimeClock());
    double seconds=CMTimeGetSeconds(converted);
    return CMTIME_IS_NUMERIC(converted)&&isfinite(seconds)?@(seconds):[NSNull null];
}
// videoSettings is an API dictionary. Preserve supported JSON values exactly;
// report an unsupported value explicitly instead of silently omitting it.
static id settingsRecord(id value) {
    if(!value)return [NSNull null];
    if([value isKindOfClass:[NSString class]]||[value isKindOfClass:[NSNull class]])return value;
    if([value isKindOfClass:[NSNumber class]])return isfinite([value doubleValue])?value:[value description];
    if([value isKindOfClass:[NSArray class]]){
        NSMutableArray *result=[NSMutableArray array];
        for(id item in value)[result addObject:settingsRecord(item)];
        return result;
    }
    if([value isKindOfClass:[NSDictionary class]]){
        NSMutableDictionary *result=[NSMutableDictionary dictionary];
        for(id key in value)result[[key description]]=settingsRecord(value[key]);
        return result;
    }
    return @{ @"unsupportedClass":NSStringFromClass([value class]),@"description":[value description] };
}
static NSDictionary *formatRecord(CMFormatDescriptionRef format) {
    if(!format)return @{};
    CMVideoDimensions dims=CMVideoFormatDescriptionGetDimensions(format);
    return @{ @"width":@(dims.width),@"height":@(dims.height),@"mediaSubtype":@(CMFormatDescriptionGetMediaSubType(format)) };
}
static NSDictionary *configurationRecord(AVCaptureSession *session,AVCaptureDevice *device,AVCaptureVideoDataOutput *output) {
    NSMutableDictionary *active=[formatRecord(device.activeFormat.formatDescription) mutableCopy];
    active[@"videoMinFrameDuration"]=timeRecord(device.activeVideoMinFrameDuration);
    active[@"videoMaxFrameDuration"]=timeRecord(device.activeVideoMaxFrameDuration);
    return @{ @"sessionPreset":session.sessionPreset?:@"",@"outputVideoSettings":settingsRecord(output.videoSettings),@"activeFormat":active };
}
// Exclusive create; existing originals are never replaced. Include fsync and readback.
static BOOL durable(NSString *path, NSData *data) {
    if (!data) return NO;
    int fd=open(path.fileSystemRepresentation,O_WRONLY|O_CREAT|O_EXCL,0600);
    if(fd<0) return NO;
    const uint8_t *p=data.bytes; size_t remaining=data.length; BOOL ok=YES;
    while(remaining) { ssize_t n=write(fd,p,remaining); if(n<=0){ok=NO;break;} p+=n; remaining-=n; }
    if(fsync(fd)!=0) ok=NO;
    if(close(fd)!=0) ok=NO;
    if(![[NSData dataWithContentsOfFile:path] isEqualToData:data]) ok=NO;
    return ok;
}
static BOOL syncDirectory(NSString *dir) { int fd=open(dir.fileSystemRepresentation,O_RDONLY); if(fd<0)return NO; BOOL ok=fsync(fd)==0; close(fd);return ok; }
static void emit(id obj) { NSData *d=json(obj); if(d){fwrite(d.bytes,1,d.length,stdout);fputc('\n',stdout);fflush(stdout);} }
// Optional, read-only diagnostics. Getter read times do not establish exposure-time state.
static BOOL opticalDiagnosticsOption(int *argc,const char *argv[]) {
    BOOL enabled=*argc>1&&!strcmp(argv[*argc-1],"--optical-diagnostics");
    if(enabled)(*argc)--;
    return enabled;
}

static NSDictionary *opticalDiagnosticsRecord(AVCaptureDevice *device,NSString *sessionId,
                                               NSString *phase,id frameId,id callbackTime) {
    double begin=hostNow();
    NSMutableDictionary *state=[@{
        @"focusMode":@(device.focusMode),@"adjustingFocus":@(device.adjustingFocus),
        @"focusLockedSupported":@([device isFocusModeSupported:AVCaptureFocusModeLocked]),
        @"focusAutoSupported":@([device isFocusModeSupported:AVCaptureFocusModeAutoFocus]),
        @"focusContinuousSupported":@([device isFocusModeSupported:AVCaptureFocusModeContinuousAutoFocus]),
        @"exposureMode":@(device.exposureMode),@"adjustingExposure":@(device.adjustingExposure),
        @"whiteBalanceMode":@(device.whiteBalanceMode),@"adjustingWhiteBalance":@(device.adjustingWhiteBalance),
        @"lensPosition":@{@"status":@"unknown_api_unavailable_macos"},
        @"videoZoomFactor":@{@"status":@"unknown_api_unavailable_macos"},
        @"videoStabilizationMode":@{@"status":@"unknown_api_unavailable_macos"},
        @"videoFieldOfView":@{@"status":@"unknown_api_unavailable_macos"}
    } mutableCopy];
    if(@available(macOS 12.3,*)){
        state[@"centerStageEnabled"]=@(AVCaptureDevice.centerStageEnabled);
        state[@"centerStageControlMode"]=@(AVCaptureDevice.centerStageControlMode);
        state[@"centerStageActive"]=@(device.centerStageActive);
        state[@"centerStageSupported"]=@(device.activeFormat.centerStageSupported);
    }else state[@"centerStage"]=@{@"status":@"unknown_os_unavailable"};
    if(@available(macOS 12.0,*)){
        state[@"portraitEffectEnabled"]=@(AVCaptureDevice.portraitEffectEnabled);
        state[@"portraitEffectActive"]=@(device.portraitEffectActive);
    }else state[@"portraitEffect"]=@{@"status":@"unknown_os_unavailable"};
    return @{@"schemaVersion":@"motionbrain.mac_camera_optical_diagnostics.v1",
        @"sessionId":sessionId,@"deviceUniqueId":device.uniqueID,@"phase":phase,
        @"sourceFrameId":frameId?:[NSNull null],@"callbackHostSeconds":callbackTime?:[NSNull null],
        @"readStartedHostSeconds":@(begin),@"readCompletedHostSeconds":@(hostNow()),
        @"state":state,@"semantics":@"getter_state_at_read_time_not_exposure_time",
        @"settingsChanged":@NO,@"controlEligible":@NO,@"observationAdmissionChanged":@NO};
}

static NSArray *devices(void) {
    return [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:@[AVCaptureDeviceTypeBuiltInWideAngleCamera,AVCaptureDeviceTypeExternal,AVCaptureDeviceTypeContinuityCamera] mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified].devices;
}
static NSDictionary *deviceInfo(AVCaptureDevice *d) {
    return @{@"deviceUniqueId":d.uniqueID,@"localizedName":d.localizedName,@"deviceType":d.deviceType,
        @"isContinuityCamera":@(d.isContinuityCamera),@"transportType":@(d.transportType),
        @"transportVerifiedUSB":@NO}; // transportType is retained, not interpreted as USB proof.
}
@interface Probe : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property AVCaptureSession *session;
@property AVCaptureDevice *device;
@property NSString *directory;
@property NSString *sessionId;
@property NSString *viewId;
@property NSMutableArray *receipts;
@property NSMutableArray *warmupTrace;
@property NSMutableArray *droppedTrace;
@property NSUInteger droppedTraceTruncatedCount;
@property NSString *configurationSha256;
@property AVCaptureDeviceFormat *initialActiveFormat;
@property NSString *reason;
@property double launchTime,firstCallback,measurementStart,lastHostPTS,lastCallback;
@property CMTime lastPTS;
@property NSUInteger warmupFrames,droppedFrames,width,height;
@property BOOL initialMirrored;
@property double initialRotation;
@property BOOL finished;
@property BOOL streamMode;
@property BOOL opticalDiagnostics;
@property NSUInteger frameLimit, sampledOutFrames;
@property unsigned long long byteLimit, savedBytes;
@property double measurementLimit, sampleInterval, lastSelectedCallback;
@end
@implementation Probe
- (void)stop:(NSString*)reason { if(!self.finished){self.reason=reason;self.finished=YES;} }
- (void)captureOutput:(AVCaptureOutput*)output didDropSampleBuffer:(CMSampleBufferRef)buffer fromConnection:(AVCaptureConnection*)connection {
    @autoreleasepool {
        double callback=hostNow();
        NSUInteger index=self.droppedFrames++;
        if(self.streamMode)[self stop:@"camera_dropped_frame"];
        if(self.droppedTrace.count>=64){self.droppedTraceTruncatedCount++;return;}
        CFTypeRef reason=CMGetAttachment(buffer,kCMSampleBufferAttachmentKey_DroppedFrameReason,NULL);
        NSMutableDictionary *trace=[@{ @"dropIndex":@(index),@"pts":timeRecord(CMSampleBufferGetPresentationTimeStamp(buffer)),
            @"convertedHostPtsSeconds":hostPTSRecord(buffer,self.session),@"callbackHostSeconds":@(callback),
            @"reason":reason?settingsRecord((__bridge id)reason):[NSNull null] } mutableCopy];
        [self.droppedTrace addObject:trace];
        trace[@"callbackReturnPreparedHostSeconds"]=@(hostNow());
    }
}
- (void)captureOutput:(AVCaptureOutput*)output didOutputSampleBuffer:(CMSampleBufferRef)sample fromConnection:(AVCaptureConnection*)connection {
    @autoreleasepool {
        if(self.finished)return;
        double callback=hostNow();
        if(callback-self.launchTime>=30 && !self.firstCallback){[self stop:@"startup_timeout"];return;}
        if(!self.firstCallback)self.firstCallback=callback;
        if(callback-self.firstCallback<1.0 && self.warmupFrames<30){
            NSMutableDictionary *trace=[@{ @"callbackIndex":@(self.warmupFrames++),
                @"pts":timeRecord(CMSampleBufferGetPresentationTimeStamp(sample)),
                @"convertedHostPtsSeconds":hostPTSRecord(sample,self.session),@"callbackHostSeconds":@(callback) } mutableCopy];
            [self.warmupTrace addObject:trace];
            trace[@"callbackReturnPreparedHostSeconds"]=@(hostNow());
            return;
        }
        if(!self.measurementStart)self.measurementStart=callback;
        if(callback-self.measurementStart>=self.measurementLimit){[self stop:self.streamMode?@"duration_complete":@"measurement_timeout"];return;}
        if(self.streamMode && self.lastSelectedCallback && callback-self.lastSelectedCallback<self.sampleInterval){self.sampledOutFrames++;return;}
        self.lastSelectedCallback=callback;
        if(!self.device.connected || !self.device.isContinuityCamera){[self stop:@"device_disconnected_or_changed"];return;}
        CMTime pts=CMSampleBufferGetPresentationTimeStamp(sample);
        CMClockRef sourceClock=self.session.synchronizationClock;
        if(!CMTIME_IS_NUMERIC(pts)||pts.timescale<=0||!sourceClock){[self stop:@"invalid_pts_or_clock"];return;}
        CMTime converted=CMSyncConvertTime(pts,sourceClock,CMClockGetHostTimeClock());
        double hostPTS=CMTimeGetSeconds(converted);
        if(!CMTIME_IS_NUMERIC(converted)||!isfinite(hostPTS)||hostPTS>callback ||
           (self.receipts.count && (CMTimeCompare(pts,self.lastPTS)<=0||pts.epoch!=self.lastPTS.epoch||hostPTS<=self.lastHostPTS||callback<self.lastCallback))){[self stop:@"pts_invalid_repeated_or_regressed"];return;}
        CVPixelBufferRef pixel=CMSampleBufferGetImageBuffer(sample);
        if(!pixel||CVPixelBufferGetPixelFormatType(pixel)!=kCVPixelFormatType_32BGRA){[self stop:@"unsupported_pixel_format"];return;}
        size_t width=CVPixelBufferGetWidth(pixel),height=CVPixelBufferGetHeight(pixel);
        if(width==0||height==0||width>1920||height>1080||(self.receipts.count&&(width!=self.width||height!=self.height))){[self stop:@"dimensions_invalid_or_changed"];return;}
        self.width=width;self.height=height;
        if(self.receipts.count && (self.initialMirrored!=connection.videoMirrored || self.initialRotation!=connection.videoRotationAngle)){
            [self stop:@"view_transform_changed"];return;
        }
        self.initialMirrored=connection.videoMirrored;self.initialRotation=connection.videoRotationAngle;
        NSMutableDictionary *actual=[configurationRecord(self.session,self.device,(AVCaptureVideoDataOutput*)output) mutableCopy];
        actual[@"sampleFormat"]=formatRecord(CMSampleBufferGetFormatDescription(sample));
        actual[@"deliveredPixelBuffer"]=@{ @"width":@(width),@"height":@(height),@"pixelFormat":@(CVPixelBufferGetPixelFormatType(pixel)) };
        actual[@"videoMirrored"]=@(connection.videoMirrored);actual[@"videoRotationAngle"]=@(connection.videoRotationAngle);
        NSData *configurationJSON=json(actual);
        if(!configurationJSON){[self stop:@"configuration_not_serializable"];return;}
        NSString *configurationHash=digest(configurationJSON);
        if(self.configurationSha256 && (![self.configurationSha256 isEqualToString:configurationHash] || self.initialActiveFormat!=self.device.activeFormat)){
            [self stop:@"actual_configuration_changed"];return;
        }
        self.configurationSha256=configurationHash;self.initialActiveFormat=self.device.activeFormat;
        if(CVPixelBufferLockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly)!=kCVReturnSuccess){[self stop:@"pixel_lock_failed"];return;}
        size_t stride=CVPixelBufferGetBytesPerRow(pixel);
        uint8_t *base=CVPixelBufferGetBaseAddress(pixel);
        if(!base||stride<width*4){CVPixelBufferUnlockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly);[self stop:@"invalid_stride"];return;}
        NSMutableData *raw=[NSMutableData dataWithLength:width*height*4];
        uint8_t *dest=raw.mutableBytes; uint64_t sum=0;
        for(size_t y=0;y<height;y++){
            memcpy(dest+y*width*4,base+y*stride,width*4);
            if(!self.streamMode)for(size_t x=0;x<width;x++){const uint8_t *p=dest+(y*width+x)*4;sum+=77*p[2]+150*p[1]+29*p[0];}
        }
        CVPixelBufferUnlockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly);
        double processed=hostNow();
        NSUInteger index=self.receipts.count;
        NSString *metadataFile=[NSString stringWithFormat:@"frame-%04lu.json",(unsigned long)index];
        double storageStart=hostNow();
        NSData *payload=nil;
        NSDictionary *storage=frameStorageRecord(raw,self.streamMode,index,&payload);
        if(!storage){[self stop:@"lossless_encoding_or_roundtrip_failed"];return;}
        if(self.streamMode && payload.length>self.byteLimit-self.savedBytes){[self stop:@"byte_budget_exhausted"];return;}
        NSString *imageFile=storage[@"imageFile"],*imageHash=storage[@"imageSha256"];
        NSMutableDictionary *metadata=[@{@"instrumentationVersion":@1,@"sessionId":self.sessionId,
            @"actualConfiguration":actual,@"configurationSha256":configurationHash,
            @"actualConfigurationJson":[[NSString alloc] initWithData:configurationJSON encoding:NSUTF8StringEncoding],
            @"viewConfigurationId":self.viewId,@"deviceUniqueId":self.device.uniqueID,
            @"sourceFrameId":[NSString stringWithFormat:@"%@:%lu",self.sessionId,(unsigned long)index],
            @"frameIndex":@(index),@"imageFile":imageFile,@"imageSha256":imageHash,
            @"width":@(width),@"height":@(height),@"pixelFormat":@"BGRA8",@"rowBytes":@(width*4),
            @"pts":timeRecord(pts),
            @"convertedHostPtsSeconds":@(hostPTS),@"callbackHostSeconds":@(callback),@"processingCompleteHostSeconds":@(processed),
            @"storageStartedHostSeconds":@(storageStart),@"timestampSemantics":@"cmsamplebuffer_presentation_timestamp",
            @"clockDomain":@"coremedia_host_clock",@"clockConversion":@"session_synchronization_clock_to_host_clock",
            @"processing":self.streamMode?@{@"method":@"lossless_record_only_v1",@"pixelCount":@(width*height)}:
                @{@"method":@"bgra_luma_mean_v1",@"lumaMean":@((double)sum/(256.0*width*height)),@"pixelCount":@(width*height)},
            @"videoMirrored":@(connection.videoMirrored),@"videoRotationAngle":@(connection.videoRotationAngle),
            @"captureHostBoundsCalibrated":@NO,@"controlEligible":@NO,@"trainingEligible":@NO} mutableCopy];
        [metadata addEntriesFromDictionary:storage];
        NSData *meta=json(metadata);
        NSString *opticalFile=nil;NSData *opticalRaw=nil;
        if(self.opticalDiagnostics){
            opticalFile=[NSString stringWithFormat:@"frame-%04lu-optical.json",(unsigned long)index];
            NSMutableDictionary *optical=[opticalDiagnosticsRecord(self.device,self.sessionId,@"frame",
                metadata[@"sourceFrameId"],@(callback)) mutableCopy];
            optical[@"metadataFile"]=metadataFile;optical[@"metadataSha256"]=digest(meta);
            optical[@"imageFile"]=imageFile;optical[@"imageSha256"]=imageHash;
            opticalRaw=json(optical);
        }
        BOOL saved=durable([self.directory stringByAppendingPathComponent:imageFile],payload) &&
            durable([self.directory stringByAppendingPathComponent:metadataFile],meta) &&
            (!self.opticalDiagnostics || (opticalRaw && durable([self.directory stringByAppendingPathComponent:opticalFile],opticalRaw))) &&
            syncDirectory(self.directory);
        double complete=hostNow(),ms=(complete-storageStart)*1000;
        NSMutableDictionary *receipt=[@{@"frameIndex":@(index),@"metadataFile":metadataFile,@"metadataSha256":digest(meta),
            @"imageFile":imageFile,@"imageSha256":imageHash,@"persistenceMs":@(ms),@"storageCompleteHostSeconds":@(complete),@"durable":@(saved)} mutableCopy];
        if(self.opticalDiagnostics){receipt[@"opticalDiagnosticsFile"]=opticalFile;
            receipt[@"opticalDiagnosticsSha256"]=opticalRaw?digest(opticalRaw):@"";}
        self.savedBytes+=payload.length;
        [self.receipts addObject:receipt]; emit(@{@"event":@"frame",@"receipt":receipt});
        self.lastPTS=pts;self.lastHostPTS=hostPTS;self.lastCallback=callback;
        if(!saved)[self stop:@"storage_failed"];
        else if(ms>250)[self stop:@"storage_budget_exceeded"];
        else if(complete-self.measurementStart>self.measurementLimit)[self stop:self.streamMode?@"duration_complete":@"measurement_timeout"];
        else if(self.receipts.count==self.frameLimit)[self stop:self.streamMode?@"frame_budget_exhausted":@"sample_budget_complete"];
        // Captured after stdout serialization/flush and stop decisions. This is
        // a return-preparation marker, not the actual delegate-return instant.
        // The authoritative terminal receipt has this field; the earlier live
        // stdout receipt necessarily does not.
        receipt[@"callbackReturnPreparedHostSeconds"]=@(hostNow());
    }
}
@end
int main(int argc,const char *argv[]){@autoreleasepool{
    if(argc==2&&!strcmp(argv[1],"--describe-capabilities")){
        emit(@{ @"schemaVersion":@"motionbrain.mac_camera_capabilities.v1",@"instrumentationVersion":@1,
            @"capture":@{@"frameSchemaVersion":@"motionbrain.mac_camera_frame.v1",@"storageEncoding":@"raw-bgra8-v1",
                @"processingMethod":@"bgra_luma_mean_v1",@"maxFrames":@21,@"maxSeconds":@10,@"overallDeadlineSeconds":@45},
            @"stream":@{@"frameSchemaVersion":@"motionbrain.mac_camera_frame.v2",@"storageEncoding":@"apple-lzbitmap-bgra8-v1",
                @"processingMethod":@"lossless_record_only_v1",@"maxFrames":@1000,@"maxSeconds":@120,
                @"maxSampleFps":@30,@"maxBytes":@(STREAM_MAX_BYTES),@"overallDeadlineSeconds":@(STREAM_OVERALL_SECONDS),
                @"gracefulStop":@YES,@"droppedFramePolicy":@"fail_close",@"storageBudgetMs":@250},
            @"captureHostBoundsCalibrated":@NO,@"controlEligible":@NO,@"trainingEligible":@NO });return 0;
    }
    if(argc==2&&!strcmp(argv[1],"--list")){
        NSMutableArray *list=[NSMutableArray array];for(AVCaptureDevice *d in devices())[list addObject:deviceInfo(d)];
        emit(@{@"devices":list,@"authorizationStatus":@([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo])});return 0;
    }
    BOOL opticalDiagnostics=opticalDiagnosticsOption(&argc,argv);
    BOOL streamMode=argc==10 && !strcmp(argv[1],"--stream");
    if(!streamMode && (argc!=6||strcmp(argv[1],"--capture"))){fprintf(stderr,"usage: --capture DEVICE OUTPUT SESSION VIEW | --stream DEVICE OUTPUT SESSION VIEW FRAMES SECONDS FPS BYTES [--optical-diagnostics]\n");return 2;}
    unsigned long long frameLimit=21,seconds=10,fps=30,byteLimit=0;
    if(streamMode){
        const char *arguments[]={argv[6],argv[7],argv[8],argv[9]};
        if(!streamBudget(arguments,&frameLimit,&seconds,&fps,&byteLimit))return 2;
    }
    double launch=hostNow();
    NSString *deviceId=@(argv[2]),*dir=@(argv[3]),*sessionId=@(argv[4]),*viewId=@(argv[5]);
    if(!sessionId.length||!viewId.length||![[NSFileManager defaultManager] fileExistsAtPath:dir])return 2;
    // A dedicated fresh directory and nonempty immutable IDs are required by the driver.
    if([[NSFileManager defaultManager] fileExistsAtPath:[dir stringByAppendingPathComponent:@"session.json"]])return 2;
    if(streamMode){
        struct sigaction action={0};action.sa_handler=requestStreamStop;action.sa_flags=SA_RESTART;sigemptyset(&action.sa_mask);
        if(sigaction(SIGTERM,&action,NULL)!=0||sigaction(SIGINT,&action,NULL)!=0)return 6;
        // A blocked AVFoundation lifecycle call must not run beyond the external
        // 170-second clock bracket. Timeout exits preserve partial artifacts and
        // never emit a successful receipt. Normal shutdown starts before this.
        double remaining=MAX(0,STREAM_OVERALL_SECONDS-(hostNow()-launch));
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW,(int64_t)(remaining*NSEC_PER_SEC)),
            dispatch_get_global_queue(QOS_CLASS_USER_INITIATED,0),^{_exit(124);});
    }
    AVAuthorizationStatus auth=[AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    if(auth==AVAuthorizationStatusNotDetermined){
        __block BOOL answered=NO;
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted){@synchronized([AVCaptureDevice class]){answered=YES;}}];
        while(hostNow()-launch<30&&(!streamMode||!streamStopRequested)){BOOL ready;@synchronized([AVCaptureDevice class]){ready=answered;}if(ready)break;[[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.02]];}
    }
    if(streamMode&&streamStopRequested)return 1;
    if([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo]!=AVAuthorizationStatusAuthorized){fprintf(stderr,"camera permission unavailable\n");return 3;}
    AVCaptureDevice *selected=nil;for(AVCaptureDevice *d in devices())if([d.uniqueID isEqualToString:deviceId])selected=d;
    if(!selected||!selected.isContinuityCamera){fprintf(stderr,"selected Continuity Camera unavailable; no fallback\n");return 4;}
    Probe *probe=[Probe new];probe.directory=dir;probe.sessionId=sessionId;probe.viewId=viewId;probe.device=selected;probe.launchTime=launch;probe.receipts=[NSMutableArray array];
    probe.warmupTrace=[NSMutableArray array];probe.droppedTrace=[NSMutableArray array];
    probe.opticalDiagnostics=opticalDiagnostics;probe.streamMode=streamMode;probe.frameLimit=(NSUInteger)frameLimit;probe.measurementLimit=(double)seconds;probe.sampleInterval=1.0/(double)fps;probe.byteLimit=byteLimit;
    AVCaptureSession *session=[AVCaptureSession new];probe.session=session;
    [session beginConfiguration];
    if(![session canSetSessionPreset:AVCaptureSessionPreset640x480]){fprintf(stderr,"640x480 preset unavailable\n");return 5;}
    session.sessionPreset=AVCaptureSessionPreset640x480;
    NSError *error=nil;AVCaptureDeviceInput *input=[AVCaptureDeviceInput deviceInputWithDevice:selected error:&error];
    if(!input||![session canAddInput:input]){fprintf(stderr,"cannot attach selected camera\n");return 5;}[session addInput:input];
    AVCaptureVideoDataOutput *output=[AVCaptureVideoDataOutput new];
    output.videoSettings=@{(id)kCVPixelBufferPixelFormatTypeKey:@(kCVPixelFormatType_32BGRA)};
    output.alwaysDiscardsLateVideoFrames=YES;
    dispatch_queue_t queue=dispatch_queue_create("motionbrain.camera-probe",DISPATCH_QUEUE_SERIAL);
    [output setSampleBufferDelegate:probe queue:queue];
    if(![session canAddOutput:output])return 5;[session addOutput:output];[session commitConfiguration];
    NSMutableDictionary *config=[deviceInfo(selected) mutableCopy];
    [config addEntriesFromDictionary:@{@"schemaVersion":streamMode?@"motionbrain.mac_camera_stream_session.v1":@"motionbrain.mac_camera_session.v1",@"instrumentationVersion":@1,@"sessionId":sessionId,@"viewConfigurationId":viewId,
       @"actualConfigurationBeforeStart":configurationRecord(session,selected,output),
       @"requestedPreset":@"640x480",@"requestedPixelFormat":@"BGRA8",@"processingMethod":streamMode?@"lossless_record_only_v1":@"bgra_luma_mean_v1",
       @"launchHostSeconds":@(launch),@"captureHostBoundsCalibrated":@NO,@"controlEligible":@NO,@"trainingEligible":@NO}];
    if(streamMode){
        config[@"streamBudget"]=@{@"maxFrames":@(frameLimit),@"maxSeconds":@(seconds),@"sampleFps":@(fps),@"maxBytes":@(byteLimit)};
        config[@"storageEncoding"]=@"apple-lzbitmap-bgra8-v1";
        config[@"frameSchemaVersion"]=@"motionbrain.mac_camera_frame.v2";
    }
    if(opticalDiagnostics){
        config[@"opticalDiagnostics"]=@{@"enabled":@YES,@"settingsChanged":@NO,
            @"schemaVersion":@"motionbrain.mac_camera_optical_diagnostics.v1"};
        if(!durable([dir stringByAppendingPathComponent:@"optical-before-start.json"],
            json(opticalDiagnosticsRecord(selected,sessionId,@"beforeStart",nil,nil))))return 6;
    }
    if(!durable([dir stringByAppendingPathComponent:@"session.json"],json(config))||!syncDirectory(dir))return 6;
    double startRunningBegin=hostNow();[session startRunning];double startRunningEnd=hostNow();
    // Main thread owns lifecycle; callbacks and deadline state are serialized on queue.
    while(YES){
        __block BOOL done=NO;
        dispatch_sync(queue,^{double now=hostNow();if(streamMode&&streamStopRequested)[probe stop:@"operator_stop"];else if(now-launch>=(streamMode?165:45))[probe stop:@"overall_timeout"];else if(!probe.firstCallback&&now-launch>=30)[probe stop:@"startup_timeout"];else if(probe.measurementStart&&now-probe.measurementStart>=probe.measurementLimit)[probe stop:streamMode?@"duration_complete":@"measurement_timeout"];done=probe.finished;});
        if(done)break;
        [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.01]];
    }
    double stopRunningBegin=hostNow();[session stopRunning];double stopRunningEnd=hostNow();
    [output setSampleBufferDelegate:nil queue:NULL];dispatch_sync(queue,^{});
    double callbackQueueDrained=hostNow();
    if(opticalDiagnostics && !durable([dir stringByAppendingPathComponent:@"optical-after-stop.json"],
        json(opticalDiagnosticsRecord(selected,sessionId,@"afterStop",nil,nil)))){
        probe.reason=@"optical_diagnostics_storage_failed";
    }
    NSString *status=completionStatus(probe.reason,streamMode);
    BOOL success=[status isEqualToString:@"completed"];
    BOOL operatorStop=streamMode&&[probe.reason isEqualToString:@"operator_stop"];
    NSDictionary *terminal=@{@"schemaVersion":streamMode?@"motionbrain.mac_camera_stream_terminal.v1":@"motionbrain.mac_camera_terminal.v1",@"instrumentationVersion":@1,@"sessionId":sessionId,@"viewConfigurationId":viewId,@"deviceUniqueId":deviceId,
       @"lifecycle":@{ @"startRunningBeginHostSeconds":@(startRunningBegin),@"startRunningEndHostSeconds":@(startRunningEnd),
            @"stopRunningBeginHostSeconds":@(stopRunningBegin),@"stopRunningEndHostSeconds":@(stopRunningEnd),
            @"callbackQueueDrainedHostSeconds":@(callbackQueueDrained) },
       @"warmupTrace":probe.warmupTrace,@"droppedTrace":probe.droppedTrace,@"droppedTraceTruncatedCount":@(probe.droppedTraceTruncatedCount),
       @"status":status,@"reason":probe.reason?:@"unknown",@"frames":probe.receipts,
       @"recordedFrames":@(probe.receipts.count),@"expectedFrames":@(frameLimit),@"sampledOutFrames":@(probe.sampledOutFrames),@"savedBytes":@(probe.savedBytes),@"warmupFrames":@(probe.warmupFrames),@"droppedFrames":@(probe.droppedFrames),
       @"captureHostBoundsCalibrated":@NO,@"controlEligible":@NO,@"trainingEligible":@NO};
    if(!durable([dir stringByAppendingPathComponent:@"terminal.json"],json(terminal))||!syncDirectory(dir))return 6;
    emit(@{@"event":@"terminal",@"status":terminal[@"status"],@"reason":terminal[@"reason"]});return (success||operatorStop)?0:1;
}}
