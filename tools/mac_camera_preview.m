// Read-only, memory-only Continuity Camera preview. Not a dataset collector.
// Reuse device discovery, configuration identity and lossless frame encoding.
#define main cameraCollectorMain
#include "mac_camera_capture.m"
#undef main

@interface Preview : NSObject <AVCaptureVideoDataOutputSampleBufferDelegate>
@property AVCaptureSession *session;
@property AVCaptureDevice *device;
@property double interval, lastSelected, lastPTS;
@property NSUInteger sequence, dropped;
@property BOOL failed;
@end

@implementation Preview
- (void)captureOutput:(AVCaptureOutput*)output didDropSampleBuffer:(CMSampleBufferRef)sample fromConnection:(AVCaptureConnection*)connection {
    self.dropped++;
}
- (void)captureOutput:(AVCaptureOutput*)output didOutputSampleBuffer:(CMSampleBufferRef)sample fromConnection:(AVCaptureConnection*)connection {
    @autoreleasepool {
        if(self.failed || streamStopRequested)return;
        double now=hostNow();
        if(self.lastSelected && now-self.lastSelected<self.interval)return;
        self.lastSelected=now;
        id converted=hostPTSRecord(sample,self.session);
        if(![converted isKindOfClass:NSNumber.class] || !self.device.connected){self.failed=YES;return;}
        double pts=[converted doubleValue];
        if(!isfinite(pts)||pts>now||(self.sequence&&pts<=self.lastPTS)){self.failed=YES;return;}
        CVPixelBufferRef pixel=CMSampleBufferGetImageBuffer(sample);
        if(!pixel||CVPixelBufferGetPixelFormatType(pixel)!=kCVPixelFormatType_32BGRA){self.failed=YES;return;}
        size_t width=CVPixelBufferGetWidth(pixel),height=CVPixelBufferGetHeight(pixel);
        if(!width||!height||width>1920||height>1080){self.failed=YES;return;}
        if(CVPixelBufferLockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly)!=kCVReturnSuccess){self.failed=YES;return;}
        size_t stride=CVPixelBufferGetBytesPerRow(pixel);
        const uint8_t *source=CVPixelBufferGetBaseAddress(pixel);
        if(!source||stride<width*4){CVPixelBufferUnlockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly);self.failed=YES;return;}
        NSMutableData *raw=[NSMutableData dataWithLength:width*height*4];
        for(size_t y=0;y<height;y++)memcpy((uint8_t*)raw.mutableBytes+y*width*4,source+y*stride,width*4);
        CVPixelBufferUnlockBaseAddress(pixel,kCVPixelBufferLock_ReadOnly);
        NSData *payload=encodeLosslessFrame(raw);
        if(!payload){self.failed=YES;return;}
        NSMutableDictionary *actual=[configurationRecord(self.session,self.device,(AVCaptureVideoDataOutput*)output) mutableCopy];
        actual[@"sampleFormat"]=formatRecord(CMSampleBufferGetFormatDescription(sample));
        actual[@"deliveredPixelBuffer"]=@{@"width":@(width),@"height":@(height),@"pixelFormat":@(kCVPixelFormatType_32BGRA)};
        actual[@"videoMirrored"]=@(connection.videoMirrored);actual[@"videoRotationAngle"]=@(connection.videoRotationAngle);
        NSDictionary *header=@{@"event":@"preview_frame",@"sequence":@(self.sequence),
            @"deviceUniqueId":self.device.uniqueID,@"width":@(width),@"height":@(height),
            @"configurationSha256":digest(json(actual)),@"videoMirrored":@(connection.videoMirrored),
            @"videoRotationAngle":@(connection.videoRotationAngle),@"sourceHostSeconds":@(pts),
            @"callbackHostSeconds":@(now),@"rawByteLength":@(raw.length),@"storedByteLength":@(payload.length),
            @"pixelSha256":digest(raw),@"imageSha256":digest(payload),@"droppedFrames":@(self.dropped),
            @"trainingEligible":@NO,@"controlEligible":@NO};
        NSData *line=json(header);
        if(fwrite(line.bytes,1,line.length,stdout)!=line.length||fputc('\n',stdout)==EOF||
           fwrite(payload.bytes,1,payload.length,stdout)!=payload.length||fflush(stdout)!=0){self.failed=YES;return;}
        self.sequence++;self.lastPTS=pts;
    }
}
@end

int main(int argc,const char *argv[]){@autoreleasepool{
    if(argc==2&&!strcmp(argv[1],"--describe-capabilities")){
        emit(@{@"schemaVersion":@"motionbrain.mac_camera_preview.v1",@"diskRecording":@NO,
            @"controlEligible":@NO,@"trainingEligible":@NO,@"maximumFps":@5});return 0;
    }
    if(argc!=3){fprintf(stderr,"usage: mac-camera-preview DEVICE_UNIQUE_ID FPS\n");return 2;}
    char *end=NULL;double fps=strtod(argv[2],&end);
    if(!*argv[2]||*end||!isfinite(fps)||fps<1||fps>5)return 2;
    if([AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo]!=AVAuthorizationStatusAuthorized){fprintf(stderr,"camera permission unavailable\n");return 3;}
    AVCaptureDevice *device=nil;
    for(AVCaptureDevice *item in devices())if([item.uniqueID isEqualToString:@(argv[1])])device=item;
    if(!device||!device.isContinuityCamera){fprintf(stderr,"exact Continuity Camera unavailable; no fallback\n");return 4;}
    signal(SIGTERM,requestStreamStop);signal(SIGINT,requestStreamStop);signal(SIGPIPE,SIG_IGN);
    Preview *preview=[Preview new];preview.device=device;preview.interval=1/fps;
    AVCaptureSession *session=[AVCaptureSession new];preview.session=session;
    [session beginConfiguration];
    if(![session canSetSessionPreset:AVCaptureSessionPreset640x480])return 5;
    session.sessionPreset=AVCaptureSessionPreset640x480;
    AVCaptureDeviceInput *input=[AVCaptureDeviceInput deviceInputWithDevice:device error:nil];
    if(!input||![session canAddInput:input])return 5;
    [session addInput:input];
    AVCaptureVideoDataOutput *output=[AVCaptureVideoDataOutput new];
    output.videoSettings=@{(id)kCVPixelBufferPixelFormatTypeKey:@(kCVPixelFormatType_32BGRA)};
    output.alwaysDiscardsLateVideoFrames=YES;
    dispatch_queue_t queue=dispatch_queue_create("motionbrain.preview",DISPATCH_QUEUE_SERIAL);
    [output setSampleBufferDelegate:preview queue:queue];
    if(![session canAddOutput:output])return 5;
    [session addOutput:output];[session commitConfiguration];
    [session startRunning];
    while(!streamStopRequested&&!preview.failed){
        [[NSRunLoop currentRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:.05]];
    }
    [session stopRunning];dispatch_sync(queue,^{});
    return preview.failed?6:0;
}}
