"""Same-Mac CoreMedia host-clock to Python monotonic conversion evidence.

Both clocks must use mach_absolute_time. Brackets bound conversion/rounding,
not optical exposure. No camera, network, firmware or motion access occurs here.
"""
import ctypes
import math
from pathlib import Path
import platform
import subprocess
import time

SCHEMA='motionbrain.mac_clock_bridge.v1'
MARGIN_SECONDS=.000001  # Float conversion + source tick rounding, conservatively 1us.
MAX_SPAN_SECONDS=50


def finite(x):
    try:return type(x) in (int,float) and math.isfinite(x)
    except OverflowError:return False


class CMTime(ctypes.Structure):
    _fields_=[('value',ctypes.c_int64),('timescale',ctypes.c_int32),
              ('flags',ctypes.c_uint32),('epoch',ctypes.c_int64)]


class MacHostClock:
    def __init__(self):
        info=time.get_clock_info('monotonic')
        if (platform.system()!='Darwin' or info.implementation!='mach_absolute_time()'
                or not info.monotonic or info.adjustable or info.resolution>MARGIN_SECONDS):
            raise ValueError('shared_mach_absolute_clock_required')
        self.info=dict(implementation=info.implementation,monotonic=info.monotonic,
                       adjustable=info.adjustable,resolution=info.resolution)
        self.cm=ctypes.CDLL('/System/Library/Frameworks/CoreMedia.framework/CoreMedia')
        self.cm.CMClockGetHostTimeClock.restype=ctypes.c_void_p
        self.cm.CMClockGetHostTimeClock.argtypes=[]
        self.cm.CMClockGetTime.restype=CMTime
        self.cm.CMClockGetTime.argtypes=[ctypes.c_void_p]
        self.clock=self.cm.CMClockGetHostTimeClock()
        if not self.clock:raise ValueError('coremedia_host_clock_unavailable')
        self.boot_id=self.boot()

    @staticmethod
    def boot():
        value=subprocess.check_output(['/usr/sbin/sysctl','-n','kern.bootsessionuuid'],timeout=2,text=True).strip()
        import uuid
        return str(uuid.UUID(value))

    def sample(self,clock_id):
        if self.boot()!=self.boot_id:raise ValueError('host_boot_changed')
        pairs=[]
        for _ in range(8):
            before=time.monotonic()
            native=self.cm.CMClockGetTime(self.clock)
            after=time.monotonic()
            if (native.flags & 1 != 1 or native.flags & (4|8|16) or native.epoch!=0 or native.timescale<=0):
                raise ValueError('native_clock_time_invalid')
            pairs.append(dict(before=before,after=after,value=native.value,timescale=native.timescale,
                              flags=native.flags,epoch=native.epoch))
        result=dict(schemaVersion=SCHEMA,hostClockId=clock_id,hostBootId=self.boot_id,
                    sourceClock='coremedia_host_clock',targetClock='python_monotonic',
                    sharedTimebase='mach_absolute_time',pythonClock=self.info,pairs=pairs,
                    marginSeconds=MARGIN_SECONDS,opticalExposureVerified=False,
                    trainingEligible=False,controlEligible=False)
        validate_sample(result)
        return result


def validate_sample(sample):
    if (not isinstance(sample,dict) or sample.get('schemaVersion')!=SCHEMA
            or sample.get('sourceClock')!='coremedia_host_clock' or sample.get('targetClock')!='python_monotonic'
            or sample.get('sharedTimebase')!='mach_absolute_time' or sample.get('marginSeconds')!=MARGIN_SECONDS
            or any(sample.get(k) is not False for k in ('opticalExposureVerified','trainingEligible','controlEligible'))):
        raise ValueError('clock_sample_contract_invalid')
    for key in ('hostClockId','hostBootId'):
        if not isinstance(sample.get(key),str) or not sample[key]:raise ValueError('clock_identity_missing')
    info=sample.get('pythonClock',{})
    if (not isinstance(info,dict) or info.get('implementation')!='mach_absolute_time()' or info.get('monotonic') is not True
            or info.get('adjustable') is not False or not finite(info.get('resolution'))
            or not 0<info['resolution']<=MARGIN_SECONDS):raise ValueError('python_clock_source_invalid')
    pairs=sample.get('pairs')
    if not isinstance(pairs,list) or len(pairs)!=8:raise ValueError('clock_pairs_incomplete')
    low,high=-math.inf,math.inf
    previous=-math.inf
    for p in pairs:
        if (not isinstance(p,dict) or not all(finite(p.get(k)) for k in ('before','after'))
                or not 0<=p['before']<=p['after'] or p['before']<previous
                or p['after']-p['before']>.01
                or any(type(p.get(k)) is not int for k in ('value','timescale','flags','epoch'))
                or not 0<p['timescale']<2**31 or not 0<=p['value']<2**63 or p['epoch']!=0
                or p['flags'] & 1 != 1 or p['flags'] & (4|8|16)):
            raise ValueError('clock_pair_invalid')
        native=p['value']/p['timescale']
        lo=p['before']-native-MARGIN_SECONDS;hi=p['after']-native+MARGIN_SECONDS
        if not finite(native) or not lo<=0<=hi:raise ValueError('shared_clock_origin_mismatch')
        low=max(low,lo);high=min(high,hi);previous=p['after']
    if low>high:raise ValueError('clock_offset_inconsistent')
    return low,high


def bridge(before,after):
    a=validate_sample(before);b=validate_sample(after)
    if any(before[k]!=after[k] for k in ('hostClockId','hostBootId','pythonClock')):
        raise ValueError('clock_identity_changed')
    start=before['pairs'][-1]['after'];end=after['pairs'][0]['before']
    if not 0<=end-start<=MAX_SPAN_SECONDS:raise ValueError('clock_bridge_interval_invalid')
    low,high=max(a[0],b[0]),min(a[1],b[1])
    if low>high:raise ValueError('clock_offset_inconsistent')
    return dict(schemaVersion=SCHEMA,hostClockId=before['hostClockId'],hostBootId=before['hostBootId'],
                validPythonBounds=[start,end],offsetBoundsSeconds=[low,high],
                basis='same_host_mach_absolute_time_bracketed_conversion',
                opticalExposureVerified=False,trainingEligible=False,controlEligible=False)


def convert_pts(coremedia_seconds,before,after):
    evidence=bridge(before,after)
    if not finite(coremedia_seconds) or coremedia_seconds<0:raise ValueError('pts_invalid')
    low,high=evidence['offsetBoundsSeconds'];bounds=[coremedia_seconds+low,coremedia_seconds+high]
    start,end=evidence['validPythonBounds']
    if not start<=bounds[0]<=bounds[1]<=end:raise ValueError('pts_outside_bracketed_session')
    return dict(clockId=evidence['hostClockId'],earliest=bounds[0],latest=bounds[1],
                semantics='presentation_timestamp_not_optical_exposure',opticalExposureVerified=False)


def sensor_poll_marker_bounds(receipt):
    """Age field bounds the firmware poll marker, not the AS5600 conversion time."""
    from learning.mac_controller_probe import _check
    _check(receipt,'status')
    age=receipt['response']['shoulderAngle']['ageMs']
    if type(age) is not int or age<0:raise ValueError('integer_sensor_age_required')
    clock_id=receipt.get('hostClockId')
    if not isinstance(clock_id,str) or not clock_id:raise ValueError('sensor_host_clock_missing')
    return dict(clockId=clock_id,earliest=receipt['requestStartedMonotonic']-(age+1)/1000,
                latest=receipt['receivedMonotonic']-max(0,age-1)/1000,
                semantics='firmware_poll_marker_from_integer_age_and_http_bounds',
                sensorPhysicalSampleTimeVerified=False)


def link_saved_frames(root,before,after,receipts,host_clock_id,process_window):
    """Post-hoc PTS/poll-marker proximity. Never a causal training pair."""
    from learning.artifacts import describe_artifact,read_json_artifact
    evidence=bridge(before,after)
    if evidence['hostClockId']!=host_clock_id:raise ValueError('collector_clock_mismatch')
    start,end=process_window
    if not finite(start) or not finite(end) or not 0<=start<=end:raise ValueError('process_window_invalid')
    if len(receipts)!=6:raise ValueError('sensor_sample_count_invalid')
    sensors=[sensor_poll_marker_bounds(r) for r in receipts]
    if any(s['clockId']!=host_clock_id for s in sensors):raise ValueError('sensor_clock_mismatch')
    frames=[]
    for i in range(21):
        ref=describe_artifact(Path(root),f'camera/frame-{i:04d}.json')
        f=read_json_artifact(Path(root),ref)
        if f.get('clockDomain')!='coremedia_host_clock':raise ValueError('camera_clock_mismatch')
        pts=convert_pts(f['convertedHostPtsSeconds'],before,after)
        if not start<=pts['earliest']<=pts['latest']<=end:raise ValueError('pts_outside_camera_process')
        distances=[max(abs(pts['latest']-s['earliest']),abs(s['latest']-pts['earliest']))*1000 for s in sensors]
        nearest=min(range(6),key=lambda k:distances[k])
        frames.append(dict(sourceFrameId=f['sourceFrameId'],metadata=ref,presentationHostBounds=pts,
                           nearestSensorOrdinal=nearest+5,pollMarkerSeparationUpperMs=distances[nearest],
                           within20ms=distances[nearest]<=20))
    return dict(schemaVersion='motionbrain.presentation_sensor_link.v1',bridge=evidence,sensors=sensors,frames=frames,
                presentationClockLinked=True,postHocAssociation=True,physicalSampleAlignmentVerified=False,
                opticalExposureVerified=False,trainingEligible=False,controlEligible=False)
