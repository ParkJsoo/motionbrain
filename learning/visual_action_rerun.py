"""Optional local RRD replay export. No spawn, connect, login or network sink."""
import json
from pathlib import Path

from learning.artifacts import _read, read_events, read_json_artifact
from learning.visual_action_collection import audit_collection


def export_recording(directory, output, *, sdk=None):
    audit=audit_collection(directory)
    if not audit['integrityOk'] or not audit['sessionClosed']:
        raise ValueError('closed_auditable_recording_required')
    # Preserving rejected input bytes does not make them a valid timeline.
    # Refuse before creating a sink instead of replaying a corrupt/missing frame.
    if audit['summary']['errors']:
        raise ValueError('validated_timeline_required')
    target=Path(output)
    if target.exists(): raise FileExistsError('rerun_output_already_exists')
    if target.suffix!='.rrd': raise ValueError('local_rrd_path_required')
    if sdk is None:
        import rerun as sdk
    import numpy as np
    events,_=read_events(directory)
    session=read_json_artifact(directory,events[0]['payload']['session'])
    # A fresh directory prevents accidental overwrite of a previously saved RRD.
    target.parent.mkdir(parents=True,exist_ok=False)
    rec=sdk.RecordingStream('motionbrain_visual_actions',recording_id=session['sessionId'])
    rec.save(str(target))
    origin=None
    try:
        for entry in events[1:]:
            if entry['kind']!='input':continue
            payload=entry['payload'];e=read_json_artifact(directory,payload['raw'])
            if origin is None:origin=e['at']
            rec.set_time('host_ingress',duration=e['at']-origin)
            # Clock origins stay distinct. Controller milliseconds are log content,
            # never plotted as a converted camera/host timeline.
            if e['type']=='frame':
                raw=_read(directory,payload['image']['artifactRef'])
                bgra=np.frombuffer(raw,np.uint8).reshape(session['height'],session['rowBytes'])[:,:session['width']*4].reshape(session['height'],session['width'],4)
                rec.log('camera/image',sdk.Image(bgra[:,:,[2,1,0]]))
                visible=[p for p in e['features'] if p['visible']]
                rec.log('camera/image/points',sdk.Points2D([[p['x'],p['y']] for p in visible],labels=[p['id'] for p in visible]))
                rec.log('camera/source_clock',sdk.TextLog(json.dumps({'sourceClockId':e['sourceClockId'],'pts':e['pts'],'frameId':e['frameId']})))
            else:
                rec.log('action/events',sdk.TextLog(json.dumps(e,ensure_ascii=False)))
        rec.flush()
    finally:
        # Disconnect finalizes the local sink and does not connect to a server.
        rec.disconnect()
    return {'output':str(target),'recordingComplete':audit['recordingComplete'],'externalUpload':False}
