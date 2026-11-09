"""감사된 M1 동작과 독립 shaft 관측을 연결한다. 물체 좌표를 생성하지 않는다."""
from __future__ import annotations
import copy
import hashlib
from pathlib import Path
import cv2
from learning.artifacts import StorageError,_read,_decode_json
from learning.gripper_camera_dataset import bind_gripper_pair
from learning.pipeline_observations import _times,_bytes
from learning.registered_gripper_crosssection import RegisteredGripperCrosssection
from learning.manual_collection_dataset import require
from learning.m1_shaft_learning import TRANSITION_SCHEMA

SCHEMA='motionbrain.m1_shaft_observations.v1'


def observe_transition(source_root,audited_row,spec,*,artifact_root):
    root=Path(source_root).resolve();assets=Path(artifact_root).resolve()
    result=dict(schemaVersion=SCHEMA,status='rejected',sourceHash=None,perFrame={},transition=None,
        trainingEligible=False,controlEligible=False,recoveryDevelopmentOnly=True,originalOutcomeChanged=False)
    try:
        require(spec['kind']=='registered_gripper_shaft_only','shaft_only_observer_required')
        source_hash=hashlib.sha256(_read(root,'session.json')).hexdigest();result['sourceHash']=source_hash
        require(audited_row.get('sourceHash',source_hash)==source_hash,'audited_source_hash_changed')
        require(audited_row['inputs']['command']['joint']=='gripper','M1_only')
        pair=spec['gripperPairRoot'];_read(assets,pair+'/esp32-before.json')
        row=bind_gripper_pair(copy.deepcopy(audited_row),assets/pair);_times(row)
        registration=_decode_json(_bytes(assets,spec['registrationPath'],spec['registrationSha256']))
        reference=_bytes(assets,spec['referenceImagePath'],spec['referenceImageSha256'])
        tracker=RegisteredGripperCrosssection(registration,reference)
        require(spec['context']==registration['sceneContext'],'registration_scene_context_changed')
        values={}
        for side,key in [('before','inputs'),('after','targets')]:
            frame=row[key]['image'];record=dict(frameId=frame['frameId'],pixelSha256=frame['pixelSha256'],
                status='rejected',perPoint=dict(object=dict(status='not_required')))
            result['perFrame'][side]=record
            try:
                raw=_bytes(assets/pair,'esp32-'+side+'.jpg',frame['imageSha256'])
                record['imageBinding']=dict(path=str(assets/pair/('esp32-'+side+'.jpg')),encoding='jpeg',
                    frameId=frame['frameId'],pixelSha256=frame['pixelSha256'],imageSha256=frame['imageSha256'],
                    width=frame['width'],height=frame['height'])
                observed=tracker.observe_shaft(raw,frame['frameId'])
                require(observed['pixelSha256']==frame['pixelSha256'],'shaft_pixels_changed')
                record.update(status='valid',perPoint=dict(observed['perPoint'],object=dict(status='not_required')),
                    shaftSeparationPx=observed['shaftSeparationPx'],quality=observed['quality'],annotation=observed['annotation'])
                values[side]=observed['shaftSeparationPx']
            except (ValueError,OSError,StorageError,KeyError,TypeError,cv2.error) as exc:
                record['reason']=str(exc);diagnostic=getattr(exc,'diagnostics',{})
                record['diagnostics']=copy.deepcopy(diagnostic)
                for name in ('jawLeft','jawRight'):
                    quality=diagnostic.get('completedQuality',{}).get(name,{})
                    record['perPoint'][name]=dict(status='valid',point=quality['point']) if 'point' in quality else dict(status='unavailable')
        if len(values)!=2:
            result.update(status='partial' if values else 'rejected',reason='complete_shaft_before_and_after_required')
            return result
        context=dict(spec['context'],landmarks=tracker.shaft_landmark_contract,
            cameraIdentity=copy.deepcopy(row['evidence']['cameraIdentity']),
            imageShape=[row['inputs']['image']['height'],row['inputs']['image']['width']],stateFields=['shaftSeparationPx'])
        result['transition']=dict(schemaVersion=TRANSITION_SCHEMA,sessionId=row['sessionId'],captureGroup=spec['captureGroup'],
            context=context,command=copy.deepcopy(row['inputs']['command']),beforeShaftPx=values['before'],afterShaftPx=values['after'],
            episode=[row['evidence']['controllerBootId'],row['evidence']['commandId']],
            frameIds=[row['inputs']['image']['frameId'],row['targets']['image']['frameId']],
            sourceHash=source_hash,recoveryDevelopmentOnly=True)
        result.update(status='eligible',trainingEligible=True)
    except (ValueError,OSError,StorageError,KeyError,TypeError,cv2.error) as exc:result['reason']=str(exc)
    return result
