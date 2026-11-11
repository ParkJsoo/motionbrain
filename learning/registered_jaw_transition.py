"""검증된 단발 원본에서 물체 없는 양턱 전이를 만든다. 장치 접근 없음."""
import copy
import json
from pathlib import Path

from learning.batch_observation import digest, require
from learning.gripper_camera_dataset import bind_gripper_pair
from learning.pipeline_evidence import ingest_session
from learning.registered_gripper_landmarks import RegisteredGripperLandmarks


def build_transition(source, source_sha256, registration, reference, context, capture_group):
    source = Path(source)
    require(isinstance(context,dict) and set(context) == {'viewId','poseRegime','object','load'}
            and all(isinstance(v,str) and v for v in context.values()), 'jaw_context_required')
    require(isinstance(capture_group,str) and bool(capture_group), 'capture_group_required')
    audit = ingest_session(source,source_sha256)
    require(all(audit[k]['status'] == 'verified' for k in ('execution','recording','visual')),
            'verified_jaw_source_required')
    row = audit['visual']['row']
    require(row['inputs']['command']['joint'] == 'gripper', 'jaw_transition_M1_required')
    pair = bind_gripper_pair(row,source/'gripper-pair')
    tracker = RegisteredGripperLandmarks(registration,reference)
    observations = {}
    for side,key in (('before','inputs'),('after','targets')):
        frame=pair[key]['image']
        observed=tracker.observe_jaws((source/'gripper-pair'/('esp32-'+side+'.jpg')).read_bytes(),frame['frameId'])
        require(observed['pixelSha256']==frame['pixelSha256'],'jaw_source_pixels_changed')
        observations[side]=observed
    before,after=observations['before'],observations['after']
    binding=dict(kind='registered_jaw_points',registration=registration,context=context)
    learning_row=dict(sessionId=row['sessionId'],episodeId=':'.join(audit['execution']['episode']),
        sourceHash=source_sha256,frameIds=[row[k]['image']['frameId'] for k in ('inputs','targets')],
        pairedFrameIds=[pair[k]['image']['frameId'] for k in ('inputs','targets')],captureGroup=capture_group,
        joint='gripper',context=dict(context,observerSha256=digest(binding),
            gripperCameraIdentity='gripper:'+pair['evidence']['cameraIdentity']['cameraBootId']),
        stateFields=before['stateFields'],before=before['state'],after=after['state'],
        command=copy.deepcopy(row['inputs']['command']),commandAt=row['evidence']['commandAt'],
        beforePersistedAt=pair['evidence']['gripperFrameEvidence']['before']['persistedAt'])
    return dict(schemaVersion='motionbrain.registered_jaw_transition.v1',status='observed',
        observations=observations,learningRow=learning_row,executionEvidence=audit['execution'],
        motionTrackingVerified=False,measurementAccuracyVerified=False,controlEligible=False,
        trainingPerformed=False,semantics='observed_projected_change_not_proven_physical_motion')
