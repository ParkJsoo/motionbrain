"""검증된 단발 원본에서 물체 없는 양턱 전이를 만든다. 장치 접근 없음."""
import copy
import hashlib
from pathlib import Path

from learning.artifacts import _decode_json
from learning.batch_observation import digest, require
from learning.gripper_camera_dataset import bind_gripper_pair
from learning.pipeline_evidence import ingest_session
from learning.registered_gripper_landmarks import RegisteredGripperLandmarks


def build_transition(source, source_sha256, registration, reference, context, capture_group, *, phase='development'):
    require(phase in ('development', 'training', 'evaluation'), 'jaw_transition_phase')
    source = Path(source)
    require(isinstance(context,dict) and set(context) == {'viewId','poseRegime','object','load'}
            and all(isinstance(v,str) and v for v in context.values()), 'jaw_context_required')
    require(isinstance(capture_group,str) and bool(capture_group), 'capture_group_required')
    audit = ingest_session(source,source_sha256)
    require(all(audit[k]['status'] == 'verified' for k in ('execution','recording','visual')),
            'verified_jaw_source_required')
    require(audit['execution'].get('offVerified') is True
            and audit.get('recoveryDevelopmentOnly') is False, 'jaw_source_off_or_recovery_invalid')
    session_raw = (source/'session.json').read_bytes()
    require(hashlib.sha256(session_raw).hexdigest() == source_sha256, 'jaw_source_hash_changed')
    session = _decode_json(session_raw)
    row = audit['visual']['row']
    require(context['viewId'] == row['evidence'].get('cameraIdentity', {}).get('viewId')
            and context['poseRegime'] == session.get('poseContext')
            and context['object'] == 'none' and context['load'] == 'empty',
            'jaw_original_context_changed')
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
        phase=phase, sourceSha256=source_sha256, context=copy.deepcopy(context),
        captureGroup=capture_group, registrationSha256=registration['registrationSha256'],
        referenceImageSha256=hashlib.sha256(reference).hexdigest(),
        observations=observations,learningRow=learning_row,executionEvidence=audit['execution'],
        motionTrackingVerified=False,measurementAccuracyVerified=False,controlEligible=False,
        trainingPerformed=False,semantics='observed_projected_change_not_proven_physical_motion')


def load_learning_transition(repository, entry, split):
    """명시한 분할의 전이를 원본과 고정 등록 영상에서 다시 계산한다."""
    require(isinstance(entry, dict) and set(entry) == {
        'sourceKind', 'transitionPath', 'transitionSha256', 'sourcePath', 'sourceSha256',
        'registrationPath', 'registrationSha256', 'referenceImagePath',
        'referenceImageSha256', 'captureGroup'}, 'jaw_training_manifest_fields')
    require(entry['sourceKind'] == 'registered_jaws' and split in ('training', 'evaluation'),
            'jaw_training_split_required')
    root = Path(repository).resolve()

    def private(name):
        require(isinstance(name, str) and bool(name), 'jaw_private_path_required')
        path = (root / name).resolve()
        require(path.is_relative_to(root / '.local'), 'jaw_private_path_required')
        return path

    def read(path_key, hash_key):
        raw = private(entry[path_key]).read_bytes()
        require(hashlib.sha256(raw).hexdigest() == entry[hash_key], 'jaw_artifact_hash_changed')
        return raw

    transition = _decode_json(read('transitionPath', 'transitionSha256'))
    require(isinstance(transition, dict), 'jaw_transition_object_required')
    registration = _decode_json(read('registrationPath', 'registrationSha256'))
    reference = read('referenceImagePath', 'referenceImageSha256')
    require(transition.get('phase') == split and transition.get('captureGroup') == entry['captureGroup']
            and transition.get('sourceSha256') == entry['sourceSha256'], 'jaw_manifest_binding_changed')
    rebuilt = build_transition(private(entry['sourcePath']), entry['sourceSha256'],
        registration, reference, transition.get('context'), entry['captureGroup'], phase=split)
    require(transition == rebuilt, 'jaw_transition_recomputed_changed')
    return copy.deepcopy(rebuilt['learningRow']), dict(sourceKind='registered_jaws',
        transitionSha256=entry['transitionSha256'], sourceSha256=entry['sourceSha256'],
        registrationSha256=entry['registrationSha256'],
        referenceImageSha256=entry['referenceImageSha256'], observationRecomputed=True)
