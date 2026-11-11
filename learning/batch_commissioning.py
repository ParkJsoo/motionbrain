"""수동 원본의 사후 결합 관측. 실행 전 선택·추적 적격을 소급 주장하지 않는다."""
import copy
import json
from pathlib import Path

from learning.batch_observation import require, validate_spec
from learning.batch_runtime import BatchRuntime, _gripper_spec, private_path, sha
from learning.gripper_camera_dataset import bind_gripper_pair
from learning.pipeline_evidence import ingest_session
from learning.manual_collection_dataset import frame_ref

SCHEMA = 'motionbrain.batch_commissioning_transition.v1'


def build_transition(repository, source, source_sha256, binding, capture_group, phase='training'):
    repository = Path(repository).resolve()
    source = private_path(repository, str(source))
    require(phase in ('training', 'evaluation'), 'commissioning_split')
    require(isinstance(capture_group, str) and bool(capture_group), 'commissioning_capture_group')
    require(isinstance(binding, dict) and set(binding) == {
        'observerSpecPath', 'observerSpecSha256', 'gripper'}, 'commissioning_binding')
    raw = private_path(repository, binding['observerSpecPath']).read_bytes()
    require(sha(raw) == binding['observerSpecSha256'], 'commissioning_observer_hash_changed')
    spec = validate_spec(json.loads(raw))
    audit = ingest_session(source, source_sha256)
    require(all(audit[k]['status'] == 'verified' for k in ('execution', 'recording', 'visual'))
            and audit['execution']['offVerified'] is True and not audit['recoveryDevelopmentOnly'],
            'commissioning_original_not_eligible')
    original = audit['visual']['row']
    camera = audit['recording']['cameraIdentity']
    require(camera['viewId'] == spec['viewId'] and camera['deviceId'] == spec['cameraIdentity'],
            'commissioning_camera_context_changed')
    session_raw = (source/'session.json').read_bytes()
    require(sha(session_raw) == source_sha256, 'commissioning_source_hash_changed')
    session = json.loads(session_raw)
    require(session.get('poseContext') == spec['poseRegime'], 'commissioning_pose_context_changed')
    runtime = object.__new__(BatchRuntime)
    runtime.repository, runtime.spec = repository, spec
    runtime.config = dict(binding, deviceId=spec['cameraIdentity'], poseContext=spec['poseRegime'])
    runtime.plan = dict(viewId=spec['viewId'])
    runtime.clock_id = original['sessionId']
    runtime.gripper_spec = _gripper_spec(repository, runtime.config, spec)
    require(runtime.gripper_spec is not None, 'commissioning_gripper_observer_required')
    joint = audit['execution']['command']['joint']
    require(joint in spec['joints'], 'commissioning_joint_not_registered')
    pair = bind_gripper_pair(original, source/'gripper-pair')
    result = dict(schemaVersion=SCHEMA, kind='developmentReplay', phase=phase,
        sourceSha256=source_sha256, binding=copy.deepcopy(binding), captureGroup=capture_group,
        selectionSemantics='posthoc_audited_default_frames_not_prospective',
        contextEvidence=dict(viewId='audited_camera',poseRegime='original_session_poseContext',
            object='analyst_declared_not_sensor_verified',load='analyst_declared_not_sensor_verified'),
        executionEvidence=audit['execution'], observations={}, status='rejected',
        motionTrackingVerified=False, measurementAccuracyVerified=False, controlEligible=False,
        prospectiveEvidenceVerified=False, deviceAccess=False, motorCommands=0)
    try:
        values = {}
        for side, key in (('before', 'inputs'), ('after', 'targets')):
            ref = original[key]['image']
            frame = next(f for f in session['frames'] if f['frameId'] == ref['frameId'])
            require(frame_ref(frame) == ref, 'commissioning_original_frame_changed')
            external = runtime._observe_native(source/'native-camera', frame, historical=True)
            result['observations'][side] = dict(external=external)
            require(external['observation']['status'] == 'accepted', 'commissioning_external_rejected')
            gripper = runtime._observe_gripper(source/'gripper-pair', side, original['sessionId'], historical=True)
            result['observations'][side]['gripper'] = gripper
            fields, values[side] = runtime._states(external['observation'], gripper, joint)
        result['learningRow'] = dict(sessionId=original['sessionId'],
            episodeId=':'.join(audit['execution']['episode']), sourceHash=source_sha256,
            frameIds=[original[k]['image']['frameId'] for k in ('inputs', 'targets')],
            pairedFrameIds=[pair[k]['image']['frameId'] for k in ('inputs', 'targets')],
            pairedSourceHashes={n:sha((source/'gripper-pair'/n).read_bytes()) for n in (
                'esp32-before.jpg','esp32-before.json','esp32-after.jpg','esp32-after.json')},
            captureGroup=capture_group, joint=joint, context=runtime._context(), stateFields=fields,
            before=values['before'], after=values['after'], command=audit['execution']['command'])
        result['status'] = 'observed'
    except (ValueError, KeyError, StopIteration) as exc:
        result['reason'] = str(exc) or type(exc).__name__
    return result


def load_learning_transition(repository, entry, split):
    require(isinstance(entry, dict) and set(entry) == {'sourceKind','transitionPath',
        'transitionSha256','sourcePath','sourceSha256'} and entry['sourceKind'] == 'commissioning_fused',
        'commissioning_manifest_fields')
    raw = private_path(Path(repository).resolve(), entry['transitionPath']).read_bytes()
    require(sha(raw) == entry['transitionSha256'], 'commissioning_transition_hash_changed')
    stored = json.loads(raw)
    require(stored.get('schemaVersion') == SCHEMA and stored.get('phase') == split,
            'commissioning_transition_schema_or_split')
    rebuilt = build_transition(repository, entry['sourcePath'], entry['sourceSha256'],
                               stored['binding'], stored['captureGroup'], split)
    require(rebuilt == stored, 'commissioning_transition_changed')
    require(rebuilt['status'] == 'observed', 'commissioning_observation_rejected')
    return copy.deepcopy(rebuilt['learningRow']), dict(sourceSha256=entry['sourceSha256'],
        transitionSha256=entry['transitionSha256'], kind='developmentReplay',
        observationRecomputed=True, prospectiveEvidenceVerified=False)
