"""새 모델 좌표만 교체하고 기존 원본·현장 보호 관측은 별도로 유지한다."""
import copy
from pathlib import Path
from hashlib import sha256
from learning.current_full_cycle_source import FullCycleSource,SourcePreflightError
from learning.artifacts import _decode_json,_read
from learning.manual_collection_dataset import require
from learning.scene_relative_observation import SceneRelativeObserver
from learning.scene_relative_models import STATE_FIELDS
from learning.gripper_camera_dataset import read_frame
from learning.development_gripper_source import scene_context
from learning.m1_shaft_scene import scene_context as shaft_scene


class SceneRelativeCycleSource(FullCycleSource):
    """sourceStages의 registration/reference는 보호용, scene*는 모델용이다."""
    def _is_scene(self,kind):
        return self.bank.is_scene_stage(kind)

    def _preflight_model(self,stage):
        kind=stage['kind'];self.bank.verify_cached()
        model=self.bank.models[kind];binding=self.bank.bindings[kind]
        require(sha256(Path(binding['path']).read_bytes()).hexdigest()==binding['sha256'],
                'scene_model_file_changed')
        require(self.bank.hashes[kind]==stage['modelSha256'],'scene_model_stage_hash')
        if self._is_scene(kind):
            require(kind in ('approach','close','release'),'scene_relative_lift_height_unobservable')
            require(stage['context'].get('sceneObservationContract')==model['observationContract']
                and stage['context'].get('object')==model['physicalContext']['object']
                and model['developmentOnly'] is True and model['controlEligible'] is False,
                'scene_model_stage_contract')
        else:
            require(model['context']==stage['context'],'frozen_model_stage_context')
        return model

    def preflight(self,scope,assets):
        self.sceneRegistrations={}
        result=super().preflight(scope,assets)
        missing={}
        for stage in scope['stages']:
            kind=stage['kind']
            if not self._is_scene(kind):continue
            try:
                spec=self.registrations[kind][0];model=self.bank.models[kind]
                observer=SceneRelativeObserver(_decode_json(self._bound(spec['sceneRegistration'])),
                                               self._bound(spec['sceneReference']))
                context={k:spec['sourceContext'][k] for k in ('physicalView','object','poseRegime')}
                context['cameraBootId']=spec['sourceContext']['gripperCameraBootId']
                require(observer.registration['sourceContext']==context,'scene_source_registration_context')
                require(model['observationContract']==dict(canonicalFrameSha256=observer.registration['canonicalFrameSha256'],
                    stateFields=list(STATE_FIELDS)) and spec['physicalContext']==model['physicalContext'],
                    'scene_source_model_contract')
                # Safety has its own source registration. It is never relabelled as the model observer.
                safety=self.registrations[kind][1]
                safety_context=scene_context(safety) if kind=='approach' else shaft_scene(safety)
                require(all(safety_context[k]==context[k] for k in ('physicalView','object','poseRegime')),
                        'scene_safety_registration_context')
                self.sceneRegistrations[kind]=observer
            except (ValueError,KeyError,OSError,RuntimeError) as exc:missing[kind]=[str(exc)]
        if missing:
            self.scope=None
            raise SourcePreflightError(missing)
        result.update(sceneModelSha256={k:self.bank.hashes[k] for k in self.sceneRegistrations},sceneDevelopmentOnly=True)
        return result

    def _validate_safety_state(self,stage,measured,grip,observer,spec):
        if not self._is_scene(stage['kind']):
            return super()._validate_safety_state(stage,measured,grip,observer,spec)
        require(measured.get('sourceFreshMetadataVerified') is True
            and measured.get('cameraBootId')==spec['sourceContext']['gripperCameraBootId'],
            'scene_safety_raw_measurement_required')
        context=scene_context(observer) if stage['kind']=='approach' else shaft_scene(observer)
        require(all(context[k]==spec['sourceContext'][k] for k in ('physicalView','object','poseRegime')),
                'scene_safety_context_changed')

    def _record_context(self,stage):
        if not self._is_scene(stage['kind']):return {}
        model=self.bank.models[stage['kind']]
        return dict(physicalContext=copy.deepcopy(model['physicalContext']),
            observationContract=copy.deepcopy(model['observationContract']),
            sceneRegistrationSha256=self.sceneRegistrations[stage['kind']].registration['registrationSha256'])

    def _audit(self,stage,root,external,gripper,producer_id,angle,*,audit_at=None):
        frames,state,safety=super()._audit(stage,root,external,gripper,producer_id,angle,audit_at=audit_at)
        if not self._is_scene(stage['kind']):return frames,state,safety
        observer=self.sceneRegistrations[stage['kind']]
        location=Path(root)/gripper['root'];label=gripper['label']
        frame,metadata,boot=read_frame(location,label,producer_id)
        raw=_read(location,'esp32-'+label+'.jpg')
        require(sha256(raw).hexdigest()==frame['imageSha256'],'scene_source_image_changed')
        observed=observer.observe(raw,frame['frameId'],observer.registration['sourceContext'])
        require(boot==observer.registration['sourceContext']['cameraBootId'],'scene_source_boot_changed')
        require(_read(location,'esp32-'+label+'.jpg')==raw,'scene_source_image_changed')
        return frames,[*observed['state'],angle],safety
