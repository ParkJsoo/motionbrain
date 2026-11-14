"""새 정지 영상에서 선언한 M3 실험 계약. 기존 모델 좌표를 이전하지 않는다."""
from learning.approach_external_guard import CurrentSceneExternalGuard
from learning.manual_collection_dataset import require

SCHEMA = 'motionbrain.current_task_M3_guard.v1'
REFERENCE_MEANING = 'current_stationary_collection_reference_not_close_ready_or_success'

class CurrentTaskM3Guard(CurrentSceneExternalGuard):
    """동일 보호 한도에서 현재 M4·열린 턱 폭만 새 기준으로 명시한다.

    새 참조 원본/추적 등록은 부모가 검증한다. 파지·충돌 안전 또는 기존
    학습자료의 전이를 부여하지 않으며 전완·물체 이동 한도는 부모 그대로다.
    """
    configuration_schema = SCHEMA

    def _configuration_fields(self):
        return super()._configuration_fields() | {'referenceM4Deg', 'referenceJawSpanPx', 'objectObservationPolicy'}

    def _validate_reference(self, config):
        require(config['referenceMeaning'] == REFERENCE_MEANING, 'current_task_reference_meaning')
        from learning.m1_shaft_scene import OBJECT_FRAGMENT_POLICY
        require(config['objectObservationPolicy'] == OBJECT_FRAGMENT_POLICY, 'current_task_object_policy')
        for key,lo,hi in [('referenceM4Deg',122.08,301.02),('referenceJawSpanPx',20,180)]:
            value=config[key]
            require(type(value) in (int,float) and lo <= value <= hi,
                    'current_task_reference_range')

    def _expected_m4(self):
        return self.config['referenceM4Deg']

    def _expected_jaw_span(self):
        return self.config['referenceJawSpanPx']

    def _measure_object(self, stats):
        from learning.approach_external_guard import ApproachExternalGuard
        from learning.m1_shaft_scene import M1ShaftSceneGuard
        objects=[s for s in stats[1:] if s[4] >= 500]
        if len(objects)==1:
            return ApproachExternalGuard._measure_object(self, stats)
        # 기존 비접촉 고정물체 2조각 정책의 경계3px/면적비.7..1.05를 그대로 재사용.
        return M1ShaftSceneGuard._measure_object(self, stats)

    def _reference_evidence(self):
        result=super()._reference_evidence()
        result['referenceMeaning']=REFERENCE_MEANING
        return result

    def review_armed(self, record, result, *, expected_sequence, deadline, tick):
        require(not hasattr(self, '_armed_sequence'), 'current_task_armed_review_active')
        require(type(expected_sequence) is int and expected_sequence > 0,
                'current_task_armed_sequence')
        self._armed_sequence=expected_sequence
        try:
            return self.review(record,result,deadline=deadline,tick=tick)
        finally:
            del self._armed_sequence

    def _review_status(self):
        if not hasattr(self, '_armed_sequence'):
            return super()._review_status()
        pulse=self.port._active_pulse
        require(isinstance(pulse,dict) and pulse.get('joint')=='elbow'
                and pulse.get('direction') in ('up','down')
                and pulse.get('percent')==60 and pulse.get('durationMs')==350,
                'current_task_armed_pulse')
        seq=self._armed_sequence
        require(self.port.boot_id==self.config['controllerBootId']
                and self.port.last_sequence==seq,'current_task_armed_identity')
        status=self.port._status()
        self.port._check_status(status,'ARMED')
        self.port._queue(1);self.port._outputs()
        require(self.port.boot_id==self.config['controllerBootId']
                and self.port.last_sequence==seq
                and self.port.last_outputs=={i:0 for i in range(1,6)},
                'current_task_armed_output_changed')
        return status
