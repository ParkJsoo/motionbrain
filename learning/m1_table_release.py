"""탁상 복귀 뒤 별도로 등록한 고정 자세의 수동 해제 한 번.

기존 집기 모델·196.17도 구도 계약을 변경하지 않는다. 탁상 지지는 별도
사람 판독·원본 결합을 actor가 확인한다. 이 guard는 힘·기구 안전 판정이 아니다.
"""
from learning.m1_shaft_scene import (M1ShaftSceneGuard, CONTACT_FRAGMENT_POLICY,
                                   OBSERVED95_GRASP650_COMMAND_POLICY)
from learning.manual_collection_dataset import require

SCHEMA = 'motionbrain.current_table_manual_release_guard.v1'
REFERENCE_MEANING = 'onsite_table_supported_manual_release_reference_not_model_or_grasp_success'
COMMAND = dict(joint='gripper', direction='open', percent=60, durationMs=650)
CONTRACT = dict(tableSupportRequired=True, maxCommands=1, automaticRetry=False,
                modelSelected=False, expectedM4=195.55, maximumM4Difference=.25)


class M1TableReleaseGuard(M1ShaftSceneGuard):
    configuration_schema = SCHEMA
    reference_meaning = REFERENCE_MEANING

    def __init__(self, *args, **kwargs):
        self._release_consumed = False
        self._armed_review_sequence = None
        super().__init__(*args, **kwargs)

    def _expected_m4(self):
        return 195.55

    def _configuration_fields(self):
        return super()._configuration_fields() | {'releaseContract'}

    def _validate_reference(self, config):
        super()._validate_reference(config)
        require(config['releaseContract'] == CONTRACT and
                type(config['releaseContract'].get('maxCommands')) is int and
                config['releaseContract'].get('tableSupportRequired') is True and
                config['releaseContract'].get('automaticRetry') is False and
                config['releaseContract'].get('modelSelected') is False and
                config.get('commandPolicy') == OBSERVED95_GRASP650_COMMAND_POLICY and
                config.get('objectObservationPolicy') == CONTACT_FRAGMENT_POLICY,
                'table_release_explicit_contract_required')

    def _validate_loaded_reference(self):
        super()._validate_loaded_reference()
        require(self.tracking['maximumM4Difference'] == .25,
                'table_release_quarter_degree_required')

    def review_armed(self, record, result, *, expected_sequence, deadline, tick):
        """RUN 요청 제한시간 밖의 명시적 ARMED·전축 OFF 관측이다."""
        try:
            require(self._armed_review_sequence is None and type(expected_sequence) is int and
                    expected_sequence > 0, 'table_release_armed_review_sequence')
            self._armed_review_sequence = expected_sequence
            return self._review_checked(record, result, deadline=deadline, tick=tick)
        except BaseException:
            self.reject()
            raise
        finally:
            self._armed_review_sequence = None

    def _review_status(self):
        if self._armed_review_sequence is None:
            return super()._review_status()
        sequence = self._armed_review_sequence
        pulse = self.port._active_pulse
        require(isinstance(pulse, dict) and all(pulse.get(k) == v for k,v in COMMAND.items()),
                'table_release_armed_exact_open_required')
        require(self.port.boot_id == self.config['controllerBootId'] and self.port.last_sequence == sequence,
                'table_release_armed_identity_changed')
        status = self.port._status()
        # 예정 pulse를 넘기지 않는다. 현재 M1을 포함한 전축 OFF를 요구한다.
        self.port._check_status(status, 'ARMED')
        self.port._queue(1)
        self.port._outputs()
        require(self.port.boot_id == self.config['controllerBootId'] and self.port.last_sequence == sequence and
                self.port.last_outputs == {i: 0 for i in range(1, 6)},
                'table_release_armed_output_changed')
        return status

    def validate_command(self, command):
        # 사후 판독 실패·잘못된 명령도 같은 guard로 재시도하지 않는다.
        try:
            require(not self._release_consumed, 'table_release_one_shot_consumed')
            self._release_consumed = True
            require(command == COMMAND and type(command.get('percent')) is int and
                    type(command.get('durationMs')) is int, 'table_release_exact_open_required')
            super().validate_command(command)
        except BaseException:
            self.reject()
            raise
