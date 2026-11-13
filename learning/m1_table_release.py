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
