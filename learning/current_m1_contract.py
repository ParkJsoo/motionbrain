"""현재 M1 보강용 관측 계약. 과거 자세·모델·파지 성공을 승계하지 않는다.

기존 shaft95 계측과 외부 고정 보호를 재사용한다. 수집 후보는 CLOSE/OPEN
60%650ms 각 두 번 이내이며, 원장·단발 actor의 더 좁은 한도도 그대로 적용된다.
"""
from __future__ import annotations

from learning.finite_task_cycle import finite
from learning.m1_shaft_scene import (M1ShaftSceneGuard, OBSERVED95_GRASP650_COMMAND_POLICY,
                                   CONTACT_FRAGMENT_POLICY)
from learning.manual_collection_dataset import require

SCHEMA = 'motionbrain.current_M1_collection_guard.v1'
REFERENCE_MEANING = 'current_M1_seed_observation_not_close_ready_contact_or_grasp_success'
COMMANDS = tuple(dict(joint='gripper',direction=direction,percent=60,durationMs=650)
                 for direction in ('close','open'))


class CurrentM1CollectionGuard(M1ShaftSceneGuard):
    configuration_schema = SCHEMA
    reference_meaning = REFERENCE_MEANING

    def __init__(self, *args, **kwargs):
        self._armed_review_sequence=None
        self._command_attempts=dict(close=0,open=0)
        super().__init__(*args,**kwargs)

    def _configuration_fields(self):
        return super()._configuration_fields() | {'referenceM4Deg'}

    def _expected_m4(self):
        return self.config['referenceM4Deg']

    def _validate_reference(self, config):
        super()._validate_reference(config)
        require(finite(config['referenceM4Deg']) and 127.58 < config['referenceM4Deg'] < 296.12
                and config.get('commandPolicy') == OBSERVED95_GRASP650_COMMAND_POLICY
                and config.get('objectObservationPolicy') == CONTACT_FRAGMENT_POLICY,
                'current_M1_explicit_seed_contract')

    def _validate_loaded_reference(self):
        super()._validate_loaded_reference()
        require(self.tracking['maximumM4Difference'] == .25,
                'current_M1_quarter_degree_required')

    def _review_checked(self, record, result, *, deadline, tick):
        value=result.get('shaftSeparationPx') if isinstance(result,dict) else None
        require(finite(value) and 66. <= value <= 139.5, 'current_M1_shaft_fence')
        return super()._review_checked(record,result,deadline=deadline,tick=tick)

    def review_armed(self, record, result, *, expected_sequence, deadline, tick):
        """RUN 제한시간 밖에서 ARMED·큐1·전축OFF와 현재 영상 원본을 감사한다."""
        try:
            require(self._armed_review_sequence is None and type(expected_sequence) is int
                    and expected_sequence > 0, 'current_M1_armed_sequence_required')
            self._armed_review_sequence=expected_sequence
            return self._review_checked(record,result,deadline=deadline,tick=tick)
        except BaseException:
            self.reject()
            raise
        finally:
            self._armed_review_sequence=None

    def _review_status(self):
        if self._armed_review_sequence is None:
            return super()._review_status()
        sequence=self._armed_review_sequence
        pulse=self.port._active_pulse
        require(isinstance(pulse,dict) and any(all(pulse.get(k)==v for k,v in command.items())
                for command in COMMANDS), 'current_M1_armed_command_changed')
        require(self.port.boot_id == self.config['controllerBootId'] and self.port.last_sequence == sequence,
                'current_M1_armed_identity_changed')
        status=self.port._status()
        self.port._check_status(status,'ARMED')
        self.port._queue(1)
        self.port._outputs()
        require(self.port.boot_id == self.config['controllerBootId'] and self.port.last_sequence == sequence
                and self.port.last_outputs == {i:0 for i in range(1,6)},
                'current_M1_armed_output_changed')
        return status

    def validate_command(self, command):
        try:
            require(isinstance(command,dict) and command in COMMANDS
                    and type(command.get('percent')) is int and type(command.get('durationMs')) is int,
                    'current_M1_exact_command_required')
            direction=command['direction']
            require(self._command_attempts[direction] < 2, 'current_M1_direction_budget')
            self._command_attempts[direction]+=1
            super().validate_command(command)
        except BaseException:
            self.reject()
            raise
