"""기존 Campaign 두 단계와 공식 학습/고정 평가를 연결하는 단일 관절 작업.

기존 양턱 schema와 API를 보존하며 공통 작업은 관절별 독립 모델을 사용한다.

명령은 수집 계획이 선택한다. 모델 자동 제어 승격·실패 재송·표본 자동 보충은 없다.
"""
from __future__ import annotations
import copy
import json
import time
import sys
from types import SimpleNamespace
from pathlib import Path

from learning.batch_campaign import Campaign, validate_plan, require, digest, encoded
from learning.batch_runtime import private_path, sha
from learning.batch_fixed_evaluation import freeze_model, load_freeze
from learning.batch_learning import command_key
from tools.train_batch_campaign import run_manifest

SCHEMA = 'motionbrain.bounded_jaw_job.v1'
JOINT_SCHEMA = 'motionbrain.bounded_joint_job.v1'


def validate_job(config):
    require(isinstance(config, dict) and set(config) == {'schemaVersion','jobId','outputRoot',
        'dispatchLockPath','maxPhysicalCommands','maxSeconds','trainingPlan','evaluationPlan','driver'}, 'jaw_job_schema')
    require(config['schemaVersion'] in (SCHEMA, JOINT_SCHEMA) and isinstance(config['jobId'], str) and config['jobId'], 'jaw_job_identity')
    require(type(config['maxPhysicalCommands']) is int and 0 < config['maxPhysicalCommands'] <= 10000 and
            type(config['maxSeconds']) is int and 0 < config['maxSeconds'] <= 86400, 'jaw_job_budget')
    plans = [config['trainingPlan'], config['evaluationPlan']]
    groups = []
    for phase, plan in zip(('training','evaluation'), plans):
        compiled = validate_plan(plan)
        require(not compiled['rejected'], 'jaw_job_unsupported_conditions')
        require(all(c['phase'] == phase and (config['schemaVersion'] == JOINT_SCHEMA or c['command']['joint'] == 'gripper') for c in plan['conditions']), 'jaw_job_phase_joint')
        groups.append({c['captureGroup'] for c in plan['conditions']})
    require(len({c['command']['joint'] for p in plans for c in p['conditions']}) == 1, 'jaw_job_joint_partition_required')
    require(plans[0]['campaignId'] != plans[1]['campaignId'] and not groups[0] & groups[1], 'jaw_job_phase_identity_overlap')
    require(all(plans[0][k] == plans[1][k] for k in ('controllerBootId','viewId')), 'jaw_job_scene_changed')
    require(sum(p['budgets']['maxPhysicalCommands'] for p in plans) <= config['maxPhysicalCommands'], 'jaw_job_combined_budget')
    require(isinstance(config['driver'], dict), 'jaw_job_driver_config')
    return copy.deepcopy(config)


class _FailClosed:
    """Campaign의 일반 조건 격리 정책보다 좁게: 이 작업은 첫 실패 뒤 정지한다."""
    def __init__(self, driver, phase): self.driver, self.phase = driver, phase
    def __getattr__(self, name): return getattr(self.driver, name)
    def _guard(self, result):
        if result is None: return None
        result = copy.deepcopy(result)
        if not (result.get('execution') == 'completed' and result.get('offVerified') is True and
                result.get('journalClear') is True and result.get('capture') == 'recorded' and
                result.get('observation') == 'eligible' and not result.get('failureScope') and
                (self.phase != 'evaluation' or result.get('prospectiveEvidenceVerified') is True)):
            result['failureScope'] = 'global'
        return result
    def execute(self, *args, **kwargs): return self._guard(self.driver.execute(*args, **kwargs))
    def reconcile(self, *args, **kwargs): return self._guard(self.driver.reconcile(*args, **kwargs))


class JawJob:
    def __init__(self, repository, config, driver_factory, *, clock=time, deadline_wall=None):
        self.repository = Path(repository).resolve()
        self.config = validate_job(config)
        self.root = private_path(self.repository, config['outputRoot'])
        self.dispatch_lock = private_path(self.repository, config['dispatchLockPath'])
        require(self.dispatch_lock == (self.repository/'.local/batch-locks'/(sha(config['trainingPlan']['controllerBootId'].encode())+'.lock')).resolve(),
                'jaw_job_shared_dispatch_lock_required')
        self.root.mkdir(parents=True, exist_ok=True)
        self.factory, self.clock = driver_factory, clock
        self.deadline_wall = deadline_wall
        if 'authorization' in config['driver']:
            auth=self._read_binding(config['driver']['authorization'])
            require(auth.get('maxPhysicalCommands') == config['maxPhysicalCommands'] and
                    private_path(self.repository,auth['outputRoot']) == self.root, 'jaw_job_authorized_total_budget')
            require(private_path(self.repository,auth.get('dispatchLockPath','')) == self.dispatch_lock, 'jaw_job_authorized_dispatch_lock')
        require(self.dispatch_lock not in (self.root/'job.lock',self.root/'campaign.lock'), 'jaw_job_lock_collision')
        self.dispatch_lock.parent.mkdir(parents=True,exist_ok=True)

    def _binding(self, path): return dict(path=str(path), sha256=sha(path.read_bytes()))
    def _read_binding(self, binding):
        path = private_path(self.repository, binding['path'])
        raw = path.read_bytes()
        require(sha(raw) == binding['sha256'], 'jaw_job_artifact_changed')
        return json.loads(raw)
    def _publish(self, path, value):
        raw = encoded(value)
        if path.exists(): require(path.read_bytes() == raw, 'jaw_job_existing_artifact_changed')
        else:
            from learning.batch_review_response import _publish
            _publish(path, raw)
        return self._binding(path)
    def _entries(self, campaign):
        entries=[]
        for raw, in campaign.db.execute("SELECT result FROM attempts ORDER BY rowid"):
            require(raw is not None, 'jaw_job_unresolved_attempt')
            r=json.loads(raw)
            require(r.get('accepted') is True, 'jaw_job_invalid_attempt')
            e=r.get('manifestEntry')
            require(isinstance(e,dict) and (e.get('sourceKind') == 'registered_jaws' or (self.config['schemaVersion'] == JOINT_SCHEMA and set(e) == {'transitionPath','transitionSha256','sourcePath','sourceSha256'})), 'jaw_job_source_entry_required')
            entries.append(e)
        require(bool(entries), 'jaw_job_no_data')
        return entries
    def _campaign(self, phase, frozen=None):
        driver_config=copy.deepcopy(self.config['driver'])
        driver_config.update(plan=copy.deepcopy(self.config[phase+'Plan']),root=str(self.root/phase))
        driver=self.factory(self.repository, driver_config, phase, frozen=frozen)
        return Campaign(self.config[phase+'Plan'], self.root/phase, _FailClosed(driver,phase),
                        lock_path=self.root/'campaign.lock', clock=self.clock)
    def run(self):
        with Campaign._Lock(self.root/'job.lock'), Campaign._Lock(self.dispatch_lock):
            train=self._campaign('training')
            evaluation=None
            try:
                require(train._get('jobHash') in (None,digest(self.config)), 'jaw_job_saved_config_changed')
                if train._get('jobHash') is None:
                    require(train.db.execute('SELECT COUNT(*) FROM attempts').fetchone()[0] == 0, 'jaw_job_unowned_existing_campaign')
                    with train.db:
                        train._set('jobHash',digest(self.config)); train._set('jobPhase','training')
                        train._set('jobDeadline',min(self.clock.time()+self.config['maxSeconds'], self.deadline_wall if self.deadline_wall is not None else float('inf')))
                phase=train._get('jobPhase')
                # A published freeze can be recovered, never issued again at a new time.
                if phase == 'freezing' and (self.root/'freeze.json').exists():
                    binding=self._binding(self.root/'freeze.json')
                    freeze,_=load_freeze(self.repository,binding)
                    require(freeze['model'] == train._get('jobModel') and freeze['frozenAt'] == train._get('jobFreezeAt'),
                            'jaw_job_recovered_freeze_changed')
                    self._read_binding(train._get('jobTrainingReport'))
                    with train.db: train._set('jobFreeze',binding);train._set('jobPhase','evaluation')
                    phase='evaluation'
                # Incomplete artifact operations halt instead of re-fit/re-freeze.
                if phase in ('fitting','freezing','reporting','finishing'):
                    with train.db: train._set('jobPhase','halted');train._set('jobReason','interrupted_'+phase)
                if train._get('jobPhase') in ('halted','completed'):
                    train.run(reconcile_only=True)
                    if (self.root/'evaluation/campaign.sqlite').exists():
                        frozen=dict(model=train._get('jobModel'),freeze=train._get('jobFreeze'))
                        evaluation=self._campaign('evaluation',frozen)
                        evaluation.run(reconcile_only=True)
                    return self._report(train)
                require(self.clock.time() < train._get('jobDeadline'), 'jaw_job_deadline_exhausted')
                if phase == 'training':
                    self._deadline(train,train._get('jobDeadline'))
                    result=train.run()
                    if result['status'] != 'completed': return self._stop(train,'training_'+result['status'])
                    close=getattr(train.driver,'close',None)
                    if callable(close): close()
                    self._phase(train,'fitting')
                    manifest=dict(schemaVersion='motionbrain.batch_training_manifest.v1',training=self._entries(train),evaluation=[])
                    self._publish(self.root/'training-manifest.json',manifest)
                    report=run_manifest(self.root/'training-manifest.json',self.root/'training-output',repository=self.repository)
                    require(len(report['models']) == 1 and not report['models'][0]['deficits'], 'jaw_job_training_support_deficit')
                    model_info=report['models'][0]; model_path=self.root/'training-output'/model_info['path']
                    model_binding=self._binding(model_path); model=self._read_binding(model_binding)
                    require(all(command_key(c['command']) in model['models'] for c in self.config['evaluationPlan']['conditions']),
                            'jaw_job_evaluation_command_unsupported')
                    with train.db: train._set('jobModel',model_binding);train._set('jobTrainingReport',self._binding(self.root/'training-output/report.json'))
                    with train.db:
                        train._set('jobFreezeAt',self.clock.monotonic());train._set('jobPhase','freezing')
                    binding=freeze_model(self.repository,model_binding,self.root/'freeze.json',
                                         clock=SimpleNamespace(monotonic=lambda:train._get('jobFreezeAt')))
                    with train.db: train._set('jobFreeze',binding);train._set('jobPhase','evaluation')
                frozen=dict(model=train._get('jobModel'),freeze=train._get('jobFreeze'))
                self._read_binding(train._get('jobTrainingReport')); self._read_binding(frozen['model'])
                load_freeze(self.repository,frozen['freeze'])
                require(self.clock.time() < train._get('jobDeadline'), 'jaw_job_deadline_exhausted')
                evaluation=self._campaign('evaluation',frozen)
                self._deadline(evaluation,train._get('jobDeadline'))
                result=evaluation.run()
                if result['status'] != 'completed': return self._stop(train,'evaluation_'+result['status'])
                self._phase(train,'reporting')
                manifest=dict(schemaVersion='motionbrain.batch_training_manifest.v1',training=[],evaluation=self._entries(evaluation),
                              frozenModels=[frozen['model']],fixedCommandEvaluation=frozen['freeze'])
                self._publish(self.root/'evaluation-manifest.json',manifest)
                report=run_manifest(self.root/'evaluation-manifest.json',self.root/'evaluation-output',repository=self.repository)
                require(report['prospectiveEvidenceVerified'] and not report['unmatchedEvaluation'], 'jaw_job_evaluation_evidence_failed')
                with train.db: train._set('jobEvaluationReport',self._binding(self.root/'evaluation-output/report.json')); train._set('jobPhase','finishing')
                result=self._report(train); result['status']='completed'
                return result
            except Exception as exc:
                # Persist failure without arbitrary exception text or automatic reattempt.
                reason=str(exc) if isinstance(exc,ValueError) and str(exc).startswith('jaw_job_') and str(exc).replace('_','').isalnum() else type(exc).__name__
                self._stop(train,'job_failure:'+reason)
                raise
            finally:
                primary_exception=sys.exc_info()[0] is not None
                cleanup_failed=False
                for c in (evaluation,train):
                    if c is not None:
                        close=getattr(c.driver,'close',None)
                        try:
                            if callable(close): close()
                        except BaseException:
                            cleanup_failed=True
                try:
                    if cleanup_failed:
                        with train.db:
                            train._set('jobCleanupFailed',True)
                            train._set('jobPhase','halted')
                            if train._get('jobReason') is None: train._set('jobReason','jaw_job_cleanup_failed')
                    elif train._get('jobPhase') == 'finishing':
                        self._phase(train,'completed')
                finally:
                    for c in (evaluation,train):
                        if c is not None: c.close()
                if cleanup_failed and not primary_exception:
                    raise ValueError('jaw_job_cleanup_failed') from None
    def _deadline(self,campaign,deadline):
        with campaign.db:
            old=campaign._get('deadlineWall')
            campaign._set('deadlineWall',min(old if old is not None else self.clock.time()+campaign.plan['budgets']['maxSeconds'],deadline))
    def _phase(self,train,phase):
        with train.db: train._set('jobPhase',phase)
    def _stop(self,train,reason):
        with train.db: train._set('jobPhase','halted'); train._set('jobReason',reason)
        return self._report(train)
    def _report(self,train):
        for key in ('jobModel','jobFreeze','jobTrainingReport','jobEvaluationReport'):
            binding=train._get(key)
            if binding is not None: self._read_binding(binding)
        return dict(schemaVersion=('motionbrain.bounded_joint_job_report.v1' if self.config['schemaVersion'] == JOINT_SCHEMA else 'motionbrain.bounded_jaw_job_report.v1'),jobId=self.config['jobId'],
            status=train._get('jobPhase'),reason=train._get('jobReason'),cleanupFailed=bool(train._get('jobCleanupFailed')),model=train._get('jobModel'),
            freeze=train._get('jobFreeze'),trainingReport=train._get('jobTrainingReport'),
            evaluationReport=train._get('jobEvaluationReport'),automaticRetry=False,
            modelSelected=False,controlEligible=False,physicalSuccessClaimed=False)
