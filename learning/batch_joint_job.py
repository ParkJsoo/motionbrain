"""M1~M5의 독립 관측 문맥을 보존하는 공통 수집·학습·고정 평가 작업.

관절별 기존 Runtime/JawDriver를 재사용한다. 실물 적격을 새로 발급하지 않는다.
"""
import copy
import json
import time
from pathlib import Path
from learning.batch_jaw_job import JawJob, JOINT_SCHEMA, validate_job
from learning.batch_campaign import Campaign, require, digest, encoded
from learning.batch_runtime import BatchRuntime, private_path, sha
from learning.batch_jaw_driver import JawDriver, driver_config_digest, read_binding

SCHEMA='motionbrain.multi_joint_job.v1'


class RuntimeJobDriver:
    """기존 Runtime의 결과를 공식 학습 manifest로 연결하는 얇은 어댑터."""
    _reserve=JawDriver._reserve
    def __init__(self,repository,config,phase,frozen=None):
        self.repository=Path(repository).resolve();self.config=copy.deepcopy(config)
        self.root=private_path(self.repository,config['root']);self.plan=copy.deepcopy(config['plan'])
        self.auth=json.loads(read_binding(self.repository,config['authorization']))
        require(self.auth.get('schemaVersion')=='motionbrain.joint_job_authorization.v1' and
                digest(self.plan) in self.auth.get('planHashes',[]) and
                self.auth.get('driverConfigSha256')==driver_config_digest(config) and
                self.auth.get('controllerBootId')==self.plan['controllerBootId'], 'joint_job_authorization_binding')
        require(self.root.is_relative_to(private_path(self.repository,self.auth['outputRoot'])), 'joint_job_authorized_root')
        require(self.auth.get('onsite')==dict(fixedCamera=True,workspaceClear=True,noEntryDuringJob=True,noncontactEmptyLoad=True),
                'joint_job_onsite_confirmation_required')
        require(type(self.auth.get('maxPhysicalCommands')) is int and self.auth['maxPhysicalCommands']>0 and
                type(self.auth.get('expiresAt')) in (int,float),'joint_job_authorization_bounds')
        joint=self.plan['conditions'][0]['command']['joint']
        if frozen:
            self.plan['runtime']['frozenModels']={joint:frozen['model']}
            self.plan['runtime']['fixedCommandEvaluation']=frozen['freeze']
        # The original session's approvalPlanSha256 includes this job authorization.
        self.plan['runtime']['jointJobAuthorization']=copy.deepcopy(config['authorization'])
        self.plan['runtime']['jointJobPlanSha256']=digest(config['plan'])
        # Credentials remain lazy. Runtime constructor preserves observer/route checks.
        def credential():
            from learning.macos_credentials import load_keychain_secret
            return load_keychain_secret(config['credential']['service'],config['credential']['account'])
        owner=self
        class AuthorizedRuntime(BatchRuntime):
            def _deadline(runtime):
                owner._verify()
                return super()._deadline()
        self.runtime=AuthorizedRuntime(self.plan,self.repository,self.root,credential)
        require(self.runtime.spec['load']=='empty' and self.runtime.spec['object']=='none', 'joint_job_noncontact_empty_only')
        require(all(b.get('route','generic')=='generic' and b.get('interaction')=='noncontact'
                    for b in self.plan['runtime'].get('conditionBindings',{}).values()), 'joint_job_attended_route_not_automatic')
    def _verify(self):
        require(json.loads(read_binding(self.repository,self.config['authorization']))==self.auth,'joint_job_authorization_changed')
        require(time.time()<self.auth['expiresAt'],'joint_job_authorization_expired')
    def condition_support(self,condition): return self.runtime.condition_support(condition)
    def observe(self):
        self._verify();return self.runtime.observe()
    def _entry(self,result):
        if result and result.get('observation')=='eligible':
            path=self.root/'transitions'/(result['transitionId']+'.json')
            require(sha(path.read_bytes())==result['transitionId'],'joint_job_transition_changed')
            evidence=result['evidence']
            result['manifestEntry']=dict(transitionPath=str(path),transitionSha256=result['transitionId'],
                sourcePath=evidence['sessionPath'],sourceSha256=evidence['sourceSha256'])
        return result
    def execute(self,condition,request_id,*,deadline_wall):
        self._verify();self._reserve(request_id)
        return self._entry(self.runtime.execute(condition,request_id,deadline_wall=min(deadline_wall,self.auth['expiresAt'])))
    def reconcile(self,condition,request_id): return self._entry(self.runtime.reconcile(condition,request_id))
    def close(self): self.runtime.close()


def create_driver(repository,config,phase,frozen=None):
    if config.get('adapter')=='registered_jaws':
        from learning.batch_jaw_driver import create_driver as create_jaw
        # adapter is part of the authorization digest, retained deliberately.
        return create_jaw(repository,config,phase,frozen)
    require(config.get('adapter')=='batch_runtime','joint_job_adapter_required')
    return RuntimeJobDriver(repository,config,phase,frozen)


def validate_suite(config):
    require(isinstance(config,dict) and set(config)=={'schemaVersion','suiteId','outputRoot','maxPhysicalCommands','maxSeconds','jobs'}
            and config['schemaVersion']==SCHEMA,'joint_suite_schema')
    require(isinstance(config['suiteId'],str) and config['suiteId'] and isinstance(config['jobs'],list) and config['jobs'],'joint_suite_identity')
    jobs=[validate_job(j) for j in config['jobs']]
    require(all(j['schemaVersion']==JOINT_SCHEMA for j in jobs),'joint_suite_job_schema')
    require(type(config['maxSeconds']) is int and 0<config['maxSeconds']<=86400 and
            type(config['maxPhysicalCommands']) is int and 0<config['maxPhysicalCommands']<=10000,'joint_suite_budget')
    require(sum(j['maxPhysicalCommands'] for j in jobs)<=config['maxPhysicalCommands'],'joint_suite_combined_budget')
    for field in ('jobId','outputRoot'):
        require(len({j[field] for j in jobs})==len(jobs),'joint_suite_duplicate_'+field)
    require(len({j['trainingPlan']['controllerBootId'] for j in jobs})==1,'joint_suite_boot_changed')
    return copy.deepcopy(config)


class JointSuite:
    def __init__(self,repository,config,factory=create_driver,*,clock=time):
        self.repository=Path(repository).resolve();self.config=validate_suite(config);self.clock=clock;self.factory=factory
        self.root=private_path(self.repository,config['outputRoot']);self.root.mkdir(parents=True,exist_ok=True)
        paths=[private_path(self.repository,j['outputRoot']) for j in config['jobs']]
        require(all(p!=self.root and p.is_relative_to(self.root) for p in paths) and
                all(not a.is_relative_to(b) for a in paths for b in paths if a!=b),'joint_suite_disjoint_roots')
    def _save(self,state):
        from learning.experiment_pipeline import atomic
        atomic(self.root/'suite-state.json',encoded(state))
    def preflight(self):
        """전체 단계의 정적 적격을 먼저 확인한다. future 모델만 생성 후 검사한다."""
        for job in self.config['jobs']:
            lock=private_path(self.repository,job['dispatchLockPath'])
            require(lock==(self.repository/'.local/batch-locks'/(sha(job['trainingPlan']['controllerBootId'].encode())+'.lock')).resolve(),'joint_suite_shared_dispatch_lock')
            if 'authorization' in job['driver']:
                auth=json.loads(read_binding(self.repository,job['driver']['authorization']))
                require(auth.get('maxPhysicalCommands')==job['maxPhysicalCommands'] and
                    private_path(self.repository,auth['outputRoot'])==private_path(self.repository,job['outputRoot']) and
                    private_path(self.repository,auth['dispatchLockPath'])==lock,'joint_suite_authorized_job_budget')
            contexts=[]
            for phase in ('training','evaluation'):
                config=copy.deepcopy(job['driver'])
                config.update(plan=copy.deepcopy(job[phase+'Plan']),root=str(private_path(self.repository,job['outputRoot'])/phase))
                # Evaluation structure is checked without inventing a future model.
                driver=self.factory(self.repository,config,'training',frozen=None)
                try:
                    verify=getattr(driver,'_verify',None)
                    if callable(verify):verify()
                    for condition in config['plan']['conditions']:
                        structural=copy.deepcopy(condition);structural['phase']='training'
                        support=getattr(driver,'condition_support',None)
                        require(not callable(support) or support(structural) is None,'joint_suite_condition_not_supported')
                    if isinstance(driver,RuntimeJobDriver):
                        joint=config['plan']['conditions'][0]['command']['joint']
                        from learning.batch_runtime import _model_fields
                        contexts.append((driver.runtime._context(),_model_fields(driver.runtime.spec,driver.runtime.gripper_spec,joint)))
                    elif isinstance(driver,JawDriver):contexts.append(driver.model_context())
                finally:
                    close=getattr(driver,'close',None)
                    if callable(close):close()
            require(not contexts or contexts[0]==contexts[1],'joint_suite_training_evaluation_context_changed')
        return dict(status='static_adapters_checked',deviceAccess=False,futureModelCheck='after_training',physicalValidation=False)

    def run(self):
        with Campaign._Lock(self.root/'suite.lock'):
            p=self.root/'suite-state.json'
            state=json.loads(p.read_bytes()) if p.exists() else dict(configSha256=digest(self.config),status='running',
                deadlineWall=self.clock.time()+self.config['maxSeconds'],results=[],automaticRetry=False,physicalValidation=False)
            require(state['configSha256']==digest(self.config),'joint_suite_config_changed')
            if state['status']!='running':return state
            self._save(state)
            try:
                self.preflight()
                for index in range(len(state['results']),len(self.config['jobs'])):
                    require(self.clock.time()<state['deadlineWall'],'joint_suite_deadline')
                    job=JawJob(self.repository,self.config['jobs'][index],self.factory,clock=self.clock,deadline_wall=state['deadlineWall'])
                    result=job.run();state['results'].append(result)
                    if result['status']!='completed':state['status']='halted'
                    self._save(state)
                    if state['status']=='halted':return state
                state['status']='completed';self._save(state);return state
            except BaseException:
                state['status']='halted';state['reason']='job_interrupted_or_failed';self._save(state);raise
