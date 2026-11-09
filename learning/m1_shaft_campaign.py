"""M1 shaft 자료 보강·고정 후 평가의 명시적 단발 수명주기. 자동 반복·재시도 없음."""
from __future__ import annotations
import copy
import hashlib
import json
from pathlib import Path
import time
from learning.artifacts import ArtifactStore, _read
from learning.manual_collection import ManualCollection, RecordingPort
from learning.pipeline_collection import collect_once
from learning.pipeline_evidence import ingest_session
from learning.grasp_observation import GripperObservationSource
from learning.registered_gripper_crosssection import RegisteredGripperCrosssection
from learning import m1_shaft_learning as learning


def require(ok, why):
    if not ok: raise ValueError(why)


def sha(raw): return hashlib.sha256(raw).hexdigest()
def load(path): return json.loads(Path(path).read_bytes())


class M1ShaftCampaign:
    def __init__(self, root, base, config, token):
        self.root, self.base = Path(root).resolve(), Path(base).resolve()
        self.plan = copy.deepcopy(config); self.plan_sha = sha(learning.canonical_bytes(self.plan))
        self._validate(); self.verify()
        require(not (self.base/'runtime').exists(), 'campaign_runtime_already_exists_no_retry')
        self.store = ArtifactStore(self.base/'runtime')
        self.store.write_json('plan.json', self.plan)
        self.spec = load(self.root/self.plan['observerSpecPath'])
        require(self.spec['kind'] == 'registered_gripper_shaft_only', 'shaft_only_observer_required')
        assets = self.root/'.local'
        registration = _read(assets, self.spec['registrationPath'])
        reference = _read(assets, self.spec['referenceImagePath'])
        require(sha(registration) == self.spec['registrationSha256'] and
                sha(reference) == self.spec['referenceImageSha256'], 'observer_assets_changed')
        self.tracker = RegisteredGripperCrosssection(json.loads(registration), reference)
        self.rows, self.evaluation, self.model = [], [], None
        self.index, self.halted, self.preparation_recovered, self.commands = 1, False, False, 0
        self.completed_commands = 0
        self.previous_models = []
        for previous in self.plan.get('previousModels', []):
            path, digest = previous['path'], previous['sha256']
            require(self.plan['sourceHashes'].get(path) == digest, 'previous_model_source_pin_required')
            relative = Path(path).relative_to('.local')
            raw = _read(self.root/'.local', relative.as_posix())
            model = learning.validate_model(json.loads(raw))
            require(sha(raw) == digest and sha(learning.canonical_bytes(model)) == digest, 'previous_model_hash_changed')
            require(digest not in {p['modelSha256'] for p in self.previous_models}, 'duplicate_previous_model')
            self.previous_models.append(dict(modelSha256=digest, model=model))
        from learning.m1_shaft_observations import observe_transition
        self.observer = observe_transition
        for index, seed in enumerate(self.plan['seeds']):
            source = self.root/seed['sourcePath']; audit = ingest_session(source, seed['sha256'])
            require(audit['visual']['status'] == 'verified', 'seed_raw_rejected')
            spec = dict(self.spec, captureGroup=seed['captureGroup'], gripperPairRoot=seed['pairRoot'])
            observed = self.observer(source, audit['visual']['row'], spec, artifact_root=assets)
            self.store.write_json(f'seed-{index:02d}.json', dict(audit=audit, observation=observed))
            require(observed['status'] == 'eligible', 'seed_observation_rejected')
            self.rows.append(observed['transition'])
        self.store.write_json('excluded-sources.json', self.plan.get('excludedSources', []))
        self.context = learning._rows(self.rows)
        if self.plan['freezeAfterStep'] == 0: self._freeze()
        p = self.plan
        class Port(RecordingPort):
            def _check_status(port, status, expected, pulse=None, baseline_angle=None):
                super()._check_status(status, expected, pulse, baseline_angle)
                require(port.boot_id == p['controllerBootId'], 'controller_boot_changed')
                require(abs(port._shoulder_angle(status)-p['M4BaselineDeg']) <= .5, 'M4_pose_changed')
        value = token() if callable(token) else token
        require(isinstance(value, str) and bool(value), 'memory_credential_required')
        self.manager = ManualCollection(self.base/'sessions', self.root/p['helperPath'], p['controllerUrl'],
            value, p['deviceId'], p['viewId'], sample_fps=8, port_factory=Port)

    def _validate(self):
        p = self.plan; steps = p['steps']; freeze = p['freezeAfterStep']
        require(p.get('automaticRetry') is False and isinstance(steps, list) and 1 <= len(steps) <= 9,
                'bounded_explicit_campaign_required')
        require(type(p['maxNewCommands']) is int and 1 <= p['maxNewCommands'] <= 9 and
                len(steps) <= p['maxNewCommands'], 'command_budget_required')
        require(type(freeze) is int and 0 <= freeze <= len(steps), 'freeze_step_required')
        require(freeze == 0 or steps[freeze-1]['phase'] == 'training', 'freeze_after_training_required')
        require(isinstance(p['seeds'], list) and bool(p['seeds']), 'seed_sources_required')
        preparation = 0; groups = {'training': set(), 'evaluation': set()}
        for index, stage in enumerate(steps, 1):
            command = stage['command']; learning.relative.command_key(command)
            require(command['joint'] == 'gripper' and command['percent'] <= 40 and command['durationMs'] <= 350,
                    'bounded_M1_only')
            phase = stage['phase']; require(phase in ('training', 'preparation', 'evaluation'), 'phase_required')
            require(isinstance(stage['captureGroup'], str) and bool(stage['captureGroup']), 'capture_group_required')
            if phase == 'preparation':
                preparation += 1
                require(command['direction'] == 'open' and command['percent'] == 40 and command['durationMs'] == 350,
                        'preparation_open_only')
            else:
                require(not stage.get('optional', False), 'only_preparation_optional')
                require((phase == 'training' and index <= freeze) or (phase == 'evaluation' and index > freeze),
                        'training_after_freeze_or_evaluation_before_freeze')
                groups[phase].add(stage['captureGroup'])
        require(preparation <= 2, 'preparation_budget_exceeded')
        training = groups['training'] | {s['captureGroup'] for s in p['seeds']}
        require(not training.intersection(groups['evaluation']), 'training_evaluation_group_overlap')

    def verify(self):
        require(sha(learning.canonical_bytes(self.plan)) == self.plan_sha, 'fixed_plan_changed')
        for path, digest in self.plan['sourceHashes'].items():
            require(sha((self.root/path).read_bytes()) == digest, 'source_changed:'+path)

    def state(self):
        return dict(nextIndex=self.index, maximumSteps=len(self.plan['steps']), halted=self.halted,
            trainingRows=len(self.rows), evaluationRows=len(self.evaluation), commandAttempts=self.commands,
            completedCommands=self.completed_commands, uncertainCommands=self.commands-self.completed_commands,
            planSha256=self.plan_sha, modelSha256=None if self.model is None else sha(learning.canonical_bytes(self.model)),
            automaticExecution=False, automaticRetry=False)

    def _freeze(self):
        self.model = learning.fit(self.rows)
        self.store._write('frozen-model.json', learning.canonical_bytes(self.model))

    def factory(self, sid, store):
        return GripperObservationSource(self.plan['cameraUrl'], sid, self.plan['cameraBootId'],
            dict(self.spec['context'], landmarks=self.tracker.shaft_landmark_contract), store, max_captures=2)

    def reviewer(self, stage):
        def review(manager, pair):
            self.verify(); frame = pair['before']['frame']; observation = None; rejection = None
            try:
                observation = self.tracker.observe_shaft(_read(Path(pair['path']), 'esp32-before.jpg'), frame['frameId'])
            except ValueError as exc:
                rejection = str(exc)
            if observation is not None:
                require(observation['pixelSha256'] == frame['pixelSha256'], 'prospective_pixels_changed')
            evidence = dict(phase=stage['phase'], preparedAt=time.monotonic(), command=stage['command'],
                frame=pair['before'], observation=observation, shaftSeparationPx=None if observation is None else observation['state'][0],
                modelSelected=False, modelSha256=None, shaftRidge=None, commandMean=None,
                noChange=None if observation is None else observation['state'][0], strictRejection=rejection, previousModels=[])
            failure = None
            try:
                if stage['phase'] != 'preparation':
                    require(observation is not None, 'prospective_shaft_unobserved')
                    x = observation['state'][0]
                    require(self.plan['shaftSeparationBandPx'][0] <= x <= self.plan['shaftSeparationBandPx'][1], 'shaft_outside_planned_band')
                    if stage['phase'] == 'evaluation':
                        require(self.model is not None, 'frozen_model_required')
                        evidence['modelSha256'] = sha(learning.canonical_bytes(self.model))
                        require(sha((self.store.root/'frozen-model.json').read_bytes()) == evidence['modelSha256'], 'frozen_model_changed')
                        evidence['shaftRidge'] = learning.predict(self.model, x, stage['command'], self.context)
                        evidence['commandMean'] = learning.predict(self.model, x, stage['command'], self.context, baseline=True)
            except ValueError as exc:
                failure = exc; evidence['strictRejection'] = str(exc)
            if stage['phase'] == 'evaluation' and observation is not None:
                for previous in self.previous_models:
                    comparison = dict(modelSha256=previous['modelSha256'], comparisonOnly=True,
                        shaftRidge=None, commandMean=None, unsupportedReason=None)
                    evidence['previousModels'].append(comparison)
                    try:
                        require(sha(learning.canonical_bytes(previous['model'])) == previous['modelSha256'], 'previous_model_memory_changed')
                        comparison['shaftRidge'] = learning.predict(previous['model'], observation['state'][0], stage['command'], self.context)
                        comparison['commandMean'] = learning.predict(previous['model'], observation['state'][0], stage['command'], self.context, baseline=True)
                    except ValueError as exc: comparison['unsupportedReason'] = str(exc)
            manager.store.write_json('prospective.json', evidence)
            if failure is not None: raise failure
        return review

    def step(self, index):
        self.verify(); require(type(index) is int and 1 <= index <= len(self.plan['steps']), 'bounded_step_required')
        saved = self.store.root/f'step-{index:02d}.json'
        if saved.exists(): return load(saved)
        require(index == self.index and not self.halted, 'exact_next_unused_step_required')
        stage = copy.deepcopy(self.plan['steps'][index-1]); result = dict(step=index, phase=stage['phase'])
        if stage['phase'] == 'preparation' and stage.get('optional') and self.preparation_recovered:
            result.update(status='skipped', reason='previous_preparation_after_valid', executed=False)
            self.index += 1; self.store.write_json(saved.name, result); return result
        if stage['phase'] == 'preparation': self.preparation_recovered = False
        previous_current = getattr(self.manager, 'current', None); attempt_charged = False
        def charge_attempt():
            nonlocal attempt_charged
            current = getattr(self.manager, 'current', None)
            if not attempt_charged and current is not None and current is not previous_current and any(
                    event.get('kind') == 'command' for event in current.get('events', [])):
                self.commands += 1; attempt_charged = True
        try:
            require(self.commands < self.plan['maxNewCommands'], 'command_budget_exceeded')
            collected = collect_once(self.manager, f'm1-shaft-{index:02d}', stage['command'], self.plan['poseContext'],
                onsite_confirmed=True, request_root=self.base/'requests', approval_plan_sha256=self.plan_sha,
                gripper_source_factory=self.factory, before_command=self.reviewer(stage))
            result['collection'] = collected
            charge_attempt()
            require(collected['status'] == 'recorded' and collected['offVerified'] is True, 'raw_collection_incomplete_no_retry')
            require(collected['gripperPair']['status'] == 'recorded', 'paired_camera_incomplete_no_retry')
            source = Path(collected['sessionPath']); audit = ingest_session(source, collected['sourceSha256'])
            result['rawAudit'] = audit
            require(audit['visual']['status'] == 'verified', 'raw_audit_rejected')
            require(audit['execution']['status'] == 'verified' and audit['execution']['offVerified'] is True, 'execution_not_verified')
            require(attempt_charged, 'command_intent_missing')
            self.completed_commands += 1
            prospective = load(source/'prospective.json'); result['prospective'] = prospective
            if stage['phase'] == 'preparation':
                try:
                    after = self.tracker.observe_shaft(_read(source/'gripper-pair', 'esp32-after.jpg'), collected['gripperPair']['after']['frame']['frameId'])
                    x = after['state'][0]
                    self.preparation_recovered = self.plan['shaftSeparationBandPx'][0] <= x <= self.plan['shaftSeparationBandPx'][1]
                    result['afterObservation'] = after
                except ValueError as exc:
                    result['afterObservation'] = None; result['observationReason'] = str(exc)
                result.update(status='preparation_recorded', transition=None, trainingEligible=False,
                              preparationRecovered=self.preparation_recovered)
            else:
                spec = dict(self.spec, captureGroup=stage['captureGroup'], gripperPairRoot=str((source/'gripper-pair').relative_to(self.root/'.local')))
                observed = self.observer(source, audit['visual']['row'], spec, artifact_root=self.root/'.local')
                result['observation'] = observed
                require(observed['status'] == 'eligible', 'post_observation_rejected_no_retry')
                row = observed['transition']; result['transition'] = row
                require(self.plan['shaftSeparationBandPx'][0] <= row['afterShaftPx'] <= self.plan['shaftSeparationBandPx'][1], 'post_shaft_outside_planned_band')
                require(row['beforeShaftPx'] == prospective['shaftSeparationPx'], 'prospective_state_disagrees')
                require(prospective['preparedAt'] < audit['visual']['row']['evidence']['commandAt'], 'prediction_not_before_command')
                if stage['phase'] == 'training': self.rows.append(row)
                else:
                    require(prospective['modelSha256'] == sha(learning.canonical_bytes(self.model)), 'evaluation_model_changed')
                    result['evaluation'] = learning.evaluate(self.model, [row]); self.evaluation.append(row)
                    result['prospectiveVerified'] = True
                    result['previousModels'] = []
                    for previous in self.previous_models:
                        comparison = dict(modelSha256=previous['modelSha256'], comparisonOnly=True)
                        result['previousModels'].append(comparison)
                        try:
                            prepared = next(p for p in prospective['previousModels'] if p['modelSha256'] == previous['modelSha256'])
                            require(prepared['unsupportedReason'] is None, 'previous_model_prospective_unsupported:'+str(prepared['unsupportedReason']))
                            require(sha(learning.canonical_bytes(previous['model'])) == previous['modelSha256'], 'previous_model_memory_changed')
                            comparison.update(status='evaluated', evaluation=learning.evaluate(previous['model'], [row]), prospectiveVerified=True)
                        except ValueError as exc: comparison.update(status='rejected', reason=str(exc), prospectiveVerified=False)
                result.update(status='completed', transition=row)
            if index == self.plan['freezeAfterStep']: self._freeze()
            self.index += 1
        except BaseException as exc:
            charge_attempt()
            self.halted = True; result.update(status='halted_no_retry', reason=str(exc), errorType=type(exc).__name__)
            if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                self.store.write_json(saved.name, result); raise
        self.store.write_json(saved.name, result)
        return dict(result, state=self.state())

    def close(self):
        try: self.manager.close()
        finally:
            self.manager.token = ''
            if self.manager.port: self.manager.port.token = ''
