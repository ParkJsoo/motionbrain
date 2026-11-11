"""전체 관절의 유한 수집 계획과 영속 진행. import·계획 검사는 장치에 접근하지 않는다.

수집 순서 선택은 모델 선택 제어 성과가 아니다. driver는 검증된 단발 실행층이며
예약·실행·OFF·원본·관측을 구분한다. 미완료 예약은 재전송하지 않는다.
"""
from __future__ import annotations

import copy
import fcntl
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import time

from learning.supervised_collection import PROFILES

SCHEMA = 'motionbrain.batch_campaign.v1'
JOINTS = tuple(PROFILES)
_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z')


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False,
                      allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(encoded(value)).hexdigest()


def _positive(value, maximum=100000):
    return type(value) is int and 0 < value <= maximum


def _id(value):
    return isinstance(value, str) and _ID.fullmatch(value) is not None


def validate_plan(plan):
    """선정한 모든 조건을 검사하되 불량 조건은 격리한다. 실제 기기 준비의 증명은 아님."""
    require(isinstance(plan, dict) and plan.get('schemaVersion') == SCHEMA, 'batch_plan_schema')
    require(_id(plan.get('campaignId')), 'batch_campaign_id')
    require(_id(plan.get('viewId')), 'batch_view_id')
    require(isinstance(plan.get('controllerBootId'), str) and bool(plan['controllerBootId']), 'batch_boot_id')
    age = plan.get('maxObservationAgeSeconds')
    require(type(age) in (float, int) and math.isfinite(age) and 0 < age <= 5, 'batch_observation_age')
    nodes = plan.get('nodes')
    require(isinstance(nodes, list) and bool(nodes) and all(_id(n) for n in nodes)
            and len(set(nodes)) == len(nodes), 'batch_nodes')
    budget = plan.get('budgets', {})
    require(isinstance(budget, dict) and set(budget) == {
        'maxPhysicalCommands', 'maxPreparationCommands', 'maxAttempts', 'maxSeconds'}, 'batch_budgets')
    require(all(_positive(v) for v in budget.values())
            and budget['maxPreparationCommands'] <= budget['maxPhysicalCommands'], 'batch_budget_values')
    conditions = plan.get('conditions')
    require(isinstance(conditions, list) and 0 < len(conditions) <= 10000, 'batch_conditions')
    # Identity collisions are a plan-wide fault: their durable request keys cannot be disambiguated.
    ids = [c.get('id') if isinstance(c, dict) else None for c in conditions]
    require(all(_id(i) for i in ids) and len(set(ids)) == len(ids), 'batch_unique_condition_ids')
    ready, rejected = [], []
    phases = {}
    for c in conditions:
        try:
            require(set(c) == {'id', 'command', 'fromNodes', 'phase', 'captureGroup', 'target', 'maxAttempts'},
                    'batch_condition_fields')
            command = c['command']
            require(isinstance(command, dict) and set(command) == {'joint', 'direction', 'percent', 'durationMs'},
                    'batch_command_fields')
            require(command['joint'] in PROFILES, 'batch_joint_unsupported')
            limits = PROFILES[command['joint']]['directions'].get(command['direction'])
            if limits is None:
                binding = plan.get('runtime', {}).get('conditionBindings', {}).get(c['id'], {})
                require(isinstance(binding, dict) and binding.get('route') == 'm4_up', 'batch_direction_unsupported')
                from learning.batch_m4 import validate_route_command
                validate_route_command(command, binding)
                limits = (0, command['percent'], command['durationMs'])
            require(type(command['percent']) is int and 1 <= command['percent'] <= limits[1]
                    and type(command['durationMs']) is int and 100 <= command['durationMs'] <= limits[2],
                    'batch_command_limits')
            require(c['phase'] in ('training', 'preparation', 'evaluation'), 'batch_phase')
            require(_id(c['captureGroup']), 'batch_capture_group')
            require(_positive(c['target']) and _positive(c['maxAttempts'])
                    and c['target'] <= c['maxAttempts'], 'batch_condition_budget')
            require(isinstance(c['fromNodes'], list) and bool(c['fromNodes'])
                    and len(set(c['fromNodes'])) == len(c['fromNodes'])
                    and all(n in nodes for n in c['fromNodes']), 'batch_condition_nodes')
            phases.setdefault(c['captureGroup'], set()).add(c['phase'])
            ready.append(copy.deepcopy(c))
        except (ValueError, TypeError, KeyError) as exc:
            rejected.append(dict(conditionId=c['id'], reason=str(exc)))
    mixed = {g for g, values in phases.items() if 'training' in values and 'evaluation' in values}
    require(not mixed, 'batch_train_evaluation_group_overlap')
    return dict(schemaVersion=SCHEMA, planSha256=digest(plan), ready=ready,
                rejected=rejected, supportedJoints=list(JOINTS), deviceAccess=False)


class Campaign:
    """단일 실행자. 같은 제어기의 모든 캠페인은 lock_path를 공유한다."""
    def __init__(self, plan, root, driver, *, lock_path, clock=time):
        self.plan = copy.deepcopy(plan)
        self.compiled = validate_plan(self.plan)
        support = getattr(driver, 'condition_support', None)
        if callable(support):
            ready = []
            for condition in self.compiled['ready']:
                reason = support(copy.deepcopy(condition))
                require(reason is None or isinstance(reason, str), 'batch_runtime_support_result')
                if reason is not None:
                    self.compiled['rejected'].append(dict(conditionId=condition['id'], reason=reason))
                else:
                    ready.append(condition)
            self.compiled['ready'] = ready
        self.root, self.driver, self.clock = Path(root), driver, clock
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock_path = Path(lock_path)
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        self.db = None
        self._active_timing = None
        with self._locked():
            self.db = sqlite3.connect(self.root/'campaign.sqlite', timeout=2)
            self.db.execute('PRAGMA synchronous=FULL')
            self.db.executescript('''
              CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
              CREATE TABLE IF NOT EXISTS attempts (
                request_id TEXT PRIMARY KEY, condition_id TEXT NOT NULL, ordinal INTEGER NOT NULL,
                phase TEXT NOT NULL, status TEXT NOT NULL, result TEXT,
                UNIQUE(condition_id, ordinal));
              CREATE TABLE IF NOT EXISTS blocked (condition_id TEXT PRIMARY KEY, reason TEXT NOT NULL);
              CREATE TABLE IF NOT EXISTS transitions (identity TEXT PRIMARY KEY, request_id TEXT UNIQUE NOT NULL);
              CREATE TABLE IF NOT EXISTS data_keys (kind TEXT NOT NULL, value TEXT NOT NULL,
                request_id TEXT NOT NULL, PRIMARY KEY(kind,value));
            ''')
            prior = self._get('planHash')
            require(prior is None or prior == self.compiled['planSha256'], 'batch_saved_plan_changed')
            self.support_changed = prior is not None and self._get('compiledHash') != digest(self.compiled)
            with self.db:
                if prior is None:
                    self._set('planHash', self.compiled['planSha256'])
                    self._set('elapsedSeconds', 0.)
                    self._set('halted', None)
                    self._set('plan', self.plan)
                    self._set('compiledHash', digest(self.compiled))
                    self._set('deadlineWall', None)

    class _Lock:
        def __init__(self, path): self.path, self.file = path, None
        def __enter__(self):
            self.file = self.path.open('a+')
            try:
                fcntl.flock(self.file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                self.file.close()
                raise ValueError('batch_dispatcher_already_running') from None
            return self
        def __exit__(self, *_):
            fcntl.flock(self.file.fileno(), fcntl.LOCK_UN)
            self.file.close()

    def _locked(self): return self._Lock(self.lock_path)

    def _get(self, key):
        row = self.db.execute('SELECT value FROM meta WHERE key=?', (key,)).fetchone()
        return None if row is None else json.loads(row[0])

    def _set(self, key, value):
        self.db.execute('INSERT OR REPLACE INTO meta VALUES (?,?)', (key, encoded(value).decode()))

    def _block(self, condition, reason):
        self.db.execute('INSERT OR REPLACE INTO blocked VALUES (?,?)', (condition, reason))

    def _halt(self, reason):
        with self.db: self._set('halted', reason)

    def _counts(self):
        rows = self.db.execute('SELECT condition_id,phase,status,result FROM attempts').fetchall()
        counts = {c['id']:dict(attempts=0, eligible=0) for c in self.plan['conditions']}
        totals = dict(reservedCommands=len(rows), preparationCommands=0,
                      completedExecutions=0, notDispatched=0, unknownExecutions=0, validTransitions=0)
        for cid, phase, status, raw in rows:
            counts[cid]['attempts'] += 1
            totals['preparationCommands'] += phase == 'preparation'
            if status == 'reserved':
                totals['unknownExecutions'] += 1
                continue
            r = json.loads(raw)
            totals[{'completed':'completedExecutions', 'not_dispatched':'notDispatched', 'unknown':'unknownExecutions'}[r['execution']]] += 1
            if r.get('accepted'):
                counts[cid]['eligible'] += 1
                totals['validTransitions'] += phase != 'preparation'
        return counts, totals

    def report(self, status=None):
        counts, totals = self._counts()
        blocked = dict(self.db.execute('SELECT condition_id,reason FROM blocked'))
        conditions = [dict(conditionId=c['id'], target=c['target'], phase=c['phase'],
                           **counts[c['id']], blockedReason=blocked.get(c['id'])) for c in self.compiled['ready']]
        elapsed = self._get('elapsedSeconds')
        if self._active_timing is not None:
            base, started = self._active_timing
            elapsed = max(elapsed, base+max(0., self.clock.monotonic()-started))
        coverage = {joint:dict(target=sum(c['target'] for c in self.compiled['ready']
                                        if c['command']['joint'] == joint and c['phase'] != 'preparation'),
                              eligible=sum(counts[c['id']]['eligible'] for c in self.compiled['ready']
                                           if c['command']['joint'] == joint and c['phase'] != 'preparation'),
                              readyConditions=sum(c['command']['joint'] == joint for c in self.compiled['ready']))
                    for joint in JOINTS}
        return dict(schemaVersion='motionbrain.batch_campaign_report.v1',
                    campaignId=self.plan['campaignId'], planSha256=self.compiled['planSha256'],
                    status=status or ('halted' if self._get('halted') else
                                      'needs_reconciliation' if totals['unknownExecutions'] else 'ready'),
                    haltedReason=self._get('halted'), elapsedSeconds=elapsed, jointCoverage=coverage,
                    conditions=conditions, rejectedConditions=self.compiled['rejected'], runtimeSupportChanged=self.support_changed, **totals,
                    automaticRetry=False, modelSelected=False, physicalSuccessClaimed=False)

    def _finish(self, condition, request, result):
        require(isinstance(result, dict), 'batch_driver_result')
        r = copy.deepcopy(result)
        require(r.get('execution') in ('completed', 'not_dispatched', 'unknown'), 'batch_execution_outcome')
        require(type(r.get('offVerified')) is bool and type(r.get('journalClear')) is bool, 'batch_output_proof')
        require(r.get('capture') in ('recorded', 'partial', 'failed')
                and r.get('observation') in ('eligible', 'rejected'), 'batch_data_outcome')
        require(r.get('failureScope') in (None,'condition','joint','global'), 'batch_failure_scope')
        if r.get('failureScope') == 'joint':
            require(r.get('blockedJoint') in JOINTS and r['blockedJoint'] == condition['command']['joint'],
                    'batch_failed_joint_binding')
        clean = r['offVerified'] and r['journalClear']
        eligible = (r.get('failureScope') not in ('joint','global')
                    and r['execution'] == 'completed' and clean and r['capture'] == 'recorded'
                    and r['observation'] == 'eligible')
        if condition['phase'] == 'evaluation':
            eligible = eligible and r.get('prospectiveEvidenceVerified') is True
        # Preparation is a completed move, never a training transition.
        accepted = eligible if condition['phase'] != 'preparation' else (
            r.get('failureScope') not in ('joint','global') and r['execution'] == 'completed' and clean)
        identity = r.get('transitionId')
        if accepted and condition['phase'] != 'preparation':
            require(isinstance(identity, str) and re.fullmatch(r'[a-f0-9]{64}', identity), 'batch_transition_identity')
        keys = []
        if accepted and condition['phase'] != 'preparation':
            data = r.get('dataIdentity')
            require(isinstance(data, dict) and {'sourceHash','sessionId','episodeId','frameIds'} <= set(data)
                    and set(data) <= {'sourceHash','sessionId','episodeId','frameIds','pairedFrameIds','supplementalFrameIds'},
                    'batch_data_identity_required')
            require(isinstance(data['sourceHash'], str) and re.fullmatch(r'[a-f0-9]{64}', data['sourceHash']),
                    'batch_source_hash_required')
            require(all(isinstance(data[k], str) and bool(data[k]) for k in ('sessionId','episodeId')),
                    'batch_episode_identity_required')
            refs = data['frameIds']
            require(isinstance(refs, list) and len(refs) == 2 and all(isinstance(f,str) and f for f in refs)
                    and refs[0] != refs[1], 'batch_frame_identity_required')
            paired = data.get('pairedFrameIds', [])
            require(isinstance(paired, list) and len(paired) in (0,2)
                    and all(isinstance(f,str) and f for f in paired)
                    and len(set(refs+paired)) == len(refs)+len(paired), 'batch_paired_frame_identity_required')
            supplemental = data.get('supplementalFrameIds', [])
            require(isinstance(supplemental, list) and len(supplemental) <= 128
                    and all(isinstance(f,str) and f for f in supplemental)
                    and len(set(refs+paired+supplemental)) == len(refs)+len(paired)+len(supplemental),
                    'batch_supplemental_frame_identity_required')
            keys = [(k,data[k]) for k in ('sourceHash','sessionId','episodeId')]+[('frameId',f) for f in refs+paired+supplemental]
        with self.db:
            if accepted and condition['phase'] != 'preparation':
                if any(self.db.execute('SELECT 1 FROM data_keys WHERE kind=? AND value=?', key).fetchone()
                       for key in keys):
                    accepted = False
                    r['reason'] = 'batch_reused_source_episode_or_frame'
            if accepted and condition['phase'] != 'preparation':
                try:
                    self.db.execute('INSERT INTO transitions VALUES (?,?)', (identity, request))
                    self.db.executemany('INSERT INTO data_keys VALUES (?,?,?)', [(k,v,request) for k,v in keys])
                except sqlite3.IntegrityError:
                    accepted = False
                    r['reason'] = 'batch_duplicate_transition'
            r['accepted'] = bool(accepted)
            self.db.execute('UPDATE attempts SET status=?,result=? WHERE request_id=? AND status=?',
                            ('finished', encoded(r).decode(), request, 'reserved'))
            if r.get('followupStopReason'):
                self._set('halted', 'batch_observed_response_outside_collection_domain')
            if r['execution'] == 'unknown' or not clean:
                self._set('halted', 'batch_unresolved_execution_or_output')
            if r.get('failureScope') == 'global':
                self._set('halted', 'batch_global_execution_fault')
            if r.get('failureScope') == 'joint':
                for candidate in self.plan['conditions']:
                    command = candidate.get('command')
                    if isinstance(command, dict) and command.get('joint') == r['blockedJoint']:
                        self._block(candidate['id'], 'batch_joint_quarantined:'+str(r.get('failureReason','execution_guard')))
            if not accepted:
                self._block(condition['id'], r.get('reason', 'batch_condition_rejected'))

    def _fresh(self, observation):
        require(isinstance(observation, dict), 'batch_observation_invalid')
        require(observation.get('bootId') == self.plan['controllerBootId'], 'batch_boot_changed')
        require(observation.get('viewId') == self.plan['viewId'], 'batch_view_changed')
        require(observation.get('offVerified') is True and observation.get('journalClear') is True,
                'batch_current_output_unverified')
        stamp = observation.get('observedAt')
        require(type(stamp) in (float, int) and math.isfinite(stamp)
                and 0 <= self.clock.monotonic()-stamp <= self.plan['maxObservationAgeSeconds'], 'batch_stale_observation')
        nodes = observation.get('nodes', [observation.get('node')])
        require(isinstance(nodes, list) and bool(nodes) and len(set(nodes)) == len(nodes)
                and all(n in self.plan['nodes'] for n in nodes), 'batch_current_node_unknown')
        return set(nodes)

    def run(self, *, reconcile_only=False):
        require(type(reconcile_only) is bool, 'batch_reconcile_flag')
        with self._locked():
            started = self.clock.monotonic()
            base_elapsed = self._get('elapsedSeconds')
            self._active_timing = (base_elapsed, started)
            now_wall = self.clock.time()
            with self.db:
                if self._get('deadlineWall') is None:
                    self._set('deadlineWall', now_wall+self.plan['budgets']['maxSeconds'])
            try:
                if self._get('halted'):
                    return self.report('halted')
                original_conditions = {c['id']:c for c in self.plan['conditions']}
                conditions = {c['id']:c for c in self.compiled['ready']}
                # Reconcile immutable old evidence only; never call execute for a reserved slot.
                for request, cid in self.db.execute("SELECT request_id,condition_id FROM attempts WHERE status='reserved'").fetchall():
                    resolver = getattr(self.driver, 'reconcile', None)
                    try:
                        result = resolver(copy.deepcopy(original_conditions[cid]), request) if callable(resolver) else None
                    except Exception:
                        self._halt('batch_reconciliation_unavailable')
                        return self.report('halted')
                    if result is None:
                        self._halt('batch_reserved_execution_unresolved')
                        return self.report('halted')
                    try:
                        self._finish(original_conditions[cid], request, result)
                    except Exception:
                        self._halt('batch_reconciliation_invalid')
                        return self.report('halted')
                if self._get('halted'):
                    return self.report('halted')
                if reconcile_only:
                    return self.report('reconciled')
                if self.support_changed:
                    return self.report('runtime_support_changed')
                while True:
                    counts, totals = self._counts()
                    remaining = [c for c in conditions.values() if counts[c['id']]['eligible'] < c['target']
                                 and counts[c['id']]['attempts'] < c['maxAttempts']
                                 and not self.db.execute('SELECT 1 FROM blocked WHERE condition_id=?', (c['id'],)).fetchone()]
                    if not remaining:
                        complete = all(counts[c['id']]['eligible'] >= c['target'] for c in conditions.values())
                        return self.report('completed' if complete and not self.compiled['rejected'] else 'partial')
                    b = self.plan['budgets']
                    elapsed = base_elapsed + self.clock.monotonic()-started
                    if (totals['reservedCommands'] >= min(b['maxPhysicalCommands'], b['maxAttempts'])
                            or elapsed >= b['maxSeconds'] or self.clock.time() >= self._get('deadlineWall')):
                        return self.report('budget_exhausted')
                    try:
                        nodes = self._fresh(self.driver.observe())
                    except Exception as exc:
                        # No dispatch occurred; fresh read can be attempted on a later explicit run.
                        return dict(self.report('awaiting_observation'), observationReason=type(exc).__name__+':'+str(exc))
                    reachable = [c for c in remaining if nodes.intersection(c['fromNodes'])
                                 and (c['phase'] != 'preparation' or totals['preparationCommands'] < b['maxPreparationCommands'])]
                    if not reachable:
                        return self.report('no_reachable_conditions')
                    # Fill coverage fairly; fixed order breaks ties, preparation never bypasses budgets.
                    condition = min(reachable, key=lambda c:(counts[c['id']]['eligible']/c['target'],
                                                             counts[c['id']]['attempts'], list(conditions).index(c['id'])))
                    # observe() time is charged too. It cannot dispatch after the wall-time budget.
                    if (base_elapsed+self.clock.monotonic()-started >= b['maxSeconds']
                            or self.clock.time() >= self._get('deadlineWall')):
                        return self.report('budget_exhausted')
                    ordinal = counts[condition['id']]['attempts']+1
                    request = 'batch-'+hashlib.sha256(encoded([self.plan['campaignId'], condition['id'], ordinal])).hexdigest()
                    with self.db:
                        self.db.execute('INSERT INTO attempts VALUES (?,?,?,?,?,NULL)',
                                        (request, condition['id'], ordinal, condition['phase'], 'reserved'))
                        self._set('elapsedSeconds', base_elapsed+self.clock.monotonic()-started)
                    try:
                        result = self.driver.execute(copy.deepcopy(condition), request,
                                                     deadline_wall=self._get('deadlineWall'))
                        self._finish(condition, request, result)
                    except BaseException:
                        # Durable reservation remains. No subsequent condition may execute.
                        # On resume only reconcile() may classify this old request.
                        # A missing/unknown durable result globally blocks the campaign.
                        raise
                    if self._get('halted'):
                        return self.report('halted')
            finally:
                with self.db:
                    self._set('elapsedSeconds', base_elapsed+max(0., self.clock.monotonic()-started))
                self._active_timing = None

    def close(self):
        if self.db is not None:
            self.db.close()
            self.db = None
