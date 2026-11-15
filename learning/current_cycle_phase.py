"""현재 두 원본에 결합된 사람의 단계 판독. 명령이나 방향은 선택하지 않는다.

새 요청을 nonce·원본 SHA·기한에 결합해 발행한다. 늦게 판독한 정지 seed는
명시된 양성 anchor→새 원본 한 번 프로토콜만 사용하며 원본 시각은 보존한다.
"""
import copy
import secrets

from learning.artifacts import _read, _decode_json
from learning.finite_task_cycle import STAGES
from learning.manual_collection_dataset import require
from learning.supervised_collection import plan_digest
from learning.artifacts import StorageError
from learning.current_cycle_observation import StationaryReviewAnchor,ANCHOR_SCHEMA

SCHEMA = 'motionbrain.current_cycle_human_phase.v1'
ENTRY = {'approach': ('noncontact',), 'close': ('objectBetweenJaws','depthVerified'),
         'lift': ('bilateralContact',), 'lower': ('bilateralContact','airborne'),
         'release': ('tableSupported',)}
GOAL = {'approach': ('noncontact','clearance'), 'close': ('bilateralContact',),
        'lift': ('airborne','retained'), 'lower': ('tableSupported',),
        'release': ('tableSupported','bothJawsClear')}


def source_record(record):
    return {k: copy.deepcopy(v) for k,v in record.items() if k not in ('entry','supportMode')}


class CurrentCyclePhase:
    def __init__(self, review_root, *, audit_observation, clock, request_store=None, max_review_seconds=60.):
        require(callable(audit_observation), 'phase_raw_auditor_required')
        self.root, self.audit, self.clock = review_root, audit_observation, clock
        self.scope = None
        require(type(max_review_seconds) in (int,float) and 0<max_review_seconds<=60.,'phase_bounded_wait')
        self.requests=request_store;self.wait_seconds=max_review_seconds;self.source=None
        self.accepted={};self.anchors={}
        self.retention={}

    def bind_source(self,source):
        require(self.source is None or self.source is source,'phase_source_owner_changed')
        self.source=source

    def preflight(self, scope, assets):
        self.scope = None
        require(assets.get('phaseMode') == 'source_bound_human', 'phase_explicit_human_mode_required')
        from pathlib import Path
        require(Path(self.root).is_dir() and not Path(self.root).is_symlink(), 'phase_review_directory_required')
        require(callable(getattr(self.requests,'write_json',None)) and self.source is not None
                and assets.get('phaseProtocol')=='nonce_seed_anchor_fresh_once_v1',
                'phase_actual_request_and_stationary_source_required')
        require(self.source.scope==scope and set(self.source.stationary)=={s['kind'] for s in scope['stages']},
                'phase_verified_source_stages_required')
        self.scope=copy.deepcopy(scope)
        return dict(scopeSha256=plan_digest(scope), assetsSha256=plan_digest(assets),
                    supportedStages=[s['kind'] for s in scope['stages']], humanAssisted=True)

    def _review(self, stage, record):
        require(self.scope is not None, 'phase_preflight_required')
        key = plan_digest(source_record(record))
        accepted=self.accepted.get(key)
        require(accepted is not None,'phase_source_not_reviewed')
        review,ref,evidence=accepted
        raw = _read(self.root,ref['path'])
        from hashlib import sha256
        require(sha256(raw).hexdigest()==ref['sha256'],'phase_response_changed')
        require(review.get('schemaVersion') == SCHEMA
                and review.get('stage') == stage['kind'] and review.get('humanAssisted') is True
                and review.get('method') == 'explicit_dual_source_visual_review'
                and type(review.get('reached')) is bool and review.get('stable') is True,
                'phase_current_raw_review_required')
        labels = review.get('labels', {})
        require(isinstance(labels, dict) and all(type(v) is bool for v in labels.values()),
                'phase_boolean_labels_required')
        return review, plan_digest(dict(responseSha256=ref['sha256'],binding=evidence))

    def _transfer(self,stage,anchor_record,review,current,deadline,tick):
        anchor_pair=self.source.stationary_pair(stage,anchor_record)
        current_pair=self.source.stationary_pair(stage,current)
        proof=self.source.zero_output_bridge(anchor_record,current)
        current_pair['outputProofObservedAt']=proof['observedAt']
        labels=review['labels']
        anchor=dict(schemaVersion=ANCHOR_SCHEMA,anchorId=plan_digest(source_record(anchor_record)),
            pair=anchor_pair,review=dict(context=anchor_pair['context'],stable=True,
                onsite=dict(operatorPresent=labels.get('operatorPresent'),clearanceVerified=labels.get('clearance'),
                            jawContactVerified=labels.get('bilateralContact',False),tableSupportVerified=labels.get('tableSupported',False))),
            tolerances=dict(externalObjectBox=1.5,externalToolCenter=1.5,gripperObjectBox=1.5,
                            gripperReferencePoints=1.5,M4AngleDeg=.1),maxFrameAgeSeconds=3.)
        check=StationaryReviewAnchor(anchor,audit_pair=self.source.audit_stationary_pair,
                                    audit_cursor=self.source.audit_zero_bridge,clock=self.clock)
        _,evidence=check.review(current_pair,proof,deadline=deadline,tick=tick)
        return evidence

    def resolve(self,stage,record,*,deadline,tick):
        """새 seed 판독→양성 anchor→새 pair 최대 한 번. ARM에서는 대기/재촬영 없다."""
        require(self.scope is not None and callable(tick) and self.clock.monotonic()<deadline,
                'phase_protocol_deadline')
        key=plan_digest(source_record(record));cached=self.anchors.get(stage['kind'])
        if cached and cached[0].get('outputSequence')==record.get('outputSequence'):
            old,review,ref=cached
            from hashlib import sha256
            require(sha256(_read(self.root,ref['path'])).hexdigest()==ref['sha256'], 'phase_response_changed')
            evidence=self._transfer(stage,old,review,record,deadline,tick)
            seed=old
        else:
            require(record.get('sourceRoute') in ('idle','general_after'), 'phase_ARM_anchor_missing')
            nonce=secrets.token_hex(16);started=self.clock.monotonic()
            expires=min(deadline,started+self.wait_seconds)
            request=dict(schemaVersion=SCHEMA,nonce=nonce,stage=stage['kind'],sourceSha256=key,
                record=copy.deepcopy(record),sources=self.source.describe_source(record),
                requestedAt=started,expiresAt=expires,hostClockId=record['hostClockId'],
                protocol='positive_seed_then_at_most_one_stationary_fresh_pair',automaticRetry=False)
            self.requests.write_json(nonce+'-request.json',request)
            response_path=nonce+'-response.json'
            while True:
                require(self.clock.monotonic()<expires,'phase_review_deadline_no_retry')
                tick()
                try:raw=_read(self.root,response_path);break
                except StorageError as exc:
                    if not isinstance(exc.__cause__,FileNotFoundError):raise
                self.clock.sleep(min(.05,expires-self.clock.monotonic()))
            review=_decode_json(raw)
            require(review.get('nonce')==nonce and review.get('sourceSha256')==key
                    and review.get('frames')==record['frames'] and review.get('accepted') is True
                    and type(review.get('reviewedAt')) in (int,float)
                    and started<=review['reviewedAt']<=self.clock.monotonic()<expires
                    and review.get('schemaVersion')==SCHEMA and review.get('stage')==stage['kind']
                    and review.get('method')=='explicit_dual_source_visual_review'
                    and review.get('humanAssisted') is True and review.get('stable') is True
                    and type(review.get('reached')) is bool
                    and isinstance(review.get('labels'),dict)
                    and all(type(v) is bool for v in review['labels'].values())
                    and review['labels'].get('operatorPresent') is True
                    and review['labels'].get('clearance') is True,'phase_nonce_source_reply_rejected')
            from hashlib import sha256
            ref=dict(path=response_path,sha256=sha256(raw).hexdigest())
            seed=copy.deepcopy(record)
            hold=stage['kind']=='lift' and review['reached'] and review['labels'].get('retained') is True
            require(not hold or review['labels'].get('airborne') is True,'phase_hold_airborne_required')
            if any(self.clock.monotonic()-f['sourceEarliest']>3. for f in record['frames']) or (hold and not review.get('retention')):
                # The seed has never been offered to RUN. It is now only a
                # source-bound human anchor; a stale actor result is never fixed.
                not_before=(max(f['sourceLatest'] for f in seed['frames'])+self.scope['retentionSeconds']
                            if hold else None)
                record=self.source.refresh_record(stage,seed,deadline=deadline,not_before=not_before)
                evidence=self._transfer(stage,seed,review,record,deadline,tick)
            else:evidence=dict(seedSourceSha256=key,currentSourceSha256=key,stationaryRefresh=False)
            if hold and not review.get('retention'):
                require(record['frames']!=seed['frames'],'phase_hold_new_pair_required')
                refs=[]
                for suffix,held in [('seed',seed),('current',record)]:
                    saved=self.requests.write_json(nonce+'-hold-'+suffix+'.json',held)
                    refs.append(dict(path=saved['artifactRef'],sha256=saved['sha256'],root='request_store'))
                self.retention[plan_digest(source_record(record))]=refs
                self.accepted[plan_digest(source_record(seed))]=(copy.deepcopy(review),ref,
                    dict(seedSourceSha256=key,rawHumanSeed=True))
                evidence=dict(evidence,retentionBindings=refs)
        self.accepted[plan_digest(source_record(record))]=(copy.deepcopy(review),ref,evidence)
        entry=self.entry(stage,record)
        # Keep the original judged geometry; repeated transfers cannot accumulate
        # a 1.5-pixel drift by moving the anchor after every observation.
        self.anchors[stage['kind']]=(copy.deepcopy(seed),copy.deepcopy(review),ref)
        return record,entry

    def entry(self, stage, record):
        review, digest = self._review(stage, record)
        labels = review['labels']
        # Entry failure is preserved; observing an already achieved goal need not
        # manufacture the opposite entry predicate (e.g. noncontact after CLOSE).
        return dict(labels, evidenceSha256=digest, humanAssisted=True)

    def goal(self, stage, record):
        review, digest = self._review(stage, record)
        result = dict(review['labels'], reached=review['reached'], humanAssisted=True,
                      evidenceSha256=digest, observationSha256=plan_digest(record))
        if result['reached']:
            require(all(result.get(k) is True for k in GOAL[stage['kind']]), 'phase_goal_labels_missing')
            if stage['kind'] == 'lift':
                holds = self.retention.get(plan_digest(source_record(record)),review.get('retention', []))
                require(isinstance(holds,list) and len(holds) >= 2, 'phase_retention_raw_records_required')
                times=[]; seen=set();intervals=[]
                for ref in holds:
                    raw = _read(self.requests.root if ref.get('root')=='request_store' else self.root, ref['path'])
                    from hashlib import sha256
                    require(sha256(raw).hexdigest() == ref['sha256'], 'phase_retention_file_changed')
                    held = _decode_json(raw)
                    require(self.audit(stage, held) is True, 'phase_retention_raw_audit_failed')
                    held_review, _ = self._review(stage, held)
                    require(all(held_review['labels'].get(k) is True for k in ('airborne','retained'))
                            and held['context'] == record['context']
                            and type(record.get('outputSequence')) is int
                            and held.get('outputSequence') == record['outputSequence']
                            and held.get('controllerBootId') == record.get('controllerBootId')
                            and held.get('hostClockId') == record.get('hostClockId'),
                            'phase_retention_support_or_epoch_unknown')
                    ids=tuple(f['frameId'] for f in held['frames'])
                    require(not seen.intersection(ids), 'phase_retention_frame_reused'); seen.update(ids)
                    intervals.append((min(f['sourceEarliest'] for f in held['frames']),
                                      max(f['sourceLatest'] for f in held['frames'])))
                times=[intervals[0][1],intervals[-1][0]]
                require(intervals==sorted(intervals) and times[-1]-times[0] >= self.scope['retentionSeconds']
                        and times[-1] <= self.clock.monotonic()
                        and set(f['frameId'] for f in record['frames']).issubset(seen),
                        'phase_retention_span_or_current_pair_missing')
                result['observationTimes']=times
        return result

    def verify_goal(self, stage, record, result):
        require(self.audit(stage, record) is True, 'phase_raw_audit_failed')
        return self.goal(stage, record) == result
