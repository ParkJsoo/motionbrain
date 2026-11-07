"""Two single-use model trials joined by original dual-camera contact evidence.

Construction does not access hardware. The close core never calls a lift; this
owner requires a new table-supported review and guards the original M4 RUN.
No reset, lowering, training, context conversion or motor retry is provided.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import time

import cv2
import numpy as np

from learning.artifacts import _decode_json, _read
from learning.manual_collection_dataset import require
from learning.model_grasp_trial import CONTACT_SCHEMA
from learning.supervised_collection import plan_digest
from learning import m4_up_commissioning as m4
from learning import m4_lift_learning as metrology

SCHEMA = 'motionbrain.close_lift_connection.v1'


class CloseLiftTrial:
    def __init__(self, scope, close, lift, contact_scope, review_contact, store,
                 *, evidence_root, clock=time):
        require(set(scope) == {'schemaVersion', 'trialId', 'closeScopeSha256',
                'liftScopeSha256', 'contactScopeSha256', 'maxSeconds'}, 'connection_scope_fields')
        require(scope['schemaVersion'] == SCHEMA and isinstance(scope['trialId'], str)
                and scope['trialId'] and type(scope['maxSeconds']) in (int, float)
                and math.isfinite(scope['maxSeconds']) and 1 <= scope['maxSeconds'] <= 600,
                'connection_scope_invalid')
        require(close.scope['schemaVersion'] == CONTACT_SCHEMA
                and close.scope['maxCommands'] == 1 and len(close.scope['stages']) == 1
                and close.scope['stages'][0]['kind'] == 'close', 'connection_one_close_required')
        require(lift.scope['schemaVersion'] == m4.MODEL_SHORT_SCHEMA
                and lift.scope['phase'] == 'model_selection' and lift.scope['maxCommands'] == 1
                and lift.scope['controllerBootId'] == close.scope['controllerBootId']
                and lift.scope['maxObservedDeltaDeg'] == 3., 'connection_lift_scope_required')
        require(lift.model is not None and callable(review_contact), 'connection_frozen_model_and_review_required')
        require(contact_scope['controllerBootId'] == lift.scope['controllerBootId']
                and contact_scope['context'] == lift.scope['context']
                and contact_scope['context']['supportMode'] == 'table_supported'
                and contact_scope['hostClockId'] != lift.scope['hostClockId'],
                'connection_contact_context_required')
        self.scope, self.contact_scope = copy.deepcopy(scope), copy.deepcopy(contact_scope)
        self.close, self.lift, self.review_contact = close, lift, review_contact
        self.store, self.clock = store, clock
        self.root = Path(evidence_root).resolve(strict=True)
        self.used = False
        self._bindings()

    def _bindings(self):
        require(self.scope['closeScopeSha256'] == plan_digest(self.close.scope)
                and self.scope['liftScopeSha256'] == plan_digest(self.lift.scope)
                and self.scope['contactScopeSha256'] == plan_digest(self.contact_scope),
                'connection_scope_changed')
        self.lift.model.verify()

    def _idle(self):
        require(self.clock.monotonic() < self.deadline, 'connection_deadline')
        self._bindings()
        self.close.port.check_idle()
        require(self.close.port.last_sequence == self.output_sequence,
                'connection_intervening_output')

    def _contact(self, proof, off_at):
        require(isinstance(proof, dict) and proof.get('status') == 'table_supported_contact_ready'
                and proof.get('method') == 'onsite_dual_camera_review'
                and all(proof.get(k) is True for k in
                        ('sameObjectVerified', 'depthVerified', 'clearanceVerified')),
                'connection_contact_not_ready')
        raw = _read(self.root, proof['evidenceFile'])
        require(len(raw) <= 65536 and m4.sha(raw) == proof['evidenceSha256']
                and _decode_json(raw) == {k: v for k, v in proof.items()
                                         if k not in ('evidenceFile', 'evidenceSha256')},
                'connection_contact_review_changed')
        observations = proof.get('observations')
        require(isinstance(observations, list) and len(observations) == 2,
                'connection_two_contact_observations_required')
        frames, slips, sealed = [], [], {}
        def seal(root, relative):
            sealed[str((root / relative).relative_to(self.root))] = m4.sha(_read(root, relative))
        seal(self.root, proof['evidenceFile'])
        for observation in observations:
            root = (self.root / observation['root']).resolve(strict=True)
            require(root.is_relative_to(self.root), 'connection_private_source_required')
            record = observation['record']
            require(record['hostClockId'] == self.contact_scope['hostClockId']
                    and record['context'] == self.contact_scope['context'] and record['stable'] is True
                    and all(record['onsite'].get(k) is True for k in
                            ('operatorPresent', 'clearanceVerified', 'jawContactVerified', 'tableSupportVerified')),
                    'connection_explicit_table_contact_required')
            declaration = _read(root, record['evidenceFile'])
            require(m4.sha(declaration) == record['evidenceSha256']
                    and _decode_json(declaration) == {k: v for k, v in record.items()
                                                      if k not in ('evidenceFile', 'evidenceSha256')},
                    'connection_observation_review_changed')
            pair = dict(external=m4._external_frame(root, record['external'], self.contact_scope),
                        gripper=m4._gripper_frame(root, record['gripper'], self.contact_scope,
                                                 record['gripper']['label']))
            for frame in pair.values():
                require(frame['sourceEarliest'] >= off_at + 3.
                        and frame['sourceLatest'] <= frame['persistedAt'] <= self.clock.monotonic() - .1,
                        'connection_contact_timing')
            binding = self.lift.registration['cameraBinding']
            require(all(pair['external'][k] == binding['external'][k] for k in binding['external'])
                    and all(pair['gripper'][k] == binding['gripper'][k] for k in binding['gripper']),
                    'connection_camera_binding_changed')
            label = record['gripper']['label']
            image_name = label + '-external.png'
            image_raw = _read(root, image_name)
            pixels = cv2.imdecode(np.frombuffer(image_raw, np.uint8), cv2.IMREAD_UNCHANGED)
            require(pixels is not None and pixels.shape ==
                    (pair['external']['height'], pair['external']['width'], 4)
                    and m4.sha(pixels.tobytes()) == pair['external']['pixelSha256'],
                    'connection_preview_not_original_pixels')
            grip_name = record['gripper']['root'] + '/esp32-' + label + '.jpg'
            source = dict(id=record['hostClockId'] + ':' + label,
                context=copy.deepcopy(record['context']),
                external=dict(path=image_name, sha256=m4.sha(image_raw)),
                gripper=dict(path=grip_name, sha256=record['gripper']['imageSha256']))
            features = metrology.measure(root, source, self.lift.registration)
            metrology.validate_before(features, self.lift.registration)
            slips.append([features['state'][2], features['quality']['slipDxPx']])
            # Full decode/metrology stays outside RUN's 100ms guard. Pin every
            # consumed original and declaration, then rehash just those bytes.
            ext = record['external']; prefix = ext['root'] + '/'
            ingress = _decode_json(_read(root, prefix + ext['ingressFile']))
            for name in (record['evidenceFile'], image_name, grip_name,
                         record['gripper']['root'] + '/esp32-' + label + '.json',
                         prefix + ext['ingressFile'], prefix + 'host-ingress/clock-before.json',
                         prefix + ingress['metadataFile'], prefix + ingress['imageFile']):
                seal(root, name)
            frames.append(pair)
        for name in ('external', 'gripper'):
            a, b = frames[0][name], frames[1][name]
            require(a['frameId'] != b['frameId'] and b['sourceEarliest'] - a['sourceLatest'] >= 1.
                    and 0 <= self.clock.monotonic() - b['sourceEarliest'] <= 10.,
                    'connection_contact_span_or_age')
        require(all(abs(a-b) <= 4. for a,b in zip(slips[0], slips[1])),
                'connection_contact_slip_exceeded')
        self.contact_seal = sealed
        self.contact_latest_earliest = min(f['sourceEarliest'] for f in frames[-1].values())
        return copy.deepcopy(proof)

    def _pinned_contact(self):
        require(all(m4.sha(_read(self.root, name)) == digest for name, digest in self.contact_seal.items()),
                'connection_contact_source_changed')
        require(0 <= self.clock.monotonic() - self.contact_latest_earliest <= 10.,
                'connection_contact_expired')

    def run(self, *, confirmed_scope_sha256):
        require(not self.used and confirmed_scope_sha256 == plan_digest(self.scope),
                'connection_exact_single_use_scope_required')
        self.used = True
        self.deadline = self.clock.monotonic() + self.scope['maxSeconds']
        report = dict(schemaVersion=SCHEMA, status='failed', errors=[], automaticRetry=False,
                      modelSelectedMotorCommands=0, trainingPerformed=False,
                      tableContactReady=False, graspVerified=False, liftVerified=False,
                      finalIdleOffVerified=False)
        old_guard = self.lift._before_run
        phase = 'close'
        try:
            self._bindings()
            # Each child retains its own immutable intent, budget and journal.
            require(self.clock.monotonic() + self.close.scope['maxSeconds'] <= self.deadline,
                    'connection_insufficient_close_time')
            result = self.close.run(confirmed_scope_sha256=self.scope['closeScopeSha256'])
            report['close'] = result
            report['finalIdleOffVerified'] = result['finalIdleOffVerified']
            report['modelSelectedMotorCommands'] = result['modelSelectedMotorCommands']
            require(result['status'] == 'closed_off' and result['finalIdleOffVerified'] is True
                    and result['modelSelectedMotorCommands'] == 1 and not result['errors'],
                    'connection_close_not_completed')
            receipt = result['stages'][0]['steps'][0]['receipt']
            require(receipt['offVerified'] is True and receipt['motor'] == 1,
                    'connection_close_receipt_required')
            self.output_sequence = self.close.port.last_sequence
            self._idle()
            require(self.clock.monotonic() + self.lift.scope['maxSeconds'] <= self.deadline,
                    'connection_insufficient_lift_time')
            phase = 'contact'
            proof = self.review_contact(after_off=receipt['offAt'], deadline=self.deadline, tick=self._idle)
            self._idle()
            report['contact'] = self._contact(proof, receipt['offAt'])
            report['tableContactReady'] = True
            # Last check before the M4 core acquires its own shared journal.
            self._idle()
            guard_calls = 0

            def connected_guard():
                nonlocal guard_calls
                guard_calls += 1
                require(guard_calls == 1, 'connection_single_lift_dispatch_required')
                started = self.clock.monotonic()
                self._bindings()
                self._pinned_contact()
                require(self.lift.port.last_sequence == self.output_sequence,
                        'connection_intervening_output')
                old_guard()
                require(self.clock.monotonic() < self.deadline
                        and self.clock.monotonic() - started <= .1,
                        'connection_lift_guard_expired')

            self.lift._before_run = connected_guard
            phase = 'lift'
            # The earlier M1 OFF proof cannot attest a later M4 attempt when
            # the child's own finalization or evidence write raises.
            report['finalIdleOffVerified'] = False
            result = self.lift.run(confirmed_scope_sha256=self.scope['liftScopeSha256'])
            report['lift'] = result
            report['finalIdleOffVerified'] = result['finalIdleOffVerified']
            report['modelSelectedMotorCommands'] += int(result.get('modelSelectedCommandExecuted') is True)
            require(result['status'] == 'completed' and result['finalIdleOffVerified'] is True,
                    'connection_lift_not_completed')
            require(guard_calls == 1, 'connection_lift_guard_not_called')
            report['status'] = 'lift_completed_pending_retention_review'
            # Normal execution is distinct from independent air-gap/retention.
        except (Exception, KeyboardInterrupt) as exc:
            report['errors'].append(dict(phase=phase, type=type(exc).__name__))
        finally:
            self.lift._before_run = old_guard
            self.store.write_json('result.json', report)
        return report
