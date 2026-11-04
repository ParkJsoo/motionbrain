"""Finite batch provenance, prospective leakage, tiny fits and grouped evaluation."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from learning import finite_batch_learning as batch
from learning import finite_profile_measurement as measurement
from learning.finite_profile_plan import catalog, prospective_input
from learning.supervised_measurement import sha


def save(base, name, value):
    path = base / name
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = (json.dumps(value, indent=2) + '\n').encode()
    path.write_bytes(raw)
    return dict(path=name, sha256=sha(raw))


def config_fixture():
    return dict(schemaVersion='motionbrain.endpoint_measurement.v1', width=64, height=48,
        stabilityThreshold=.5, points=[dict(id=name, roi=[0,0,64,48], hsvLower=[0,0,0],
        hsvUpper=[179,255,255], minAreaPx=1, maxAreaPx=100) for name in 'ABCD'],
        representation=dict(kind='relative_angle', parent=['A','B'], child=['C','D']))


def fixture(base, eligible=16, config=None):
    config = config or config_fixture()
    index = dict(schemaVersion=batch.INDEX_SCHEMA, contract=batch.CONTRACT, actionContract='finite_profile.v1',
        split='development', captureGroup='new-finite-batch', acquisitionPolicy=copy.deepcopy(batch.ACQUISITION),
        measurementConfig=save(base, 'config.json', config), slots=[])
    for i, condition in enumerate(sum(batch.BLOCKS['P'] + batch.BLOCKS['D'], [])):
        command = f'new-{i}'
        slot = dict(id=command, condition=condition, phaseRole='commissioning' if i < 4 else 'development',
            blockId=f'block-{i//4}', sources={}, unstartedReason=None if i < eligible else 'phase_stopped',
            capturePath=f'slot-{i}/native-camera')
        index['slots'].append(slot)
        if i >= eligible: continue
        profile = catalog()[condition]
        identity = dict(bootId='fresh-boot', commandId=command, requestEpoch=i+1)
        before = dict(value=10.+i/10, spread=.1, velocityPerSecond=.02, frameIds=[0,1,2,3], window=[19.25,19.9])
        delta = profile['direction'] * (.2 + .05 * int(condition[1]))
        after = dict(value=before['value']+delta, spread=.1, velocityPerSecond=0., frameIds=[4,5,6,7], window=[23.25,24.])
        receipt = dict(status='eligible', contract=batch.CONTRACT, anchorHostMonotonic=20.,
            maxBeforeAnchorAgeSeconds=2., measurementConfigSha256=measurement.digest(config),
            requestedProfileSha256=measurement.digest(profile), executionIdentity=identity, before=before,
            cameraIdentity=['unused-cached'], sourceHashes={})
        receipt['receiptSha256'] = measurement.digest(receipt)
        points = profile['points']; end = 1000 + points[-1]['offsetMs']
        entries = [dict(sequence=j+1, atMs=1000+p['offsetMs'], motor=2, pwm=p['pwm'], reason='driver_write') for j,p in enumerate(points)]
        entries += [dict(sequence=len(points)+j+1, atMs=end+(j==4), motor=j+1, pwm=0, reason='forced_off') for j in range(5)]
        record = dict(**identity, state='Completed', reason='NominalEnd', executed=True, anyNonzeroWrite=True,
            softwareOffConfirmed=True, offWriteRecorded=True, offConfirmationRecorded=True,
            timingConformant=True, traceComplete=True, quietHeld=False, offFailureLatched=False,
            nextEventIndex=len(points), requestedProfile=profile, executeAcceptedAt=700, plannedStartAt=1000,
            plannedOffAt=end, firstNonzeroWriteAt=1000, offWriteAt=end, offConfirmedAt=end+1,
            plannedEventReceipts=[dict(index=j,present=True,atMs=1000+p['offsetMs'],pwm=p['pwm']) for j,p in enumerate(points)])
        closed = dict(record=record, actualOutput=dict(**identity, evidence='driver_write',
            clockDomain='controller_boot_millis', closed=True,captureComplete=True,dropped=0,
            lease=i+1,ordinal=i+1,openedAt=650,closedAt=5000,totalWrites=len(entries),entries=entries))
        report = dict(schemaVersion=measurement.SCHEMA, actionContract='finite_profile.v1',
            split='development', captureGroup=index['captureGroup'],expectedBootId='fresh-boot',expectedEpoch=i+1,
            requestedProfile=profile,trial=dict(id=command,beforeReceipt=receipt,closedEvidence=closed,
              dispatchBefore=20.1,dispatchAfter=20.2,hostOffReceiptAt=21.,quietVerifiedUntil=24.1,
              postEnd=24.1,releaseStartedAt=24.2))
        refs = slot['sources']
        refs['beforeReceipt'] = save(base, f'slot-{i}/before-receipt.json', receipt)
        refs['closedEvidence'] = save(base, f'slot-{i}/closed-actual-evidence.json', closed)
        measured = dict(schemaVersion=measurement.RESULT_SCHEMA,contract=batch.CONTRACT,actionContract='finite_profile.v1',
            split='development',captureGroup=index['captureGroup'],measurementConfigSha256=measurement.digest(config),
            physicalJointAngleCalibrated=False,units='projected_degrees',status='eligible',before=before,after=after,delta=delta,
            beforeReceiptSha256=receipt['receiptSha256'],captureReportSha256=measurement.digest(report),
            actualOutputSummary=measurement.validate_output(report['trial'],profile,identity),
            sourceHashes={'../before-receipt.json':refs['beforeReceipt']['sha256'],
                          '../closed-actual-evidence.json':refs['closedEvidence']['sha256']})
        refs['measurement'] = save(base, f'slot-{i}/measurement.json', measured)
        refs['report'] = save(base, f'slot-{i}/report.json', report)
        refs['prospectiveInput'] = save(base, f'slot-{i}/prospective-input.json', prospective_input(condition,receipt,identity,20.05))
        refs['result'] = save(base, f'slot-{i}/result.json', dict(executionComplete=True,measurementEligible=True,finalIdleOffVerified=True))
    save(base, 'index.json', index)
    return base/'index.json', index


def update_source(base, index, slot, key, mutate):
    ref = index['slots'][slot]['sources'][key]
    value = json.loads((base/ref['path']).read_text())
    mutate(value)
    index['slots'][slot]['sources'][key] = save(base,ref['path'],value)
    save(base,'index.json',index)


class BatchTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)

    def test_full_batch_excludes_pilot_and_groups_by_block(self):
        path,_ = fixture(self.base)
        result = batch.run(path)
        self.assertEqual(result['counts'],dict(planned=16,attempted=16,unstarted=0,eligible=16,rejected=0,commissioningExcluded=4,trainingRows=12))
        self.assertTrue(result['trainingPerformed']); self.assertTrue(result['modelSelectionPerformed'])
        self.assertFalse(result['evaluation']['independentTest'])
        self.assertEqual(result['evaluation']['foldCount'],3)
        for fold in result['evaluation']['folds']:
            self.assertEqual(len(fold['fitIds']),8); self.assertEqual(len(fold['heldOutIds']),4)
            self.assertFalse(set(fold['fitIds']) & set(fold['heldOutIds']))
            self.assertTrue(all(int(name.split('-')[1])>=4 for name in fold['fitIds']+fold['heldOutIds']))

    def test_no_data_and_one_development_row_not_evaluation(self):
        for eligible in [0,4,5]:
            with self.subTest(eligible=eligible):
                path,_ = fixture(self.base,eligible)
                result=batch.run(path)
                self.assertEqual(result['trainingRows'],max(0,eligible-4))
                self.assertEqual(result['trainingPerformed'],eligible>4)
                self.assertEqual(result['evaluation']['status'],'unavailable')
                self.assertFalse(result['modelSelectionPerformed'])
                self.assertIsNone(result['selectedModel'])
                self.assertEqual(result['counts']['unstarted'],16-eligible)
        self.assertEqual(result['models']['ridge1']['model']['coefficients'],[0.,0.,0.])

    def test_rejected_and_remaining_unstarted_preserved(self):
        path,index=fixture(self.base,5)
        update_source(self.base,index,4,'measurement',lambda m:m.update(status='rejected',reason='measurement_window_unsettled'))
        update_source(self.base,index,4,'result',lambda r:r.update(measurementEligible=False,failure='post_measurement_rejected'))
        result=batch.run(path)
        self.assertEqual(result['counts']['attempted'],5)
        self.assertEqual(result['counts']['rejected'],1)
        self.assertEqual(result['counts']['unstarted'],11)
        self.assertFalse(result['trainingPerformed'])

    def test_attempt_after_failure_or_gap_rejected(self):
        path,index=fixture(self.base,6)
        update_source(self.base,index,4,'measurement',lambda m:m.update(status='rejected'))
        update_source(self.base,index,4,'result',lambda r:r.update(measurementEligible=False))
        with self.assertRaisesRegex(ValueError,'batch_stop'): batch.load_index(path)

    def test_attempt_after_unstarted_gap_rejected(self):
        path,index=fixture(self.base,6)
        index['slots'][4].update(sources={}, unstartedReason='not_started')
        save(self.base,'index.json',index)
        with self.assertRaisesRegex(ValueError,'batch_stop'): batch.load_index(path)

    def test_attempt_after_prearm_failure_rejected(self):
        path,index=fixture(self.base,2)
        index['slots'][0]['sources'] = {'result': save(self.base, 'slot-0/result.json',
            dict(executionComplete=False,measurementEligible=False,finalIdleOffVerified=True,
                 failure='pre_arm_markers_not_ready'))}
        save(self.base,'index.json',index)
        with self.assertRaisesRegex(ValueError,'batch_stop'): batch.load_index(path)

    def test_missing_slots_reordered_or_relabelled_blocks_rejected(self):
        for change in [lambda i:i['slots'].pop(),lambda i:i['slots'][4].update(condition='D4'),
                       lambda i:i['slots'][0].update(phaseRole='development'),
                       lambda i:i['slots'][5].update(blockId='new-split'),
                       lambda i:i['slots'][4].update(id=i['slots'][0]['id'])]:
            path,index=fixture(self.base);change(index);save(self.base,'index.json',index)
            with self.subTest(change=change),self.assertRaises(ValueError):batch.load_index(path)

    def test_legacy_and_group_mixing_rejected(self):
        path,index=fixture(self.base)
        index['contract']='settled_endpoints_after_confirmed_off_v1';save(self.base,'index.json',index)
        with self.assertRaises(ValueError):batch.load_index(path)
        path,index=fixture(self.base)
        update_source(self.base,index,4,'report',lambda r:r.update(captureGroup='old-finite-pilot'))
        with self.assertRaisesRegex(ValueError,'report_contract'):batch.load_index(path)

    def test_hash_tamper_and_outside_path_rejected(self):
        path,index=fixture(self.base,1)
        ref=index['slots'][0]['sources']['measurement'];(self.base/ref['path']).write_text('{}')
        with self.assertRaisesRegex(ValueError,'hash_mismatch'):batch.load_index(path)
        path,index=fixture(self.base,1)
        index['slots'][0]['sources']['measurement']['path']='../outside.json';save(self.base,'index.json',index)
        with self.assertRaisesRegex(ValueError,'unsafe_evidence'):batch.load_index(path)

    def test_rehashed_label_corruption_rejected(self):
        path,index=fixture(self.base,1)
        update_source(self.base,index,0,'measurement',lambda m:m.update(delta=m['delta']+1))
        with self.assertRaisesRegex(ValueError,'delta_changed'):batch.load_index(path)

    def test_rehashed_output_or_receipt_corruption_rejected(self):
        for source,change in [('closedEvidence',lambda c:c['actualOutput'].update(dropped=1)),
                              ('beforeReceipt',lambda r:r['before'].update(value=99)),
                              ('prospectiveInput',lambda r:r['features'].update(plannedSignedPwmSeconds=999)),
                              ('measurement',lambda m:m['actualOutputSummary'].update(activeMs=999))]:
            path,index=fixture(self.base,1);update_source(self.base,index,0,source,change)
            with self.subTest(source=source),self.assertRaises(ValueError):batch.load_index(path)

    def test_pre_features_and_models_do_not_use_actual_output(self):
        path,_=fixture(self.base)
        rows=batch.load_index(path)['rows'];changed=copy.deepcopy(rows)
        for row in changed:row['actualOutputSummary']={'activeMs':999999,'signedPwmIntegralCountMs':123456}
        first,second=batch.train_and_evaluate(rows),batch.train_and_evaluate(changed)
        self.assertEqual(first['models'],second['models']);self.assertEqual(first['evaluation'],second['evaluation'])
        row=copy.deepcopy(rows[0]);expected=batch.pre_features(row)
        del row['delta'];del row['actualOutputSummary']
        self.assertEqual(batch.pre_features(row),expected)
        row['features']['actualDuration']=1
        with self.assertRaises(ValueError):batch.pre_features(row)

    def test_fold_scaler_never_sees_held_extreme_and_tie_prefers_zero(self):
        path,_=fixture(self.base);rows=batch.load_index(path)['rows']
        for row in rows:row['delta']=0.
        for row in rows[:4]:row['features']['beforeProjectedAngle']=9999.
        result=batch.train_and_evaluate(rows)
        self.assertEqual(result['selectedModel'],'zeroChange')
        self.assertLess(result['evaluation']['folds'][0]['models']['ridge1']['mean'][0],100)
        self.assertFalse(result['evaluation']['selectionScoreIsIndependentPerformance'])

    def test_no_calibrated_intervals_from_residuals(self):
        path,_=fixture(self.base,8);result=batch.run(path)
        for score in result['fullDevelopmentFitMetrics'].values():
            self.assertEqual(score['uncertaintyInterpretation'],'descriptive_residuals_not_calibrated_intervals')
        self.assertEqual(result['evaluation']['status'],'unavailable')

    def test_raw_reaudit_is_called_and_mismatch_not_silently_excluded(self):
        path,index=fixture(self.base,1)
        (self.base/'slot-0/native-camera').mkdir()
        cached=json.loads((self.base/index['slots'][0]['sources']['measurement']['path']).read_text())
        with patch.object(batch,'measure',return_value=cached) as check:
            dataset=batch.load_index(path,reaudit_frames=True)
            self.assertEqual(check.call_count,1)
            self.assertTrue(dataset['metadata']['rawFramesReaudited'])
        cached['delta']+=.01
        with patch.object(batch,'measure',return_value=cached),self.assertRaisesRegex(ValueError,'raw_frame'):
            batch.load_index(path,reaudit_frames=True)


if __name__=='__main__':unittest.main()
