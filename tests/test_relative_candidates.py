"""고정된 개발 모델의 오프라인 후보 비교. 실물·반사실 성공률 없음."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from learning import relative_action_learning as relative
from learning.relative_model_bundle import load_frozen_model
from tests.test_pipeline_learning import row
from tools import replay_relative_actions as cli


def model():
    rows=[]
    for direction,delta in [('up',-10.),('down',10.)]:
        for i,y in enumerate([20.,60.]):
            r=row(direction+str(i),'train',joint='wrist')
            r['command']=dict(joint='wrist',direction=direction,percent=63,durationMs=550)
            r['context']['landmarks']='development-gripper-visible-v1:'+'a'*64
            r['context']['stateFields']=relative.state_fields(r['context'])
            r['before']=[3.,y,122.];r['after']=[3.,y+delta,122.]
            r.update(developmentObservationOnly=True,recoveryDevelopmentOnly=True)
            rows.append(r)
    return relative.fit(rows,allow_development_observations=True)


def query(value,target_y=20.,**kwargs):
    return dict(state=[3.,40.,122.],target=[3.,target_y,122.],
                commands=[m['command'] for m in value['models']],context=value['context'],**kwargs)


class CandidateTests(unittest.TestCase):
    def test_command_mean_compares_up_down_and_stop(self):
        value=model();original=copy.deepcopy(value)
        for target,direction in [(20.,'up'),(60.,'down'),(40.,None)]:
            q=query(value,target)
            result=relative.select(value,**q,predictor='commandMean')
            self.assertEqual(result['selected']['command']['direction'] if direction else result['selected']['command'],direction)
            self.assertFalse(result['controlEligible'])
            self.assertFalse(result['counterfactualOutcomesObserved'])
        self.assertEqual(value,original)

    def test_small_predicted_improvement_selects_stop_without_hiding_ranking(self):
        value=model();q=query(value,34.)
        result=relative.select(value,**q,predictor='commandMean',minimum_improvement_px=4.)
        self.assertEqual(result['selected']['kind'],'stop')
        self.assertEqual(result['candidates'][0]['kind'],'action')
        self.assertEqual(result['bestPredictedImprovementPx'],2.)
        self.assertEqual(result['selectionReason'],'predicted_improvement_below_margin')
        with patch('learning.relative_action_learning.predict',wraps=relative.predict) as predict:
            relative.select(value,**q,predictor='commandMean')
        self.assertTrue(all(call.kwargs['baseline'] for call in predict.call_args_list))

    def test_bad_parameters_and_unsupported_candidates_are_preserved(self):
        value=model();q=query(value)
        for margin in [True,-1,float('nan'),float('inf')]:
            with self.assertRaises(ValueError):relative.select(value,**q,minimum_improvement_px=margin)
        with self.assertRaisesRegex(ValueError,'unsupported_selection_predictor'):
            relative.select(value,**q,predictor='unverified')
        q['state']=[3.,80.,122.]
        result=relative.select(value,**q,predictor='commandMean')
        self.assertEqual(result['status'],'abstained');self.assertEqual(len(result['rejected']),2)
        q=query(value);q['commands'][0]=dict(q['commands'][0],percent=62)
        self.assertEqual(len(relative.select(value,**q)['rejected']),1)

    def test_query_only_cli_loads_exact_model_without_fit_and_keeps_physical_loader_blocked(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'.local').mkdir();value=model()
            raw=json.dumps(value).encode();path=root/'.local/model.json';path.write_bytes(raw)
            sha=hashlib.sha256(raw).hexdigest();manifest=root/'queries.json'
            payload=dict(mode='offline_candidate_queries',queries=[query(value,targetKind='synthetic_image_state',predictor='commandMean')])
            manifest.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError,'development_model_not_admitted'):
                cli.run(manifest,root/'.local/blocked',frozen_model=path,model_sha256=sha,root=root)
            with patch.object(cli,'fit',side_effect=AssertionError('refit')),patch.object(cli,'load_manifest',side_effect=AssertionError('source loading')):
                report=cli.run(manifest,root/'.local/replay',frozen_model=path,model_sha256=sha,root=root,allow_development_model=True)
            self.assertEqual(report['modelSha256'],sha);self.assertEqual(report['evaluated'],0)
            self.assertFalse(report['independentEvaluation']);self.assertFalse(report['modelRefit'])
            self.assertEqual(report['motorCommands'],0)
            self.assertEqual(report['selections'][0]['selection']['selected']['command']['direction'],'up')
            self.assertEqual(path.read_bytes(),raw)
            with self.assertRaisesRegex(ValueError,'development_model_not_admitted'):load_frozen_model(path,sha)
            with self.assertRaisesRegex(ValueError,'hash_changed'):
                cli.run(manifest,root/'.local/hashfail',frozen_model=path,model_sha256='0'*64,root=root,allow_development_model=True)
            for change in [dict(records=[{}]),dict(queries=[dict(payload['queries'][0],targetKind='physical_grasp')])]:
                manifest.write_text(json.dumps(dict(payload,**change)))
                with self.assertRaises(ValueError):
                    cli.run(manifest,root/'.local/bad',frozen_model=path,model_sha256=sha,root=root,allow_development_model=True)


if __name__=='__main__':unittest.main()
