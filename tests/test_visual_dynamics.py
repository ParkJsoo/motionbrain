"""CPU software tests. Synthetic data never becomes a physical training corpus."""
import copy
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

from learning.artifacts import ArtifactStore, StorageError
from learning.dataset import build_manifest
from learning.dry_run import SyntheticAdapter
from learning.runner import CaptureRunner
from learning.features import transition_features, INPUT_NAMES, TARGET_NAMES
from learning.models import fit_linear, predict_linear, compare, metrics
from learning.visual_dynamics import load_dataset, load_model, train, evaluate, DatasetUnavailable
from tools.smoke_visual_dynamics import run_smoke, smoke_data


def feature_fixture():
    def observation(frame,x,angle):
        return {'synthetic':True,'width':320,'height':240,'label':'synthetic_target','classId':0,
                'targetBox':{'x':x,'y':30,'width':40,'height':60},'detected':True,
                'liveDetected':True,'held':False,'sourceFrameId':frame,'detectionSourceFrameId':frame,
                'm4':{'motor':4,'sensor':'AS5600','measured':True,'synthetic':True,'angleDegrees':angle}}
    return {'synthetic':True,'before':observation('synthetic-before',20,359),
            'after':observation('synthetic-after',24,1),
            'action':{'request':{'motor':4,'kind':'shoulder','pwm':0,'durationMs':100,
                                  'targetDegrees':1,'percent':30},'record':{'elapsedMs':50},'outputLog':{}}}


class FeaturesTest(unittest.TestCase):
    def test_observed_visual_and_m4_delta_no_other_joint_truth(self):
        x,y=transition_features(feature_fixture())
        self.assertEqual(len(x),len(INPUT_NAMES)); self.assertEqual(len(y),len(TARGET_NAMES))
        self.assertAlmostEqual(y[0],4/320); self.assertEqual(y[-1],2)
        self.assertAlmostEqual(x[-2],2/180)
        self.assertFalse(any('m1_angle' in name for name in INPUT_NAMES+TARGET_NAMES))

    def test_actual_output_and_after_target_do_not_leak_into_inputs(self):
        fixture=feature_fixture(); x,y=transition_features(fixture)
        fixture['action']['record']={'elapsedMs':999,'targetReached':True}
        fixture['action']['outputLog']={'entries':[{'pwm':255}]}
        fixture['after']['targetBox']['x']=80
        fixture['after']['m4']['angleDegrees']=20
        x2,y2=transition_features(fixture)
        self.assertEqual(x,x2); self.assertNotEqual(y,y2)

    def test_missing_nan_wrong_columns_and_unobserved_sensor_rejected(self):
        changes=[lambda t:t['before'].pop('targetBox'),
                 lambda t:t['before']['targetBox'].update(x=float('nan')),
                 lambda t:t['before']['targetBox'].update(width=0),
                 lambda t:t['before']['targetBox'].update(x=310),
                 lambda t:t['after'].update(label='other'),
                 lambda t:t['after'].update(width=640),
                 lambda t:t['before'].update(held=True),
                 lambda t:t['before']['m4'].update(measured=False),
                 lambda t:t['before']['m4'].update(angleDegrees='1'),
                 lambda t:t['action']['request'].update(motor=True),
                 lambda t:t['action']['request'].update(kind='unknown'),
                 lambda t:t['action']['request'].update(pwm=10)]
        for change in changes:
            with self.subTest(change=change):
                t=feature_fixture(); change(t)
                with self.assertRaises((ValueError,KeyError)): transition_features(t)


class ModelsTest(unittest.TestCase):
    def setUp(self):
        self.data,_=smoke_data()
        self.x,self.y=self.data['train']['x'],self.data['train']['y']

    def test_reproducible_train_only_normalization_and_fit(self):
        a=fit_linear(self.x,self.y); b=fit_linear(self.x.copy(),self.y.copy())
        self.assertEqual(a,b)
        np.testing.assert_allclose(a['mean'],self.x.mean(axis=0))
        self.assertEqual(a['fitRows'],len(self.x))
        held=self.data['test']['x'].copy(); held[:,0]=1000
        predict_linear(a,held)
        self.assertEqual(a,b)

    def test_linear_smoke_is_better_for_known_synthetic_mapping(self):
        result=compare(fit_linear(self.x,self.y),self.data['test']['x'],self.data['test']['y'])
        self.assertGreater(result['linear']['n'],0)
        self.assertLess(result['linear']['mae']['delta_m4_degrees'],result['noChange']['mae']['delta_m4_degrees'])

    def test_matrix_empty_nan_shape_and_type_rejected(self):
        for x in ([],np.empty((0,len(INPUT_NAMES))),np.ones((3,2)),
                  np.full(self.x.shape,np.nan),self.x.astype(str),np.ones(self.x.shape,dtype=bool)):
            with self.subTest(shape=np.shape(x)),self.assertRaises(ValueError): fit_linear(x,self.y)
        with self.assertRaises(ValueError): fit_linear(self.x,self.y[:-1])

    def test_wrong_model_columns_and_nonfinite_parameters_rejected(self):
        model=fit_linear(self.x,self.y)
        for key,value in [('inputNames',list(reversed(INPUT_NAMES))),('scale',[0]*len(INPUT_NAMES)),
                          ('intercept',[float('nan')]*len(TARGET_NAMES)),('coefficients',[[1]*5]),
                          ('featureVersion','unknown')]:
            bad=copy.deepcopy(model); bad[key]=value
            with self.assertRaises(ValueError): predict_linear(bad,self.x)

    def test_no_change_and_angular_error_and_empty_denominator(self):
        y=np.zeros((1,len(TARGET_NAMES))); p=y.copy(); y[0,-1]=179; p[0,-1]=-179
        self.assertEqual(metrics(y,p)['mae']['delta_m4_degrees'],2)
        self.assertEqual(metrics(np.empty((0,5)),np.empty((0,5)))['status'],'unavailable')

    def test_high_level_train_never_fits_held_out_rows(self):
        # Mock loader boundary tests numerical routing only. No physical files produced.
        metadata={'fixture':'synthetic-in-memory-routing-test'}
        with patch('learning.visual_dynamics.load_dataset',return_value=(self.data,metadata)):
            a=train('unused')
            self.data['test']['y'][:]=999
            self.data['validation']['x'][:]=999
            b=train('unused')
            self.assertEqual(a,b)
            self.assertEqual(a['model']['fitRows'],len(self.x))
            report=evaluate('unused',a)
            self.assertEqual(report['splits']['test']['linear']['n'],len(self.data['test']['x']))
            broken=copy.deepcopy(a); broken['metadata']={'different':True}
            with self.assertRaises(ValueError): evaluate('unused',broken)
            with self.assertRaises(ValueError): evaluate('unused',run_smoke())


class DatasetGateTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        run=self.root/'dry-run'
        runner=CaptureRunner(ArtifactStore(run),SyntheticAdapter())
        runner.capture(transition_id='t1',episode_id='e1'); runner.close()
        self.manifest=build_manifest([run]); self.path=self.root/'manifest.json'
        self.write_manifest()

    def write_manifest(self):
        self.path.write_text(json.dumps(self.manifest,allow_nan=False))

    def test_actual_synthetic_corpus_refused_no_training_escape_flag(self):
        with self.assertRaises(DatasetUnavailable) as e: load_dataset(self.path)
        self.assertEqual(e.exception.validation['eligibleCount'],0)
        self.assertFalse(e.exception.validation['readyForTraining'])
        with self.assertRaises(DatasetUnavailable): train(self.path)

    def test_forged_eligibility_split_or_provenance_rejected(self):
        for field,value in [('eligible',True),('provenance','physical')]:
            saved=copy.deepcopy(self.manifest)
            self.manifest['rows'][0][field]=value; self.write_manifest()
            with self.assertRaisesRegex(DatasetUnavailable,'invalid_manifest'): load_dataset(self.path)
            self.manifest=saved
        self.manifest['assignments'][self.manifest['rows'][0]['sampleId']]='train'; self.write_manifest()
        with self.assertRaisesRegex(DatasetUnavailable,'invalid_manifest'): load_dataset(self.path)

    def test_original_image_tamper_or_missing_fails_reload(self):
        image=self.manifest['rows'][0]['images'][0]
        path=Path(self.manifest['rows'][0]['sourceRoot'])/image['artifactRef']
        path.write_bytes(b'changed')
        with self.assertRaisesRegex(DatasetUnavailable,'invalid_manifest'): load_dataset(self.path)
        path.unlink()
        with self.assertRaisesRegex(DatasetUnavailable,'invalid_manifest'): load_dataset(self.path)

    def test_empty_manifest_and_nan_json_rejected(self):
        self.path.write_text('{}')
        with self.assertRaises(DatasetUnavailable): load_dataset(self.path)
        self.path.write_text('{"x":NaN}')
        with self.assertRaises(StorageError): load_dataset(self.path)

    def test_cli_refusal_is_saved_and_no_model_is_created(self):
        from tools.train_visual_dynamics import main
        out=self.root/'trained'
        with contextlib.redirect_stderr(io.StringIO()):
            code=main(['--manifest',str(self.path),'--output-dir',str(out)])
        self.assertEqual(code,2); self.assertFalse((out/'model.json').exists())
        self.assertEqual(json.loads((out/'refusal.json').read_text())['status'],'refused')

    def test_evaluation_unavailable_is_explicit(self):
        from tools.evaluate_visual_dynamics import main
        model=self.root/'smoke.json'; model.write_text(json.dumps(run_smoke()))
        out=self.root/'evaluation'
        with contextlib.redirect_stderr(io.StringIO()):
            code=main(['--manifest',str(self.path),'--model',str(model),'--output-dir',str(out)])
        self.assertEqual(code,2)
        self.assertEqual(json.loads((out/'unavailable.json').read_text())['status'],'unavailable')

    def test_smoke_is_separate_and_deterministic(self):
        a,b=run_smoke(),run_smoke()
        self.assertEqual(a,b)
        self.assertEqual(a['evidence'],'synthetic_math_only')
        self.assertFalse(a['trainingEligible']); self.assertFalse(a['controlEligible'])


class ModelReceiptTest(unittest.TestCase):
    def test_model_receipt_hash_and_metadata_required(self):
        # Storage contract fixture, no physical trained artifact declaration.
        with tempfile.TemporaryDirectory() as d:
            store=ArtifactStore(Path(d)/'fixture')
            model={'metadata':{'fixture':'synthetic'},'weights':[1,2]}
            ref=store.write_json('model.json',model)
            store.write_json('result.json',{'status':'trained','model':ref,'metadata':model['metadata']})
            self.assertEqual(load_model(store.root/'model.json'),model)
            (store.root/'model.json').write_text('{}')
            with self.assertRaises(StorageError): load_model(store.root/'model.json')


    def test_malformed_receipt_and_model_json_produce_unavailable(self):
        from tools.evaluate_visual_dynamics import main
        for value in ([],None):
            with self.subTest(value=value),tempfile.TemporaryDirectory() as d:
                root=Path(d); (root/'result.json').write_text(json.dumps(value))
                output=root/'output'
                with contextlib.redirect_stderr(io.StringIO()):
                    code=main(['--manifest',str(root/'unused'), '--model',str(root/'model.json'),
                               '--output-dir',str(output)])
                self.assertEqual(code,2)
                self.assertEqual(json.loads((output/'unavailable.json').read_text())['status'],'unavailable')

    def test_null_artifact_or_nested_model_refused(self):
        data,_=smoke_data(); metadata={'fixture':'synthetic-in-memory-only'}
        with patch('learning.visual_dynamics.load_dataset',return_value=(data,metadata)):
            good=train('unused')
            for value in ([],None,{**good,'model':None},{**good,'model':[]}):
                with self.subTest(value=value),self.assertRaises(ValueError): evaluate('unused',value)
        for value in ([],None):
            with self.assertRaises(ValueError): predict_linear(value,data['train']['x'])
