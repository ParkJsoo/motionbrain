import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from learning import manual_collection_learning as m
from learning.artifacts import StorageError

COMMAND=dict(joint='shoulder',direction='down',percent=25,durationMs=350)


def rows_images():
    rows=[];images=[]
    for i,y in enumerate((-1.32,-1.41,-1.32,-1.14)):
        images.append(np.full((16,32,4),40+i*10,np.uint8))
        sensor=lambda value:dict(medianDeg=value,rangeDeg=.1,samples=[dict(valueDeg=value-.05),dict(valueDeg=value+.05)])
        rows.append(dict(sessionId=str(i),captureGroup='one',status='eligible',inputs=dict(command=copy.deepcopy(COMMAND)),
            targets=dict(sensorDeltaDeg=y,sensorBefore=sensor(220.),sensorAfter=sensor(220.+y))))
    return rows,images


class LearningTests(unittest.TestCase):
    def test_feature_path_uses_only_prospective_image_and_command(self):
        image=np.zeros((16,32,4),np.uint8)
        x=m.image_features(image,COMMAND)
        self.assertEqual(x.shape,(35,));self.assertEqual(x[-3:].tolist(),[-1.,.25,.35])
        rows,images=rows_images();a=m.evaluate(rows,images)
        for row in rows:
            row['evidence']={'actualOutputTrace':['not_an_input']}
            row['targets']['image']={'unused':'post-image'}
        b=m.evaluate(rows,images)
        self.assertEqual(a,b)

    def test_leave_one_out_centres_and_intercepts_use_only_training_rows(self):
        rows,images=rows_images();result=m.evaluate(rows,images)
        for i,fold in enumerate(result['diagnostics']['folds']):
            train=[j for j in range(4) if j!=i]
            expected=np.mean([m.image_features(images[j],COMMAND) for j in train],axis=0)
            np.testing.assert_allclose(fold['model']['mean'],expected)
            self.assertEqual(fold['model']['scale'],[1.]*35)
            self.assertAlmostEqual(fold['model']['intercept'],np.mean([rows[j]['targets']['sensorDeltaDeg'] for j in train]))
            self.assertNotIn(str(i),fold['trainingSessionIds'])
        self.assertAlmostEqual(result['diagnostics']['metrics']['trainingMean']['maeDeg'],.105)
        self.assertAlmostEqual(result['diagnostics']['metrics']['noChange']['maeDeg'],1.2975)
        self.assertFalse(result['diagnostics']['independentTest'])

    def test_held_out_label_cannot_change_its_prediction(self):
        rows,images=rows_images();a=m.evaluate(rows,images)
        rows[0]['targets']['sensorDeltaDeg']=999.
        b=m.evaluate(rows,images)
        self.assertEqual(a['diagnostics']['predictions'][0]['predictedDeltaDeg'],b['diagnostics']['predictions'][0]['predictedDeltaDeg'])

    def test_final_model_prediction_roundtrip_and_unsupported_commands(self):
        rows,images=rows_images();result=m.evaluate(rows,images);model=json.loads(json.dumps(result['model']))
        expected=result['diagnostics']['trainingFit']['predictionsDeg'][0]
        self.assertEqual(m.predict(model,images[0],COMMAND),expected)
        for field,value in (('joint','elbow'),('direction','up'),('percent',26),('durationMs',351)):
            command=dict(COMMAND);command[field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):m.predict(model,images[0],command)
        with self.assertRaisesRegex(ValueError,'image_shape'):m.predict(model,images[0][:8],COMMAND)

    def test_observed_envelope_is_not_calibrated_interval(self):
        rows,_=rows_images();u=m.sensor_uncertainty(rows[0])
        np.testing.assert_allclose(u['observedDeltaEnvelopeDeg'],[-1.42,-1.22])
        self.assertFalse(u['sensorAccuracyCalibrated'])
        self.assertIn('not_confidence',u['semantics'])

    def test_constant_images_and_nonfinite_fit(self):
        rows,images=rows_images();images=[images[0]]*4
        result=m.evaluate(rows,images)
        self.assertEqual(result['diagnostics']['imageFeaturesWithVariation'],0)
        for p in result['diagnostics']['predictions']:
            self.assertEqual(p['predictedDeltaDeg']['imageRidge'],p['predictedDeltaDeg']['trainingMean'])
        with self.assertRaises(ValueError):m.fit(np.zeros((3,35)),[0,float('nan'),0],COMMAND,images[0].shape)

    def fixture(self,root):
        rows,images=rows_images();saved=dict(rows=rows,counts=dict(listed=4,eligible=4,rejected=0),excluded=[],indexSha256='a'*64)
        for i,(row,image) in enumerate(zip(rows,images)):
            row['inputs']['image']={};row['targets']['image']={}
            for section in ('inputs','targets'):
                _,encoded=cv2.imencode('.png',image);raw=encoded.tobytes();name=f'{i}-{section}.png';(root/name).write_bytes(raw)
                row[section]['image']=dict(width=32,height=16,pixelSha256=m.digest(image.tobytes()),
                    png=dict(artifactRef=name,bytes=len(raw),sha256=m.digest(raw)))
        rebuilt=copy.deepcopy(saved)
        for row in rebuilt['rows']:
            for section in ('inputs','targets'):del row[section]['image']['png']
        path=root/'dataset.json';path.write_text(json.dumps(saved));return saved,rebuilt,path

    def test_manifest_labels_unknown_inputs_and_png_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);saved,rebuilt,path=self.fixture(root)
            with patch.object(m,'build_dataset',return_value=rebuilt):
                self.assertEqual(len(m.load_verified(root/'index.json',path)[1]),4)
                for mutate in (lambda d:d['rows'][0]['targets'].update(sensorDeltaDeg=7),
                               lambda d:d['rows'][0]['inputs'].update(actualPwm=60)):
                    changed=copy.deepcopy(saved);mutate(changed);path.write_text(json.dumps(changed))
                    with self.assertRaisesRegex(ValueError,'reaudited'):m.load_verified(root/'index.json',path)
                path.write_text(json.dumps(saved));(root/'0-inputs.png').write_bytes(b'tamper')
                with self.assertRaisesRegex(ValueError,'png_bytes'):m.load_verified(root/'index.json',path)

    def test_rehashed_png_with_changed_pixels_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);saved,rebuilt,path=self.fixture(root)
            _,encoded=cv2.imencode('.png',np.zeros((16,32,4),np.uint8));raw=encoded.tobytes()
            (root/'0-inputs.png').write_bytes(raw)
            saved['rows'][0]['inputs']['image']['png'].update(bytes=len(raw),sha256=m.digest(raw))
            path.write_text(json.dumps(saved))
            with patch.object(m,'build_dataset',return_value=rebuilt),self.assertRaisesRegex(ValueError,'png_pixels'):
                m.load_verified(root/'index.json',path)

    def test_export_preserves_rejected_and_excluded_and_is_immutable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);saved,rebuilt,path=self.fixture(root)
            rejected=dict(status='rejected',reason='capture_missing')
            for d in (saved,rebuilt):
                d['rows'].append(rejected);d['counts']=dict(listed=5,eligible=4,rejected=1);d['excluded']=[dict(reason='different_condition')]
            path.write_text(json.dumps(saved))
            with patch.object(m,'build_dataset',return_value=rebuilt):
                result=m.export(root/'index.json',path,root/'output')
                self.assertEqual(result['counts']['listed'],5);self.assertEqual(result['rejected'],[rejected])
                self.assertEqual(len(result['excluded']),1);self.assertTrue(result['trainingPerformed']);self.assertFalse(result['controlEligible'])
                self.assertTrue((root/'output/model.json').exists())
                with self.assertRaises(StorageError):m.export(root/'index.json',path,root/'output')


if __name__=='__main__':unittest.main()
