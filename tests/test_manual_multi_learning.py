import copy
import unittest
from unittest.mock import patch
import numpy as np
from learning import manual_multi_learning as m
from tests.test_manual_collection_learning import rows_images


class MultiTests(unittest.TestCase):
    def fixtures(self):
        rows,images=rows_images()
        for i,r in enumerate(rows):
            r['inputs']['command']['percent']=20+i*5
            r['evidence']=dict(controllerBootId='a'*32,commandId=str(i),cameraIdentity={'view':'one'})
        return rows,images

    def test_heldout_training_only_and_inputs(self):
        rows,images=self.fixtures();a=m.evaluate(rows,images)
        self.assertFalse(a['diagnostics']['independentTest'])
        for i,f in enumerate(a['diagnostics']['folds']):
            self.assertNotIn(rows[i]['sessionId'],f['trainingSessionIds'])
        rows[0]['targets']['sensorDeltaDeg']=999.
        rows[0]['evidence']['actualOutputTrace']=[999]
        b=m.evaluate(rows,images)
        self.assertEqual(a['diagnostics']['predictions'][0]['predictedDeltaDeg'],b['diagnostics']['predictions'][0]['predictedDeltaDeg'])

    def test_predict_support_and_fixed_features(self):
        rows,images=self.fixtures();model=m.evaluate(rows,images)['model']
        self.assertTrue(np.isfinite(m.predict(model,images[0],rows[0]['inputs']['command'])))
        c=dict(rows[0]['inputs']['command'],percent=99)
        with self.assertRaisesRegex(ValueError,'outside_training'):m.predict(model,images[0],c)

    def test_prepare_rejects_duplicate_and_camera_mismatch(self):
        rows,images=self.fixtures();saved=dict(policy={'p':1},counts={'listed':1},excluded=[],rows=[] ,indexSha256='x')
        def source(i):return (copy.deepcopy(saved),[copy.deepcopy(rows[i])],[images[i]],str(i))
        with patch.object(m,'load_verified',side_effect=[source(0),source(1),source(0)]):
            with self.assertRaisesRegex(ValueError,'duplicate_dataset'):m.prepare([('a','b')]*3)
        bad=source(2);bad[1][0]['evidence']['cameraIdentity']={'view':'different'}
        with patch.object(m,'load_verified',side_effect=[source(0),source(1),bad]):
            with self.assertRaisesRegex(ValueError,'camera_changed'):m.prepare([('a','b')]*3)

    def test_prepare_uses_reaudit_and_preserves_rejected(self):
        rows,images=self.fixtures();saved=dict(policy={'p':1},counts={'listed':2,'eligible':1,'rejected':1},excluded=[{'reason':'x'}],rows=[{'status':'rejected','reason':'bad'}],indexSha256='x')
        responses=[(copy.deepcopy(saved),[rows[i]],[images[i]],str(i)) for i in range(3)]
        with patch.object(m,'load_verified',side_effect=responses) as loader:
            rr,_,sources,_,_=m.prepare([('a','b')]*3)
        self.assertEqual(len(rr),3);self.assertEqual(loader.call_count,3)
        self.assertEqual(sources[0]['rejected'][0]['reason'],'bad')
        self.assertEqual(sources[0]['excluded'][0]['reason'],'x')

if __name__=='__main__':unittest.main()
