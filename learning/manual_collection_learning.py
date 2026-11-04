"""Small offline image/command baseline; one-group diagnostics, never control."""
from __future__ import annotations
import copy
import platform
from pathlib import Path

import cv2
import numpy as np

from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.manual_collection_dataset import build_dataset, digest, require

SCHEMA = 'motionbrain.manual_image_command_learning.v1'
MODEL = 'motionbrain.manual_image_command_ridge.v1'
FEATURES = dict(kind='whole_frame_gray_area_8x4', width=8, height=4,
                pixelDivisor=255., commandColumns=['direction_sign','percent_div_100','duration_seconds'])
ALPHA = 1.
COLUMNS = 35


def same(a, b):
    return a == b


def command_features(command):
    require(isinstance(command,dict) and set(command)=={'joint','direction','percent','durationMs'}, 'command_fields')
    require(command['joint']=='shoulder' and command['direction']=='down', 'unsupported_joint_or_direction')
    require(type(command['percent']) is int and 1<=command['percent']<=100
            and type(command['durationMs']) is int and command['durationMs']>0, 'command_values')
    return [-1.,command['percent']/100.,command['durationMs']/1000.]


def image_features(image, command):
    require(isinstance(image,np.ndarray) and image.dtype==np.uint8 and image.ndim==3
            and image.shape[2]==4 and min(image.shape[:2])>=8, 'bgra_image_required')
    gray=cv2.cvtColor(image,cv2.COLOR_BGRA2GRAY)
    small=cv2.resize(gray,(FEATURES['width'],FEATURES['height']),interpolation=cv2.INTER_AREA)
    return np.r_[small.astype(float).ravel()/255.,command_features(command)]


def matrix(x):
    x=np.asarray(x)
    require(x.dtype.kind in 'iuf' and x.ndim==2 and x.shape[1]==COLUMNS
            and len(x)>0 and np.isfinite(x).all(), 'finite_feature_matrix')
    return x.astype(float)


def fit(x,y,command,image_shape):
    x=matrix(x);y=np.asarray(y,dtype=float)
    require(y.shape==(len(x),) and len(x)>=2 and np.isfinite(y).all(), 'finite_training_labels')
    command_features(command)
    mean=x.mean(axis=0);scale=np.ones(COLUMNS)
    z=(x-mean)/scale;intercept=float(y.mean())
    coefficients=np.linalg.solve(z.T@z+ALPHA*np.eye(COLUMNS),z.T@(y-intercept))
    require(np.isfinite(coefficients).all(),'nonfinite_fit')
    return dict(schemaVersion=MODEL,features=copy.deepcopy(FEATURES),alpha=ALPHA,scaling='fixed_units_train_centering',
        mean=mean.tolist(),scale=scale.tolist(),coefficients=coefficients.tolist(),intercept=intercept,
        fitRows=len(x),supportedCommand=copy.deepcopy(command),imageShape=list(image_shape),
        preprocessingFit='training_rows_only',controlEligible=False,independentTest=False,
        uncertaintyCalibration='unavailable',commandEffectIdentifiable=False)


def predict_matrix(model,x):
    require(model.get('schemaVersion')==MODEL and model.get('features')==FEATURES
            and model.get('alpha')==ALPHA and model.get('scaling')=='fixed_units_train_centering'
            and model.get('controlEligible') is False,'model_contract')
    x=matrix(x)
    mean=matrix([model['mean']])[0];scale=matrix([model['scale']])[0];coef=matrix([model['coefficients']])[0]
    require((scale==1).all() and np.isfinite(model['intercept']), 'model_parameters')
    prediction=(x-mean)/scale@coef+model['intercept']
    require(np.isfinite(prediction).all(),'nonfinite_prediction')
    return prediction


def predict(model,image,command):
    command_features(command)
    require(command==model['supportedCommand'],'command_outside_training_support')
    require(list(image.shape)==model['imageShape'],'image_shape_changed')
    return float(predict_matrix(model,[image_features(image,command)])[0])


def load_verified(index_path,dataset_path):
    dataset_path=Path(dataset_path);raw=_read(dataset_path.parent,dataset_path.name)
    saved=_decode_json(raw);stripped=copy.deepcopy(saved)
    for row in stripped['rows']:
        if row['status']=='eligible':
            for section in ('inputs','targets'):
                row[section]['image'].pop('png',None)
    rebuilt=build_dataset(index_path)
    require(same(stripped,rebuilt),'dataset_differs_from_reaudited_sources')
    rows=[];images=[]
    for row in saved['rows']:
        if row['status']!='eligible':continue
        for section in ('inputs','targets'):
            frame=row[section]['image'];ref=frame['png']
            require(set(ref)=={'artifactRef','sha256','bytes'},'png_reference_fields')
            encoded=_read(dataset_path.parent,ref['artifactRef'])
            require(len(encoded)==ref['bytes'] and digest(encoded)==ref['sha256'],'png_bytes_changed')
            image=cv2.imdecode(np.frombuffer(encoded,np.uint8),cv2.IMREAD_UNCHANGED)
            require(image is not None and image.dtype==np.uint8 and image.shape==(frame['height'],frame['width'],4)
                    and digest(image.tobytes())==frame['pixelSha256'],'png_pixels_changed')
            if section=='inputs':images.append(image)
        rows.append(row)
    require(len(rows)>=3,'at_least_three_eligible_records_required')
    require(len({r['captureGroup'] for r in rows})==1,'single_development_group_required')
    require(all(r['inputs']['command']==rows[0]['inputs']['command'] for r in rows),'single_command_required')
    require(all(im.shape==images[0].shape for im in images),'image_shape_changed')
    return saved,rows,images,digest(raw)


def scores(y,p):
    error=np.asarray(p)-np.asarray(y)
    return dict(n=len(error),maeDeg=float(np.abs(error).mean()),rmseDeg=float(np.sqrt(np.mean(error**2))),
                biasDeg=float(error.mean()))


def sensor_uncertainty(row):
    before=row['targets']['sensorBefore'];after=row['targets']['sensorAfter']
    a=[s['valueDeg'] for s in before['samples']];b=[s['valueDeg'] for s in after['samples']]
    return dict(before=copy.deepcopy(before),after=copy.deepcopy(after),
        observedDeltaEnvelopeDeg=[min(b)-max(a),max(b)-min(a)],
        semantics='observed_sample_envelope_not_confidence_or_prediction_interval',
        sensorAccuracyCalibrated=False)


def evaluate(rows,images):
    require(len(rows)==len(images) and len(rows)>=3,'aligned_examples_required')
    command=rows[0]['inputs']['command']
    require(all(r['inputs']['command']==command for r in rows),'single_command_required')
    x=matrix([image_features(im,r['inputs']['command']) for r,im in zip(rows,images)])
    y=np.asarray([r['targets']['sensorDeltaDeg'] for r in rows],dtype=float)
    require(len(x)==len(rows)==len(images) and np.isfinite(y).all(),'aligned_examples_required')
    predictions=[];folds=[]
    for i,row in enumerate(rows):
        train=[j for j in range(len(rows)) if j!=i]
        model=fit(x[train],y[train],command,images[0].shape)
        values=dict(imageRidge=float(predict_matrix(model,x[i:i+1])[0]),
                    trainingMean=float(y[train].mean()),noChange=0.)
        predictions.append(dict(sessionId=row['sessionId'],actualDeltaDeg=float(y[i]),predictedDeltaDeg=values,
            residualDeg={k:v-float(y[i]) for k,v in values.items()},sensorObservation=sensor_uncertainty(row)))
        folds.append(dict(heldOutSessionId=row['sessionId'],trainingSessionIds=[rows[j]['sessionId'] for j in train],model=model))
    final=fit(x,y,command,images[0].shape)
    fit_predictions=predict_matrix(final,x)
    return dict(model=final,diagnostics=dict(kind='leave_one_record_out_within_one_development_group',
        independentTest=False,hyperparameterSelection=False,
        metrics={k:scores(y,[p['predictedDeltaDeg'][k] for p in predictions]) for k in ('noChange','trainingMean','imageRidge')},
        predictions=predictions,folds=folds,
        imageFeatureRanges=(x[:,:32].max(axis=0)-x[:,:32].min(axis=0)).tolist(),
        imageFeaturesWithVariation=int(np.count_nonzero(np.ptp(x[:,:32],axis=0))),
        trainingFit=dict(metrics=scores(y,fit_predictions),predictionsDeg=fit_predictions.tolist(),independentTest=False)))


def run(index_path,dataset_path):
    saved,rows,images,dataset_hash=load_verified(index_path,dataset_path)
    result=evaluate(rows,images)
    result['model'].update(datasetSha256=dataset_hash,indexSha256=saved['indexSha256'],
        captureGroup=rows[0]['captureGroup'],cameraIdentity=copy.deepcopy(rows[0].get('evidence',{}).get('cameraIdentity')),
        labelPolicy=copy.deepcopy(saved.get('policy')),target='M4_sensor_delta_degrees_not_independent_visual_angle')
    result.update(schemaVersion=SCHEMA,trainingPerformed=True,independentTest=False,controlEligible=False,
        datasetSha256=dataset_hash,indexSha256=saved['indexSha256'],captureGroup=rows[0]['captureGroup'],
        counts=copy.deepcopy(saved['counts']),excluded=copy.deepcopy(saved['excluded']),
        rejected=[copy.deepcopy(r) for r in saved['rows'] if r['status']!='eligible'],
        target='M4_sensor_delta_degrees_not_independent_visual_angle',
        limitations=['four_record_scale_development_diagnostic','same_command_effect_not_identifiable',
                    'whole_frame_downsampling_can_remove_small_motion','no_calibrated_prediction_interval',
                    'same_group_sequential_records_not_independent_test','no_exposure_or_exact_controller_clock_calibration'],
        reproducibility=dict(python=platform.python_version(),numpy=np.__version__,opencv=cv2.__version__,
            codeSha256=digest(Path(__file__).read_bytes())))
    return result


def export(index_path,dataset_path,output):
    store=ArtifactStore(Path(output))
    result=run(index_path,dataset_path)
    store.write_json('model.json',result['model'])
    store.write_json('development-evaluation.json',result)
    return result
