"""Offline exploratory multi-command regression; never use for control."""
import copy
import platform
from pathlib import Path
import cv2
import numpy as np
from learning.artifacts import ArtifactStore, _decode_json, _read
from learning.manual_collection_dataset import digest, require
from learning.manual_collection_learning import (load_verified, image_features, fit, predict_matrix,
                                                  command_features, scores, sensor_uncertainty)

SCHEMA='motionbrain.manual_multi_command_learning.v1'
MULTI_MODEL='motionbrain.manual_multi_command_ridge.v1'

def supported(rows):
    return [dict(joint=a,direction=b,percent=c,durationMs=d) for a,b,c,d in sorted({(v['joint'],v['direction'],v['percent'],v['durationMs']) for v in (r['inputs']['command'] for r in rows)})]

def as_multi(model,rows):
    model['supportedCommands']=supported(rows);model.pop('supportedCommand')
    model['schemaVersion']=MULTI_MODEL;model['commandEffectIdentifiable']=False
    return model

def raw_predict(model,x):
    require(model.get('schemaVersion')==MULTI_MODEL,'multi_model_contract')
    base=copy.deepcopy(model);base['schemaVersion']='motionbrain.manual_image_command_ridge.v1'
    return predict_matrix(base,x)


def prepare(pairs):
    require(isinstance(pairs,list) and len(pairs)>=3,'three_sources_required')
    ids=set();episodes=set();hashes=set();rows=[];images=[];sources=[];policy=None;camera=None
    for index,dataset in pairs:
        saved,rr,ii,h=load_verified(index,dataset,minimum_rows=0)
        require(h not in hashes,'duplicate_dataset');hashes.add(h)
        require(policy is None or saved['policy']==policy,'label_policy_changed');policy=saved['policy']
        for r,im in zip(rr,ii):
            ep=(r['evidence']['controllerBootId'],r['evidence']['commandId'])
            require(r['sessionId'] not in ids and ep not in episodes,'duplicate_episode')
            ids.add(r['sessionId']);episodes.add(ep)
            identity=r['evidence']['cameraIdentity']
            require(camera is None or camera==identity,'camera_changed');camera=identity
            if images:require(im.shape==images[0].shape,'image_shape_changed')
            rows.append(r);images.append(im)
        sources.append(dict(datasetSha256=h,indexSha256=saved['indexSha256'],counts=copy.deepcopy(saved['counts']),
                            excluded=copy.deepcopy(saved['excluded']),rejected=[copy.deepcopy(r) for r in saved['rows'] if r['status']!='eligible']))
    require(len(rows)>=3,'three_eligible_records_required')
    commands=[r['inputs']['command'] for r in rows]
    require(len({(c['joint'],c['direction'],c['percent'],c['durationMs']) for c in commands})>=2,'multiple_commands_required')
    return rows,images,sources,policy,camera


def evaluate(rows,images):
    require(len(rows)==len(images) and len(rows)>=3,'aligned_examples_required')
    x=np.asarray([image_features(im,r['inputs']['command']) for r,im in zip(rows,images)])
    y=np.asarray([r['targets']['sensorDeltaDeg'] for r in rows],dtype=float)
    require(np.isfinite(y).all(),'finite_labels_required')
    preds=[];folds=[]
    for i,row in enumerate(rows):
        train=[j for j in range(len(rows)) if j!=i]
        model=fit(x[train],y[train],rows[train[0]]['inputs']['command'],images[0].shape)
        as_multi(model,[rows[j] for j in train])
        values=dict(imageRidge=float(raw_predict(model,x[i:i+1])[0]),trainingMean=float(y[train].mean()),noChange=0.)
        preds.append(dict(sessionId=row['sessionId'],command=copy.deepcopy(row['inputs']['command']),actualDeltaDeg=float(y[i]),
                          predictedDeltaDeg=values,residualDeg={k:v-float(y[i]) for k,v in values.items()},sensorObservation=sensor_uncertainty(row)))
        folds.append(dict(heldOutSessionId=row['sessionId'],trainingSessionIds=[rows[j]['sessionId'] for j in train],model=model))
    model=fit(x,y,rows[0]['inputs']['command'],images[0].shape)
    as_multi(model,rows)
    return dict(model=model,diagnostics=dict(kind='leave_one_record_out_sequential_conditions_exploratory',independentTest=False,
        hyperparameterSelection=False,metrics={k:scores(y,[p['predictedDeltaDeg'][k] for p in preds]) for k in ('noChange','trainingMean','imageRidge')},
        predictions=preds,folds=folds))


def predict(model,image,command):
    command_features(command)
    require(command in model['supportedCommands'],'command_outside_training_support')
    require(list(image.shape)==model['imageShape'],'image_shape_changed')
    return float(raw_predict(model,[image_features(image,command)])[0])


def run(pairs):
    rows,images,sources,policy,camera=prepare(pairs);result=evaluate(rows,images)
    result.update(schemaVersion=SCHEMA,trainingPerformed=True,independentTest=False,controlEligible=False,
        sourceDatasets=sources,labelPolicy=policy,cameraIdentity=camera,target='M4_sensor_delta_degrees_not_independent_visual_angle',
        limitations=['one_record_per_condition_and_sequential_pose_confounded','unseen_commands_in_each_holdout',
                     'no_independent_test_or_control','no_calibrated_prediction_interval','downsampling_may_lose_small_motion'],
        reproducibility=dict(python=platform.python_version(),numpy=np.__version__,opencv=cv2.__version__,codeSha256=digest(Path(__file__).read_bytes())))
    result['model'].update(sourceDatasets=sources,labelPolicy=policy,cameraIdentity=camera)
    return result


def export(pairs,output):
    store=ArtifactStore(Path(output));result=run(pairs)
    store.write_json('model.json',result['model']);store.write_json('development-evaluation.json',result)
    return result
