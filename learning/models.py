"""Small deterministic NumPy/CPU baselines. No hyperparameter selection."""
import numpy as np
from learning.features import INPUT_NAMES, TARGET_NAMES, FEATURE_VERSION, TARGET_UNITS

RIDGE_ALPHA = 1.0  # Fixed before observing validation or test outcomes.


def matrix(values, columns):
    raw = np.asarray(values)
    if raw.dtype.kind not in 'iuf' or raw.ndim != 2 or raw.shape[1] != columns or not np.isfinite(raw).all():
        raise ValueError('finite numeric matrix with exact feature columns required')
    return raw.astype(np.float64)


def fit_linear(x, y):
    x, y = matrix(x,len(INPUT_NAMES)), matrix(y,len(TARGET_NAMES))
    if not len(x) or len(x)!=len(y):
        raise ValueError('nonempty aligned train matrices required')
    mean, scale = x.mean(axis=0), x.std(axis=0)
    scale[scale<1e-12] = 1.
    z = (x-mean)/scale
    intercept = y.mean(axis=0)
    coefficients = np.linalg.solve(z.T@z + RIDGE_ALPHA*np.eye(z.shape[1]), z.T@(y-intercept))
    result = {'schemaVersion':'motionbrain.linear_dynamics.v1','featureVersion':FEATURE_VERSION,
              'inputNames':INPUT_NAMES,'targetNames':TARGET_NAMES,'targetUnits':TARGET_UNITS,
              'alpha':RIDGE_ALPHA,'fitRows':len(x),'preprocessingFitSplit':'train',
              'mean':mean.tolist(),'scale':scale.tolist(),'intercept':intercept.tolist(),
              'coefficients':coefficients.tolist()}
    predict_linear(result,x)  # Reject numerical overflow rather than save it.
    return result


def predict_linear(model, x):
    if not isinstance(model, dict):
        raise ValueError('model must be a JSON object')
    if (model.get('schemaVersion')!='motionbrain.linear_dynamics.v1' or
            model.get('featureVersion')!=FEATURE_VERSION or model.get('inputNames')!=INPUT_NAMES or
            model.get('targetNames')!=TARGET_NAMES or model.get('targetUnits')!=TARGET_UNITS or
            model.get('alpha')!=RIDGE_ALPHA or model.get('preprocessingFitSplit')!='train'):
        raise ValueError('model schema/columns/policy mismatch')
    x = matrix(x,len(INPUT_NAMES))
    mean = matrix([model['mean']],len(INPUT_NAMES))[0]
    scale = matrix([model['scale']],len(INPUT_NAMES))[0]
    intercept = matrix([model['intercept']],len(TARGET_NAMES))[0]
    coef = matrix(model['coefficients'],len(TARGET_NAMES))
    if coef.shape[0]!=len(INPUT_NAMES) or np.any(scale<=0):
        raise ValueError('model dimensions/scale invalid')
    result = (x-mean)/scale@coef + intercept
    if not np.isfinite(result).all(): raise ValueError('nonfinite prediction')
    return result


def metrics(y, prediction):
    y, prediction = matrix(y,len(TARGET_NAMES)), matrix(prediction,len(TARGET_NAMES))
    if y.shape!=prediction.shape: raise ValueError('prediction shape mismatch')
    if not len(y): return {'status':'unavailable','n':0,'reason':'empty_split'}
    error = prediction-y
    error[:,-1] = (error[:,-1]+180)%360-180
    return {'status':'evaluated','n':len(y), 'mae':dict(zip(TARGET_NAMES,np.mean(np.abs(error),axis=0).tolist())),
            'rmse':dict(zip(TARGET_NAMES,np.sqrt(np.mean(error**2,axis=0)).tolist()))}


def compare(model, x, y):
    y = matrix(y,len(TARGET_NAMES))
    return {'noChange':metrics(y,np.zeros_like(y)), 'linear':metrics(y,predict_linear(model,x))}
