"""Strict physical-manifest loader and reproducible offline training/evaluation.

The synthetic smoke deliberately has a separate entry point and artifact schema.
"""
import hashlib
import json
from pathlib import Path
import platform
import numpy as np

from learning.artifacts import describe_artifact, read_json_artifact
from learning.dataset import validate_manifest
from learning.features import FEATURE_VERSION, INPUT_NAMES, TARGET_NAMES, transition_features
from learning.models import fit_linear, compare
from learning.provenance import synthetic_reasons
from learning.schema import validate_transition
from learning.splits import LABELS, validate_splits

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ['learning/features.py','learning/models.py','learning/visual_dynamics.py',
           'learning/dataset.py','learning/splits.py','learning/schema.py','learning/provenance.py',
           'learning/artifacts.py','tools/train_visual_dynamics.py',
           'tools/evaluate_visual_dynamics.py','tools/smoke_visual_dynamics.py']


class DatasetUnavailable(ValueError):
    def __init__(self, reason, validation=None):
        super().__init__(reason)
        self.validation = validation


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def read_json(path):
    path = Path(path).absolute()
    return read_json_artifact(path.parent,describe_artifact(path.parent,path.name))


def load_model(path):
    path = Path(path).absolute()
    result = read_json(path.parent/'result.json')
    if not isinstance(result, dict):
        raise ValueError('trained receipt must be a JSON object')
    if (result.get('status') != 'trained' or not isinstance(result.get('model'), dict) or
            result['model'].get('artifactRef') != path.name):
        raise ValueError('trained model receipt required; synthetic smoke is not a physical model')
    artifact = read_json_artifact(path.parent,result['model'])
    if not isinstance(artifact, dict) or artifact.get('metadata') != result.get('metadata'):
        raise ValueError('model metadata/receipt mismatch')
    return artifact


def source_hashes():
    return {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}


def load_dataset(path):
    manifest = read_json(path)
    validation = validate_manifest(manifest)  # Reopens originals, journal, provenance and assignments.
    if not validation['valid']:
        raise DatasetUnavailable('invalid_manifest_or_originals',validation)
    if not validation['readyForTraining']:
        raise DatasetUnavailable('no_eligible_physical_data_or_empty_split',validation)
    rows = [r for r in manifest['rows'] if r['eligible']]
    if validate_splits(rows,manifest['assignments']):
        raise DatasetUnavailable('split_leakage',validation)
    data = {k:{'sampleIds':[],'x':[],'y':[]} for k in LABELS}
    for row in rows:
        if row['provenance']!='physical':
            raise DatasetUnavailable('synthetic_or_unknown_row',validation)
        summary = read_json_artifact(Path(row['sourceRoot']),row['summary'])
        transition = summary['transition']
        check = validate_transition(transition)
        if synthetic_reasons(summary) or not check['trainingEligible']:
            raise DatasetUnavailable('transition_not_physical_training_eligible',validation)
        try:
            x,y = transition_features(transition)
        except (KeyError,TypeError,ValueError) as exc:
            raise DatasetUnavailable(f"invalid_features:{row['sampleId']}:{exc}",validation) from exc
        entry = data[manifest['assignments'][row['sampleId']]]
        entry['sampleIds'].append(row['sampleId']); entry['x'].append(x); entry['y'].append(y)
    for entry in data.values():
        entry['x'] = np.asarray(entry['x'],dtype=float).reshape(-1,len(INPUT_NAMES))
        entry['y'] = np.asarray(entry['y'],dtype=float).reshape(-1,len(TARGET_NAMES))
    metadata = {'manifestSha256':digest(manifest),'splitSha256':digest(manifest['assignments']),
                'seed':manifest['seed'],'featureVersion':FEATURE_VERSION,
                'inventoryRows':len(manifest['rows']),'eligibleRows':len(rows),
                'excludedRows':manifest['excludedCount'],'splitCounts':manifest['splitCounts'],
                'trainDataSha256':digest({k:v.tolist() if isinstance(v,np.ndarray) else v
                                         for k,v in data['train'].items()}),
                'sourceSha256':source_hashes(),'numpyVersion':np.__version__,'pythonVersion':platform.python_version()}
    return data,metadata


def train(path):
    data,metadata = load_dataset(path)
    model = fit_linear(data['train']['x'],data['train']['y'])
    # A training run never evaluates the test set. Alpha is fixed, no search.
    return {'schemaVersion':'motionbrain.physical_dynamics_artifact.v1','evidence':'physical_manifest',
            'metadata':metadata,'model':model,'controlEligible':False,
            'physicalManipulationImprovementEstablished':False}


def evaluate(path, artifact):
    data,metadata = load_dataset(path)
    if not isinstance(artifact, dict):
        raise ValueError('model artifact must be a JSON object')
    if (artifact.get('schemaVersion')!='motionbrain.physical_dynamics_artifact.v1' or
            artifact.get('evidence')!='physical_manifest' or artifact.get('metadata')!=metadata):
        raise ValueError('model/manifest/split/source/runtime mismatch; smoke models cannot be evaluated as physical')
    model = artifact['model']
    if not isinstance(model, dict):
        raise ValueError('linear model must be a JSON object')
    if model.get('fitRows')!=len(data['train']['x']): raise ValueError('train denominator mismatch')
    result = {label:{'sampleIds':entry['sampleIds'],**compare(model,entry['x'],entry['y'])}
              for label,entry in data.items()}
    return {'schemaVersion':'motionbrain.physical_dynamics_evaluation.v1','status':'evaluated',
            'metadata':metadata,'modelSha256':digest(artifact),'splits':result,
            'physicalManipulationImprovementEstablished':False,'controlEligible':False}
