"""Validate and load an immutable relative-state model without fitting it."""
from __future__ import annotations

import copy
import hashlib
import math
from pathlib import Path
import re

from learning.artifacts import _decode_json
from learning.manual_collection_dataset import require
from learning.relative_action_learning import SCHEMA, STATE, command_key


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def _array(value, size):
    require(isinstance(value, list) and len(value) == size
            and all(type(v) in (int, float) and math.isfinite(v) for v in value),
            'finite_model_parameters_required')
    return value


def _unique_strings(value, reason):
    require(isinstance(value, list) and bool(value)
            and all(isinstance(v, str) and bool(v) for v in value)
            and len(value) == len(set(value)), reason)


def validate_model(model):
    """Check the saved v1 contract; a hash is not an original-source audit."""
    require(isinstance(model, dict) and model.get('schemaVersion') == SCHEMA
            and model.get('controlEligible') is False and model.get('independentTest') is False
            and type(model.get('alpha')) in (int, float) and model['alpha'] == 1.,
            'offline_relative_model_required')
    context = model.get('context')
    require(isinstance(context, dict) and set(context) == {
        'physicalView', 'object', 'poseRegime', 'landmarks', 'cameraIdentity', 'imageShape', 'stateFields'},
        'complete_observation_context_required')
    require(all(isinstance(context[k], str) and context[k] for k in
                ('physicalView', 'object', 'poseRegime', 'landmarks'))
            and isinstance(context['cameraIdentity'], dict) and bool(context['cameraIdentity'])
            and isinstance(context['imageShape'], list) and len(context['imageShape']) == 2
            and all(type(v) is int and v > 0 for v in context['imageShape'])
            and context['stateFields'] == STATE, 'complete_observation_context_required')
    for field in ('trainingSessionIds', 'trainingCaptureGroups', 'trainingFrameIds'):
        _unique_strings(model.get(field), 'model_training_provenance_required')
    episodes = model.get('trainingEpisodes')
    require(isinstance(episodes, list) and len(episodes) == len(model['trainingSessionIds'])
            and all(isinstance(e, list) and len(e) == 2
                    and all(isinstance(v, str) and v for v in e) for e in episodes)
            and len({tuple(e) for e in episodes}) == len(episodes)
            and len(model['trainingFrameIds']) == 2 * len(episodes),
            'model_training_provenance_required')
    require(isinstance(model.get('models'), list), 'model_commands_required')
    keys, total = set(), 0
    for entry in model['models']:
        require(isinstance(entry, dict), 'model_command_parameters_required')
        key = command_key(entry.get('command'))
        require(key not in keys, 'duplicate_model_command')
        keys.add(key)
        require(type(entry.get('count')) is int and entry['count'] >= 2,
                'two_training_transitions_required')
        total += entry['count']
        for name in ('mean', 'scale', 'minimum', 'maximum', 'deltaMean'):
            _array(entry.get(name), 3)
        require(all(s >= 1 for s in entry['scale'])
                and all(lo <= hi and lo - 1e-9 <= mean <= hi + 1e-9
                        for lo, mean, hi in zip(entry['minimum'], entry['mean'], entry['maximum'])),
                'invalid_model_normalization_or_range')
        matrix = entry.get('coefficient')
        require(isinstance(matrix, list) and len(matrix) == 3, 'model_coefficient_shape')
        for row in matrix:
            _array(row, 3)
    require(total <= len(episodes), 'model_support_exceeds_training_sources')
    return copy.deepcopy(model)


def load_frozen_model(path, expected_sha256):
    require(isinstance(expected_sha256, str)
            and re.fullmatch('[0-9a-f]{64}', expected_sha256) is not None, 'model_sha256_required')
    raw = Path(path).read_bytes()
    require(sha256(raw) == expected_sha256, 'frozen_model_hash_changed')
    return validate_model(_decode_json(raw))
