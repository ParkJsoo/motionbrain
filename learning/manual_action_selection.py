"""Rank predicted M4 actions offline; results never authorize control."""
import copy
import math

from learning.manual_collection_dataset import require
from learning.manual_collection_learning import command_features
from learning.manual_multi_learning import predict


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


def rank_predictions(predictions, target_delta_deg):
    """Rank predicted signed sensor changes and a no-op against a target.

    Equal errors prefer no-op, then the original candidate order. Predicted
    counterfactuals are estimates; their actual outcomes are not observed.
    """
    require(finite_number(target_delta_deg) and target_delta_deg <= 0, 'nonpositive_finite_target_required')
    require(isinstance(predictions, list) and len(predictions) > 0,
            'candidates_required')
    ranked = [dict(kind='stop', command=None, predictedDeltaDeg=0.0,
                   predictedResidualDeg=-float(target_delta_deg))]
    seen = []
    for entry in predictions:
        require(isinstance(entry, dict) and set(entry) == {'command', 'predictedDeltaDeg'},
                'prediction_fields')
        command = entry['command']
        command_features(command)
        require(command not in seen, 'duplicate_command')
        seen.append(copy.deepcopy(command))
        value = entry['predictedDeltaDeg']
        require(finite_number(value), 'finite_prediction_required')
        residual = value - target_delta_deg
        require(math.isfinite(residual), 'finite_prediction_required')
        ranked.append(dict(kind='action', command=copy.deepcopy(command),
                           predictedDeltaDeg=float(value), predictedResidualDeg=float(residual)))
    ranked.sort(key=lambda item: abs(item['predictedResidualDeg']))
    return dict(candidates=ranked, selected=copy.deepcopy(ranked[0]),
                targetDeltaDeg=float(target_delta_deg), controlEligible=False,
                counterfactualOutcomesObserved=False)


def rank_actions(model, image, target_delta_deg, candidates):
    """Predict with a fixed offline ridge model, accepting only supported actions."""
    require(finite_number(target_delta_deg) and target_delta_deg <= 0, 'nonpositive_finite_target_required')
    require(isinstance(model, dict) and model.get('controlEligible') is False,
            'offline_model_required')
    require(isinstance(candidates, list) and len(candidates) > 0, 'candidates_required')
    predictions = [dict(command=command, predictedDeltaDeg=predict(model, image, command))
                   for command in candidates]
    return rank_predictions(predictions, target_delta_deg)
