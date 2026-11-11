"""사람이 판독한 기존 원본 envelope를 검증해 파일 프로토콜로 제출한다.

판독·긍정 판단을 생성하지 않으며 촬영·장치 접근·동작 명령도 수행하지 않는다.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile
import time

from learning.artifacts import _read
from learning.batch_observation import digest, require
from learning import batch_interaction as interaction
from learning.experiment_pipeline import encoded


def _private(repository, path):
    path = Path(path).resolve(strict=True)
    require(path.is_relative_to(repository / '.local'), 'review_private_path_required')
    return path


def _json(path):
    raw = _read(path.parent, path.name)
    require(len(raw) <= 12 * 1024 * 1024, 'review_file_too_large')
    return json.loads(raw), raw


def _publish(path, raw):
    """완성된 파일만 공개하고 이미 있는 응답은 덮어쓰지 않는다."""
    fd, temporary = tempfile.mkstemp(prefix='.review-', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as out:
            out.write(raw)
            out.flush()
            os.fsync(out.fileno())
        os.link(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        os.unlink(temporary)


def submit(repository, plan_path, request_path, review_path, *, operator_reviewed=False, clock=time):
    require(operator_reviewed is True, 'review_explicit_operator_judgment_required')
    repository = Path(repository).resolve(strict=True)
    plan_path, request_path, review_path = [
        _private(repository, p) for p in (plan_path, request_path, review_path)]
    # The runtime copies the envelope here. Relative original paths must keep
    # their meaning when it consumes the response.
    require(review_path.parent == request_path.parent, 'review_source_root_changed')
    plan, plan_raw = _json(plan_path)
    request, request_raw = _json(request_path)
    review, review_raw = _json(review_path)
    before = request.get('schemaVersion') == 'motionbrain.batch_before_review_request.v1'
    stage = 'before' if before else 'after'
    require(request_path.name == f'batch-{stage}-review-request.json', 'review_request_filename')
    require(review.get('stage') == stage and review.get('conditionId') == request.get('conditionId') and
            review.get('context') == request.get('context') and
            review.get('outputSequence') == request.get('outputSequence'), 'review_request_binding_changed')
    conditions = [c for c in plan['conditions'] if c['id'] == request['conditionId']]
    require(len(conditions) == 1, 'review_condition_not_unique')
    condition = conditions[0]
    binding = dict(plan['runtime'].get('conditionBindings', {}).get(condition['id'], {}))
    binding.update(interaction=review['interaction'], reviewPath=str(review_path.relative_to(repository)),
                   reviewSha256=interaction.sha(review_raw))
    if before:
        require(set(request) == {'schemaVersion','conditionId','command','sessionId','context',
                'controllerBootId','outputSequence','sourcePath','legacySnapshot','frames'}, 'review_before_request_schema')
        require(condition['command'] == request['command'] and
                request['controllerBootId'] == plan['controllerBootId'] and
                request['controllerBootId'] == review['controllerBootId'] and
                request['sessionId'] == review['scope']['hostClockId'] and
                Path(request['sourcePath']).resolve() == request_path.parent, 'review_before_source_changed')
        expected = plan['runtime'].get('conditionBindings', {}).get(condition['id'], {}).get('interaction')
        require(expected == review['interaction'], 'review_before_interaction_changed')
        response = dict(requestSha256=digest(request), review=review)
    else:
        response = dict(requestSha256=interaction.sha(encoded(request)), review=review)
        interaction.validate_after_response(request, response)
        require(Path(request['source']['path']).resolve() == request_path.parent, 'review_after_source_changed')
        sources = [request_path.parent / name for name in ('session.json', 'result.json')]
        require(any(p.is_file() and interaction.sha(_read(p.parent, p.name)) == request['source']['sha256']
                    for p in sources), 'review_after_source_hash_changed')
    checked = interaction.validate_binding(plan, condition, binding, repository,
                                           purpose='dispatch' if before else 'label')
    token = interaction.verify_review(checked, now=clock.monotonic(),
        output_sequence=request['outputSequence'], after_off=None if before else request['offAt'])
    if before:
        for camera in ('external', 'gripper'):
            allowed = {(f['frameId'], f['pixelSha256']) for f in request['frames'][camera]}
            actual = token.report.get('frameBindings', {}).get(camera, [])
            require(bool(actual) and all((f['frameId'], f['pixelSha256']) in allowed for f in actual),
                    'review_verified_frame_not_requested')
    else:
        interaction.verify_after_frame_membership(token, request)
    # Recheck the sealed evidence and inputs immediately before publication.
    interaction.before_run_cached(token, clock.monotonic(), clock.time(), request['outputSequence'])
    for path, raw in ((plan_path, plan_raw), (request_path, request_raw), (review_path, review_raw)):
        require(_read(path.parent, path.name) == raw, 'review_submission_input_changed')
    response_path = request_path.with_name(f'batch-{stage}-review-response.json')
    raw = encoded(response)
    require(len(raw) <= 1024 * 1024, 'review_response_too_large')
    _publish(response_path.with_suffix('.submission.json'), encoded(dict(
        requestSha256=response['requestSha256'], responseSha256=interaction.sha(raw),
        operatorReviewed=True, physicalJudgmentGenerated=False, outcome=review['outcome'])))
    _publish(response_path, raw)
    return dict(status='submitted', responsePath=str(response_path),
                operatorReviewed=True, physicalJudgmentGenerated=False, outcome=review['outcome'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--plan', required=True)
    parser.add_argument('--request', required=True)
    parser.add_argument('--review', required=True)
    parser.add_argument('--operator-reviewed', action='store_true')
    args = parser.parse_args()
    try:
        result = submit(args.repository, args.plan, args.request, args.review,
                        operator_reviewed=args.operator_reviewed)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, '검토 제출 실패: ' + type(exc).__name__ + ': ' + str(exc) + '\n')
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()
