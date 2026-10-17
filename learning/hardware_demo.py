"""Offline, immutable archives of manually observed hardware demonstrations.

No controller/camera access or training admission. Input directories are private
operator evidence, not a trusted account of physical success or exposure times.
"""
from pathlib import Path
import os
import re
import stat

from learning.artifacts import ArtifactStore, StorageError, _decode_json, verify_artifacts

SCHEMA = 'motionbrain.hardware_demo.v1'
POLICY = {'maxSourceAgeMs': 100, 'maxAnchorAgeMs': 15000, 'maxPersistenceMs': 250}
FILES = ('plan.json', 'execution/result.json', 'after.png', 'after.jpg', 'camera.jpg', 'camera-receipt.json')
MAX_BYTES = 16 * 1024 * 1024
CHECKS = {'sourceAge': 'unknown', 'anchorAge': 'unknown', 'sourceCapturePersistence': 'unknown'}
TIMING = {'images': 'unsynchronized_observation_snapshots',
          'controllerEvents': 'source_reported_host_monotonic_and_device_uptime',
          'sourceExposureTimeVerified': False}
STEP = re.compile(r'step-([0-9]{2})\Z')


def _read_file(path):
    # Parent directories and leaf are checked; input must remain unchanged while archiving.
    if any(p.is_symlink() for p in (path, *path.parents)):
        raise StorageError('symlink source is not permitted')
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, 'rb') as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise StorageError('source must be a regular file')
        data = stream.read(MAX_BYTES + 1)
    if not data or len(data) > MAX_BYTES:
        raise StorageError('source is empty or exceeds 16 MiB')
    return data


def _json(raw):
    value = _decode_json(raw)
    if not isinstance(value, dict):
        raise StorageError('source JSON must be an object')
    # Do not copy arbitrary runtime directories, headers, credentials or scripts.
    def visit(item):
        if isinstance(item, dict):
            for key, child in item.items():
                normalized = re.sub('[^a-z]', '', key.lower())
                if any(word in normalized for word in ('token', 'password', 'authorization', 'secret', 'cookie')):
                    raise StorageError('credential-like key in source JSON')
                visit(child)
        elif isinstance(item, list):
            for child in item:
                visit(child)
    visit(value)
    return value


def archive_demo(source, destination, *, observation='unknown', note=''):
    """Snapshot a single phase, preserving missing/failed attempts explicitly.

    observation is an operator annotation, never inferred from motor outputs.
    There is no resume or append; errors leave an incomplete archive without its
    terminal demo.json. Nothing in this format is eligible for training/control.
    """
    if observation not in ('unknown', 'grasped', 'placed', 'unsuccessful'):
        raise ValueError('invalid observation')
    if not isinstance(note, str) or len(note) > 2000:
        raise ValueError('note must be at most 2000 characters')
    source = Path(source).absolute()
    if not source.is_dir() or any(p.is_symlink() for p in (source, *source.parents)):
        raise StorageError('source must be a real directory')
    steps = sorted(p for p in source.iterdir() if STEP.fullmatch(p.name))
    if not steps:
        raise StorageError('no step directories')
    expected = [f'step-{n:02d}' for n in range(1, len(steps) + 1)]
    issues = [] if [p.name for p in steps] == expected else ['step_number_gap']
    store = ArtifactStore(Path(destination))
    attempts = []
    descriptors = []
    for step in steps:
        if not step.is_dir() or step.is_symlink():
            raise StorageError('step must be a real directory')
        assets, missing, result = {}, [], None
        for relative in FILES:
            path = step / relative
            if not path.exists() and not path.is_symlink():
                missing.append(relative)
                continue
            raw = _read_file(path)
            if relative.endswith('.json'):
                value = _json(raw)
                if relative == 'execution/result.json':
                    result = value
            elif relative.endswith('.png') and not raw.startswith(b'\x89PNG\r\n\x1a\n'):
                raise StorageError('invalid PNG signature')
            elif relative.endswith('.jpg') and not (raw.startswith(b'\xff\xd8') and raw.endswith(b'\xff\xd9')):
                raise StorageError('invalid JPEG signature')
            descriptor = store._write(f'{step.name}/{relative}', raw)
            assets[relative] = descriptor
            descriptors.append(descriptor)
        attempts.append({
            'step': step.name, 'artifacts': assets, 'missing': missing,
            'reportedFailure': None if result is None else result.get('failure'),
            'reportedFinalIdle': None if result is None else result.get('finalIdleVerified'),
            'executionRecordPresent': result is not None,
            'trainingEligible': False, 'controlEligible': False,
        })
    errors = verify_artifacts(store.root, descriptors)
    if errors:
        raise StorageError('archive readback validation failed')
    manifest = {
        'schemaVersion': SCHEMA, 'purpose': 'raw_hardware_demonstration',
        'archiveComplete': True, 'attempts': attempts, 'sourceIssues': issues,
        'observation': {'label': observation, 'note': note, 'basis': 'operator_visual_annotation',
                        'evidenceRefs': [a['artifacts'][name]['artifactRef']
                                         for a in attempts for name in ('after.png', 'after.jpg')
                                         if name in a['artifacts']]},
        'sourceRecordsComplete': not issues and all(
            {'plan.json', 'execution/result.json'} <= set(a['artifacts'])
            and bool({'after.png', 'after.jpg'} & set(a['artifacts'])) for a in attempts),
        'qualityChecks': dict(CHECKS),
        'qualityPolicy': dict(POLICY), 'trainingEligible': False, 'controlEligible': False,
        'eligibilityReasons': ['manual_lease_not_bounded_action', 'optical_capture_time_unverified',
                               'semantic_anchor_unverified', 'transition_persistence_unverified'],
        'timing': dict(TIMING),
    }
    try:
        store.write_json('demo.json', manifest)  # terminal marker after asset readbacks
    except BaseException:
        # _write may have linked the file before its fsync/readback failed.
        (store.root / 'demo.json').unlink(missing_ok=True)
        directory = os.open(store.root, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        raise
    return manifest


def audit_demo(root):
    """Check saved bytes and raw-only contract; does not certify scene labels."""
    root = Path(root).absolute()
    manifest = _json(_read_file(root / 'demo.json'))
    errors = []
    for key, value in (('schemaVersion', SCHEMA), ('archiveComplete', True),
                       ('purpose', 'raw_hardware_demonstration'),
                       ('trainingEligible', False), ('controlEligible', False),
                       ('qualityPolicy', POLICY), ('qualityChecks', CHECKS), ('timing', TIMING)):
        if type(manifest.get(key)) is not type(value) or manifest.get(key) != value:
            errors.append('invalid_' + key)
    attempts = manifest.get('attempts')
    if not isinstance(attempts, list) or not attempts:
        return errors + ['invalid_attempts']
    seen = set()
    for attempt in attempts:
        if not isinstance(attempt, dict) or not isinstance(attempt.get('artifacts'), dict):
            errors.append('invalid_attempt')
            continue
        if attempt.get('trainingEligible') is not False or attempt.get('controlEligible') is not False:
            errors.append('attempt_eligibility_changed')
        step = attempt.get('step')
        if not isinstance(step, str) or not STEP.fullmatch(step) or step in seen:
            errors.append('invalid_step')
            continue
        seen.add(step)
        assets, missing = attempt['artifacts'], attempt.get('missing')
        if (not isinstance(missing, list) or not all(isinstance(x, str) for x in missing)
                or len(missing) != len(set(missing)) or set(assets) & set(missing)
                or set(assets) | set(missing) not in (set(FILES), set(FILES) - {'after.jpg'})):
            errors.append('invalid_file_inventory')
        for name, descriptor in assets.items():
            if not isinstance(descriptor, dict) or descriptor.get('artifactRef') != f'{step}/{name}':
                errors.append('invalid_artifact_mapping')
        errors.extend(verify_artifacts(root, list(assets.values())))
    ordered = [a.get('step') for a in attempts if isinstance(a, dict)]
    expected = [f'step-{n:02d}' for n in range(1, len(attempts) + 1)]
    expected_issues = [] if ordered == expected else ['step_number_gap']
    if manifest.get('sourceIssues') != expected_issues:
        errors.append('invalid_sourceIssues')
    complete = not expected_issues and all(
        isinstance(a, dict) and isinstance(a.get('artifacts'), dict)
        and {'plan.json', 'execution/result.json'} <= set(a['artifacts'])
        and bool({'after.png', 'after.jpg'} & set(a['artifacts']))
        for a in attempts)
    if manifest.get('sourceRecordsComplete') is not complete:
        errors.append('invalid_sourceRecordsComplete')
    return errors
