#!/usr/bin/env python3
"""Collect one bounded, read-only controller HTTP diagnostic; never retry."""
import argparse
import datetime
import json
import math
from pathlib import Path
import subprocess
import time
from urllib.parse import urlsplit

ENDPOINTS = ('status', 'sequence', 'actions', 'action_outputs')
POLICY = {'connectTimeoutSeconds': 0.8, 'requestTimeoutSeconds': 2.0,
          'sessionBudgetSeconds': 12.0, 'diagnosticLatencyLimitSeconds': 0.8,
          'maxResponseBytes': 65536, 'maxRequests': 4, 'retries': 0,
          'cameraRequests': 0, 'controlRequests': 0}
METRICS = '\n'.join(('%{http_code}', '%{time_connect}', '%{time_starttransfer}',
                     '%{time_total}', '%{size_download}'))


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def validate_target(value):
    parsed = urlsplit(value)
    if (parsed.scheme != 'http' or not parsed.hostname or parsed.username is not None
            or parsed.password is not None or parsed.path not in ('', '/')
            or parsed.query or parsed.fragment or any(c.isspace() for c in value)
            or any(c in value for c in ('\\', '{', '}', '[', ']'))):
        raise ValueError('target must be a plain HTTP hostname or IPv4 URL, without credentials or a path')
    # Evaluating port also rejects invalid/out-of-range ports before any request.
    parsed.port
    return value.rstrip('/')


def parse_metrics(raw):
    lines = raw.decode('ascii').strip().splitlines()
    if len(lines) != 5 or len(lines[0]) != 3 or not lines[0].isdigit():
        raise ValueError('invalid curl metrics')
    values = [float(v) for v in lines[1:]]
    if not all(math.isfinite(v) and v >= 0 for v in values):
        raise ValueError('nonfinite or negative curl metrics')
    return dict(zip(('httpCode', 'connectSeconds', 'firstByteSeconds',
                     'totalSeconds', 'downloadBytes'), [int(lines[0]), *values]))


def idle_error(endpoint, payload):
    if not isinstance(payload, dict):
        return 'payload_not_object'
    if endpoint == 'status':
        motors = payload.get('motors', {})
        if (payload.get('state') != 'IDLE' or payload.get('motorEnabled') is not False
                or not isinstance(motors, dict) or set(motors) != {'M1','M2','M3','M4','M5'}
                or any(not isinstance(m, dict) or m.get('speed') != 0
                       or m.get('enabled') is not False for m in motors.values())):
            return 'controller_not_confirmed_idle'
    elif endpoint == 'sequence':
        sequence = payload.get('sequence', {})
        if (not isinstance(sequence, dict) or sequence.get('state') != 'IDLE'
                or sequence.get('totalCount') != 0):
            return 'sequence_not_confirmed_empty'
    elif endpoint == 'actions':
        if (payload.get('active') is not False or payload.get('enabled') is not False
                or not isinstance(payload.get('bootId'), str) or not payload['bootId']):
            return 'actions_not_confirmed_inactive'
    elif endpoint == 'action_outputs':
        if not isinstance(payload.get('bootId'), str) or not isinstance(payload.get('entries'), list):
            return 'output_evidence_invalid'
    return None


def collect(target, output):
    target = validate_target(target)
    output = Path(output).resolve()
    output.mkdir(mode=0o700, parents=True, exist_ok=False)  # Closed sessions cannot be resumed.
    plan = {'target': target, 'endpoints': ENDPOINTS, 'policy': POLICY,
            'createdAt': datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'scope': 'HTTP observation only; no motion safety or camera timing qualification'}
    write_json(output / 'plan.json', plan)
    start = time.monotonic()
    report = {'complete': False, 'requests': [], 'failure': None,
              'trainingEligible': False, 'motionQualified': False}
    boot_id = None
    pending_row = None
    try:
        for ordinal, endpoint in enumerate(ENDPOINTS, 1):
            remaining = POLICY['sessionBudgetSeconds'] - (time.monotonic() - start)
            if remaining <= 0:
                report['failure'] = 'session_budget_exhausted'
                break
            folder = output / f'{ordinal:02d}-{endpoint}'
            folder.mkdir()
            row = {'endpoint': endpoint, 'startedMonotonic': time.monotonic(),
                   'startedAt': datetime.datetime.now(datetime.timezone.utc).isoformat()}
            report['requests'].append(row)
            pending_row = row
            write_json(folder / 'attempt.json', row)
            cmd = ['curl', '--disable', '--globoff', '--noproxy', '*', '--proto', '=http',
                   '--connect-timeout', str(POLICY['connectTimeoutSeconds']),
                   '--max-time', str(min(POLICY['requestTimeoutSeconds'], remaining)),
                   '--max-filesize', str(POLICY['maxResponseBytes']),
                   '--silent', '--show-error', '--output', str(folder / 'response.body'),
                   '--write-out', METRICS, target + '/' + endpoint]
            # Files, rather than JSON interpolation, preserve HTTP 000 and partial failures.
            with (folder / 'curl.stdout').open('wb') as stdout, (folder / 'curl.stderr').open('wb') as stderr:
                try:
                    remaining = POLICY['sessionBudgetSeconds'] - (time.monotonic() - start)
                    if remaining <= 0:
                        row['failure'] = 'session_budget_exhausted'
                    else:
                        cmd[cmd.index('--max-time') + 1] = str(min(POLICY['requestTimeoutSeconds'], remaining))
                        row['requestStartedMonotonic'] = time.monotonic()
                        result = subprocess.run(cmd, stdout=stdout, stderr=stderr, timeout=remaining, check=False)
                        row['curlExit'] = result.returncode
                except KeyboardInterrupt:
                    row['failure'] = 'interrupted'
                except subprocess.TimeoutExpired:
                    row['failure'] = 'session_deadline'
                except OSError as exc:
                    row['failure'] = 'curl_unavailable'
                    row['error'] = str(exc)
            row['finishedMonotonic'] = time.monotonic()
            if not row.get('failure'):
                try:
                    row['metrics'] = parse_metrics((folder / 'curl.stdout').read_bytes())
                except (ValueError, UnicodeError) as exc:
                    row['metricsError'] = str(exc)
                if row['curlExit'] != 0:
                    row['failure'] = 'transport_error'
                elif 'metrics' not in row:
                    row['failure'] = 'metrics_invalid'
                elif row['metrics']['httpCode'] != 200:
                    row['failure'] = 'http_error'
                elif row['metrics']['totalSeconds'] > POLICY['diagnosticLatencyLimitSeconds']:
                    row['failure'] = 'diagnostic_latency_exceeded'
                else:
                    try:
                        body = (folder / 'response.body').read_bytes()
                        if len(body) > POLICY['maxResponseBytes']:
                            raise ValueError('response exceeds byte limit')
                        payload = json.loads(body)
                        row['failure'] = idle_error(endpoint, payload)
                        if not row['failure'] and endpoint == 'actions':
                            boot_id = payload['bootId']
                        if not row['failure'] and endpoint == 'action_outputs' and payload['bootId'] != boot_id:
                            row['failure'] = 'controller_boot_changed'
                    except (OSError, ValueError, UnicodeError) as exc:
                        row['failure'] = 'payload_invalid'
                        row['payloadError'] = str(exc)
            if time.monotonic() - start > POLICY['sessionBudgetSeconds']:
                row['failure'] = row.get('failure') or 'session_budget_exhausted'
            write_json(folder / 'result.json', row)
            pending_row = None
            write_json(output / 'result.json', report)
            if row.get('failure'):
                report['failure'] = row['failure']
                break
        else:
            report['complete'] = True
    except KeyboardInterrupt:
        report['failure'] = 'interrupted'
    finally:
        if pending_row is not None:
            row = pending_row
            row.setdefault('finishedMonotonic', time.monotonic())
            row['failure'] = report['failure'] or 'collection_incomplete'
            write_json(folder / 'result.json', row)
        report['elapsedSeconds'] = time.monotonic() - start
        write_json(output / 'result.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', required=True, help='http://controller-host[:port]')
    parser.add_argument('--output', type=Path, required=True, help='New private evidence directory, e.g. .local/http-diagnostic/session-01')
    args = parser.parse_args(argv)
    try:
        result = collect(args.target, args.output)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(json.dumps({'complete': result['complete'], 'failure': result['failure'],
                      'requests': len(result['requests']), 'motionQualified': False}))
    return 0 if result['complete'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
