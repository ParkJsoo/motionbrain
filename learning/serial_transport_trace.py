"""Bounded Python I/O observations; never UART, USB, or sensor event timestamps."""
from contextlib import contextmanager
from copy import deepcopy
import base64
import hashlib
import re
import time

from learning.mac_clock_bridge import finite

SCHEMA = 'motionbrain.serial_transport_trace.v1'
MAX_READS = 256
KINDS = ('write', 'buffer_query', 'read', 'parse')


class TransportTraceLimit(ValueError):
    pass


class TransportTrace:
    def __init__(self, clock=time):
        self.clock = clock
        self.phases = []
        self.reads = 0
        self.active = False

    @contextmanager
    def operation(self, kind, **fields):
        if kind not in KINDS or self.active:
            raise ValueError('trace_operation_invalid')
        if kind == 'read':
            if self.reads >= MAX_READS:
                raise TransportTraceLimit('transport_trace_read_limit')
            self.reads += 1
        phase = dict(fields, kind=kind, startedMonotonic=self.clock.monotonic())
        self.phases.append(phase)
        self.active = True
        try:
            yield phase
        except BaseException as exc:
            phase['errorType'] = type(exc).__name__
            raise
        finally:
            try:
                try:
                    phase['endedMonotonic'] = self.clock.monotonic()
                except BaseException as exc:
                    # A single cancellation may arrive during timestamping,
                    # after I/O returned. Preserve that cancellation and take
                    # one later host observation as the conservative end. This
                    # never repeats an I/O operation. Repeated interruption can
                    # leave an explicitly incomplete phase; never invent time.
                    phase['errorType'] = type(exc).__name__
                    try:
                        phase['endedMonotonic'] = self.clock.monotonic()
                    finally:
                        raise exc
            finally:
                self.active = False

    def snapshot(self):
        if self.active:
            raise ValueError('trace_operation_incomplete')
        return dict(schemaVersion=SCHEMA, phases=deepcopy(self.phases))


def _integer(value, lower, upper):
    return type(value) is int and lower <= value <= upper


def validate_trace(trace, receipt, raw):
    """Validate every saved observation against the original receipt bytes.

    A failed receipt may end at any operation boundary, including a final
    exception. Errors never excuse missing bytes or inconsistent timestamps.
    """
    try:
        return _validate(trace, receipt, raw)
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('transport_trace_shape_invalid') from exc


def _validate(trace, receipt, raw):
    if (not isinstance(trace, dict) or set(trace) != {'schemaVersion', 'phases'}
            or trace['schemaVersion'] != SCHEMA or not isinstance(trace['phases'], list)
            or not isinstance(receipt, dict) or type(raw) is not bytes or len(raw) > 4097):
        raise ValueError('transport_trace_shape_invalid')
    start, finish = receipt['requestStartedMonotonic'], receipt['receivedMonotonic']
    if not all(finite(x) for x in (start, finish)) or not 0 <= start <= finish:
        raise ValueError('transport_trace_receipt_clock_invalid')
    completed = receipt['transportCompleted']
    if type(completed) is not bool:
        raise ValueError('transport_trace_completion_invalid')
    try:
        request = base64.b64decode(receipt['requestBase64'], validate=True)
    except (ValueError, TypeError) as exc:
        raise ValueError('transport_trace_request_invalid') from exc
    phases = trace['phases']
    if len(phases) > 2 * MAX_READS + 3:
        raise ValueError('transport_trace_phase_limit')
    previous = start
    expected = 'write'
    offset = reads = 0
    first = last_nonempty = parse_duration = complete_observed = None
    write_complete = parsed = False
    last_error = False
    allowed = {
        'write': {'requestedBytes', 'returnedBytes'},
        'buffer_query': {'availableBytes'},
        'read': {'requestedBytes', 'returnedBytes', 'rawOffset', 'rawSha256'},
        'parse': {'rawBytes', 'rawSha256'},
    }
    for index, phase in enumerate(phases):
        if not isinstance(phase, dict):
            raise ValueError('transport_trace_phase_invalid')
        if index and not write_complete:
            raise ValueError('transport_trace_after_incomplete_write')
        kind = phase['kind']
        if kind not in KINDS or (kind != expected and not (expected == 'buffer_query' and kind == 'parse' and reads)):
            raise ValueError('transport_trace_order_invalid')
        error = phase.get('errorType')
        if error is not None and (not isinstance(error, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]{0,99}', error)
                                  or index != len(phases) - 1 or completed):
            raise ValueError('transport_trace_error_invalid')
        last_error = error is not None
        a, b = phase['startedMonotonic'], phase['endedMonotonic']
        if not all(finite(x) for x in (a, b)) or not previous <= a <= b <= finish:
            raise ValueError('transport_trace_clock_invalid')
        previous = b
        if set(phase) - (allowed[kind] | {'kind', 'startedMonotonic', 'endedMonotonic', 'errorType'}):
            raise ValueError('transport_trace_fields_invalid')
        if kind == 'write':
            if not _integer(phase['requestedBytes'], 1, 4097) or phase['requestedBytes'] != len(request):
                raise ValueError('transport_trace_write_size_invalid')
            if 'returnedBytes' not in phase:
                if not error: raise ValueError('transport_trace_write_missing')
            elif not _integer(phase['returnedBytes'], 0, phase['requestedBytes']):
                raise ValueError('transport_trace_write_size_invalid')
            write_complete = not error and phase.get('returnedBytes') == phase['requestedBytes']
            expected = 'buffer_query'
        elif kind == 'buffer_query':
            if 'availableBytes' not in phase:
                if not error: raise ValueError('transport_trace_query_missing')
            elif not _integer(phase['availableBytes'], 0, 2**63 - 1):
                raise ValueError('transport_trace_query_invalid')
            expected = 'read'
        elif kind == 'read':
            reads += 1
            if reads > MAX_READS:
                raise ValueError('transport_trace_read_limit')
            requested = phase['requestedBytes']
            if (not _integer(requested, 1, 4097) or type(phase['rawOffset']) is not int
                    or phase['rawOffset'] != offset):
                raise ValueError('transport_trace_read_size_invalid')
            if 'returnedBytes' not in phase:
                if not error or 'rawSha256' in phase: raise ValueError('transport_trace_read_missing')
                count = 0
            else:
                count = phase['returnedBytes']
                if not _integer(count, 0, requested) or offset + count > len(raw):
                    raise ValueError('transport_trace_read_size_invalid')
                if phase.get('rawSha256') != hashlib.sha256(raw[offset:offset + count]).hexdigest():
                    raise ValueError('transport_trace_chunk_hash_invalid')
            offset += count
            if count:
                if first is None: first = b
                last_nonempty = b
            expected = 'buffer_query'
        else:
            if (type(phase['rawBytes']) is not int or phase['rawBytes'] != len(raw)
                    or phase['rawSha256'] != hashlib.sha256(raw).hexdigest() or last_nonempty is None):
                raise ValueError('transport_trace_parse_raw_invalid')
            parsed = not error
            parse_duration = (b - a) * 1000
            if parsed: complete_observed = last_nonempty
            expected = 'done'
    if offset != len(raw):
        raise ValueError('transport_trace_raw_count_invalid')
    if completed and (not write_complete or not parsed or last_error or expected != 'done'):
        raise ValueError('transport_trace_success_incomplete')
    return dict(readCalls=reads, rawBytes=offset,
                firstNonemptyReadReturnedMonotonic=first,
                completeResponseObservedMonotonic=complete_observed if completed else None,
                parseDurationMs=parse_duration, hostObservationOnly=True,
                wireTimeVerified=False, sensorPhysicalSampleTimeVerified=False)
