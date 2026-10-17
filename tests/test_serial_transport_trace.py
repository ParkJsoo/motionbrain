"""Offline transport trace tests; no serial ports, network, or hardware."""
import base64
from copy import deepcopy
import hashlib
import unittest

from learning.serial_transport_trace import TransportTrace, TransportTraceLimit, validate_trace


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


class Clock:
    def __init__(self): self.value = 1.0
    def monotonic(self):
        self.value += .001
        return self.value


class TraceTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.trace = TransportTrace(self.clock)
        self.receipt = dict(requestStartedMonotonic=1., transportCompleted=True,
                            requestBase64=base64.b64encode(b'\x1e0123abcd\n').decode())
        self.raw = b'prefix\nresponse\n'

    def write(self, returned=10):
        with self.trace.operation('write', requestedBytes=10) as p:
            p['returnedBytes'] = returned

    def read(self, raw, offset=0, available=None):
        with self.trace.operation('buffer_query') as p:
            p['availableBytes'] = len(raw) if available is None else available
        with self.trace.operation('read', requestedBytes=max(1, len(raw)), rawOffset=offset) as p:
            p['returnedBytes'] = len(raw)
        p['rawSha256'] = digest(raw)

    def parse(self):
        with self.trace.operation('parse', rawBytes=len(self.raw), rawSha256=digest(self.raw)):
            pass

    def finish(self, completed=True, raw=None):
        self.receipt.update(receivedMonotonic=self.clock.monotonic(), transportCompleted=completed)
        return validate_trace(self.trace.snapshot(), self.receipt, self.raw if raw is None else raw)

    def success(self):
        self.write()
        self.read(b'')
        self.read(self.raw[:7])
        self.read(self.raw[7:], 7)
        self.parse()
        return self.finish()

    def test_success_empty_reads_and_chunk_observations(self):
        s = self.success()
        self.assertEqual(s['readCalls'], 3)
        self.assertEqual(s['rawBytes'], len(self.raw))
        self.assertLess(s['firstNonemptyReadReturnedMonotonic'], s['completeResponseObservedMonotonic'])
        self.assertGreater(s['parseDurationMs'], 0)
        self.assertTrue(s['hostObservationOnly'])
        self.assertFalse(s['wireTimeVerified'])
        self.assertFalse(s['sensorPhysicalSampleTimeVerified'])

    def test_snapshot_does_not_allow_mutation_of_collector(self):
        self.success()
        snapshot = self.trace.snapshot()
        snapshot['phases'].clear()
        self.assertTrue(self.trace.snapshot()['phases'])

    def test_cancelled_read_preserves_prior_bytes_and_error(self):
        self.write()
        self.read(self.raw[:7])
        with self.trace.operation('buffer_query') as p: p['availableBytes'] = 0
        with self.assertRaises(KeyboardInterrupt):
            with self.trace.operation('read', requestedBytes=1, rawOffset=7):
                raise KeyboardInterrupt()
        s = self.finish(False, self.raw[:7])
        self.assertEqual(s['rawBytes'], 7)
        self.assertEqual(self.trace.phases[-1]['errorType'], 'KeyboardInterrupt')
        self.assertIsNone(s['completeResponseObservedMonotonic'])

    def test_exception_during_write_records_no_claimed_bytes(self):
        with self.assertRaises(OSError):
            with self.trace.operation('write', requestedBytes=10): raise OSError('no device')
        self.assertEqual(self.finish(False, b'')['rawBytes'], 0)

    def test_partial_write_return_is_valid_failed_receipt(self):
        self.write(4)
        self.finish(False, b'')
        with self.assertRaises(ValueError): self.finish(True, b'')

    def test_partial_write_cannot_continue_to_read(self):
        self.write(4); self.read(self.raw)
        with self.assertRaises(ValueError): self.finish(False)

    def test_empty_failed_trace(self):
        self.finish(False, b'')
        with self.assertRaises(ValueError): self.finish(True, b'')

    def test_query_only_prefix_after_read_limit(self):
        self.write()
        for _ in range(256): self.read(b'')
        with self.trace.operation('buffer_query') as p: p['availableBytes'] = 0
        with self.assertRaises(TransportTraceLimit):
            with self.trace.operation('read', requestedBytes=1, rawOffset=0):
                self.fail('must not execute read')
        self.assertEqual(self.finish(False, b'')['readCalls'], 256)

    def test_parse_exception_and_original_raw_hash(self):
        self.write(); self.read(self.raw)
        with self.assertRaises(ValueError):
            with self.trace.operation('parse', rawBytes=len(self.raw), rawSha256=digest(self.raw)):
                raise ValueError('invalid reply')
        self.assertIsNone(self.finish(False)['completeResponseObservedMonotonic'])

    def test_success_requires_write_read_and_parse(self):
        self.write(); self.read(self.raw)
        with self.assertRaises(ValueError): self.finish()

    def test_timestamp_and_types_tampering_rejected(self):
        self.success()
        for key, value in [('startedMonotonic', float('nan')), ('endedMonotonic', float('inf')),
                           ('startedMonotonic', -1), ('endedMonotonic', 10**10000),
                           ('startedMonotonic', True)]:
            t = self.trace.snapshot(); t['phases'][2][key] = value
            with self.subTest(key=key, type=type(value)), self.assertRaises(ValueError):
                validate_trace(t, self.receipt, self.raw)
        t = self.trace.snapshot(); t['phases'][2]['startedMonotonic'] = 1.
        with self.assertRaises(ValueError): validate_trace(t, self.receipt, self.raw)

    def test_byte_hash_offset_and_count_tampering_rejected(self):
        self.success()
        for index, key, value in [(4, 'rawOffset', 1), (4, 'returnedBytes', True),
                                  (4, 'returnedBytes', 8), (4, 'rawSha256', '0' * 64),
                                  (7, 'rawBytes', len(self.raw) + 1), (7, 'rawSha256', '0' * 64),
                                  (0, 'requestedBytes', 11), (1, 'availableBytes', -1)]:
            t = self.trace.snapshot(); t['phases'][index][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_trace(t, self.receipt, self.raw)
        with self.assertRaises(ValueError): validate_trace(self.trace.snapshot(), self.receipt, self.raw + b'x')

    def test_operation_reordering_error_and_extra_fields_rejected(self):
        self.success()
        for mutate in [lambda p: p.__setitem__('kind', 'parse'),
                       lambda p: p.__setitem__('errorType', 'KeyboardInterrupt'),
                       lambda p: p.__setitem__('extra', 1)]:
            t = self.trace.snapshot(); mutate(t['phases'][2])
            with self.assertRaises(ValueError): validate_trace(t, self.receipt, self.raw)
        t = self.trace.snapshot(); t['phases'].append(deepcopy(t['phases'][-1]))
        with self.assertRaises(ValueError): validate_trace(t, self.receipt, self.raw)

    def test_schema_and_missing_fields_rejected(self):
        self.success()
        for t in [None, {}, {'schemaVersion': 'old', 'phases': []},
                  {'schemaVersion': self.trace.snapshot()['schemaVersion'], 'phases': [None]}]:
            with self.assertRaises(ValueError): validate_trace(t, self.receipt, self.raw)
        t = self.trace.snapshot(); del t['phases'][4]['rawOffset']
        with self.assertRaises(ValueError): validate_trace(t, self.receipt, self.raw)

    def test_nested_or_unknown_operation_rejected(self):
        with self.assertRaises(ValueError):
            with self.trace.operation('unknown'): pass
        with self.trace.operation('write', requestedBytes=10) as p:
            with self.assertRaises(ValueError):
                with self.trace.operation('read'): pass
            with self.assertRaises(ValueError): self.trace.snapshot()
            p['returnedBytes'] = 10

    def test_single_cancel_during_each_end_timestamp_preserves_snapshot(self):
        class InterruptClock(Clock):
            interrupt = False
            def monotonic(self):
                value = super().monotonic()
                if self.interrupt:
                    self.interrupt = False
                    raise KeyboardInterrupt()
                return value

        for kind in ('write', 'buffer_query', 'read', 'parse'):
            with self.subTest(kind=kind):
                self.clock = InterruptClock()
                self.trace = TransportTrace(self.clock)
                raw = b''
                if kind != 'write': self.write()
                if kind == 'read':
                    with self.trace.operation('buffer_query') as p:
                        p['availableBytes'] = len(self.raw)
                if kind == 'parse':
                    self.read(self.raw)
                    raw = self.raw
                fields = {
                    'write': dict(requestedBytes=10, returnedBytes=10),
                    'buffer_query': dict(availableBytes=0),
                    'read': dict(requestedBytes=len(self.raw), returnedBytes=len(self.raw),
                                 rawOffset=0, rawSha256=digest(self.raw)),
                    'parse': dict(rawBytes=len(self.raw), rawSha256=digest(self.raw)),
                }[kind]
                with self.assertRaises(KeyboardInterrupt):
                    with self.trace.operation(kind, **fields):
                        if kind == 'read': raw = self.raw
                        self.clock.interrupt = True
                snapshot = self.trace.snapshot()
                phase = snapshot['phases'][-1]
                self.assertEqual(phase['errorType'], 'KeyboardInterrupt')
                self.assertGreater(phase['endedMonotonic'], phase['startedMonotonic'])
                self.assertFalse(self.trace.active)
                self.finish(False, raw)

    def test_repeated_end_clock_interrupt_does_not_leave_active_operation(self):
        class InterruptClock(Clock):
            interrupts = 0
            def monotonic(self):
                if self.interrupts:
                    self.interrupts -= 1
                    raise KeyboardInterrupt()
                return super().monotonic()
        self.clock = InterruptClock()
        self.trace = TransportTrace(self.clock)
        with self.assertRaises(KeyboardInterrupt):
            with self.trace.operation('write', requestedBytes=10, returnedBytes=10):
                self.clock.interrupts = 2
        self.assertFalse(self.trace.active)
        snapshot = self.trace.snapshot()
        self.assertEqual(snapshot['phases'][0]['errorType'], 'KeyboardInterrupt')
        self.assertNotIn('endedMonotonic', snapshot['phases'][0])
        with self.assertRaises(ValueError): self.finish(False, b'')

    def test_read_limit_cannot_be_bypassed_by_forged_trace(self):
        self.write()
        for _ in range(256): self.read(b'')
        self.receipt.update(receivedMonotonic=self.clock.monotonic(), transportCompleted=False)
        t = self.trace.snapshot()
        extra = deepcopy(t['phases'][-2:])
        for p in extra:
            p['startedMonotonic'] = p['endedMonotonic'] = t['phases'][-1]['endedMonotonic']
        t['phases'].extend(extra)
        with self.assertRaises(ValueError): validate_trace(t, self.receipt, b'')


if __name__ == '__main__':
    unittest.main()
