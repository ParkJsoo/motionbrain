"""Reproduce response/log chunks without USB or controller access."""
import base64
import unittest

from learning.serial_m4_probe import read_sample, validate_receipt, trace_summary
from tests.test_action_execution import FakeClock
from tests.test_serial_m4_probe import BOOT, line, fresh_line
from tests.test_compact_m4_probe import compact_line


class Chunks:
    def __init__(self, clock, chunks, *, delay=.01, cancel=False):
        self.clock=clock;self.chunks=list(chunks);self.delay=delay;self.cancel=cancel
    def write(self, command):return len(command)
    @property
    def in_waiting(self):return len(self.chunks[0]) if self.chunks else 0
    def read(self, size):
        self.clock.sleep(self.delay)
        if not self.chunks:
            if self.cancel:raise KeyboardInterrupt()
            return b''
        chunk=self.chunks.pop(0)
        assert len(chunk)<=size
        return chunk


class LogBoundaryTest(unittest.TestCase):
    def observe(self, chunks, *, fresh=False, compact=False, delay=.01, cancel=False):
        clock=FakeClock();device=Chunks(clock,chunks,delay=delay,cancel=cancel)
        row=read_sample(device,'0123abcd',BOOT,'host',clock=clock,
                        fresh_sample=fresh,compact_sample=compact,transport_trace=True)
        trace_summary(row,True)
        return row
    def test_all_formats_wait_for_split_log_without_discarding_or_backdating(self):
        for reply,mode in ((line(),{}),(fresh_line(),dict(fresh=True)),(compact_line(),dict(compact=True))):
            raw=reply+b'[WARN] log\n'
            row=self.observe([reply+b'[WARN]',b' log\n'],**mode)
            self.assertTrue(row['transportCompleted'],row)
            self.assertEqual(base64.b64decode(row['rawBase64']),raw)
            self.assertAlmostEqual(row['receivedMonotonic']-row['requestStartedMonotonic'],.02)
            validate_receipt(row,'0123abcd',BOOT,fresh_sample=mode.get('fresh',False),
                             compact_sample=mode.get('compact',False),transport_trace=True)
    def test_response_split_itself_still_works(self):
        reply=compact_line();row=self.observe([reply[:12],reply[12:]],compact=True)
        self.assertTrue(row['transportCompleted'])
    def test_unfinished_log_times_out_preserving_original(self):
        raw=compact_line()+b'[WARN]'
        row=self.observe([raw],compact=True)
        self.assertFalse(row['transportCompleted']);self.assertEqual(row['error'],'serial_request_deadline')
        self.assertEqual(base64.b64decode(row['rawBase64']),raw)
    def test_split_duplicate_or_partial_magic_is_never_discarded(self):
        for tail in (b'M4S3\n',compact_line()):
            row=self.observe([compact_line()+tail[:3],tail[3:]],compact=True)
            self.assertFalse(row['transportCompleted'])
            self.assertEqual(base64.b64decode(row['rawBase64']),compact_line()+tail)
    def test_continuous_log_does_not_restart_deadline(self):
        chunks=[compact_line()+b'[WARN]']+[b'x']*100
        row=self.observe(chunks,compact=True,delay=.02)
        self.assertFalse(row['transportCompleted']);self.assertEqual(row['error'],'serial_request_deadline')
        self.assertLess(row['receivedMonotonic']-row['requestStartedMonotonic'],.83)
        self.assertLess(len(base64.b64decode(row['rawBase64'])),sum(map(len,chunks)))
    def test_raw_limit_still_bounds_unterminated_log(self):
        row=self.observe([compact_line()+b'x'*(4097-58)],compact=True)
        self.assertFalse(row['transportCompleted']);self.assertEqual(row['error'],'serial_raw_limit')
        self.assertEqual(len(base64.b64decode(row['rawBase64'])),4097)
    def test_cancellation_waiting_for_log_keeps_reply_and_fragment(self):
        raw=compact_line()+b'[WARN]';row=self.observe([raw],compact=True,cancel=True)
        self.assertTrue(row['cancelled']);self.assertFalse(row['transportCompleted'])
        self.assertEqual(base64.b64decode(row['rawBase64']),raw)


if __name__=='__main__':unittest.main()
