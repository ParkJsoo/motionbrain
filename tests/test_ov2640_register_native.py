"""Exercise production snapshot logic with a faulting bus; no device accesses."""
import base64
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RegisterNativeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which('clang++') or shutil.which('g++')
        if not compiler:
            raise unittest.SkipTest('native C++ compiler unavailable')
        temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(temp.cleanup)
        cls.binary = Path(temp.name)/'register-diagnostic'
        flags = []
        if sys.platform == 'darwin':
            sdk = subprocess.check_output(['xcrun', '--sdk', 'macosx', '--show-sdk-path'], text=True).strip()
            flags = ['-isysroot', sdk]
        subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror', *flags,
                        '-I', str(ROOT/'firmware/esp32cam/src'),
                        str(ROOT/'tests/native/test_ov2640_register_diagnostic.cpp'),
                        '-o', str(cls.binary)], check=True, capture_output=True, timeout=60)

    def test_failure_at_every_transaction_restores_or_latches_failure(self):
        subprocess.run([str(self.binary)], check=True, capture_output=True, timeout=10)

    def test_native_sequence_is_accepted_by_actual_host_parser(self):
        from learning.ov2640_register_probe import validate_snapshot
        from tests.test_ov2640_register_probe import snapshot
        # Preserve the fixture's synthetic provenance; replace its transaction
        # body with output from the real native snapshot implementation.
        s = snapshot('0123456789abcdef', 1)
        native = json.loads(subprocess.check_output([str(self.binary), 'json'], text=True))
        s.update(native)
        raw = json.dumps(s).encode()
        row = dict(method='GET', route='/diagnostics/ov2640?nonce=0123456789abcdef',
                   nonce='0123456789abcdef', transportCompleted=True, httpStatus=200,
                   requestStartedMonotonic=1.0, receivedMonotonic=1.1,
                   responseBodyBase64=base64.b64encode(raw).decode(), bodySha256=hashlib.sha256(raw).hexdigest())
        validate_snapshot(row, '0123456789abcdef', s['appElfSha256'], 1)


if __name__ == '__main__':
    unittest.main()
