"""Run UI behavior tests without a browser, camera, or controller."""
from pathlib import Path
import shutil
import subprocess
import unittest


class CollectionUITests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('node'), 'Node.js required for browser script tests')
    def test_browser_contracts(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(['node', str(root/'tests/js/manual_collection_ui_test.js')],
                                cwd=root, capture_output=True, text=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
