"""Offline recovery tests: no cameras, model runtime, or motor endpoints."""
import contextlib
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

from tools import motionbrain_dashboard as dashboard
from tools.raspi import publish_vision_endpoint as publisher

ROOT = Path(__file__).resolve().parents[1]


class DeviceAddressRecoveryTest(unittest.TestCase):
    def test_browser_recovery_behaviors(self):
        node = shutil.which('node')
        self.assertIsNotNone(node, 'Node.js is required for browser recovery regression tests')
        cpp = (ROOT / 'src/network/web_server.cpp').read_text()
        script = '\n'.join(json.loads(m) for m in re.findall(r'server_\.sendContent\(("(?:\\.|[^"\\])*")\);', cpp))
        controller = script[script.index('const DEFAULT_CAMERA_URL'):script.index('function updateStatus')]
        js = dashboard.INDEX_HTML.split('<script>')[1].split('</script>')[0]
        shared = (ROOT / 'web/device_recovery.js').read_text()
        recovery = js[js.index('    let dashboardConfig'):js.index('    let dashboardAuthToken')]
        overlay = js[js.index('    function setTargetOverlay'):js.index('    function updateDetection')]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'sources.json'
            path.write_text(json.dumps(dict(shared=shared,controller=controller,dashboard=recovery,dashboardOverlay=overlay)))
            # Compile the entire page too, including code outside the isolated scenarios.
            full = Path(tmp) / 'dashboard.js'; full.write_text(js)
            subprocess.run([node,'--check',str(full)],check=True,capture_output=True,text=True)
            result = subprocess.run([node,str(ROOT/'tests/js/device_recovery_test.js'),str(path)],capture_output=True,text=True,timeout=30)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn('18 behavioral cases passed',result.stdout)

    def test_publisher_uses_only_authenticated_metadata_request(self):
        calls=[]
        class Opener:
            def open(self, request, timeout):
                calls.append(request)
                payload = {'service':'motionbrain-dashboard','recoveryVersion':1} if isinstance(request,str) else {'schema':'motionbrain.vision_endpoint.v1','available':True}
                return contextlib.closing(io.BytesIO(json.dumps(payload).encode()))
        self.assertTrue(publisher.publish('http://192.168.1.20','http://127.0.0.1:8765','secret',opener=Opener()))
        self.assertEqual(calls[0],'http://127.0.0.1:8765/api/config')
        request=calls[1]
        self.assertEqual(request.full_url,'http://192.168.1.20/vision_endpoint?port=8765')
        self.assertEqual(request.get_method(),'POST')
        self.assertEqual(request.get_header('X-motionbrain-token'),'secret')
        self.assertEqual(request.data,b'')

    def test_publisher_rejects_missing_token_redirects_and_wrong_service(self):
        self.assertFalse(publisher.publish('http://192.168.1.20','http://127.0.0.1:8765',''))
        self.assertIsNone(publisher.NoRedirect().redirect_request(None,None,302,'',{},'http://elsewhere'))
        class Opener:
            def open(self,*args,**kwargs):
                return contextlib.closing(io.BytesIO(b'{"service":"unrelated"}'))
        self.assertFalse(publisher.publish('http://192.168.1.20','http://127.0.0.1:8765','secret',opener=Opener()))

    def test_reconcile_renews_lease_without_restarting_healthy_services(self):
        import os
        import sys
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'discover_device_url.py').write_text("import sys; print('http://192.168.1.20' if 'controller' in sys.argv else 'http://192.168.1.30')")
            (root/'apply_camera_profile.py').write_text('')
            (root/'publish_vision_endpoint.py').write_text("import os, pathlib; pathlib.Path(os.environ['TEST_PUBLICATION']).write_text(os.environ['MOTIONBRAIN_HTTP_TOKEN'])")
            (root/'systemctl').write_text('#!/bin/sh\nif [ "$1" = is-active ]; then echo active; else exit 90; fi\n')
            (root/'curl').write_text("#!/bin/sh\necho '{\"ok\":true,\"cameraUrl\":\"http://192.168.1.30\",\"motionBaseUrl\":\"http://192.168.1.20\"}'\n")
            for name in ('curl','systemctl'): (root/name).chmod(0o755)
            env=dict(os.environ, PATH=str(root)+os.pathsep+os.environ['PATH'],
                MOTIONBRAIN_HELPER_DIR=str(root), MOTIONBRAIN_DISCOVERY_PYTHON=sys.executable,
                MOTIONBRAIN_DASHBOARD_ENV=str(root/'missing'), MOTIONBRAIN_PERCEPTION_ENV=str(root/'missing'),
                MOTIONBRAIN_CAMERA_PROFILE='0', MOTIONBRAIN_RECONCILE_ROS_BRIDGE='0',
                MOTIONBRAIN_HTTP_TOKEN='fake-test-token', TEST_PUBLICATION=str(root/'publication'))
            result=subprocess.run(['bash',str(ROOT/'tools/raspi/reconcile_dashboard_services.sh')],env=env,capture_output=True,text=True,timeout=10)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertEqual((root/'publication').read_text(),'fake-test-token')
            self.assertNotIn('fake-test-token',result.stdout+result.stderr)
            self.assertIn('already match',result.stdout)

    def test_lease_expiration_replacement_wraparound_and_peer_validation(self):
        compiler=shutil.which('clang++') or shutil.which('g++')
        self.assertIsNotNone(compiler)
        source = r'''
#include "network/vision_endpoint_lease.h"
#include <cassert>
int main() {
  VisionEndpointLease lease;
  uint8_t oldIP[]={192,168,1,10}, newIP[]={192,168,1,20}, external[]={8,8,8,8};
  assert(lease.remaining(0)==0);
  assert(lease.publish(oldIP,8765,10));
  assert(lease.remaining(180009)==1); assert(lease.remaining(180010)==0);
  assert(lease.publish(newIP,8765,0xfffffff0));
  assert(lease.address[3]==20); assert(lease.remaining(0x10)==180000-32);
  assert(!lease.publish(external,8765,50)); assert(!lease.publish(newIP,80,50));
  assert(lease.address[3]==20);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'lease.cpp';path.write_text(source);binary=Path(tmp)/'lease'
            flags=[]
            import sys
            if sys.platform=='darwin':
                sdk=subprocess.check_output(['xcrun','--sdk','macosx','--show-sdk-path'],text=True).strip()
                flags=['-isysroot',sdk]
            subprocess.run([compiler,'-std=c++11',*flags,'-I',str(ROOT/'src'),str(path),'-o',str(binary)],check=True,capture_output=True)
            subprocess.run([str(binary)],check=True,capture_output=True)


if __name__ == '__main__': unittest.main()
