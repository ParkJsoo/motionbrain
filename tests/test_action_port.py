"""Loopback HTTP contract checks, never a controller or deployed service."""
import json
from pathlib import Path
import tempfile
import threading
import unittest
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from learning.action_port import BoundedActionHTTP
from learning.action_execution import BoundedActionExecutor
from learning.device_adapter import DeviceReadError
from tests.test_action_execution import FakeClock, FakePort, limits, request

TOKEN="local-fixture-not-a-device-token"


def status():
    return dict(state="ARMED",sensor=dict(connected=True,imuOk=True,faultLatched=False,blocked=False,simulated=False),
                teleop=dict(controlActive=False,deadman=False),
                motors={f'M{i}':dict(speed=0,enabled=False) for i in range(1,6)},
                shoulderAngle=dict(available=True,sensorConnected=True,sensorFresh=True,sensorReady=True,
                                   active=False,ageMs=5,angleDeg=234,softMinDeg=230,softMaxDeg=245))


class ActionHTTPTest(unittest.TestCase):
    def setUp(self):
        self.clock=FakeClock();self.fake=FakePort(self.clock)
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.requests=[];self.redirect=False;self.bad_json=False;self.enabled=True;self.state=status()
        outer=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def send(self,value,code=200):
                data=(b'{"accepted":true,"accepted":false}' if outer.bad_json else json.dumps(value).encode())
                self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(data)))
                self.end_headers();self.wfile.write(data)
            def do_GET(self):
                path=urllib.parse.urlsplit(self.path);q=dict(urllib.parse.parse_qsl(path.query))
                outer.requests.append(('GET',self.path,{}))
                if path.path=='/status':self.send(outer.state)
                elif path.path=='/actions' and not q:
                    self.send(dict(schemaVersion='motionbrain.bounded_action.v1',clockDomain='controller_boot_millis',
                                   bootId='a'*32,enabled=outer.enabled,active=False,recordCount=0,recordCapacity=32))
                elif path.path=='/actions':
                    self.send(outer.fake.lookup(request(q['commandId'])))
                elif path.path=='/action_outputs':
                    self.send(outer.fake.outputs() if outer.fake.last_record else outer.fake.log('a'*32,[]))
                else:self.send({},404)
            def do_POST(self):
                payload=dict(urllib.parse.parse_qsl(self.rfile.read(int(self.headers.get('Content-Length',0))).decode()))
                outer.requests.append(('POST',self.path,dict(self.headers)))
                if outer.redirect:
                    self.send_response(307);self.send_header('Location','/other');self.end_headers();return
                if self.headers.get('X-MotionBrain-Token')!=TOKEN or self.headers.get('X-MotionBrain')!='1':self.send({},403);return
                if self.path=='/actions':
                    r={k:(int(v) if k in ('motor','pwm','durationMs','percent') else float(v) if k=='targetDegrees' else v) for k,v in payload.items()}
                    self.send(outer.fake.submit(r))
                elif self.path=='/actions/cancel':self.send(outer.fake.cancel(request(payload['commandId'])))
                else:self.send({},404)
        self.server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.server.server_close);self.addCleanup(lambda:self.thread.join(timeout=2));self.addCleanup(self.server.shutdown)
        self.url=f'http://127.0.0.1:{self.server.server_port}'
    def port(self,enabled=True,**kwargs):
        return BoundedActionHTTP(self.url,clock=self.clock,writes_enabled=enabled,token_provider=lambda:TOKEN,**kwargs)
    def test_default_disabled_rejects_all_writes_without_http(self):
        port=BoundedActionHTTP(self.url,token_provider=lambda:TOKEN)
        for call in (port.preflight,port.submit,port.cancel):
            with self.assertRaises(DeviceReadError):call(request())
        self.assertEqual(self.requests,[])
    def test_controller_disabled_idle_and_missing_safety_refuse(self):
        port=self.port()
        self.enabled=False
        with self.assertRaises(DeviceReadError):port.preflight(request())
        self.enabled=True
        for mutation in [lambda s:s.update(state='IDLE'),lambda s:s['sensor'].pop('blocked'),
                         lambda s:s['motors']['M1'].update(speed=1),lambda s:s['teleop'].update(deadman=True)]:
            self.state=status();mutation(self.state)
            with self.assertRaises(DeviceReadError):port.preflight(request())
        self.assertFalse(any(m=='POST' for m,_,_ in self.requests))
    def test_m4_age_and_soft_range_require_verified_status(self):
        port=self.port();r={**request(), 'kind':'shoulder','motor':4,'pwm':0,'targetDegrees':234.25,'percent':75}
        port.preflight(r)
        self.state['shoulderAngle']['ageMs']=21
        with self.assertRaises(DeviceReadError):port.preflight(r)
        self.state=status()
        with self.assertRaises(DeviceReadError):port.preflight({**r,'targetDegrees':250})
    def test_exact_post_then_result_and_output_collection_no_token_in_journal(self):
        port=self.port();path=Path(self.temp.name)/'journal.sqlite'
        core=BoundedActionExecutor(path,port,limits(),clock=self.clock);self.addCleanup(core.close)
        core.propose(request());result=core.confirm('first',confirmed_request=request(),dispatch_guard=lambda r:None)
        self.assertEqual(result['executionState'],'known_terminal',result)
        posts=[r for r in self.requests if r[0]=='POST']
        self.assertEqual(len(posts),1);self.assertEqual(posts[0][1],'/actions')
        self.assertEqual(posts[0][2]['X-Motionbrain-Token'],TOKEN)
        self.assertNotIn(TOKEN,json.dumps(core.journal.records()))
        self.assertNotIn(TOKEN,path.read_bytes().decode(errors='replace'))
    def test_redirect_never_forwards_credentials_or_request(self):
        port=self.port();port.preflight(request());self.redirect=True
        with self.assertRaises(DeviceReadError):port.submit(request())
        self.assertEqual([p for m,p,_ in self.requests if m=='POST'],['/actions'])
    def test_malformed_json_and_missing_token_fail_without_retry(self):
        port=self.port();port.preflight(request());self.requests.clear();self.bad_json=True
        with self.assertRaises(DeviceReadError):port.submit(request())
        self.assertEqual(len(self.requests),1)
        port.token_provider=lambda:None
        with self.assertRaises(DeviceReadError):port.preflight(request())
        self.assertEqual(len(self.requests),1)
    def test_slow_credentials_expire_before_guard_without_any_post(self):
        port=self.port()
        def slow_token():
            self.clock.sleep(21)
            return TOKEN
        port.token_provider=slow_token
        core=BoundedActionExecutor(Path(self.temp.name)/'slow.sqlite',port,limits(),clock=self.clock)
        self.addCleanup(core.close)
        core.propose(request());guards=[]
        result=core.confirm('first',confirmed_request=request(),dispatch_guard=lambda r:guards.append(r))
        self.assertEqual(result['executionState'],'not_sent')
        self.assertFalse(guards)
        self.assertFalse(any(m=='POST' for m,_,_ in self.requests))
    def test_submit_and_cancel_reuse_prepared_auth_and_submit_is_single_use(self):
        port=self.port();calls=[]
        def token():
            calls.append(1)
            return TOKEN
        port.token_provider=token;port.preflight(request())
        port.submit(request());port.cancel(request())
        with self.assertRaises(DeviceReadError):port.submit(request())
        self.assertEqual(calls,[1])
        self.assertEqual([p for m,p,_ in self.requests if m=='POST'],['/actions','/actions/cancel'])
    def test_wrong_controller_boot_never_posts(self):
        with self.assertRaises(DeviceReadError):self.port().preflight({**request(),'bootId':'b'*32})
        self.assertFalse(any(m=='POST' for m,_,_ in self.requests))


if __name__=='__main__':unittest.main()
