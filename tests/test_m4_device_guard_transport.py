import unittest
from tests.test_supervised_transport import Clock,Store,Controller,Response
from learning.supervised_transport import SequencePort
from learning import m4_device_guard_transport as g
from learning.m4_device_guard_policy import device_guard_policy,device_guard_task_pulse

class GuardController(Controller):
 def __init__(self,clock):
  super().__init__(clock);self.boot='a'*32;self.angle=197.22;self.record=None;self.enabled=True;self.bad=None;self.lost_run=False
 def envelope(self):
  return dict(schemaVersion=g.CAPABILITY_SCHEMA,enabled=self.enabled,bootId=self.boot,policySha256=g.POLICY_SHA256,
   execution='main_loop_http_deferred',nonceCapacity=32,nonceCount=int(self.record is not None),record=self.record)
 def open(self,request,timeout):
  import urllib.parse
  path=urllib.parse.urlsplit(request.full_url).path
  data=None if request.data is None else {k:v[0] for k,v in urllib.parse.parse_qs(request.data.decode()).items()}
  if self.running:
   if self.clock.now < self.end:raise AssertionError('active HTTP forbidden')
   self.running=False;self.append(4,0);self.angle=self.record['goalDeg']
   self.record.update(state='terminal',reason='goal',decisionAtMs=11000,offAtMs=11001,
    sampleAtMs=10990,sampleAgeMs=10,maxTickGapMs=10,firstTickDelayMs=20,lastAngleDeg=self.angle,offVerified=True,outputMayHaveOccurred=True)
   if self.bad:self.record.update(self.bad)
  if path=='/sequence' and data is None:
   self.requests.append((path,None));return Response(dict(sequence=dict(state='IDLE',totalCount=len(self.queue)),m4GoalGuard=self.envelope()))
  if path=='/sequence' and data and data['action']=='add_m4_guarded':
   self.requests.append((path,data));self.queue.append(data)
   self.record=dict(nonce=data['guardNonce'],scopeSha256=data['guardScopeSha256'],bootId=data['guardBootId'],
    policySha256=data['guardPolicySha256'],up=data['direction']=='up',percent=int(data['speed']),durationMs=int(data['duration']),
    expectedStartDeg=float(data['expectedStartDeg']),goalDeg=float(data['goalDeg']),state='queued',queuedAtMs=10000,
    runAtMs=10020,startAngleDeg=self.angle)
   return Response(dict(success=True))
  if path=='/sequence' and data and data['action']=='run_m4_guarded':
   self.requests.append((path,data));self.running=True;self.end=self.clock.now+1.2;self.append(4,127 if self.record['up'] else -89)
   if self.lost_run:raise TimeoutError('lost RUN reply')
   return Response(dict(success=True))
  return super().open(request,timeout)

class GuardTests(unittest.TestCase):
 def setup_port(self):
  clock=Clock();store=Store();controller=GuardController(clock)
  port=SequencePort('http://127.0.0.1:8181','private-token',controller.boot,store,clock=clock)
  request=dict(nonce='b'*32,bootId=controller.boot,scopeSha256='c'*64,policySha256=g.POLICY_SHA256,
   expectedStartDeg=197.22,goalDeg=197.85)
  return port,controller,store,request
 def run_guard(self,port,controller,request,direction='up'):
  port.opener=controller
  return port._execute_validated(device_guard_task_pulse(direction),lambda:None,device_goal_guard=request)
 def test_terminal_flow_no_active_get(self):
  p,c,s,r=self.setup_port();result=self.run_guard(p,c,r)
  self.assertEqual(result['deviceGoalGuard']['reason'],'goal');self.assertTrue(result['offVerified'])
  self.assertEqual(sum(bool(d and d.get('action')=='run_m4_guarded') for _,d in c.requests),1)
  self.assertIn('m4-device-guard-terminal.json',s.rows)
 def test_down_terminal_uses_direction_and_same_scope(self):
  p,c,s,r=self.setup_port();c.angle=197.75;r.update(expectedStartDeg=197.75,goalDeg=197.22)
  result=self.run_guard(p,c,r,'down')
  self.assertFalse(result['deviceGoalGuard']['up']);self.assertTrue(result['offVerified'])
 def test_deadline_is_negative_effect_not_goal(self):
  p,c,s,r=self.setup_port();c.bad=dict(reason='deadline',decisionAtMs=11220,offAtMs=11221,
   sampleAtMs=11210,lastAngleDeg=197.5)
  result=self.run_guard(p,c,r)
  self.assertEqual(result['stopReason'],'device_deadline');self.assertFalse(result['stoppedEarly'])
 def test_old_firmware_zero_writes(self):
  p,c,s,r=self.setup_port();c.enabled=False
  with self.assertRaisesRegex(ValueError,'capability_required'):self.run_guard(p,c,r)
  # Existing emergency OFF cleanup may write STOP/clear; guarded queue and RUN never dispatch.
  self.assertFalse(any(d and d.get('action') in ('add_m4_guarded','run_m4_guarded','default') for _,d in c.requests))
 def test_terminal_nonce_or_sensor_failure_no_retry(self):
  for bad in ({'nonce':'e'*32},{'sampleAgeMs':61},{'reason':'tick_gap'},{'offVerified':False},{'offAtMs':11006},{'maxTickGapMs':41},{'firstTickDelayMs':201}):
   with self.subTest(bad=bad):
    p,c,s,r=self.setup_port();c.bad=bad
    with self.assertRaises(ValueError):self.run_guard(p,c,r)
    self.assertEqual(sum(bool(d and d.get('action')=='run_m4_guarded') for _,d in c.requests),1)
 def test_lost_run_reply_no_retry(self):
  p,c,s,r=self.setup_port();c.lost_run=True
  with self.assertRaises(TimeoutError):self.run_guard(p,c,r)
  self.assertTrue(p.may_have_run)
  self.assertEqual(sum(bool(d and d.get('action')=='run_m4_guarded') for _,d in c.requests),1)
 def test_queued_contract_changed_zero_run(self):
  p,c,s,r=self.setup_port();original=c.envelope
  def changed():
   value=original()
   if value['record'] is not None:
    value['record']=dict(value['record'],goalDeg=197.22)
   return value
  c.envelope=changed
  with self.assertRaisesRegex(ValueError,'record_changed'):self.run_guard(p,c,r)
  self.assertFalse(any(d and d.get('action')=='run_m4_guarded' for _,d in c.requests))
 def test_guarded_terminal_budget_cannot_be_general_opt_in(self):
  p,c,s,r=self.setup_port();p.opener=c
  with self.assertRaisesRegex(ValueError,'terminal_scope'):
   p._call('/sequence',request_timeout=2.,device_guard_terminal=True)
  self.assertEqual(c.requests,[])
 def test_old_policy_distinct(self):
  from learning.m4_effective_policy import effective_policy
  self.assertNotEqual(device_guard_policy(),effective_policy())
 def test_invalid_request_before_io(self):
  p,c,s,r=self.setup_port();r['policySha256']='f'*64
  with self.assertRaisesRegex(ValueError,'policy_changed'):self.run_guard(p,c,r)
  self.assertEqual(c.requests,[])

if __name__=='__main__':unittest.main()
