import unittest
from learning.joint_batch_audit import validate_cancel_evidence


class CancelEvidenceTests(unittest.TestCase):
 def fixture(self):
  return dict(dispatchGuardPassed=False,executionStatus='not_dispatched',offVerified=True),{'error':'ValueError'},[dict(motor=i,pwm=0) for i in range(1,6)]
 def test_known_cancel_requires_off_for_every_motor(self):
  a,e,t=self.fixture();validate_cancel_evidence(a,e,t)
  for trace in ([],t[:-1],t+[dict(motor=2,pwm=1)]):
   with self.assertRaisesRegex(ValueError,'cancel_output'):validate_cancel_evidence(a,e,trace)
 def test_network_unknown_and_passed_guard_never_become_cancel(self):
  for key,value in [('dispatchGuardPassed',True),('executionStatus','outcome_unknown'),('offVerified',False)]:
   a,e,t=self.fixture();a[key]=value
   with self.assertRaisesRegex(ValueError,'unproven'):validate_cancel_evidence(a,e,t)
  for error in ('TimeoutError','URLError','AssertionError'):
   a,e,t=self.fixture();e['error']=error
   with self.assertRaisesRegex(ValueError,'unproven'):validate_cancel_evidence(a,e,t)
