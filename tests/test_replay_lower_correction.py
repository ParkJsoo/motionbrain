import unittest

from tools.replay_lower_correction import SimulatedCorrection


class CorrectionTests(unittest.TestCase):
    def test_timed_writes_and_one_shot(self):
        sim = SimulatedCorrection([(0,-10),(82,-20),(554,0)])
        sim.decide(100, "demonstrated_correction")
        sim.decide(181, "demonstrated_correction")
        self.assertEqual(sim.motors, [0,0,0,-10,0])
        sim.advance(182)
        self.assertEqual(sim.motors[3], -20)
        sim.advance(700)
        self.assertEqual(sim.motors, [0]*5)
        for external, near in [((653,655),(660,661)), ((660,661),(654,655)), ((690,710),(690,691))]:
            sim.observe(700, external, near)
            self.assertEqual(sim.state, "awaiting_post_off_pair")
        sim.observe(4000,(655,656),(3990,3992))
        self.assertEqual(sim.state,"awaiting_post_off_pair")
        sim.observe(4000,(3990,3992),(655,656))
        self.assertEqual(sim.state,"awaiting_post_off_pair")
        sim.observe(4000,(3990,3992),(3993,3995))
        sim.decide(4001,"demonstrated_correction")
        self.assertEqual(sim.state,"ready_for_reassessment")
        self.assertEqual(sim.writes,[(100,-10),(182,-20),(654,0)])

    def test_release_path_stays_off(self):
        sim = SimulatedCorrection([(0,-10),(554,0)])
        sim.decide(5,"retain_release_check")
        sim.advance(999)
        self.assertEqual(sim.writes,[])
