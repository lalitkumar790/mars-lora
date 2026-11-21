import unittest
from mars_lora.route_gate import choose_route,simulated_pilot
from experiments.heldout_validation import PROFILES,STATIONS

class PreflightGateTests(unittest.TestCase):
    def test_good_link_uses_shortest(self):
        self.assertEqual(choose_route(-110,.05),"shortest")
    def test_poor_link_uses_radio_aware(self):
        self.assertEqual(choose_route(-136,.05),"rf_aware")
    def test_high_busy_uses_shortest(self):
        self.assertEqual(choose_route(-136,.45),"shortest")
    def test_threshold_edges(self):
        self.assertEqual(choose_route(-119.,.249),"shortest")
        self.assertEqual(choose_route(-119.01,.249),"rf_aware")
        with self.assertRaises(ValueError):
            choose_route(float("nan"),.1)
        with self.assertRaises(ValueError):
            choose_route(-130,1.5)
    def test_reproducible_simulated_pilot(self):
        profile=PROFILES["nominal_spatial"]
        a=simulated_pilot(profile,STATIONS["nominal_spatial"],5)
        b=simulated_pilot(profile,STATIONS["nominal_spatial"],5)
        self.assertEqual(a,b)
        self.assertTrue(0<=a["observed_channel_busy"]<=1)
if __name__=="__main__":
    unittest.main()
