import unittest
from mars_lora.channel import airtime_s, run_trial, percentile

class TestChannel(unittest.TestCase):
    def test_airtime_increases_with_spreading_factor(self):
        times = [airtime_s(sf, 20) for sf in range(7,13)]
        self.assertEqual(times, sorted(times))
    def test_payload_increases_airtime(self):
        self.assertGreater(airtime_s(9,40),airtime_s(9,12))
    def test_repeatable(self):
        self.assertEqual(run_trial("fixed",5,4,17),run_trial("fixed",5,4,17))
    def test_counts(self):
        for name in ("fixed","reactive","priority","mars"):
            r=run_trial(name,5,4,2)
            self.assertLessEqual(r.received,r.generated)
            self.assertLessEqual(r.critical_on_time,r.critical_received)
            self.assertLessEqual(r.critical_received,r.critical_generated)
            self.assertGreater(r.p95_aoi,0)
    def test_percentile(self):
        self.assertEqual(percentile([0,10],0.5),5)
    def test_invalid(self):
        with self.assertRaises(ValueError):
            run_trial("invalid",5,4,1)

if __name__ == "__main__":
    unittest.main()
