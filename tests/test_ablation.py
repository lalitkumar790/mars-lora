import unittest
from mars_lora.ablation import Variant,run_ablation

class AblationTests(unittest.TestCase):
    def test_determinism(self):
        self.assertEqual(run_ablation(Variant(True,False,True),7),run_ablation(Variant(True,False,True),7))
    def test_traffic_generation_is_equal(self):
        x=[run_ablation(Variant(a,p,s),4) for a in (False,True) for p in (False,True) for s in (False,True)]
        self.assertEqual(len(set(r.generated for r in x)),1)
        self.assertEqual(len(set(r.alert_total for r in x)),1)
        self.assertEqual(len(set(r.seed for r in x)),1)
        self.assertEqual(len(set(r.speed for r in x)),1)
        self.assertLess(x[0].mission_progress_m,x[-1].mission_progress_m+1e9)
        for r in x:
            self.assertGreaterEqual(r.alert_satisfaction,0)
            self.assertLessEqual(r.alert_satisfaction,1)
            self.assertLessEqual(r.alert_timely,r.alert_total)
    def test_slowdown_affects_progress_not_wallclock(self):
        a=run_ablation(Variant(False,False,False),8)
        b=run_ablation(Variant(False,False,True),8)
        self.assertAlmostEqual(b.mission_progress_m/a.mission_progress_m,.8)
if __name__=="__main__":
    unittest.main()
