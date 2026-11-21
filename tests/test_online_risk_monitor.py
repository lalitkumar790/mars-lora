import unittest
from mars_lora.online_risk_monitor import predict_clearance_alarm

class RiskMonitorTests(unittest.TestCase):
    def test_safe_corridor(self):
        signal=predict_clearance_alarm((0.,0.,4.),(0.,0.,0.),.1,
                                     lambda p:5.)
        self.assertFalse(signal.warning)
        self.assertEqual(signal.reason,"clear")
    def test_approach_hazard_even_before_contact(self):
        signal=predict_clearance_alarm((0.,0.,4.),(2.,0.,0.),.1,
                                     lambda p:3.5-p[0])
        self.assertTrue(signal.warning)
        self.assertEqual(signal.reason,"uncertain_stopping_corridor")
    def test_uncertainty_alone_can_trigger_warning(self):
        safe=predict_clearance_alarm((0.,0.,4.),(0.,0.,0.),.1,lambda p:1.)
        risky=predict_clearance_alarm((0.,0.,4.),(0.,0.,0.),.3,lambda p:1.)
        self.assertFalse(safe.warning)
        self.assertTrue(risky.warning)
    def test_estimator_divergence_does_not_allocate_unbounded_samples(self):
        r=predict_clearance_alarm((0.,0.,4.),(1000.,0.,0.),.1,lambda p:10.)
        self.assertTrue(r.warning)
        self.assertEqual(r.reason,"unbounded_stopping_distance")
    def test_bad_braking_bound(self):
        with self.assertRaises(ValueError):
            predict_clearance_alarm((0.,0.,4.),(1.,0.,0.),.1,lambda p:10.,
                                   braking_accel_m_s2=.1,wind_accel_m_s2=.15)
if __name__=="__main__":
    unittest.main()
