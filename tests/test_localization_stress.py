import unittest
from mujoco_sim.validate_mission import run
class SyntheticLocalizationTests(unittest.TestCase):
    def test_zero_error_matches_truth(self):
        r=run("heavy",duration=.8,wind=0,localization_seed=27)
        self.assertEqual(r["navigation_ate_rmse_m"],0)
    def test_noisy_estimator_is_seed_deterministic(self):
        args=dict(airframe="heavy",duration=.8,wind=0,
            localization_drift_m_sqrt_s=.03,
            localization_noise_sigma_m=.02,localization_seed=17)
        a=run(**args);b=run(**args)
        self.assertGreater(a["navigation_ate_rmse_m"],0)
        self.assertEqual(a["navigation_ate_rmse_m"],b["navigation_ate_rmse_m"])
    def test_negative_noise_rejected(self):
        with self.assertRaises(ValueError):
            run("heavy",duration=.8,localization_drift_m_sqrt_s=-1)
if __name__=="__main__":unittest.main()
