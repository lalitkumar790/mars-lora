import unittest
from analysis.evaluate_slam import metric

class SLAMMetricsTests(unittest.TestCase):
    def test_odometry_vs_estimate_error(self):
        truth=[(float(i),.2*i,0.) for i in range(30)]
        scans=[{'t':float(i),'range_max':18,'ranges':[3,18,18,5],
               'odom':{'x':.2*i+.03*i,'y':0},'estimated':{'x':.2*i,'y':0}}
                for i in range(30)]
        out=metric(scans,truth)
        self.assertAlmostEqual(out['est_ate_rmse_m'],0.)
        self.assertGreater(out['odom_ate_rmse_m'],.4)
        self.assertAlmostEqual(out['mean_observable_fraction'],.5)
        self.assertIsNotNone(out['est_rpe_10s_rmse_m'])

if __name__=='__main__':
    unittest.main()
