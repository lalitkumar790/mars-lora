import math,unittest
import numpy as np
from mars_lora.lidar_imu_eskf import TranslationalESKF

class TranslationalESKFTests(unittest.TestCase):
 def test_unbiased_constant_accel_propagation(self):
  f=TranslationalESKF.initial()
  for _ in range(10):f.predict((1.,0.,0.),.1)
  self.assertAlmostEqual(f.position[0],.5,places=5)
  self.assertAlmostEqual(f.velocity[0],1.,places=5)
  self.assertTrue(np.all(np.linalg.eigvalsh(f.P)>0))
 def test_hard_outlier_rejected_without_state_jump(self):
  f=TranslationalESKF.initial()
  old=f.position.copy()
  result=f.update_position((40.,-20.,4.),std_m=.12)
  self.assertFalse(result.accepted)
  self.assertEqual(result.reason,"innovation_gate")
  np.testing.assert_array_equal(old,f.position)
  self.assertGreater(result.nis,result.gate)
 def test_corrections_preserve_symmetric_positive_covariance(self):
  f=TranslationalESKF.initial()
  for i in range(40):
   f.predict((.01,-.01,.02),.1)
   f.update_altitude(4.+.01*math.sin(i),std_m=.06)
   f.update_horizontal_flow(.0,.0,std_m=.045)
   if i%10==0:f.update_position((0.,0.,4.),std_m=.2)
  np.testing.assert_allclose(f.P,f.P.T,atol=1e-12)
  self.assertGreater(float(np.linalg.eigvalsh(f.P).min()),0.)
  self.assertGreater(f.accepted,0)
 def test_constant_bias_correctable_under_observable_measurements(self):
  f=TranslationalESKF.initial()
  for i in range(450):
   f.predict((.02,-.015,.0),.1)
   f.update_horizontal_flow(0.,0.,std_m=.018)
   f.update_altitude(4.,std_m=.03)
   if i%10==0:f.update_position((0.,0.,4.),std_m=.08)
  self.assertLess(abs(f.bias[0]-.02),.011)
  self.assertLess(abs(f.bias[1]+.015),.011)
  self.assertLess(math.dist(f.position,(0.,0.,4.)),.20)
 def test_uncertainty_grows_without_measurements(self):
  f=TranslationalESKF.initial()
  sigma=f.position_rms_sigma_m
  for _ in range(80):f.predict((0.,0.,0.),.1)
  self.assertGreater(f.position_rms_sigma_m,sigma)
 def test_bad_sensor_rejected_and_positivity_required(self):
  f=TranslationalESKF.initial()
  with self.assertRaises(ValueError):f.predict((float("nan"),0.,0.),.1)
  with self.assertRaises(ValueError):f.predict((0.,0.,0.),0.)
  with self.assertRaises(ValueError):f.update_position((0.,0.,4.),std_m=0.)
  with self.assertRaises(ValueError):TranslationalESKF.initial(P=np.zeros((9,9)))
 def test_nees_is_ground_truth_metric_only(self):
  f=TranslationalESKF.initial()
  before=f.position.copy()
  score=f.position_nees((.1,0.,4.))
  self.assertGreater(score,0.)
  np.testing.assert_array_equal(before,f.position)

if __name__=="__main__": unittest.main()
