import math,unittest
import numpy as np
from mars_lora.body_imu_eskf15 import (
    InertialESKF15,exp_quat,qmul,quat_to_rot,quat_body_delta)
from mars_lora.body_imu_surrogate import BodyIMUSurrogate

class FullESKF15Tests(unittest.TestCase):
 def test_stationary_gravity_compensation(self):
  f=InertialESKF15.initial()
  for i in range(100):
   f.predict((0.,0.,9.81),(0.,0.,0.),.01)
  np.testing.assert_allclose(f.position,[0.,0.,4.],atol=2e-8)
  np.testing.assert_allclose(f.velocity,[0.,0.,0.],atol=2e-8)
  self.assertAlmostEqual(np.linalg.norm(f.quat_wb),1.,places=10)
 def test_body_specific_force_rotates_into_world(self):
  # At yaw +90 deg, body +X is world +Y.
  q=exp_quat((0.,0.,math.pi/2))
  f=InertialESKF15.initial(quat_wb=q)
  for _ in range(100):f.predict((1.,0.,9.81),(0.,0.,0.),.01)
  self.assertAlmostEqual(f.position[1],.5,places=4)
  self.assertAlmostEqual(f.position[0],0.,places=4)
 def test_gyroscope_quaternion_integrates(self):
  f=InertialESKF15.initial()
  for _ in range(100):f.predict((0.,0.,9.81),(0.,0.,.5),.01)
  self.assertAlmostEqual(quat_body_delta((1.,0.,0.,0.),f.quat_wb)[2],.5,places=5)
  self.assertGreater(np.linalg.eigvalsh(f.P).min(),0.)
 def test_outlier_gated_with_state_unchanged(self):
  f=InertialESKF15.initial()
  p=f.position.copy();q=f.quat_wb.copy()
  x=f.correct_xyz((50.,-50.,4.))
  self.assertFalse(x.accepted)
  np.testing.assert_array_equal(p,f.position)
  np.testing.assert_array_equal(q,f.quat_wb)
 def test_quaternion_sign_invariant_body_gyro_simulation(self):
  sensor=BodyIMUSurrogate(seed=1,gyro_noise_std=0.,accel_noise_std=0.,
                          gyro_bias_std=0.,accel_bias_std=0.)
  sensor.reset(0.,(0.,0.,0.),(1.,0.,0.,0.))
  q=exp_quat((0.,0.,.10))
  packet=sensor.measure(.1,(0.,0.,0.),-q)
  self.assertAlmostEqual(packet.angular_rate_b[2],1.,places=5)
  self.assertAlmostEqual(packet.specific_force_b[2],9.81,places=5)
 def test_covariance_stays_psd_with_observation_updates(self):
  f=InertialESKF15.initial()
  for i in range(70):
   f.predict((0.,0.,9.81),(0.,0.,0.),.02)
   f.correct_altitude(4.+.005*math.sin(i),std_m=.06)
   f.correct_horizontal_flow(0.,0.)
   if i%10==0:f.correct_xyz((0.,0.,4.),std_m=.2)
  np.testing.assert_allclose(f.P,f.P.T,atol=1e-11)
  self.assertGreater(np.linalg.eigvalsh(f.P).min(),0.)
  self.assertGreater(f.accepted,100)
 def test_no_observations_increase_position_uncertainty(self):
  f=InertialESKF15.initial()
  initial=f.sigma_position_rms_m
  for _ in range(300):f.predict((0.,0.,9.81),(0.,0.,0.),.02)
  self.assertGreater(f.sigma_position_rms_m,initial)
 def test_bad_inputs(self):
  f=InertialESKF15.initial()
  with self.assertRaises(ValueError):f.predict((0.,0.,9.81),(0.,0.,0.),0.)
  with self.assertRaises(ValueError):f.predict((0.,0.,9.81),(float('nan'),0.,0.),.01)
  with self.assertRaises(ValueError):f.correct_xyz((0.,0.,4.),std_m=0.)
  with self.assertRaises(ValueError):InertialESKF15.initial(quat_wb=(0.,0.,0.,0.))

if __name__=="__main__":unittest.main()
