import math,random,unittest
from mars_lora.lidar_imu3d import InertialMapEstimator,ray_box,scan_at,simulate_scan,register_scan,unit_rays
class LidarIMU3DTests(unittest.TestCase):
 def setUp(self):
  self.boxes=((8.,0.,5.,1.5,2.,5.),(-8.,0.,5.,1.5,2.,5.),(0.,9.,5.,2.,1.5,5.),(0.,-9.,5.,2.,1.5,5.),(7.,7.,5.,2.,2.,5.),(-7.,-7.,5.,2.,2.,5.),(-8.,8.,5.,1.8,1.8,5.))
 def test_3d_rays(self):
  self.assertEqual(len(unit_rays()),48)
  self.assertAlmostEqual(ray_box((0.,0.,5.),(1.,0.,0.),self.boxes[0]),6.5)
  self.assertEqual(ray_box((0.,0.,20.),(1.,0.,0.),self.boxes[0]),24.)
 def test_altitude_changes_scan(self):
  self.assertNotEqual(scan_at((.1,.2,5.),.17,self.boxes),scan_at((.1,.2,20.),.17,self.boxes))
 def test_register_xyz_without_truth_leak(self):
  truth=(.1,.2,5.);prior=(.8,-.5,5.4)
  readings=simulate_scan(truth,.12,self.boxes,random.Random(3),noise=.01,dropout=0)
  corrected,accepted,score,hits=register_scan(prior,.12,readings,self.boxes)
  self.assertTrue(accepted);self.assertGreaterEqual(hits,7)
  self.assertLess(math.dist(corrected,truth),math.dist(prior,truth))
 def test_unobservable_scan(self):
  corrected,accepted,score,hits=register_scan((1.,2.,15.),0.,[24.]*48,())
  self.assertFalse(accepted);self.assertEqual(corrected,(1.,2.,15.))
 def test_inertial_predict_and_uncertainty(self):
  state=InertialMapEstimator.initial((0.,0.,4.))
  state.predict((1.,0.,0.),0.,.5);state.predict((1.,0.,0.),0.,.5)
  self.assertAlmostEqual(state.position[0],.5)
  self.assertAlmostEqual(state.velocity[0],1.)
  self.assertGreater(state.sigma_m,.06)
 def test_bad_measurement(self):
  with self.assertRaises(ValueError):
   InertialMapEstimator.initial().predict((float("nan"),0.,0.),0.,.02)
if __name__=="__main__":unittest.main()
