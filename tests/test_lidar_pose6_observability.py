import math,random,unittest
import numpy as np
from mars_lora.body_imu_eskf15 import exp_quat
from mars_lora.lidar_pose6_observability import (
    body_scan,simulate_body_scan,assess_pose6)

SCENE=((4.,0.,5.,1.,2.,4.),(-4.,1.,3.,1.,2.,3.),
       (0.,5.,6.,2.,1.,4.),(2.,-5.,5.,2.,1.,4.),
       (4.,4.,9.,1.5,1.,2.),(-3.,-4.,9.,1.,1.,2.))

class Pose6Tests(unittest.TestCase):
 def test_no_returns_do_not_claim_rank(self):
  a=assess_pose6((0.,0.,5.),(1.,0.,0.,0.),[24.]*48,())
  self.assertEqual(a.rank6,0)
  self.assertFalse(a.locally_full_rank)
 def test_body_tilt_changes_ranges_where_geometry_visible(self):
  q=(1.,0.,0.,0.)
  r=body_scan((0.,0.,5.),q,SCENE)
  tilted=body_scan((0.,0.,5.),exp_quat((.21,-.11,.07)),SCENE)
  self.assertGreater(float(np.max(np.abs(r-tilted))),.05)
 def test_pose_fit_perfect_scan_has_zero_residual(self):
  p=(0.,0.,5.)
  q=exp_quat((.04,.08,.15))
  r=body_scan(p,q,SCENE)
  a=assess_pose6(p,q,r,SCENE,min_hits=3)
  self.assertGreater(a.hits,0)
  self.assertGreater(a.usable,0)
  self.assertEqual(a.residual_rmse_m,0.)
  self.assertTrue(0<=a.rank6<=6)
  self.assertTrue(0<=a.translation_rank<=3)
  self.assertTrue(0<=a.rotation_rank<=3)
 def test_wrong_pose_produces_residual(self):
  p=(0.,0.,5.)
  q=(1.,0.,0.,0.)
  true=body_scan(p,q,SCENE)
  candidate=assess_pose6((.3,.2,5.),q,true,SCENE,min_hits=3)
  self.assertGreater(candidate.residual_rmse_m,.02)
 def test_reproducible_simulated_sensor(self):
  p=(0.,0.,5.);q=(1.,0.,0.,0.)
  a=simulate_body_scan(p,q,SCENE,random.Random(123))
  b=simulate_body_scan(p,q,SCENE,random.Random(123))
  self.assertEqual(a,b)
 def test_bad_input(self):
  with self.assertRaises(ValueError):
   assess_pose6((0.,0.,5.),(1.,0.,0.,0.),[24.]*47,SCENE)
  with self.assertRaises(ValueError):
   assess_pose6((0.,0.,5.),(1.,0.,0.,0.),[24.]*48,SCENE,rotation_step_rad=0)
if __name__=="__main__":unittest.main()
