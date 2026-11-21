import math,unittest
from mars_lora.kinodynamic_reference import quintic_reference
class QuinticReferenceTests(unittest.TestCase):
 def test_exact_endpoints_and_bounded_kinematics(self):
  r=quintic_reference([(0.,0.,4.),(4.,0.,4.),(4.,4.,4.)],(),
      max_speed_m_s=1.,max_accel_m_s2=.8,max_jerk_m_s3=1.5)
  self.assertEqual(r.samples[0].xyz,(0.,0.,4.))
  self.assertEqual(r.samples[-1].xyz,(4.,4.,4.))
  self.assertLessEqual(r.measured_max_speed_m_s,1.+1e-7)
  self.assertLessEqual(r.measured_max_accel_m_s2,.8+1e-7)
  self.assertLessEqual(r.measured_max_jerk_m_s3,1.5+1e-7)
  self.assertTrue(all(b.t_s>a.t_s for a,b in zip(r.samples,r.samples[1:])))
  at_corner=[p for p in r.samples if math.dist(p.xyz,(4.,0.,4.))<1e-7]
  self.assertEqual(len(at_corner),1)
  self.assertLess(math.dist(at_corner[0].velocity_xyz,(0.,0.,0.)),1e-7)
  self.assertLess(math.dist(at_corner[0].acceleration_xyz,(0.,0.,0.)),1e-7)
 def test_blocked_polyline_rejected(self):
  box=((2.,0.,4.,.4,.4,1.),)
  with self.assertRaises(ValueError):
   quintic_reference([(0.,0.,4.),(4.,0.,4.)],box)
 def test_deadline_rejection(self):
  with self.assertRaises(RuntimeError):
   quintic_reference([(0.,0.,4.),(50.,0.,4.)],(),
       max_speed_m_s=.1,max_duration_s=5.)
 def test_invalid_model_rejection(self):
  with self.assertRaises(ValueError):
   quintic_reference([(0.,0.,4.),(4.,0.,4.)],(),max_jerk_m_s3=0)
if __name__=="__main__":
 unittest.main()
