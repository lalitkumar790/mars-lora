import math,unittest
from mars_lora.lidar_imu3d import scan_at
from mars_lora.lidar_observability import assess_scan_observability

class LiDARObservabilityTests(unittest.TestCase):
 def test_featureless_airspace_unobservable(self):
  scan=[24.]*48
  a=assess_scan_observability((0.,0.,10.),0.,scan,())
  self.assertEqual(a.geometric_rank,0)
  self.assertEqual(a.valid_hits,0)
  self.assertFalse(a.informative)
  self.assertTrue(math.isinf(a.condition_number))
 def test_actual_known_map_scan_yields_finite_quality(self):
  scene=((4.,0.,4.,1.,2.,3.),(-3.,1.,5.,1.5,2.,3.),
         (0.,5.,6.,2.,1.,4.),(1.,-5.,5.,2.,1.,3.))
  sample=scan_at((0.,0.,4.),0.,scene)
  a=assess_scan_observability((0.,0.,4.),0.,sample,scene,min_hits=3)
  self.assertGreater(a.valid_hits,0)
  self.assertGreater(a.used_hits,0)
  self.assertAlmostEqual(a.rms_range_residual_m,0.,places=6)
  self.assertTrue(0<=a.geometric_rank<=3)
  self.assertGreaterEqual(a.largest_info_eigenvalue,a.smallest_info_eigenvalue)
 def test_mismatched_scan_residual_detected(self):
  scene=((4.,0.,4.,1.,2.,3.),(-3.,1.,5.,1.5,2.,3.),
         (0.,5.,6.,2.,1.,4.),(1.,-5.,5.,2.,1.,3.))
  scan=scan_at((0.,0.,4.),0.,scene)
  scan=[v+.3 if v<23.5 else v for v in scan]
  a=assess_scan_observability((0.,0.,4.),0.,scan,scene,min_hits=3)
  self.assertGreater(a.rms_range_residual_m,.25)
 def test_nonfinite_packet_rejected(self):
  with self.assertRaises(ValueError):
   assess_scan_observability((float('nan'),0.,4.),0.,[24.]*48,())
  with self.assertRaises(ValueError):
   assess_scan_observability((0.,0.,4.),0.,[24.]*47,())

if __name__=="__main__":unittest.main()
