import math,unittest
from mars_lora.active_viewpoint import choose_observable_detour
from mars_lora.rf_astar import path_clearance
from mars_lora.kinodynamic_reference import quintic_reference

class ViewpointPlanningTests(unittest.TestCase):
 def test_featureless_scene_never_invents_gain(self):
  route=[(0.,0.,4.),(0.,0.,10.),(-12.,0.,10.),
         (-12.,0.,4.)]
  p=choose_observable_detour(route,(),max_reference_s=210.)
  self.assertIsNone(p.inserted_waypoint)
  self.assertEqual(p.outcome,"baseline_no_verified_gain")
  self.assertEqual(p.points[0],route[0])
  self.assertEqual(p.points[-1],route[-1])
  self.assertEqual(p.baseline_informative_fraction,0.)
  self.assertEqual(p.candidate_informative_fraction,0.)
 def test_enforced_zero_extra_length(self):
  route=[(0.,0.,4.),(0.,0.,10.),(-12.,0.,10.),(-12.,0.,4.)]
  p=choose_observable_detour(route,(),max_extra_length_m=0.)
  self.assertIsNone(p.inserted_waypoint)
  self.assertEqual(p.points[-1],route[-1])
 def test_never_changes_exact_endpoints_or_exceeds_deadline(self):
  obstacles=((6.,4.,10.,1.5,1.5,2.),(-6.,-4.,10.,1.5,1.5,2.))
  route=[(0.,0.,4.),(0.,0.,10.),(-15.,0.,10.),(-15.,0.,4.)]
  p=choose_observable_detour(route,obstacles,max_reference_s=210.,
                              minimum_gain=0.)
  self.assertEqual(p.points[0],route[0])
  self.assertEqual(p.points[-1],route[-1])
  self.assertLessEqual(p.candidate_duration_s,210.)
  self.assertGreaterEqual(
     path_clearance(p.points,radius=.32,solids=obstacles),.85-1e-6)
  ref=quintic_reference(p.points,obstacles,radius_m=.32,sigma_m=.1,
        clearance_margin_m=.55,max_duration_s=210.)
  self.assertLessEqual(ref.measured_max_jerk_m_s3,1.5+1e-7)
 def test_invalid_route_rejected(self):
  with self.assertRaises(ValueError):
   choose_observable_detour([(0.,0.,4.)],())
  with self.assertRaises(ValueError):
   choose_observable_detour([(0.,0.,4.),(float('nan'),0.,4.)],())

if __name__=="__main__":unittest.main()
