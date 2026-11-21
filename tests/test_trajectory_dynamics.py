import math
import unittest
from mars_lora.trajectory_dynamics import build_trajectory
from mars_lora.rf_astar import path_clearance

class DynamicTrajectoryTests(unittest.TestCase):
    def test_exact_endpoints_and_timing(self):
        p=build_trajectory("shortest",(),start=(0.,0.,4.),
            goal=(-12.,12.,4.))
        self.assertEqual(p.points[0],(0.,0.,4.))
        self.assertEqual(p.points[-1],(-12.,12.,4.))
        self.assertEqual(len(p.points),len(p.waypoint_speed_limits_m_s))
        self.assertEqual(len(p.points),len(p.arrival_times_s))
        self.assertEqual(p.waypoint_speed_limits_m_s[0],0.)
        self.assertEqual(p.waypoint_speed_limits_m_s[-1],0.)
        self.assertTrue(all(b>a for a,b in zip(p.arrival_times_s,p.arrival_times_s[1:])))
        self.assertTrue(all(0<=v<=p.max_speed_m_s+1e-7
                            for v in p.waypoint_speed_limits_m_s))
    def test_blocked_launch_column_uses_alternative(self):
        # The obstacle starts above the initial pose; a straight
        # climb into it must not be used.
        roof=((0.,0.,8.,1.4,1.4,1.),)
        p=build_trajectory("shortest",roof,
            start=(0.,0.,4.),goal=(-12.,12.,4.))
        self.assertEqual(p.points[0],(0.,0.,4.))
        self.assertNotEqual(p.points[1],(0.,0.,10.))
        self.assertGreaterEqual(path_clearance(p.points,radius=.32,solids=roof),p.minimum_margin_m-1e-5)
    def test_blocked_end_point_is_rejected(self):
        with self.assertRaises(ValueError):
            build_trajectory("shortest",((12.,0.,4.,2.,2.,2.),),
                 start=(0.,0.,4.),goal=(12.,0.,4.))
    def test_nonphysical_braking_assumptions_rejected(self):
        with self.assertRaises(ValueError):
            build_trajectory("shortest",(),braking_m_s2=.1,wind_accel_bound_m_s2=.2)
    def test_reject_zero_route(self):
        with self.assertRaises(ValueError):
            build_trajectory("shortest",(),start=(0.,0.,4.),goal=(0.,0.,4.))
if __name__=="__main__":
    unittest.main()
