import unittest
from mars_lora.replan_envelope import check_handoff

BOX=((4.,0.,4.,.6,1.,4.),)
BASE=dict(vehicle_radius_m=.3,localization_sigma_m=.1,
          required_clearance_m=.25,braking_accel_m_s2=.8,
          actuator_delay_s=.2,wind_accel_bound_m_s2=.15)

class ReplanEnvelopeTests(unittest.TestCase):
    def test_open_airspace_allowed(self):
        r=check_handoff((0.,0.,4.),(0.,0.,0.),
                        [(0.,0.,4.),(0.,3.,4.)],BOX,**BASE)
        self.assertTrue(r.accepted,r.reason)

    def test_fast_vehicle_stopping_into_wall_rejected(self):
        r=check_handoff((0.,0.,4.),(2.,0.,0.),
                        [(0.,0.,4.),(0.,3.,4.)],BOX,**BASE)
        self.assertFalse(r.accepted)
        self.assertEqual(r.reason,'unsafe_stopping_corridor')

    def test_safe_nominal_splice_but_colliding_first_leg_rejected(self):
        r=check_handoff((0.,0.,4.),(0.,0.,0.),
                        [(1.,0.,4.),(8.,0.,4.)],BOX,**BASE)
        self.assertFalse(r.accepted)
        self.assertEqual(r.reason,'unsafe_splice')

    def test_unsafe_map_splice_rejected(self):
        r=check_handoff((0.,0.,4.),(0.,0.,0.),
                        [(4.,0.,4.)],BOX,**BASE)
        self.assertFalse(r.accepted)
        self.assertIn(r.reason,('splice_too_long','unsafe_splice'))

    def test_uncertainty_inflation_matters(self):
        box=((1.5,0.,4.,.3,.3,1.),)
        low=check_handoff((0.,0.,4.),(0.,0.,0.),
                          [(0.,0.,4.),(0.,2.,4.)],box,
                          **{**BASE,'localization_sigma_m':0.})
        high=check_handoff((0.,0.,4.),(0.,0.,0.),
                           [(0.,0.,4.),(0.,2.,4.)],box,
                           **{**BASE,'localization_sigma_m':.5})
        self.assertTrue(low.accepted)
        self.assertFalse(high.accepted)

    def test_bad_sensor_state_rejected(self):
        with self.assertRaises(ValueError):
            check_handoff((float('nan'),0.,4.),(0.,0.,0.),
                          [(0.,0.,4.)],BOX,**BASE)

    def test_bad_braking_model_rejected(self):
        with self.assertRaises(ValueError):
            check_handoff((0.,0.,4.),(1.,0.,0.),
                          [(0.,0.,4.)],BOX,**{**BASE,'braking_accel_m_s2':.1})

if __name__=='__main__':
    unittest.main()
