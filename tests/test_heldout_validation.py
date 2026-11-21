import math
import unittest
from experiments.replay_mujoco_radio import spatial_shadow_field,trial
from experiments.heldout_validation import PROFILES,STATIONS

class HeldOutValidationTests(unittest.TestCase):
    def test_spatial_field_repeats_at_same_location(self):
        position=(3.2,-5.7,13.)
        a=spatial_shadow_field(position,241,5.)
        self.assertEqual(a,spatial_shadow_field(position,241,5.))
        self.assertNotEqual(a,spatial_shadow_field(position,242,5.))
        self.assertEqual(spatial_shadow_field(position,241,0),0)
    def test_spatial_field_is_continuous(self):
        a=spatial_shadow_field((4.1,4.3,10.),31,5.)
        b=spatial_shadow_field((4.101,4.301,10.),31,5.)
        self.assertLess(abs(a-b),.01)
    def test_two_distinct_channels_and_deterministic_replay(self):
        trace=[(i*.25,.04*i,.03*i,4.) for i in range(140)]
        a=trial(trace,'priority-adaptive',17,STATIONS['nominal_spatial'],4.,
                profile=PROFILES['nominal_spatial'],spatial_fading=True)
        b=trial(trace,'priority-adaptive',17,STATIONS['nominal_spatial'],4.,
                profile=PROFILES['nominal_spatial'],spatial_fading=True)
        self.assertEqual(a,b)
        self.assertTrue(0<=a['pdr']<=1)
        self.assertTrue(0<=a['critical_deadline_ratio']<=1)
        self.assertTrue(math.isfinite(a['aoi_p95_s']))
        self.assertEqual(a['propagation_profile'],'nominal_spatial')
        self.assertTrue(a['spatial_fading'])
    def test_no_always_better_assumption(self):
        self.assertEqual(len(PROFILES),5)
        self.assertNotEqual(STATIONS['receiver_mismatch'],STATIONS['nominal_spatial'])
        trace=[(i*.25,0.,0.,5.) for i in range(120)]
        out=trial(trace,'fixed',8,(230.,220.,120.),profile=PROFILES['dense_urban'],spatial_fading=True)
        self.assertGreaterEqual(out['nlos_fraction'],0)
        self.assertLessEqual(out['outage_fraction'],1)
    def test_invalid_channel_parameters_raise(self):
        trace=[(i*.25,0.,0.,4.) for i in range(20)]
        with self.assertRaises(ValueError):
            trial(trace,'fixed',1,(20.,20.,5.),profile={'path_loss_exponent':0})
if __name__=='__main__':
    unittest.main()
