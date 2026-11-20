import math
import unittest
from mars_lora.urban_scene import city,add_city_mjcf
from experiments.replay_mujoco_radio import intersect_segment,obstructed,trial

class UrbanReplayTests(unittest.TestCase):
    def test_city_determinism_and_mjcf(self):
        buildings=city()
        self.assertEqual(buildings,city())
        self.assertGreater(len(buildings),40)
        text=add_city_mjcf('<mujoco><worldbody></worldbody></mujoco>')
        self.assertEqual(text.count('name="urban_'),len(buildings))
    def test_los_ray_not_ground_truth_cheat(self):
        box=(5,0,4,2,2,4)
        self.assertTrue(intersect_segment((0,0,4),(10,0,4),box))
        self.assertFalse(intersect_segment((0,0,12),(10,0,12),box))
    def test_policy_replay_repeatable(self):
        trace=[(i*.25,float(i*.03),0.,3.) for i in range(80)]
        a=trial(trace,'adaptive',11,(230.,220.,5.))
        b=trial(trace,'adaptive',11,(230.,220.,5.))
        self.assertEqual(a,b)
        self.assertTrue(math.isfinite(a['aoi_p95_s']))
        self.assertGreaterEqual(a['pdr'],0)
        self.assertLessEqual(a['pdr'],1)

if __name__=='__main__':
    unittest.main()
