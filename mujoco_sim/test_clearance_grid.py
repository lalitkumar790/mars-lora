"""Guard conservative broadphase never misses a dangerous urban collider."""
import math,random,unittest
from mujoco_sim.validate_mission import fast_city_clearance,CLEARANCE_CAP
from mars_lora.rf_astar import signed_clearance,SOLIDS

class FullCityClearanceGridTests(unittest.TestCase):
    def test_same_as_exact_inside_safety_envelope(self):
        rng=random.Random(7351)
        for i in range(300):
            x=rng.uniform(-38,38)
            y=rng.uniform(-38,38)
            z=rng.uniform(0,28)
            radius=.32 if i%2 else .08
            p=(x,y,z)
            exact=signed_clearance(p,SOLIDS)-radius
            fast=fast_city_clearance(p,radius)
            self.assertAlmostEqual(fast,min(CLEARANCE_CAP,exact),places=6)
    def test_near_building_collision_detected(self):
        for cx,cy,cz,hx,hy,hz in SOLIDS:
            actual=fast_city_clearance((cx,cy,cz),.08)
            self.assertLess(actual,0)
    def test_outside_obstacle_is_safe(self):
        self.assertGreater(fast_city_clearance((0,0,35),.32),.25)
if __name__=="__main__":unittest.main()
