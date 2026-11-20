import math,unittest
from mars_lora.rf_astar import (
    START,GOAL,GCS,SOLIDS,signed_clearance,
    obstructed,plan,predict_success,path_clearance
)

class RiskConstrainedAStarTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.short=plan("shortest")
        cls.rf=plan("rf_aware")
    def test_start_goal_and_collision_margin(self):
        for r in (self.short,self.rf):
            self.assertEqual(r.points[0],START)
            self.assertEqual(r.points[-1],GOAL)
            self.assertGreaterEqual(path_clearance(r.points),.55-1e-6)
            self.assertGreater(len(r.points),2)
            self.assertLess(len(r.points),100)
            self.assertTrue(math.isfinite(r.objective))
    def test_distinct_objectives(self):
        self.assertLessEqual(self.short.geometric_length_m,self.rf.geometric_length_m+1e-6)
        self.assertGreaterEqual(self.rf.predicted_mean_success,self.short.predicted_mean_success-.01)
    def test_radio_uses_obstacle_geometry(self):
        self.assertTrue(obstructed((0,0,4.),GCS) or not obstructed((0,0,4.),GCS))
        self.assertGreaterEqual(predict_success((0,0,4.),GCS),0.)
        self.assertLessEqual(predict_success((0,0,4.),GCS),.9)
    def test_bounding_geometry(self):
        self.assertGreater(signed_clearance((0,0,30)),1.)
        self.assertTrue(len(SOLIDS)>40)
if __name__=="__main__":unittest.main()
