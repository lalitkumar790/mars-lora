import math,random,unittest
from mars_lora.lidar_map_matching import predict_scan,observe_scan,match_scan,RANGE_M
class MapAidedLidarTests(unittest.TestCase):
    def setUp(self):
        self.scene=(
            (6.,0.,4.,1.3,1.8,4.),
            (-7.,0.,4.,1.6,2.1,4.),
            (0.,7.,4.,2.2,1.2,4.),
            (0.,-8.,4.,1.2,2.0,4.),
            (5.,5.,4.,1.0,1.3,4.)
        )
    def test_deterministic_perfect_scan_is_position_dependent(self):
        a=predict_scan((.0,.0),.15,4.,self.scene)
        b=predict_scan((1.,0.),.15,4.,self.scene)
        self.assertEqual(len(a),16)
        self.assertNotEqual(a,b)
        self.assertEqual(a,predict_scan((0.,0.),.15,4.,self.scene))
    def test_no_gps_position_in_matcher(self):
        truth=(.0,.0);prior=(.7,-.6)
        scan=observe_scan(truth,.15,4.,self.scene,random.Random(55),noise_sigma=.01)
        corrected,accepted,score=match_scan(prior,.15,4.,scan,self.scene)
        self.assertTrue(accepted)
        self.assertLess(math.dist(corrected,truth),math.dist(prior,truth))
    def test_unobservable_high_altitude_rejected(self):
        scan=observe_scan((0.,0.),0.,19.,self.scene,random.Random(1))
        corrected,accepted,score=match_scan((.6,.6),0.,19.,scan,self.scene)
        self.assertFalse(accepted)
        self.assertEqual(corrected,(.6,.6))
    def test_invalid_scan_rejected(self):
        with self.assertRaises(ValueError):
            match_scan((0,0),0,4,[float("nan")]*16,self.scene)
if __name__=="__main__":unittest.main()
