import unittest
from mars_lora.deadline_gate import choose_deadline_route,predict_candidate
from mars_lora.rf_astar import START,GOAL
from experiments.replay_mujoco_radio import trial

class JointGateTests(unittest.TestCase):
    def setUp(self):
        self.routes={"shortest":[START,(-3.,0.,4.),GOAL],
                     "rf_aware":[START,(0.,0.,17.),(-30.,24.,17.),GOAL]}
    def test_returns_route_sf_and_score_without_outcomes(self):
        chosen,candidates=choose_deadline_route(self.routes,-121.,.13)
        self.assertIn(chosen.route,self.routes)
        self.assertIn(chosen.spreading_factor,range(7,13))
        self.assertEqual(len(candidates),2)
        self.assertGreater(chosen.estimated_alert_success,0)
        self.assertLessEqual(chosen.estimated_alert_success,1)
        self.assertLessEqual(chosen.score,max(x.score for x in candidates))
    def test_budgeted_deadline_policy_never_chooses_long_airtime(self):
        choice,candidates=choose_deadline_route(
            self.routes,-132.,.32,airtime_budget_fraction=.20)
        for cand in candidates:
            self.assertLessEqual(cand.expected_airtime_s,3.*.20*(1-.32)+1e-9)
        self.assertIn(choice.spreading_factor,(7,8,9,10,11,12))
        with self.assertRaises(ValueError):
            choose_deadline_route(self.routes,-132.,.32,airtime_budget_fraction=-.2)
    def test_invalid_radio_constraints(self):
        with self.assertRaises(ValueError):
            choose_deadline_route(self.routes,float("nan"),.1)
        with self.assertRaises(ValueError):
            choose_deadline_route(self.routes,-110.,1.2)
        with self.assertRaises(ValueError):
            choose_deadline_route({"shortest":self.routes["shortest"]},-110.,.1)
    def test_mismatched_mission_endpoints_rejected(self):
        altered=dict(self.routes,rf_aware=[START,(3.,3.,7.),(5.,5.,4.)])
        with self.assertRaises(ValueError):
            choose_deadline_route(altered,-120.,.1)
    def test_override_changes_actual_radio_policy(self):
        trace=[(i*.25,0.,.01*i,4.) for i in range(140)]
        x=trial(trace,'priority-adaptive',42,(230.,220.,120.),
                sf_override=12,spatial_fading=True)
        y=trial(trace,'priority-adaptive',42,(230.,220.,120.),
                sf_override=7,spatial_fading=True)
        self.assertEqual(x['sf_override'],12)
        self.assertEqual(y['sf_override'],7)
        self.assertNotEqual(x['airtime_s'],y['airtime_s'])
        with self.assertRaises(ValueError):
            trial(trace,'fixed',1,(10.,10.,5.),sf_override=14)

if __name__=="__main__":unittest.main()
