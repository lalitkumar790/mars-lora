import math,unittest
from mars_lora.urban_scene import CITY_SEED,city
from mars_lora.rf_astar import INDUSTRIAL,plan,path_clearance,GOAL,START
from mars_lora.deadline_gate import choose_deadline_route
from mars_lora.route_gate import simulated_pilot
from experiments.unseen_city_validation import CITY_SEEDS,HOLDOUT_PROFILES,SIGMA_M

class UnseenCityPolicyTests(unittest.TestCase):
    def test_seeds_are_distinct_from_original(self):
        self.assertEqual(len(CITY_SEEDS),len(set(CITY_SEEDS)))
        self.assertNotIn(CITY_SEED,CITY_SEEDS)
        self.assertNotEqual(city(CITY_SEEDS[0]),city(CITY_SEEDS[1]))
    def test_spatial_clearance_and_finite_radio_scores(self):
        seed=CITY_SEEDS[0]
        solids=INDUSTRIAL+tuple(city(seed))
        routes={name:plan(name,solids=solids,localization_sigma_m=SIGMA_M)
                for name in ("shortest","rf_aware")}
        for r in routes.values():
            self.assertEqual(tuple(r.points[0]),START)
            self.assertEqual(tuple(r.points[-1]),GOAL)
            self.assertGreaterEqual(path_clearance(r.points,.32,solids=solids),
                                    .55+3*SIGMA_M-1e-6)
        config=HOLDOUT_PROFILES["shifted_relay"]
        station=config["station"]
        pilot=simulated_pilot(config,station,701,scene=solids)
        route,alternatives=choose_deadline_route(
            {k:r.points for k,r in routes.items()},
            pilot["pilot_rssi_dbm"],pilot["observed_channel_busy"],
            receiver=station,solids=solids,airtime_budget_fraction=.20,
            flight_hover_power_w=139.6,propulsion_energy_weight=.003)
        self.assertIn(route.route,routes)
        self.assertEqual(len(alternatives),2)
        self.assertTrue(all(math.isfinite(r.score) for r in alternatives))
        self.assertTrue(all(r.estimated_propulsion_energy_j>0 for r in alternatives))
    def test_wider_uncertainty_never_reduces_safety_margin(self):
        solids=INDUSTRIAL+tuple(city(CITY_SEEDS[0]))
        a=plan("shortest",solids=solids,localization_sigma_m=.0)
        b=plan("shortest",solids=solids,localization_sigma_m=.10)
        self.assertGreaterEqual(b.minimum_clearance_m,.85-1e-6)
        self.assertGreaterEqual(b.geometric_length_m,a.geometric_length_m)
if __name__=="__main__":unittest.main()
