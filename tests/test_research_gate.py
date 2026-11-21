"""Regression checks for honest heldout 3D UAV safety denominators."""
import unittest
from mars_lora.research_gate import score_safety

MODES = (("lidar_match", False),
         ("lidar_imu3d_hybrid", False),
         ("lidar_imu3d_hybrid", True))
ROUTES = ("shortest", "rf_aware")


def flight(seed, route, label, completed=True, contacts=0, clearance=.5):
    return dict(city_seed=seed, route=route, evaluation_policy=label,
                completed=completed, contact_samples=contacts,
                min_clearance_m=clearance, navigation_ate_rmse_m=.1,
                map_replans=0, map_replan_failures=0,
                lidar_matches_accepted=2)


class ResearchSafetyGateTests(unittest.TestCase):
    def test_missing_city_is_failure_not_removed_denominator(self):
        rows = [flight(18931, route, mode + ("_replan" if rp else "_no_replan"))
                for route in ROUTES for mode, rp in MODES]
        summary, gate = score_safety(
            rows, [{"city_seed": 73571, "error": "Unsafe start or goal"}],
            (73571, 18931), ROUTES, MODES)
        self.assertEqual(gate["expected_flights"], 12)
        self.assertEqual(gate["executed_flights"], 6)
        self.assertEqual(gate["planning_failure_flights"], 6)
        self.assertFalse(gate["complete_matrix"])
        self.assertFalse(gate["safety_gate_passed"])
        for policy in summary:
            self.assertEqual(policy["scenarios"], 4)
            self.assertEqual(policy["executed_flights"], 2)
            self.assertEqual(policy["planning_failures"], 2)
            self.assertEqual(policy["safety_pass"], 2)
            self.assertEqual(policy["safe_fraction_of_all_scenarios"], .5)

    def test_contact_or_insufficient_clearance_fails(self):
        rows = [flight(1, "shortest", "lidar_match_no_replan", contacts=1),
                flight(1, "rf_aware", "lidar_match_no_replan", clearance=.24)]
        summary, gate = score_safety(rows, [], (1,), ROUTES,
                                   (("lidar_match", False),))
        self.assertEqual(summary[0]["safety_pass"], 0)
        self.assertFalse(gate["safety_gate_passed"])

    def test_all_pass_is_only_successful_gate(self):
        rows = [flight(1, route, "lidar_match_no_replan") for route in ROUTES]
        _, gate = score_safety(rows, [], (1,), ROUTES,
                              (("lidar_match", False),))
        self.assertTrue(gate["safety_gate_passed"])

    def test_unexplained_missing_run_is_an_error(self):
        with self.assertRaisesRegex(ValueError, "Missing flight"):
            score_safety([], [], (1,), ROUTES, (("lidar_match", False),))

    def test_duplicate_run_is_an_error(self):
        row = flight(1, "shortest", "lidar_match_no_replan")
        with self.assertRaisesRegex(ValueError, "Duplicate flight"):
            score_safety([row, row], [], (1,), ROUTES,
                         (("lidar_match", False),))

    def test_partial_city_is_an_error(self):
        row = flight(1, "shortest", "lidar_match_no_replan")
        with self.assertRaisesRegex(ValueError, "Partial city"):
            score_safety([row], [{"city_seed": 1, "error": "plan failed"}],
                         (1,), ROUTES, (("lidar_match", False),))


if __name__ == "__main__":
    unittest.main()
