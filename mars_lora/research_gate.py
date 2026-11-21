"""End-to-end scenario accounting for experimental flight safety.

A planner failure is a failed mission, never an omitted denominator.
ATE is calculated only for physically executed flights and is NOT a safety score.
"""
from __future__ import annotations
from statistics import mean


def score_safety(rows, planner_errors, city_seeds, routes, modes):
    """Return per-policy and global safety outcomes for a frozen experiment matrix.

    modes is a sequence of (localization_mode, dynamic_replanning) pairs.
    Each (city, route, mode, replanning) tuple must occur at most once.
    """
    seeds = tuple(city_seeds)
    route_names = tuple(routes)
    policies = tuple((mode, replan, mode + ("_replan" if replan else "_no_replan"))
                     for mode, replan in modes)
    if len(set(seeds)) != len(seeds) or len(set(route_names)) != len(route_names):
        raise ValueError("Experiment matrix has duplicate cities or routes")
    if len({label for _, _, label in policies}) != len(policies):
        raise ValueError("Experiment matrix has duplicate policy labels")
    expected = {(seed, route, label)
                for seed in seeds for route in route_names
                for _, _, label in policies}
    observed = {}
    for row in rows:
        key = (int(row["city_seed"]), row["route"], row["evaluation_policy"])
        if key not in expected:
            raise ValueError("Unexpected flight in experiment results: %r" % (key,))
        if key in observed:
            raise ValueError("Duplicate flight in experiment results: %r" % (key,))
        observed[key] = row
    failed_cities = {int(item["city_seed"]) for item in planner_errors}
    if failed_cities.difference(seeds):
        raise ValueError("Planner failure belongs to an unknown city")
    for seed in failed_cities:
        if any(key[0] == seed for key in observed):
            raise ValueError("Partial city with planner failure needs explicit per-route status")
    summaries = []
    for _, _, label in policies:
        keys = {(seed, route, label) for seed in seeds for route in route_names}
        executed = [observed[key] for key in sorted(keys) if key in observed]
        missing = keys.difference(observed)
        unexplained = [key for key in missing if key[0] not in failed_cities]
        if unexplained:
            raise ValueError("Missing flight without a recorded planner failure: %r" %
                             (unexplained,))
        passed = sum(bool(x["completed"]) and int(x["contact_samples"]) == 0
                     and float(x["min_clearance_m"]) >= .25 for x in executed)
        summaries.append({
            "mode": label, "scenarios": len(keys), "executed_flights": len(executed),
            "planning_failures": len(missing), "safety_pass": passed,
            "safety_fail": len(keys) - passed,
            "safe_fraction_of_all_scenarios": passed / len(keys) if keys else 0.,
            "mean_ate_m_executed_only": (
                mean(float(x["navigation_ate_rmse_m"]) for x in executed)
                if executed else None),
            "total_map_replans": sum(int(x["map_replans"]) for x in executed),
            "total_replan_failures": sum(int(x["map_replan_failures"]) for x in executed),
            "total_lidar_corrections": sum(int(x["lidar_matches_accepted"])
                                           for x in executed)
        })
    total = len(expected)
    return summaries, {
        "expected_flights": total, "executed_flights": len(observed),
        "planning_failure_flights": total - len(observed),
        "safe_flights": sum(x["safety_pass"] for x in summaries),
        "complete_matrix": len(observed) == total,
        "safety_gate_passed": (len(observed) == total and
                              all(x["safety_pass"] == x["scenarios"] for x in summaries))
    }
