# Phase 7 continuation: full-matrix independent-city safety scoring

**Scope:** This is a research-integrity and safety-acceptance fix, not a successful UAV controller improvement. The experimental 3D hybrid remains rejected for deployment.

## Why the previous report overstates coverage

The frozen V5 hybrid was evaluated on additional procedural cities 73571 and 18931. For seed 73571, static A* returned `Unsafe start or goal`, causing **all six expected policy/route combinations to be skipped**. Seed 18931 produced six physical trials: the 2D known-map baseline was safe in 2/2; the 3D hybrid was safe in 0/2 without replanning and 0/2 with replanning. These results come from `results/imu3d_independent_cities/summary.json`. They were executed in MuJoCo, never physical aircraft tests.

The original summary grouped only completed simulations, giving a misleading denominator of two per policy, not the four *planned* scenarios. The full-matrix report now scores unplannable mission scenarios as unsuccessful, without pretending that those flights occurred.

| Policy | Safe / expected missions | Actually flown | Blocked at planning |
|---|---:|---:|---:|
| 2D map-aided reference | 2/4 | 2 | 2 |
| 3D hybrid, no replan | 0/4 | 2 | 2 |
| 3D hybrid, replan | 0/4 | 2 | 2 |

**Overall:** 2 safe missions out of 12 expected policy-scenario combinations. The 2D reference is not considered fully validated either: a preflight planning failure is still a failed mission attempt.

The frozen development scenarios (91311, 20483) remain a separate set and were not pooled into the independent-city table.

## New scoring invariant

`mars_lora/research_gate.py` enumerates the frozen Cartesian product of city seeds, routes and policies. It explicitly reports:
- planned and physically executed flights;
- no-plan outcomes, recorded as end-to-end failures;
- collision/clearance/completion failures among executed flights;
- position ATE only for physically executed flights, never as evidence of safety;
- a hard success gate that only passes if every predeclared case executes and all meet the safety criterion.

Duplicate runs, unknown cases, missing unexplained runs, and ambiguous partially executed city failures raise errors instead of silently influencing the denominator. Regression cases are in `tests/test_research_gate.py`; a lightweight GitHub workflow runs them on changes.

## Controller remains blocked

The replan handoff currently verifies geometric clearance and low speed, but not a rigorous forward reachable set for the actual state-estimation tail, rotor lag, braking distance, wind acceleration, and vehicle footprint. During a 30-second LiDAR outage, a low **mean** trajectory error cannot guarantee a low *worst-case local error* near a building.

The next controller experiment should:
1. Stop any transition when the current estimated position's **full uncertainty tube** plus swept vehicle radius intersects an obstacle; do not substitute a nominal A* path check.
2. Add a dynamically feasible stopping corridor from estimated velocity, bounded deceleration, actuator delay and wind; permit emergency hold only if the hold location and stopping trajectory are safe.
3. Track stale localization age and innovation consistency; explicitly fail safely when sensors cannot constrain pose.
4. Freeze dynamics and uncertainty thresholds using development cities. Evaluate new heldout cities, both airframes, varied winds, LiDAR outages, plus contact and 0.25 m clearance accounting. A failed preflight is a failed attempt, not excluded.
5. Only after passing the declared simulation envelope, compare real LiDAR/IMU and LoRa/energy measurements using the separate hardware protocol. Do **not** promote the 3D estimator into the live browser controller based on this audit.

Even a successful native MuJoCo sweep would not establish field-safe flight, unknown-map 3D SLAM, RF calibration, or authorization to fly.
