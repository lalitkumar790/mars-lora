# Phase 11 heldout results: staged route + bounded altitude command fails full safety gate

**Run:** [GitHub Actions #44907582206](https://github.com/lalitkumar790/mars-lora/actions/runs/44907582206). Complete artifact: `phase11-frozen-unseen-city-flights`, containing `summary.json`, trial CSV and physical MuJoCo traces. This is an *independent procedural city split* for simulation, not field or hardware validation.

## Frozen settings before observing results

Four new city seeds 31489, 56003, 87643, 102671; both geometric-shortest and RF-aware staged 10m routes; wind force 0.25N and 0.50N; heavy UAV; 0.65m/s commanded altitude slew; legacy controller; 3D map-aided LiDAR/IMU hybrid; 45–75s LiDAR blackout; dynamic replanning disabled. Seeded synthetic sensor noise and 240s simulation duration. Declared acceptance gate: **16/16** complete, zero MuJoCo contact samples, minimum city obstacle clearance >=0.25m; failed planning also counts as a failed attempt.

## Actually executed outcomes

| Metric | Result |
|---|---:|
| Expected missions | 16 |
| Executed flight trials | 12 |
| Safe (complete/no contacts/clearance >=0.25m) | **10/16** |
| Failed before flight at preflight planning | 4 |
| Executed trials with physical contacts | 2 |
| Executed trials with insufficient city clearance | 2 |
| Incomplete flights | 1 |

The under-clearance/contact/incompletion failure counts overlap. Do not sum them.

- City 31489: the staged route violates static clearance for **both policies**, so 4/4 planned route/wind combinations are explicitly failed as preflight rejection, not ignored. A failed plan is safer than forcing an unsafe trajectory, but it is not a completed mission.
- City 56003, RF-aware: fails at both wind forces. At 0.25N, first 0.25m clearance breach t=118.796s, first contact t=119.096s near (-31.46,27.92,20.99)m. At 0.50N, first clearance breach t=111.456s, contact t=111.916s near (-32.43,19.83,20.97)m.
- The other ten physically executed scenarios pass. This is not evidence of 10/12 unconditional mission safety; scientific denominator includes four rejected preflights.

**Decision: full safety gate FAILED. Do not merge this controller into the live simulator. Do not retune 10m or 0.65m/s using these holdout cases and then call the rerun independent validation.** The previous Phase 10 four-flight development pass is preserved as a development observation only.

## Root-cause hypotheses still to test

1. **Departure/arrival feasibility:** stage generation hardcodes a vertical raise/descent at the original endpoint and can be geometrically impossible in unseen cities. A future planner must select feasible approach/terminal waypoints or explicitly no-go, with no hiding missing missions.
2. **Corner/obstacle tracking envelope:** nominal A* path clearance did not prevent actual heavy UAV collision near high-rise geometry at altitude ~21m. Smooth dynamically feasible timed trajectories, acceleration/jerk bounds and uncertainty tubes need native physics tests against *separate development cities*.
3. **Localization uncertainty:** current estimator's scalar `sigma_m` is not a statistically consistent covariance. Poor scan observability and false registration remain possible. The 56003 collisions cannot be causally attributed to localization without time-aligned innovation/velocity/waypoint telemetry.
4. **Vehicle control:** a setpoint slew fixed the late ground-contact failure in development but does not demonstrate bounded real vertical acceleration or no collision under stronger winds.

Next experiment should be developed using other training seeds, then frozen and evaluated in a **new** holdout split. Do not reuse the four Phase 11 scenes as confirmatory evidence. Tests on Crazyflie, dynamic obstacles, actual IMU/LiDAR and radio hardware are still absent.
