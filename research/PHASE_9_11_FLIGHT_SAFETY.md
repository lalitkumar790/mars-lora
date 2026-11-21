# Phase 9–11: root-cause isolation and frozen new-city validation

## Phase 8 negative outcome
Native MuJoCo paired development run 44883525144: the nominal 3D LIO hybrid and added stopping-envelope veto both passed only 1/4 missions. Both accepted 7 replans; the added envelope veto fired zero times. A correct unit test does not establish useful flight safety.

## Phase 9: observed failure modes
Run 44907209375 matched two cities (91311,20483), shortest and RF-aware routes, heavy model, original hybrid estimator, identical wind and LiDAR blackout, **no dynamic replanning**. Compared original A* route and an independently static-collision-checked climb-to-10m / high-altitude traverse / descend-to-4m route.

| Metric (4 routes per variant) | Original A* | Staged 10m |
|---|---:|---:|
| Fully safe flights | 1 | 0 |
| Under-clearance flights | 2 | 0 |
| Contact flights | 1 | 4 |
| Completed flights | 4 | 4 |

Original low-route under-clearance occurred by ~7.3 s near (-3,0,~5)m, consistent with flying into the overhead beam's clearance envelope. The staged route eliminates these geometry breaches but every flight subsequently hits the ground near the destination around z=0.04 m. The latter implicates the altitude setpoint/controller dynamics rather than a nearby building. Evidence is *correlation with changing the stage*; controller mechanism remains a modeling hypothesis until additional dynamics tests.

## Phase 10: altitude-command slew development result
Run 44907405276 compared the *same* staged 10m geometry with original instantaneous altitude command versus a bounded 0.65 m/s **command slew**. Four matched heavy-UAV development flights per variant, dynamic replanning off.

| Metric | Instantaneous setpoint | Command slew 0.65m/s |
|---|---:|---:|
| Fully safe flights | 0/4 | 4/4 |
| Contact flights | 4/4 | 0/4 |
| Under-clearance flights | 0/4 | 0/4 |
| Mission completed | 4/4 | 4/4 |

This is a favorable **development** result, NOT independent confirmation or a guarantee of the real UAV's speed/acceleration limits. The command rate is not actual vertical velocity and does not prove robust stopping or controllability. The original V1–V5 and Phase 8 negatives remain intact.

## Phase 11 frozen holdout
The predeclared candidate is **exactly**:
- 10m staged static map-known route;
- 0.65m/s setpoint rate limit;
- heavy airframe, legacy controller, 3D LiDAR/IMU hybrid;
- dynamic replanning **OFF**, drift 0.03m/sqrt(s), position noise 0.02m, 30 s LiDAR blackout from t=45–75s;
- two route types, cities [31489,56003,87643,102671], wind forces [0.25N,0.50N] (16 expected missions);
- hard gate is 16/16 completion and zero MuJoCo contacts and city clearance >=0.25m. Any no-plan or simulation-failure scenario stays in denominator.

No tuning to Phase 11 outcomes is permitted. If it fails, preserve failure and develop a *new* candidate against a separate training/development set; later confirmation needs another frozen environment split. Even passing Phase 11 would only verify this specific deterministic simulation envelope, with static known-map geometry and one airframe.

## Future evidence required
Both airframes; moving obstacle and uncertain map perception; independently simulated sensor drivers with realistic extrinsics, timing, motion compensation and IMU bias estimation; actual whole-vehicle power and RF handshake; hardware LoRa measurement and safe supervisory flights. Keep the browser simulator separate from offline research claims.
