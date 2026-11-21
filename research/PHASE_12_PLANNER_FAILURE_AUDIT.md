# Phase 12 development audit — preserve failed planner denominator

## Completed original Phase 12 comparison
Native MuJoCo run [44909014185](https://github.com/lalitkumar790/mars-lora/actions/runs/44909014185), development seeds 48221 and 63047, shortest/RF-aware, heavy UAV, 0.35 N disturbance, synthetic hybrid LiDAR+IMU and 45–75 s LiDAR outage.

| Policy | Expected | Physically executed | Safe missions | Planning rejects |
|---|---:|---:|---:|---:|
| Staged 10 m, .65 m/s altitude setpoint slew | 4 | 4 | 3 | 0 |
| First dynamic waypoint candidate | 4 | 0 | 0 | 4 |

First new candidate was universally rejected as `No dynamically screened exact-endpoint trajectory`. The CI workflow passed because it successfully ran the experiment; **zero flights must not be described as zero collisions or safety superiority**. A single different quintic reference native MuJoCo test completed safely in [44909014141](https://github.com/lalitkumar790/mars-lora/actions/runs/44909014141): 124.96 s, 0 contacts, min clearance 1.211 m, estimated ATE 0.0685 m. It is only a one-city development smoke test.

## Suspected mechanism and explicit diagnostic work
The original dynamic planner divided every segment length by the average of its *endpoint corner speeds*, even though the vehicle can accelerate above these knot speeds in the segment interior. This can make a 240-second route budget erroneously fail. This diagnosis was not established by the original summary (which lacked rejection reasons). The next commit adds reason counters (A* failure, connector collision, braking margin, duration budget) and uses a bounded trapezoidal/triangular segment-time formula.

New results must be reported separately as a modified development candidate. No reinterpretation of the original 0/4 candidate; do not tune using previously frozen Phase 11 cities 31489, 56003, 87643, 102671.

## Hard outstanding gaps
A 3D A* polyline with conservative stopping-distance speed caps is not a true rotor-feasible trajectory. Quintic point-mass reference constrains translation, but current tracking laws assume idealized state quantities, fixed rotor lag and wind model. Neither offers a guarantee for the heavy and Crazyflie airframes, consistent state-estimation covariance, dynamic obstacle prediction or LoRa time/energy optimization. Require explicit native completion/contact/clearance checks, then independently frozen test seeds before promoting.
