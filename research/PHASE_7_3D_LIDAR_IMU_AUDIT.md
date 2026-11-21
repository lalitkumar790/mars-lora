# Phase 7: 3D LiDAR/IMU and uncertainty-aware replanning

## The problem addressed

The earlier native UAV controller used oracle map, heading, velocity and ground-truth pose for most navigation. Uncertain map-known A* could be collision-safe geometrically, but drift in actual estimated position made that guarantee invalid. This phase tests the **failure modes** of 3D LiDAR/inertial corrections and online path handoffs. It does NOT establish full simultaneous mapping or hardware GPS-denied autonomy.

## Implementation

- mars_lora/lidar_imu3d.py: 48 rays over 12 azimuths and 4 elevation angles; direct ray-box intersection against known 3D urban obstacles; seeded range noise and dropout; bounded coarse/fine pose registration. Inertial state propagates XYZ, velocity and yaw from synthetic finite-difference accelerations and angular rate, corrected by range registration.
- Native MuJoCo controller supports lidar_imu3d, and lidar_imu3d_hybrid (barometric altitude observation and optical-flow horizontal velocity aiding, plus planar scan fallback when 3D registration fails). All sensor observations are generated with known simulator truth, but the estimator receives the noisy measurements, not that truth position; yaw and motor-attitude control still have idealized MuJoCo dependencies.
- Runtime planner uses a map-known uncertainty estimate, three-sigma inflated clearance, and 3D A* after localization confidence decreases. Its policy has been revised to reject replan transitions without clearance and low-speed handoff. A safe path through A* grid cells is not a guarantee of dynamically safe tracking or a successful real flight.
- Tests/test_lidar_imu3d.py covers vertical ray geometry, 3D correction, unobservable scans and inertial integration.
- Experiments/imu3d_replanning_audit.py and the corresponding native GitHub workflow preserve results separately for successive versions, and never credit radio packet delivery on failed physical flights.

## Negative results must not disappear

- V1 (results/imu3d_replanning): direct brownian accelerometer bias caused extreme pose divergence, physical safety 0/4. This is a modeling/controller defect, NOT a realistic physical IMU error observation.
- V2 (results/imu3d_replanning_v2): fixed low-magnitude accelerometer bias greatly reduced divergence but 3D range-only registration was often unobservable in open corridors/roof-level flight; safety still 0/4.
- V3 (results/imu3d_replanning_v3): barometer, noisy optical-flow, 3D range and planar fallback reduced mean 3D ATE to roughly 0.133m; however 3/4 hybrid flights failed full safety despite completing. The same 30-second LiDAR-outage control baseline (2D known-map) passed 4/4.
- V4 (results/imu3d_replanning_v4): checking the spliced first grid waypoint reduced neither the localized tracking issues nor the contact risk sufficiently: hybrid with replanning still passed only 1/4. Actual clearance and native contacts, not route pictures, determine success.
- V5 is a paired control ablation that compares 2D known-map baseline, 3D hybrid WITHOUT replanning, and 3D hybrid WITH stricter clearance/speed-gated replanning. Its safety result must be inspected before approving any new controller.

## V5 verified controlled ablation — stop/go result

The corrected, predeclared hybrid comparison completed 12 native MuJoCo cases under the same two procedurally unseen city seeds (91311, 20483), heavy aircraft, both route types, and a 30-second LiDAR blackout (t=45–75 s). The corresponding GitHub Actions workflow **passed** and published results/imu3d_replanning_v5/summary.json and trials.csv.

| Policy | Physical safety (4 flights) | Mean ATE (m) | Accepted replan count |
|---|---:|---:|---:|
| 2D map-aided scan matching, no replan | 4/4 | 0.1271 | 0 |
| 3D LiDAR/IMU + noisy barometer/flow, no replan | 1/4 | 0.1880 | 0 |
| Same hybrid with clearance/speed-gated A* replan | 1/4 | 0.1708 | 7 |

These are development scenarios after inspecting the failed V1–V4 results, not independent confirmatory data. Some missions reached the destination but had building contacts or insufficient clearance, and **do not count** as safe. Mean trajectory ATE is not a proxy for rare near-wall errors. In particular, the joint estimator and proposed replan policy do NOT meet the research safety acceptance criterion.

**Decision:** retain the 2D map-aided reference mode; reject promotion of the current 3D LiDAR/IMU hybrid and dynamic replan controller. The next research design must predict vehicle tracking envelopes from actual dynamics (including braking, momentum, wind, rotor lag and sensor-tail uncertainty), validate every spliced segment, and have safe-hold/relocalization protocols when observations become insufficient. Freeze controller gains before running NEW cities and adversarial scene distributions. No real flight or calibrated sensor claim is justified.

## Scientific limitations and next gates

1. The system has not implemented unknown-map 3D SLAM, IMU bias estimation with a consistent covariance, graph optimization/loop closure or full sensor frame calibration.
2. Synthetic LiDAR and IMU samples are idealized from MuJoCo; real sensor timing, observability, extrinsics, IMU saturation and scanner multipath have not been validated.
3. Only two independent procedural seeds, one aircraft type and mild deterministic force were used for V1–V5. These development repetitions are NOT independent confirmatory validation; threshold changes were made in response to failed trials.
4. Even excellent average pose error is not a flight-safety guarantee: false corrections, path handoff, dynamic feasibility and tail-risk matter more than the mean.
5. The current replanner is a geometric shortest-safe planner and does not yet jointly optimize live LoRa alerts, SF changes or whole-mission energy.

## Physical validation is blocked, not simulated

See research/HARDWARE_VALIDATION_PROTOCOL.md and analysis/hardware_lora_validation.py. The latter refuses to emit a real-world calibration/holdout summary without genuine site-disjoint packet logs. Do not fabricate real RSSI/PDR and do not certify GPS-denied flight using synthetic LiDAR.
