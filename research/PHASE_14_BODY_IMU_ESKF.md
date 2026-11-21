# Phase 14 — Body-frame IMU and full 15-state ESKF (development)

**Status:** Experimental estimator, not flight safety certification, not a deployed autopilot.

## Implemented in code
- `mars_lora/body_imu_surrogate.py`: timestamped body-frame specific force and full 3-axis body gyro measurements derived **inside a simulated sensor generator** from consecutive MuJoCo world velocities and world-from-body quaternions. Specific force is `R_WB.T @ (a_W - g_W)`, with independently seeded noise and fixed sensor bias. Quaternion sign is explicitly handled. Real accelerometer lever-arm effects, vibration, calibration offsets, timing latency and sensor saturation are not simulated.
- `mars_lora/body_imu_eskf15.py`: nominal state (world position, velocity, unit quaternion, accelerometer bias, gyroscope bias) with 15-dimensional right-invariant small-error state. Midpoint attitude inertial propagation and gravity compensation, first-order error Jacobian plus second-order transition approximation, continuous-noise discretization, Joseph-form measurement update and right-error attitude reset. Gated prior-map LiDAR XYZ/XY, synthetic barometer and ground-flow updates.
- Optional `eskf15_shadow=True` in the native MuJoCo test. Strictly observation-only. Existing legacy controller, synthetic hybrid IMU and planned route remain untouched; shadow IMU uses a separate seeded RNG. True simulator position/attitude/velocity enter the IMU *generator*, initialization and offline scoring but do not correct filter state directly.
- Tests verify stationary gravity, rotated specific force, quaternion gyro integration, sign invariance, NIS rejection, PSD covariance and uncertainty growth. `experiments/phase14_body_imu_shadow.py` pairs safe city 91311/shortest with failed 48221/RF-aware, including a no-shadow deterministic flight-control baseline.

## Research caveats
1. This is a **physically structured synthetic sensor** and 15-state ESKF, not a hardware-calibrated IMU or tightly-coupled LiDAR-inertial odometry system. 3D ray matching still relies on a preknown exact geometric map and simplified scan timing. Unknown-map SLAM and loop closure remain unimplemented.
2. The ESKF uses first-order Jacobians/noise and approximate midpoint integration, not a formally consistency-proven invariant filter. Calibration requires NEES/NIS across independent seeded motion plus measurements (and pre-contact analysis).
3. Gyro biases and global yaw may be unobservable in feature-poor city scenes without informative independent heading cues. A small estimated covariance cannot establish an actual physical position error bound.
4. The body IMU is generated at discrete physics samples; in a real UAV one must model sensor extrinsics, asynchronous measurement timestamps, delay, bias instability, gravity, yaw alignment, thermal drift and motor vibration.
5. **No live UAV control promotion.** A safe recovery, fail-safe landing and uncertainty-constrained replanner must be physically demonstrated before autonomous flight. Prior Phase 11 new-city safety gate failed (10/16).

## Acceptance criteria
- Unit tests pass.
- No-shadow and shadow baseline complete with exact same completion, contacts, clearance, elapsed time and legacy RMSE.
- Report full and **pre-contact** position RMSE, maximum error, calibrated coverage and accepted 3D/2D map corrections on a safe and a known-unsafe native flight.
- Preserve every failure. Do not claim a win unless independently confirmed across new cities, alternate airframe, wind/heading/outages and a real sensor stream.

## Executed native MuJoCo results (21 November 2026)

CI [Phase 14 run 44912646807](https://github.com/lalitkumar790/mars-lora/actions/runs/44912646807) **passed the unit tests and completed the physical-simulation experiment**. A no-shadow control showed the legacy flight was unchanged when the new estimator observed sensors.

| Scenario | Legacy position RMSE | 15-state shadow RMSE | Outcome |
|---|---:|---:|---|
| Development seed 91311, shortest | 0.0685 m | **0.0451 m** | Safe (0 contacts, 1.211 m minimum clearance) |
| Development seed 48221, RF-aware | 37.475 m | **97.8731 m** | Unsafe (6,693 contact samples, incomplete) |

At the **first** contact in the failing route, old legacy error was 4.113m, 15-state error **70.960m**, while the new ESKF's reported RMS positional sigma was **445.974m**; 15-state precontact RMSE was 50.5614m, versus 1.6501m old. The new ESKF accepted 8/105 3D matches and 4/96 planar matches in this failed flight. Coverage values from massively inflated covariance must never be presented as estimator accuracy or actual safety. A 15-state filter alone demonstrably **did not solve localization drift**.

CI [Phase 14b divergence run 44912849636](https://github.com/lalitkumar790/mars-lora/actions/runs/44912849636) further traced this **same** development failure without new tuning:

- First position error >=1m at **41.220s** (sigma RMS 0.307m);
- First position error >=5m at **45.980s** (sigma RMS 3.726m);
- LiDAR blackout is scheduled **45–75s**, so material drift began before and around the outage.
- Precontact attitude RMSE: **1.543 degrees**; first-contact attitude error: **2.277 degrees**. Even a small tilt error may project nontrivial gravity into horizontal acceleration, but that mechanism is a **hypothesis**, not proven from these aggregate logs.
- At first contact accelerometer bias estimate was (-0.00088, +0.00053, +0.00440) m/s² and gyro bias estimate (-0.001851, -0.001532, -0.000318) rad/s; do not equate estimates with actual injected bias without a paired sensor-calibration audit.

**Research go/no-go:** REJECT promotion of body-frame ESKF to autonomous flight. Prior Phase 11 independently split city matrix remains 10/16 safe and FAILED; Phase 14 had only two already observed development routes. Do not claim consistent covariance, real hardware sensors, unknown-map LiDAR SLAM, or improved vehicle safety.

The next isolated engineering experiment should measure successful LiDAR observation age, scan residual/Jacobian rank and geometric feature diversity versus attitude/velocity error, with independently seeded IMU bias and wind. Only then design a feature-aware LiDAR correction or additional attitude/velocity observation. Freeze model/noise/gates on development scenarios before any new unseen-city holdout.
