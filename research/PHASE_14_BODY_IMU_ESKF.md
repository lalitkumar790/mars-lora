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
