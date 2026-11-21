# Phase 13 — first ESKF implementation and native shadow audit

**Date:** 21 November 2026. **Status:** research implementation complete; NOT a validated GPS-denied navigation or safety controller.

## What is actually implemented

`mars_lora/lidar_imu_eskf.py` implements a **9-error-state translational error-state Kalman filter** with position (3), velocity (3), accelerometer bias (3), a 9×9 covariance, IMU propagation, Joseph-form covariance updates, barometer/optical-flow observation fusion, 3D-position and 2D known-map scan fixes, and normalized-innovation-squared gating with declared chi-square gates.

**Why nine and not fifteen states?** The repository's existing synthetic IMU emits *world-frame gravity-compensated acceleration* made by finite-differencing MuJoCo world velocities, plus an isolated yaw-rate channel. It does **not** produce a physically accurate time-synchronized body-frame specific-force/3-axis angular-rate packet. Adding an attitude error state without fixing that sensor model would misrepresent the result. A full 15-state attitude/velocity/position/bias ESKF requires synthetic body-frame accelerometer+gyroscope simulation, quaternion attitude, gravity, bias random walk, extrinsics/time offsets, and corresponding Jacobians, and has NOT been built or tested here.

## Bench comparison: controlled injected outliers

Run [44911696614](https://github.com/lalitkumar790/mars-lora/actions/runs/44911696614). Paired 24 seeds, each 50 seconds, 15–30 second position-fix outage, 2 strong injected position-fix outliers per seed, barometer and horizontal optical flow, same random measurements across candidate and old complementary baseline.

| Metric | Existing complement/scan correction surrogate | New 9-state ESKF |
|---|---:|---:|
| Mean position RMSE (m) | 1.8433 | 0.1028 |
| Mean maximum error (m) | 6.4717 | 0.2034 |
| Outlier rejection | not comparably NIS-gated | 48/48 |
| Nominal 99% 3D ellipsoid empirical coverage | not defined | 98.08% |

**Comparison limitation:** the old estimator receives and applies synthetic position-fix outliers, while the new ESKF has an explicit NIS gate. These results demonstrate that rejecting adversarially bad position fixes can help **in this constructed data model**; the RMSE difference is NOT a fair isolated measure of EKF versus non-EKF state estimation, and observations are direct noisy positions, not actual LiDAR scan matching. Coverage is computed using ground truth for offline evaluation and has correlated time samples.

## Real native MuJoCo sensor streams, before planar fallback

Run [44911866095](https://github.com/lalitkumar790/mars-lora/actions/runs/44911866095). ESKF runs as an **observation-only shadow**, reusing exact baseline-world-frame synthetic IMU, barometer, flow and 3D LiDAR ranges; the original controller receives unchanged data and the same baseline mission outcome (explicit deterministic control run).

| Scenario | Legacy RMSE | 9-state shadow ESKF RMSE | Status |
|---|---:|---:|---|
| 91311 shortest (safe) | .0685m | .0544m | Safe |
| 48221 RF-aware (collision/incomplete) | 37.475m | 25.2029m | Unsafe |

The ESKF accepted **one 3D scan correction** for each flight and rejected many. Covariance coverage close to 100% on the failed flight can result simply from covariance inflation after collision and is not evidence of reliable safety monitoring.

## Phase 13b: reuse existing planar fallback

Run [44912014553](https://github.com/lalitkumar790/mars-lora/actions/runs/44912014553). When the baseline hybrid pipeline already generates a planar scan after uninformative 3D rays, shadow ESKF reuses that exact observation and applies another NIS gate. **Zero new random draws are used**. Again, simulator control traces and baseline outcomes are unchanged.

| Scenario | Legacy RMSE | ESKF shadow RMSE | 3D / planar accepted | Outcome |
|---|---:|---:|---:|---|
| 91311 shortest | .0685m | .0451m | 1 / 3 | Safe |
| 48221 RF-aware | 37.475m | **123.6023m** | 1 / 4 | Unsafe |

At failure, ESKF reached maximum position error 325.23m and only 37.92% empirical nominal-99%-ellipsoid coverage over the whole failed flight. **This was a regression, not a safety improvement.** It suggests that merely adding scan fixes under a non-calibrated motion/bias/scan model may worsen estimator consistency. Analysis must distinguish errors *before first contact* from divergence after collision; `phase13c` adds that instrumentation.

## Scientific conclusions and next engineering gate

1. NIS-gated fusion and bias states are functional in controlled synthetic streams; the initial unit tests and 24-seed CI pass.
2. Field-relevant reliability is unproven: in native MuJoCo with synthetic ray casting and model mismatch, only sparse 3D corrections are usable and one long RF-aware route remains catastrophic.
3. Do **not** promote the ESKF as the navigation source, publish 'calibrated 3-sigma safety', or claim 3D unknown-map SLAM.
4. Next implement physically meaningful **body-frame IMU** with gravity removal and gyroscope bias, validated attitude propagation and full **15-state ESKF**, all with timestamped/consistent 3D LiDAR scan constraints.
5. Instrument sensor observability, accepted-match geometry, NIS by sensor type, pre-contact NEES/coverage and bias estimates, not only whole-flight RMSE. Use separate development cities before a newly frozen holdout.
6. Only after reliable localization should the flight controller use it. A safe-hold/landing supervisor and LoRa-deadline/energy coupling remain later, independently validated research tasks.

All known failures remain counted, previous Phase 11 10/16 failed independent-city safety gate remains unchanged, and no Vercel/live controller promotion is authorized.
