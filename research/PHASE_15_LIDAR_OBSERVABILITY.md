# Phase 15 — Local LiDAR observability and measurement-age diagnostic

**Research status:** Passive diagnosis only. No flight-controller change, no uncertainty-gate tuning, no new heldout claims.

## Motivation

Phase 14b documented 15-state ESKF position errors of 1m by t=41.22s and 5m by t=45.98s around a 45–75s LiDAR outage on the known failed development-city RF-aware mission (seed 48221). The failed run had only 8 accepted 3D position updates versus 97 rejected and 4 accepted planar updates versus 92 rejected; precontact attitude RMSE ~1.543°. We must measure whether those rejected/accepted scans actually constrained XYZ independently, rather than assuming every range hit provides usable localization.

## Implementation

`mars_lora/lidar_observability.py` evaluates an estimated-pose, finite-difference Jacobian of 48 synthetic ray ranges with respect to world XYZ. It excludes missing-range returns and rays that cross discontinuous geometry under +/-8 cm perturbations, then constructs a local approximate Fisher information matrix `J.T @ J / sigma_range^2`. It returns:
- number of valid observed and usable matched rays;
- approximate translational rank (0–3), smallest/largest information eigenvalues and condition number;
- range residual RMS between observed and map-predicted returns;
- a *diagnostic only* designation of locally informative geometry.

Native `eskf15_shadow` tracks local scan geometry/rank/timing, last informative measurement age and first large range residual, without feeding observability decisions back into estimator updates or UAV control. Phase 15 saves a per-scan JSON timeline for previously characterized safe (91311/shortest) and failed (48221/RF-aware) development flights.

## Caveats

- A rank-3 translational Jacobian **does not establish global uniqueness, correct data association, heading observability, global SLAM consistency or collision avoidance**.
- Finite differencing box-ray hits has discontinuities. The Jacobian is an approximate local diagnostic, not a reliable filter correction Jacobian or covariance estimator.
- `yaw` comes from ESKF estimated quaternion, not true simulator orientation, so model residual can reflect angle error in addition to positional error.
- Information thresholds are tentative diagnostics; not optimized against the previous failed run. No new scan is accepted/rejected because of this feature.
- 3D LiDAR still raycasts against a *perfect prior map* with synthetic noise. This is not unknown-map SLAM or hardware LiDAR validation.
- The right next step after verifying the metrics is to log scan quality versus *time since last trustworthy correction* and estimator divergence, then design feature-aware, properly gated LiDAR observation models on **separate development seeds** before another frozen holdout.

## Acceptance criteria

1. Featureless scene yields zero information and does not pretend to be a position fix.
2. Multi-obstacle geometry yields finite, interpretable residual/rank metrics; corrupted readings yield measurably worse residual.
3. Native experiment executes and preserves original controller contact/completion outcomes.
4. Any ranking or residual claims are tied to saved per-scan outputs, not inferred from aggregate flight error alone.

## Executed native development comparison

[GitHub Actions run 44913719354](https://github.com/lalitkumar790/mars-lora/actions/runs/44913719354) completed successfully with 3D local geometry tests, native MuJoCo replay and saved per-scan timelines. The first attempt had only a NumPy boolean JSON-serialization error; corrected without changing the estimator or controller.

| Local scan information before first contact (full safe flight for reference) | Seed 91311 shortest SAFE | Seed 48221 RF-aware FAILED |
|---|---:|---:|
| Non-blackout 3D scans observed | 47 | 29 |
| Rank-3 / informative according to fixed diagnostic | **19** | **3** |
| Rank-2 (missing one local translation direction) | 28 | 18 |
| Rank-0 | 0 | 7 |
| First noninformative scan (s) | 9.02 | 3.00 |
| 15-state position first >1m | never | 41.22s |
| 15-state position first >5m | never | 45.98s |
| First MuJoCo contact | none | 88.736s |

**Interpretation:** a strong **correlation** exists between sparse local 3D range geometry and the known bad estimator flight, and it is demonstrable *before the crash*. Of 29 observed precontact scans, only 3 were locally informative under the fixed XYZ criterion. The original 15-state filter was already 1m wrong before the 45–75s LiDAR outage; insufficient geometry can make it impossible to correct accumulated inertial position drift.

**Limitations on attribution:** the safe flight is completed at ~125s while the unsafe flight's precontact window ends ~88.7s, so their observation windows are not equal. The scan information is calculated at the potentially incorrect *estimated pose*, and low local rank is not necessarily a bad sensor: geometry can be rank 2 while other sensors provide the third direction. Also, the diagnostic includes rank and residual but does not yet model attitude/yaw in the scan Jacobian, so true full 6-DoF observability cannot be asserted. The existing planner/controller were unchanged. Counts are **two development trajectories, not an independently sampled population**.

**Research decision:** instrumentation is accepted; controller/estimator promotion is **rejected**. A robust next study requires local 6-DoF scan Jacobians and sensor-fusion observability, matching windows, scan innovation/feature association diagnostics, and calibrated estimation consistency under unseen randomized maps. Do not retrofit a 3D LiDAR correction based solely on rank and call it safe navigation.
