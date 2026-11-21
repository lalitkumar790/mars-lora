# Phase 16 — Local six-DoF body-fixed range observability

**Status:** exploratory synthetic-sensor analysis. This is NOT an improved navigation estimator, flight controller, unknown-map SLAM or operational safety method.

## Experimental motivation

Phase 15 [native test 44913719354](https://github.com/lalitkumar790/mars-lora/actions/runs/44913719354) found 19 locally informative XYZ scans among 47 evaluated on a safe mission, versus only 3 informative among 29 evaluated **before first collision** on a failed RF-aware mission. Its map-known sensor rotates only around yaw. A position-only rank-3 calculation does not test whether heading, roll or pitch is identifiable.

## What's new and what it is not

`mars_lora/lidar_pose6_observability.py` defines a distinct **hypothetical body-fixed 48-ray LiDAR**, with full quaternion rotation of rays. It computes scan range Jacobians by central finite differences in XYZ and right-multiplicative body roll/pitch/yaw using a known geometry map. The angular state columns are scaled by a specified 2m lever arm so rank/conditioning calculations have a reproducible unit convention. The module reports valid and consistently predicted returns, residual RMSE, rank of full 6D information, and the separate XYZ/rotation principal-block ranks.

**Important:** the pre-existing LiDAR emits yaw-only rays. Those observations MUST NOT be passed to the six-dimensional body-fixed model because they represent different measurement functions. The new sensor receives its own independent seed and synthesizes range returns from simulated physical pose. Simulator truth is used only within this sensor generator and for offline evaluation; scan analysis itself uses the **estimated** ESKF pose. The six-dimensional observability tool never updates the ESKF or flight commands.

`experiments/phase16_pose6_dev.py` replays the previously used safe and unsafe development missions, measures rank **only before first contact** in the failed case, exports per-scan timelines, and asserts that a no-additional-sensor control flight gives identical legacy flight results.

## Interpretation limits / quality gates

- Rank 6 is a local linearized geometric property. It is not global pose uniqueness, guaranteed convergence, 3D SLAM, or reliable feature correspondence.
- Rotation and translation columns have different physical units; finite-difference step, angular_scale_m and rank threshold are experimental hyperparameters, fixed before viewing these paired results.
- A high residual or rank-deficient geometry can indicate bad alignment or few features; a full-rank Jacobian does NOT guarantee correct correspondence at a distant prior.
- The current sensor uses 48 rays against axis-aligned truth boxes, with simple fixed noise/dropout, no rolling acquisition, inter-frame motion compensation, true extrinsic calibration, or asynchronous delivery.
- The two development scenarios have **different precontact durations** and should not be treated as independent generalization or statistical population estimates.
- Existing 15-state ESKF is demonstrably unsafe as a control source in the failed city (70.96m first-contact error); nothing in this phase changes that result. Previous independent-city Phase 11 safety gate remains FAILED at 10/16.

## Research decision

Do not promote estimation or flight controller. Next, identify per-direction weak information subspaces and compare their eigenvectors with actual position/attitude errors on a *separate* development scenario set. Only then consider an observability-aware scan correction model that passes estimator-consistency trials and physically safe flight. Any eventual holdout must use fresh city seeds and frozen decision rules.

## Verified native development outcomes

[GitHub Actions run 44916447048](https://github.com/lalitkumar790/mars-lora/actions/runs/44916447048) completed mathematical tests and all matched native MuJoCo trials. No-shadow and observer-enabled runs of safe seed 91311 reproduced identical completion, contact count, minimum clearance, elapsed time, legacy position RMSE and 15-state precontact RMSE.

| Metric (pre-first-contact; whole flight for safe case) | Safe 91311/shortest | Failed 48221/RF-aware |
|---|---:|---:|
| Synthetic body-fixed scans evaluated | 47 | 29 |
| Local full 6D rank | **15/47** | **4/29** |
| Local rank-5 (one weak direction) | 32 | 17 |
| Rank-0 | 0 | 8 |
| Translation principal-subspace rank 3 | **15/47** | **5/29** |
| Rotation principal-subspace rank 3 | 47/47 | 21/29 |
| Flight completion | yes | no |
| MuJoCo contact samples | 0 | 6,693 |

**Interpretation:** locally, the body-fixed synthetic LiDAR constrains orientation more frequently than position in these specific selected city trajectories; a 15-state inertial filter cannot recover position reliably without adequate translation information. In the failed route, 8/29 pre-impact scans offered effectively no local information under this finite-difference criterion. This is consistent with Phase 15's position-only result. It is **correlation, not proven causation** of the collision, and it is based on a hypothetical additional sensor stream, not the original yaw-only scan.

**Crucial units/rank caveat:** right-multiplicative orientation columns are scaled with a fixed 2m characteristic length. The chosen rank ratio, range dropouts, small perturbations and map geometry influence these counts. The translation and rotation **principal-block** ranks can each be 3 while full 6D rank is 5, reflecting cross-correlation. The current study does not quantify the eigenvector of the missing direction or statistically validate information thresholds.

**Research decision:** successful diagnostic implementation, but **no go for onboard estimator or planner control**. The next experiment should calculate the weakest information eigenvector per scan and compare it with the direction of pre-impact position error, using new development cities and equal-duration windows; then test an active-perception waypoint policy constrained by actual rotor stopping dynamics. Any paper should identify this as a simulation-only geometric observability analysis.
