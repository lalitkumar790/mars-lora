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
