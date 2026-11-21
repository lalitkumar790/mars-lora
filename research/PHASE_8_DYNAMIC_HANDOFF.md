# Phase 8 — dynamic handoff envelope (development hypothesis)

**Status:** Opt-in candidate, never a deployment-ready controller.

## Frozen baseline

V5 hybrid with replanning passed only 1/4 declared development flights under 30-second LiDAR blackout; the known-map planar LiDAR reference passed 4/4. In the subsequent additional-city holdout, the hybrid passed zero of two executed flights; another city failed preflight. These remain immutable negative results. New Phase 8 results must NOT be backfilled into V5 directories.

## Why the old handoff was insufficient

A static A* route is swept-clear geometrically, but a UAV may already be moving into an obstacle when it is handed a different path. The prior `0.45 m/s` horizontal-speed threshold alone is insufficient to bound displacement during actuator lag and braking. High mean pose accuracy does not constrain rare false registrations, which can be fatal near buildings.

## Proposed necessary veto

`check_handoff(estimated_xyz, estimated_velocity_xyz, points, solids)` computes a hypothesized stopping distance
`speed * delay + speed²/(2 * (braking_accel - wind_bound))`.

It samples the segment from estimated pose to predicted stop, and the path splice plus first commanded leg. Each sample must clear city solids by aircraft radius + 3× scalar localization uncertainty + a 0.25 m margin. Parameters are explicitly fixed in `mujoco_sim/validate_mission.py`: braking 0.8 m/s², wind acceleration 0.15 m/s², delay 0.2 s. These are conservative **hypotheses**, not experimentally fitted bounds. The current geometry test does not model all thrust transients or actual wind-direction reachability and is insufficient to prove safety. A rejection merely retains the old route: it does not establish a safe stopping/hover maneuver.

The opt-in guard runs in addition to the original A* route and low-speed splice checks; it never bypasses V5's checks or modifies the live browser.

## Pre-registered development comparison

Run the `phase8-envelope.yml` job, or locally:

```bash
python -m unittest discover -s tests -p 'test_replan_envelope.py' -v
python -m experiments.phase8_envelope --city-seeds 91311 20483 --output results/phase8_envelope_dev
```

For each of two development cities × two paths, compare the same frozen V5 control with envelope off/on using the same noise seeds, wind phase, 45–75 s LiDAR blackout and 240 s maximum simulated duration. Denominator is four expected flights per method. Record no-plan, incomplete, contact, under-clearance, replan attempts, rejection count, and ATE separately.

A controller with no accepted replans can seem 'safer' by refusing all transitions, but that is not success if it stalls, collides later, or fails to finish. Do not optimize thresholds on additional-city holdout results; choose any new development parameters before running fresh unseen seeds.

## Before a safe-autonomy claim

1. Run paired MuJoCo trials and review artifact-level per-flight telemetry, contacts and actual clearance.
2. Quantify braking and wind bounds from native dynamics for *both* airframes; rerun under altered rotor lag, wind direction, and nonideal yaw.
3. Add a measured/consistent estimator covariance and innovation gating, plus a safety fallback when both planned and retained routes are untrustworthy.
4. Freeze hyperparameters and evaluate genuinely new, non-selected procedural cities with complete preflight-failure accounting.
5. Compare field/bench sensor calibration and independent LiDAR-inertial package; real LoRa and aircraft energy validation remains entirely separate.

No unexecuted simulation may be represented as a success, and 3D map-aided localization must not be labeled full unknown-map SLAM.
