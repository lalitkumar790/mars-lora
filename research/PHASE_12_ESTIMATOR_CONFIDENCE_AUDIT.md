# Phase 12c — first-contact localization and risk monitor audit

**Date:** 21 November 2026. **Status:** negative development findings, no validated onboard safety supervisor.

## Paired native flight findings

- Phase 12a development: [run 44909014185](https://github.com/lalitkumar790/mars-lora/actions/runs/44909014185), original dynamic waypoint candidate 0/4 safe with **four no-plan failures** (0 physically executed). Staged 10 m + .65 m/s altitude slew reference 3/4 safe, all four physically executed.
- Phase 12b quintic reference: [run 44909960934](https://github.com/lalitkumar790/mars-lora/actions/runs/44909960934), four physically executed development routes, **3/4 safe**; no demonstrated improvement over 3/4 staged control. The RF-aware route in seed 48221 had 6,693 contact samples, did not finish by 240s and reached minimum city clearance -0.064m.
- Earlier one-off quintic smoke [run 44909014141](https://github.com/lalitkumar790/mars-lora/actions/runs/44909014141) was 1/1 safe. It was only a development smoke and must not override the four-route failure.

## First-contact time-synchronization

Paired replay of the failed 48221 RF-aware route [run 44910124775](https://github.com/lalitkumar790/mars-lora/actions/runs/44910124775), with exactly the same native vehicle and seeded noise:

| State at first contact | Staged controller | Quintic tracking |
|---|---:|---:|
| Time (s) | 142.264 | 88.736 |
| True vs estimated XYZ error (m) | 21.435 | 4.113 |
| Scalar sigma proxy (m) | 0.693 | 0.604 |
| Precontact navigation RMSE (m) | 7.129 | 1.650 |
| Early 3-sigma corridor alarm lead (s) | +7.264 | **-9.384** |

The sign of lead is contact time minus first alarm time: negative means an alarm *after* impact. For quintic, the estimated Y differs from true Y by ~4.1m at first collision near Z=21m, even though the sigma proxy is ~0.60m. **This is empirical evidence of large estimation errors under these synthetic conditions, not a calibrated consistency test**; sigma is not a statistically consistent EKF covariance.

## Threshold alarm: high sensitivity, unacceptable specificity

Follow-up [run 44910455687](https://github.com/lalitkumar790/mars-lora/actions/runs/44910455687): experimental `sigma >= 0.50m` warning fired **117.264 seconds before** first staged contact and **19.716 seconds before** first quintic contact, both on the same development seed used to choose the threshold.

The same workflow independently replayed three *known-safe development flights* with the identical 0.50m threshold: seeds/routes 48221 shortest, 63047 shortest, 63047 RF-aware. All three completed safely with no contact but **all three raised confidence alarms**, at first times 25s, 27s and 27s. Therefore event-level false-warning rate was **3/3 safe missions** in this post-hoc small sample. The threshold is unusable as an autonomous abort policy without a credible recovery/mission-performance analysis. A geometrically gated 3-sigma stopping monitor also issued alarms on two safe missions, while missing the quintic collision until 9.384s after impact.

These are not representative population sensitivity/specificity estimates. None of these alarm-only branches changed the actual controller's commands; the flights cannot be credited as a mitigation success.

## Research decision / next experiment

1. **Reject** simple confidence threshold and geometric stopping-alarm as a safety controller. They were tested only observationally.
2. Instrument sensor innovations/accepted range matches, last valid correction time, LiDAR observability/feature density and independent barometer/flow residuals; estimate uncertainty with a calibrated filter rather than multiplying a scalar uncertainty proxy by three.
3. Separate state-estimation consistency (e.g., coverage of estimated error envelopes and NEES for a true covariance) from track control, and measure early-warning precision/recall with confidence intervals across *new* development and frozen holdout cities.
4. Only design a fail-safe maneuver after empirically validating that the craft can stop/hover/land from uncertain state with bounded thrust and static/dynamic obstacle geometry.
5. For the route planner, inspect corrected timing and exact-endpoint connector rejection reasons; do not tune against the old Phase 11 holdout seeds or reclassify planning failures as safe.

**Publication scope:** results support an honest methodological failure study on estimation uncertainty and risk alarms, not a claim of robust GPS-denied or collision-free autonomy. Previous Phase 11 10/16 heldout failure remains the most recent safety gate. No hardware LoRa, 3D unknown-map SLAM, true battery/propulsion energy or autonomous emergency recovery has been validated.
