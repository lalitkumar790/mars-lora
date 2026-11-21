# Phase 17 — Oracle observability-biased 3D waypoint pilot

**Research hypothesis:** An additional clearance-checked waypoint might move the aircraft through geometry that provides more independently informative LiDAR ranges. This is a **development-only offline map-oracle proposal**, not operational active SLAM, not a trajectory safety proof and not a LoRa/deadline-aware joint planner.

## Existing observations motivating this phase

Phase 16 [run 44916447048](https://github.com/lalitkumar790/mars-lora/actions/runs/44916447048) found on two selected MuJoCo flights:
- Safe (seed 91311 shortest): 15 of 47 locally full-rank six-dimensional body-LiDAR scans.
- Failed (48221 RF-aware): 4 of 29 locally full-rank scans before first contact, 8 rank-zero.
- The six-dimensional body-fixed LiDAR was a **separate hypothetical sensor**, not the actual legacy yaw-only LiDAR. Its local information matrix assumes a known static map, nominal pose and numerical finite-difference Jacobian.

## What this phase changes

`mars_lora/active_viewpoint.py` scores a finite number of candidates: insert one lateral viewpoint in an existing staged 3D route's longest horizontal segments (up to three). Each route candidate:
1. starts and finishes at exactly the same mission endpoints;
2. passes static collision clearance with vehicle radius, requested margin and 3-sigma nominal geometric inflation;
3. stays within an added-path-length bound;
4. admits a quintic stop-at-waypoint reference respecting defined translational speed, acceleration and jerk bounds and a maximum reference duration;
5. improves the fraction of nominal range scans locally rank six by a predeclared minimum.

If none passes, the algorithm returns the unchanged route with an explicit `baseline_no_verified_gain` outcome. It does **not** fabricate a path or misreport a rejected mission as successful.

**Computational and scientific limitations:** It uses exact known map boxes, a fixed level body-frame LiDAR with yaw roughly tangent to the route, and local geometric rank at a fixed sampling interval. It does not model 3D actual sensor scans during aircraft roll/pitch, misalignment, uncertain map, RF fading, message AoI, battery power or closed-loop rotor reachable sets. Treat the nominal reference duration as a geometric/kinematic constraint, not a guarantee the MuJoCo aircraft completes on time.

## Experimental design and hard decision

Development seeds `91311` (previously characterized) and `62731` (new procedural development), shortest and RF-aware baseline policies, heavy airframe, .25 N wind, 45–75s LiDAR blackout, identical synthetic sensor seeds. Each city/route receives unchanged staged+quintic baseline flight and a candidate route flight. Expected four missions **per policy**; any planning error remains in denominator. Diagnostics report plan selection, local rank fraction, route length, nominal duration, physical contacts, achieved clearance, completion, and position RMSE. The controller still uses the original yaw-only map-aided hybrid, **not the new body-fixed LiDAR**. Therefore a better viewpoint score is not evidence of an improved onboard estimation system by itself.

Run via `.github/workflows/phase17-viewpoint.yml`, with full traces in CI artifact `phase17-observability-route-dev`. A future *independent* test would require a newly frozen city split, external sensor validation, paired disturbance seeds, full-mission deadlines and energy accounting.

**Stop/go:** Until at least reliable native improvement on development plus independently heldout cities and physically validated recovery, DO NOT promote this algorithm to autonomous flight. Previous Phase 11 10/16 independent-city safety gate remains FAILED.
