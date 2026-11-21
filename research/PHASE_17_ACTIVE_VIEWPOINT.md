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

## Verified first native development run — a negative result

[GitHub Actions run 44938907387](https://github.com/lalitkumar790/mars-lora/actions/runs/44938907387), paired heavy MuJoCo and identical seeded noise, executed both policy variants where a valid staged route existed. Unit tests and the native runner completed; a successful workflow does **not** imply successful flight.

| Metric | Unchanged staged+quintic baseline | Oracle observability waypoint insertion |
|---|---:|---:|
| Expected city-route missions | 4 | 4 |
| Physically executed | 2 | 2 |
| Safe of all expected | **1/4** | **1/4** |
| Failed at preflight route generation | 2 | 2 |
| Physically executed but failed/contacts | 1 | 1 |
| Accepted viewpoint detours | n/a | **0** |

Seed 91311 shortest completed without contacts, 1.211m minimum clearance, 0.0685m legacy RMSE. Its nominal six-dimensional information fraction was 0.20 (both variants), and the algorithm kept the same route after checking 12 candidates.

Seed 91311 RF-aware had first physical contact at **t=83.600s** near (-8.439, 23.430, 16.997)m with **5.885m** estimated position error, 4,660 contact samples, and incomplete 240-second flight. Its route nominal information fraction was ~0.14286; 12 candidates evaluated, none accepted, so both variants were identical and unsafe.

For **new development city 62731**, both shortest and RF-aware map-known staged routes failed with `Unsafe start or goal`. Both policies retain those two planning failures in their four-case denominator. This must not be hidden as an unexecuted scenario in a safety-success fraction.

**Interpretation:** A single lateral waypoint insertion into the longest horizontal route legs, with an imposed 0.05 absolute rank-six gain and quintic feasibility/clearance limits, has no demonstrated improvement. The output is a correct *fail-closed* baseline fallback, not active perception. It is possible that no safe, beneficial short detour exists at all in those scenes, or that the search is too narrow. The current data do not separate these explanations.

**Next preflight experiment:** `experiments/phase17b_preflight.py` records per-constraint reject counts over known development seeds 91311/48221/63047 and new development 62731; no threshold relaxation or independent confirmation is claimed. Do not move to a new frozen holdout until a policy achieves feasible, independently measured improvements in *both* perception and safe flight on separate development cases.

**Hard stop:** The new candidate does not use the body-fixed scan in the online estimator and changes no flight controller. Real adaptive LiDAR trajectory planning, LoRa timeliness and whole-mission energy remain unproven. Prior Phase 11 independent-city safety gate FAILED 10/16.

## Phase 17b/17c development diagnosis and selected-positive case

Preflight [run 44939074725](https://github.com/lalitkumar790/mars-lora/actions/runs/44939074725) evaluated 8 **development** city-route combinations (91311, 48221, 63047, 62731 × shortest/RF-aware), with all route rejects logged. Exactly **one of eight** generated a viewpoint insertion passing the predeclared static clearance, added distance, reference feasibility and nominal rank-gain checks: seed **48221 shortest**, versus two preflight failures in city 62731. The one accepted candidate raised the nominal rank-six informative fraction from **0.153846 to 0.230769**, at **+1.4164m** geometric route length and **+2.6558s** reference duration.

The **post-selected development** matched native MuJoCo experiment [run 44939194532](https://github.com/lalitkumar790/mars-lora/actions/runs/44939194532) compared that exact 48221 shortest route using the original hybrid yaw-only estimator:

| Metric | Unchanged baseline | Added oracle viewpoint |
|---|---:|---:|
| Fully safe (completion/no contacts, clearance >= .25m) | yes | yes |
| MuJoCo contact samples | 0 | 0 |
| Minimum obstacle clearance (m) | .926 | .928 |
| Localization RMSE (m) | .1823 | .1785 |
| Elapsed flight (s) | 124.96 | 127.62 |
| Route length (m) | 66.0000 | 67.4164 |
| Nominal informative-scan fraction | .153846 | .230769 |

The nominal fraction increased by **7.69 percentage points** (one additional rank-six sample if sample denominators align), but the executed legacy position RMSE decreased by only **0.0038m**. With one *deliberately selected* known development pair this is **not evidence of statistical advantage, true scan-information gain, LoRa deadline success, or generalizable safety**. Both safe trajectories pass and the added flight duration is a real cost. The drone still receives the original yaw-only scan, not the hypothetical body-fixed scan used to rank route candidates. The reason for the 3.8mm RMSE difference is not established and could be numerical/trajectory-related rather than improved geometric sensing.

**Research decision:** Keep the prototype as a *possible hypothesis generator*, not an accepted autonomy algorithm. Next evaluate on additional non-selected development geometries under independently seeded winds/IMU errors; stop if the fraction of actual useful corrections and **precontact** position error do not improve meaningfully. No new frozen holdout before jointly checking mission safety, deadline, RF reliability and propulsion power under the same physically executed traces.
