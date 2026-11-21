# MARS-LoRa: Research audit and acceptance gates — 21 November 2026

## Intended research claim

A collision-safe mobile robot (UAV in the current prototype) can adapt its **flight route**, **LoRa spreading factor/scheduling**, and **communication-risk response** using uncertain urban localization to improve the **timeliness of critical messages** compared with shortest-safe routing and fixed/reactive communication policies—without disproportionate mission-time or energy overhead.

**Important scope distinction:** The conference's accepted abstract was titled *"LoRa-Based Communication for Mobile Robotics and Autonomous Systems: A Critical Review of Reliability, Mobility, and Deployment Challenges."* The current source paper is a simulation-based experimental study, not just a critical review. Verify with the venue whether the substantial change in paper scope/title is acceptable before calling this the accepted full paper.

## Evidence-based status

| Gate | Current evidence | Status | Requirement to pass |
|---|---|---|---|
| Physically executed UAV routes | Two MuJoCo airframes, actual 3D routes, saved CSV, collision checks | Baseline complete | Repeat under seeded perturbations, report failure rates; no real-world calibration yet |
| Multi-building missions | Browser A* two inspection stops and return to base | Demo only | Native physics trials for full itineraries with inspection dwell and RF packet deadlines |
| RF-aware path planning | 54 m shortest versus 80 m radio-aware, both executed and replayed with packet events | Prototype complete | Joint decision that accounts for **critical-alert deadlines**, mission time, energy and uncertainty |
| Cross-route propagation fairness | Initial random fading tied to simulation time; new test uses a common seeded 3D spatial shadow field | Improved | Empirical/independent radio propagation validation and randomized spatial scenes |
| Model mismatch robustness | 400 trials, five assumed propagation profiles, two vehicles, 20 paired seeds | Exploratory complete | Test out-of-sample locations/scenes and real radio measurements |
| Safety under disturbance | Legacy 12/20 scenarios pass; higher-authority controller 14/20 pass under five force levels. Failures include stalling, collisions, clearance breaches | **Fails** | Both aircraft must pass a declared disturbance envelope with >0.25 m clearance; larger randomized battery after gains are frozen |
| GPS-denied navigation | 2D MuJoCo rangefinder mapping and dead-reckoned scan-matching; ROS 2 replay adapter | Incomplete | Measured ATE/RPE under multiple seeds; 3D localization/loop closure; same controller driven by estimate, not simulator truth |
| LoRa physics | Raw point-to-point LoRa airtime/SF sensitivity abstractions, synthetic NLOS and BER probability | Incomplete | Measure LoRa transceiver range/packet timings; realistic SF receiver coordination, collisions, duty-cycle and TX/RX energy |
| Statistics | Paired differences and conditional 95% intervals, carefully labeled | Partial | Independent environment replicates with spatial fading + flight perturbations + multiple-comparison handling |
| Manuscript | LaTeX/figures in repository, CI imposes <=8 pages including references | Draft | Update all claims with negative tests, references/methodology, actual calibration, venue approval |

## Four findings that change the research direction

1. **Radio-aware routing is conditional, not uniformly optimal.** In a 20-seed held-out *mild urban* profile (Skydio model), shortest-safe PDR is 90.88%, RF-aware PDR is 90.23% (difference -0.64 percentage points; 95% interval -2.64 to +1.36). The route detour costs 13.30 s. Under dense urban conditions the RF route improves delivery by 71.02 points but adds modeled radio energy. Under high interference it improves PDR by just 3.12 points while **worsening critical on-time rate by 13.75 points**. Source: `results/heldout_validation/paired_robustness.csv`; synthetic propagation stress, *not field inference*.
2. **Temporal shadowing was unfair for route comparisons.** A spatially fixed, seeded 3D interpolation field now allows different routes to experience the same shadow value at the same location. The model remains uncalibrated and its marginal variance changes with interpolation.
3. **Current flight controls fail disturbances.** `results/wind_robustness/summary.json`: baseline 12/20 pass and an experimental higher-authority variant 14/20 pass, conditional on 2 airframes × 2 routes × 5 deterministic force scales. At high disturbance the candidate occasionally completes with less than the 0.25 m safety margin. That is not a deployable control improvement.
4. **A preflight LoRa gate is an idea, not a validated algorithm.** `mars_lora/route_gate.py` uses only simulated observable pilot RSSI and a sampled channel busy fraction. `results/preflight_gate/` reports whether it chooses a short route or an RF detour. The thresholds were selected after inspecting the existing scenario results, so this evaluation is deliberately labeled *post-hoc* and must not be cited as independent validation.

## Work done in this research pass

- Added the frozen-route held-out propagation experiment `experiments/heldout_validation.py` and 400 matched route-trial results, 3D spatial fading, five alternate channel settings and paired per-scenario differences.
- Added `analysis/plot_heldout.py` and integrated the evidence and counterexamples into `paper/paper.tex`, with `paper/figures/heldout_robustness.pdf` compiled by CI.
- Added the native flight disturbance experiment `experiments/wind_robustness.py` and a nondefault high-authority controller option. Recorded every negative outcome.
- Added an experimental observable-RSSI + channel-busy gate `mars_lora/route_gate.py` and an explicit post-hoc policy audit under `results/preflight_gate`.
- Kept all research outputs in GitHub, never deployed benchmark plots/results to Vercel.

## Next tests, ordered by research value

**P0 — Safety as a hard constraint.** Plan with larger clearance under uncertainty, implement a dynamic collision-recovery or emergency return controller with a sensor-based safety envelope, and rerun the *same* 40 disturbance cases plus independently seeded wind directions. No policy may be considered successful if it strikes geometry, loses stabilization, or fails to return.

**P1 — A true joint, deadline-aware objective.** Rather than optimizing predicted SF9 packet reception, optimize estimated missed-critical-alert risk and P95 AoI while jointly selecting SF, queue priority, trajectory and mission duration. Compare against minimum-flight-time, shortest+adaptive radio, and an alert-aware detour gate with route/SF decisions made **before** outcomes are revealed.

**P2 — Evaluate across different cities and receivers.** Randomize building geometry *in the native MuJoCo planner and the independent evaluator together*; use spatially locked shadow fields. Current random browser city is not part of the paper's fixed native research matrix. Include receiver relocation and model-mismatch conditions.

**P3 — GPS-denied localization.** Verify native MuJoCo LiDAR and drift estimates independently against ground truth (ATE/RPE) on at least two cities, then plan in the map frame with safety margins proportional to estimated localization uncertainty. Include failure/relocalization logic.

**P4 — Calibrate radio and vehicles.** Obtain physical LoRa RSSI/PDR versus distance, SF timing and duty-cycle measurements in controlled outdoor/urban tests, and measured airframe mass/rotor time constants where possible. Compare with at least one independently configured simulator/model.

**P5 — Paper.** Treat observed negative results and safety failures as primary findings, remove redundant background material to meet the 8-page limit, add actual references and disclose the conference abstract/paper scope change.

## Research stage

We have a **physics-linked, falsifiable proof-of-concept** and an evidence-rich simulator. We do **not** have a validated GPS-denied, collision-robust, jointly optimized aerial LoRa system or a field-calibrated journal result yet. The next meaningful milestone is to pass the safety gate and demonstrate route/SF/alert coordination under new, unseen environmental conditions.

## Follow-up experiments: safety, route/SF coupling, and full-flight energy

### Actual native physics safety (not a CAD-only check)
The old native MuJoCo flight logger checked clearance against industrial obstacles but omitted static city buildings. It now computes city-wide signed collision clearance using a conservative broadphase (exact for all safety-critical distances), with randomized and obstacle-center regression checks. The new comparison keeps the same 20 deterministic wind-force cases per controller (2 airframes x 2 routes x 5 force settings). Safe means completion AND zero contacts AND minimum clearance of at least 0.25 m.

- Original controller: 12/20 safe missions.
- Higher-authority position control: 14/20.
- Bounded-integral disturbance compensation: 11/20.
- Conservative obstacle-distance acceleration barrier: 10/20 (commonly stalls).
- Closing-speed-aware acceleration barrier: 13/20 (still exceeds clearance limits and fails under high force).

All five failed this declared envelope. The latter two variants are separate, non-default experiments, not a field-certified collision avoidance system. Evidence: results/wind_robustness, results/barrier_guard, results/barrier_ttc.

### Joint route and spreading factor is no longer only predicted
The preflight selector uses only synthetic observed RSSI, busy-channel fractions, known static geometry and a flight-time proxy. Its choice of a route and fixed spreading factor is subsequently subjected to *real synthetic packet replay on physical MuJoCo traces*. There are 200 matched cases in each of three versions: initial selector; selector with a 20%-of-deadline LoRa airtime budget; and a third version that falls back to shorter-route adaptive LoRa when observed busy fraction reaches 0.20.

Heavy UAV, harsh-interference scenario (20 seeds): shortest/adaptive baseline PDR=0.551 and timely-critical=0.725; initial joint selector PDR=0.594, timely=0.638; airtime-budgeted selector PDR=0.546, timely=0.475; congestion-guarded selector PDR=0.556, timely=0.713. The guarded controller mitigates its own introduced error but DOES NOT beat baseline safety-alert timeliness. Evidence: results/joint_gate, results/joint_gate_v2, results/joint_gate_v3.

All parameters and thresholds were developed after studying the five scenario profiles. These are post-hoc development experiments and cannot establish out-of-distribution gains; radio sync is still optimistic.

### Flight-energy proxy reverses the radio-only narrative
Flight duration matters. The total quadrotor ideal-hover induced-power proxy follows (mg)^(3/2) / sqrt(2 rho A), with A = 4*pi*R^2. Assumed rotor radii are 0.11 m for the 1.325-kg model and 0.023 m for the 27-g model; assumed air density 1.225 kg/m^3 and efficiency 0.55. These are ILLUSTRATIVE, not OEM-verified rotor or battery data. For the 1.325-kg model, the 13.28-s radio-aware detour adds an estimated 1,854 J of hovering-power cost, while saving just 4.93 J of modeled LoRa TX energy under 4-dB shadowing. The overall energy advantage is reversed **under these assumptions**. Evidence: analysis/flight_energy_proxy.py and results/flight_energy_proxy. Background: A. M. Perez Gordillo et al., Energies 12(2):261, DOI 10.3390/en12020261.

### Localization-error allowance added, but not SLAM
The map-known A* planner can conservatively inflate obstacle clearance by three times a user-supplied localization standard deviation. This is only an uncertainty-budget primitive; it does not infer actual 3-D pose, perform SLAM covariance propagation or establish GPS-denied autonomy.

### Strict next gates
- Physics: independently seeded wind phases, at least two native city layouts and close-range obstacle cases; zero contacts, clearance >=0.25 m and 100% mission completion inside the declared design envelope.
- Communications: real LoRa radio calibration, reception-time SF sync, channel access, realistic queue and deadline tracking; measure energy from actual TX, RX, onboard processing and propulsion.
- Joint autonomy: tune an objective for critical-alert missed-deadline risk, full mission energy, localization uncertainty, and collision risk on training scenarios, then evaluate WITHOUT threshold tuning on genuinely unseen scenes.
- Academic: verify whether the experimental-paper title and scope remains within the accepted review abstract's conference permission; keep the paper including all figures and references <=8 pages.


## Phase 6: unseen-city and GPS-free localization evaluation (21 November 2026)

- Two initially unseen city seeds (94103, 58241): 8/8 safe native MuJoCo flights, 192 actual LoRa-replay trials; results/unseen_city.
- Two further replication seeds (15193, 77099): 8/8 safe native MuJoCo flights, another 192 LoRa trials; results/unseen_city_replication. All flights are conditional on mild deterministic disturbances, a perfectly known map, and an assumed 0.10 m localization sigma. The joint deadline-energy policy does NOT consistently outperform shortest-route adaptive LoRa.
- In results/localization_stress, drifting noisy XY estimates are fed into the physical controller. 32 runs give safety passes of 8/8, 8/8, 7/8 and 3/8 at Brownian drift 0, .01, .03, .06 m/sqrt(s). Mean truth-referenced ATE rises to 1.039 m at highest drift. The static planning covariance is an external assumption, not a learned state estimator.
- Map-aided two-dimensional scan matching is now supported by mars_lora/lidar_map_matching.py and the native MuJoCo controller. Sixteen horizontal simulated range beams (25 m cap) are matched to known city AABBs every 2 s, accepting only geometrically consistent correction candidates.
- Results/lidar_map_matching contains 32 matched flights. At drift .03, dead-reckoning ATE 0.369 m and safety 7/8 versus LiDAR ATE 0.099 m and safety 8/8. At drift .06, dead-reckoning ATE 1.039 m and safety 3/8 versus LiDAR ATE 0.122 m and safety 8/8.
- IMPORTANT: the LiDAR matcher is GPS-free MAP-AIDED LOCALIZATION, NOT simultaneous mapping or validated GPS-denied autonomy. It assumes exact prior buildings, ideal attitude and velocity signals, synthetic range returns and previously explored cities. High-altitude horizontal sensing may become unobservable. None of these 32 trials establishes real sensor robustness or field safety.

Next work: measured IMU/velocity uncertainty and 3D LiDAR/visual-inertial pose fusion; unknown-map exploration and loop closure; independent city distributions, wind, moving obstacles and real LoRa and propulsion energy calibration. Require a frozen joint deadline/energy policy with real-time failure detection before claiming robust GPS-less urban flight.
