# Journal paper readiness — evidence and claim boundaries (21 Nov 2026)

## Most defensible existing contribution
**Research topic:** Deadline-, AoI-, and whole-mission energy-aware *evaluation* of raw LoRa over physically executed UAV missions under simplified synthetic urban RF, including counterexamples to radio-aware detours.

Suggested working title:
> When RF-Aware UAV Detours Hurt: A Reproducible MuJoCo–LoRa Study of Alert Deadlines, Propulsion Energy Proxies and Navigation Safety

**Do not claim a new globally superior joint optimizer, physically calibrated LoRa link, unknown-map 3D SLAM, or certified collision-free control.**

### Verifiable evidence already in the repository
- 720 initial seeded channel-policy trials (kinematic motion), followed by 1,440 factorial trials with SF adaptation, message priority and mission slowdown separated; available via `experiments/run.py`, `experiments/ablation.py`, `results/ablation` and `paper/paper.tex`.
- 480 MuJoCo-position-conditioned raw-LoRa replays on actual native dynamic quadrotor trajectories (two airframes, two receiver heights, two shadow conditions, 20 radio seeds, three policies). This is a *synthetic radio* experiment, not measured link behavior.
- 3D map-known A* geometric-shortest versus RF-aware route evaluation with native MuJoCo executed flight traces, paired radio replays and explicit reachability/safety failure handling. Results need exact per-run reference, confidence intervals and complete denominators in the final paper.
- Frozen propagation-profile robustness counterexample: under severe interference, a selected RF-aware detour may improve delivery fraction by ~3.12 percentage points while worsening timely critical alerts by ~13.75 points (see `results/heldout_validation/paired_robustness.csv`). Avoid extrapolating outside the simulated model.
- Illustrative *propulsion-power* approximation reverses optimistic transmitter-only accounting: one extra 13.28-second detour adds ~1,854 J ideal-hover induced-power proxy while saving only ~4.93 J modeled LoRa-TX energy. Depends heavily on assumed rotor size, efficiency, energy accounting, air density and mission policy; NOT measured battery energy. Source `results/flight_energy_proxy`.
- 2D prior-map LiDAR matching at higher injected drift: 8/8 physically safe versus dead-reckoning 3/8 at .06 m/sqrt(s) in one matched suite; mean ATE ~.122m versus ~1.039m. The estimator uses synthetic rays and exact map, and is not simultaneous localization **and** mapping. Source `results/lidar_map_matching`.
- V5: 2D map-aided baseline 4/4 safe, 3D hybrid with/without replanning 1/4; later independent city led to planning rejects and zero hybrid success in executed subset. No 3D flight-safety superiority demonstrated.
- Phase 8 added stopping veto 1/4, zero extra rejections; Phase 9 staged 10m climb cleared static buildings but caused ground contacts 4/4; Phase 10 added .65m/s commanded-altitude slew and reached 4/4 on development flights; Phase 11 *new-city* holdout failed with 10 safe / 16 expected. These findings constitute a reproducible failure investigation, not deployment readiness. `research/PHASE_11_HELDOUT_FINDINGS.md`.
- Phase 12 maps exact endpoint departure/arrival corridors, creates a bounded-speed waypoint proposal, and adds analytic quintic point-mass reference samples. These alone do not prove closed-loop rotor dynamics or uncertainty-covariance consistency; new experiments are development only.

## Recommended journal paper scope and novelty test
A *simulation methodology / empirical trade-off paper* focused on **LoRa critical-message timeliness vs movement-induced outage / mission duration / approximate propulsion energy**, with controlled negative results, is the most mature. The distinctive angle is paired native-physics route execution + complete failure accounting + uncertainty/safety stress tests, **not** claiming that LoRa, SF optimization, A*, AoI or spline trajectory generation were invented here.

Before submission:
1. State precisely the hypothesis and methods; compare against state-of-the-art LoRa/UAV AoI and energy scheduling, especially an existing 2025 IEEE article on Energy-AoI Trade-Off LoRa-Based AAV Data Collection (IEEE Xplore 11073149), plus modern perception-aware planning and robust collision avoidance.
2. Include fair flight/time-spatial common-random-number baselines, paired effect sizes, bootstrap intervals, enough cities and wind/receiver/scattering seeds, and heldout evaluation; **never drop no-plan/unsafe cases**.
3. Include radio receiver SF-switch handshake/duty-cycle constraints; validate path loss, PDR and alert delay with real hardware LoRa packet traces or make the simulation-only limitation unmissable.
4. Replace ideal-hover proxy with rotor thrust/velocity energy integral or battery measurements; report full-mission energy against TX-only accounting.
5. Separate known-map localization from unknown-map SLAM. Do not combine Phase 12 partial reference planning with a claim of real GPS-denied urban autonomy.
6. Ensure the accepted NIT Goa conference review abstract and a changed experimental paper scope do not conflict with conference submission/reuse rules. Journal extension requires substantial new material and disclosure of any conference overlap.
7. Release environment versions, fixed seeds, complete planned-case denominators, workflow/trace hashes, and generate all figures from saved verified results.

**Publication judgment:** Existing work can support a credible *simulation-based trade-off/negative findings article* after statistical strengthening, literature positioning and methodological review. It cannot currently support a strong journal claim of a robust, jointly optimized GPS-denied autonomous LoRa UAV. Acceptance depends on journal fit, novelty and reviewer evaluation. No real hardware claims.
