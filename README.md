# MARS-LoRa: Mobility-Aware Reliability and Safety for LoRa UAV Links

> **Research status (21 November 2026):** Physically executed 3D A* flight and reproducible LoRa comparisons exist, but the system is **not yet a validated GPS-denied, robust or field-calibrated autonomous LoRa UAV**. A held-out propagation experiment uncovered cases where RF-aware detours have no benefit or worsen critical-alert timeliness. A native MuJoCo disturbance sweep failed 8/20 legacy scenarios; experimental higher-control-authority gains failed 6/20 scenarios. See the [evidence and acceptance-gate audit](research/RESEARCH_AUDIT_AND_GATES.md), [radio robustness results](results/heldout_validation/), [wind stress results](results/wind_robustness/), and the [exploratory preflight route-gate audit](results/preflight_gate/). These are simulation outputs, not field measurements.

Research prototype accompanying an **eight-page maximum** manuscript on LoRa for autonomous aerial robots.

## Scope and scientific status
The **runnable baseline** is a deterministic, event-driven **raw LoRa link abstraction** operating on configurable 3D inspection trajectories. It is **not** a hardware-validated PHY, an ns-3 LoRaWAN model, or proof of real-world UAV safety. Native MuJoCo trajectories are already coupled to reproducible raw-LoRa radio replay; ROS 2 closed-loop autonomy remains experimental. Never present synthetic/example plots as measurements.

Our testable question: does combining deadline-aware message prioritization, proactive SF adjustment, and mission slowdown improve timely telemetry/alert delivery over fixed-SF or reactive-SF policies?

## Quick start

Python >=3.10; standard library only for experiment generation.

```bash
python -m experiments.run --output results/raw --seeds 20
python -m unittest discover -s tests -v
python analysis/plot_results.py --input results/raw/summary.csv --output paper/figures
```

Matplotlib is required **only** for plot generation: `pip install matplotlib`.

The experiment runner generates `results/raw/trials.csv`, `results/raw/summary.csv`, and `results/raw/manifest.json`. The graph builder produces figure PDFs/PNGs for `paper/paper.tex`. Research plots and summary CSVs are never included in the Vercel site. Check in generated outputs only after verifying they were actually executed.

## Reproducibility boundaries
- The original 720-trial rectangular-route **baseline remains kinematic**. Later RF-aware route results use actual native MuJoCo flight traces. Neither is a field flight.
- Link model: log-distance path loss, correlated Gaussian shadowing, SF receiver thresholds, LoRa airtime, single-channel contention approximation, application queues, and deadline expiry.
- SF changes currently assume synchronized receiver settings (an optimistic upper bound). A practical design needs a control-channel/handshake and its cost.
- Radio power is a proxy for TX energy, excluding MCU, RX, and propulsive energy.
- No flight safety certification or measured radio validation.

## Project layout
- `mars_lora/channel.py` — event-driven link and policy implementation.
- `experiments/run.py` — reproducible experiment matrix and CSV export.
- `tests/` — invariants and regression tests.
- `analysis/plot_results.py` — measured-data-only plotting.
- `paper/paper.tex` — manuscript source, strict <=8 pages including bibliography.
- `public/` — Vercel-hosted interactive MuJoCo WebAssembly flight console, with a true four-rotor MJCF model, manual commands, waypoint mission, camera orbit and CSV trace export.
- `ros2_bridge/` — optional ROS 2 Pose subscriber and CSV logger.
- `mujoco_sim/` — integration instructions; not falsely represented as validated flight.

## Run the browser flight simulator
```bash
cd public
npm install
npm run dev
```
Open the URL printed by Vite (usually `http://localhost:5173`). The public app uses official MuJoCo WebAssembly (`@mujoco/mujoco`) for real, client-side rigid-body dynamics, with a Three.js 3D visualizer. Vercel serves the prebuilt browser app; the user's browser runs the simulation. It has takeoff/land, pause/reset, hover, WASD + Q/E movement, waypoint autopilot, wind disturbance and downloadable flight CSV. The LoRa link indicator is an **illustrative RSSI estimate**, not a ROS 2/LoRa co-simulation.

**Research results remain in GitHub only.** The paper and plots are built by CI; the deployed application does not expose summary CSVs, benchmark graphs, or manuscript figures.

The MuJoCo browser scene is a purpose-built compact testbed and is not yet the identical 900-m kinematic route used by the radio experiments. Do not interpret browser flight as verification of the publication's link model.

## Manuscript build
`cd paper && latexmk -pdf -interaction=nonstopmode paper.tex`. Verify `pdfinfo paper.pdf | grep Pages` is <=8. The manuscript explicitly marks unexecuted experiments as pending. Manuscript PDF is expected to be provided through CI artifacts once the build passes.

## License
Code: MIT (see LICENSE). MuJoCo Menagerie Crazyflie and Skydio visual CAD meshes are vendored in `public/assets/` with their separate upstream license files.

## Multi-airframe MuJoCo flight console (November 2026)
- Airframe selector: 1.325 kg Skydio-derived inspection quad or a **27 g Crazyflie 2-derived** dynamics model. Real MuJoCo free-body integration and four independent rotor forces are applied in either case, with distinct thrust limits, rotor arms, mass and moments of inertia.
- Crazyflie physical parameters sourced from [Google DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/bitcraze_crazyflie_2) (upstream MIT copyright preserved under `mujoco_sim/reference/`). The browser visualizer now loads the original OBJ CAD meshes; MuJoCo rigid-body inertias and motor control remain partly approximated.
- Shared industrial site: three buildings, container, transmission mast, pylon and overhead beam with MuJoCo contacts. 3D scene rendering is separate from but geometrically aligned with the physical static colliders. Extra distant visualization tanks are only scenery and have **no collision shape**.
- Browser telemetry includes airframe pose, speed, rotor thrust, waypoint status, and an **interactive approximate raw-LoRa link** showing RSSI, information freshness and packet delivery. The packet simulation depends on the drone's current physics state but remains a simplified, seedable stochastic model; SF synchronization, collision and regulatory constraints are idealized. No research figures or static trial results are deployed on Vercel.
- Headless Chromium CI tests physics initialization, takeoff, waypoint activation, pause/reset, airframe switching and live critical-alert injection.

## Controlled research ablation
Use `python -m experiments.ablation --seeds 20 --output results/ablation` for eight policy combinations with identical telemetry generation rates and common seeded time-indexed fading. Toggle adaptive SF (A), critical packet priority (P), and conservative slowdown (S) independently. GitHub Actions `.github/workflows/ablation.yml` publishes 1,440 synthetic trial logs and summaries to `results/ablation`; nothing from these files is hosted by Vercel. Manuscript includes descriptive table; no hardware validation or demonstrated safety claims.

## Flight stability, obstacle avoidance, diagnostics (Phase 3)

The previous 3.5–4 m mission path geometrically intersected the 5 m antenna tower. The revised safe corridor climbs to 7 m first (above the 5.68 m overhead beam), performs high-altitude horizontal inspection, and only descends after returning. `public/src/mission.js` includes a conservative swept-segment clearance check; `public/tests/mission-safety.mjs` is a regression that asserts the former route intersects a collider while the new route clears the environment by >=0.5 m. In-flight controller safety hold is triggered if geometric clearance falls below 0.25 m or MuJoCo reports contact.

New displayed flight debug channels: desired-versus-actual altitude line trace, tilt, motor thrust vector, velocity vector, live radio link, target pose and Cartesian tracking error, nearest collision geometry and clearance, MuJoCo contact count, industrial site minimap, LoRa RSSI/AoI trace, TX airtime and energy approximation, losses/expiry, priority alert deadlines, and radio model assumptions. Download flight telemetry CSV from the app for external analysis.

`mujoco_sim/validate_mission.py` simulates the entire 7-waypoint mission for both airframes using native MuJoCo with 4-ms integration, 50-Hz control, 60-ms motor lag, simple velocity-dependent aerodynamic drag, time-varying wind and obstacle contacts. GitHub CI reports completion, clearance, tilt, and contact steps. This is a deterministic demonstration, **not** certified real-world flight safety or model validation under random disturbances.

The live browser LoRa channel is a simplified raw-LoRa event engine: received power is modeled via nominal 14 dBm TX, 125 kHz bandwidth, path-loss exponent 4, 17 dB obstruction loss, 4 dB correlated shadowing and synthetic 10% contention. It accounts for packet completion time, SF-dependent airtime, queue expiry, critical deadlines, and transmission-energy proxy. All values are **simulation estimates**, not direct radio measurements. Research plots and results are saved to GitHub only, never to Vercel.

Next phase detail: [Phase 4 research plan](research/PHASE_4_PLAN.md). Goals: multi-seed physics stability sweeps; time-aligned MuJoCo pose replay in the research LoRa runner; measured/corroborated LoRa propagation and realistic SF handshaking; end-to-end safety-event experiment, paired confidence intervals and ablations.

## GPS-denied / SLAM demonstration (experimental)

The browser now includes a selectable **GPS-denied odometry + planar scan-matching** mode. Each airframe is instrumented with **32 native MuJoCo rangefinder sensors** around the horizontal body plane (18 m cap). These 2D LiDAR returns are fused with biased pseudo-odometry into a 100×100 occupancy grid (0.35 m cells). The controller uses the estimated XY pose in GPS-denied mode; it does **not** read perfect simulator XY for waypoint guidance. A **3.2 m landmark survey** is offered in addition to the safer 7 m high-altitude inspection mission because a level-mounted lidar at 7 m sees few obstacles. The on-screen GPS-denied panel shows map hits, correction count, pose error (evaluation only), and a live generated occupancy map. Use **Export LiDAR / odometry** to download JSONL suitable for future ROS 2 replay.

This is **not** a verified 3D SLAM solution, visual-inertial navigation, true IMU-based state estimation, or a certified GPS-less autopilot. The odometry is derived from noisy/bias-injected MuJoCo twist; altitude/attitude use a simulated IMU-like proxy; a ground-truth collision safety supervisor remains separate. Native MuJoCo rangefinders are real simulation measurements, not measurements from hardware. For limitations, evaluation metrics, and optional ROS 2 `slam_toolbox` integration, see [GPS-Denied SLAM research notes](research/GPS_DENIED_SLAM.md).

## Phase 4 — executed RF-aware 3D A* route planning

This repository now contains a genuine **map-known 3D route optimization experiment** rather than only using communication policies on one prescribed flight.

- `mars_lora/rf_astar.py`: independent geometric-shortest and predicted-LoRa-aware 3D A* planning over a 3 m horizontal grid and variable altitude layers, subject to explicit swept collision clearance.
- `mujoco_sim/validate_mission.py`: native MuJoCo simulator executes **each** planned route for the Skydio-based and Crazyflie-derived models; exports actual 10 Hz trajectories, safety clearance, tracking error, mission time and contact steps.
- `experiments/rf_navigation.py`: time-aligns physical flight with synthetic LoRa packet events; contrasts (S0) shortest-route SF9, (S1) same shortest-route adaptive SF + priority and (R2) RF-aware route with the **same** adaptive radio policy. Three methods × 2 aircraft × 2 shadow levels × 20 matched seeds = **240 radio trials**.
- `analysis/paired_rf_navigation.py`: output `results/rf_navigation/paired_effects.csv` contains matched-seed changes and descriptive 95% normal-approximation confidence intervals.
- `public/src/rf_route_planner.js`: live client-side A* controls for choosing the shortest versus RF-aware city mission in the MuJoCo browser lab. These are **map-known reference modes** and require simulator/reference navigation, not the still-experimental planar SLAM estimator.
- `paper/figures/rf_aware_routes.pdf` and `rf_aware_tradeoff.pdf`: built exclusively from actual simulated flight and packet results, never hosted as a static results dashboard on Vercel.

**Representative controlled result (Skydio-derived, 4 dB shadowing, 20 seeds):** the shortest-route adaptive policy has mean application PDR 0.432, alert on-time 0.500 and P95 AoI 28.56 s; the RF-aware route with identical adaptive radio settings yields mean PDR 0.784, alert on-time 0.838 and P95 AoI 4.33 s. Planned distance rises from 54 m to 80 m and flight time from 105.23 s to 118.51 s. The planner is trained only on deterministic obstruction geometry, but **planning and simulated evaluation use the same uncalibrated propagation model**, so these results cannot yet be generalized to hardware or unseen cities.

**Not yet accomplished:** truly simultaneous optimization of route + SF + schedules under localization uncertainty, reliable 3D GPS-denied SLAM, randomized-wind physics trials, independent RF simulator/hardware calibration, and a field-ready collision-avoidance controller. The radio update simulation remains outside the onboard stabilization loop.

## Randomized browser-city flights (repaired)
The **default browser mission** is now RF-aware 3D A*, not the old preplanned 7 m inspection demo. Each press of **Generate NEW city + start selected mission**:
1. Generates a new 32-bit seed and repositions/spaces urban buildings, leaving wide flight corridors, grass parks and plazas.
2. Rebuilds the matching MuJoCo collision scene, clears the previous flight trace/telemetry and resets the UAV to the takeoff position.
3. Chooses a different destination among valid open-airspace cells and solves a fresh 3D collision-checked route (geometric A* or expected-RF-weighted A*).
4. Commands initial climb to 4 m and only then follows the lateral route. The LoRa-aware option uses idealized expected RF quality, not future random fading.
5. Displays an explicit mission number, city seed, destination, planned length, static clearance and the live controller error.

Root causes addressed: the old fixed goal and seeded city, A* refusal under the experimental SLAM setting, false obstacle brakes triggered by ground contact or non-travel-direction LiDAR returns, and asynchronous drone CAD status updates. A* transparently switches to physics-truth **reference navigation**, because planar 2D SLAM has not been validated for this larger 3D urban mission.

**Scientific separation:** random browser scenarios are exploratory, not automatically included in the published comparisons. The paper's fixed-seed physics/LoRa datasets remain reproducible in GitHub. Parks, trees, windows and street decorations are visual-only; urban structural buildings have matching MuJoCo collision geometry.

## Building-to-building urban inspection missions

Two new options in the browser simulator:
- **Building-to-building inspection · shortest-safe**
- **Building-to-building inspection · LoRa-aware** (default)

Press **Generate NEW city + start selected mission**. The site planner chooses **two distinct, separated urban buildings**, locates collision-safe standoff observation points, computes **three 3D A* legs** (base → building A → building B → base), and commands the same MuJoCo quadrotor to fly the joined trajectory. At each inspection stop it holds position for three simulated seconds and updates the site-completion counter. Stops are shown as highlighted green rings in 3D and numbered dots on the top-down flight map. Every new start chooses a fresh city seed and different building sites; both aircraft use the same physically collidable urban building definitions as the map-known planner.

**Important:** These are overhead/adjacent building *inspection visits*, not rooftop landings or autonomous close-contact construction-site surveys. The current 3D planner uses a known static map and MuJoCo reference position; it does **not** yet supply a fully validated GPS-denied SLAM state estimator or moving-object replanner. Decorative parks and trees are visual assets and are not included in the collision/SF propagation models. Publication-grade results continue to come only from the fixed-seed reproducible research workflow, not these exploratory random flights.

## Ongoing research audit: deadline-aware cross-layer safety (21 November 2026)

The project now includes additional **negative-result** experiments: native full-city 3D collision clearance, five UAV controller variants (20 wind scenarios each), three ex-ante route/spreading-factor decision policies (200 actual LoRa-replay trials each), and an illustrative induced-hover propulsion-energy accounting. Results and precise limitations are in [Research audit and acceptance gates](research/RESEARCH_AUDIT_AND_GATES.md). New data folders: [wind robustness](results/wind_robustness/), [barrier V1](results/barrier_guard/), [closing-speed barrier V2](results/barrier_ttc/), [joint route+SF V1](results/joint_gate/), [V2](results/joint_gate_v2/), [V3](results/joint_gate_v3/), and [propulsion energy proxy](results/flight_energy_proxy/).

**Do not interpret any candidate controller as robust:** the legacy, higher-authority, integral and both obstacle barriers fail at least one fixed-seed safety criterion. Full UAV battery energy, GPS-denied SLAM, realistic LoRa radio and unseen-city robustness are still unvalidated. The browser simulator remains separate from these offline research results; Vercel has not been used to host graphs or datasets.

## Phase 6: Native city replication and map-aided LiDAR localization

The research pipeline now tests the frozen preliminary communication-and-route policy on four urban city seeds that differ from the initial map. Across the two evaluations, 16/16 reference-navigation MuJoCo missions were completed with no contact and required clearance, under mild deterministic force disturbances. Paired synthetic LoRa outcomes from 384 packet-replay trials show that the joint route/SF decision does not always improve timely critical-alert delivery or total mission energy proxy.

Native controller experiments now feed a noisy XY position estimate into the actual MuJoCo flight loop. Brownian drift causes safety to drop from 8/8 at zero drift to 3/8 at drift 0.06 m/sqrt(s) on two city seeds. A 16-beam, known-map horizontal LiDAR scan matcher reduces the mean estimated XY error from 1.039 m to 0.122 m under that setting and changes safe completions from 3/8 to 8/8. This is a GPS-free **map-aided localization experiment**, **NOT verified SLAM**: the map, orientation and body velocity are idealized, and no real LiDAR or IMU hardware has been tested.

Research files:
- results/unseen_city and results/unseen_city_replication: 4 urban seeds, flight failures and LoRa trials.
- results/localization_stress: 32 drifting-estimate flight tests.
- results/lidar_map_matching: 32 paired closed-loop known-map LiDAR correction comparisons.
- research/RESEARCH_AUDIT_AND_GATES.md: acceptance gates and negative findings.

The live Vercel browser lab remains separate; native experimental data are published in GitHub, not hosted on Vercel.

## Phase 7: experimental 3D LiDAR/IMU and uncertainty-triggered replanning

The native physics research branch now has a 48-ray 3D LiDAR scan matcher, inertial XYZ/velocity/yaw state propagation, synthetic barometer + optical-flow aiding, an uncertainty proxy, and map-known 3D A* replanning with geometric and low-speed route-handoff checks. These are Python research modules, **not** a field-ready ROS 2 autopilot or the default browser controller.

**Critical safety result:** Under a 30-second LiDAR blackout, the reproducible V5 test (two seeded cities, both routes, one heavy UAV) found 4/4 safe physical flights for the established 2D map-aided baseline, but only 1/4 for the new 3D hybrid without replanning and 1/4 for the same hybrid with the proposed clearance/speed-gated replanner. All V1–V5 negative results are preserved. Although map-aided corrections reduced average position drift, the hybrid controller is **rejected for promotion** until it can satisfy the full zero-contact/clearance/mission-completion envelope in new environments.

See [Phase 7 audit](research/PHASE_7_3D_LIDAR_IMU_AUDIT.md) and [V5 native MuJoCo results](results/imu3d_replanning_v5/). The **physical validation pathway** is documented in [Hardware validation protocol](research/HARDWARE_VALIDATION_PROTOCOL.md). [Physical-only LoRa log analysis](analysis/hardware_lora_validation.py) refuses to claim field measurements without genuine site-disjoint TX/RX logs. No real LoRa packets, 3D LiDAR scans or UAV flight records have been supplied or processed; the original GPS-denied hardware research objective is **not yet demonstrated**.
