# Phase 3 — Closed-loop UAV mission safety and communication evidence

**Status:** engineering implementation and validation in progress. This document defines future work, not completed research findings.

## Root cause and fix: collision during mission
The original waypoint sequence ended with a diagonal from [-7,-7,4] to [0,0,2.5]. This passes through the storage-container XY footprint x∈[-5,-1], y∈[-4.7,-3.3]. The previous guidance law aimed directly at point waypoints and did not query obstacle collisions or clearance. Near objects at mission altitude 3.5–4 m it also allowed risky lateral commands without a dedicated climb phase.

Implemented now:
- Conservative high-altitude flight corridor: first climb to **7 m**, above the highest 5.68 m overhead beam plus clearance, traverse the survey route, return to base above obstacles, then descend.
- Pure JS swept-segment clearance tests (0.1 m samples), automatically evaluated in CI; old mission is required to fail and the replacement to pass.
- Altitude-first waypoint follower, velocity damping, 16° tilt cap, gravity/tilt compensation, approximate first-order rotor response (60 ms), and linear aerodynamic drag.
- On-screen actual-vs-commanded altitude graph, motor thrust vector (amber), vehicle velocity (cyan), waypoint/path geometry, MuJoCo contact count, obstacle clearance and controller diagnostics.
- Export flight CSV now contains target XYZ, flight errors, tilt, clearance, contacts, RSSI, SF, packet counts, and AoI.

**Caveats:** This is conservative preplanned clearance, not online 3-D motion planning; scenery tanks without collision shapes are not included in validation. Static geometry estimates cannot replace dynamic collision testing. No sensor fusion, onboard obstacle perception, calibrated rotor aerodynamics, or rotor-speed state has been proven.

## Next phase (Phase 4) — publishable research protocol

### P4.1 Physics reliability (first gate)
1. Test both airframes through **full multi-waypoint missions** in actual MuJoCo, not only takeoff smoke tests. Record position/attitude trajectories, minimum swept clearance, MuJoCo contact events, tracking error, ascent/descent timing, and torque/thrust saturation.
2. Introduce controlled wind profiles and sensor noise; test low-level control tracking with and without motor lag and drag. Tune gains independently for 1.2 kg inspection drone and 27 g Crazyflie surrogate. Keep a baseline with the old controller for ablation.
3. Use metrics: waypoint completion (%), collision count, min obstacle clearance (m), position RMS (m), altitude overshoot (m), settle time (s), rotor saturation fraction (%), modeled force/energy, mission duration (s). Report confidence intervals over seeded disturbances.

### P4.2 Communication-physics coupling
1. Replace synthetic rectangular kinematic trajectories in the **research runner** with timestamped MuJoCo CSV flight traces; use actual altitude, attitude, speed, and obstruction geometry at the radio receiver.
2. Add an explicit ground-station/radio state machine with LoRa SF switching acknowledgement, SF negotiation delay, uplink/downlink airtime, packet queue expiry and legal regional airtime constraints. No claims of realistic collisions until a MAC/PHY model is implemented.
3. Implement correctly timestamped packet completions and age-of-information only on *freshest* successfully delivered telemetry; do not falsely update AoI for a late packet.
4. Model sensitivity to LoRa frequency, bandwidth, coding rate, antenna orientation, height, LOS/NLOS conditions and shadow autocorrelation.

### P4.3 Safety-critical task
1. Define a ground-station-issued exclusion zone with a 3-second alert deadline and onboard fall-back behavior on timeout. **Do not place motor stabilization on LoRa.**
2. Compare the original fixed/feedback-adaptive/priority policies against genuinely prediction-triggered mission modification, under identical flight trajectories, packet generation, synthetic RF seed fields and disturbances.
3. Run 20–30+ matched seeds across speed, obstruction, wind, SF/airtime, alert urgency, and background traffic.
4. Study **tradeoffs**, not just wins: critical on-time ratio, 95th-percentile AoI, outage-length distribution, PDR, radio energy proxy, mission success, track error, collision/contact events, critical-command response delay, and cost in flight time/energy.

### P4.4 External validation
Validate LoRa physical assumptions against measured SX126x/SX127x logs or an independent ns-3 LoRaWAN/raw LoRa setup (specify exact stack). Record 95% confidence intervals, control-policy ablations, and limitations. A fully synthetic channel plus MuJoCo is **not** a field-validated UAV communication safety result.

## Publication honesty checkpoints
- Manuscript remains <=8 pages including references, compiled in GitHub Actions.
- Only figures constructed from executed experiment logs go in paper.tex.
- The Vercel deployment is an **interactive simulator** and displays ephemeral live physics/LoRa telemetry, not publication trial data or generated research graphs.
- Comparisons should not imply safety or flight certification.
