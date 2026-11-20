# MARS-LoRa: Mobility-Aware Reliability and Safety for LoRa UAV Links

Research prototype accompanying an **eight-page maximum** manuscript on LoRa for autonomous aerial robots.

## Scope and scientific status
The **runnable baseline** is a deterministic, event-driven **raw LoRa link abstraction** operating on configurable 3D inspection trajectories. It is **not** a hardware-validated PHY, an ns-3 LoRaWAN model, or proof of real-world UAV safety. MuJoCo and ROS 2 adapters are developed separately. Never present synthetic/example plots as measurements.

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
- Trajectory generation is a **kinematic surrogate**. To claim physically realized flight, run the MuJoCo bridge and feed logged poses into the channel model.
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
Code: MIT (see LICENSE). Third-party drone models retain their original licenses and are **not vendored** into this repository.

## Multi-airframe MuJoCo flight console (November 2026)
- Airframe selector: original 1.2 kg inspection quad or a **27 g Crazyflie 2-derived** dynamics model. Real MuJoCo free-body integration and four independent rotor forces are applied in either case, with distinct thrust limits, rotor arms, mass and moments of inertia.
- Crazyflie physical parameters sourced from [Google DeepMind MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/bitcraze_crazyflie_2) (upstream MIT copyright preserved under `mujoco_sim/reference/`). The source model needs its original OBJ meshes; the Vercel variant is a browser-compatible primitive reconstruction, **not** an identical imported mesh model.
- Shared industrial site: three buildings, container, transmission mast, pylon and overhead beam with MuJoCo contacts. 3D scene rendering is separate from but geometrically aligned with the physical static colliders. Extra distant visualization tanks are only scenery and have **no collision shape**.
- Browser telemetry includes airframe pose, speed, rotor thrust, waypoint status, and an **interactive approximate raw-LoRa link** showing RSSI, information freshness and packet delivery. The packet simulation depends on the drone's current physics state but remains a simplified, seedable stochastic model; SF synchronization, collision and regulatory constraints are idealized. No research figures or static trial results are deployed on Vercel.
- Headless Chromium CI tests physics initialization, takeoff, waypoint activation, pause/reset, airframe switching and live critical-alert injection.

## Controlled research ablation
Use `python -m experiments.ablation --seeds 20 --output results/ablation` for eight policy combinations with identical telemetry generation rates and common seeded time-indexed fading. Toggle adaptive SF (A), critical packet priority (P), and conservative slowdown (S) independently. GitHub Actions `.github/workflows/ablation.yml` publishes 1,440 synthetic trial logs and summaries to `results/ablation`; nothing from these files is hosted by Vercel. Manuscript includes descriptive table; no hardware validation or demonstrated safety claims.
