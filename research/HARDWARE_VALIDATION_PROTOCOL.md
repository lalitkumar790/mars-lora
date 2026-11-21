# MARS-LoRa — Independent hardware validation protocol

**Status:** No physical radio measurements, measured 3D LiDAR scans, or UAV test-flight logs are yet available. Every existing LoRa PDR/AoI result is from simulation. This protocol cannot substitute for collected evidence.

## Equipment and calibration

- Two compatible raw-LoRa transceivers (Semtech SX1276-class or equivalent), antennas, fixed gateway/receiver, radio firmware able to log **all attempted TX packet IDs**, and received packet RSSI/SNR/arrival time. Preserve SF, bandwidth, coding rate, CRC, TX power, frequency, preamble and explicit payload lengths.
- Synchronized sender/receiver clocks if reporting **one-way** delay. Otherwise measure ACK/RTT without presenting it as one-way latency.
- Calibrated shunt or independent power sensor for radio TX/RX/MCU and independent battery current/voltage for entire aircraft, with measurement accuracy and sample frequency recorded.
- 3D LiDAR or calibrated visual-inertial sensor with timestamp synchronization and known extrinsics; IMU rate/bias/noise measurements; a scoring-only independent truth trajectory (motion capture/RTK/total station). The available 2D RPLIDAR C1 does **not** by itself produce a full aerial 3D scan.
- In a real deployment, monitor actuator temperature, battery voltage sag, motor current, radio interference, obstacles, and implement emergency-stop/manual takeover.

## Split and sample design

Pre-register site IDs and freeze model/hyperparameters *before* heldout experiments. Start with at least three separate calibration sites and three separate heldout sites; do not reuse locations. At each site vary LOS/NLOS, distance, antenna orientation, SF7–SF12, payload, moving/static transmitter and background traffic. Initially collect at least 100 attempted packets per condition, with randomized order and multiple sessions. Estimate confidence intervals and collect more as needed; 100 packets is a study starting point, NOT a safety certification.

Progress through stationary bench tests, fixed ground-based spatial tests, then supervised tethered/caged flight, and only finally appropriately authorized UAV experiments. Validate emergency stop, geofence and manual control first.

Do not conflate raw LoRa modulation with LoRaWAN network operation. The LoRa Alliance RP002-1.0.5 specification includes regional LoRaWAN parameters such as IN865; it is **not itself an authorization to transmit arbitrary raw LoRa**. Verify the applicable Indian WPC and aviation rules independently before any RF emission or flight.

References:
- https://resources.lora-alliance.org/technical-specifications/rp002-1-0-5-lorawan-regional-parameters
- https://www.semtech.com/products/wireless-rf/lora-connect/sx1276

## Physical packet log schema and validator

The required columns are:

site_id,split,device_id,sf,packet_id,payload_bytes,tx_timestamp_s,rx_success,tx_current_a,tx_voltage_v,tx_duration_s

Optional received metrics: rx_timestamp_s,rx_rssi_dbm,rx_snr_db,distance_m,obstruction_tag

Each **transmitted packet**, including every lost packet, must be represented by one row. The split field must be exactly calibration or heldout. Heldout sites must be disjoint. Never generate fake "physical" rows from the simulation.

Use:

    python -m analysis.hardware_lora_validation --input path/to/real_packets.csv --output results/physical_lora_validation

The validator computes per-site/SF empirical PDR, successful-packet latency, RSSI/SNR and radio-TX current*voltage*time energy. It rejects missing RX measurements for delivered packets, duplicate IDs, and overlapping training/test sites. It cannot independently verify actual clock synchronization, RF legal compliance or whole-vehicle battery energy.

## GPS-free 3D estimator comparison

Record synchronized ROS 2 topics: /imu/data_raw, /lidar/points, /odom/estimated, /tf, /planner/path, /radio/tx, /radio/rx, /power/current, /safety/events. Save rosbag2 with clear coordinate frames and transformation calibration. An external LiDAR-inertial package (e.g. FAST-LIO2, LIO-SAM port) can serve as an independent benchmark when compatible with the deployed ROS2 distribution/driver/hardware; integration has **not** yet been tested here.

Measure 3D absolute and relative pose errors (ATE/RPE), drift during sensor dropout, map reconstruction/loop-closure, error-estimate calibration, CPU and memory, scan-to-action latency and catastrophic false registration. Truth data must be scoring-only, not the estimator input.

## Acceptance gates for the research claim

1. Safety gate: mission completion or explicit emergency landing, no collisions, clearance >= 0.25m in the declared controlled test envelope; report failures and collision severity, never only surviving packet traces.
2. Radio gate: delivered/attempted PDR, fraction of critical alerts before a 3-second deadline, p95 data age, time-on-air, channel occupancy, receiver SF agreement and duty-cycle compliance.
3. Joint-policy gate: compare shortest-safe+adaptive LoRa, RF-aware route+adaptive LoRa and deadline/uncertainty/energy aware route+SF on the *same* blind sites and critical traffic, using measured energy of flight, RX, TX and computation.
4. Statistical gate: frozen pre-registered parameters, independent locations, paired trials and uncertainty intervals over sites and environmental conditions; keep negative results.
5. Scope gate: GPS-denied **3D SLAM** requires initially unknown-map estimation and validation, not simply matching scans to a perfect map.

No simulated or bench output should be called real flight, measured GPS-denied SLAM or hardware-calibrated communication until primary physical logs and approvals exist.
