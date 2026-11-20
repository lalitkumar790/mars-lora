# GPS-Denied Navigation + SLAM: Implementation and Validation

## What is available now

**Browser flight lab** at https://mars-lora.vercel.app

1. Choose **GPS-denied odometry + scan matching** before takeoff.
2. Choose **GPS-denied landmark survey (3.2 m)**. The normal elevated inspection path is 7 m, above all buildings; a *horizontal* range scan at that height will not normally observe any surfaces and localization is not observable. The survey runs lower, along a collision-checked route outside the building footprints.
3. Click **Start selected mission** (you may click Take off first).
4. Watch the occupancy map and LiDAR beams, scan corrections, error versus simulator truth, controller, obstacle clearance, and LoRa telemetry.
5. Export the **LiDAR/odometry JSONL** for offline ROS 2 mapping, and the regular flight CSV for quantitative trajectory analysis.

### Implemented sensors and math

- Native **MuJoCo rangefinders**: 32 horizontally oriented body-attached rays spanning 360°, maximum range 18 m. Read from `MjData.sensordata` at 2 Hz; generated physics/range measurements use the actual MuJoCo collision geometry, not a geometry-only WebGL animation.
- Approximate inertial/visual odometry: body-frame velocity and yaw changes are obtained from physics-state proxies, perturbed by fixed bias and time-varying deterministic noise; the estimated pose is integrated without GPS XY updates. **These pseudo-measurements are NOT actual onboard IMU or camera data.**
- Mapping: a 100×100-cell, 0.35 m/cell signed occupancy grid. Free-space rays lower log odds; hit endpoints increase occupancy.
- Localization: bounded local scan-to-map correlation search in XY and yaw. It only corrects the pose where there are sufficient map features; no global relocalization, pose-graph optimization, IMU bias filter, or loop closure.
- Navigation: when GPS-denied mode is selected, waypoint following uses the estimated XY position; attitude is an IMU-like orientation proxy, and altitude uses the simulated vertical state. This is **planar GPS-denied guidance**, not independently validated 6-DoF autonomy.
- Simulator-only safety: collision clearance and contact monitoring use MuJoCo ground truth as a supervisor. This is NOT an onboard sensor-based obstacle detector; it must be replaced with range-based collision constraints before real deployments.
- LoRa is **not** a control-loop transport. Onboard stabilization continues even when LoRa suffers packet loss.

### Export format

`mars-lora-lidar-odom.jsonl` has one JSON object for each scan:

```json
{"t":1.5,"angle_min":-3.14159,"angle_increment":0.19635,"ranges":[...],"range_min":0.08,"range_max":18,"odom":{"x":0.01,"y":0.02,"yaw":0.0},"estimated":{"x":0.01,"y":0.02,"yaw":0.0}}
```

`odom` is uncorrected (drifty). `estimated` is the map-corrected pose. **The exported JSONL does not include the true MuJoCo position**, which remains restricted to separate evaluation flight logs.

### ROS 2 Jazzy / SLAM Toolbox offline integration

This optional adapter is checked in under `ros2_bridge/slam_replay.py`. It reads the JSONL and publishes a `sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `/tf` (odom→base_link and base_link→laser), and `/clock`. Use two terminals after sourcing your installed ROS distribution.

Terminal 1:
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

Terminal 2:
```bash
source /opt/ros/jazzy/setup.bash
python3 ros2_bridge/slam_replay.py /path/to/mars-lora-lidar-odom.jsonl
```

Visualize `/scan`, `/odom`, and `/map` in RViz2; frame fixed to `map`. `slam_toolbox` provides the map→odom estimate. This replay program is **not yet end-to-end verified in a live ROS 2 environment**. It is a path to use a maintained pose-graph SLAM system rather than equating the browser's local scan matching with a research-grade SLAM implementation.

Official references:
- https://index.ros.org/p/slam_toolbox/
- https://mujoco.readthedocs.io/en/stable/XMLreference.html (rangefinder sensors)

## Next research experiments

### Validation gate A — Localization
Use matched perturbations/seeded scene changes for >=20 trials, both copters and several altitude bands. Report absolute trajectory error (ATE), relative pose error (RPE), drift (m/min), scan matching success rate, occupancy IoU against known ground-truth map *for evaluation only*, map entropy, and runtime latency. Compare raw drifty odometry, local scan matching, and **ROS 2 slam_toolbox**. Measure degradation in feature-poor aerial corridors.

### Validation gate B — SLAM + RF flight policy
Use the GPS-denied estimated pose inside the mission planner, while the LoRa transmitter uses the real *physical* position and attitude for synthetic propagation. Do not let the LoRa optimizer access true XY position when evaluating GPS-denied planning. Compare a conservative safe corridor against a 3D obstacle-aware route and an RF-aware route. Evaluate risk of localization failure, obstacle clearance, mission time/energy, message deadline satisfaction and AoI.

### Validation gate C — Journal-quality depth
Add camera/visual-inertial odometry, actual calibrated IMU error and sensor bias, 3D LiDAR or stereo depth, loop closure, dynamic obstacles, map changes, altitude/attitude effects on RF, and realistic SF coordination cost. Use hardware measurements or independent RF validation before field reliability claims.

## Limitations

This is an experimental **2D slice + estimated horizontal-position navigation**. It is not 3D SLAM, real IMU integration, a certified GPS-less autopilot, or proof of safe autonomous navigation without external sensors. At 7 m there may be no returns at all, which the UI explicitly reports rather than pretending to localize.
