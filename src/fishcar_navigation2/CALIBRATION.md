# Fishcar Calibration & Debug Guide

Steps to verify odometry, timestamps, and TF before (re)building a map and running Nav2.
Do them in order — later steps are meaningless if an earlier one fails.

## 1. Odometry sanity test (do this first)

Bad wheel odometry (especially yaw) is the number-one cause of rotated/smeared maps.
SLAM can correct small odom errors, not systematic ones.

Bring up the robot base only:

```bash
ros2 launch fishcar_bringup bringup.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 1a. Straight-line test (wheel diameter / ticks-per-meter)

1. Mark the robot's start position on the floor (tape at a wheel axle reference point).
2. Reset or note the current odom pose: `ros2 topic echo /odom --field pose.pose.position`
3. Drive straight ahead exactly **1.0 m** (measure with a tape measure, slowly, ≤ 0.15 m/s).
4. Compare reported `position.x` change vs. the measured 1.0 m.

- Reported > actual → wheel diameter in firmware is too large (or ticks/rev too small). Scale it by `actual / reported`.
- Error should be **< 1–2 %** (≤ 2 cm over 1 m). Repeat 3 times and average.

### 1b. Rotation test (wheel separation / track width)

1. Note starting yaw:
   `ros2 topic echo /odom --field pose.pose.orientation`
   (or watch yaw directly with `ros2 run tf2_ros tf2_echo odom base_footprint`).
2. Rotate the robot in place exactly **one full turn** (use a tape mark on the floor and on the chassis; rotate slowly, ≤ 0.3 rad/s).
3. Reported yaw change should be exactly 2π (quaternion back to start).

- Reported > 2π → firmware `wheel_separation` too small; multiply it by `reported / 2π`.
- Reported < 2π → `wheel_separation` too large; same formula.
- Repeat for 3 turns (6π) to average out noise. Target error **< 2–3° per full turn**.

Fix these in the ESP32 firmware, re-flash, and re-run both tests until they pass.

## 2. Timestamp / clock-sync check

The ESP32 (micro-ROS) clock must agree with the host, and no node may stamp TF in the future.

```bash
# Both should show small positive delays (a few ms .. tens of ms).
# NEGATIVE delay = message stamped in the future = broken.
ros2 topic delay /odom
ros2 topic delay /scan

# Compare stamps directly (should differ < ~50 ms from each other and from wall clock):
ros2 topic echo /odom --field header.stamp --once
ros2 topic echo /scan --field header.stamp --once
```

If `/odom` stamps are far off, add periodic time sync in the firmware
(`rmw_uros_sync_session()` + stamp with the synced epoch time).

Note: `/scan` arrives over the WiFi→TCP serial bridge, so expect more jitter than a
wired lidar. If `ros2 topic delay /scan` regularly exceeds ~100 ms or `ros2 topic hz /scan`
drops below the configured 5 Hz, improve the WiFi link before blaming SLAM.

## 3. TF tree check

```bash
ros2 run tf2_tools view_frames        # produces frames.pdf
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_link
```

Verify:

- Single connected tree: `map → odom → base_footprint → base_link → laser_link`, no duplicate publishers for the same transform.
- `odom → base_footprint` stamps are current (not future-dated) and update at the odom rate.
- `base_link → laser_link` matches the real mounting (x/y offset and yaw). A wrong lidar
  yaw offset shows up as a constant rotation of every scan — and a rotated map.
  Cross-check with the `reversion`/`inverted` flags in `ydlidar.yaml`.

## 4. RViz smear test (live scan quality)

1. Open RViz, set **Fixed Frame = `odom`**.
2. Add a LaserScan display for `/scan`, set **Decay Time = 5 s**.
3. Drive straight, then rotate in place.

Interpretation:

- Walls stay crisp while rotating → odometry + timestamps are good; proceed to mapping.
- Walls smear into arcs **only while rotating** → yaw odometry or timestamp offset still wrong (re-check steps 1b and 2).
- Walls offset by a constant angle → lidar mount yaw / `reversion` flags wrong (step 3).

## 5. Mapping best practices (slam_toolbox)

```bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=False \
  slam_params_file:=src/fishcar_bringup/config/slam_toolbox.yaml
```

- The lidar spins at only **5 Hz** (one sweep = 200 ms): drive ≤ 0.2 m/s, rotate ≤ 0.3 rad/s, and **pause 1–2 s at corners** so the matcher gets clean scans.
- Prefer one slow continuous loop around the room over back-and-forth wandering; close the loop by returning to the start.
- Watch the map live in RViz — if a wall gets stitched in rotated, stop, back up, and let it re-match rather than continuing to build on a bad graph.
- Keep match-quality params near defaults (`link_match_minimum_response_fine` ≈ 0.45–0.5). Loosening them "to make it stitch" bakes rotation errors into the map.
- Save with: `ros2 run nav2_map_server map_saver_cli -f src/fishcar_navigation2/maps/room`

## 6. Navigation checks (Nav2)

```bash
ros2 launch fishcar_navigation2 navigation2.launch.py use_sim_time:=False
```

- After setting the initial pose in RViz, confirm the laser scan overlays the map walls
  **before** sending a goal. If the scan is rotated off the walls, localization is bad —
  navigation will hit things no matter how the costmaps are tuned.
- Verify `robot_radius` covers the widest point of the chassis (wheels included).
- If the robot cuts corners close, raise `inflation_radius` (both costmaps) so planned
  paths keep a cost "valley" away from walls, and/or lower `cost_scaling_factor`.
- If odometry remains mediocre, raise AMCL `alpha1`–`alpha4` (e.g. 0.2 → 0.3) so the
  particle filter trusts the laser more than the wheels.

## Quick regression checklist

| Check | Command | Pass criterion |
|---|---|---|
| Straight 1 m | teleop + `/odom` | error < 2 % |
| Rotate 360° | teleop + `tf2_echo odom base_footprint` | error < 3° |
| Odom delay | `ros2 topic delay /odom` | small positive, never negative |
| Scan delay | `ros2 topic delay /scan` | < ~100 ms, 5 Hz steady |
| TF tree | `view_frames` | single tree, current stamps |
| Smear test | RViz, fixed frame `odom`, decay 5 s | walls crisp while rotating |
| Localization | RViz after initial pose | scan overlays map walls |
