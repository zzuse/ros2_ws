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
2. Note the current odom pose: `ros2 topic echo /odom --field pose.pose.position`
3. Drive straight ahead exactly **1.0 m** (measure with a tape measure, slowly, ≤ 0.15 m/s).
4. Note the odom pose again and compute the reported distance:
   **d = √(Δx² + Δy²)** — do NOT compare `position.x` alone. Odometry is in the
   odom frame: unless the robot's yaw was exactly 0 when you started, the motion
   splits between x and y (e.g. Δx = 0.40, Δy = 0.88 just means the robot was
   facing ~65° in the odom frame; the distance is √(0.40² + 0.88²) ≈ 0.97 m).
   Restarting `bringup.launch.py` right before the test resets odom to (0,0,0)
   and puts all motion in x, if you prefer that.

- Reported > actual → wheel diameter in firmware is too large (or ticks/rev too small). Scale the diameter by `actual / reported`.
- Reported < actual → wheel diameter too small (or ticks/rev too large). Same formula (factor > 1).
- Error should be **< 1–2 %** (≤ 2 cm over 1 m). Repeat 3 times and average.
- Sanity check: the robot must physically track straight. If it veers, the path is
  longer than the displacement and the scale correction will be overstated — fix
  left/right balance first (or push it along a straightedge).

### 1b. Rotation test (wheel separation / track width)

1. Watch yaw directly with `ros2 run tf2_ros tf2_echo odom base_footprint`
   (prints roll/pitch/yaw in radians and degrees — much easier than quaternions).
   If you use `ros2 topic echo /odom --field pose.pose.orientation` instead,
   convert each full quaternion to an angle with **yaw = 2·atan2(z, w)** and
   subtract the two yaws. Do NOT subtract quaternion components (Δz, Δw) —
   those deltas are meaningless.
2. Rotate the robot in place exactly **one full turn** (use a tape mark on the floor and on the chassis; rotate slowly, ≤ 0.3 rad/s).
3. Reported yaw change should be exactly 2π. Note: the endpoint yaw alone can't
   count whole turns (+15° and +375° look identical), so the physical tape mark
   is what defines "one turn"; the leftover yaw difference is the error.

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
wired lidar. The lidar actually spins at ~9.8 Hz (the `frequency: 5.0` in ydlidar.yaml
is ignored — single-channel lidars have no speed control), so one sweep takes ~102 ms
and a `ros2 topic delay /scan` around 100 ms is pure acquisition time, not WiFi lag.
Measured 2026-07-06: /odom 9 ms, /scan 104 ms, 9.8 Hz — pass. If the delay spikes well
above ~150 ms or the rate sags below ~9 Hz, improve the WiFi link before blaming SLAM.

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

- The lidar spins at ~**9.8 Hz** with a 3 kHz sample rate → ~300 points/rev (~1.2° resolution): drive ≤ 0.3 m/s, rotate ≤ 0.5 rad/s, and **pause 1–2 s at corners** so the matcher gets clean scans. So bump the turn rate to ~0.3–0.5, optionally linear to ~0.2, and keep the habit of pausing 1–2 s at corners.
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
| Scan delay | `ros2 topic delay /scan` | ~100 ms (≈1 sweep), ~9.8 Hz steady |
| TF tree | `view_frames` | single tree, current stamps |
| Smear test | RViz, fixed frame `odom`, decay 5 s | walls crisp while rotating |
| Localization | RViz after initial pose | scan overlays map walls |
