## Debug Thinking Process
0. ASK LLM why action goal been rejected in ros2 jazzy.
1. compare old project yaml and world sdf files, compare and using LLM to convert to jazzy format.
2. examine every config_bridge to check mapping topic
3. list every topic info and topic message format

## Try 1
Commenting out cmd_vel, odometry, tf, and joint_states in bridge.yaml is the correct solution. Here is why:

Because you are using the gz_ros2_control plugin with the diff_drive_controller and joint_state_broadcaster, ROS 2 Control is already handling
these topics directly on the ROS 2 side:
   * cmd_vel: The diff_drive_controller directly subscribes to the ROS 2 /cmd_vel topic.
   * odometry & tf: The diff_drive_controller calculates the wheel odometry and directly publishes it to the ROS 2 /odom and /tf topics. Bridging
   them from Gazebo would create a conflicting TF tree (which breaks Nav2).
   * joint_states: The joint_state_broadcaster directly publishes to the ROS 2 /joint_states topic.

## Try 2
Leaving those in the bridge.yaml creates duplicate, conflicting publishers for odometry and TF, which explains why the behavior_server and Nav2
stack were failing to calculate a valid path and timing out.

  1. Fixed TF and Topic Conflicts (Gazebo vs. ROS 2 Control)
  The core reason navigation was initially failing to calculate paths was a conflict between your Gazebo bridge
  and ROS 2 Control:
   * The Problem: You were bridging cmd_vel, odom, tf, and joint_states via ros_gz_bridge in bridge.yaml. However,
     you are also using the gz_ros2_control plugin (with diff_drive_controller and joint_state_broadcaster).
   * The Fix: You commented out those topics in bridge.yaml. Because ros2_control already handles these natively
     on the ROS 2 side, bridging them simultaneously created duplicate publishers, causing a conflicting TF tree
     that completely broke Nav2.
   * Result: Clean, conflict-free Odometry and TF trees published directly by diff_drive_controller.

  2. Standardized Coordinate Frames
  Inconsistent base frames between the URDF and Nav2 parameters were causing the global costmap to fail to align
  with the robot.
   * The Fix: You established base_footprint as the standard base frame. We went through nav2_params.yaml and
     ensured that base_frame_id (AMCL) and robot_base_frame (Costmaps, BT Navigator, Behavior Server, Collision
     Monitor) were all uniformly set to base_footprint.

  3. Global Costmap Map Reception
  The global costmap wasn't receiving the map from the map_server.
   * The Fix: We explicitly added map_topic: /map and enabled: True to the static_layer configuration under
     global_costmap to guarantee the costmap subscribed to the correct global map topic.

  4. ROS 2 Jazzy Compatibility (Behavior Tree Navigator)
  When launching, bt_navigator was crashing with an ID [ComputePathToPose] already registered error.
   * The Fix: We updated the bt_navigator configuration to meet ROS 2 Jazzy standards:
     * Removed the massive manual list of plugin_lib_names (Jazzy auto-registers default plugins, so listing them
       manually caused duplication errors).
     * Replaced the deprecated error_code_names with the mandatory error_code_name_prefixes:
       ["nav2_behavior_tree"].
     * Updated plugin references to use the modern nav2_bt_navigator:: namespace syntax.

  5. General Nav2 Parameter Polish
   * Use Sim Time: Ensured use_sim_time: True was correctly applied to all nodes (especially robot_state_publisher
     and all Nav2 lifecycle nodes) so ROS and Gazebo share the same clock.
   * Plugin Syntaxes: Updated older plugin notations (e.g., using :: instead of / for things like
     nav2_navfn_planner::NavfnPlanner and nav2_behaviors::Spin).
   * Robot Radius: Tweaked the robot_radius in the costmaps to 0.12 to accurately reflect the physical footprint
     of the Fishbot, preventing false-positive collision detections when planning tight paths.

  Summary: 
  You essentially cleaned up a major architectural conflict between Gazebo bridging and ros2_control, standardized
  your TF frames, and updated a legacy Nav2 configuration to fully comply with ROS 2 Jazzy standards. Excellent
  debugging!


Since we also successfully updated the frame names (base_footprint), remapped the odom topic, and fixed the use_sim_time for the
robot_state_publisher, your workspace is fully aligned now. 


## The True Root Cause: Command Velocity Multiplexing and Stamped vs Unstamped

While fixing the `bridge.yaml` resolved the Odometry/TF conflicts, the primary reason the robot wasn't moving efficiently (or correctly) was a structural issue with how velocity commands were routed to the `diff_drive_controller`. 

In ROS 2, controllers like `diff_drive_controller` expect a `TwistStamped` message, but various Nav2 components (like `controller_server`, `collision_monitor`, and `docking_server`) often output unstamped `Twist` messages or publish to overlapping topics. 

### The Cleanest Fix — Multiplex + Single Relay

The definitive solution was to introduce `twist_mux` to manage competing velocity commands based on priority, and then use a single relay to convert the final output to a `TwistStamped` message.

**1. Install `twist_mux`**
```bash
sudo apt install ros-jazzy-twist-mux
```

**2. Configure `twist_mux` priorities (`twist_mux_config.yaml`)**
```yaml
twist_mux:
  ros__parameters:
    topics:
      navigation:
        topic: /cmd_vel_nav        # from Nav2 controller+smoother
        timeout: 0.5
        priority: 10
      collision:
        topic: /cmd_vel_collision  # from collision_monitor
        timeout: 0.5
        priority: 20               # higher = takes precedence
      docking:
        topic: /cmd_vel_dock       # from docking_server
        timeout: 0.5
        priority: 30
```

**3. The Architecture**
By routing everything through `twist_mux`, priority logic is handled cleanly (e.g., collision avoidance and docking take precedence over standard navigation). The output is then converted once.

```text
controller_server → /cmd_vel_nav       ─┐
collision_monitor → /cmd_vel_collision ─┤→ twist_mux → /cmd_vel (Twist)
docking_server    → /cmd_vel_dock      ─┘                      ↓
                                              cmd_vel_relay (single relay)
                                                               ↓
                                         /diff_drive_controller/cmd_vel (TwistStamped)
```

This ensures a single, clean conversion point regardless of how many Nav2 nodes are in the pipeline, eliminating conflicting commands and message-type mismatches.

**4. The AI helping** 
https://claude.ai/share/ccc20778-9d51-466d-8e3c-9940fed92235
