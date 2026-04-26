# Executed Commands Log

Here is the chronological list of shell commands executed during the troubleshooting session:

```bash
# 1. Start Navigation (background)
ros2 launch fishbot_navigation2 navigation2.launch.py use_sim_time:=true

# 2. Check active nodes and lifecycle states
sleep 10 && ros2 node list && ros2 lifecycle nodes

# 3. Check bt_navigator node info
ros2 node info /bt_navigator

# 4. Check TF from map to base_footprint (cancelled due to long output)
ros2 run tf2_ros tf2_echo map base_footprint

# 5. Check /scan topic info and frequency
ros2 topic info /scan && ros2 topic hz /scan --window 10 & sleep 5 && kill $!

# 6. Check Gazebo topics and ROS /clock frequency
ros2 topic hz /clock --window 10 & sleep 2 && kill $! && gz topic -l

# 7. Check running Gazebo processes
ps aux | grep gz

# 8. Attempt to echo Gazebo /clock (failed syntax)
gz topic -e -n 1 /clock

# 9. Get detailed /scan topic info
ros2 topic info /scan --verbose

# 10. Attempt to echo Gazebo /lidar topic (failed syntax)
gz topic -t /world/empty/clock -e --count 1 && gz topic -t /lidar -e --count 1

# 11. Attempt to echo ROS /clock (failed syntax)
timeout 5 ros2 topic echo /clock --count 1

# 12. Echo ROS /clock and list all ROS topics
timeout 5 ros2 topic echo /clock && ros2 topic list

# 13. Run manual bridge for /scan (debug mode)
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan --ros-args --log-level debug

# 14. Echo Gazebo /lidar topic
timeout 2 gz topic -e -t /lidar

# 15. Run manual bridge for /lidar -> /scan mapping (GZ to ROS)
ros2 run ros_gz_bridge parameter_bridge "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" --ros-args --log-level info

# 16. Check if /lidar topic exists in ROS
ros2 topic list | grep lidar && timeout 2 ros2 topic echo /lidar --count 1

# 17. Attempt to echo /lidar in ROS (failed syntax)
timeout 2 ros2 topic echo /lidar -n 1

# 18. Check frequency of /lidar in ROS
timeout 5 ros2 topic hz /lidar

# 19. Run corrected manual bridge with topic remapping (/lidar to /scan)
kill 51914 && ros2 run ros_gz_bridge parameter_bridge "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan" --ros-args -r /lidar:=/scan

# 20. Check frequency of /scan in ROS
timeout 5 ros2 topic hz /scan

# 21. List recent ROS logs
ls -t ~/.ros/log | head -n 5

# 22. Search logs for lidar/scan errors
grep -riE "error|warn" ~/.ros/log | grep -E "lidar|scan" | tail -n 20

# 23. Attempt to compare timestamps (failed syntax)
ros2 topic echo /clock -n 1 && ros2 topic echo /scan -n 1 | grep stamp -A 5
timeout 2 ros2 topic echo /clock && timeout 2 ros2 topic echo /scan | grep stamp -A 2 | head -n 3

# 24. List all pose-related topics
ros2 topic list | grep pose

# 25. Echo /amcl_pose
timeout 2 ros2 topic echo /amcl_pose

# 26. Get lifecycle state of amcl
ros2 lifecycle get /amcl

# 27. Check use_sim_time parameter for amcl
ros2 param get /amcl use_sim_time

# 28. Get info for /goal_pose and /initialpose
ros2 topic info /goal_pose && ros2 topic info /initialpose

# 29. Find ros_gz_bridge logs
find ~/.ros/log -name "*.log" -exec grep -l "ros_gz_bridge" {} + | xargs ls -t | head -n 5

# 30. Search for errors in specific bridge log
cat /home/zzuse/.ros/log/bridge_node_32695_1776833862136.log | grep -iE "error|warn"

# 31. Read first 50 lines of bridge log
head -n 50 /home/zzuse/.ros/log/bridge_node_32695_1776833862136.log

# 32. Publish manual initial pose
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

# 33. Check if /amcl_pose is published after setting initial pose
timeout 5 ros2 topic echo /amcl_pose

# 34. Publish manual goal pose and attempt to read logs
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}" & sleep 5 && ros2 read_background_output --pid 36911

# 35. Echo /cmd_vel
timeout 5 ros2 topic echo /cmd_vel

# 36. Get verbose info for /cmd_vel
ros2 topic info /cmd_vel --verbose

# 37. Get info for smoothed and raw cmd_vel topics
ros2 topic info /cmd_vel_smoothed && ros2 topic info /cmd_vel_raw

# 38. Check subscriptions of /diff_drive_controller
ros2 node info /diff_drive_controller

# 39. Check use_stamped_vel param for /diff_drive_controller
ros2 param get /diff_drive_controller use_stamped_vel

# 40. Inspect robot_description for Gazebo plugin config
ros2 param get /robot_state_publisher robot_description | grep -A 5 "GazeboSimROS2ControlPlugin"

# 41. Verify existence of controller yaml config file
ls -l /home/zzuse/chapt4/chapt4_ws/install/fishbot_description/share/fishbot_description/config/fishbot_ros2_controller.yaml

# 42. List all params for /diff_drive_controller
ros2 param list /diff_drive_controller

# 43. Check use_stamped_vel param for /controller_server
ros2 param get /controller_server use_stamped_vel

# 44. Publish TwistStamped to /cmd_vel to test diff_drive_controller
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"

# 45. List params for /velocity_smoother
ros2 param list /velocity_smoother

# 46. List params for /collision_monitor
ros2 param list /collision_monitor

# 47. List params for /controller_server
ros2 param list /controller_server
```