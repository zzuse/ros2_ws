# ROS 2 Workspace

This workspace contains various ROS 2 packages demonstrating Service-Client communication patterns in C++ and Python.

## Packages

### 1. `chapt4_interfaces`
Contains custom service definitions used across the workspace.
- **`Patrol.srv`**: Service for sending a target (x, y) coordinate to a turtle.
- **`FaceDetector.srv`**: Service for face detection, taking an image as input and returning detection counts and bounding boxes.
```sh
ros2 pkg create chapt4_interfaces --dependencies sensor_msgs rosidl_default_generators --license Apache-2.0
ros2 interface chapt4_interfaces/srv/FaceDetector
```

### 2. `demo_cpp_service`
C++ implementation of a turtle control system.
- **`turtle_control_node`**: A service server that receives `Patrol` requests and drives the `turtlesim` turtle to the target location using a simple P-controller.
- **`patrol_client_node`**: A service client that periodically sends random valid coordinates (1.0 to 10.0) to the patrol service.
```bash
ros2 interface show chapt4_interfaces/srv/Patrol
ros2 pkg create demo_cpp_service --build-type ament_cmake --dependencies chapt_interfaces rclcpp geometry_msgs turtlesim --license Apache-2.0
```

### 3. `demo_python_service`
Python implementation of face detection services.
- **`face_detect_node`**: A service server that processes images to detect faces.
- **`face_detect_client_node`**: A client that sends images to the detector node.
- **`learn_face_detect.py`**: Educational script for learning face detection concepts.
```sh
ros2 pkg create demo_python_service --build-type ament_python --dependencies rclpy chapt4_interfaces --license Apache-2.0
```

### 4. `demo_python_tf`
Python implementaion of cordinate transformation
```sh
ros2 pkg create --build-type ament_python --dependencies rclpy geometry_msgs tf2_ros tf_transformations --license Apache-2.0 demo_python_tf
```

### 5. `demo_cpp_tf`
```sh
ros2 pkg create demo_cpp_tf --build-type ament_cmake --dependencies rclcpp tf2_ros geometry_msgs tf2_geometry_msgs --license Apache-2.0
```

### 6. `fishbot_description`
```sh
ros2 pkg create fishbot_description --build-type ament_cmake --license Apache-2.0
```

### 7. `fishbot_navigation2`
```sh
ros2 pkg create fishbot_navigation2
```

### 8. `fishbot_application`
```sh
ros2 pkg create fishbot_application --build-type ament-python --license Apache-2.0
```

### 9. `autopatrol_robot`
```sh
ros2 pkg create autopatrol_robot --build-type ament_python --dependencies rclpy nav2_simple_commander --license Apache-2.0
```


## Prerequisites
- ROS 2 (jazzy or newer recommended)
- `turtlesim` package
- OpenCV (for Python face detection)
```
sudo apt install python3-opencv
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-vision-opencv
sudo apt install ros-jazzy-launch-ros
pip3 install face_recognition
```
ENV source
```
. /opt/ros/jazzy/setup.bash  # CV BRIDGE
. ~/ros2_jazzy/install/setup.bash # ros2
source install/setup.bash # local src
ros2 service list
```
GUI control
```
rqt -> plugin -> Services -> Caller
rqt -> plugin -> Configuration -> Dynamic
rqt -> plugin -> Visualization -> TF tree
sudo apt install ros-$ROS_DISTRO-rqt-tf-tree
mv ~/.config/ros.org/rqt_gui.ini ~/.config/ros.org/rqt_gui.ini.bak
```
Keyboard control robot
```
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
```

## Building

```bash
cd ~/workspace
colcon build
source install/setup.bash
```

## Usage

### Turtle Patrol (C++)

1. Start turtlesim:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```
2. Run the control node:
   ```bash
   ros2 run demo_cpp_service turtle_control
   ```
3. Run the patrol client:
   ```bash
   ros2 run demo_cpp_service patrol_client
   ```
4. or Run with lauch script
   ```
   ros2 launch demo_cpp_service demo.launch.py
   ros2 launch demo_cpp_service demo.launch.py background_green:=255
   ros2 launch demo_cpp_service actions.launch.py rqt_startup:=True
   ```

### Face Detection (Python)

1. Run the face detection server:
   ```bash
   ros2 run demo_python_service face_detect_node
   ```
2. Run the client:
   ```bash
   ros2 run demo_python_service face_detect_client_node
   ```
3. Other
    ```bash
    ros2 run demo_python_service learn_face_detect
    ros2 service call /face_detect chapt4_interfaces/srv/FaceDetector
    ```

### Cordinate Transformation

1. base_link to base_laser
   ```
   ros2 run tf2_ros static_transform_publisher --x 0.1 --y 0.0 --z 0.2 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_link --child-frame-id base_laser
   ```
2. 发布base_laser到wall_point之间的变换：
   ```
   ros2 run tf2_ros static_transform_publisher --x 0.3 --y 0.0 --z 0.0 --roll 0.0 --pitch 0.0 --yaw 0.0 --frame-id base_laser --child-frame-id wall_point
   ```
3. 查询base_link到wall_point之间的关系：
   ```
   ros2 run tf2_ros tf2_echo base_link wall_point
   ros2 run tf2_tools view_frames
   ros2 topic list
   ros2 topic info /tf_static
   ros2 interface show tf2_msgs/msg/TFMessage
   ros2 topic echo /tf_static
   ```
4. 3D 可视化
   https://mrpt.github.io/webapp-demos/3d-rotation-converter.html
   ```
   sudo apt install ros-jazzy-mrpt-apps
   LIBGL_ALWAYS_SOFTWARE=1 3d-rotation-converter
   ```
5. python static tf
   ```
   ros2 run demo_python_tf static_tf_broadcaster
   ros2 topic info /tf_static -v
   ros2 run tf2_ros tf2_echo base_link camera_link
   ```
6. python dynamic tf
   ```
   ros2 run demo_python_tf dynamic_tf_broadcaster
   ros2 topic hz /tf
   ros2 run tf2_ros tf2_echo camera_link bottle_link
   ```
7. python tf listener
   ```
   ros2 run demo_python_tf tf_listener
   ```

###  Display robot model

1. Generate model relations
   ```
   urdf_to_graphviz first_robot.urdf
   xacro src/fishbot_description/urdf/first_robot.xacro
   xacro src/fishbot_description/urdf/fishbot.urdf.xacro
   ```
2. Install dependency for state tf messages
   ```
   sudo apt install ros-$ROS_DISTRO-joint-state-publisher
   sudo apt install ros-$ROS_DISTRO-robot-state-publisher
   sudo apt install ros-$ROS_DISTRO-xacro
   sudo apt install ros-${ROS_DISTRO}-ros-gz
   
   ```
3. Run Rviz2 to display model and receive state publish
   ```
   ros2 launch fishbot_description display_robot.launch.py
   ros2 launch fishbot_description display_robot.launch.py model:=install/fishbot_description/share/fishbot_description/urdf/fishbot.urdf.xacro
   ```
4. Run Gazebo to display robot model, using keyboard to control it
   ```
   ros2 launch fishbot_description gz_sim.launch.py
   ros2 topic list
   ros2 topic info /cmd_vel
   ros2 run teleop_twist_keyboard teleop_twist_keyboard  # run keyboard control
   rqt    # plugin tf-tree can show the topic relation
   rviz2  # show topic dynamic, like pointcloud2, odometry arraws
   ```
5. ros2-controller to simulate hardware
   ```
   ros2 launch fishbot_description gz_control.launch.py
   sudo apt install ros-$ROS_DISTRO-ros2-control
   sudo apt info    ros-$ROS_DISTRO-ros2-controllers
   sudo apt install ros-$ROS_DISTRO-ros2-controllers
   sudo apt install ros-$ROS_DISTRO-gz-ros2-control
   ros2 control list_hardware_interfaces
   ros2 control list_hardware_components
   ros2 control load_controller joint_state_broadcaster --set-state active
   ros2 topic echo /joint_states --once
   ros2 control list_controllers
   ros2 control unload_controller joint_state_broadcaster
   # effort control
   ros2 topic list -t |grep effort
   ros2 topic pub /effort_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.001, 0.001]}"
   ros2 control list_hardware_interfaces
   # diff wheel keyboard control
   ros2 param dump /controller_manager
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
   ```
6. ros2 navigation
   ```
   ros2 launch fishbot_navigation2 navigation2.launch.py use_sim_time:=True
   sudo apt install ros-jazzy-twist-mux # translate all cmd message to stamped
   sudo apt install ros-$ROS_DISTRO-slam-toolbox
   ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
   rviz2 #open map subscription, then scan the map
   sudo apt install ros-$ROS_DISTRO-nav2-map-server
   ros2 run nav2_map_server map_saver_cli -f room # save map in maps directroy 
   sudo apt install ros-$ROS_DISTRO-navigation2
   sudo apt install ros-$ROS_DISTRO-nav2-bringup
   cp /opt/ros/$ROS_DISTRO/share/nav2_bringup/params/nav2_params.yaml src/fishbot_navigation2/config/. # robot_radius: 0.12
   ros2 run fishbot_application init_robot_pose
   ros2 run fishbot_application get_robot_pose
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0} }}}" --feedback
   ros2 run fishbot_application nav_to_pose
   ros2 action info /follow_waypoints -t
   ros2 run fishbot_application waypoint_follower
   ```

