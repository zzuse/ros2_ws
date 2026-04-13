# Chapt4 ROS 2 Workspace

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
rqt -> plugin -> Services -> Caller
rqt -> plugin -> Configuration -> Dynamic

## Building

```bash
cd ~/chapt4/chapt4_ws
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

## Recent Fixes
- **Turtle Collision Fix**: Updated `patrol_client.cpp` and `turtle_control.cpp` to ensure random coordinates stay within bounds (1.0-10.0) and improved turn-in-place logic to prevent wide arcs hitting walls.
