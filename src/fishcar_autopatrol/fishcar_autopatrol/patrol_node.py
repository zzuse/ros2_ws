import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import quaternion_from_euler
import math


class PatrolNode(BasicNavigator):
    def __init__(self, node_name='patrol_node'):
        super().__init__(node_name)
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])  # x, y, yaw
        self.declare_parameter('waypoints', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])  # List of waypoints as [x, y, yaw]
        self.initial_pose = self.get_parameter('initial_pose').value
        self.target_waypoints = self.get_parameter('waypoints').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)


    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        Create a PoseStamped message from x, y, and yaw values. 
        TODO: but lacks timestamp information.
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose


    def init_robot_pose(self):
        self.initial_pose = self.get_parameter('initial_pose').value
        init_pose = self.get_pose_by_xyyaw(*self.initial_pose)
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()


    def get_target_points(self):
        waypoints = []
        self.target_waypoints = self.get_parameter('waypoints').value
        for index in range(int(len(self.target_waypoints)/3)):
            x = self.target_waypoints[index * 3]
            y = self.target_waypoints[index * 3 + 1]
            yaw = self.target_waypoints[index * 3 + 2]
            pose = self.get_pose_by_xyyaw(x, y, yaw)
            waypoints.append(pose)
            self.get_logger().info(f"Waypoint {index}: x={x}, y={y}, yaw={yaw}")
        return waypoints


    def nav_to_pose(self, pose):
        self.goToPose(pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f"Distance remaining: {feedback.distance_remaining} meters")
            result = self.getResult()
            self.get_logger().info(f"Navigation result: {result}")

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            print("Navigation succeeded!")
        else:
            print("Navigation failed.")


    def get_current_pose(self):
        # Get the current pose of the robot
        while  rclpy.ok():
            try:
                result = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
                transform = result.transform
                self.get_logger().info(f"Current pose: x={transform.translation.x}, y={transform.translation.y}, z={transform.translation.z}")
                self.get_logger().info(f"Current orientation: x={transform.rotation.x}, y={transform.rotation.y}, z={transform.rotation.z}, w={transform.rotation.w}")
                return transform
            except Exception as e:
                self.get_logger().error(f"Error getting current pose: {str(e)}")


def main():
    rclpy.init()
    patrol_node = PatrolNode()
    patrol_node.init_robot_pose()

    while rclpy.ok():
        waypoints = patrol_node.get_target_points()
        for target_pose in waypoints:
            patrol_node.nav_to_pose(target_pose)

    rclpy.shutdown()
