from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Define the initial pose of the robot
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.orientation.x = 2.0
    goal_pose.pose.orientation.y = 1.0
    goal_pose.pose.orientation.w = 1.0

    # Set the initial pose of the robot
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'当前导航状态: {feedback.distance_remaining:.2f} m, {feedback.navigation_time:.2f} s')
    
    result = navigator.getResult()
    navigator.get_logger().info(f'导航结果: {result}')
