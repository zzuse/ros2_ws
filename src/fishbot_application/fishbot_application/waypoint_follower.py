from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    # Define the initial pose of the robot
    goal_poses = []
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.orientation.x = 2.0
    goal_pose.pose.orientation.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    goal_poses.append(goal_pose)

    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.pose.orientation.x = 0.0
    goal_pose1.pose.orientation.y = 1.0
    goal_pose1.pose.orientation.w = 1.0
    goal_poses.append(goal_pose1)

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.pose.orientation.x = 0.0
    goal_pose2.pose.orientation.y = 0.0
    goal_pose2.pose.orientation.w = 1.0
    goal_poses.append(goal_pose2)


    # Set the initial pose of the robot
    navigator.followWaypoints(goal_poses)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info(f'路点编号: {feedback.current_waypoint}')
    
    result = navigator.getResult()
    navigator.get_logger().info(f'导航结果: {result}')
