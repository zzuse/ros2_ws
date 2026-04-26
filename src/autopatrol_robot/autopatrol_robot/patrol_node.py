import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from autopatrol_interface.srv import SpeechText
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import os


class PatrolNode(BasicNavigator):
    def __init__(self):
        super().__init__('patrol_node')
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])  # x, y, yaw
        self.declare_parameter('patrol_points', [0.0, 0.0, 0.0, 1.0, 1.0, 1.57])  # List of patrol points as [x, y, yaw]
        self.declare_parameter('img_save_path', '')
        self.initial_pose_ = self.get_parameter('initial_pose').get_parameter_value().double_array_value
        self.patrol_points_ = self.get_parameter('patrol_points').get_parameter_value().double_array_value
        self.img_save_path_ = self.get_parameter('img_save_path').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.speech_client = self.create_client(SpeechText, 'speak')
        self.cv_bridge = CvBridge()
        self.latest_image = None
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 1)


    def image_callback(self, msg):
        self.latest_image = msg
        self.get_logger().info('收到新的图像数据')


    def save_image(self, pose):
        # 确保处理最新的图像消息
        if self.latest_image is not None:
            try:
                cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                if isinstance(pose, (list, tuple)):
                    x, y = pose[0], pose[1]
                else:
                    x, y = pose.translation.x, pose.translation.y

                filename = f'patrol_point_{x:.2f}_{y:.2f}.png'
                if self.img_save_path_:
                    if not os.path.exists(self.img_save_path_):
                        os.makedirs(self.img_save_path_)
                    filename = os.path.join(self.img_save_path_, filename)

                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'图像已保存: {filename}')
            except Exception as e:
                self.get_logger().error(f'保存图像失败: {e}')
        else:
            self.get_logger().warn('没有可用的图像数据来保存')


    def get_pose_by_xyyaw(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id ='map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    

    def init_robot_pose(self):
        self.initial_pose_ = self.get_parameter('initial_pose').get_parameter_value().double_array_value
        initial_pose = self.get_pose_by_xyyaw(self.initial_pose_[0], self.initial_pose_[1], self.initial_pose_[2])
        self.setInitialPose(initial_pose)
        self.waitUntilNav2Active()

    
    def get_target_points(self):
        target_points = []
        self.patrol_points_ = self.get_parameter('patrol_points').get_parameter_value().double_array_value
        for i in range(0, len(self.patrol_points_), 3):
            x = self.patrol_points_[i]
            y = self.patrol_points_[i + 1]
            yaw = self.patrol_points_[i + 2]
            target_points.append([x, y, yaw])
            self.get_logger().info(f'巡逻点: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.2f}°')
        return target_points


    def patrol(self, point):

        self.goToPose(point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback is not None:
                self.get_logger().info(f'当前导航状态: {feedback.distance_remaining:.2f} m')
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('成功到达巡逻点')
        else:
            self.get_logger().warn('未能到达巡逻点')


    def get_current_pose(self):
        while rclpy.ok():
            try:
                t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0))
                transform = t.transform
                translation = transform.translation
                rotation = transform.rotation
                euler = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
                self.get_logger().info(f'当前位姿: x={translation.x:.2f}, y={translation.y:.2f}, yaw={math.degrees(euler[2]):.2f}°')
                return transform
            except Exception as e:
                self.get_logger().warn(f'无法获取当前位姿: {e}')


    def speech_text(self, text):
        while not self.speech_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待语音合成服务...')
        request = SpeechText.Request()
        request.text = text
        future = self.speech_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().result:
                self.get_logger().info('语音合成成功')
            else:
                self.get_logger().warn('语音合成失败')
        else:
            self.get_logger().warn(f'调用语音合成服务失败: {future.exception()}')


def main(args=None):
    rclpy.init(args=args)

    patrol = PatrolNode()
    patrol.speech_text('正在初始化巡逻机器人...')
    patrol.init_robot_pose()
    patrol.speech_text("巡逻开始")

    while rclpy.ok():
        points = patrol.get_target_points()
        for point in points:
            patrol.speech_text(f'正在前往巡逻点: x={point[0]:.2f}, y={point[1]:.2f}')
            patrol.patrol(patrol.get_pose_by_xyyaw(point[0], point[1], point[2]))
            patrol.speech_text(f'到达巡逻点记录图像')
            pose = patrol.get_current_pose()
            patrol.save_image(pose)
            patrol.speech_text(f'图像记录完成')
    
    rclpy.shutdown()