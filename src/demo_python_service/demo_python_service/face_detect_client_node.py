import time
import face_recognition
import cv2
import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge


class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = get_package_share_directory('demo_python_service') + "/resource/bus.jpg"
        self.get_logger().info('Face detection client is started.')
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.default_image_path)


    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) == False:
            self.get_logger().info('Service not available, waiting again...')
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image, "passthrough")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        response = future.result()
        self.get_logger().info(f'Number of faces detected: {response.number}')
        self.show_response(response)


    def show_response(self, response):
        for i in range (response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 4)
        cv2.imshow('Face Detection Result', self.image)
        cv2.waitKey(0)


def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.send_request()
    # rclpy.spin(node) # No need to spin after send_request as it uses spin_until_future_complete and then exits or waits for key
    rclpy.shutdown()
