import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector

import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory

from cv_bridge import CvBridge
import time

from rcl_interfaces.msg import SetParametersResult

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.get_logger().info('Face detection service is ready.')
        self.bridge = CvBridge()
        self.declare_parameter('number_of_times_to_unsample', 1)
        self.declare_parameter('model', "hog")
        self.number_of_times_to_unsample = 1
        self.model = 'hog'
        self.default_image_path = get_package_share_directory('demo_python_service') + "/resource/zidane.jpg"
        self.service = self.create_service(FaceDetector, 'face_detect', self.handle_face_detect)
        self.add_on_set_parameters_callback(self.parameter_callback)


    def parameter_callback(self, params):
        for param in params:
            if param.name == 'number_of_times_to_unsample' and param.type_ == rclpy.Parameter.Type.INTEGER:
                self.number_of_times_to_unsample = param.value
                self.get_logger().info(f'Updated number_of_times_to_unsample: {self.number_of_times_to_unsample}')
            elif param.name == 'model' and param.type_ == rclpy.Parameter.Type.STRING:
                self.model = param.value
                self.get_logger().info(f'Updated model: {self.model}')
        return SetParametersResult(successful=True)


    def handle_face_detect(self, request, response):
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image, "passthrough")
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info('No image provided in the request, using default image.')
        
        start_time = time.time()
        self.get_logger().info('Face Loaded... starting face detection...')
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.number_of_times_to_unsample, model=self.model)
        end_time = time.time()
        response.use_time=end_time - start_time
        response.number = len(face_locations)
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
        return response

def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()