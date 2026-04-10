import time
import face_recognition
import cv2
import rclpy
from rclpy.node import Node
from chapt4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = get_package_share_directory('demo_python_service') + "/resource/bus.jpg"
        self.get_logger().info('Face detection client is started.')
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.default_image_path)


    def call_set_parameters(self, parameters):
        set_parameters_client = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        while not set_parameters_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetParameters service not available, waiting again...')
        request = SetParameters.Request()
        request.parameters = parameters
        future = set_parameters_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is not None:
            self.get_logger().info('Parameters updated successfully.')
        else:
            self.get_logger().error('Failed to update parameters.')
        return response
    

    def update_model(self, model):
        param = Parameter()
        param.name = 'model'
        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_STRING
        param_value.string_value = model
        param.value = param_value
        parameters = [param]
        # parameters = [Parameter(name='model', value=model)] # or just one liner
        response = self.call_set_parameters(parameters)
        for result in response.results:
            if not result.successful:
                self.get_logger().error(f'Failed to update model: {result.reason}')
                return False
            else:
                self.get_logger().info(f'Model updated to: {model}')
        return True


    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) == False:
            self.get_logger().info('Service not available, waiting again...')
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image, "passthrough")
        future = self.client.call_async(request)
        # rclpy.spin_until_future_complete(self,future)
        def handle_response(future):
            response = future.result()
            self.get_logger().info(f'Number of faces detected: {response.number}, using time: {response.use_time:.4f} seconds')
            # self.show_response(response)
        future.add_done_callback(handle_response)


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
    node.update_model('cnn') # Update the model parameter before sending the request
    node.send_request()
    node.update_model('hog') # Update the model parameter again to test dynamic reconfiguration
    node.send_request()
    rclpy.spin(node) # No need to spin if send_request uses spin_until_future_complete and then exits
    rclpy.shutdown()
