import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
import math


class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.lookup_transform)  # Check for transform at 1 Hz

    def lookup_transform(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'bottle_link', rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0))
            transform=t.transform
            translation = transform.translation
            self.get_logger().info(f'平移 from base_link to bottle_link: {translation}')
            rotation = transform.rotation
            self.get_logger().info(f'旋转 from base_link to bottle_link: {rotation}')
            euler = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            self.get_logger().info(f'欧拉角 from base_link to bottle_link: roll={math.degrees(euler[0])}, pitch={math.degrees(euler[1])}, yaw={math.degrees(euler[2])}')
            self.get_logger().info(f'Got transform from camera_link to bottle_link: translation={translation}, rotation={euler}')
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()