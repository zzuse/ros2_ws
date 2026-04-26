import rclpy
from rclpy.node import Node
from autopatrol_interface.srv import SpeechText
import espeakng


class SpeakerNode(Node):
    def __init__(self):
        super().__init__('speaker')
        self.speech_service_ = self.create_service(SpeechText, 'speak', self.speak_callback)
        self.esng = espeakng.Speaker()
        self.esng.voice = 'zh'  # 设置中文语音
        self.esng.speed = 150  # 设置语速
        self.get_logger().info('SpeakerNode 已启动，等待语音合成请求...')

    def speak_callback(self, request, response):
        text = request.text
        self.get_logger().info(f'收到语音合成请求: "{text}"')
        try:
            self.esng.say(text)
            self.esng.wait()  # 等待语音播放完成
            response.result = True
            self.get_logger().info("语音合成成功")
        except Exception as e:
            response.result = False
            self.get_logger().error(f"语音合成失败: {str(e)}")
        return response


def main(args=None):
    rclpy.init(args=args)
    speaker_node = SpeakerNode()
    rclpy.spin(speaker_node)
    speaker_node.destroy_node()
    rclpy.shutdown()