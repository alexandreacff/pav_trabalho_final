import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import numpy as np
from openwakeword.model import Model
from ament_index_python.packages import get_package_share_directory
import os


class WakeWord(Node):

    def __init__(self):
        self.data = None
        self.data_diff = None
        super().__init__('wake_word_detector')

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'microphone_data',
            self.detector_callback,
            10
        )

        # self.declare_parameter("engine", os.path.join(get_package_share_directory('yolo_pose'), 'config', 'yolov8n-pose.engine'))
        # engine = self.get_parameter(
        #     "engine").get_parameter_value().string_value
        print(os.getcwd())

        self.model = Model(wakeword_models=['src/wakeword/config/models/hey_jarvis_v0.1.onnx'], inference_framework='onnx')
        self.mdl = list(self.model.models.keys())[0]

    def detector_callback(self, msg):


        data =  np.array(msg.data)
        prediction = self.model.predict(data)
        # scores = list(self.model.prediction_buffer[self.mdl])
        # self.get_logger().info(f"{scores}")
        if scores[-1] > 0.8:
            self.get_logger().info(f"Wakeword Detected! - Score: {scores[-1]}")



def main(args=None):

    rclpy.init(args=args)
    wake_word_detector = WakeWord()
    rclpy.spin(wake_word_detector)
    wake_word_detector.destroy_node()
    rclpy.shutdown()

    



if __name__ == '__main__':
    main()
