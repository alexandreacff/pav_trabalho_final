import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool
import numpy as np
from openwakeword.model import Model
from ament_index_python.packages import get_package_share_directory
import os


class WakeWord(Node):

    def __init__(self):
        super().__init__('wake_word_detector')

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'microphone_data',
            self.detector_callback,
            10
        )

        self.publisher = self.create_publisher(Bool, 'wakeword_status', 10)

        self.declare_parameter("model", 'hey_jarvis_v0.1.onnx')
        model_path = os.path.join(get_package_share_directory('wakeword'), 'config', self.get_parameter(
            "model").get_parameter_value().string_value)
        self.get_logger().info(f"Model path: {model_path}")

        self.status = Bool()
        self.status.data = False

        self.model = Model(wakeword_models=[model_path], inference_framework=os.path.splitext(model_path)[1][1:])
        self.mdl = list(self.model.models.keys())[0]

    def detector_callback(self, msg):


        data =  np.array(msg.data)
        prediction = self.model.predict(data)
        scores = list(self.model.prediction_buffer[self.mdl])
        if scores[-1] > 0.8:
            self.status.data = True
        else:  
            self.status.data = False
        self.publisher.publish(self.status)



def main(args=None):

    rclpy.init(args=args)
    wake_word_detector = WakeWord()
    rclpy.spin(wake_word_detector)
    wake_word_detector.destroy_node()
    rclpy.shutdown()

    



if __name__ == '__main__':
    main()
