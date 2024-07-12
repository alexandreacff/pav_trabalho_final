import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool
import numpy as np
from collections import deque
import time

class AudioProcessor(Node):

    def __init__(self):
        super().__init__('audio_processor')

        self.declare_parameter("buffer_duration", 5.0)  # seconds
        self.buffer_duration = self.get_parameter("buffer_duration").get_parameter_value().double_value
        self.sample_rate = 16000  # Assuming a sample rate of 16000 Hz
        self.buffer_size = int(self.sample_rate * self.buffer_duration)
        self.audio_buffer = deque(maxlen=self.buffer_size)

        self.mic_subscription = self.create_subscription(
            Int16MultiArray,
            'microphone_data',
            self.mic_callback,
            10
        )

        self.wakeword_subscription = self.create_subscription(
            Bool,
            'wakeword_status',
            self.wakeword_callback,
            10
        )

    def mic_callback(self, msg):
        data = np.array(msg.data)
        self.audio_buffer.extend(data)

    def wakeword_callback(self, msg):
        if msg.data:
            self.get_logger().info('Wake word detected. Processing buffered audio...')
            self.process_audio()

    def process_audio(self):
        buffered_audio = np.array(self.audio_buffer)
        # Perform inference on buffered_audio
        # For demonstration, we'll just print the buffer length
        self.get_logger().info(f'Buffered audio length: {len(buffered_audio)} samples')
        # Here you would normally call your inference model
        # e.g., result = your_model.predict(buffered_audio)
        # self.get_logger().info(f'Inference result: {result}')

def main(args=None):
    rclpy.init(args=args)
    audio_processor = AudioProcessor()
    rclpy.spin(audio_processor)
    audio_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()