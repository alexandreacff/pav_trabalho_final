import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray
import pyaudio
import numpy as np

class MicrophonePublisher(Node):
    def __init__(self):

        super().__init__('microphone_publisher')

        self.publisher_ = self.create_publisher(Int16MultiArray, 'microphone_data', 10)
        self.timer_ = self.create_timer(0.08, self.publish_microphone_data)

        # Specify the input device (example: device index 1)
        self.input_device = 8

        # Get microphone stream
        format = pyaudio.paInt16
        channels = 1
        rate = 16000
        self.chunk = 1280
        self.p = pyaudio.PyAudio()
        self.list_devices()

        self.mic_stream = self.p.open(format=format, channels=channels, rate=rate, input=True, frames_per_buffer=self.chunk, input_device_index=self.input_device)


        self.get_logger().info(f"Recording ...")

    def publish_microphone_data(self):

        # Get microphone data from the specified device
        data = np.frombuffer(self.mic_stream.read(self.chunk, exception_on_overflow=False), dtype=np.int16)

        # self.get_logger().info(f'shape: {data.shape}')
        # Create and publish AudioData message
        audio_msg = Int16MultiArray()
        audio_msg.data = data.tolist()
        self.publisher_.publish(audio_msg)

    def list_devices(self):
        self.get_logger().info("Dispositivos de entrada (microfones):")
        for i in range(self.p.get_device_count()):
            device_info = self.p.get_device_info_by_index(i)
            if device_info["maxInputChannels"] > 0:
                self.get_logger().info(f"ID: {device_info['index']}, Nome: {device_info['name']}")


def main(args=None):
    rclpy.init(args=args)
    microphone_publisher = MicrophonePublisher()
    try: 
        rclpy.spin(microphone_publisher)
    except KeyboardInterrupt:
        microphone_publisher.get_logger().info(f"Audio stream closed.")
        microphone_publisher.mic_stream.stop_stream()
        microphone_publisher.mic_stream.close()
        microphone_publisher.p.terminate() 
    finally:
        microphone_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
