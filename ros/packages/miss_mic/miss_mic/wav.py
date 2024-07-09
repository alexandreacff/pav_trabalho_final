import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import sounddevice as sd
import numpy as np
import wave
import struct


class MicrophoneSubscriber(Node):
    def __init__(self):
        self.data = None
        self.wf = None
        super().__init__('microphone_subscriber')
        self.subscription = self.create_subscription(
            AudioData,
            'microphone_data',
            self.audio_data_callback,
            10
        )
        self.output_file = 'src/miss_mic/output.wav'
        self.wf = wave.open(self.output_file, 'w')
        self.wf.setnchannels(1)
        self.wf.setsampwidth(2)
        self.wf.setframerate(16000)

    def audio_data_callback(self, msg):
        # Convert received data to numpy array

        self.data = np.frombuffer(msg.data, dtype=np.int16)
        # self.get_logger().info(f'{type(data)}')
        self.data = self.data.tolist()
        try:
            if self.wf is not None:
                self.get_logger().info(f'{type(self.wf)}')
                self.get_logger().info(f'{len(self.data)}')
                self.wf.writeframes(struct.pack("h" * len(self.data), *self.data))
        except KeyboardInterrupt:
            self.get_logger().info('Saving')
        # finally:
        #     if self.wf is not None:
        #         self.wf.close()
        # Specify the output file path and name
        

        # Create a wave file with the received audio data



def main(args=None):
    rclpy.init(args=args)
    microphone_subscriber = MicrophoneSubscriber()
    try:
        rclpy.spin(microphone_subscriber)
    except SystemExit:              
        rclpy.logging.get_logger("Quitting").info(f'Audio saved to {microphone_subscriber.output_file}')

    microphone_subscriber.destroy_node()
    rclpy.shutdown()
    microphone_subscriber.wf.close()
    



if __name__ == '__main__':
    main()
