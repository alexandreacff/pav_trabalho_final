import rclpy
from rclpy.node import Node
from audio_common_msgs.msg import AudioData
import sounddevice as sd
import numpy as np
import wave
import struct

import pvporcupine


class WakeWord(Node):
    def __init__(self):
        self.data = None
        self.data_diff = None
        super().__init__('microphone_subscriber')
        self.subscription = self.create_subscription(
            AudioData,
            'microphone_data',
            self.detector_callback,
            10
        )

        self.porcupine = pvporcupine.create(
            access_key='UeJUDsKeIbf5UQC9aMCCMcWhH3a4hogFRIA15KoD8XJa3e5TP5duyg==',
            keyword_paths=['/home/alexandre/teste/mic/mic_ws/src/miss_mic/resource/miss-piggy_en_linux_v2_2_0.ppn'],)

    def detector_callback(self, msg):
        # Convert received data to numpy array
        
        frame_lenght = 512
        self.data = np.frombuffer(msg.data, dtype=np.int16)
        # self.get_logger().info(f'{type(data)}')
        # self.data = self.data.tolist()
        if self.data_diff is not None:
            self.get_logger().info(f'{self.data_diff.shape[0]}')
            self.data = np.concatenate((self.data_diff, self.data))
        self.get_logger().info(f'{self.data.shape[0]}')
        num_frames = self.data.shape[0] // frame_lenght

        try:
            for frame in range(num_frames):
                # print(frame)
                # self.get_logger().info(f'result {frame, frame+512} | {self.data[512:1024]}')
                result = self.porcupine.process(self.data[frame*frame_lenght:(frame+1)* frame_lenght])

            if result >= 0:
                self.get_logger().info('Detected MISS PIGGY!')
                
        except KeyboardInterrupt:
            self.get_logger().info('Saving')

        if num_frames < 4:
            self.data_diff = self.data[1536:]
        else:
            self.data_diff = None
        # finally:
        #     if self.wf is not None:
        #         self.wf.close()
        # Specify the output file path and name
        

        # Create a wave file with the received audio data



def main(args=None):
    rclpy.init(args=args)
    microphone_subscriber = WakeWord()
    rclpy.spin(microphone_subscriber)
    microphone_subscriber.get_logger().info(f'Audio saved to {microphone_subscriber.output_file}')
    microphone_subscriber.destroy_node()
    rclpy.shutdown()
    microphone_subscriber.porcupine.delete()
    # microphone_subscriber.wf.close()

    



if __name__ == '__main__':
    main()
