import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray
import numpy as np
from collections import deque
import soundfile as sf



class Recorder(Node):

    def __init__(self):
        self.data = None
        self.data_diff = None
        super().__init__('mic_recorder')

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'microphone_data',
            self.detector_callback,
            10
        )

        self.buffer = deque(maxlen=100)

    def detector_callback(self, msg):


        data =  np.array(msg.data)
        self.buffer.append(data)
        sf.write('recording.wav', np.concatenate(self.buffer), 16000)



def main(args=None):

    rclpy.init(args=args)
    mic_recorder = Recorder()
    rclpy.spin(mic_recorder)
    mic_recorder.destroy_node()
    rclpy.shutdown()

    



if __name__ == '__main__':
    main()
