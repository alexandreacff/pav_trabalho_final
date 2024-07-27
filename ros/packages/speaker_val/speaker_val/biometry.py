import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool
import numpy as np
from collections import deque
import os
from sensor_msgs.msg import Image
import cv2
import torch
import nemo.collections.asr as nemo_asr
from speaker_val.titanet_utils import verify_speakers, generate_embeddings, make_testmanifest
import soundfile as sf
import librosa

class AudioProcessor(Node):

    def __init__(self):
        super().__init__('audio_processor')

        self.declare_parameter("buffer_duration", 2.0)  # seconds
        self.buffer_duration = self.get_parameter("buffer_duration").get_parameter_value().double_value
        self.sample_rate = 16000  # Assuming a sample rate of 16000 Hz
        self.buffer_size = int(self.sample_rate * self.buffer_duration)
        self.audio_buffer = deque(maxlen=self.buffer_size)

        self.model = nemo_asr.models.EncDecSpeakerLabelModel.from_pretrained("nvidia/speakerverification_en_titanet_large")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.declare_parameter("release", False)
        self.release = self.get_parameter(
            "release").get_parameter_value().bool_value
        
        if self.release:
            make_testmanifest()
            self.embs_allow = generate_embeddings(self.model, batch_size=4, device=self.device)
        else:
            # Load the embeddings
            package_dir = os.path.dirname(os.path.abspath(__file__))
            self.embs_allow = torch.load(os.path.join(package_dir, "config", "embs_allow.pt"))

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

        frame = np.zeros((840, 1720, 3), dtype=np.uint8)
        self.image = Image()
        self.image.header.stamp = Node.get_clock(self).now().to_msg()
        self.image.header.frame_id = 'ANI717'
        self.image.height = np.shape(frame)[0]
        self.image.width = np.shape(frame)[1]
        self.image.encoding = "bgr8"
        self.image.is_bigendian = False
        self.image.step = np.shape(frame)[2] * np.shape(frame)[1]
        self.image.data = frame.tobytes()
        self.image_publisher = self.create_publisher(
            Image,
            'Allow_image',
            10
        )

    def mic_callback(self, msg):
        data = np.array(msg.data)
        self.audio_buffer.extend(data)

    def wakeword_callback(self, msg):
        if msg.data:
            self.get_logger().info('Wake word detected. Processing buffered audio...')
            sf.write(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'recording.wav'), self.audio_buffer, 16000)
            self.process_audio()
        self.image_publisher.publish(self.image)

    def process_audio(self):
        buffered_audio = np.array(self.audio_buffer)
        energy = np.sum(buffered_audio ** 2)
        normalized_audio = buffered_audio / np.sqrt(energy)
        allowed, label = verify_speakers(self.model, self.embs_allow, normalized_audio)
        if allowed:
            self.get_logger().info('Speaker allowed')
            self.update_image(f"Speaker Allowed\nWelcome {label}", (0, 255, 0))
        else:
            self.get_logger().info('Speaker not allowed')
            self.update_image("Speaker not Allowed", (0, 0, 255))

    def update_image(self, text, color):

        # Create a blank image with the specified color
        img = np.full((840, 1720, 3), color, dtype=np.uint8)
        # Set the font, size, and color
        font = cv2.FONT_HERSHEY_TRIPLEX
        font_scale = 2
        font_color = (255, 255, 255)  # White text
        thickness = 2
        # Add the text to the image
        lines = text.split('\n')
        for i, line in enumerate(lines):

            # Get the text size to center it
            text_size = cv2.getTextSize(line, font, font_scale, thickness)[0]
            text_x = (img.shape[1] - text_size[0]) // 2
            text_y = (img.shape[0] + text_size[1]) // 2
            text_y = text_y + i * (cv2.getTextSize(line, font, font_scale, thickness)[0][1] + 30)  

            cv2.putText(img, line, (text_x, text_y), font, font_scale, font_color, thickness)
        # Flatten the image data and update the Image message
        self.image.data = img.tobytes()


def main(args=None):
    rclpy.init(args=args)
    audio_processor = AudioProcessor()
    rclpy.spin(audio_processor)
    audio_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()