# Copyright 2022 David Scripka. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Imports
import pyaudio
import numpy as np
from openwakeword.model import Model
import argparse
import soundfile as sf
from collections import deque

# Parse input arguments
parser=argparse.ArgumentParser()
parser.add_argument(
    "--chunk_size",
    help="How much audio (in number of samples) to predict on at once",
    type=int,
    default=1280,
    required=False
)
parser.add_argument(
    "--model_path",
    help="The path of a specific model to load",
    type=str,
    default="models/wk/hey_jarvis_v0.1.tflite",
    required=False
)
parser.add_argument(
    "--inference_framework",
    help="The inference framework to use (either 'onnx' or 'tflite'",
    type=str,
    default='tflite',
    required=False
)

args=parser.parse_args()

# Get microphone stream
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = args.chunk_size
p = pyaudio.PyAudio()
mic_stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK, input_device_index=8)

# Load pre-trained openwakeword models
if args.model_path != "":
    owwModel = Model(wakeword_models=[args.model_path], inference_framework=args.inference_framework)
else:
    owwModel = Model(inference_framework=args.inference_framework)

n_models = len(owwModel.models.keys())

# Run capture loop continuosly, checking for wakewords
if __name__ == "__main__":
    # Generate output string header
    print("\n\n")
    print("#"*100)
    print("Listening for wakewords...")
    print("#"*100)
    print("\n"*(n_models*3))
    mdl = list(owwModel.models.keys())[0]
    frames= deque(maxlen=30)

    try:
        while True:
            # Get audio
            audio = np.frombuffer(mic_stream.read(CHUNK, exception_on_overflow=False), dtype=np.int16)
            frames.append(audio)

            # Feed to openWakeWord model
            prediction = owwModel.predict(audio)

            # Column titles
            n_spaces = 16
            output_string_header = """
                Model Name         | Score | Wakeword Status
                --------------------------------------
                """
            
            scores = list(owwModel.prediction_buffer[mdl])
            
            curr_score = format(scores[-1], '.20f').replace("-", "")

            if scores[-1] <= 0.8:

                status = "--" + " " * 20

            else:
                if status != "Wakeword Detected!":
                    wk_audio = frames
                    print(len(wk_audio))
                    audio_data = np.concatenate(wk_audio)
                    sf.write('recording.wav', audio_data, RATE)
                status = "Wakeword Detected!"
                print(status)
                
                

            output_string_header += f"{mdl}{' '*(n_spaces - len(mdl))}   | {curr_score[0:5]} | {status}\n"

            # Print results table
            # print("\033[F"*(4*n_models+1))
            # print(output_string_header, "                             ", end='\r')
    except KeyboardInterrupt:
        print("Recording stopped.")
        mic_stream.stop_stream()
        mic_stream.close()
        p.terminate()
        print("Audio stream closed.")
        print("Saving audio file...")
        # audio_data = np.concatenate(wk_audio)
        # sf.write('recording.wav', audio_data, RATE)
        print("Audio file saved.")
        print("Exiting...")
