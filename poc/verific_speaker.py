import argparse
import nemo.collections.asr as nemo_asr
import torch
from nemo.utils import logging
import pyaudio
import numpy as np
import soundfile as sf
import os
import time

def init_stream(p):

    FORMAT = pyaudio.paInt16  # Formato dos dados de áudio (int16)
    CHANNELS = 1  # Número de canais (1 para mono, 2 para estéreo)
    RATE = 16000  # Taxa de amostragem
    CHUNK = 1280  # Número de frames no buffer 


    # Encontra o dispositivo de entrada padrão
    # device_index = p.get_default_input_device_info()['index']
    device_index = 8
    print(device_index)

    # Abre o stream
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    input_device_index=device_index,
                    frames_per_buffer=CHUNK)

    print("Gravando...")

    return stream, CHUNK


def verify_speakers(model, embs_allow, chunk, threshold=0.65):
    """
    Verify if two audio files are from the same speaker or not.

    Args:
        path2audio_file1: path to audio wav file of speaker 1
        path2audio_file2: path to audio wav file of speaker 2
        threshold: cosine similarity score used as a threshold to distinguish two embeddings (default = 0.7)

    Returns:
        True if both audio files are from same speaker, False otherwise
    """
    embs = embs_allow
    embs2 = model.get_embedding(chunk).squeeze()

    # Length Normalize
    for embs1 in embs:
        if isinstance(embs1, np.ndarray):
            embs1 = torch.from_numpy(embs1).to('cuda') 
    
        X = embs1 / torch.linalg.norm(embs1)
        Y = embs2 / torch.linalg.norm(embs2)
        # Score
        similarity_score = torch.dot(X, Y) / ((torch.dot(X, X) * torch.dot(Y, Y)) ** 0.5)
        similarity_score = (similarity_score + 1) / 2
        # Decision
        if similarity_score >= threshold:
            logging.info("Allowing the speaker")
            return True
    logging.info("Not allowing the speaker")
    return False

def generate_embeddings(model, batch_size, sample_rate=16000, device="cuda", json="data/enrollment_manifest.json"):

    # Get the embeddings
    embs, _, _, _ = model.batch_inference(json, batch_size=batch_size, sample_rate=sample_rate, device=device)

    # Save the embeddings
    torch.save(embs, "embs_allow.pt")

    return embs

def main():
    # Cria o parser para os argumentos da linha de comando
    parser = argparse.ArgumentParser(description='Verifica se dois arquivos de áudio são do mesmo locutor.')
    parser.add_argument('--release', action='store_true', help='Se presente, a lista de embeddings permitidos será atualizada.')

    # Analisa os argumentos da linha de comando
    args = parser.parse_args()

    # Load the model
    model = nemo_asr.models.EncDecSpeakerLabelModel.from_pretrained("nvidia/speakerverification_en_titanet_large")

    if args.release:
        embs_allow = generate_embeddings(model, batch_size=4)
    else:
        # Load the embeddings
        embs_allow = torch.load("embs_allow.pt")

    while True:
        if os.path.exists("recording.wav"):
            time.sleep(1)
            verify_speakers(model, embs_allow, "recording.wav")
            os.remove("recording.wav")


    
if __name__ == '__main__':
    main()

