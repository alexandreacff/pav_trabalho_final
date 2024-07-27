
import torch
import numpy as np
import os
import tempfile
import soundfile as sf 
import librosa
from glob import glob
import pandas as pd


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

    if isinstance(chunk, np.ndarray):
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tmpfile:
            sf.write(tmpfile.name, chunk, 16000)  # Assumindo uma taxa de amostragem de 16000Hz
            tmpfile.seek(0)  # Volta para o início do arquivo para leitura
            embs2 = model.get_embedding(tmpfile.name).squeeze()
    else:
        embs2 = model.get_embedding(chunk).squeeze()

    # Length Normalize
    for label, embs1 in embs.items():
        if isinstance(embs1, np.ndarray):
            embs1 = torch.from_numpy(embs1)
    
        X = embs1 / torch.linalg.norm(embs1)
        Y = embs2 / torch.linalg.norm(embs2)
        # Score
        similarity_score = torch.dot(X, Y) / ((torch.dot(X, X) * torch.dot(Y, Y)) ** 0.5)
        similarity_score = (similarity_score + 1) / 2
        # Decision
        if similarity_score >= threshold:
            return True, label
    return False, None

def generate_embeddings(model, batch_size, sample_rate=16000, device="cuda", json="enrollment_manifest.json"):

    # Get the package directory
    package_dir = os.path.dirname(os.path.abspath(__file__))

    # Get the embeddings
    embs, _, labels, _ = model.batch_inference(os.path.join(package_dir, "config", json), batch_size=batch_size, sample_rate=sample_rate, device=device)

    # Define the path to save the embeddings
    save_path = os.path.join(package_dir, "config", "embs_allow.pt")

    # Create a dictionary to store the embeddings
    embs_dict = {}

    # Add the embeddings to the dictionary
    for emb, label in zip(embs, labels):
        if label in embs_dict:
            embs_dict[label].append(emb)
        else:
            embs_dict[label] = [emb]

    mean_dict = {}
    for label, embeddings in embs_dict.items():
        mean_dict[label] = np.mean(embeddings, axis=0)

    # Save the embeddings
    torch.save(mean_dict, save_path)

    return mean_dict

def get_audio_duration(file_path):
    # Carrega o arquivo de áudio apenas para pegar a duração
    duration = librosa.get_duration(filename=file_path)
    return duration

def resample_audio_if_needed(file_path, target_sr=16000):

    # Carrega o arquivo de áudio para verificar a taxa de amostragem
    y, sr = librosa.load(file_path, sr=None)  # sr=None garante que a taxa de amostragem original seja mantida

    if sr != target_sr:
        print(f"Reamostrando {file_path} de {sr} Hz para {target_sr} Hz.")

        # Reamostra o áudio para a taxa de amostragem desejada
        y_resampled = librosa.resample(y, orig_sr=sr, target_sr=target_sr)
        
        # Salva o áudio reamostrado no mesmo caminho, substituindo o original
        sf.write(file_path, y_resampled, target_sr)
    else:
        print(f"Áudio {file_path} já está na taxa de amostragem {sr} Hz.")

def make_testmanifest():

    # Get the list of files in the current working directory
    package_dir = os.path.dirname(os.path.abspath(__file__))
    config_dir = os.path.join(package_dir, 'config')
    files = glob(os.path.join(config_dir, 'chunks', '**', '*.wav'), recursive=True)

    # if os.path.isfile(os.path.join(config_dir, 'enrollment_manifest.json')):
    #     df = pd.read_json(os.path.join(config_dir, 'enrollment_manifest.json'), lines=True)
    # else:
    df = pd.DataFrame(columns=['audio_filepath', 'offset' , 'duration', 'label'])

    for file in files:
        if not df['audio_filepath'].str.contains(file).any():
            duration = get_audio_duration(file)

            # Verifica e realiza a reamostragem se necessário antes de adicionar ao DataFrame
            resample_audio_if_needed(file, target_sr=16000)
            df = df._append({'audio_filepath': file, 'offset': 0, 'duration': duration, 'label': file.split("/")[-2]}, ignore_index=True)
    
    df.to_json(os.path.join(config_dir, 'enrollment_manifest.json'), orient='records', lines=True)
