import os
from glob import glob
import pandas as pd
import librosa
import soundfile as sf

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
    files = glob(os.path.join( '**', '*.wav'), recursive=True)
    print(files)

    if os.path.isfile('enrollment_manifest.json'):
        df = pd.read_json('enrollment_manifest.json', lines=True)
    else:
        df = pd.DataFrame(columns=['audio_filepath', 'offset' , 'duration', 'label'])

    for file in files:
        if not df['audio_filepath'].str.contains(file).any():
            duration = get_audio_duration(file)

            # Verifica e realiza a reamostragem se necessário antes de adicionar ao DataFrame
            resample_audio_if_needed(file, target_sr=16000)
            print(file)

            df = df._append({'audio_filepath': file, 'offset': 0, 'duration': duration, 'label': 'alexandre'}, ignore_index=True)
    
    df.to_json('enrollment_manifest.json', orient='records', lines=True)

if __name__ == '__main__':
    os.chdir('data')
    make_testmanifest()