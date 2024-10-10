# Trabalho Final Processamento de áudio e voz

Este repositório explora a aplicação de técnicas de inteligência artificial no processamento de áudio e voz, com ênfase na integração de palavras de ativação (wake words) e reconhecimento de falantes para garantir interações seguras em sistemas embarcados. O objetivo é aumentar a segurança em dispositivos ativados por voz, restringindo o uso a indivíduos autorizados.

## Requisitos

Lista dos requisitos de software e hardware para rodar o projeto.

- ROS Distribution: Foxy
- Sistema operacional: Ubuntu 20.04
- Dependências adicionais: Docker, git 

## Instalação

### 1. Clonar o repositório

```bash
git clone https://github.com/alexandreacff/pav_trabalho_final.git
cd pav_trabalho_final/ros
```

### 2. Iniciar container ROS

Utilize o `docker`:

```bash
docker run -it --name pav_container \
  --privileged  \
  --network host \
  --volume /dev:/dev \
  --runtime nvidia \
  --ulimit memlock=-1:-1   --ulimit stack=67108864:67108864 \
  --workdir /dev_ws/ \
  --volume $PWD/packages:/dev_ws/src/ \
  alexandreacff/yollov8-ros-foxy:tensorrt-ros 
```

### 3. Instalar requirements

```bash
apt update
apt-get install python3-pyaudio -y
pip install openwakeword==0.5.0

```

### 4. Fonte do setup.bash

Após compilar o workspace, rode o comando abaixo para adicionar os pacotes ao ambiente:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

## Como usar

### 1. Executar o nó

Inicie o nó que captura o mic com o seguinte comando:

```bash
ros2 run miss_mic mic
```

Inicie o nó de wakeword com o seguinte comando:

```bash
ros2 run wakeword inference
```

### 2. Visualizar tópicos

Pode verificar os topicos que estão sendo publicados.

```bash
ros2 topic list 
```

## Estrutura do Repositório

Explique brevemente a organização do repositório.

```
├── ROS/                    # Códigos ros e documentação docker
├── POC/                    # Scripts python feito para desenvolver a POC do trabalho
└── README.md               # Documentação
```
