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
  --ulimit memlock=-1:-1   --ulimit stack=67108864:67108864
  --workdir /dev_ws/
  alexandreacff/yollov8-ros-foxy:tensorrt-ros 
```

### 3. Instalar requirements

```bash

```

### 4. Fonte do setup.bash

Após compilar o workspace, rode o comando abaixo para adicionar os pacotes ao ambiente:

```bash
source devel/setup.bash
```

## Como usar

### 1. Executar o nó

Inicie o nó principal com o seguinte comando:

```bash
roslaunch nome_do_pacote nome_do_arquivo.launch
```

### 2. Exemplos de execução

Descreva aqui como utilizar as funcionalidades do projeto.

```bash
rosrun nome_do_pacote nome_do_no
```

## Estrutura do Repositório

Explique brevemente a organização do repositório.

```
├── src/                    # Código fonte
├── launch/                 # Arquivos de launch
├── config/                 # Arquivos de configuração
└── README.md               # Documentação
```

## Testes

### Testes unitários

Instruções para executar os testes unitários:

```bash
rostest nome_do_pacote nome_do_teste.test
```
