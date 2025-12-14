# 🤖 Projeto Robótica - Empilhadeira Autônoma

Este repositório contém o projeto completo de uma empilhadeira autônoma desenvolvido para a disciplina de Robótica da PUC-Rio no semestre 2025.2. O curso tem como objetivo projetar e construir um robô funcional capaz de operar de forma autônoma e remota.

## 👥 Equipe

- Eduardo Eugênio de Souza
- Enzo Mediano
- Luisa Silveira
- Luiz Felipe Neves Batista

## 🎯 Objetivo do Projeto

O desafio proposto foi desenvolver uma empilhadeira autônoma com os seguintes requisitos:

- **Interface remota** para controle manual e controle de missão
- **Carga máxima**: 1 Kg
- **Altura máxima** do levantamento do garfo: 15 cm
- **Leitura de AprilTags** para localização no terreno e identificação de pallets
- **Alimentação** por baterias 18650

## 🏗️ Estrutura do Projeto

O projeto está organizado em quatro componentes principais:

### 1. Firmware do ESP32 (`src/`)

Código embarcado rodando no microcontrolador ESP32-S3 DevKitC-1.

### 2. Backend Python (`ui/`)

Servidor intermediário responsável pelo processamento de visão computacional e controle autônomo.

### 3. Frontend Web (`ui-frontend/`)

Interface de usuário moderna desenvolvida em React + TypeScript para controle e monitoramento.

### 4. Modelagem 3D (`Modelagem3D/`)

Peças estruturais do robô projetadas e impressas em 3D.

---

## 💻 Sistema de Software

### 🔧 Firmware ESP32 (C++)

O firmware foi desenvolvido em C++ usando o framework Arduino para ESP32. Principais funcionalidades implementadas:

#### **Comunicação WiFi e WebSocket**

- Ponto de acesso WiFi (`ESP32_Robot_AP`) para conexão direta com o robô
- Servidor WebSocket na porta 8000 para comunicação bidirecionional em tempo real
- Transmissão de telemetria a 20Hz (50ms de período)
- Sistema de heartbeat para failsafe automático

#### **Controle dos Motores de Locomoção**

- Controle diferencial de duas rodas independentes
- **Encoders** para feedback de posição e velocidade (64 pulsos por rotação)
- **Controlador PID** otimizado com:
  - Feedforward para compensação de velocidade desejada
  - Termo proporcional (kp) para correção de erro
  - Termo integral (ki) com anti-windup
  - Constante estática (ks) para vencer atrito inicial
  - **Dithering** (vibração de alta frequência) para reduzir zona morta e stiction
- **Rampa de aceleração** para evitar movimentos bruscos
- **Filtro passa-baixas** exponencial para suavização da leitura de velocidade
- Loop de controle de alta precisão a 33Hz usando timer de hardware (`esp_timer`)
- Tratamento especial para detecção de parada (timeout de encoder)

#### **Sistema de Câmera**

- Câmera ESP32-CAM integrada (resolução 320x240)
- Streaming de vídeo em JPEG via WebSocket
- Taxa de transmissão configurável (padrão: 5fps)
- Buffer otimizado para evitar travamentos

#### **Thread Safety**

- Uso de `portMUX_TYPE` para proteção de variáveis compartilhadas
- Seções críticas ISR-safe (`portENTER_CRITICAL_ISR`)
- Separação clara entre leitura de comandos e escrita de telemetria

**Arquivos principais:**

- [main.cpp](src/main.cpp) - Loop principal e coordenação de sistemas
- [camera_pins.h](src/camera_pins.h) - Definição de pinos da câmera

---

### 🐍 Backend Python (FastAPI)

O servidor backend em Python atua como intermediário inteligente entre a interface do usuário e o robô. Principais componentes:

#### **Processamento de Visão Computacional**

- **Detecção de AprilTags** usando a biblioteca `pupil_apriltags`
- Decodificação eficiente de JPEG usando `TurboJPEG`
- Calibração de câmera (parâmetros intrínsecos: fx, fy, cx, cy)
- Cálculo de pose 3D das tags detectadas
- Transformação de coordenadas 3D→2D para navegação planar
- Mapeamento de tags para posições globais no ambiente

#### **Odometria e Localização**

- Cálculo de pose do robô (x, y, θ) baseado em AprilTags
- Integração de dados de encoder para odometria
- Estimativa de posição em tempo real
- Cálculo de distância percorrida por cada roda

#### **Sistema de Navegação Autônoma**

- Controle de missões com waypoints
- Algoritmo de navegação diferencial
- **Controlador angular** proporcional para alinhamento
- **Limitação de velocidade angular** para estabilidade
- Tolerância de chegada configurável (5cm, 0.5rad)
- Estados de missão: idle, running, completed, error

#### **Comunicação Multi-Cliente**

- Servidor FastAPI com endpoints REST e WebSocket
- WebSocket dedicado para comunicação com ESP32
- WebSocket broadcast para dashboards (múltiplos clientes)
- Heartbeat automático a 5Hz para manter conexão
- CORS habilitado para desenvolvimento

#### **Streaming de Dados**

- Broadcast de estado do robô (pose, velocidades, debug PID)
- Snapshot de detecções de visão
- Gráficos em tempo real de telemetria
- Histórico de comandos e respostas

**Arquivos principais:**

- [main.py](ui/main.py) - Servidor principal e lógica de controle
- [gabriel_client.py](ui/gabriel_client.py) - Cliente simples para testes
- [calibrate_camera.py](ui/calibrate_camera.py) - Calibração da câmera

---

### ⚛️ Frontend Web (React + TypeScript)

Interface moderna e responsiva desenvolvida com React 18, TypeScript e Chakra UI:

#### **Controle Manual**

- Botões direcionais (W, A, S, D) para movimentação
- Controle via teclado com detecção de teclas
- Botão de parada de emergência
- Feedback visual do estado de conexão

#### **Visualização de Câmera**

- Stream de vídeo em tempo real do robô
- Overlay de detecções de AprilTags
- Visualização de pose estimada
- Indicadores de distância e ângulo das tags

#### **Painel de Telemetria**

- Gráficos em tempo real das velocidades (target vs medida)
- Visualização dos componentes do PID (P, I, Feedforward)
- Monitoramento de PWM aplicado
- Gráficos independentes para roda esquerda e direita
- Histórico de até 600 pontos (30 segundos a 20Hz)

#### **Controle de Missões**

- Interface para definir waypoints (x, y)
- Controle de velocidade de navegação
- Indicador de estado da missão
- Visualização da pose atual do robô

#### **Monitoramento**

- Badge de status de conexão WebSocket
- Indicadores de latência
- Timestamp da última atualização
- Valores de encoder em tempo real

**Tecnologias utilizadas:**

- React 18 + TypeScript
- Chakra UI para componentes
- Vite para build otimizado
- Canvas API para gráficos customizados
- WebSocket API nativa

**Arquivos principais:**

- [App.tsx](ui-frontend/src/App.tsx) - Componente principal
- [CameraFeed.tsx](ui-frontend/src/components/CameraFeed.tsx) - Visualização de vídeo
- [types.ts](ui-frontend/src/types.ts) - Definições de tipos

---

## ⚙️ Configuração e Execução

### ESP32 Firmware

```bash
# Instalar PlatformIO
pip install platformio

# Compilar e fazer upload
cd Projeto-Robotica
pio run --target upload
```

### Backend Python

```bash
cd ui
pip install -e .
python main.py
```

### Frontend

```bash
cd ui-frontend
pnpm install
pnpm dev
```

---

## 🎮 Modos de Operação

### 1. Modo Manual

- Controle direto via interface web ou cliente Python
- Comandos WASD para movimento
- Feedback em tempo real de telemetria

### 2. Modo Autônomo

- Definição de waypoints via interface
- Navegação automática com correção de trajetória
- Localização baseada em AprilTags
- Parada automática ao atingir objetivo

---

## 📊 Características Técnicas do Software

### Controle PID Otimizado

- **Frequência**: 33Hz (30ms de período)
- **Dithering**: 25Hz para redução de stiction
- **Anti-windup**: Clamp do integrador em ±6.0
- **Rampa**: Aceleração máxima de 0.4 m/s²
- **Filtro**: α = 0.1 para suavização exponencial

### Comunicação

- **Protocolo**: WebSocket (texto JSON + binário)
- **Telemetria**: 20Hz
- **Vídeo**: 5fps (320x240 JPEG)
- **Heartbeat**: 5Hz com timeout de 3s

### Visão Computacional

- **Tags**: Família AprilTag 36h11
- **Tamanho**: 12cm (físico)
- **Calibração**: Fx=298.3, Fy=306.9, Cx=158.8, Cy=121.7
- **Detecção**: Tempo real (<50ms por frame)

---

## ⚠️ Limitações e Trabalho Futuro

### ❌ O que não foi implementado

#### **Sistema de Elevação do Garfo**

Infelizmente, não tivemos tempo de implementar o código de controle para o motor de elevação do garfo. A **parte mecânica e eletrônica estava completa**, incluindo:

- Motor DC com encoder
- Sistema de polias e guias impressas em 3D
- Driver de motor conectado ao ESP32
- Sensor de fim de curso (opcional)

**O que faltou:** Implementação do código de controle no firmware, incluindo:

- Interface de comandos via WebSocket
- Controle de posição ou velocidade
- Limites de segurança (altura mínima/máxima)
- Integração com a interface gráfica

### 🚧 Desafios Encontrados

Durante o desenvolvimento, enfrentamos alguns desafios importantes que impactaram o desempenho do robô:

1. **Atrito excessivo no piso**

   - O tipo de piso utilizado nos testes apresentava muito atrito
   - Isso dificultou o movimento suave do robô
   - Exigiu ajustes constantes nos parâmetros do PID
   - Afetou a precisão da odometria

2. **Problemas mecânicos nas rodas**

   - As rodas não giravam perfeitamente livre
   - Havia resistência mecânica além do esperado
   - Possível desalinhamento ou fricção nos eixos
   - Zona morta maior que o ideal no sistema de acionamento

3. **Navegação autônoma imprecisa**
   - A navegação para pontos específicos não estava 100% precisa
   - Desvios acumulativos devido ao atrito e problemas mecânicos
   - Dificuldade em manter trajetória reta por longas distâncias
   - Necessidade de ajuste fino dos ganhos do controlador
   - Limitações na calibração da câmera e detecção de AprilTags em condições de iluminação variável

Estes problemas mecânicos e de atrito impactaram significativamente a performance do sistema autônomo, demonstrando a importância da integração mecânica, eletrônica e de software em projetos de robótica.

### ✅ O que foi concluído com sucesso

1. **Sistema de locomoção**

   - Motores DC com controle PID implementado
   - Odometria funcional (com limitações pelo atrito)
   - Controle diferencial básico operacional

2. **Comunicação WiFi/WebSocket**

   - Conexão estável e de baixa latência
   - Protocolo bem definido e extensível

3. **Visão computacional**

   - Detecção confiável de AprilTags
   - Cálculo de pose implementado
   - Streaming de vídeo funcional

4. **Interface de usuário**

   - Interface moderna e intuitiva
   - Telemetria em tempo real
   - Gráficos de debug detalhados

5. **Navegação autônoma básica**
   - Waypoint navigation implementada
   - Correção de trajetória (com margem para melhorias)
   - Sistema de missões funcional

### 🔮 Melhorias Futuras

- Implementar controle do garfo de elevação
- Adicionar sensor ultrassônico para detecção de obstáculos
- Implementar SLAM para mapeamento do ambiente
- Melhorar odometria com fusão de sensores (IMU)
- Adicionar planejamento de trajetória (A\*, RRT)
- Implementar pickup/place automático de pallets
- Adicionar sistema de filas de missões

---

## 📁 Estrutura de Diretórios

```
Projeto-Robotica/
├── src/                    # Firmware ESP32 (C++)
│   ├── main.cpp           # Código principal
│   ├── camera_pins.h      # Configuração da câmera
│   ├── david_code.cpp     # Experimentos
│   └── gabriel_code.cpp   # Experimentos
├── ui/                     # Backend Python
│   ├── main.py            # Servidor FastAPI
│   ├── gabriel_client.py  # Cliente de teste
│   ├── calibrate_camera.py # Calibração
│   └── fake_robot.py      # Simulador
├── ui-frontend/           # Frontend React
│   ├── src/
│   │   ├── App.tsx        # Componente principal
│   │   ├── components/    # Componentes React
│   │   └── types.ts       # Definições TypeScript
│   └── package.json
├── Modelagem3D/           # Peças 3D (STL)
│   ├── Guia + Garfo - *.stl
│   ├── Case_Baterias.stl
│   └── ...
├── mosquitto/             # Configuração MQTT (não usado)
├── platformio.ini         # Configuração ESP32
└── README.md
```

---

## 🛠️ Dependências Principais

### ESP32

- Arduino Framework
- ArduinoJson (^7.0.4)
- ESP32 Camera (^2.0.4)
- WebSockets (^2.7.1)

### Python Backend

- FastAPI
- Uvicorn
- Websockets
- OpenCV (cv2)
- NumPy
- pupil-apriltags
- TurboJPEG

### Frontend

- React 18
- TypeScript
- Chakra UI
- Vite
- Lucide Icons

---

## 📝 Licença

Este projeto foi desenvolvido para fins educacionais como parte da disciplina de Robótica da PUC-Rio.

---

## 🙏 Agradecimentos

Agradecemos aos professores da disciplina de Robótica pelo suporte e orientação durante o desenvolvimento deste projeto desafiador.
