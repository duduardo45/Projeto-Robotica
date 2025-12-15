# 🤖 Projeto Robótica - Empilhadeira Autônoma

Este repositório contém o projeto completo de uma empilhadeira autônoma desenvolvido para a disciplina de Robótica da PUC-Rio no semestre 2025.2. O curso tem como objetivo projetar e construir um robô funcional capaz de operar de forma autônoma e remota.

## 👥 Equipe

- Eduardo Eugênio de Souza
- Enzo Mediano
- Luísa Ferreira da Silveira
- Luiz Felipe Neves Batista

## 🎯 Objetivo do Projeto

O desafio proposto foi desenvolver uma empilhadeira autônoma com os seguintes requisitos:

- **Interface remota** para controle manual e controle de missão
- **Carga máxima**: 1 Kg
- **Altura máxima** do levantamento do garfo: 15 cm
- **Leitura de AprilTags** para localização no terreno e identificação de pallets
- **Alimentação** por baterias 18650

## Arquitetura do Sistema

O projeto foi desenvolvido seguindo uma abordagem integrada de hardware e software, dividido em 5 soluções principais:

---

## 1️⃣ Solução Mecânica

### Projeto e Construção

Todas as peças estruturais foram projetadas em CAD e impressas em 3D. Os arquivos STL estão disponíveis na pasta [Modelagem3D](Modelagem3D/).

#### **Componentes Mecânicos Projetados:**

**Garfo e Sistema de Elevação:**

- [Guia + Garfo - Part 1](<Modelagem3D/Guia%20+%20Garfo%20-%20Part%201%20(4).stl>) - Estrutura principal do garfo
- [Guia + Garfo - Part 2](<Modelagem3D/Guia%20+%20Garfo%20-%20Part%202%20(3).stl>) - Componente secundário do garfo
- [Guia + Garfo - Part 4](Modelagem3D/Guia%20+%20Garfo%20-%20Part%204.stl) - Suporte adicional
- Guia vertical de **15 cm** para elevação do garfo
- Sistema de polias para transmissão ([Polia - Part 1](Modelagem3D/Polia%20-%20Part%201.stl))

**Estrutura e Suportes:**

- [Case Baterias](Modelagem3D/Case_Baterias%20-%20Part%201.stl) - Compartimento para baterias 18650
- [Case Contrapeso](Modelagem3D/Case-Contrapeso%20-%20Part%202.stl) - Peso para balanceamento
- [Suporte ESP32 - Part 1](<Modelagem3D/Suporte%20esp32%20-%20Part%201%20(1).stl>) e [Part 2](<Modelagem3D/Suporte%20esp32%20-%20Part%202%20(1).stl>) - Fixação do microcontrolador
- [Roof Holder - Parts 1-4](Modelagem3D/) - Suportes para o teto do robô
- [Suporte Motor Elevação](Modelagem3D/suporte%20motor%20elevação%20-%20Part%201.stl) - Base do motor de elevação

#### **Modelagens Não Utilizadas:**

Algumas peças foram modeladas mas não foram utilizadas na montagem final:

- [Battery Support (Suporte de Bateria)](Modelagem3D/Battery%20Container%20Support.stl) - Não foi utilizado pois se tornou redundante, já que o compartimento de baterias conseguia ser travado através do teto (roof holder) e havia a necessidade de passagem de fios pela parte inferior do suporte.
- [Motor Holder (Suporte do Motor)](Modelagem3D/Motor%20Holder.stl) - Não houve tempo hábil para instalar o componente.
- [Polia](Modelagem3D/Polia%20-%20Part%201.stl) - A polia acabou não sendo utilizada, preferindo conectar o fio que levanta o garfo diretamente no motor.

#### **Limitações de Tempo na Montagem:**

1. **Motor Elevation Base** - Não foi impresso devido ao tempo limitado do projeto
2. **Roof (teto completo)** - Foi impresso, mas não montado. Esta peça seguraria o motor de elevação que ficaria em cima do roof holder. Como não foi implementada, o motor de elevação permaneceu dentro do chassis do robô
3. **Organização de fiação** - Como a câmera utilizada era integrada ao ESP32 e ele ficou posicionado no alto (em cima da guia), muitos fios ficaram expostos e desorganizados, comprometendo a estética do projeto. Com mais tempo, seria implementada uma calha para passagem de fios na guia
4. **Polia** - Preferimos conectar os fios diretamente ao motor de elevação, sem utilizar a polia impressa. Isso simplificou a montagem e evitou possíveis problemas mecânicos"

### Montagem Final

O robô foi montado com:

- Garfo móvel com guia de elevação de 15cm
- sistema de passagem dos fios que puxam o garfo
- Compartimento de baterias integrado
- Teto para o chassis (roof holder)
- Contrapeso para balanceamento
- ESP32 com câmera posicionado na parte superior

---

## 2️⃣ Solução Eletrônica

### Componentes Utilizados

#### **Microcontrolador:**

- **ESP32-S3 DevKitC-1** (16MB Flash, PSRAM)
  - WiFi integrado para comunicação
  - Câmera OV2640 integrada (320x240)
  - 2x Timer hardware para controle preciso
  - Múltiplos canais PWM

#### **Sistema de Alimentação:**

- **3x Baterias 18650** em série → **12V total**
  - **12V direto** para os motores DC
  - ⚠️ **BMS não implementado** - Por limitação de tempo, não foi utilizado Battery Management System. Isso seria uma melhoria importante para segurança e longevidade das baterias

#### **Motorização:**

- **2x Motores DC com encoder** (locomoção)
  - 64 pulsos por rotação
  - Alimentação: 12V
  - Controle via Ponte H
- **1x Motor DC para elevação do garfo**
  - Hardware montado e conectado
  - ⚠️ Software não implementado

#### **Drivers e Interfaces:**

- **Ponte H (H-Bridge)** para controle bidirecional dos motores
  - Controle de direção (pinos IN1, IN2)
  - Controle de velocidade via PWM
  - Duas Ponte H (uma para motores de locomoção e uma para o de elevação)
- **Encoders ópticos** conectados via interrupção

## 3️⃣ Solução de Comunicação

### Arquitetura de Comunicação

O sistema utiliza uma arquitetura de três camadas:

```
[Frontend Web] ↔ [Backend Python] ↔ [ESP32]
   (React)      WebSocket/HTTP      WebSocket
```

### Protocolo de Comunicação

#### **ESP32 ↔ Backend Python:**

- **WebSocket** na porta 8000
- **Formato:** JSON (comandos e telemetria) + Binário (imagens)
- **Taxa de telemetria:** 20Hz (50ms)
- **Taxa de vídeo:** 5fps (200ms)

#### **Backend ↔ Frontend:**

- **WebSocket** para dados em tempo real
- **REST API** para comandos pontuais
- **Broadcast** de estado para múltiplos clientes

### Mecanismos de Fail-Safe

1. **Heartbeat System:**

   - Cliente deve enviar heartbeat a cada 200ms
   - Timeout de 3 segundos sem heartbeat → parada automática dos motores
   - Implementado via timestamp no ESP32

2. **Watchdog de Conexão:**

   - Detecção de desconexão WebSocket
   - Parada segura em caso de perda de comunicação

3. **Validação de Comandos:**
   - Limites de velocidade no firmware
   - Validação de JSON no backend

### Interface de Controle

- **Interface Web moderna** (React + TypeScript)
- **Controle manual** via teclado (WASD)
- **Controle autônomo** via waypoints
- **Visualização em tempo real** de câmera e telemetria
- **Gráficos de debug** para ajuste de PID

---

## 4️⃣ Solução de Controle

### Controle em Malha Aberta vs Malha Fechada

O sistema implementa controle em **malha fechada** para os motores de locomoção, utilizando feedback dos encoders para correção contínua.

### 🎯 Controlador PID

O firmware implementa um controlador PID otimizado para cada roda:

#### **Componentes do Controlador:**

1. **Termo Proporcional (P):** `kp × erro`

   - Correção proporcional ao erro de velocidade
   - kp = 40.0 (ajustado experimentalmente)

2. **Termo Integral (I):** `ki × ∫erro·dt`

   - Elimina erro em regime permanente
   - ki = 30.0
   - **Anti-windup:** Limita integrador a ±6.0 para evitar saturação

3. **Feedforward (F):** `kf × velocidade_desejada`

   - Compensação antecipada da velocidade
   - kf = 10.0

4. **Termo Estático (S):** `ks`

   - Constante para vencer atrito estático inicial
   - ks = 120.0

5. **Dithering:**
   - Vibração de 25Hz (±70 PWM) para reduzir zona morta
   - Elimina stiction e melhora resposta em baixas velocidades

#### **Características do Sistema:**

- **Frequência de controle:** 33Hz (30ms de período)
- **Rampa de aceleração:** 0.4 m/s² máximo
- **Filtro passa-baixas:** α = 0.1 (suavização exponencial)
- **Encoders:** 64 pulsos/rotação
- **Thread-safe:** Mutex para proteção de variáveis compartilhadas

### 🗺️ Navegação e Localização

#### **Detecção de AprilTags:**

- Biblioteca `pupil_apriltags` para detecção
- Família de tags: 36h11
- Tamanho físico: 12cm
- Calibração de câmera: fx=298.3, fy=306.9, cx=158.8, cy=121.7

#### **Localização:**

- Cálculo de pose (x, y, θ) baseado em AprilTags detectadas
- Transformação 3D → 2D para navegação planar
- Fusão com odometria dos encoders

#### **Navegação Autônoma:**

- Waypoint navigation com correção de trajetória
- Controlador angular proporcional
- Tolerância: 5cm (posição), 0.5rad (orientação)
- Estados: idle, running, completed, error

---

## 5️⃣ Montagem Geral

### Processo de Montagem

A montagem do robô seguiu as seguintes etapas:

#### **1. Impressão 3D das Peças**

Todas as peças foram impressas utilizando os arquivos STL disponíveis:

- Estrutura do garfo e guias
- Compartimentos para eletrônica
- Suportes e fixações

#### **2. Montagem da Estrutura Base**

- Montagem do roof holder do robô
- Montagem do compartimento de baterias
- Instalação do contrapeso para balanceamento

#### **3. Montagem do Sistema de Elevação**

- Instalação da guia vertical (15cm)
- Fixação do garfo móvel
- Montagem do sistema onde passam os fios
- Conexão do motor de elevação (hardware pronto)

#### **4. Integração Eletrônica**

- Instalação do ESP32 no suporte superior
- Conexão da Ponte H aos motores
- Fiação das baterias (3x em série = 12V)
- Conexão dos encoders

#### **5. Limitações da Montagem**

**Problemas Estéticos:**

- Como a câmera é integrada ao ESP32 e foi posicionada no alto (em cima da guia), muitos fios ficaram expostos e desorganizados
- Isso comprometeu a aparência final do projeto
- **Solução planejada (não implementada):** Calha para passagem de fios na guia

**Componentes não Montados:**

- **Motor Elevation Base:** Não foi impresso por falta de tempo
- **Roof completo:** Foi impresso mas não montado. Esta peça seguraria o motor de elevação que ficaria em cima do roof holder
- Como resultado, o motor de elevação permaneceu dentro do chassis

**Sistema de Alimentação:**

- BMS (Battery Management System) não foi implementado
- Seria importante para segurança e longevidade das baterias
- Configuração atual: 3 baterias 18650 em série (12V total)

### Resultado Final

O robô foi montado com:

- ✅ Chassis estrutural completo
- ✅ Sistema de locomoção com 2 motores + encoders
- ✅ Garfo com guia de 15cm
- ✅ Sistema de passagem do fios
- ✅ ESP32 com câmera integrada
- ✅ Ponte H para controle dos motores
- ✅ Sistema de baterias (3x 18650 em série)
- ⚠️ Fiação exposta (sem calha organizadora)
- ⚠️ Motor de elevação sem software
- ⚠️ Roof não montado completamente

---

## 💻 Detalhes de Implementação de Software

### 🔧 Firmware ESP32 (C++)

Desenvolvido em C++ usando o framework Arduino para ESP32.

**Arquivos principais:**

- [main.cpp](src/main.cpp) - Loop principal e coordenação de sistemas
- [camera_pins.h](src/camera_pins.h) - Definição de pinos da câmera

### 🐍 Backend Python (FastAPI)

O servidor backend em Python atua como intermediário inteligente entre a interface do usuário e o robô.

**Arquivos principais:**

- [main.py](ui/main.py) - Servidor principal e lógica de controle
- [gabriel_client.py](ui/gabriel_client.py) - Cliente simples para testes
- [calibrate_camera.py](ui/calibrate_camera.py) - Calibração da câmera
- [fake_robot.py](ui/fake_robot.py) - Simulador para testes
- [simple_server.py](ui/simple_server.py) - Servidor simplificado

### ⚛️ Frontend Web (React + TypeScript)

Interface moderna e responsiva desenvolvida com React 18, TypeScript e Chakra UI.

**Funcionalidades:**

- Controle manual (WASD) e autônomo (waypoints)
- Visualização de câmera em tempo real
- Gráficos de telemetria (velocidade, PID, PWM)
- Monitoramento de AprilTags
- Dashboard de estado do robô

**Arquivos principais:**

- [App.tsx](ui-frontend/src/App.tsx) - Componente principal
- [CameraFeed.tsx](ui-frontend/src/components/CameraFeed.tsx) - Visualização de vídeo
- [types.ts](ui-frontend/src/types.ts) - Definições de tipos
- [theme.ts](ui-frontend/src/theme.ts) - Configuração de tema

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

## ⚠️ Limitações e Trabalho Futuro

### ❌ O que não foi implementado

#### **Sistema de Elevação do Garfo**

Infelizmente, não tivemos tempo de implementar o código de controle para o motor de elevação do garfo. A **parte mecânica e eletrônica estava completa**, incluindo:

- Motor DC com encoder
- Sistema da passagem dos fios, guias e garfo impressas em 3D
- Driver de motor conectado ao ESP32

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

4. **Latência na comunicação WiFi**
   - Atraso na transmissão de informações no modo manual
   - Possível instabilidade da conexão WiFi em tempo real
   - Impactou a responsividade e controle direto do robô
   - Comandos demoravam alguns milissegundos para chegar
   - Feedback de telemetria não era tão rápido quanto ideal

Estes problemas mecânicos, de atrito e de comunicação impactaram significativamente a performance do sistema autônomo e manual, demonstrando a importância da integração mecânica, eletrônica e de software em projetos de robótica.

### ✅ O que foi concluído com sucesso

1. **Sistema de locomoção**

   - Motores DC com controle PID implementado
   - Odometria funcional (com limitações pelo atrito)
   - Controle diferencial básico operacional

2. **Comunicação WiFi/WebSocket**

   - Conexão funcional (com latência perceptível no controle manual)
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

#### **Software - Comunicação**

- Otimizar comunicação WiFi para reduzir latência
- Implementar compressão de dados para mais rápida transmissão
- Aumentar frequência de heartbeat
- Otimizar protocolo WebSocket

#### **Software - Controle de Elevação**

- Implementar controle manual do garfo (subir/descer)
- Implementar controle autônomo do garfo em missões
- Integrar interface gráfica para controle da elevação

#### **Hardware e Mecânica**

- Pensar em estratégias para lidar com o atrito do piso
- Implementar BMS para segurança das baterias
- Imprimir as peças não montadas (Motor Elevation Base, Roof) e montá-las corretamente
- Adicionar sensor de fim de curso para detectar quando o pallet está posicionado
- Adicionar sensor ultrassônico para detecção de obstáculos

#### **Algoritmos Avançados**

- Implementar SLAM para mapeamento do ambiente
- Melhorar odometria com fusão de sensores (IMU)
- Adicionar planejamento de trajetória (A\*, RRT)
- Implementar pickup/place automático de pallets
- Adicionar sistema de filas de missões

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
