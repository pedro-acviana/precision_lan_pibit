# Sistema de Controle para Pouso Autônomo Preciso - PIBIT

![Status](https://img.shields.io/badge/Status-Em%20Desenvolvimento-yellow)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.10-green)
![PX4](https://img.shields.io/badge/PX4-Autopilot-orange)

## 📋 Descrição do Projeto

Este repositório contém o desenvolvimento de um **Sistema de Controle em Tempo Real para Estabilização de Voo e Pouso Autônomo Preciso em Terrenos Variados**, desenvolvido como parte do Programa de Iniciação em Desenvolvimento Tecnológico e Inovação (PIBIT).

### 🎯 Objetivo Principal

Desenvolver um sistema avançado de controle que seja capaz de:

- **Estabilização de voo em tempo real** com adaptação a condições variáveis
- **Pouso autônomo preciso** em diferentes tipos de terreno
- **Atenuação de distúrbios externos** e filtragem de ruídos
- **Controle e estabilização** completa do veículo aéreo
- **Posicionamento preciso** em relação a pontos de referência para pouso seguro

### 🔬 Metodologia

O projeto foca exclusivamente no **desenvolvimento da estratégia de controle e identificação de modelo**, incluindo:

1. **Estudo do modelo dinâmico** do veículo aéreo
2. **Análise de parâmetros** de acompanhamento em tempo real
3. **Design do sistema eletrônico** de controle
4. **Desenvolvimento de algoritmos** de controle e estimação de parâmetros
5. **Implementação de soluções eficientes** para pouso de precisão

### 👨‍🎓 Pesquisador

**Pedro Araujo Cordeiro Viana**  
*Programa de Iniciação em Desenvolvimento Tecnológico e Inovação (PIBIT)*

## 🏗️ Arquitetura do Sistema

```
precision_lan_pibit/
├── pibiti_ws/                    # Workspace ROS2 principal
│   ├── src/
│   │   ├── precision_landing/    # Pacote principal de pouso de precisão
│   │   │   ├── nodos/           # Nodos de controle
│   │   │   │   ├── achar_local_seguro.py    # Detecção de local seguro
│   │   │   │   ├── img2local.py             # Conversão imagem->coordenadas (MELHORADO)
│   │   │   │   └── aproxima.py              # Aproximação controlada (MELHORADO)
│   │   │   ├── utils/           # Utilitários avançados (NOVO)
│   │   │   │   ├── kalman_filter.py         # Filtros de Kalman
│   │   │   │   └── enhanced_pose_estimation.py  # Pose estimation
│   │   │   ├── sub_arvores/     # Subárvores de comportamento
│   │   │   └── main.py          # Programa principal
│   │   └── drone_behaviors/      # Biblioteca de comportamentos
│   │       ├── commander/        # Interface PX4
│   │       ├── flight/          # Comportamentos de voo
│   │       ├── conditions/      # Condições de segurança
│   │       └── camera/          # Interface de câmera
│   ├── build/                   # Arquivos de compilação
│   ├── install/                 # Instalação dos pacotes
│   └── log/                     # Logs de compilação
├── camera-calibration/          # Sistema de calibração de câmera
└── node-dronestream/           # Streaming de vídeo (legado)
```

## 🚀 Tecnologias Utilizadas

### **Framework Principal**
- **ROS2 Humble** - Sistema de middleware robótico
- **PX4 Autopilot** - Stack de controle de voo
- **Gazebo** - Simulador 3D para testes

### **Linguagens e Bibliotecas**
- **Python 3.10** - Linguagem principal
- **py-trees** - Árvores de comportamento para controle
- **OpenCV** - Processamento de imagem e visão computacional
- **NumPy** - Computação científica e álgebra linear
- **threading** - Processamento paralelo

### **Algoritmos Avançados Implementados**
- **Filtros de Kalman** - Fusão de dados visuais e IMU para estimativas precisas
- **Structure-from-Motion (SfM)** - Estimativa de profundidade por variação de features
- **Template Matching** - Rastreamento robusto usando correlação normalizada
- **Pose Estimation (PnP)** - Conversão precisa 2D para 3D baseada em Corke
- **Controladores PI** - Eliminação de erro em estado estacionário
- **Homografia Planar** - Rastreamento de features entre frames
- **Visual Servoing** - Controle baseado em informações visuais

### **Comunicação**
- **DDS/RTPS** - Protocolo de comunicação em tempo real
- **uXRCE-DDS** - Micro XRCE-DDS para sistemas embarcados

## 🎮 Funcionalidades Implementadas

### ✅ **Sistema de Visão**
- [x] Captura de imagem em tempo real via Gazebo (resolução 1280x960)
- [x] Detecção de locais seguros com análise FAST features
- [x] Sistema de estabilização e rastreamento de alvo (5 segundos)
- [x] Conversão pixel-para-coordenadas com parâmetros intrínsecos de câmera
- [x] Suporte a câmera frontal com cálculo trigonométrico de projeção
- [x] Visualização em tempo real com marcações e score de qualidade
- [x] **NOVO**: Conversão 2D-3D melhorada usando pose estimation (Capítulo 15 - Corke)
- [x] **NOVO**: Estimativa de profundidade por Structure-from-Motion
- [x] **NOVO**: Correção de distorção da lente com parâmetros intrínsecos
- [x] **NOVO**: Validação geométrica de coordenadas 3D

### ✅ **Controle de Voo**
- [x] Armamento e desarmamento automático
- [x] Modo Offboard para controle customizado
- [x] Controle de velocidade adaptativo baseado em altitude e distância
- [x] Sistema de coordenadas customizado (X=longitudinal, Y=lateral)
- [x] Sincronização de takeoff com flags de blackboard
- [x] Controle proporcional suavizado para aproximação precisa
- [x] Sequências de decolagem e pouso com validação de altitude
- [x] **NOVO**: Controladores PI com eliminação de erro em estado estacionário
- [x] **NOVO**: Anti-windup para prevenir saturação do erro integral
- [x] **NOVO**: Velocidade adaptativa baseada em múltiplos fatores
- [x] **NOVO**: Limitação de aceleração para movimento suave
- [x] **NOVO**: Fusão com filtros de Kalman para controle preciso

### ✅ **Filtros de Kalman e Fusão de Dados** 🆕
- [x] **DepthEstimationKalmanFilter**: Estimativa de profundidade do alvo
- [x] **PositionKalmanFilter**: Fusão de dados visuais + IMU
- [x] **HomographyTracker**: Rastreamento de features entre frames
- [x] **TemplateTracker**: Template matching com SfM real
- [x] Redução de ruído em medições visuais
- [x] Estimativas suaves de posição e velocidade
- [x] Fusão robusta de múltiplas fontes de dados
- [x] Compatibilidade condicional com OpenCV

### ✅ **Template Matching + Structure-from-Motion** 🆕
- [x] **Tracking de Local Seguro**: Usa região detectada como template
- [x] **Correlação Normalizada**: Template matching robusto com OpenCV
- [x] **SfM Real**: Estimativa de profundidade por mudança de área
- [x] **Lei Física**: área aparente ∝ 1/distância²
- [x] **Histórico Temporal**: Análise de múltiplos frames consecutivos
- [x] **Estimativa de Velocidade**: Velocidade em pixels/s do alvo
- [x] **Detecção de Aproximação**: Taxa de mudança da área do template
- [x] **Fallback Inteligente**: Funciona sem OpenCV com estimativas básicas

### ✅ **Arquitetura de Controle**
- [x] Árvores de comportamento (Behavior Trees) com execução paralela
- [x] Processamento paralelo: detecção contínua durante toda a missão
- [x] Sistema de blackboard para sincronização entre nodos
- [x] Fases de estabilização antes da conversão de coordenadas
- [x] Condições de segurança integradas com validação de horizonte
- [x] Threading daemon para processamento de câmera em background
- [x] **NOVO**: Fallbacks robustos para compatibilidade
- [x] **NOVO**: Estrutura modular com utilitários avançados
- [x] **NOVO**: Logging detalhado com incertezas dos filtros

### 🔄 **Em Desenvolvimento**
- [ ] Controle adaptativo para diferentes condições de vento
- [ ] Estimação de parâmetros em tempo real
- [ ] Filtros avançados para atenuação de ruído *(Parcialmente implementado)*
- [ ] Algoritmos de pouso em terrenos inclinados
- [ ] Sistema de recuperação de falhas
- [ ] Calibração adaptativa de câmera durante o voo
- [ ] Homografia completa para tracking avançado

## 🛠️ Instalação e Configuração

### **Implementações Técnicas Principais**

#### **Sistema de Coordenadas Customizado**
- **X**: Movimento longitudinal (frente/trás) 
- **Y**: Movimento lateral (direita/esquerda)
- **Mapeamento NED**: Conversão automática para o sistema PX4 (Norte-Leste-Down)

#### **Filtros de Kalman Avançados** 🆕
- **DepthEstimationKalmanFilter**: Estado [profundidade, velocidade_profundidade]
  - Estima profundidade usando variação do tamanho das features
  - Anti-windup para prevenir saturação
- **PositionKalmanFilter**: Estado [x, y, vx, vy] em coordenadas NED
  - Fusão de medições visuais (posição) + IMU (velocidade)
  - Modelo de velocidade constante com ruído de processo

#### **Conversão 2D-3D Melhorada** 🆕
- **Enhanced Pose Estimation**: Baseado no Capítulo 15 do livro de Corke
- **Structure-from-Motion**: Estimativa de profundidade por variação de features
- **Raycast Avançado**: Considera inclinação da câmera e validação geométrica
- **Correção de Distorção**: Usa parâmetros intrínsecos para precisão

#### **Controladores PI** 🆕
- **Controle Proporcional-Integral**: Elimina erro em estado estacionário
- **Anti-windup**: Previne saturação do erro integral
- **Ganhos Separados**: Controladores independentes para X, Y e Z
- **Velocidade Adaptativa**: Considera altitude, distância e velocidade atual

#### **Template Matching + SfM Avançado** 🆕
- **TemplateTracker**: Rastreamento da região do local seguro entre frames
  - Template baseado na área de menor densidade de features
  - Correlação normalizada com busca inteligente
  - Histórico temporal para análise de múltiplos frames
- **Structure-from-Motion Real**: Estimativa de profundidade pela lei física
  - Lei: área_aparente ∝ 1/distância²
  - Se área aumenta por fator k → distância diminui por √k
  - Suavização temporal para evitar mudanças abruptas
- **Métricas Avançadas**: Velocidade do alvo e taxa de aproximação
- **Integração Kalman**: Fusão com filtros existentes para robustez

#### **Processamento Paralelo**
- **Thread principal**: Árvore de comportamento e controle de missão
- **Thread de câmera**: Captura e análise contínua de imagem
- **Sincronização**: Blackboard compartilhado entre threads

#### **Estabilização Inteligente**
- **Período de análise**: 5 segundos de observação antes da decisão
- **Rastreamento de alvo**: Fixação no melhor local detectado
- **Score de qualidade**: Avaliação contínua da adequação do local

#### **Visão Computacional Avançada**
- **Parâmetros intrínsecos**: Calibração precisa da câmera (fx, fy, cx, cy)
- **Projeção trigonométrica**: Cálculo de distâncias reais no solo
- **Validação de horizonte**: Apenas pixels abaixo do horizonte são válidos
- **Homografia Planar**: Rastreamento de movimento entre frames

### **🎯 Como Usar o Template Matching + SfM**

#### **1. Configuração Básica**
```python
from precision_landing.utils.kalman_filter import TemplateTracker

# Inicializar tracker
tracker = TemplateTracker(initial_depth=5.0)
```

#### **2. Definir Template (no achar_local_seguro)**
```python
# Quando local seguro é detectado
def analisar_local_seguro(self, image):
    # ... código existente de detecção ...
    
    # Usar região detectada como template
    if self.local_fixado:
        center_x, center_y = self.local_fixado
        
        # Define template baseado na região do local seguro
        success = tracker.set_reference_template(
            image, center_x, center_y, region_size=40
        )
        
        if success:
            blackboard.set("template_initialized", True)
            blackboard.set("tracker", tracker)
```

#### **3. Tracking Entre Frames (no img2local)**
```python
# Durante conversão de coordenadas
def enhanced_pixel_to_relative_position(self, u, v, altitude, tracker=None):
    if tracker is not None:
        # Rastreia template na imagem atual
        track_result = tracker.track_template(current_image)
        
        if track_result['found']:
            # Usa posição atualizada do template
            u, v = track_result['position']
            
            # Usa estimativa de profundidade do SfM
            depth_estimate = track_result['depth_estimate']
            
            # Log de debug
            print(f"Template tracking: conf={track_result['confidence']:.2f}")
            print(f"Área mudou: {track_result['area_ratio']:.2f}x")
            print(f"Profundidade SfM: {depth_estimate:.2f}m")
```

#### **4. Integração com Kalman**
```python
# No DepthEstimationKalmanFilter
def update_with_template_tracking(self, track_result):
    if track_result['found'] and track_result['confidence'] > 0.7:
        # Usa estimativa de profundidade do SfM
        depth_measurement = track_result['depth_estimate']
        
        # Atualiza filtro de Kalman
        self.update(depth_measurement)
        
        # Log filtrado
        filtered_depth = self.get_depth_estimate()
        uncertainty = self.get_depth_uncertainty()
        
        print(f"Profundidade filtrada: {filtered_depth:.2f}±{uncertainty:.2f}m")
```

#### **5. Métricas de Performance**
```python
# Velocidade do alvo (pixels/s)
vx, vy = tracker.get_velocity_estimate()

# Taxa de mudança da área (aproximação/afastamento)
area_rate = tracker.get_area_change_rate()

if area_rate > 0:
    print("Alvo crescendo - drone se aproximando")
else:
    print("Alvo diminuindo - drone se afastando")
```

#### **6. Fluxo Completo SfM**
```
Frame 1: Local seguro detectado
   ↓
   Template definido (região 80x80 pixels)
   ↓
Frame 2: Template tracking
   ↓
   Área mudou de 6400 → 7200 pixels² (1.125x maior)
   ↓
   Lei SfM: distância = distância_anterior / √1.125 = distância_anterior / 1.06
   ↓
   Se estava a 5m, agora está a ~4.7m
   ↓
   Filtro de Kalman suaviza: 4.85m
   ↓
   Usa profundidade filtrada para conversão 2D→3D
```

### **🔧 Vantagens do Template Matching**

1. **Precisão**: Usa exatamente a região que será o local de pouso
2. **Robustez**: Não depende de features específicas, usa correlação de toda região
3. **SfM Real**: Estimativa de profundidade fisicamente correta
4. **Integração**: Funciona com filtros de Kalman existentes
5. **Fallback**: Continua funcionando mesmo sem OpenCV
6. **Temporal**: Usa histórico de múltiplos frames para estabilidade

### **⚠️ Considerações Importantes**

- **Resolução**: Template de 80x80 pixels é adequado para alvos de 2-10m
- **Confiança**: Threshold de 0.6 para correlação normalizada
- **Busca**: Margem de 30 pixels ao redor da última posição conhecida
- **Histórico**: Mantém últimos 10 frames para análise temporal
- **Suavização**: Factor α=0.3 para mudanças graduais de profundidade
```bash
# Ubuntu 22.04 LTS
# ROS2 Humble
# PX4 Autopilot
# Gazebo Garden
```

### **Instalação**
```bash
# Clone o repositório
git clone [URL_DO_REPOSITORIO]
cd precision_lan_pibit/pibiti_ws

# Instale as dependências
rosdep install --from-paths src --ignore-src -r -y

# Compile o workspace
colcon build

# Configure o ambiente
source install/setup.bash
```

### **Execução**
```bash
# Terminal 1: Inicie o simulador PX4+Gazebo
make px4_sitl gazebo

# Terminal 2: Execute o sistema de pouso de precisão
cd precision_lan_pibit/pibiti_ws
source install/setup.bash
ros2 run precision_landing main
```

## 🔧 Novos Arquivos Implementados

### **📁 utils/ - Algoritmos Avançados** 🆕
```
precision_landing/utils/
├── __init__.py                           # Inicialização do pacote
├── kalman_filter.py                      # Filtros de Kalman completos
│   ├── DepthEstimationKalmanFilter      # Estimativa de profundidade
│   ├── PositionKalmanFilter             # Fusão visual + IMU
│   ├── HomographyTracker                # Rastreamento por homografia
│   └── TemplateTracker                  # Template matching + SfM real
└── enhanced_pose_estimation.py          # Pose estimation avançada
    ├── EnhancedPose2D3D                 # Conversão 2D-3D melhorada
    ├── AdaptiveCameraCalibration        # Calibração adaptativa
    └── Structure-from-Motion algorithms  # SfM para profundidade
```

### **📋 Documentação Criada** 🆕
```
precision_lan_pibit/
├── README_MELHORIAS.md              # Explicação detalhada das melhorias
├── RESUMO_IMPLEMENTACOES.md         # Status completo das implementações
└── README.md                        # README principal (atualizado)
```

### **🔄 Arquivos Modificados (Melhorados)**
- `nodos/img2local.py` - Integração dos filtros de Kalman e pose estimation
- `nodos/aproxima.py` - Controladores PI e velocidade adaptativa
- `nodos/achar_local_seguro.py` - **Preservado intacto** (exibição da câmera)
- `main.py` - **Preservado intacto** (árvore de comportamento)

## 📊 Resultados Esperados

### **Métricas de Performance**
- **Precisão de pouso**: < 50cm do ponto alvo *(melhorada com filtros de Kalman)*
- **Tempo de estabilização**: 5 segundos (configurável)
- **Taxa de sucesso**: > 95% em condições normais *(esperada melhoria para 98%)*
- **Frequência de controle**: 20Hz (tempo real)
- **Resolução de câmera**: 1280x960 pixels
- **Sistema de coordenadas**: X=longitudinal, Y=lateral (customizado)
- **Redução de ruído**: ~60% com filtros de Kalman (estimativa)
- **Eliminação de erro steady-state**: Controladores PI

### **Condições de Teste**
- Terrenos planos e inclinados
- Diferentes condições de iluminação
- Presença de obstáculos
- Variações de vento simuladas
- Câmera frontal com validação de horizonte
- Sincronização de takeoff e estabilização

## 🔬 Contribuições Científicas

Este projeto avança o estado da arte em:

1. **Controle Adaptativo**: Algoritmos que se adaptam em tempo real às condições do ambiente
2. **Visão Computacional**: Técnicas robustas para detecção de locais seguros com câmera frontal
3. **Arquiteturas Híbridas**: Combinação de controle clássico com árvores de comportamento paralelas
4. **Sistemas Embarcados**: Implementação eficiente para hardware limitado
5. **Coordenadas Customizadas**: Sistema de mapeamento adaptado para controle de drones
6. **Sincronização Temporal**: Coordenação precisa entre processamento de visão e controle
7. **Estabilização Inteligente**: Período de análise antes da tomada de decisão de pouso

### 🆕 **Novas Contribuições Implementadas**

8. **Fusão de Sensores Avançada**: Filtros de Kalman para combinar dados visuais e IMU
9. **Pose Estimation Robusta**: Implementação baseada nos conceitos do livro "Robotics, Vision and Control" de Peter Corke
10. **Structure-from-Motion Aplicado**: Estimativa de profundidade usando variação temporal de features
11. **Controle PI Hierárquico**: Eliminação de erro em estado estacionário respeitando arquitetura PX4
12. **Visual Servoing Avançado**: Conversão 2D-3D precisa com validação geométrica
13. **Compatibilidade Robusta**: Sistema funciona com ou sem OpenCV, com fallbacks inteligentes
14. **Template Matching Adaptativo**: Rastreamento do local de pouso usando a própria região detectada
15. **SfM com Lei Física**: Estimativa de profundidade baseada em área aparente ∝ 1/distância²
16. **Tracking Temporal**: Análise de múltiplos frames para estimativas robustas de velocidade e aproximação

## 📈 Roadmap de Desenvolvimento

### **Fase 1: Fundamentos** ✅
- [x] Configuração do ambiente de desenvolvimento
- [x] Integração PX4 + ROS2 + Gazebo
- [x] Implementação básica de visão computacional

### **Fase 2: Controle Básico** ✅
- [x] Sistema de decolagem e pouso automático
- [x] Detecção de locais seguros com estabilização
- [x] Controle de posição com sistema de coordenadas customizado
- [x] Conversão precisa pixel-para-mundo com trigonometria
- [x] Execução paralela de detecção durante missão completa
- [x] Sincronização entre nodos via blackboard

### **Fase 3: Controle Avançado** ✅🔄
- [x] **Filtros de Kalman**: Implementação completa para fusão de dados
- [x] **Controladores PI**: Eliminação de erro em estado estacionário
- [x] **Pose Estimation**: Conversão 2D-3D baseada em Corke (Capítulos 14-15)
- [x] **Structure-from-Motion**: Estimativa de profundidade por variação de features
- [x] **Visual Servoing**: Controle baseado em informação visual avançada
- [ ] Estimação de parâmetros em tempo real
- [ ] Controle robusto contra distúrbios específicos
- [ ] Calibração adaptativa durante o voo

### **Fase 4: Validação** 📋
- [ ] Testes extensivos em simulação
- [ ] Análise de performance
- [ ] Documentação científica

## 📚 Documentação Adicional

- [**Manual de Instalação**](docs/INSTALL.md)
- [**Guia de Desenvolvimento**](docs/DEVELOPMENT.md)
- [**API Reference**](docs/API.md)
- [**Resultados Experimentais**](docs/RESULTS.md)
- [**🆕 Melhorias Implementadas**](README_MELHORIAS.md)
- [**🆕 Resumo das Implementações**](RESUMO_IMPLEMENTACOES.md)

### 📖 **Base Teórica**

As implementações seguem os conceitos fundamentais do livro:
**"Robotics, Vision and Control: Fundamental Algorithms in Python"** por Peter Corke

- **Capítulo 3**: Filtros de Kalman e fusão de sensores
- **Capítulo 4**: Sistemas de controle robótico (PI/PID)
- **Capítulo 14**: Structure-from-Motion e Homografia
- **Capítulo 15**: Pose Estimation e Perspective-n-Point (PnP)
- **Apêndice H**: Implementação prática de filtros

## 🤝 Colaboração e Suporte

Este projeto faz parte do Programa PIBIT e está sendo desenvolvido sob orientação acadêmica. Para questões técnicas ou colaborações:

- **Issues**: Use o sistema de issues do GitHub
- **Documentação**: Consulte a pasta `docs/`
- **Contato**: [email do pesquisador]

## 📄 Licença

Este projeto é licenciado sob [LICENSE](LICENSE) - consulte o arquivo para detalhes.

---

**Desenvolvido com ❤️ para o avanço da robótica aérea autônoma**

*Programa de Iniciação em Desenvolvimento Tecnológico e Inovação (PIBIT)*
