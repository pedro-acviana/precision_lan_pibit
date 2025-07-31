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
│   │   │   │   ├── img2local.py             # Conversão imagem->coordenadas
│   │   │   │   └── aproxima.py              # Aproximação controlada
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

### ✅ **Controle de Voo**
- [x] Armamento e desarmamento automático
- [x] Modo Offboard para controle customizado
- [x] Controle de velocidade adaptativo baseado em altitude e distância
- [x] Sistema de coordenadas customizado (X=longitudinal, Y=lateral)
- [x] Sincronização de takeoff com flags de blackboard
- [x] Controle proporcional suavizado para aproximação precisa
- [x] Sequências de decolagem e pouso com validação de altitude

### ✅ **Arquitetura de Controle**
- [x] Árvores de comportamento (Behavior Trees) com execução paralela
- [x] Processamento paralelo: detecção contínua durante toda a missão
- [x] Sistema de blackboard para sincronização entre nodos
- [x] Fases de estabilização antes da conversão de coordenadas
- [x] Condições de segurança integradas com validação de horizonte
- [x] Threading daemon para processamento de câmera em background

### 🔄 **Em Desenvolvimento**
- [ ] Controle adaptativo para diferentes condições de vento
- [ ] Estimação de parâmetros em tempo real
- [ ] Filtros avançados para atenuação de ruído
- [ ] Algoritmos de pouso em terrenos inclinados
- [ ] Sistema de recuperação de falhas

## 🛠️ Instalação e Configuração

### **Implementações Técnicas Principais**

#### **Sistema de Coordenadas Customizado**
- **X**: Movimento longitudinal (frente/trás) 
- **Y**: Movimento lateral (direita/esquerda)
- **Mapeamento NED**: Conversão automática para o sistema PX4 (Norte-Leste-Down)

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

### **Pré-requisitos**
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

## 📊 Resultados Esperados

### **Métricas de Performance**
- **Precisão de pouso**: < 50cm do ponto alvo
- **Tempo de estabilização**: 5 segundos (configurável)
- **Taxa de sucesso**: > 95% em condições normais
- **Frequência de controle**: 20Hz (tempo real)
- **Resolução de câmera**: 1280x960 pixels
- **Sistema de coordenadas**: X=longitudinal, Y=lateral (customizado)

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

### **Fase 3: Controle Avançado** 🔄
- [ ] Implementação de filtros adaptativos
- [ ] Estimação de parâmetros em tempo real
- [ ] Controle robusto contra distúrbios

### **Fase 4: Validação** 📋
- [ ] Testes extensivos em simulação
- [ ] Análise de performance
- [ ] Documentação científica

## 📚 Documentação Adicional

- [**Manual de Instalação**](docs/INSTALL.md)
- [**Guia de Desenvolvimento**](docs/DEVELOPMENT.md)
- [**API Reference**](docs/API.md)
- [**Resultados Experimentais**](docs/RESULTS.md)

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
