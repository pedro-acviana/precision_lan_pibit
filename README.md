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
- [x] Captura de imagem em tempo real via Gazebo
- [x] Detecção de locais seguros para pouso
- [x] Análise de variação de textura para identificar áreas planas
- [x] Visualização em tempo real com marcações

### ✅ **Controle de Voo**
- [x] Armamento e desarmamento automático
- [x] Modo Offboard para controle customizado
- [x] Controle de altitude e posição
- [x] Sequências de decolagem e pouso

### ✅ **Arquitetura de Controle**
- [x] Árvores de comportamento (Behavior Trees)
- [x] Execução paralela de tarefas
- [x] Sistema de blackboard para compartilhamento de dados
- [x] Condições de segurança integradas

### 🔄 **Em Desenvolvimento**
- [ ] Controle adaptativo para diferentes condições de vento
- [ ] Estimação de parâmetros em tempo real
- [ ] Filtros avançados para atenuação de ruído
- [ ] Algoritmos de pouso em terrenos inclinados
- [ ] Sistema de recuperação de falhas

## 🛠️ Instalação e Configuração

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
- **Tempo de estabilização**: < 5 segundos
- **Taxa de sucesso**: > 95% em condições normais
- **Frequência de controle**: 20Hz (tempo real)

### **Condições de Teste**
- Terrenos planos e inclinados
- Diferentes condições de iluminação
- Presença de obstáculos
- Variações de vento simuladas

## 🔬 Contribuições Científicas

Este projeto avança o estado da arte em:

1. **Controle Adaptativo**: Algoritmos que se adaptam em tempo real às condições do ambiente
2. **Visão Computacional**: Técnicas robustas para detecção de locais seguros
3. **Arquiteturas Híbridas**: Combinação de controle clássico com árvores de comportamento
4. **Sistemas Embarcados**: Implementação eficiente para hardware limitado

## 📈 Roadmap de Desenvolvimento

### **Fase 1: Fundamentos** ✅
- [x] Configuração do ambiente de desenvolvimento
- [x] Integração PX4 + ROS2 + Gazebo
- [x] Implementação básica de visão computacional

### **Fase 2: Controle Básico** ✅
- [x] Sistema de decolagem e pouso automático
- [x] Detecção de locais seguros
- [x] Controle de posição básico

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
