import py_trees
import numpy as np
import math
import pickle
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition


class img2local(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        
        # Parâmetros intrínsecos da câmera
        # self.fx = 1009.622747206156419  # focal length x (pixels)
        # self.fy = 1007.783677020086202  # focal length y (pixels)

        # Mudar de volta para os valores da raspcam!!!!!!!
        self.fx = 540  # focal length x (pixels)
        self.fy = 627  # focal length y (pixels)
        self.cx = 640  # Centro X = 1280/2
        self.cy = 480  # Centro Y = 960/2

        # self.cx = 294.9315450028845476  # principal point x
        # self.cy = 233.5219972126571619  # principal point y
        
        # Dimensões da imagem
        self.image_width = 1280   # Largura da imagem em pixels
        self.image_height = 960   # Altura da imagem em pixels

        # Matriz de calibração intrínseca K
        self.K = np.array([
            [self.fx, 0,       self.cx],
            [0,       self.fy, self.cy],
            [0,       0,       1]
        ])
        
        # Matriz de rotação (assumindo câmera apontando para a frente)
        # Para câmera apontando para frente: sem rotação (matriz identidade)
        self.R = np.eye(3)
        
        # Vetor de translação (sem translação adicional)
        self.t = np.array([0, 0, 0])
        
        # Matriz de projeção P = K [R|t]
        self.P = np.dot(self.K, np.hstack((self.R, self.t.reshape(3,1))))
        
        # Inicializa variáveis
        self.current_altitude = None
        self.node = None
        
    def setup(self, **kwargs):
        if 'node' not in kwargs:
            self.logger.warning("Parâmetro 'node' não fornecido - subscriber não será criado")
            return True
            
        self.node = kwargs['node']
        
        # Initialize ROS2 components
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.node.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )

        return True
    
    def local_position_callback(self, msg):
        """Callback para atualizar a altitude atual"""
        self.current_altitude = -msg.z  # Inverte Z para ter altitude positiva
        
    def initialise(self):
        self.logger.info("Iniciando conversão de coordenadas pixel para coordenadas relativas")
    
    def pixel_to_relative_position(self, u, v, altitude):
        """Converte coordenadas de pixel para posição relativa ao drone com câmera apontando para frente"""
        try:
            # Para câmera apontando para frente, precisamos calcular onde o raio da câmera intersecta o solo
            
            # 1. Converte pixel para coordenadas normalizadas da câmera usando parâmetros intrínsecos
            x_normalized = (u - self.cx) / self.fx  # Coordenada X normalizada
            y_normalized = (v - self.cy) / self.fy  # Coordenada Y normalizada
            
            # 2. Para câmera apontando para frente, o horizonte está no meio da imagem (cy)
            # Pixels abaixo do horizonte (v > cy) podem ser projetados no solo
            if v <= self.cy:
                self.logger.warning(f"Pixel ({u}, {v}) está no horizonte ou acima - não pode ser projetado no solo")
                return np.array([0, 0, 0])
            
            # 3. Calcula o ângulo de depressão (abaixo do horizonte)
            # y_normalized > 0 significa que estamos olhando para baixo em relação ao horizonte
            angle_depression = math.atan(y_normalized)
            
            # 4. Calcula o ângulo lateral (direita/esquerda do centro)
            angle_lateral = math.atan(x_normalized)
            
            # 5. Calcula a distância no solo usando trigonometria
            # altitude / tan(angle_depression) = distância horizontal até o ponto
            if angle_depression <= 0:
                self.logger.warning("Ângulo de depressão inválido - pixel não está abaixo do horizonte")
                return np.array([0, 0, 0])
                
            distance_forward = altitude / math.tan(angle_depression)  # Distância para frente
            distance_lateral = distance_forward * math.tan(angle_lateral)  # Deslocamento lateral
            
            # 6. Monta a posição relativa seguindo nossa convenção:
            # X = movimento longitudinal (frente/trás)
            # Y = movimento lateral (direita/esquerda)
            # No sistema da imagem: u aumenta da esquerda para direita
            # No sistema do drone: Y positivo é para a esquerda
            # Portanto: Y_drone = -distance_lateral (inverte o sinal)
            relative_position = np.array([
                distance_forward,     # X = longitudinal (+ frente)
                -distance_lateral,    # Y = lateral (+ esquerda quando u > cx, - direita quando u < cx)
                0                     # Z = mantém altitude
            ])
            
            return relative_position
            
        except Exception as e:
            self.logger.error(f"Erro no cálculo de posição relativa: {e}")
            return np.array([0, 0, 0])
    
    def update(self):
        try:
            blackboard = py_trees.blackboard.Blackboard()
            
            # Verifica se o local seguro foi determinado após estabilização
            if not blackboard.exists("local_seguro_pixel"):
                # self.logger.warning("Aguardando local seguro ser determinado pela estabilização...")
                return py_trees.common.Status.RUNNING
                
            # Verifica se há score (indica que a estabilização foi concluída)
            if not blackboard.exists("local_seguro_score"):
                self.logger.warning("Aguardando conclusão da fase de estabilização...")
                return py_trees.common.Status.RUNNING
                
            pixel_pos = blackboard.get("local_seguro_pixel")
            pixel_x, pixel_y = pixel_pos
            
            # Validação das coordenadas
            pixel_x = max(0, min(pixel_x, self.image_width - 1))
            pixel_y = max(0, min(pixel_y, self.image_height - 1))
            
            if self.current_altitude is None:
                self.logger.warning("Posição atual do drone não disponível")
                return py_trees.common.Status.RUNNING
                
            altitude = abs(self.current_altitude)
            if altitude < 0.1:
                self.logger.warning("Altitude muito baixa - usando valor mínimo")
                altitude = 1.0
            
            # Usa o método de conversão baseado em matriz de projeção
            relative_pos = self.pixel_to_relative_position(pixel_x, pixel_y, altitude)
            
            relative_target = {
                'x': relative_pos[0],  # X = Longitudinal (frente/trás)
                'y': relative_pos[1],  # Y = Lateral (esquerda/direita, + esquerda, - direita) 
                'z': 0                 # Z = Altitude mantida
            }
            
            blackboard.set("target_relative_position", relative_target)
            
            score = blackboard.get("local_seguro_score")
            self.logger.info(f"Conversão pós-estabilização: pixel({pixel_x}, {pixel_y}) -> local({relative_target['x']:.2f}, {relative_target['y']:.2f}, 0) [score: {score:.2f}]")
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.logger.error(f"Erro na conversão: {e}")
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.logger.info("Conversão concluída com sucesso")